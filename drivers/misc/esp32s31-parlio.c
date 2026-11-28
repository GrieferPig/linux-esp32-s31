// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/uaccess.h>

#include <uapi/linux/esp32s31-parlio.h>

/* S31 TRM ch.33 Parallel IO Controller: RX/TX config and interrupt registers. */
#define RX_MODE			0x00
#define  RX_SOFTWARE_MODE	FIELD_PREP(GENMASK(31, 30), 2)
#define  RX_SW_EN		BIT(25)
#define  RX_MODE_MASK		GENMASK(31, 25)
#define RX_DATA			0x04
#define  DATA_WIDTH		GENMASK(31, 29)
#define  DATA_MSB_FIRST		BIT(28)
#define  DATA_BITLEN		GENMASK(27, 9)
#define RX_GENERAL		0x08
#define  RX_EXT_EOF		BIT(30)
#define  RX_TIMEOUT_EN		BIT(29)
#define RX_START		0x0c
#define TX_DATA			0x10
#define TX_START		0x14
#define TX_GENERAL		0x18
#define  TX_VALID		BIT(31)
#define  TX_CLK_GATING		BIT(30)
#define FIFO_CFG		0x1c
#define  RX_FIFO_RESET		BIT(31)
#define  TX_FIFO_RESET		BIT(30)
#define RX_UPDATE		0x20
#define TX_STATUS		0x24
#define  TX_READY		BIT(31)
#define INT_EN			0x28
#define  INT_TX_UNDERRUN	BIT(0)
#define  INT_RX_OVERFLOW	BIT(1)
#define  INT_TX_EOF		BIT(2)
#define  INT_ALL		GENMASK(2, 0)
#define INT_STATUS		0x30
#define INT_CLEAR		0x34
#define RX_CLOCK		0x44
#define TX_CLOCK		0x48
#define  TX_CLOCK_INVERT	BIT(31)
#define REG_CLOCK		0x120
#define  REG_CLOCK_ON		BIT(31)
#define START			BIT(31)
#define UPDATE			BIT(31)

#define PARLIO_MAX_BYTES	4096
#define PARLIO_MAX_HZ		40000000
#define PARLIO_IRQ_MARGIN_MS	20

struct s31_parlio_dma {
	struct dma_chan *chan;
	struct completion done;
	enum dmaengine_tx_result result;
};

struct s31_parlio {
	void __iomem *base;
	struct clk *xtal, *rx_pad, *rx_clk, *tx_clk;
	struct reset_control *rx_reset, *tx_reset;
	struct s31_parlio_dma rx, tx;
	struct completion tx_eof;
	struct mutex lock;
	struct miscdevice misc;
	struct kref ref;
	u32 width;
	u32 irq_error;
	int irq_rx, irq_tx;
	bool removed;
};

static irqreturn_t s31_parlio_irq(int irq, void *arg)
{
	struct s31_parlio *s31 = arg;
	u32 mask = irq == s31->irq_rx ? INT_RX_OVERFLOW :
		   INT_TX_EOF | INT_TX_UNDERRUN;
	u32 status = readl(s31->base + INT_STATUS) & mask;

	if (!status)
		return IRQ_NONE;
	writel(status, s31->base + INT_CLEAR);
	if (status & (INT_RX_OVERFLOW | INT_TX_UNDERRUN)) {
		WRITE_ONCE(s31->irq_error, status);
		complete(&s31->rx.done);
		complete(&s31->tx.done);
		complete(&s31->tx_eof);
	}
	if (status & INT_TX_EOF)
		complete(&s31->tx_eof);
	return IRQ_HANDLED;
}

static void s31_parlio_dma_done(void *arg, const struct dmaengine_result *result)
{
	struct s31_parlio_dma *dma = arg;

	WRITE_ONCE(dma->result, result->result);
	complete(&dma->done);
}

static int s31_parlio_submit(struct s31_parlio_dma *dma, dma_addr_t addr,
			    size_t length, enum dma_transfer_direction dir)
{
	struct dma_async_tx_descriptor *desc;

	reinit_completion(&dma->done);
	WRITE_ONCE(dma->result, DMA_TRANS_ABORTED);
	desc = dmaengine_prep_slave_single(dma->chan, addr, length, dir,
					  DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -ENOMEM;

	desc->callback_result = s31_parlio_dma_done;
	desc->callback_param = dma;

	return dma_submit_error(dmaengine_submit(desc));
}

static int s31_parlio_reset_unit(struct s31_parlio *s31, bool receive)
{
	struct clk *clk = receive ? s31->rx_clk : s31->tx_clk;
	struct reset_control *rst = receive ? s31->rx_reset : s31->tx_reset;
	unsigned long rate;
	int ret;

	if (receive) {
		ret = clk_set_parent(clk, s31->xtal);
		if (ret)
			return ret;
	}
	ret = clk_prepare_enable(clk);
	if (ret)
		return ret;

	rate = clk_get_rate(clk);
	ret = reset_control_reset(rst);
	if (!ret) {
		writel(receive ? RX_FIFO_RESET : TX_FIFO_RESET, s31->base + FIFO_CFG);
		/* S31 TRM 33.5.2: the async FIFO reset needs two core clock cycles. */
		ndelay(DIV_ROUND_UP_ULL(2ULL * NSEC_PER_SEC, rate));
		writel(0, s31->base + FIFO_CFG);
	}
	clk_disable_unprepare(clk);

	if (!ret && receive)
		ret = clk_set_parent(clk, s31->rx_pad);
	return ret;
}

static int s31_parlio_wait(struct completion *done, unsigned long deadline)
{
	long ret, remaining = (long)(deadline - jiffies);

	ret = wait_for_completion_interruptible_timeout(done, max(remaining, 0L));
	if (ret < 0)
		return ret;
	return ret ? 0 : -ETIMEDOUT;
}

static int s31_parlio_transfer(struct s31_parlio *s31, dma_addr_t rx_addr,
			      dma_addr_t tx_addr, size_t length, u32 width,
			      bool receive, bool transmit)
{
	void __iomem *base = s31->base;
	unsigned int rate, usec;
	unsigned long deadline;
	bool rx_running = false, tx_running = false;
	u32 v, data_cfg, mask = 0;
	int ret, stop_ret;

	if (receive) {
		ret = s31_parlio_reset_unit(s31, true);
		if (ret)
			return ret;
		mask |= INT_RX_OVERFLOW;
	}
	if (transmit) {
		ret = s31_parlio_reset_unit(s31, false);
		if (ret)
			return ret;
		mask |= INT_TX_UNDERRUN | INT_TX_EOF;
	}

	data_cfg = FIELD_PREP(DATA_WIDTH, ilog2(width)) |
		   FIELD_PREP(DATA_BITLEN, length * 8);
	if (width < 8)
		data_cfg |= DATA_MSB_FIRST;

	writel(INT_ALL, base + INT_CLEAR);
	WRITE_ONCE(s31->irq_error, 0);
	if (receive) {
		v = readl(base + RX_MODE) & ~RX_MODE_MASK;
		writel(v | RX_SOFTWARE_MODE, base + RX_MODE);
		writel(data_cfg, base + RX_DATA);
		v = readl(base + RX_GENERAL);
		writel(v & ~(RX_EXT_EOF | RX_TIMEOUT_EN), base + RX_GENERAL);
		writel(0, base + RX_CLOCK);
		writel(UPDATE, base + RX_UPDATE);
		ret = readl_poll_timeout(base + RX_UPDATE, v, !(v & UPDATE), 1, 1000);
		if (ret)
			return ret;
		ret = s31_parlio_submit(&s31->rx, rx_addr, length, DMA_DEV_TO_MEM);
		if (ret)
			return ret;
	}
	if (transmit) {
		/* S31 TRM 33.5.9: TX VALID occupies data line 15. */
		writel(width == 16 ? 0 : TX_VALID | TX_CLK_GATING, base + TX_GENERAL);
		writel(data_cfg, base + TX_DATA);
		writel(TX_CLOCK_INVERT, base + TX_CLOCK);
		reinit_completion(&s31->tx_eof);
		ret = s31_parlio_submit(&s31->tx, tx_addr, length, DMA_MEM_TO_DEV);
		if (ret)
			goto stop;
	}
	writel(mask, base + INT_EN);

	if (receive) {
		dma_async_issue_pending(s31->rx.chan);
		writel(START, base + RX_START);
		ret = clk_prepare_enable(s31->rx_clk);
		if (ret)
			goto stop;
		rx_running = true;
		writel(readl(base + RX_MODE) | RX_SW_EN, base + RX_MODE);
	}
	if (transmit) {
		dma_async_issue_pending(s31->tx.chan);
		ret = readl_poll_timeout(base + TX_STATUS, v, v & TX_READY, 1, 1000);
		if (ret)
			goto stop;
		writel(START, base + TX_START);
		ret = clk_prepare_enable(s31->tx_clk);
		if (ret)
			goto stop;
		tx_running = true;
	}

	rate = receive ? clk_get_rate(s31->rx_pad) : clk_get_rate(s31->tx_clk);
	if (transmit)
		rate = min_t(unsigned int, rate, clk_get_rate(s31->tx_clk));
	usec = DIV_ROUND_UP_ULL((u64)length * 8 * USEC_PER_SEC, rate * width);
	deadline = jiffies + usecs_to_jiffies(usec) + msecs_to_jiffies(PARLIO_IRQ_MARGIN_MS);

	ret = receive ? s31_parlio_wait(&s31->rx.done, deadline) : 0;
	if (!ret && transmit)
		ret = s31_parlio_wait(&s31->tx.done, deadline);
	if (!ret && transmit)
		ret = s31_parlio_wait(&s31->tx_eof, deadline);
	if (!ret && (READ_ONCE(s31->irq_error) ||
		     (receive && READ_ONCE(s31->rx.result) != DMA_TRANS_NOERROR) ||
		     (transmit && READ_ONCE(s31->tx.result) != DMA_TRANS_NOERROR)))
		ret = -EIO;
stop:
	writel(0, base + INT_EN);
	if (rx_running)
		clk_disable_unprepare(s31->rx_clk);
	if (tx_running)
		clk_disable_unprepare(s31->tx_clk);
	writel(0, base + RX_START);
	writel(0, base + TX_START);
	synchronize_irq(s31->irq_rx);
	synchronize_irq(s31->irq_tx);
	if (receive) {
		stop_ret = dmaengine_terminate_sync(s31->rx.chan);
		if (!ret)
			ret = stop_ret;
	}
	if (transmit) {
		stop_ret = dmaengine_terminate_sync(s31->tx.chan);
		if (!ret)
			ret = stop_ret;
	}
	return ret;
}

static ssize_t s31_parlio_frame(struct file *file, void __user *rx, const void __user *tx,
			      size_t length, unsigned int width)
{
	struct s31_parlio *s31 = file->private_data;
	struct device *rx_dev = NULL, *tx_dev = NULL;
	dma_addr_t rx_addr = 0, tx_addr = 0;
	void *in = NULL, *out = NULL;
	int ret;

	if (!length)
		return 0;
	if (length > PARLIO_MAX_BYTES)
		return -EMSGSIZE;
	if (!rx && !tx)
		return -EINVAL;
	width = width ?: s31->width;
	if (width > s31->width || !is_power_of_2(width))
		return -EINVAL;
	if (file->f_flags & O_NONBLOCK)
		return -EAGAIN;
	ret = mutex_lock_interruptible(&s31->lock);
	if (ret)
		return ret;
	if (s31->removed) {
		ret = -ENODEV;
		goto out;
	}
	if (tx) {
		tx_dev = dmaengine_get_dma_device(s31->tx.chan);
		out = dma_alloc_noncoherent(tx_dev, length, &tx_addr, DMA_TO_DEVICE,
					    GFP_KERNEL);
		if (!out) {
			ret = -ENOMEM;
			goto out;
		}
		if (copy_from_user(out, tx, length)) {
			ret = -EFAULT;
			goto out;
		}
	}
	if (rx) {
		rx_dev = dmaengine_get_dma_device(s31->rx.chan);
		in = dma_alloc_noncoherent(rx_dev, length, &rx_addr,
					   DMA_FROM_DEVICE, GFP_KERNEL);
		if (!in) {
			ret = -ENOMEM;
			goto out;
		}
		memset(in, 0, length);
	}
	/*
	 * S31 TRM ch. 3 and ch. 5: AXI GDMA writes PSRAM outside the CPU
	 * cache. Regain CPU ownership after GDMA has recovered any
	 * unaligned bytes through the direct PSRAM window.
	 */
	if (out)
		dma_sync_single_for_device(tx_dev, tx_addr, length, DMA_TO_DEVICE);
	if (in)
		dma_sync_single_for_device(rx_dev, rx_addr, length, DMA_FROM_DEVICE);
	ret = s31_parlio_transfer(s31, rx_addr, tx_addr, length, width, !!rx, !!tx);
	if (out)
		dma_sync_single_for_cpu(tx_dev, tx_addr, length, DMA_TO_DEVICE);
	if (in) {
		dma_sync_single_for_cpu(rx_dev, rx_addr, length, DMA_FROM_DEVICE);
		if (!ret && copy_to_user(rx, in, length))
			ret = -EFAULT;
	}
out:
	if (in)
		dma_free_noncoherent(rx_dev, length, in, rx_addr, DMA_FROM_DEVICE);
	if (out)
		dma_free_noncoherent(tx_dev, length, out, tx_addr, DMA_TO_DEVICE);
	mutex_unlock(&s31->lock);
	return ret ? ret : length;
}

static ssize_t s31_parlio_read(struct file *file, char __user *buf, size_t count,
			     loff_t *offset)
{
	return s31_parlio_frame(file, buf, NULL, count, 0);
}

static ssize_t s31_parlio_write(struct file *file, const char __user *buf, size_t count,
			      loff_t *offset)
{
	return s31_parlio_frame(file, NULL, buf, count, 0);
}

static long s31_parlio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct esp32s31_parlio_frame frame;
	ssize_t ret;

	if (cmd != ESP32S31_PARLIO_IOC_FRAME)
		return -ENOTTY;
	if (copy_from_user(&frame, (void __user *)arg, sizeof(frame)))
		return -EFAULT;
	if (frame.rx_buf > ULONG_MAX || frame.tx_buf > ULONG_MAX)
		return -EFAULT;
	if ((frame.rx_buf && !(file->f_mode & FMODE_READ)) ||
	    (frame.tx_buf && !(file->f_mode & FMODE_WRITE)))
		return -EBADF;
	ret = s31_parlio_frame(file, u64_to_user_ptr(frame.rx_buf),
			      u64_to_user_ptr(frame.tx_buf), frame.length, frame.data_width);
	return ret < 0 ? ret : 0;
}

static void s31_parlio_free(struct kref *ref)
{
	kfree(container_of(ref, struct s31_parlio, ref));
}

static void s31_parlio_put(void *arg)
{
	struct s31_parlio *s31 = arg;

	kref_put(&s31->ref, s31_parlio_free);
}

static int s31_parlio_open(struct inode *inode, struct file *file)
{
	struct s31_parlio *s31 = container_of(file->private_data, struct s31_parlio, misc);

	kref_get(&s31->ref);
	file->private_data = s31;
	return nonseekable_open(inode, file);
}

static int s31_parlio_release(struct inode *inode, struct file *file)
{
	s31_parlio_put(file->private_data);
	return 0;
}

static const struct file_operations s31_parlio_fops = {
	.owner = THIS_MODULE,
	.open = s31_parlio_open,
	.release = s31_parlio_release,
	.read = s31_parlio_read,
	.write = s31_parlio_write,
	.unlocked_ioctl = s31_parlio_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static int s31_parlio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *bus_reset;
	struct clk *bus;
	struct s31_parlio *s31;
	struct dma_slave_config cfg = {
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_maxburst = 8,
		.src_maxburst = 8,
	};
	u32 tx_rate;
	int ret;

	s31 = kzalloc(sizeof(*s31), GFP_KERNEL);
	if (!s31)
		return -ENOMEM;
	kref_init(&s31->ref);
	ret = devm_add_action_or_reset(dev, s31_parlio_put, s31);
	if (ret)
		return ret;
	mutex_init(&s31->lock);
	init_completion(&s31->rx.done);
	init_completion(&s31->tx.done);
	init_completion(&s31->tx_eof);

	s31->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(s31->base))
		return PTR_ERR(s31->base);
	if (device_property_read_u32(dev, "espressif,data-width", &s31->width) ||
	    !is_power_of_2(s31->width) || s31->width > 16 ||
	    device_property_read_u32(dev, "clock-frequency", &tx_rate) ||
	    !tx_rate || tx_rate > PARLIO_MAX_HZ)
		return -EINVAL;

	bus = devm_clk_get_enabled(dev, "bus");
	if (IS_ERR(bus))
		return dev_err_probe(dev, PTR_ERR(bus), "failed to enable bus clock\n");
	s31->rx_clk = devm_clk_get(dev, "rx");
	if (IS_ERR(s31->rx_clk))
		return dev_err_probe(dev, PTR_ERR(s31->rx_clk), "failed to get RX clock\n");
	s31->tx_clk = devm_clk_get(dev, "tx");
	if (IS_ERR(s31->tx_clk))
		return dev_err_probe(dev, PTR_ERR(s31->tx_clk), "failed to get TX clock\n");
	s31->xtal = devm_clk_get(dev, "xtal");
	if (IS_ERR(s31->xtal))
		return dev_err_probe(dev, PTR_ERR(s31->xtal), "failed to get crystal clock\n");
	s31->rx_pad = devm_clk_get(dev, "rx-pad");
	if (IS_ERR(s31->rx_pad))
		return dev_err_probe(dev, PTR_ERR(s31->rx_pad), "failed to get pad clock\n");
	if (!clk_get_rate(s31->xtal) || !clk_get_rate(s31->rx_pad) ||
	    clk_get_rate(s31->rx_pad) > PARLIO_MAX_HZ)
		return -EINVAL;
	ret = clk_set_rate(s31->tx_clk, tx_rate);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set TX clock rate\n");

	bus_reset = devm_reset_control_get_exclusive(dev, "bus");
	if (IS_ERR(bus_reset))
		return dev_err_probe(dev, PTR_ERR(bus_reset), "failed to get bus reset\n");
	s31->rx_reset = devm_reset_control_get_exclusive(dev, "rx");
	if (IS_ERR(s31->rx_reset))
		return dev_err_probe(dev, PTR_ERR(s31->rx_reset), "failed to get RX reset\n");
	s31->tx_reset = devm_reset_control_get_exclusive(dev, "tx");
	if (IS_ERR(s31->tx_reset))
		return dev_err_probe(dev, PTR_ERR(s31->tx_reset), "failed to get TX reset\n");

	ret = reset_control_reset(bus_reset);
	if (ret)
		return ret;
	writel(REG_CLOCK_ON, s31->base + REG_CLOCK);

	s31->rx.chan = devm_dma_request_chan(dev, "rx");
	if (IS_ERR(s31->rx.chan))
		return dev_err_probe(dev, PTR_ERR(s31->rx.chan), "failed to get RX DMA\n");
	s31->tx.chan = devm_dma_request_chan(dev, "tx");
	if (IS_ERR(s31->tx.chan))
		return dev_err_probe(dev, PTR_ERR(s31->tx.chan), "failed to get TX DMA\n");
	cfg.direction = DMA_DEV_TO_MEM;
	ret = dmaengine_slave_config(s31->rx.chan, &cfg);
	if (ret)
		return ret;
	cfg.direction = DMA_MEM_TO_DEV;
	ret = dmaengine_slave_config(s31->tx.chan, &cfg);
	if (ret)
		return ret;

	s31->irq_rx = platform_get_irq_byname(pdev, "rx");
	if (s31->irq_rx < 0)
		return s31->irq_rx;
	s31->irq_tx = platform_get_irq_byname(pdev, "tx");
	if (s31->irq_tx < 0)
		return s31->irq_tx;
	ret = devm_request_irq(dev, s31->irq_rx, s31_parlio_irq, 0, dev_name(dev), s31);
	if (ret)
		return ret;
	ret = devm_request_irq(dev, s31->irq_tx, s31_parlio_irq, 0, dev_name(dev), s31);
	if (ret)
		return ret;

	s31->misc.minor = MISC_DYNAMIC_MINOR;
	s31->misc.name = "esp32s31-parlio";
	s31->misc.fops = &s31_parlio_fops;
	s31->misc.parent = dev;
	platform_set_drvdata(pdev, s31);
	return misc_register(&s31->misc);
}

static void s31_parlio_remove(struct platform_device *pdev)
{
	struct s31_parlio *s31 = platform_get_drvdata(pdev);

	misc_deregister(&s31->misc);
	mutex_lock(&s31->lock);
	s31->removed = true;
	writel(0, s31->base + INT_EN);
	mutex_unlock(&s31->lock);
}

static const struct of_device_id s31_parlio_of_match[] = {
	{ .compatible = "espressif,esp32s31-parlio" },
	{ }
};
MODULE_DEVICE_TABLE(of, s31_parlio_of_match);

static struct platform_driver s31_parlio_driver = {
	.probe = s31_parlio_probe,
	.remove = s31_parlio_remove,
	.driver = {
		.name = "esp32s31-parlio",
		.of_match_table = s31_parlio_of_match,
	},
};
module_platform_driver(s31_parlio_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 parallel I/O controller");
MODULE_LICENSE("GPL");
