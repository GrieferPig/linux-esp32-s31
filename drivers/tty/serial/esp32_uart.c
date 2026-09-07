// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>
#include <asm/serial.h>
#include "../../dma/esp32s31-ahb-gdma.h"

#define DRIVER_NAME	"esp32-uart"
#define DEV_NAME	"ttyS"
#define UART_NR		4

#define ESP32_UART_TX_FIFO_SIZE	127
#define ESP32_UART_RX_FIFO_SIZE	127

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
#define UHCI_CONF0		0x00
#define UHCI_CONF1		0x14
#define UHCI_ESCAPE_CONF	0x20
#define UHCI_PKT_THRES		0x7c
#define UHCI_LEN_EOF_EN		BIT(9)
#define UHCI_UART_IDLE_EOF_EN	BIT(8)
#define UHCI_CLK_EN		BIT(11)
#define UHCI_TX_RST		BIT(0)
#define UHCI_RX_RST		BIT(1)
#define UHCI_UART_SEL_MASK	GENMASK(4, 2)
#define UHCI_SYS_RST_EN		BIT(2)
#define UHCI_DMA_RING_SLOTS	4U
#define UHCI_DMA_PERIOD		1536U
#define UHCI_DMA_RING_BYTES	(UHCI_DMA_RING_SLOTS * UHCI_DMA_PERIOD)
/*
 * TX slots are bigger than RX periods so a completion interrupt does not
 * throttle the transmit path.  Keep one transaction active in hardware and
 * one queued in software so console traffic cannot starve RX IRQ handling.
 * As in ESP-IDF, each EOF callback mounts and starts the next transaction.
 */
#define UHCI_TX_SLOTS		2U
#define UHCI_TX_PERIOD		2048U
#define UHCI_TX_RING_BYTES	(UHCI_TX_SLOTS * UHCI_TX_PERIOD)
#define UHCI_DMA_BUF_BYTES	(UHCI_TX_RING_BYTES + UHCI_DMA_RING_BYTES)
#endif

#define UART_FIFO_REG			0x00
#define UART_INT_RAW_REG		0x04
#define UART_INT_ST_REG			0x08
#define UART_INT_ENA_REG		0x0c
#define UART_INT_CLR_REG		0x10
#define UART_RXFIFO_FULL_INT			BIT(0)
#define UART_TXFIFO_EMPTY_INT			BIT(1)
#define UART_BRK_DET_INT			BIT(7)
#define UART_CLKDIV_REG			0x14
#define ESP32_UART_CLKDIV			GENMASK(19, 0)
#define ESP32S3_UART_CLKDIV			GENMASK(11, 0)
#define UART_CLKDIV_SHIFT			0
#define UART_CLKDIV_FRAG			GENMASK(23, 20)
#define UART_STATUS_REG			0x1c
#define ESP32_UART_RXFIFO_CNT			GENMASK(7, 0)
#define ESP32S3_UART_RXFIFO_CNT			GENMASK(9, 0)
#define UART_RXFIFO_CNT_SHIFT			0
#define UART_DSRN				BIT(13)
#define UART_CTSN				BIT(14)
#define ESP32_UART_TXFIFO_CNT			GENMASK(23, 16)
#define ESP32S3_UART_TXFIFO_CNT			GENMASK(25, 16)
#define UART_TXFIFO_CNT_SHIFT			16
#define UART_CONF0_REG			0x20
#define UART_RXFIFO_RST			BIT(22)
#define UART_TXFIFO_RST			BIT(23)
#define UART_CLK_CONF_REG		0x88
#define UART_TX_SCLK_EN			BIT(24)
#define UART_RX_SCLK_EN			BIT(25)
#define UART_PARITY				BIT(0)
#define UART_PARITY_EN				BIT(1)
#define UART_BIT_NUM				GENMASK(3, 2)
#define UART_BIT_NUM_5				0
#define UART_BIT_NUM_6				1
#define UART_BIT_NUM_7				2
#define UART_BIT_NUM_8				3
#define UART_STOP_BIT_NUM			GENMASK(5, 4)
#define UART_STOP_BIT_NUM_1			1
#define UART_STOP_BIT_NUM_2			3
#define UART_SW_RTS				BIT(6)
#define UART_SW_DTR				BIT(7)
#define UART_LOOPBACK				BIT(14)
#define UART_TX_FLOW_EN				BIT(15)
#define UART_RTS_INV				BIT(23)
#define UART_DTR_INV				BIT(24)
#define UART_CONF1_REG			0x24
#define UART_RXFIFO_FULL_THRHD_SHIFT		0
#define ESP32_UART_TXFIFO_EMPTY_THRHD_SHIFT	8
#define ESP32S3_UART_TXFIFO_EMPTY_THRHD_SHIFT	10
#define ESP32_UART_RX_FLOW_EN			BIT(23)
#define ESP32S3_UART_RX_FLOW_EN			BIT(22)
#define ESP32S3_UART_CLK_CONF_REG	0x78
#define ESP32S31_UART_REG_UPDATE_REG	0x98
#define ESP32S31_UART_REG_UPDATE		BIT(0)
#define ESP32S31_UART_IRDA_EN		BIT(14)
#define ESP32S31_UART_LOOPBACK		BIT(12)
#define ESP32S31_UART_SW_RTS		BIT(21)
#define ESP32S31_UART_TX_FLOW_EN		BIT(13)
#define ESP32S31_UART_RX_FLOW_EN		BIT(8)
#define ESP32S31_UART_RTS_INV		BIT(18)
#define ESP32S31_UART_RS485_CONF_REG	0x4c
#define ESP32S31_UART_RS485_EN		BIT(0)
#define ESP32S31_UART_RS485_DL0_EN	BIT(1)
#define ESP32S31_UART_RS485_DL1_EN	BIT(2)
#define ESP32S31_UART_RS485TX_RX_EN	BIT(3)
#define ESP32S3_UART_SCLK_DIV_B			GENMASK(5, 0)
#define ESP32S3_UART_SCLK_DIV_A			GENMASK(11, 6)
#define ESP32S3_UART_SCLK_DIV_NUM		GENMASK(19, 12)
#define ESP32S3_UART_SCLK_SEL			GENMASK(21, 20)
#define APB_CLK					1
#define RC_FAST_CLK				2
#define XTAL_CLK				3
#define ESP32S3_UART_SCLK_EN			BIT(22)
#define ESP32S3_UART_RST_CORE			BIT(23)
#define ESP32S3_UART_TX_SCLK_EN			BIT(24)
#define ESP32S3_UART_RX_SCLK_EN			BIT(25)
#define ESP32S3_UART_TX_RST_CORE		BIT(26)
#define ESP32S3_UART_RX_RST_CORE		BIT(27)

/* ESP32-S3+ keeps CLKDIV/CONF0/CONF1 in the APB clock domain; writes only take
 * effect in the UART core after pulsing REG_UPDATE and waiting for it to clear.
 * The original ESP32 has no such register (gated on has_clkconf below).
 */
#define UART_REG_UPDATE_REG		0x98
#define UART_REG_UPDATE			BIT(0)

#define ESP32S3_UART_CLK_CONF_DEFAULT \
	(ESP32S3_UART_RX_SCLK_EN | \
	 ESP32S3_UART_TX_SCLK_EN | \
	 ESP32S3_UART_SCLK_EN | \
	 FIELD_PREP(ESP32S3_UART_SCLK_SEL, XTAL_CLK))

struct esp32_port;

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
struct esp32_uhci_dma_slot {
	struct esp32_port *sport;
	unsigned int index;
	unsigned int len;
	bool queued;
};
#endif

struct esp32_port {
	struct uart_port port;
	struct clk *clk;
	struct reset_control *rst;
	bool irda_mode;
#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
	struct clk *uhci_clk;
	void __iomem *uhci_base;
	void __iomem *uhci_clkrst;
	struct dma_chan *tx_dma;
	struct dma_chan *rx_dma;
	void __iomem *tx_buf;
	void __iomem *rx_buf;
	dma_addr_t tx_dma_addr;
	dma_addr_t rx_dma_addr;
	unsigned int tx_pending;
	unsigned int rx_pending;
	unsigned int rx_flip_short;
	bool tx_dma_active;
	bool rx_dma_active;
	bool dma_started;
	struct esp32_uhci_dma_slot tx_slots[UHCI_TX_SLOTS];
	struct esp32_uhci_dma_slot rx_slots[UHCI_DMA_RING_SLOTS];
	u8 rx_stage[UHCI_DMA_PERIOD];
	struct kfifo console_fifo;
#endif
};

struct esp32_uart_variant {
	u32 clkdiv_mask;
	u32 rxfifo_cnt_mask;
	u32 txfifo_cnt_mask;
	u32 txfifo_empty_thrhd_shift;
	u32 rx_flow_en;
	const char *type;
	bool has_clkconf;
	bool needs_reg_update;
	bool uhci_dma;
};

static const struct esp32_uart_variant esp32_variant = {
	.clkdiv_mask = ESP32_UART_CLKDIV,
	.rxfifo_cnt_mask = ESP32_UART_RXFIFO_CNT,
	.txfifo_cnt_mask = ESP32_UART_TXFIFO_CNT,
	.txfifo_empty_thrhd_shift = ESP32_UART_TXFIFO_EMPTY_THRHD_SHIFT,
	.rx_flow_en = ESP32_UART_RX_FLOW_EN,
	.type = "ESP32 UART",
};

static const struct esp32_uart_variant esp32s3_variant = {
	.clkdiv_mask = ESP32S3_UART_CLKDIV,
	.rxfifo_cnt_mask = ESP32S3_UART_RXFIFO_CNT,
	.txfifo_cnt_mask = ESP32S3_UART_TXFIFO_CNT,
	.txfifo_empty_thrhd_shift = ESP32S3_UART_TXFIFO_EMPTY_THRHD_SHIFT,
	.rx_flow_en = ESP32S3_UART_RX_FLOW_EN,
	.type = "ESP32S3 UART",
	.has_clkconf = true,
};

static const struct esp32_uart_variant esp32s31_variant = {
	.clkdiv_mask = ESP32S3_UART_CLKDIV,
	.rxfifo_cnt_mask = ESP32_UART_RXFIFO_CNT,
	.txfifo_cnt_mask = ESP32_UART_TXFIFO_CNT,
	.txfifo_empty_thrhd_shift = ESP32_UART_TXFIFO_EMPTY_THRHD_SHIFT,
	.rx_flow_en = ESP32S31_UART_RX_FLOW_EN,
	.type = "ESP32-S31 UART",
	.needs_reg_update = true,
};

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
static const struct esp32_uart_variant esp32s31_uhci_variant = {
	.clkdiv_mask = ESP32S3_UART_CLKDIV,
	.rxfifo_cnt_mask = ESP32_UART_RXFIFO_CNT,
	.txfifo_cnt_mask = ESP32_UART_TXFIFO_CNT,
	.txfifo_empty_thrhd_shift = ESP32_UART_TXFIFO_EMPTY_THRHD_SHIFT,
	.rx_flow_en = ESP32S31_UART_RX_FLOW_EN,
	.type = "ESP32-S31 UHCI UART",
	.needs_reg_update = true,
	.uhci_dma = true,
};
#endif

static const struct of_device_id esp32_uart_dt_ids[] = {
	{
		.compatible = "esp,esp32-uart",
		.data = &esp32_variant,
	}, {
		.compatible = "esp,esp32s3-uart",
		.data = &esp32s3_variant,
	}, {
		.compatible = "espressif,esp32s31-uart",
		.data = &esp32s31_variant,
	}, {
		.compatible = "espressif,esp32s31-uhci-uart",
		.data = &esp32s31_uhci_variant,
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, esp32_uart_dt_ids);

static struct esp32_port *esp32_uart_ports[UART_NR];

static const struct esp32_uart_variant *port_variant(struct uart_port *port)
{
	return port->private_data;
}

static void esp32_uart_write(struct uart_port *port, unsigned long reg, u32 v)
{
	writel(v, port->membase + reg);
}

static u32 esp32_uart_read(struct uart_port *port, unsigned long reg)
{
	return readl(port->membase + reg);
}

/* Latch APB-domain config writes (CLKDIV/CONF0/CONF1/CLK_CONF) into the UART
 * core. Without this, e.g. the RX-FIFO-full threshold never takes effect and
 * interrupt-driven RX is unreliable. No-op on the original ESP32.
 */
static void esp32_uart_sync_regs(struct uart_port *port)
{
	if (!port_variant(port)->has_clkconf &&
	    !port_variant(port)->needs_reg_update)
		return;
	esp32_uart_write(port, UART_REG_UPDATE_REG, UART_REG_UPDATE);
	while (esp32_uart_read(port, UART_REG_UPDATE_REG) & UART_REG_UPDATE)
		cpu_relax();
}

static void esp32_uart_update(struct uart_port *port)
{
	esp32_uart_sync_regs(port);
}

static u32 esp32_uart_tx_fifo_cnt(struct uart_port *port)
{
	u32 status = esp32_uart_read(port, UART_STATUS_REG);

	return (status & port_variant(port)->txfifo_cnt_mask) >> UART_TXFIFO_CNT_SHIFT;
}

static u32 esp32_uart_rx_fifo_cnt(struct uart_port *port)
{
	u32 status = esp32_uart_read(port, UART_STATUS_REG);

	return (status & port_variant(port)->rxfifo_cnt_mask) >> UART_RXFIFO_CNT_SHIFT;
}

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
static void esp32_uhci_rx_done(void *arg, const struct dmaengine_result *result);
static void esp32_uhci_tx_done(void *arg);

static void esp32_uhci_hw_init(struct esp32_port *sport)
{
	u32 val;

	val = readl(sport->uhci_clkrst);
	writel(val | UHCI_SYS_RST_EN, sport->uhci_clkrst);
	writel(val & ~UHCI_SYS_RST_EN, sport->uhci_clkrst);
	writel(UHCI_CLK_EN, sport->uhci_base + UHCI_CONF0);
	writel(UHCI_CLK_EN | UHCI_UART_IDLE_EOF_EN |
	       FIELD_PREP(UHCI_UART_SEL_MASK, sport->port.line),
	       sport->uhci_base + UHCI_CONF0);
	writel(0, sport->uhci_base + UHCI_CONF1);
	writel(0, sport->uhci_base + UHCI_ESCAPE_CONF);
	/* Keep the UART RX FIFO drained continuously; the idle-EOF (16 ms)
	 * terminates each burst, so no length threshold is needed (IDF's UHCI
	 * UART path enables only UHCI_RX_IDLE_EOF). */
	writel(0, sport->uhci_base + UHCI_PKT_THRES);
	/* IDF enables the UART core clocks in addition to the bus clock. */
	val = esp32_uart_read(&sport->port, UART_CLK_CONF_REG);
	esp32_uart_write(&sport->port, UART_CLK_CONF_REG,
			 val | UART_TX_SCLK_EN | UART_RX_SCLK_EN);

	/* UART0 is exclusively owned by UHCI after startup. */
	esp32_uart_write(&sport->port, UART_INT_ENA_REG, 0);
	esp32_uart_write(&sport->port, UART_INT_CLR_REG, 0xffffffff);
	/* Drop any loader/PIO bytes before UHCI starts feeding the UART. */
	val = esp32_uart_read(&sport->port, UART_CONF0_REG);
	esp32_uart_write(&sport->port, UART_CONF0_REG,
			 val | UART_RXFIFO_RST | UART_TXFIFO_RST);
	esp32_uart_update(&sport->port);
	esp32_uart_write(&sport->port, UART_CONF0_REG, val &
			 ~(UART_RXFIFO_RST | UART_TXFIFO_RST));
	esp32_uart_update(&sport->port);
}

static int esp32_uhci_prime_rx_ring_locked(struct esp32_port *sport)
{
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	unsigned int i;

	/*
	 * IDF mounts the RX descriptors once as a circular link and re-arms
	 * after every idle EOF with a plain gdma_start of the same ring.
	 * dmaengine_prep_dma_cyclic gives us exactly that: the DMA driver
	 * keeps the ring alive, counts the bytes of each burst and restarts
	 * the link itself, so the UART driver never re-submits.
	 */
	desc = dmaengine_prep_dma_cyclic(sport->rx_dma, sport->rx_dma_addr,
					 UHCI_DMA_RING_BYTES, UHCI_DMA_PERIOD,
					 DMA_DEV_TO_MEM,
					 DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -EIO;
	desc->callback_result = esp32_uhci_rx_done;
	desc->callback_param = sport;
	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie))
		return cookie;
	for (i = 0; i < UHCI_DMA_RING_SLOTS; i++) {
		sport->rx_slots[i].queued = true;
		sport->rx_pending++;
	}
	sport->rx_dma_active = true;
	dma_async_issue_pending(sport->rx_dma);
	return 0;
}

static void esp32_uhci_rx_done(void *arg, const struct dmaengine_result *result)
{
	struct esp32_port *sport = arg;
	struct uart_port *port = &sport->port;
	struct tty_port *tty_port;
	unsigned long flags;
	unsigned int i, count, len, off, pos;
	spin_lock_irqsave(&port->lock, flags);
	/*
	 * The DMA driver keeps a circular ring alive and reports, via the
	 * residual, how many bytes the UHCI wrote since the previous
	 * callback (per-node RX_DONE and idle-EOF events, IDF model).  Consume
	 * exactly `len` bytes beginning at the driver's node-index position.
	 */
	len = result ? UHCI_DMA_RING_BYTES - result->residue : 0;
	if (len > UHCI_DMA_RING_BYTES)
		len = UHCI_DMA_RING_BYTES;
	if (result) {
		const struct esp32s31_ahb_rx_result *rxres =
			container_of(result, struct esp32s31_ahb_rx_result, res);

		pos = rxres->pos;
	} else {
		pos = 0;
	}
	if (sport->rx_pending)
		sport->rx_pending = 0;
	tty_port = port->state ? &port->state->port : NULL;
	for (off = 0; off < len; ) {
		unsigned int ring_pos = (pos + off) % UHCI_DMA_RING_BYTES;
		unsigned int chunk = min_t(unsigned int, UHCI_DMA_PERIOD,
					   len - off);

		chunk = min(chunk, UHCI_DMA_RING_BYTES - ring_pos);

		memcpy_fromio(sport->rx_stage,
			      sport->rx_buf + ring_pos, chunk);
		off += chunk;
		if (!tty_port)
			continue;
		count = chunk;
		if (unlikely(port->sysrq)) {
			for (i = 0, count = 0; i < chunk; i++) {
				if (uart_handle_sysrq_char(port,
							   sport->rx_stage[i]))
					continue;
				sport->rx_stage[count++] = sport->rx_stage[i];
			}
		}
		port->icount.rx += count;
		if (count) {
			size_t ins = tty_insert_flip_string(tty_port,
						     sport->rx_stage,
						     count);

			if (ins != count)
				sport->rx_flip_short += count - ins;
		}
	}
	for (i = 0; i < UHCI_DMA_RING_SLOTS; i++)
		sport->rx_slots[i].queued = false;
	sport->rx_dma_active = sport->rx_pending != 0;
	spin_unlock_irqrestore(&port->lock, flags);
	if (tty_port)
		tty_flip_buffer_push(tty_port);
}

static int esp32_uhci_submit_tx_slot_locked(struct esp32_port *sport,
					struct esp32_uhci_dma_slot *slot)
{
	struct uart_port *port = &sport->port;
	struct tty_port *tty_port = port->state ? &port->state->port : NULL;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	unsigned int count;
	bool console;
	u8 *tail;

	if (slot->queued || uart_tx_stopped(port))
		return 0;
	if (!kfifo_is_empty(&sport->console_fifo)) {
		console = true;
		count = kfifo_out_linear_ptr(&sport->console_fifo, &tail,
					     UHCI_TX_PERIOD);
	} else if (tty_port && !kfifo_is_empty(&tty_port->xmit_fifo)) {
		console = false;
		count = kfifo_out_linear_ptr(&tty_port->xmit_fifo, &tail,
					     UHCI_TX_PERIOD);
	} else {
		return 0;
	}
	count = min_t(unsigned int, count, UHCI_TX_PERIOD);
	memcpy_toio(sport->tx_buf + slot->index * UHCI_TX_PERIOD, tail, count);
	desc = dmaengine_prep_slave_single(sport->tx_dma,
					   sport->tx_dma_addr +
					   slot->index * UHCI_TX_PERIOD,
					   count, DMA_MEM_TO_DEV,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -EIO;
	slot->len = count;
	desc->callback = esp32_uhci_tx_done;
	desc->callback_param = slot;
	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie))
		return cookie;
	/* The DMA engine has accepted the slot, so retire it from the source FIFO. */
	if (console)
		kfifo_skip_count(&sport->console_fifo, count);
	else
		uart_xmit_advance(port, count);
	slot->queued = true;
	sport->tx_pending++;
	sport->tx_dma_active = true;
	return 0;
}

static void esp32_uhci_submit_tx_locked(struct esp32_port *sport)
{
	unsigned int i;

	for (i = 0; i < UHCI_TX_SLOTS; i++) {
		if (sport->tx_pending >= UHCI_TX_SLOTS)
			break;
		if (esp32_uhci_submit_tx_slot_locked(sport,
						      &sport->tx_slots[i]))
			break;
	}
	if (sport->tx_pending)
		dma_async_issue_pending(sport->tx_dma);
}

static void esp32_uhci_tx_done(void *arg)
{
	struct esp32_uhci_dma_slot *slot = arg;
	struct esp32_port *sport = slot->sport;
	struct uart_port *port = &sport->port;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	if (!slot->queued) {
		spin_unlock_irqrestore(&port->lock, flags);
		return;
	}
	slot->queued = false;
	if (sport->tx_pending)
		sport->tx_pending--;
	sport->tx_dma_active = sport->tx_pending != 0;
	esp32_uhci_submit_tx_locked(sport);
	if (port->state && kfifo_len(&port->state->port.xmit_fifo) < WAKEUP_CHARS)
		uart_write_wakeup(port);
	spin_unlock_irqrestore(&port->lock, flags);
}

static int esp32_uhci_dma_start(struct esp32_port *sport)
{
	struct dma_slave_config tx_cfg = {
		.direction = DMA_MEM_TO_DEV,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE,
	};
	struct dma_slave_config rx_cfg = {
		.direction = DMA_DEV_TO_MEM,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE,
	};
	int ret;
	unsigned int i;

	if (sport->dma_started)
		return 0;

	esp32_uhci_hw_init(sport);
	ret = dmaengine_slave_config(sport->tx_dma, &tx_cfg);
	if (!ret)
		ret = dmaengine_slave_config(sport->rx_dma, &rx_cfg);
	if (ret)
		return ret;

	/* Allow a completion racing the initial submissions to rearm its slot. */
	sport->dma_started = true;
	ret = esp32_uhci_prime_rx_ring_locked(sport);
	if (ret) {
		int error = ret;

		sport->dma_started = false;
		dmaengine_terminate_sync(sport->rx_dma);
		sport->rx_pending = 0;
		for (i = 0; i < UHCI_DMA_RING_SLOTS; i++)
			sport->rx_slots[i].queued = false;
		return error;
	}
	return 0;
}
#endif

/* return TIOCSER_TEMT when transmitter is not busy */
static unsigned int esp32_uart_tx_empty(struct uart_port *port)
{
#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
	struct esp32_port *sport = container_of(port, struct esp32_port, port);

	if (port_variant(port)->uhci_dma)
		return sport->tx_dma_active || esp32_uart_tx_fifo_cnt(port) ?
			0 : TIOCSER_TEMT;
#endif
	return esp32_uart_tx_fifo_cnt(port) ? 0 : TIOCSER_TEMT;
}

static void esp32_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	u32 conf0 = esp32_uart_read(port, UART_CONF0_REG);

	if (port_variant(port)->needs_reg_update) {
		conf0 &= ~(ESP32S31_UART_LOOPBACK | ESP32S31_UART_SW_RTS);
		if (mctrl & TIOCM_RTS)
			conf0 |= ESP32S31_UART_SW_RTS;
		if (mctrl & TIOCM_LOOP)
			conf0 |= ESP32S31_UART_LOOPBACK;
		esp32_uart_write(port, UART_CONF0_REG, conf0);
		esp32_uart_update(port);
		return;
	}

	conf0 &= ~(UART_LOOPBACK |
		   UART_SW_RTS | UART_RTS_INV |
		   UART_SW_DTR | UART_DTR_INV);

	if (mctrl & TIOCM_RTS)
		conf0 |= UART_SW_RTS;
	if (mctrl & TIOCM_DTR)
		conf0 |= UART_SW_DTR;
	if (mctrl & TIOCM_LOOP)
		conf0 |= UART_LOOPBACK;

	esp32_uart_write(port, UART_CONF0_REG, conf0);
	esp32_uart_update(port);
}

static const struct serial_rs485 esp32s31_rs485_supported = {
	.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND |
		 SER_RS485_RTS_AFTER_SEND | SER_RS485_RX_DURING_TX,
};

/* Called with port->lock held, or from probe before the port is registered. */
static int esp32s31_uart_rs485_config(struct uart_port *port,
				      struct ktermios *termios,
				      struct serial_rs485 *rs485)
{
	struct esp32_port *sport = container_of(port, struct esp32_port, port);
	u32 conf0 = esp32_uart_read(port, UART_CONF0_REG);
	u32 conf1 = esp32_uart_read(port, UART_CONF1_REG);
	u32 rs485_conf = 0;

	if ((rs485->flags & SER_RS485_ENABLED) && sport->irda_mode)
		return -EBUSY;

	conf0 &= ~(ESP32S31_UART_IRDA_EN | ESP32S31_UART_SW_RTS);
	conf1 &= ~ESP32S31_UART_RTS_INV;

	if (rs485->flags & SER_RS485_ENABLED) {
		/* Match the S31 HAL half-duplex setup, including one-bit turns. */
		conf0 |= ESP32S31_UART_SW_RTS;
		rs485_conf = ESP32S31_UART_RS485_EN |
			     ESP32S31_UART_RS485_DL0_EN |
			     ESP32S31_UART_RS485_DL1_EN;
		if (rs485->flags & SER_RS485_RX_DURING_TX)
			rs485_conf |= ESP32S31_UART_RS485TX_RX_EN;
		if (!(rs485->flags & SER_RS485_RTS_ON_SEND))
			conf1 |= ESP32S31_UART_RTS_INV;
	} else if (sport->irda_mode) {
		conf0 |= ESP32S31_UART_IRDA_EN;
	}

	esp32_uart_write(port, ESP32S31_UART_RS485_CONF_REG, rs485_conf);
	esp32_uart_write(port, UART_CONF0_REG, conf0);
	esp32_uart_write(port, UART_CONF1_REG, conf1);
	esp32_uart_update(port);

	return 0;
}

static unsigned int esp32_uart_get_mctrl(struct uart_port *port)
{
	u32 status = esp32_uart_read(port, UART_STATUS_REG);
	unsigned int ret = TIOCM_CAR;

	if (status & UART_DSRN)
		ret |= TIOCM_DSR;
	if (status & UART_CTSN)
		ret |= TIOCM_CTS;

	return ret;
}

static void esp32_uart_stop_tx(struct uart_port *port)
{
	struct esp32_port *sport = container_of(port, struct esp32_port, port);
	unsigned int i;
	u32 int_ena;

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
	if (port_variant(port)->uhci_dma) {
		esp32s31_ahb_terminate_direction(sport->tx_dma,
						 DMA_MEM_TO_DEV);
		sport->tx_pending = 0;
		for (i = 0; i < UHCI_TX_SLOTS; i++)
			sport->tx_slots[i].queued = false;
		sport->tx_dma_active = false;
		return;
	}
#endif

	int_ena = esp32_uart_read(port, UART_INT_ENA_REG);
	int_ena &= ~UART_TXFIFO_EMPTY_INT;
	esp32_uart_write(port, UART_INT_ENA_REG, int_ena);
}

static void esp32_uart_rxint(struct uart_port *port)
{
	struct tty_port *tty_port = &port->state->port;
	u32 rx_fifo_cnt = esp32_uart_rx_fifo_cnt(port);
	unsigned long flags;
	u32 i;

	if (!rx_fifo_cnt)
		return;

	spin_lock_irqsave(&port->lock, flags);

	for (i = 0; i < rx_fifo_cnt; ++i) {
		u32 rx = esp32_uart_read(port, UART_FIFO_REG);

		if (!rx &&
		    (esp32_uart_read(port, UART_INT_ST_REG) & UART_BRK_DET_INT)) {
			esp32_uart_write(port, UART_INT_CLR_REG, UART_BRK_DET_INT);
			++port->icount.brk;
			uart_handle_break(port);
		} else {
			if (uart_handle_sysrq_char(port, (unsigned char)rx))
				continue;
			tty_insert_flip_char(tty_port, rx, TTY_NORMAL);
			++port->icount.rx;
		}
	}
	spin_unlock_irqrestore(&port->lock, flags);

	tty_flip_buffer_push(tty_port);
}

static void esp32_uart_put_char(struct uart_port *port, u8 c)
{
	esp32_uart_write(port, UART_FIFO_REG, c);
}

static void esp32_uart_put_char_sync(struct uart_port *port, u8 c)
{
	unsigned long timeout = jiffies + HZ;

	while (esp32_uart_tx_fifo_cnt(port) >= ESP32_UART_TX_FIFO_SIZE) {
		if (time_after(jiffies, timeout)) {
			dev_warn(port->dev, "timeout waiting for TX FIFO\n");
			return;
		}
		cpu_relax();
	}
	esp32_uart_put_char(port, c);
}

static void esp32_uart_transmit_buffer(struct uart_port *port)
{
	u32 tx_fifo_used = esp32_uart_tx_fifo_cnt(port);
	unsigned int pending;
	u8 ch;

	if (tx_fifo_used >= ESP32_UART_TX_FIFO_SIZE)
		return;

	pending = uart_port_tx_limited(port, ch,
				       ESP32_UART_TX_FIFO_SIZE - tx_fifo_used,
				       true, esp32_uart_put_char(port, ch),
				       ({}));
	if (pending) {
		u32 int_ena;

		int_ena = esp32_uart_read(port, UART_INT_ENA_REG);
		int_ena |= UART_TXFIFO_EMPTY_INT;
		esp32_uart_write(port, UART_INT_ENA_REG, int_ena);
	}
}

static void esp32_uart_txint(struct uart_port *port)
{
	esp32_uart_transmit_buffer(port);
}

static irqreturn_t esp32_uart_int(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	u32 status;

	status = esp32_uart_read(port, UART_INT_ST_REG);

	if (status & (UART_RXFIFO_FULL_INT | UART_BRK_DET_INT))
		esp32_uart_rxint(port);
	if (status & UART_TXFIFO_EMPTY_INT)
		esp32_uart_txint(port);

	esp32_uart_write(port, UART_INT_CLR_REG, status);

	return IRQ_RETVAL(status);
}

static void esp32_uart_start_tx(struct uart_port *port)
{
	struct esp32_port *sport = container_of(port, struct esp32_port, port);

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
	if (port_variant(port)->uhci_dma) {
		esp32_uhci_submit_tx_locked(sport);
		return;
	}
#endif
	esp32_uart_transmit_buffer(port);
}

static void esp32_uart_stop_rx(struct uart_port *port)
{
	struct esp32_port *sport = container_of(port, struct esp32_port, port);
	unsigned int i;
	u32 int_ena;

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
	if (port_variant(port)->uhci_dma) {
		esp32s31_ahb_terminate_direction(sport->rx_dma,
						 DMA_DEV_TO_MEM);
		sport->rx_pending = 0;
		for (i = 0; i < UHCI_DMA_RING_SLOTS; i++)
			sport->rx_slots[i].queued = false;
		sport->rx_dma_active = false;
		return;
	}
#endif

	int_ena = esp32_uart_read(port, UART_INT_ENA_REG);
	int_ena &= ~UART_RXFIFO_FULL_INT;
	esp32_uart_write(port, UART_INT_ENA_REG, int_ena);
}

static int esp32_uart_startup(struct uart_port *port)
{
	int ret = 0;
	unsigned long flags;
	struct esp32_port *sport = container_of(port, struct esp32_port, port);

	ret = clk_prepare_enable(sport->clk);
	if (ret)
		return ret;
	ret = reset_control_reset(sport->rst);
	if (ret) {
		clk_disable_unprepare(sport->clk);
		return ret;
	}

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
	if (port_variant(port)->uhci_dma) {
		if (!sport->dma_started) {
			ret = clk_prepare_enable(sport->uhci_clk);
			if (ret) {
				clk_disable_unprepare(sport->clk);
				return ret;
			}
			ret = esp32_uhci_dma_start(sport);
			if (ret) {
				clk_disable_unprepare(sport->uhci_clk);
				clk_disable_unprepare(sport->clk);
				return ret;
			}
		}
		return ret;
	}
#endif

	ret = request_irq(port->irq, esp32_uart_int, 0, DRIVER_NAME, port);
	if (ret) {
		clk_disable_unprepare(sport->clk);
		return ret;
	}

	spin_lock_irqsave(&port->lock, flags);
	if (port_variant(port)->has_clkconf)
		esp32_uart_write(port, ESP32S3_UART_CLK_CONF_REG,
				 ESP32S3_UART_CLK_CONF_DEFAULT);
	esp32_uart_write(port, UART_CONF1_REG,
			 (1 << UART_RXFIFO_FULL_THRHD_SHIFT) |
			 (1 << port_variant(port)->txfifo_empty_thrhd_shift));
	esp32_uart_sync_regs(port);	/* latch the RX-FIFO-full threshold */
	esp32_uart_write(port, UART_INT_CLR_REG, UART_RXFIFO_FULL_INT | UART_BRK_DET_INT);
	esp32_uart_write(port, UART_INT_ENA_REG, UART_RXFIFO_FULL_INT | UART_BRK_DET_INT);
	spin_unlock_irqrestore(&port->lock, flags);

	return ret;
}

static void esp32_uart_shutdown(struct uart_port *port)
{
	struct esp32_port *sport = container_of(port, struct esp32_port, port);
	unsigned int i;

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
	if (port_variant(port)->uhci_dma) {
		sport->dma_started = false;
		dmaengine_terminate_sync(sport->tx_dma);
		dmaengine_terminate_sync(sport->rx_dma);
		sport->tx_pending = 0;
		sport->rx_pending = 0;
		for (i = 0; i < UHCI_TX_SLOTS; i++)
			sport->tx_slots[i].queued = false;
		for (i = 0; i < UHCI_DMA_RING_SLOTS; i++)
			sport->rx_slots[i].queued = false;
		sport->tx_dma_active = false;
		sport->rx_dma_active = false;
		clk_disable_unprepare(sport->uhci_clk);
		reset_control_assert(sport->rst);
		clk_disable_unprepare(sport->clk);
		return;
	}
#endif

	esp32_uart_write(port, UART_INT_ENA_REG, 0);
	free_irq(port->irq, port);
	reset_control_assert(sport->rst);
	clk_disable_unprepare(sport->clk);
}

static bool esp32_uart_set_baud(struct uart_port *port, u32 baud)
{
	u32 sclk = port->uartclk;
	u32 div = sclk / baud;

	if (port_variant(port)->has_clkconf) {
		u32 sclk_div = div / port_variant(port)->clkdiv_mask;

		if (div > port_variant(port)->clkdiv_mask) {
			sclk /= (sclk_div + 1);
			div = sclk / baud;
		}
		esp32_uart_write(port, ESP32S3_UART_CLK_CONF_REG,
				 FIELD_PREP(ESP32S3_UART_SCLK_DIV_NUM, sclk_div) |
				 ESP32S3_UART_CLK_CONF_DEFAULT);
	}

	if (div <= port_variant(port)->clkdiv_mask) {
		u32 frag = (sclk * 16) / baud - div * 16;

		esp32_uart_write(port, UART_CLKDIV_REG,
				 div | FIELD_PREP(UART_CLKDIV_FRAG, frag));
		esp32_uart_sync_regs(port);
		return true;
	}

	return false;
}

static void esp32_uart_set_termios(struct uart_port *port,
				   struct ktermios *termios,
				   const struct ktermios *old)
{
	unsigned long flags;
	u32 conf0, conf1;
	u32 baud;
	const u32 rx_flow_en = port_variant(port)->rx_flow_en;
	u32 max_div = port_variant(port)->clkdiv_mask;

	termios->c_cflag &= ~CMSPAR;

	if (port_variant(port)->has_clkconf)
		max_div *= FIELD_MAX(ESP32S3_UART_SCLK_DIV_NUM);

	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / max_div,
				  port->uartclk / 16);

	spin_lock_irqsave(&port->lock, flags);

	conf0 = esp32_uart_read(port, UART_CONF0_REG);
	conf0 &= ~(UART_PARITY_EN | UART_PARITY | UART_BIT_NUM | UART_STOP_BIT_NUM);
	if (port_variant(port)->needs_reg_update) {
		conf0 &= ~ESP32S31_UART_IRDA_EN;
		if (container_of(port, struct esp32_port, port)->irda_mode &&
		    !(port->rs485.flags & SER_RS485_ENABLED))
			conf0 |= ESP32S31_UART_IRDA_EN;
	}

	conf1 = esp32_uart_read(port, UART_CONF1_REG);
	conf1 &= ~rx_flow_en;

	if (termios->c_cflag & PARENB) {
		conf0 |= UART_PARITY_EN;
		if (termios->c_cflag & PARODD)
			conf0 |= UART_PARITY;
	}

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		conf0 |= FIELD_PREP(UART_BIT_NUM, UART_BIT_NUM_5);
		break;
	case CS6:
		conf0 |= FIELD_PREP(UART_BIT_NUM, UART_BIT_NUM_6);
		break;
	case CS7:
		conf0 |= FIELD_PREP(UART_BIT_NUM, UART_BIT_NUM_7);
		break;
	case CS8:
		conf0 |= FIELD_PREP(UART_BIT_NUM, UART_BIT_NUM_8);
		break;
	}

	if (termios->c_cflag & CSTOPB)
		conf0 |= FIELD_PREP(UART_STOP_BIT_NUM, UART_STOP_BIT_NUM_2);
	else
		conf0 |= FIELD_PREP(UART_STOP_BIT_NUM, UART_STOP_BIT_NUM_1);

	if (termios->c_cflag & CRTSCTS)
		conf1 |= rx_flow_en;

	esp32_uart_write(port, UART_CONF0_REG, conf0);
	esp32_uart_write(port, UART_CONF1_REG, conf1);
	esp32_uart_sync_regs(port);

	if (baud) {
		esp32_uart_set_baud(port, baud);
		uart_update_timeout(port, termios->c_cflag, baud);
	} else {
		if (esp32_uart_set_baud(port, 115200)) {
			baud = 115200;
			tty_termios_encode_baud_rate(termios, baud, baud);
			uart_update_timeout(port, termios->c_cflag, baud);
		} else {
			dev_warn(port->dev,
				 "unable to set speed to %d baud or the default 115200\n",
				 baud);
		}
	}
	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *esp32_uart_type(struct uart_port *port)
{
	return port_variant(port)->type;
}

/* configure/auto-configure the port */
static void esp32_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_GENERIC;
}

#ifdef CONFIG_CONSOLE_POLL
static int esp32_uart_poll_init(struct uart_port *port)
{
	struct esp32_port *sport = container_of(port, struct esp32_port, port);

	return clk_prepare_enable(sport->clk);
}

static void esp32_uart_poll_put_char(struct uart_port *port, unsigned char c)
{
	esp32_uart_put_char_sync(port, c);
}

static int esp32_uart_poll_get_char(struct uart_port *port)
{
	if (esp32_uart_rx_fifo_cnt(port))
		return esp32_uart_read(port, UART_FIFO_REG);
	else
		return NO_POLL_CHAR;

}
#endif

static const struct uart_ops esp32_uart_pops = {
	.tx_empty	= esp32_uart_tx_empty,
	.set_mctrl	= esp32_uart_set_mctrl,
	.get_mctrl	= esp32_uart_get_mctrl,
	.stop_tx	= esp32_uart_stop_tx,
	.start_tx	= esp32_uart_start_tx,
	.stop_rx	= esp32_uart_stop_rx,
	.startup	= esp32_uart_startup,
	.shutdown	= esp32_uart_shutdown,
	.set_termios	= esp32_uart_set_termios,
	.type		= esp32_uart_type,
	.config_port	= esp32_uart_config_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_init	= esp32_uart_poll_init,
	.poll_put_char	= esp32_uart_poll_put_char,
	.poll_get_char	= esp32_uart_poll_get_char,
#endif
};

static void esp32_uart_console_putchar(struct uart_port *port, u8 c)
{
	esp32_uart_put_char_sync(port, c);
}

static void esp32_uart_string_write(struct uart_port *port, const char *s,
				    unsigned int count)
{
	struct esp32_port *sport = container_of(port, struct esp32_port, port);

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
	if (port_variant(port)->uhci_dma) {
		unsigned char ch;

		if (!sport->dma_started)
			return;
		while (count--) {
			if (*s == '\n' && kfifo_avail(&sport->console_fifo) > 1) {
				ch = '\r';
				kfifo_in(&sport->console_fifo, &ch, 1);
			}
			if (!kfifo_avail(&sport->console_fifo))
				break;
			ch = *s++;
			kfifo_in(&sport->console_fifo, &ch, 1);
		}
		esp32_uhci_submit_tx_locked(sport);
		return;
	}
#endif
	uart_console_write(port, s, count, esp32_uart_console_putchar);
}

static void
esp32_uart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct esp32_port *sport = esp32_uart_ports[co->index];
	struct uart_port *port = &sport->port;
	unsigned long flags;
	bool locked = true;

	if (port->sysrq)
		locked = false;
	else if (oops_in_progress)
		locked = spin_trylock_irqsave(&port->lock, flags);
	else
		spin_lock_irqsave(&port->lock, flags);

	esp32_uart_string_write(port, s, count);

	if (locked)
		spin_unlock_irqrestore(&port->lock, flags);
}

static int __init esp32_uart_console_setup(struct console *co, char *options)
{
	struct esp32_port *sport;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;

	/*
	 * check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index == -1 || co->index >= ARRAY_SIZE(esp32_uart_ports))
		co->index = 0;

	sport = esp32_uart_ports[co->index];
	if (!sport)
		return -ENODEV;

	ret = clk_prepare_enable(sport->clk);
	if (ret)
		return ret;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	ret = uart_set_options(&sport->port, co, baud, parity, bits, flow);
	if (ret)
		goto err_uart_clk;

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
	if (port_variant(&sport->port)->uhci_dma) {
		ret = clk_prepare_enable(sport->uhci_clk);
		if (ret)
			goto err_uart_clk;
		ret = esp32_uhci_dma_start(sport);
		if (ret) {
			clk_disable_unprepare(sport->uhci_clk);
			goto err_uart_clk;
		}
	}
#endif
	return 0;

err_uart_clk:
	clk_disable_unprepare(sport->clk);
	return ret;
}

static int esp32_uart_console_exit(struct console *co)
{
	struct esp32_port *sport = esp32_uart_ports[co->index];

	clk_disable_unprepare(sport->clk);
	return 0;
}

static struct uart_driver esp32_uart_reg;
static struct console esp32_uart_console = {
	.name		= DEV_NAME,
	.write		= esp32_uart_console_write,
	.device		= uart_console_device,
	.setup		= esp32_uart_console_setup,
	.exit		= esp32_uart_console_exit,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &esp32_uart_reg,
};

static void esp32_uart_earlycon_putchar(struct uart_port *port, u8 c)
{
	esp32_uart_put_char_sync(port, c);
}

static void esp32_uart_earlycon_write(struct console *con, const char *s,
				      unsigned int n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, esp32_uart_earlycon_putchar);
}

#ifdef CONFIG_CONSOLE_POLL
static int esp32_uart_earlycon_read(struct console *con, char *s, unsigned int n)
{
	struct earlycon_device *dev = con->data;
	unsigned int num_read = 0;

	while (num_read < n) {
		int c = esp32_uart_poll_get_char(&dev->port);

		if (c == NO_POLL_CHAR)
			break;
		s[num_read++] = c;
	}
	return num_read;
}
#endif

static int __init esp32xx_uart_early_console_setup(struct earlycon_device *device,
						   const char *options)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = esp32_uart_earlycon_write;
#ifdef CONFIG_CONSOLE_POLL
	device->con->read = esp32_uart_earlycon_read;
#endif
	if (device->port.uartclk != BASE_BAUD * 16)
		esp32_uart_set_baud(&device->port, device->baud);

	return 0;
}

static int __init esp32_uart_early_console_setup(struct earlycon_device *device,
						 const char *options)
{
	device->port.private_data = (void *)&esp32_variant;

	return esp32xx_uart_early_console_setup(device, options);
}

OF_EARLYCON_DECLARE(esp32uart, "esp,esp32-uart",
		    esp32_uart_early_console_setup);

static int __init esp32s3_uart_early_console_setup(struct earlycon_device *device,
						   const char *options)
{
	device->port.private_data = (void *)&esp32s3_variant;

	return esp32xx_uart_early_console_setup(device, options);
}

OF_EARLYCON_DECLARE(esp32s3uart, "esp,esp32s3-uart",
		    esp32s3_uart_early_console_setup);

static int __init esp32s31_uart_early_console_setup(struct earlycon_device *device,
						    const char *options)
{
	device->port.private_data = (void *)&esp32s31_variant;

	return esp32xx_uart_early_console_setup(device, options);
}

OF_EARLYCON_DECLARE(esp32s31uart, "espressif,esp32s31-uart",
		    esp32s31_uart_early_console_setup);

static struct uart_driver esp32_uart_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= DEV_NAME,
	.nr		= ARRAY_SIZE(esp32_uart_ports),
	.cons		= &esp32_uart_console,
};

static int esp32_uart_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct esp32_uart_variant *variant;
	struct uart_port *port;
	struct esp32_port *sport;
	struct resource *res;
	unsigned int i;
	int ret;

	sport = devm_kzalloc(&pdev->dev, sizeof(*sport), GFP_KERNEL);
	if (!sport)
		return -ENOMEM;

	port = &sport->port;

	ret = of_alias_get_id(np, "serial");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", ret);
		return ret;
	}
	if (ret >= UART_NR) {
		dev_err(&pdev->dev, "driver limited to %d serial ports\n", UART_NR);
		return -ENOMEM;
	}

	port->line = ret;
	variant = device_get_match_data(&pdev->dev);
	if (!variant)
		return -EINVAL;

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
	if (variant->uhci_dma) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "uart");
		if (!res)
			return -ENODEV;
		port->mapbase = res->start;
		port->membase = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(port->membase))
			return PTR_ERR(port->membase);
		sport->uhci_base = devm_platform_ioremap_resource_byname(pdev,
									"uhci");
		if (IS_ERR(sport->uhci_base))
			return PTR_ERR(sport->uhci_base);
		sport->uhci_clkrst = devm_platform_ioremap_resource_byname(pdev,
									"clkrst");
		if (IS_ERR(sport->uhci_clkrst))
			return PTR_ERR(sport->uhci_clkrst);
		sport->clk = devm_clk_get(&pdev->dev, "uart");
		if (IS_ERR(sport->clk))
			return PTR_ERR(sport->clk);
		sport->uhci_clk = devm_clk_get(&pdev->dev, "bus");
		if (IS_ERR(sport->uhci_clk))
			return PTR_ERR(sport->uhci_clk);
		sport->tx_dma = dma_request_chan(&pdev->dev, "tx");
		if (IS_ERR(sport->tx_dma))
			return PTR_ERR(sport->tx_dma);
		/* AHB GDMA TX/RX are coupled to one hardware pair. */
		sport->rx_dma = sport->tx_dma;
		ret = kfifo_alloc(&sport->console_fifo, 4096, GFP_KERNEL);
		if (ret)
			goto err_dma;
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma-buf");
		if (!res || resource_size(res) < UHCI_DMA_BUF_BYTES) {
			ret = -EINVAL;
			goto err_dma;
		}
		sport->tx_buf = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(sport->tx_buf)) {
			ret = PTR_ERR(sport->tx_buf);
			goto err_dma;
		}
		sport->rx_buf = (u8 __iomem *)sport->tx_buf +
			UHCI_TX_RING_BYTES;
		sport->tx_dma_addr = res->start;
		sport->rx_dma_addr = res->start + UHCI_TX_RING_BYTES;
		for (i = 0; i < UHCI_TX_SLOTS; i++) {
			sport->tx_slots[i].sport = sport;
			sport->tx_slots[i].index = i;
		}
		for (i = 0; i < UHCI_DMA_RING_SLOTS; i++) {
			sport->rx_slots[i].sport = sport;
			sport->rx_slots[i].index = i;
		}
		port->irq = -1;
	} else
#endif
	{
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res)
			return -ENODEV;

		port->mapbase = res->start;
		port->membase = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(port->membase))
			return PTR_ERR(port->membase);

		sport->clk = devm_clk_get(&pdev->dev, NULL);
		if (IS_ERR(sport->clk))
			return PTR_ERR(sport->clk);
		port->irq = platform_get_irq(pdev, 0);
	}

	port->uartclk = clk_get_rate(sport->clk);
	sport->rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(sport->rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(sport->rst),
				     "reset unavailable\n");
	port->dev = &pdev->dev;
	port->type = PORT_GENERIC;
	port->iotype = UPIO_MEM;
	port->ops = &esp32_uart_pops;
	port->flags = UPF_BOOT_AUTOCONF;
	port->has_sysrq = 1;
	port->fifosize = ESP32_UART_TX_FIFO_SIZE;
	port->private_data = (void *)variant;
	if (variant->needs_reg_update) {
		sport->irda_mode = device_property_read_bool(&pdev->dev,
						       "espressif,irda-mode");
		port->rs485_config = esp32s31_uart_rs485_config;
		port->rs485_supported = esp32s31_rs485_supported;
		ret = uart_get_rs485_mode(port);
		if (ret)
			goto err_port;
		if (sport->irda_mode &&
		    (port->rs485.flags & SER_RS485_ENABLED)) {
			ret = dev_err_probe(&pdev->dev, -EINVAL,
					    "IrDA and RS-485 cannot be enabled together\n");
			goto err_port;
		}
	}

	esp32_uart_ports[port->line] = sport;

	platform_set_drvdata(pdev, port);

	ret = uart_add_one_port(&esp32_uart_reg, port);
	if (ret) {
#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
		if (variant->uhci_dma)
			goto err_dma;
#endif
	}
	return ret;

err_port:
#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
	if (variant->uhci_dma)
		goto err_dma;
#endif
	return ret;

#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
err_dma:
	kfifo_free(&sport->console_fifo);
	dma_release_channel(sport->tx_dma);
	return ret;
#endif
}

static void esp32_uart_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct esp32_port *sport = container_of(port, struct esp32_port, port);

	uart_remove_one_port(&esp32_uart_reg, port);
#if IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI)
	if (port_variant(port)->uhci_dma) {
		kfifo_free(&sport->console_fifo);
		dma_release_channel(sport->tx_dma);
		esp32_uart_ports[port->line] = NULL;
	}
#endif
}


static struct platform_driver esp32_uart_driver = {
	.probe		= esp32_uart_probe,
	.remove		= esp32_uart_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table	= esp32_uart_dt_ids,
	},
};

static int __init esp32_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&esp32_uart_reg);
	if (ret)
		return ret;

	ret = platform_driver_register(&esp32_uart_driver);
	if (ret)
		uart_unregister_driver(&esp32_uart_reg);

	return ret;
}

static void __exit esp32_uart_exit(void)
{
	platform_driver_unregister(&esp32_uart_driver);
	uart_unregister_driver(&esp32_uart_reg);
}

module_init(esp32_uart_init);
module_exit(esp32_uart_exit);

MODULE_AUTHOR("Max Filippov <jcmvbkbc@gmail.com>");
MODULE_DESCRIPTION("Espressif ESP32 UART support");
MODULE_LICENSE("GPL");
