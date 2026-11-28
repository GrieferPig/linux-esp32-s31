// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 LP-core remoteproc and hardware mailbox driver. */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/crc32.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kfifo.h>
#include <linux/ktime.h>
#include <linux/mailbox_controller.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/pm_wakeirq.h>
#include <linux/remoteproc.h>
#include <linux/reboot.h>
#include <linux/spi/spi.h>
#include <linux/suspend.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/soc/espressif/esp32s31-lp.h>

#define S31_LP_RESET_VECTOR             0x2e000080U

/* S31 TRM ch.2 Low-Power CPU: LP_SYS_LP_CORE_BOOT_ADDR_REG. */
#define S31_LP_SYS_BOOT_ADDR_REG        0x18
#define S31_LP_SYS_STORE0_REG           0x2c
#define S31_LP_SYS_STORE2_REG           0x34
#define S31_LP_SYS_STORE3_REG           0x38
#define S31_LP_SYS_STORE4_REG           0x3c
#define S31_LP_SYS_STORE5_REG           0x40
#define S31_LP_STAGE_RUNNING            0x4c500004U
#define S31_DEEP_SLEEP_MAGIC            0x53314453U

#define S31_MB_MSG(n)                   ((n) * 4)
#define S31_MB_HP_ST                    0x54
#define S31_MB_HP_ENA                   0x58
#define S31_MB_HP_CLR                   0x5c
#define S31_MB_CLK_REG                  0x60
#define S31_MB_CLK_EN                   BIT(0)
#define S31_MB_ACK                      0xe5U
#define S31_MB_RX_MASK                  (BIT(0) | BIT(2) | BIT(4) | BIT(6))
#define S31_MB_TX_ACK_MASK              (BIT(9) | BIT(11) | BIT(13) | BIT(15))
#define S31_MB_IRQ_MASK                 (S31_MB_RX_MASK | S31_MB_TX_ACK_MASK)

#define S31_LP_RX_DEPTH                 64
#define S31_LP_TIMEOUT_MS               1000

#if IS_ENABLED(CONFIG_TTY)
struct esp32s31_lp_uart {
	struct tty_port port;
	struct tty_driver *driver;
	struct esp32s31_lp *lp;
	struct work_struct work;
	struct mutex lock;
	u32 pins;
	unsigned int tx_pending;
	bool active;
	bool needs_stop;
};
#endif

struct esp32s31_lp {
	struct device *dev;
	void __iomem *sram;
	void __iomem *mailbox;
	void __iomem *lp_sys;
	struct clk *cpu_clk;
	struct notifier_block reboot_notifier;
	int irq;

	struct mbox_controller mbox;
	struct mbox_chan chan;
	spinlock_t tx_state_lock;
	struct completion tx_done;
	bool tx_pending;
	u8 tx_slot;
	u8 tx_next;
	struct mutex user_tx_lock;
	struct mutex rpc_lock;
	struct completion rpc_done;
	bool peripheral_rpc;
	u32 rpc_expected;
	u32 rpc_response;
	atomic_t rpc_sequence;

	DECLARE_KFIFO_PTR(rx_fifo, u32);
	spinlock_t rx_lock;
	wait_queue_head_t rx_wait;
	u32 last_message;
	bool ready;

	atomic_t ping_sequence;
	ktime_t ping_started;
	s64 ping_rtt_us;

	atomic64_t tx_count;
	atomic64_t rx_count;
	atomic64_t ack_count;
	atomic64_t timeout_count;

#if IS_REACHABLE(CONFIG_I2C)
	struct i2c_adapter i2c;
	u32 i2c_pins;
	u32 i2c_hz;
	u32 i2c_flags;
	bool i2c_registered;
#endif

#if IS_REACHABLE(CONFIG_SPI)
	struct spi_controller *spi;
	struct mutex spi_lock;
	u32 spi_pins;
	u32 spi_mode;
	bool spi_selected;
#endif

#if IS_ENABLED(CONFIG_TTY)
	struct esp32s31_lp_uart *uart;
	struct mutex uart_lock;
	bool uart_removing;
#endif

	struct miscdevice miscdev;
	struct s31_lp_sleep_control s2idle_control;
	bool s2idle_armed;
	struct s31_lp_sleep_control mem_control;
	bool mem_armed;
	struct s31_lp_sleep_control deep_control;
	bool deep_armed;
	struct s31_lp_sleep_control sleep_test_control;
	int sleep_test_result;
	struct s31_lp_sleep_control gpio_test_control;
	int gpio_test_result;
};

#if IS_ENABLED(CONFIG_TTY)
static void esp32s31_lp_uart_event(struct esp32s31_lp *lp);
#endif

static struct esp32s31_lp *esp32s31_lp_sleep_owner;
static unsigned int esp32s31_lp_s2idle_wake_ms;
module_param_named(s2idle_wake_ms, esp32s31_lp_s2idle_wake_ms, uint, 0644);
MODULE_PARM_DESC(s2idle_wake_ms,
	"LP timer wakeup for s2idle in milliseconds (0 disables it)");
static unsigned int esp32s31_lp_mem_wake_ms = 2000;
module_param_named(mem_wake_ms, esp32s31_lp_mem_wake_ms, uint, 0644);
MODULE_PARM_DESC(mem_wake_ms,
	"LP timer wakeup for suspend-to-RAM in milliseconds (10..600000)");
static int esp32s31_lp_mem_wake_gpio = -1;
module_param_named(mem_wake_gpio, esp32s31_lp_mem_wake_gpio, int, 0644);
MODULE_PARM_DESC(mem_wake_gpio,
	"Optional LP GPIO wake pin for suspend-to-RAM (-1 disables, 0..7)");
static bool esp32s31_lp_mem_gpio_active_high = true;
module_param_named(mem_gpio_active_high, esp32s31_lp_mem_gpio_active_high,
		   bool, 0644);
MODULE_PARM_DESC(mem_gpio_active_high, "LP GPIO wake active level");
static unsigned int esp32s31_lp_mem_gpio_pull;
module_param_named(mem_gpio_pull, esp32s31_lp_mem_gpio_pull, uint, 0644);
MODULE_PARM_DESC(mem_gpio_pull,
	"LP GPIO wake pull: 0=none, 1=pull-up, 2=pull-down");

static void esp32s31_lp_s2idle_check(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);
	void __iomem *state_reg;
	u32 state;
	int ret;

	if (!lp || !lp->s2idle_armed || !esp32s31_lp_s2idle_wake_ms)
		return;

	/*
	 * INTMTX/CLIC cannot yet deliver the LP mailbox wake through the
	 * noirq s2idle phase.  Poll only for the explicitly requested test
	 * timer so device suspend/resume and the LP transaction remain
	 * testable without claiming an HP low-power state.
	 */
	state_reg = lp->sram + S31_LP_SLEEP_CONTROL_OFFSET +
		    offsetof(struct s31_lp_sleep_control, state);
	/*
	 * timekeeping is already suspended here, so ktime_get() cannot bound a
	 * polling loop.  Use the atomic polling helper, whose timeout is based on
	 * calibrated delays and remains live throughout the noirq phase.
	 */
	ret = readl_poll_timeout_atomic(state_reg, state,
					state == S31_LP_SLEEP_WAKING, 10,
					(esp32s31_lp_s2idle_wake_ms + 100) *
					USEC_PER_MSEC);
	if (!ret) {
		/* Keep s2idle_enter() from sleeping after this check. */
		pm_wakeup_hard_event(lp->dev);
		return;
	}

	/* Fail safe: never strand a test suspend if the LP timer did not fire. */
	pm_wakeup_hard_event(lp->dev);
}

static bool esp32s31_lp_s2idle_wake(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);
	u32 state;

	if (!lp || !lp->s2idle_armed)
		return false;
	state = readl(lp->sram + S31_LP_SLEEP_CONTROL_OFFSET +
		      offsetof(struct s31_lp_sleep_control, state));
	return state == S31_LP_SLEEP_WAKING;
}

static int esp32s31_lp_s2idle_begin(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);

	if (!lp || !READ_ONCE(lp->ready))
		return -ENODEV;
	memset(&lp->s2idle_control, 0, sizeof(lp->s2idle_control));
	lp->s2idle_control.flags = S31_LP_SLEEP_F_S2IDLE |
					 S31_LP_SLEEP_F_DRY_RUN;
	if (esp32s31_lp_s2idle_wake_ms) {
		if (esp32s31_lp_s2idle_wake_ms < 10 ||
		    esp32s31_lp_s2idle_wake_ms > 600000)
			return -ERANGE;
		lp->s2idle_control.wake_mask = S31_LP_WAKE_TIMER;
		lp->s2idle_control.deadline_lo =
			esp32s31_lp_s2idle_wake_ms * 1000U;
	}
	lp->s2idle_armed = false;
	return 0;
}

static int esp32s31_lp_s2idle_prepare(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);
	int ret;

	if (!lp)
		return -ENODEV;
	ret = esp32s31_lp_sleep_prepare(&lp->s2idle_control);
	if (ret)
		return ret;
	ret = esp32s31_lp_sleep_arm(&lp->s2idle_control);
	if (ret) {
		esp32s31_lp_sleep_abort(&lp->s2idle_control);
		return ret;
	}
	lp->s2idle_armed = true;
	return 0;
}

static void esp32s31_lp_s2idle_restore(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);
	u64 sleep_ticks, wake_ticks;
	int ret;

	if (!lp || !lp->s2idle_armed)
		return;
	ret = esp32s31_lp_sleep_query(&lp->s2idle_control);
	if (ret)
		dev_warn(lp->dev, "failed to read LP wake status: %d\n", ret);
	else {
		sleep_ticks = (u64)lp->s2idle_control.sleep_ticks_lo |
			      ((u64)lp->s2idle_control.sleep_ticks_hi << 32);
		wake_ticks = (u64)lp->s2idle_control.wake_ticks_lo |
			     ((u64)lp->s2idle_control.wake_ticks_hi << 32);
		dev_info(lp->dev,
			 "s2idle wake reason=%#x raw=%#x lp_cycles=%llu requested_ms=%u\n",
			 lp->s2idle_control.wake_reason,
			 lp->s2idle_control.wake_raw,
			 wake_ticks - sleep_ticks,
			 esp32s31_lp_s2idle_wake_ms);
	}
	ret = esp32s31_lp_sleep_reclaim(&lp->s2idle_control);
	if (ret)
		dev_warn(lp->dev, "failed to reclaim LP sleep state: %d\n", ret);
	lp->s2idle_armed = false;
}

static void esp32s31_lp_s2idle_end(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);

	if (!lp || !lp->s2idle_armed)
		return;
	esp32s31_lp_sleep_abort(&lp->s2idle_control);
	esp32s31_lp_sleep_reclaim(&lp->s2idle_control);
	lp->s2idle_armed = false;
}

static const struct platform_s2idle_ops esp32s31_lp_s2idle_ops = {
	.begin = esp32s31_lp_s2idle_begin,
	.prepare = esp32s31_lp_s2idle_prepare,
	.check = esp32s31_lp_s2idle_check,
	.wake = esp32s31_lp_s2idle_wake,
	.restore = esp32s31_lp_s2idle_restore,
	.end = esp32s31_lp_s2idle_end,
};

static int esp32s31_lp_start_tx(struct esp32s31_lp *lp, u32 message)
{
	unsigned long flags;
	u8 slot;

	spin_lock_irqsave(&lp->tx_state_lock, flags);
	if (!READ_ONCE(lp->ready)) {
		spin_unlock_irqrestore(&lp->tx_state_lock, flags);
		return -ENOTCONN;
	}
	if (lp->tx_pending) {
		spin_unlock_irqrestore(&lp->tx_state_lock, flags);
		return -EBUSY;
	}

	slot = lp->tx_next;
	lp->tx_next += 2;
	if (lp->tx_next > 14)
		lp->tx_next = 8;
	lp->tx_slot = slot;
	lp->tx_pending = true;
	reinit_completion(&lp->tx_done);

	writel(BIT(slot) | BIT(slot + 1), lp->mailbox + S31_MB_HP_CLR);
	writel(message, lp->mailbox + S31_MB_MSG(slot));
	/* A mailbox write signals both cores. Clear the HP self-interrupt only. */
	writel(BIT(slot), lp->mailbox + S31_MB_HP_CLR);
	atomic64_inc(&lp->tx_count);
	spin_unlock_irqrestore(&lp->tx_state_lock, flags);

	return 0;
}

static int esp32s31_lp_send_sync(struct esp32s31_lp *lp, u32 message)
{
	unsigned long flags;
	long timeout;
	int ret;

	mutex_lock(&lp->user_tx_lock);
	ret = esp32s31_lp_start_tx(lp, message);
	if (ret)
		goto out_unlock;

	timeout = wait_for_completion_interruptible_timeout(
		&lp->tx_done, msecs_to_jiffies(S31_LP_TIMEOUT_MS));
	if (timeout > 0) {
		ret = 0;
	} else {
		spin_lock_irqsave(&lp->tx_state_lock, flags);
		lp->tx_pending = false;
		writel(BIT(lp->tx_slot + 1), lp->mailbox + S31_MB_HP_CLR);
		spin_unlock_irqrestore(&lp->tx_state_lock, flags);
		atomic64_inc(&lp->timeout_count);
		ret = timeout ? (int)timeout : -ETIMEDOUT;
	}

out_unlock:
	mutex_unlock(&lp->user_tx_lock);
	return ret;
}

static int esp32s31_lp_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct esp32s31_lp *lp = container_of(chan->mbox,
					      struct esp32s31_lp, mbox);

	if (!data)
		return -EINVAL;

	return esp32s31_lp_start_tx(lp, *(u32 *)data);
}

static const struct mbox_chan_ops esp32s31_lp_mbox_ops = {
	.send_data = esp32s31_lp_mbox_send_data,
};

static irqreturn_t esp32s31_lp_irq(int irq, void *data)
{
	struct esp32s31_lp *lp = data;
	unsigned long flags;
	u32 status;
	u32 handled = 0;
	bool tx_done = false;
	int slot;

	status = readl(lp->mailbox + S31_MB_HP_ST) & S31_MB_IRQ_MASK;
	if (!status)
		return IRQ_NONE;

	for (slot = 0; slot <= 6; slot += 2) {
		u32 message;

		if (!(status & BIT(slot)))
			continue;

		message = readl(lp->mailbox + S31_MB_MSG(slot));
		writel(S31_MB_ACK, lp->mailbox + S31_MB_MSG(slot + 1));
		writel(BIT(slot) | BIT(slot + 1),
		       lp->mailbox + S31_MB_HP_CLR);
		handled |= BIT(slot);

#if IS_ENABLED(CONFIG_TTY)
		if (message == S31_LP_MSG_UART_EVENT)
			esp32s31_lp_uart_event(lp);
#endif
		WRITE_ONCE(lp->last_message, message);
		if (message == S31_LP_MSG_WAKE) {
			pm_wakeup_event(lp->dev, 0);
			s2idle_wake();
		}
		if ((message & S31_LP_MESSAGE_MASK) == S31_LP_RSP_PONG)
			lp->ping_rtt_us = ktime_us_delta(ktime_get(), lp->ping_started);
		if (READ_ONCE(lp->rpc_expected) == message ||
		    (((message & S31_LP_MESSAGE_MASK) == S31_LP_RSP_ERROR) &&
		     ((message & S31_LP_SEQUENCE_MASK) ==
		      (READ_ONCE(lp->rpc_expected) & S31_LP_SEQUENCE_MASK)))) {
			WRITE_ONCE(lp->rpc_response, message);
			complete(&lp->rpc_done);
		}

		spin_lock_irqsave(&lp->rx_lock, flags);
		kfifo_in(&lp->rx_fifo, &message, 1);
		spin_unlock_irqrestore(&lp->rx_lock, flags);
		atomic64_inc(&lp->rx_count);
		wake_up_interruptible(&lp->rx_wait);
		/* The misc/sysfs ABI works without a mailbox-framework client. */
		if (READ_ONCE(lp->chan.cl))
			mbox_chan_received_data(&lp->chan, &message);
	}

	spin_lock_irqsave(&lp->tx_state_lock, flags);
	if (lp->tx_pending && (status & BIT(lp->tx_slot + 1))) {
		writel(BIT(lp->tx_slot + 1), lp->mailbox + S31_MB_HP_CLR);
		handled |= BIT(lp->tx_slot + 1);
		lp->tx_pending = false;
		tx_done = true;
		atomic64_inc(&lp->ack_count);
	}
	spin_unlock_irqrestore(&lp->tx_state_lock, flags);

	if (tx_done) {
		complete(&lp->tx_done);
		if (READ_ONCE(lp->chan.cl))
			mbox_chan_txdone(&lp->chan, 0);
	}

	if (status & ~handled)
		writel(status & ~handled, lp->mailbox + S31_MB_HP_CLR);

	return IRQ_HANDLED;
}

static u32 esp32s31_lp_crc(const void *buffer, size_t length)
{
	return crc32_le(~0U, buffer, length) ^ ~0U;
}

static int esp32s31_lp_read_control(struct esp32s31_lp *lp,
				    struct s31_lp_sleep_control *control)
{
	int attempt;

	/*
	 * An already-active GPIO level may move ARMED to WAKING immediately
	 * after the LP core sends the ARM response.  In that case Linux can
	 * observe the shared record between the field writes and the final CRC
	 * write.  Retry the bounded snapshot instead of treating that legitimate
	 * state transition as protocol corruption.
	 */
	for (attempt = 0; attempt < 100; attempt++) {
		memcpy_fromio(control,
			      lp->sram + S31_LP_SLEEP_CONTROL_OFFSET,
			      sizeof(*control));
		if (control->magic != S31_LP_SLEEP_CONTROL_MAGIC ||
		    control->version != S31_LP_ABI_VERSION ||
		    control->size != sizeof(*control))
			return -EPROTO;
		if (esp32s31_lp_crc(control,
			    offsetof(struct s31_lp_sleep_control, response_crc)) ==
		    control->response_crc)
			return 0;
		udelay(1);
	}

	return -EBADMSG;
}

static u32 esp32s31_lp_next_sequence(struct esp32s31_lp *lp)
{
	u32 sequence = atomic_inc_return(&lp->rpc_sequence) & S31_LP_SEQUENCE_MASK;

	if (!sequence)
		sequence = atomic_inc_return(&lp->rpc_sequence) & S31_LP_SEQUENCE_MASK;
	return sequence;
}

static int esp32s31_lp_sleep_rpc(u32 command, u32 expected,
				 struct s31_lp_sleep_control *control,
				 bool write_request)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);
	u32 sequence;
	long timeout;
	int ret;

	if (!lp || !READ_ONCE(lp->ready))
		return -ENODEV;
	if (!control)
		return -EINVAL;

	mutex_lock(&lp->rpc_lock);
	sequence = esp32s31_lp_next_sequence(lp);

	if (write_request) {
		control->magic = S31_LP_SLEEP_CONTROL_MAGIC;
		control->version = S31_LP_ABI_VERSION;
		control->size = sizeof(*control);
		control->sequence = sequence;
		control->request_crc = esp32s31_lp_crc(control,
			offsetof(struct s31_lp_sleep_control, request_crc));
		memcpy_toio(lp->sram + S31_LP_SLEEP_CONTROL_OFFSET, control,
			    sizeof(*control));
	}

	reinit_completion(&lp->rpc_done);
	WRITE_ONCE(lp->rpc_response, 0);
	WRITE_ONCE(lp->rpc_expected, expected | sequence);
	ret = esp32s31_lp_send_sync(lp, command | sequence);
	if (ret)
		goto out_clear;

	timeout = wait_for_completion_interruptible_timeout(&lp->rpc_done,
					msecs_to_jiffies(S31_LP_TIMEOUT_MS));
	if (timeout <= 0) {
		ret = timeout ? (int)timeout : -ETIMEDOUT;
		goto out_clear;
	}

	ret = esp32s31_lp_read_control(lp, control);
	if (ret)
		goto out_clear;
	ret = control->result == S31_LP_SLEEP_OK ? 0 : -EREMOTEIO;

out_clear:
	WRITE_ONCE(lp->rpc_expected, 0);
	mutex_unlock(&lp->rpc_lock);
	return ret;
}

bool esp32s31_lp_sleep_available(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);

	return lp && READ_ONCE(lp->ready);
}
EXPORT_SYMBOL_GPL(esp32s31_lp_sleep_available);

int esp32s31_lp_sleep_prepare(struct s31_lp_sleep_control *control)
{
	return esp32s31_lp_sleep_rpc(S31_LP_CMD_SLEEP_PREPARE,
		S31_LP_RSP_SLEEP_PREPARED, control, true);
}
EXPORT_SYMBOL_GPL(esp32s31_lp_sleep_prepare);

int esp32s31_lp_sleep_arm(struct s31_lp_sleep_control *control)
{
	return esp32s31_lp_sleep_rpc(S31_LP_CMD_SLEEP_ARM,
		S31_LP_RSP_SLEEP_ARMED, control, true);
}
EXPORT_SYMBOL_GPL(esp32s31_lp_sleep_arm);

int esp32s31_lp_sleep_abort(struct s31_lp_sleep_control *control)
{
	return esp32s31_lp_sleep_rpc(S31_LP_CMD_SLEEP_ABORT,
		S31_LP_RSP_SLEEP_ABORTED, control, false);
}
EXPORT_SYMBOL_GPL(esp32s31_lp_sleep_abort);

int esp32s31_lp_sleep_query(struct s31_lp_sleep_control *control)
{
	return esp32s31_lp_sleep_rpc(S31_LP_CMD_SLEEP_QUERY,
		S31_LP_RSP_SLEEP_STATUS, control, false);
}
EXPORT_SYMBOL_GPL(esp32s31_lp_sleep_query);

int esp32s31_lp_sleep_reclaim(struct s31_lp_sleep_control *control)
{
	return esp32s31_lp_sleep_rpc(S31_LP_CMD_SLEEP_RECLAIM,
		S31_LP_RSP_SLEEP_RECLAIMED, control, false);
}
EXPORT_SYMBOL_GPL(esp32s31_lp_sleep_reclaim);

int esp32s31_lp_system_suspend_prepare(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);
	int ret;

	if (!lp || !READ_ONCE(lp->ready))
		return -ENODEV;
	if (esp32s31_lp_mem_wake_ms < 10 ||
	    esp32s31_lp_mem_wake_ms > 600000)
		return -ERANGE;
	if (esp32s31_lp_mem_wake_gpio < -1 ||
	    esp32s31_lp_mem_wake_gpio > 7 || esp32s31_lp_mem_gpio_pull > 2)
		return -ERANGE;
	memset(&lp->mem_control, 0, sizeof(lp->mem_control));
	lp->mem_control.flags = S31_LP_SLEEP_F_MEM;
	lp->mem_control.wake_mask = S31_LP_WAKE_TIMER;
	lp->mem_control.deadline_lo = esp32s31_lp_mem_wake_ms * 1000U;
	if (esp32s31_lp_mem_wake_gpio >= 0) {
		u32 gpio = BIT(esp32s31_lp_mem_wake_gpio);

		lp->mem_control.wake_mask |= S31_LP_WAKE_GPIO;
		lp->mem_control.gpio_mask_lo = gpio;
		if (esp32s31_lp_mem_gpio_active_high)
			lp->mem_control.gpio_level_lo = gpio;
		if (esp32s31_lp_mem_gpio_pull == 1)
			lp->mem_control.flags |= S31_LP_SLEEP_F_GPIO_PULL_UP;
		else if (esp32s31_lp_mem_gpio_pull == 2)
			lp->mem_control.flags |= S31_LP_SLEEP_F_GPIO_PULL_DOWN;
	}
	/* Conservative stage one: retain every domain until measured otherwise. */
	lp->mem_control.retention_mask = ~0U;
	lp->mem_control.domain_mask = ~0U;
	lp->mem_control.clock_mask = ~0U;
	lp->mem_armed = false;

	ret = esp32s31_lp_sleep_prepare(&lp->mem_control);
	if (ret)
		return ret;
	if (!(lp->mem_control.capabilities & S31_LP_CAP_RETENTION_DESC))
		return -EOPNOTSUPP;
	ret = esp32s31_lp_sleep_arm(&lp->mem_control);
	if (ret) {
		esp32s31_lp_sleep_abort(&lp->mem_control);
		return ret;
	}
	lp->mem_armed = true;
	dev_info(lp->dev, "armed suspend-to-RAM timer for %u ms%s\n",
		 esp32s31_lp_mem_wake_ms,
		 esp32s31_lp_mem_wake_gpio >= 0 ? " with LP GPIO wake" : "");
	return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_lp_system_suspend_prepare);

void esp32s31_lp_system_suspend_finish(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);
	int ret;

	if (!lp || !lp->mem_armed)
		return;
	ret = esp32s31_lp_sleep_query(&lp->mem_control);
	if (ret)
		dev_warn(lp->dev, "failed to query suspend wake: %d\n", ret);
	else
		dev_info(lp->dev, "resume state=%u reason=%#x raw=%#x\n",
			 lp->mem_control.state, lp->mem_control.wake_reason,
			 lp->mem_control.wake_raw);
	ret = esp32s31_lp_sleep_reclaim(&lp->mem_control);
	if (ret)
		dev_warn(lp->dev, "failed to reclaim suspend state: %d\n", ret);
	lp->mem_armed = false;
}
EXPORT_SYMBOL_GPL(esp32s31_lp_system_suspend_finish);

static int esp32s31_lp_peripheral_rpc(struct esp32s31_lp *lp,
				      struct s31_lp_rpc_slot *request)
{
	struct s31_lp_rpc_ring __iomem *ring = lp->sram + S31_LP_RPC_OFFSET;
	struct s31_lp_rpc_slot __iomem *slot;
	u32 operation = request->operation;
	u32 output_capacity = request->output_len;
	u32 producer, sequence;
	long timeout;
	int ret;

	ret = mutex_lock_interruptible(&lp->rpc_lock);
	if (ret)
		return ret;
	/* Pair with attach publishing the validated firmware capabilities. */
	if (!smp_load_acquire(&lp->ready)) {
		ret = -ENOTCONN;
		goto unlock;
	}
	if (!lp->peripheral_rpc) {
		ret = -EOPNOTSUPP;
		goto unlock;
	}

	producer = readl(&ring->producer);
	if (producer != readl(&ring->consumer)) {
		ret = -EBUSY;
		goto unlock;
	}

	sequence = esp32s31_lp_next_sequence(lp);
	slot = &ring->slots[producer % S31_LP_RPC_SLOTS];
	request->generation = producer + 1;
	request->sequence = sequence;
	memcpy_toio(slot, request, sizeof(*request));
	/* TRM ch.11 LP Mailbox: the doorbell follows the LP SRAM publication. */
	writel(producer + 1, &ring->producer);

	reinit_completion(&lp->rpc_done);
	WRITE_ONCE(lp->rpc_response, 0);
	WRITE_ONCE(lp->rpc_expected, S31_LP_RSP_PERIPHERAL_RPC | sequence);
	ret = esp32s31_lp_send_sync(lp, S31_LP_CMD_PERIPHERAL_RPC | sequence);
	if (ret) {
		/* These errors occur before start_tx writes the mailbox. */
		if (ret == -EBUSY || ret == -ENOTCONN)
			writel(producer, &ring->producer);
		goto clear;
	}

	timeout = wait_for_completion_interruptible_timeout(&lp->rpc_done,
						msecs_to_jiffies(S31_LP_TIMEOUT_MS));
	if (timeout <= 0) {
		ret = timeout ? (int)timeout : -ETIMEDOUT;
		goto clear;
	}
	if (READ_ONCE(lp->rpc_response) != (S31_LP_RSP_PERIPHERAL_RPC | sequence) ||
	    readl(&ring->consumer) != producer + 1) {
		ret = -EPROTO;
		goto clear;
	}

	memcpy_fromio(request, slot, sizeof(*request));
	if (request->generation != producer + 1 || request->operation != operation ||
	    request->sequence != sequence || request->output_len > output_capacity) {
		ret = -EPROTO;
		goto clear;
	}
	switch (request->result) {
	case S31_LP_RPC_OK:
		ret = 0;
		break;
	case S31_LP_RPC_INVALID:
		ret = -EINVAL;
		break;
	case S31_LP_RPC_BUSY:
		ret = -EBUSY;
		break;
	case S31_LP_RPC_NACK:
		ret = -EREMOTEIO;
		break;
	case S31_LP_RPC_TIMEOUT:
		ret = -ETIMEDOUT;
		break;
	case S31_LP_RPC_IO:
		ret = -EIO;
		break;
	case S31_LP_RPC_ARBITRATION:
		ret = -EAGAIN;
		break;
	case S31_LP_RPC_UNSUPPORTED:
		ret = -EOPNOTSUPP;
		break;
	default:
		ret = -EPROTO;
		break;
	}

clear:
	WRITE_ONCE(lp->rpc_expected, 0);
unlock:
	mutex_unlock(&lp->rpc_lock);
	return ret;
}

#if IS_REACHABLE(CONFIG_I2C)
static int esp32s31_lp_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct esp32s31_lp *lp = i2c_get_adapdata(adapter);
	struct s31_lp_rpc_slot request = {
		.operation = S31_LP_RPC_I2C_XFER,
		.args = { 0, lp->i2c_pins, lp->i2c_hz, lp->i2c_flags },
	};
	struct i2c_msg *rd = NULL, *wr = NULL;
	int ret;

	if (num < 1 || num > 2)
		return -EINVAL;
	request.args[0] = msgs[0].addr;
	for (int i = 0; i < num; i++) {
		if (msgs[i].len > sizeof(request.data))
			return -EMSGSIZE;
		if (msgs[i].flags & ~(I2C_M_RD | I2C_M_DMA_SAFE))
			return -EOPNOTSUPP;
		if (msgs[i].flags & I2C_M_RD)
			rd = &msgs[i];
		else
			wr = &msgs[i];
	}

	if (wr) {
		request.input_len = wr->len;
		memcpy(request.data, wr->buf, wr->len);
	}
	if (rd)
		request.output_len = rd->len;

	ret = esp32s31_lp_peripheral_rpc(lp, &request);
	if (ret)
		return ret;
	if (rd) {
		if (request.output_len != rd->len)
			return -EPROTO;
		memcpy(rd->buf, request.data, rd->len);
	}
	return num;
}

static u32 esp32s31_lp_i2c_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

/* TRM ch.26.5 LP I2C has a 16-byte FIFO, including address bytes on TX. */
static const struct i2c_adapter_quirks esp32s31_lp_i2c_quirks = {
	.flags = I2C_AQ_COMB_WRITE_THEN_READ | I2C_AQ_NO_ZERO_LEN,
	.max_num_msgs = 2,
	.max_write_len = S31_LP_I2C_FIFO_BYTES - 1,
	.max_read_len = S31_LP_I2C_FIFO_BYTES,
	.max_comb_1st_msg_len = S31_LP_I2C_FIFO_BYTES - 2,
	.max_comb_2nd_msg_len = S31_LP_I2C_FIFO_BYTES,
};

static const struct i2c_algorithm esp32s31_lp_i2c_algo = {
	.xfer = esp32s31_lp_i2c_xfer,
	.functionality = esp32s31_lp_i2c_functionality,
};

static void esp32s31_lp_del_i2c(void *data)
{
	struct esp32s31_lp *lp = data;
	struct device_node *node = lp->i2c.dev.of_node;

	if (lp->i2c_registered) {
		i2c_del_adapter(&lp->i2c);
		of_node_put(node);
	}
}

static int esp32s31_lp_add_i2c(struct esp32s31_lp *lp)
{
	struct device_node *node __free(device_node) =
		of_get_compatible_child(lp->dev->of_node, "espressif,esp32s31-lp-i2c");
	struct s31_lp_rpc_slot caps = { .operation = S31_LP_RPC_CAPABILITIES };
	struct i2c_adapter *adapter = &lp->i2c;
	u32 sda, scl, hz = S31_LP_I2C_STANDARD_HZ;
	int ret;

	if (!of_device_is_available(node) || lp->i2c_registered)
		return 0;
	if (of_property_read_u32(node, "espressif,sda-pin", &sda) ||
	    of_property_read_u32(node, "espressif,scl-pin", &scl) ||
	    sda >= S31_LP_GPIO_COUNT || scl >= S31_LP_GPIO_COUNT || sda == scl)
		return -EINVAL;
	if (of_property_present(node, "clock-frequency") &&
	    of_property_read_u32(node, "clock-frequency", &hz))
		return -EINVAL;
	if (hz != S31_LP_I2C_STANDARD_HZ && hz != S31_LP_I2C_FAST_HZ)
		return -EINVAL;
	ret = esp32s31_lp_peripheral_rpc(lp, &caps);
	if (ret)
		return ret;
	if (!(caps.args[0] & S31_LP_SERVICE_I2C))
		return -EOPNOTSUPP;

	lp->i2c_pins = sda | scl << 8;
	lp->i2c_hz = hz;
	lp->i2c_flags = of_property_read_bool(node, "espressif,internal-pullups") ?
			S31_LP_I2C_PULLUP : 0;
	adapter->owner = THIS_MODULE;
	adapter->algo = &esp32s31_lp_i2c_algo;
	adapter->quirks = &esp32s31_lp_i2c_quirks;
	adapter->dev.parent = lp->dev;
	adapter->dev.of_node = node;
	strscpy(adapter->name, "ESP32-S31 LP I2C", sizeof(adapter->name));
	i2c_set_adapdata(adapter, lp);
	ret = i2c_add_adapter(adapter);
	if (!ret) {
		of_node_get(node);
		lp->i2c_registered = true;
	}
	return ret;
}
#endif

#if IS_REACHABLE(CONFIG_SPI)
static int esp32s31_lp_spi_rpc(struct esp32s31_lp *lp, u32 mode, u32 speed, u32 action,
			       const u8 *tx, u8 *rx, unsigned int len, u32 *effective)
{
	struct s31_lp_rpc_slot req = {
		.operation = S31_LP_RPC_SPI,
		.args = { lp->spi_pins, speed, mode, action },
		.input_len = tx ? len : 0,
		.output_len = rx ? len : 0,
	};
	int ret;

	if (tx)
		memcpy(req.data, tx, len);
	ret = esp32s31_lp_peripheral_rpc(lp, &req);
	if (ret)
		return ret;
	if (rx) {
		if (req.output_len != len)
			return -EPROTO;
		memcpy(rx, req.data, len);
	}
	if (effective)
		*effective = req.args[1];
	return 0;
}

static int esp32s31_lp_spi_release(struct esp32s31_lp *lp)
{
	int ret;

	if (!lp->spi_selected)
		return 0;
	ret = esp32s31_lp_spi_rpc(lp, lp->spi_mode, 0, S31_LP_SPI_RELEASE,
				   NULL, NULL, 0, NULL);
	if (!ret)
		lp->spi_selected = false;
	return ret;
}

static int esp32s31_lp_spi_message(struct spi_controller *ctlr, struct spi_message *msg)
{
	struct esp32s31_lp *lp = spi_controller_get_devdata(ctlr);
	struct spi_device *spi = msg->spi;
	const u32 mode = spi->mode & (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST);
	struct spi_transfer *xfer, *last = NULL;
	bool keep_cs = false;
	int ret = 0;

	static_assert(S31_LP_SPI_CPHA == SPI_CPHA && S31_LP_SPI_CPOL == SPI_CPOL);
	static_assert(S31_LP_SPI_CS_HIGH == SPI_CS_HIGH && S31_LP_SPI_LSB_FIRST == SPI_LSB_FIRST);

	mutex_lock(&lp->spi_lock);
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		u32 speed = xfer->speed_hz;
		bool first = !last;
		const u8 *tx = xfer->tx_buf;
		u8 *rx = xfer->rx_buf;

		/* Drop a selection retained by the preceding message or transfer. */
		if (lp->spi_selected && (xfer->cs_off || lp->spi_mode != mode)) {
			ret = spi_delay_exec(&spi->cs_hold, last ?: xfer);
			if (!ret)
				ret = esp32s31_lp_spi_release(lp);
			if (!ret)
				ret = spi_delay_exec(&spi->cs_inactive, last ?: xfer);
			if (ret)
				break;
		}
		last = xfer;
		if (!xfer->cs_off && (first || !lp->spi_selected)) {
			/* SELECT is idempotent, including after an interrupted RPC. */
			lp->spi_selected = true;
			lp->spi_mode = mode;
			ret = esp32s31_lp_spi_rpc(lp, mode, speed, S31_LP_SPI_SELECT,
						   NULL, NULL, 0, &xfer->effective_speed_hz);
			if (ret == -ENOTCONN)
				lp->spi_selected = false;
			if (ret)
				break;
			ret = spi_delay_exec(&spi->cs_setup, xfer);
			if (ret)
				break;
		}

		if (xfer->len && !tx && !rx) {
			ret = -EINVAL;
			break;
		}
		for (unsigned int offset = 0; offset < xfer->len; ) {
			unsigned int count = xfer->word_delay.value ? 1 :
					     min(xfer->len - offset, S31_LP_SPI_CHUNK_BYTES);

			ret = esp32s31_lp_spi_rpc(lp, mode, speed, S31_LP_SPI_CLOCK,
						   tx ? tx + offset : NULL,
						   rx ? rx + offset : NULL, count,
						   &xfer->effective_speed_hz);
			if (ret)
				break;
			offset += count;
			msg->actual_length += count;
			if (offset < xfer->len && xfer->word_delay.value) {
				ret = spi_delay_exec(&xfer->word_delay, xfer);
				if (ret)
					break;
			}
		}
		if (ret)
			break;

		ret = spi_transfer_delay_exec(xfer);
		if (ret)
			break;
		if (!xfer->cs_change)
			continue;
		if (list_is_last(&xfer->transfer_list, &msg->transfers)) {
			keep_cs = !xfer->cs_off;
			continue;
		}
		if (lp->spi_selected) {
			ret = spi_delay_exec(&spi->cs_hold, xfer);
			if (!ret)
				ret = esp32s31_lp_spi_release(lp);
			if (!ret)
				ret = spi_delay_exec(&spi->cs_inactive, xfer);
			if (ret)
				break;
		}
		if (!xfer->cs_change_delay.value &&
		    xfer->cs_change_delay.unit == SPI_DELAY_UNIT_USECS) {
			/* Match the default delay in the SPI core. */
			udelay(10);
		} else {
			ret = spi_delay_exec(&xfer->cs_change_delay, xfer);
			if (ret)
				break;
		}
	}

	if ((ret || !keep_cs) && lp->spi_selected) {
		int cleanup;

		if (!ret && last)
			ret = spi_delay_exec(&spi->cs_hold, last);
		cleanup = esp32s31_lp_spi_release(lp);
		if (!ret)
			ret = cleanup;
		if (!ret && last)
			ret = spi_delay_exec(&spi->cs_inactive, last);
	}
	mutex_unlock(&lp->spi_lock);

	msg->status = ret;
	spi_finalize_current_message(ctlr);
	return 0;
}

static void esp32s31_lp_del_spi(void *data)
{
	struct esp32s31_lp *lp = data;

	if (lp->spi) {
		struct device_node *node = lp->spi->dev.of_node;

		spi_unregister_controller(lp->spi);
		of_node_put(node);
	}
}

static int esp32s31_lp_add_spi(struct esp32s31_lp *lp)
{
	struct device_node *node __free(device_node) =
		of_get_compatible_child(lp->dev->of_node, "espressif,esp32s31-lp-spi");
	struct s31_lp_rpc_slot caps = { .operation = S31_LP_RPC_CAPABILITIES };
	struct spi_controller *ctlr;
	u32 pin[4], mask = 0;
	int ret;

	if (!of_device_is_available(node) || lp->spi)
		return 0;
	/* SCLK, MOSI, MISO, and LP-driven chip select. */
	if (of_property_read_u32_array(node, "espressif,pins", pin, ARRAY_SIZE(pin)))
		return -EINVAL;
	for (unsigned int i = 0; i < ARRAY_SIZE(pin); i++) {
		if (pin[i] >= S31_LP_GPIO_COUNT || mask & BIT(pin[i]))
			return -EINVAL;
		mask |= BIT(pin[i]);
	}
	ret = esp32s31_lp_peripheral_rpc(lp, &caps);
	if (ret)
		return ret;
	if (!(caps.args[0] & S31_LP_SERVICE_SPI))
		return -EOPNOTSUPP;

	ctlr = spi_alloc_host(lp->dev, 0);
	if (!ctlr)
		return -ENOMEM;
	lp->spi_pins = pin[0] | pin[1] << 8 | pin[2] << 16 | pin[3] << 24;
	ctlr->dev.of_node = node;
	ctlr->bus_num = -1;
	ctlr->num_chipselect = 1;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8);
	ctlr->min_speed_hz = S31_LP_SPI_MIN_HZ;
	ctlr->max_speed_hz = S31_LP_SPI_MAX_HZ;
	ctlr->transfer_one_message = esp32s31_lp_spi_message;
	spi_controller_set_devdata(ctlr, lp);
	ret = spi_register_controller(ctlr);
	if (ret) {
		spi_controller_put(ctlr);
		return ret;
	}
	of_node_get(node);
	lp->spi = ctlr;
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_TTY)
static int esp32s31_lp_uart_config(struct esp32s31_lp_uart *uart, struct tty_struct *tty)
{
	tcflag_t cflag = tty->termios.c_cflag;
	unsigned int baud = tty_get_baud_rate(tty);
	u32 format = (cflag & CSIZE) >> __ffs(CSIZE);
	struct s31_lp_rpc_slot request = {
		.operation = S31_LP_RPC_UART,
		.args = { S31_LP_UART_CONFIG, uart->pins },
	};
	int ret;

	if (!uart->lp)
		return -ENODEV;
	if (!READ_ONCE(uart->lp->ready))
		return -ENOTCONN;

	if (cflag & CSTOPB)
		format |= S31_LP_UART_STOP_2;
	if (cflag & PARENB)
		format |= S31_LP_UART_PARITY;
	if (cflag & PARODD)
		format |= S31_LP_UART_ODD;

	request.args[2] = clamp(baud ?: 115200, S31_LP_UART_MIN_BAUD, S31_LP_UART_MAX_BAUD);
	request.args[3] = format;

	/* A timed-out CONFIG may still be executed by the LP firmware. */
	uart->needs_stop = true;
	ret = esp32s31_lp_peripheral_rpc(uart->lp, &request);
	if (ret)
		return ret;
	if (!request.args[2])
		return -EPROTO;

	tty->termios.c_cflag &= ~(CMSPAR | CRTSCTS);
	tty->termios.c_cflag |= CLOCAL;
	tty_encode_baud_rate(tty, request.args[2], request.args[2]);
	return 0;
}

static int esp32s31_lp_uart_stop(struct esp32s31_lp_uart *uart)
{
	struct s31_lp_rpc_slot request = {
		.operation = S31_LP_RPC_UART,
		.args = { S31_LP_UART_STOP },
	};
	int ret;

	if (!uart->needs_stop)
		return 0;
	ret = esp32s31_lp_peripheral_rpc(uart->lp, &request);
	if (!ret)
		uart->needs_stop = false;
	return ret;
}

static int esp32s31_lp_uart_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct esp32s31_lp_uart *uart = container_of(port, struct esp32s31_lp_uart, port);
	int ret;

	mutex_lock(&uart->lock);
	ret = esp32s31_lp_uart_config(uart, tty);
	if (!ret) {
		WRITE_ONCE(uart->tx_pending, 0);
		WRITE_ONCE(uart->active, true);
	} else if (uart->lp && esp32s31_lp_uart_stop(uart)) {
		dev_warn(uart->lp->dev, "could not disable LP UART after activation failed\n");
	}
	mutex_unlock(&uart->lock);
	return ret;
}

static void esp32s31_lp_uart_shutdown(struct tty_port *port)
{
	struct esp32s31_lp_uart *uart = container_of(port, struct esp32s31_lp_uart, port);
	int ret;

	mutex_lock(&uart->lock);
	WRITE_ONCE(uart->active, false);
	if (uart->lp) {
		ret = esp32s31_lp_uart_stop(uart);
		if (ret)
			dev_warn(uart->lp->dev, "LP UART shutdown failed: %d\n", ret);
	}
	mutex_unlock(&uart->lock);
}

static void esp32s31_lp_uart_destruct(struct tty_port *port)
{
	kfree(container_of(port, struct esp32s31_lp_uart, port));
}

static const struct tty_port_operations esp32s31_lp_uart_port_ops = {
	.activate = esp32s31_lp_uart_activate,
	.shutdown = esp32s31_lp_uart_shutdown,
	.destruct = esp32s31_lp_uart_destruct,
};

static int esp32s31_lp_uart_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct esp32s31_lp_uart *uart = driver->driver_state;
	struct tty_port *port = tty_port_get(&uart->port);
	int ret;

	ret = tty_port_install(port, driver, tty);
	if (ret)
		tty_port_put(port);
	return ret;
}

static void esp32s31_lp_uart_tty_put(struct tty_struct *tty)
{
	tty_port_put(tty->port);
}

static int esp32s31_lp_uart_exchange(struct esp32s31_lp_uart *uart, const u8 *buf,
				    unsigned int len)
{
	struct s31_lp_rpc_slot request = {
		.operation = S31_LP_RPC_UART,
		.input_len = len,
		.output_len = S31_LP_RPC_DATA_BYTES,
		.args = { S31_LP_UART_EXCHANGE },
	};
	u32 errors;
	int ret, copied;

	if (len)
		memcpy(request.data, buf, len);
	ret = esp32s31_lp_peripheral_rpc(uart->lp, &request);
	if (ret)
		return ret;
	if (request.args[1] > len || request.args[2] > 1)
		return -EPROTO;

	if (request.args[1] || request.args[2])
		WRITE_ONCE(uart->tx_pending, request.args[1]);
	errors = request.args[3];
	copied = tty_insert_flip_string(&uart->port, request.data, request.output_len);
	if (errors & S31_LP_UART_BREAK)
		tty_insert_flip_char(&uart->port, 0, TTY_BREAK);
	if (errors & S31_LP_UART_BAD_FRAME)
		tty_insert_flip_char(&uart->port, 0, TTY_FRAME);
	if (errors & S31_LP_UART_BAD_PARITY)
		tty_insert_flip_char(&uart->port, 0, TTY_PARITY);
	if (errors & S31_LP_UART_OVERRUN || copied != request.output_len)
		tty_insert_flip_char(&uart->port, 0, TTY_OVERRUN);
	if (request.output_len || errors)
		tty_flip_buffer_push(&uart->port);
	return request.args[1];
}

static void esp32s31_lp_uart_work(struct work_struct *work)
{
	struct esp32s31_lp_uart *uart = container_of(work, struct esp32s31_lp_uart, work);
	bool wake = false;
	int ret;

	mutex_lock(&uart->lock);
	if (uart->active) {
		ret = esp32s31_lp_uart_exchange(uart, NULL, 0);
		if (ret < 0)
			dev_warn_ratelimited(uart->lp->dev, "LP UART event failed: %d\n", ret);
		wake = !ret && !uart->tx_pending;
	}
	mutex_unlock(&uart->lock);

	if (wake)
		tty_port_tty_wakeup(&uart->port);
}

static void esp32s31_lp_uart_event(struct esp32s31_lp *lp)
{
	struct esp32s31_lp_uart *uart = READ_ONCE(lp->uart);

	if (uart)
		schedule_work(&uart->work);
}

static ssize_t esp32s31_lp_uart_write(struct tty_struct *tty, const u8 *buf, size_t len)
{
	struct esp32s31_lp_uart *uart = container_of(tty->port, struct esp32s31_lp_uart, port);
	int ret = -EIO;

	mutex_lock(&uart->lock);
	if (uart->active)
		ret = esp32s31_lp_uart_exchange(uart, buf,
					       min_t(size_t, len, S31_LP_UART_TX_BYTES));
	mutex_unlock(&uart->lock);
	return ret;
}

static unsigned int esp32s31_lp_uart_room(struct tty_struct *tty)
{
	struct esp32s31_lp_uart *uart = container_of(tty->port, struct esp32s31_lp_uart, port);

	return READ_ONCE(uart->active) && !READ_ONCE(uart->tx_pending) ? S31_LP_UART_TX_BYTES : 0;
}

static unsigned int esp32s31_lp_uart_pending(struct tty_struct *tty)
{
	struct esp32s31_lp_uart *uart = container_of(tty->port, struct esp32s31_lp_uart, port);

	return READ_ONCE(uart->active) ? READ_ONCE(uart->tx_pending) : 0;
}

static void esp32s31_lp_uart_termios(struct tty_struct *tty, const struct ktermios *old)
{
	struct esp32s31_lp_uart *uart = container_of(tty->port, struct esp32s31_lp_uart, port);
	int ret;

	mutex_lock(&uart->lock);
	if (uart->active) {
		ret = esp32s31_lp_uart_config(uart, tty);
		if (ret) {
			tty_termios_copy_hw(&tty->termios, old);
			dev_warn_ratelimited(uart->lp->dev, "LP UART config failed: %d\n", ret);
		}
	}
	mutex_unlock(&uart->lock);
}

static int esp32s31_lp_uart_open(struct tty_struct *tty, struct file *file)
{
	return tty_port_open(tty->port, tty, file);
}

static void esp32s31_lp_uart_close(struct tty_struct *tty, struct file *file)
{
	tty_port_close(tty->port, tty, file);
}

static void esp32s31_lp_uart_hangup(struct tty_struct *tty)
{
	tty_port_hangup(tty->port);
}

static const struct tty_operations esp32s31_lp_uart_tty_ops = {
	.install = esp32s31_lp_uart_install,
	.cleanup = esp32s31_lp_uart_tty_put,
	.open = esp32s31_lp_uart_open,
	.close = esp32s31_lp_uart_close,
	.hangup = esp32s31_lp_uart_hangup,
	.write = esp32s31_lp_uart_write,
	.write_room = esp32s31_lp_uart_room,
	.chars_in_buffer = esp32s31_lp_uart_pending,
	.set_termios = esp32s31_lp_uart_termios,
};

static void esp32s31_lp_del_uart(void *data)
{
	struct esp32s31_lp *lp = data;
	struct esp32s31_lp_uart *uart;
	int ret;

	mutex_lock(&lp->uart_lock);
	lp->uart_removing = true;
	uart = lp->uart;
	WRITE_ONCE(lp->uart, NULL);
	mutex_unlock(&lp->uart_lock);
	if (!uart)
		return;

	synchronize_irq(lp->irq);
	cancel_work_sync(&uart->work);
	mutex_lock(&uart->lock);
	WRITE_ONCE(uart->active, false);
	ret = esp32s31_lp_uart_stop(uart);
	if (ret)
		dev_warn(lp->dev, "could not disable LP UART on removal: %d\n", ret);
	uart->lp = NULL;
	mutex_unlock(&uart->lock);

	tty_port_tty_hangup(&uart->port, false);
	tty_unregister_device(uart->driver, 0);
	tty_unregister_driver(uart->driver);
	tty_driver_kref_put(uart->driver);
	tty_port_put(&uart->port);
}

static int esp32s31_lp_add_uart(struct esp32s31_lp *lp)
{
	struct s31_lp_rpc_slot caps = { .operation = S31_LP_RPC_CAPABILITIES };
	struct device_node *node __free(device_node) =
		of_get_child_by_name(lp->dev->of_node, "serial");
	struct esp32s31_lp_uart *uart;
	struct tty_driver *driver;
	struct device *tty_dev;
	u32 pin[2];
	int ret;

	guard(mutex)(&lp->uart_lock);
	if (lp->uart || lp->uart_removing || !of_device_is_available(node))
		return 0;

	if (of_property_count_u32_elems(node, "espressif,pins") != ARRAY_SIZE(pin) ||
	    of_property_read_u32_array(node, "espressif,pins", pin, ARRAY_SIZE(pin)) ||
	    pin[0] >= S31_LP_GPIO_COUNT || pin[1] >= S31_LP_GPIO_COUNT || pin[0] == pin[1])
		return -EINVAL;

	ret = esp32s31_lp_peripheral_rpc(lp, &caps);
	if (ret)
		return ret;
	if (!(caps.args[0] & S31_LP_SERVICE_UART))
		return -EOPNOTSUPP;

	driver = tty_alloc_driver(1, TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(driver))
		return PTR_ERR(driver);
	uart = kzalloc(sizeof(*uart), GFP_KERNEL);
	if (!uart) {
		tty_driver_kref_put(driver);
		return -ENOMEM;
	}

	uart->lp = lp;
	uart->driver = driver;
	uart->pins = pin[0] | pin[1] << 8;
	mutex_init(&uart->lock);
	INIT_WORK(&uart->work, esp32s31_lp_uart_work);
	tty_port_init(&uart->port);
	uart->port.ops = &esp32s31_lp_uart_port_ops;

	driver->driver_state = uart;
	driver->driver_name = "esp32s31-lp-uart";
	driver->name = "ttyLP";
	driver->type = TTY_DRIVER_TYPE_SERIAL;
	driver->subtype = SERIAL_TYPE_NORMAL;
	driver->init_termios = tty_std_termios;
	driver->init_termios.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
	tty_set_operations(driver, &esp32s31_lp_uart_tty_ops);
	ret = tty_register_driver(driver);
	if (ret)
		goto free_uart;

	WRITE_ONCE(lp->uart, uart);
	tty_dev = tty_port_register_device(&uart->port, driver, 0, lp->dev);
	if (IS_ERR(tty_dev)) {
		ret = PTR_ERR(tty_dev);
		WRITE_ONCE(lp->uart, NULL);
		synchronize_irq(lp->irq);
		cancel_work_sync(&uart->work);
		tty_unregister_driver(driver);
		goto free_uart;
	}
	return 0;

free_uart:
	tty_driver_kref_put(driver);
	tty_port_put(&uart->port);
	return ret;
}
#endif

static int esp32s31_lp_attach(struct rproc *rproc)
{
	struct esp32s31_lp *lp = rproc->priv;
	struct s31_lp_rpc_ring __iomem *ring = lp->sram + S31_LP_RPC_OFFSET;
	struct s31_lp_sleep_control control;
	int ret;

	static_assert(sizeof(*ring) <= S31_LP_RPC_BYTES);
	static_assert(S31_LP_SLEEP_CONTROL_OFFSET + sizeof(control) <= S31_LP_RPC_OFFSET);

	if (readl(lp->lp_sys + S31_LP_SYS_BOOT_ADDR_REG) != S31_LP_RESET_VECTOR ||
	    readl(lp->lp_sys + S31_LP_SYS_STORE0_REG) != S31_LP_STAGE_RUNNING)
		return dev_err_probe(lp->dev, -ENODEV,
				     "bootloader did not start LP firmware\n");

	ret = esp32s31_lp_read_control(lp, &control);
	if (ret)
		return dev_err_probe(lp->dev, ret, "invalid LP sleep control\n");
	if (!(control.capabilities & S31_LP_CAP_HANDSHAKE) ||
	    control.state == S31_LP_SLEEP_BOOT)
		return -EPROTO;

	lp->peripheral_rpc = control.capabilities & S31_LP_CAP_PERIPHERAL_RPC;
	if (lp->peripheral_rpc &&
	    (readl(&ring->magic) != S31_LP_RPC_MAGIC ||
	     readl(&ring->version) != S31_LP_RPC_VERSION))
		return -EPROTO;

	/* TRM ch.11 LP Mailbox: enable HP delivery and keep any pending messages. */
	if (!(readl(lp->mailbox + S31_MB_CLK_REG) & S31_MB_CLK_EN))
		return -EIO;
	writel(S31_MB_IRQ_MASK, lp->mailbox + S31_MB_HP_ENA);
	/* Publish peripheral_rpc before accepting its first request. */
	smp_store_release(&lp->ready, true);
#if IS_REACHABLE(CONFIG_I2C)
	ret = esp32s31_lp_add_i2c(lp);
	if (ret)
		dev_warn(lp->dev, "LP I2C unavailable: %d\n", ret);
#endif
#if IS_REACHABLE(CONFIG_SPI)
	ret = esp32s31_lp_add_spi(lp);
	if (ret)
		dev_warn(lp->dev, "LP SPI unavailable: %d\n", ret);
#endif
#if IS_ENABLED(CONFIG_TTY)
	ret = esp32s31_lp_add_uart(lp);
	if (ret)
		dev_warn(lp->dev, "LP UART unavailable: %d\n", ret);
#endif
	return 0;
}

static int esp32s31_lp_detach(struct rproc *rproc)
{
	struct esp32s31_lp *lp = rproc->priv;
	unsigned long flags;
	int ret = 0;

#if IS_ENABLED(CONFIG_TTY)
	guard(mutex)(&lp->uart_lock);
	if (lp->uart) {
		mutex_lock(&lp->uart->lock);
		ret = lp->uart->active ? -EBUSY : esp32s31_lp_uart_stop(lp->uart);
		if (ret) {
			mutex_unlock(&lp->uart->lock);
			return ret;
		}
	}
#endif

#if IS_REACHABLE(CONFIG_SPI)
	mutex_lock(&lp->spi_lock);
	ret = esp32s31_lp_spi_release(lp);
	if (ret)
		goto unlock_spi;
#endif
	mutex_lock(&lp->rpc_lock);
	mutex_lock(&lp->user_tx_lock);
	spin_lock_irqsave(&lp->tx_state_lock, flags);
	if (lp->tx_pending) {
		ret = -EBUSY;
	} else {
		WRITE_ONCE(lp->ready, false);
		writel(0, lp->mailbox + S31_MB_HP_ENA);
	}
	spin_unlock_irqrestore(&lp->tx_state_lock, flags);
	if (!ret)
		synchronize_irq(lp->irq);
	mutex_unlock(&lp->user_tx_lock);
	mutex_unlock(&lp->rpc_lock);
#if IS_REACHABLE(CONFIG_SPI)
unlock_spi:
	mutex_unlock(&lp->spi_lock);
#endif
#if IS_ENABLED(CONFIG_TTY)
	if (lp->uart)
		mutex_unlock(&lp->uart->lock);
#endif
	return ret;
}

static const struct rproc_ops esp32s31_lp_rproc_ops = {
	.attach = esp32s31_lp_attach,
	.detach = esp32s31_lp_detach,
};

static int esp32s31_lp_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct esp32s31_lp *lp = container_of(misc, struct esp32s31_lp, miscdev);

	file->private_data = lp;
	return 0;
}

static ssize_t esp32s31_lp_read(struct file *file, char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct esp32s31_lp *lp = file->private_data;
	unsigned long flags;
	u32 message;
	int ret;

	if (count < sizeof(message))
		return -EINVAL;

	if (file->f_flags & O_NONBLOCK) {
		if (kfifo_is_empty(&lp->rx_fifo))
			return -EAGAIN;
	} else {
		ret = wait_event_interruptible(lp->rx_wait,
					       !kfifo_is_empty(&lp->rx_fifo));
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&lp->rx_lock, flags);
	ret = kfifo_out(&lp->rx_fifo, &message, 1);
	spin_unlock_irqrestore(&lp->rx_lock, flags);
	if (!ret)
		return -EAGAIN;
	if (copy_to_user(buffer, &message, sizeof(message)))
		return -EFAULT;

	return sizeof(message);
}

static ssize_t esp32s31_lp_write(struct file *file, const char __user *buffer,
				 size_t count, loff_t *ppos)
{
	struct esp32s31_lp *lp = file->private_data;
	u32 message;
	int ret;

	if (count != sizeof(message))
		return -EINVAL;
	if (copy_from_user(&message, buffer, sizeof(message)))
		return -EFAULT;

	ret = esp32s31_lp_send_sync(lp, message);
	return ret ? ret : sizeof(message);
}

static __poll_t esp32s31_lp_poll(struct file *file, poll_table *wait)
{
	struct esp32s31_lp *lp = file->private_data;
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;

	poll_wait(file, &lp->rx_wait, wait);
	if (!kfifo_is_empty(&lp->rx_fifo))
		mask |= EPOLLIN | EPOLLRDNORM;
	return mask;
}

static const struct file_operations esp32s31_lp_fops = {
	.owner = THIS_MODULE,
	.open = esp32s31_lp_open,
	.read = esp32s31_lp_read,
	.write = esp32s31_lp_write,
	.poll = esp32s31_lp_poll,
	.llseek = noop_llseek,
};

static ssize_t ready_show(struct device *dev, struct device_attribute *attr,
			  char *buffer)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);

	return sysfs_emit(buffer, "%u\n", READ_ONCE(lp->ready));
}
static DEVICE_ATTR_RO(ready);

static ssize_t last_message_show(struct device *dev,
				 struct device_attribute *attr, char *buffer)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);

	return sysfs_emit(buffer, "0x%08x\n", READ_ONCE(lp->last_message));
}
static DEVICE_ATTR_RO(last_message);

static ssize_t mailbox_stats_show(struct device *dev,
				  struct device_attribute *attr, char *buffer)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);

	return sysfs_emit(buffer, "tx=%lld rx=%lld ack=%lld timeout=%lld\n",
			  atomic64_read(&lp->tx_count), atomic64_read(&lp->rx_count),
			  atomic64_read(&lp->ack_count),
			  atomic64_read(&lp->timeout_count));
}
static DEVICE_ATTR_RO(mailbox_stats);

static ssize_t tx_message_store(struct device *dev,
				struct device_attribute *attr,
				const char *buffer, size_t count)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	u32 message;
	int ret;

	ret = kstrtou32(buffer, 0, &message);
	if (ret)
		return ret;
	ret = esp32s31_lp_send_sync(lp, message);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(tx_message);

static ssize_t rpc_capabilities_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	struct s31_lp_rpc_slot request = { .operation = S31_LP_RPC_CAPABILITIES };
	int ret = esp32s31_lp_peripheral_rpc(lp, &request);

	if (ret)
		return ret;
	return sysfs_emit(buf, "version=%u services=0x%08x\n",
			  S31_LP_RPC_VERSION, request.args[0]);
}
static DEVICE_ATTR_RO(rpc_capabilities);

static ssize_t ping_show(struct device *dev, struct device_attribute *attr,
			 char *buffer)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);

	return sysfs_emit(buffer, "ready=%u rtt_us=%lld\n",
			  READ_ONCE(lp->ready), READ_ONCE(lp->ping_rtt_us));
}

static ssize_t ping_store(struct device *dev, struct device_attribute *attr,
			  const char *buffer, size_t count)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	u32 sequence = (u32)atomic_inc_return(&lp->ping_sequence) & 0xffff;
	u32 expected = S31_LP_RSP_PONG | sequence;
	long timeout;
	int ret;

	lp->ping_started = ktime_get();
	ret = esp32s31_lp_send_sync(lp, S31_LP_CMD_PING | sequence);
	if (ret)
		return ret;

	timeout = wait_event_interruptible_timeout(
		lp->rx_wait, READ_ONCE(lp->last_message) == expected,
		msecs_to_jiffies(S31_LP_TIMEOUT_MS));
	if (timeout <= 0)
		return timeout ? timeout : -ETIMEDOUT;

	return count;
}
static DEVICE_ATTR_RW(ping);

static ssize_t sleep_test_show(struct device *dev,
			       struct device_attribute *attr, char *buffer)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	struct s31_lp_sleep_control *control = &lp->sleep_test_control;
	u64 start = (u64)control->sleep_ticks_lo |
		    ((u64)control->sleep_ticks_hi << 32);
	u64 wake = (u64)control->wake_ticks_lo |
		   ((u64)control->wake_ticks_hi << 32);

	return sysfs_emit(buffer,
		"result=%d capabilities=%#x state=%u wake_reason=%#x raw=%#x start_ticks=%llu wake_ticks=%llu\n",
		lp->sleep_test_result, control->capabilities, control->state,
		control->wake_reason, control->wake_raw, start, wake);
}

static ssize_t sleep_test_store(struct device *dev,
				struct device_attribute *attr,
				const char *buffer, size_t count)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	struct s31_lp_sleep_control *control = &lp->sleep_test_control;
	unsigned int duration_ms;
	int ret;

	ret = kstrtouint(buffer, 0, &duration_ms);
	if (ret)
		return ret;
	if (duration_ms < 10 || duration_ms > 5000)
		return -ERANGE;

	memset(control, 0, sizeof(*control));
	control->flags = S31_LP_SLEEP_F_S2IDLE | S31_LP_SLEEP_F_DRY_RUN;
	control->wake_mask = S31_LP_WAKE_TIMER;
	control->deadline_lo = duration_ms * 1000U;
	ret = esp32s31_lp_sleep_prepare(control);
	if (!ret && !(control->capabilities & S31_LP_CAP_TIMER_WAKE))
		ret = -EOPNOTSUPP;
	if (!ret)
		ret = esp32s31_lp_sleep_arm(control);
	if (!ret) {
		msleep(duration_ms + 20);
		ret = esp32s31_lp_sleep_query(control);
	}
	if (!ret && (!(control->wake_reason & S31_LP_WAKE_TIMER) ||
		     control->state != S31_LP_SLEEP_WAKING))
		ret = -ETIMEDOUT;
	if (ret)
		esp32s31_lp_sleep_abort(control);
	else
		esp32s31_lp_sleep_reclaim(control);
	lp->sleep_test_result = ret;

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(sleep_test);

static ssize_t gpio_test_show(struct device *dev,
			      struct device_attribute *attr, char *buffer)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	struct s31_lp_sleep_control *control = &lp->gpio_test_control;
	u64 start = (u64)control->sleep_ticks_lo |
		    ((u64)control->sleep_ticks_hi << 32);
	u64 wake = (u64)control->wake_ticks_lo |
		   ((u64)control->wake_ticks_hi << 32);

	return sysfs_emit(buffer,
		"result=%d capabilities=%#x state=%u wake_reason=%#x raw=%#x start_ticks=%llu wake_ticks=%llu\n",
		lp->gpio_test_result, control->capabilities, control->state,
		control->wake_reason, control->wake_raw, start, wake);
}

static ssize_t gpio_test_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buffer, size_t count)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	struct s31_lp_sleep_control *control = &lp->gpio_test_control;
	ktime_t timeout;
	unsigned int pin, level, pull, timeout_ms;
	u32 state;
	int ret;

	if (sscanf(buffer, "%u %u %u %u", &pin, &level, &pull,
		   &timeout_ms) != 4)
		return -EINVAL;
	if (pin > 7 || level > 1 || pull > 2 ||
	    timeout_ms < 10 || timeout_ms > 5000)
		return -ERANGE;

	memset(control, 0, sizeof(*control));
	control->flags = S31_LP_SLEEP_F_S2IDLE | S31_LP_SLEEP_F_DRY_RUN;
	if (pull == 1)
		control->flags |= S31_LP_SLEEP_F_GPIO_PULL_UP;
	else if (pull == 2)
		control->flags |= S31_LP_SLEEP_F_GPIO_PULL_DOWN;
	control->wake_mask = S31_LP_WAKE_GPIO;
	control->gpio_mask_lo = BIT(pin);
	if (level)
		control->gpio_level_lo = BIT(pin);

	ret = esp32s31_lp_sleep_prepare(control);
	if (!ret && !(control->capabilities & S31_LP_CAP_GPIO_WAKE))
		ret = -EOPNOTSUPP;
	if (!ret)
		ret = esp32s31_lp_sleep_arm(control);
	if (!ret) {
		timeout = ktime_add_ms(ktime_get(), timeout_ms);
		do {
			state = readl(lp->sram + S31_LP_SLEEP_CONTROL_OFFSET +
				      offsetof(struct s31_lp_sleep_control, state));
			if (state == S31_LP_SLEEP_WAKING)
				break;
			usleep_range(500, 1000);
		} while (ktime_before(ktime_get(), timeout));
		if (state != S31_LP_SLEEP_WAKING)
			ret = -ETIMEDOUT;
		else
			ret = esp32s31_lp_sleep_query(control);
	}
	if (!ret && (!(control->wake_reason & S31_LP_WAKE_GPIO) ||
		     !(control->wake_raw & BIT(pin))))
		ret = -EPROTO;
	if (ret)
		esp32s31_lp_sleep_abort(control);
	else
		esp32s31_lp_sleep_reclaim(control);
	lp->gpio_test_result = ret;

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(gpio_test);

static ssize_t deep_sleep_show(struct device *dev,
			       struct device_attribute *attr, char *buffer)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	u32 marker = readl(lp->lp_sys + S31_LP_SYS_STORE2_REG);
	u32 reason = readl(lp->lp_sys + S31_LP_SYS_STORE3_REG);

	return sysfs_emit(buffer, "armed=%u previous=%u wake_reason=%#x\n",
			  lp->deep_armed, marker == S31_DEEP_SLEEP_MAGIC,
			  marker == S31_DEEP_SLEEP_MAGIC ? reason : 0);
}

static int esp32s31_lp_reboot_notify(struct notifier_block *notifier,
				     unsigned long action, void *data)
{
	struct esp32s31_lp *lp = container_of(notifier, struct esp32s31_lp,
					      reboot_notifier);
	int ret;

	if (action != SYS_POWER_OFF || !READ_ONCE(lp->deep_armed))
		return NOTIFY_DONE;

	/*
	 * SOC_CLK_UPDATE requires both HP harts to participate in the clock-tree
	 * handshake.  Do the terminal CPLL-to-XTAL transition from the reboot
	 * notifier, before migrate_to_reboot_cpu() stops the secondary hart.
	 * OpenSBI then only has to verify the already-latched 40 MHz state before
	 * the PMU powers the PLL domains down.
	 */
	ret = clk_set_rate(lp->cpu_clk, 40000000UL);
	if (!ret) {
		dev_info(lp->dev, "deep-sleep clock handoff to 40 MHz XTAL complete\n");
		return NOTIFY_OK;
	}

	/* Fail closed: revoke the always-on authorization so a failed handoff can
	 * never enter deep sleep while the CPU still depends on CPLL. */
	WRITE_ONCE(lp->deep_armed, false);
	writel(0, lp->lp_sys + S31_LP_SYS_STORE4_REG);
	writel(0, lp->lp_sys + S31_LP_SYS_STORE5_REG);
	esp32s31_lp_sleep_abort(&lp->deep_control);
	dev_err(lp->dev, "deep-sleep 40 MHz clock handoff failed: %d\n", ret);
	return NOTIFY_BAD;
}

static void esp32s31_lp_unregister_reboot_notifier(void *data)
{
	struct esp32s31_lp *lp = data;

	unregister_reboot_notifier(&lp->reboot_notifier);
}

static ssize_t deep_sleep_store(struct device *dev,
				struct device_attribute *attr,
				const char *buffer, size_t count)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	struct s31_lp_sleep_control *control = &lp->deep_control;
	unsigned int duration_ms;
	int ret;

	ret = kstrtouint(buffer, 0, &duration_ms);
	if (ret)
		return ret;
	if (duration_ms < 1000 || duration_ms > 600000)
		return -ERANGE;
	if (!READ_ONCE(lp->ready) || lp->mem_armed || lp->deep_armed)
		return -EBUSY;

	memset(control, 0, sizeof(*control));
	control->flags = S31_LP_SLEEP_F_DEEP_REBOOT;
	control->wake_mask = S31_LP_WAKE_TIMER;
	control->deadline_lo = duration_ms * 1000U;
	ret = esp32s31_lp_sleep_prepare(control);
	if (!ret && !(control->capabilities & S31_LP_CAP_TIMER_WAKE))
		ret = -EOPNOTSUPP;
	if (!ret)
		ret = esp32s31_lp_sleep_arm(control);
	if (ret) {
		esp32s31_lp_sleep_abort(control);
		return ret;
	}
	lp->deep_armed = true;
	/* The normal Linux poweroff path can reset/restart the LP remoteproc
	 * before the final SBI shutdown call.  Latch authorization in an
	 * always-on LP_SYS store register so OpenSBI does not depend on the
	 * volatile shared-SRAM state at that late boundary. */
	writel(S31_DEEP_SLEEP_MAGIC, lp->lp_sys + S31_LP_SYS_STORE4_REG);
	writel(duration_ms, lp->lp_sys + S31_LP_SYS_STORE5_REG);
	dev_info(lp->dev, "armed cold-resume deep sleep for %u ms\n",
		 duration_ms);
	/* Run the normal orderly shutdown path.  OpenSBI accepts SHUTDOWN only
	 * after seeing this LP-owned descriptor and never returns on success. */
	orderly_poweroff(true);
	return count;
}
static DEVICE_ATTR_RW(deep_sleep);

static struct attribute *esp32s31_lp_attrs[] = {
	&dev_attr_ready.attr,
	&dev_attr_last_message.attr,
	&dev_attr_mailbox_stats.attr,
	&dev_attr_tx_message.attr,
	&dev_attr_rpc_capabilities.attr,
	&dev_attr_ping.attr,
	&dev_attr_sleep_test.attr,
	&dev_attr_gpio_test.attr,
	&dev_attr_deep_sleep.attr,
	NULL,
};
static const struct attribute_group esp32s31_lp_group = {
	.attrs = esp32s31_lp_attrs,
};

static void esp32s31_lp_misc_deregister(void *data)
{
	struct esp32s31_lp *lp = data;

	misc_deregister(&lp->miscdev);
}

static void esp32s31_lp_fifo_free(void *data)
{
	struct esp32s31_lp *lp = data;

	kfifo_free(&lp->rx_fifo);
}

static void esp32s31_lp_clear_sleep_owner(void *data)
{
	struct esp32s31_lp *lp = data;

	if (READ_ONCE(esp32s31_lp_sleep_owner) == lp)
		s2idle_set_ops(NULL);
	if (READ_ONCE(esp32s31_lp_sleep_owner) == lp)
		WRITE_ONCE(esp32s31_lp_sleep_owner, NULL);
}

static void esp32s31_lp_clear_wake_irq(void *data)
{
	struct device *dev = data;

	dev_pm_clear_wake_irq(dev);
	device_init_wakeup(dev, false);
}

static int esp32s31_lp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_lp *lp;
	struct rproc *rproc;
	const char *firmware = "esp32s31/s31-lp-core.elf";
	int ret;

	of_property_read_string(dev->of_node, "firmware-name", &firmware);
	rproc = devm_rproc_alloc(dev, "esp32s31-lp", &esp32s31_lp_rproc_ops,
				 firmware, sizeof(*lp));
	if (!rproc)
		return -ENOMEM;

	lp = rproc->priv;
	lp->dev = dev;
	lp->reboot_notifier.notifier_call = esp32s31_lp_reboot_notify;
	platform_set_drvdata(pdev, lp);

	lp->cpu_clk = devm_clk_get(dev, "cpu");
	if (IS_ERR(lp->cpu_clk))
		return dev_err_probe(dev, PTR_ERR(lp->cpu_clk),
				     "failed to get CPU clock\n");

	lp->sram = devm_platform_ioremap_resource_byname(pdev, "sram");
	if (IS_ERR(lp->sram))
		return PTR_ERR(lp->sram);
	lp->mailbox = devm_platform_ioremap_resource_byname(pdev, "mailbox");
	if (IS_ERR(lp->mailbox))
		return PTR_ERR(lp->mailbox);

	lp->lp_sys = devm_platform_ioremap_resource_byname(pdev, "lp-sys");
	if (IS_ERR(lp->lp_sys))
		return PTR_ERR(lp->lp_sys);
	/* A request latch authorizes only the immediately following shutdown.
	 * Clear any stale value left by a reset before userspace can arm a new
	 * deep-sleep transaction.  STORE2/3 are deliberately retained as the
	 * previous-wake record. */
	writel(0, lp->lp_sys + S31_LP_SYS_STORE4_REG);
	writel(0, lp->lp_sys + S31_LP_SYS_STORE5_REG);

	spin_lock_init(&lp->tx_state_lock);
	spin_lock_init(&lp->rx_lock);
	mutex_init(&lp->user_tx_lock);
	mutex_init(&lp->rpc_lock);
	init_completion(&lp->tx_done);
	init_completion(&lp->rpc_done);
	init_waitqueue_head(&lp->rx_wait);
	lp->tx_next = 8;

	ret = kfifo_alloc(&lp->rx_fifo, S31_LP_RX_DEPTH, GFP_KERNEL);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, esp32s31_lp_fifo_free, lp);
	if (ret)
		return ret;

	lp->irq = platform_get_irq(pdev, 0);
	if (lp->irq < 0)
		return lp->irq;
	/* The LP mailbox is IPC, not background peripheral work. Keep its
	 * CPU0 owner when ordinary device interrupts default to CPU1. */
	ret = irq_set_affinity(lp->irq, cpumask_of(0));
	if (ret)
		return dev_err_probe(dev, ret, "cannot place LP IPC IRQ on CPU0\n");
	if (!cpumask_equal(irq_get_effective_affinity_mask(lp->irq), cpumask_of(0)))
		return dev_err_probe(dev, -EAGAIN, "LP IPC IRQ is not on CPU0\n");
	ret = devm_request_irq(dev, lp->irq, esp32s31_lp_irq, IRQF_NOBALANCING,
			       dev_name(dev), lp);
	if (ret)
		return ret;
	ret = device_init_wakeup(dev, true);
	if (ret)
		return ret;
	ret = dev_pm_set_wake_irq(dev, lp->irq);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, esp32s31_lp_clear_wake_irq, dev);
	if (ret)
		return ret;

	lp->mbox.dev = dev;
	lp->mbox.ops = &esp32s31_lp_mbox_ops;
	lp->mbox.chans = &lp->chan;
	lp->mbox.num_chans = 1;
	lp->mbox.txdone_irq = true;
	ret = devm_mbox_controller_register(dev, &lp->mbox);
	if (ret)
		return ret;

	lp->miscdev.minor = MISC_DYNAMIC_MINOR;
	lp->miscdev.name = "s31-lp";
	lp->miscdev.fops = &esp32s31_lp_fops;
	lp->miscdev.parent = dev;
	ret = misc_register(&lp->miscdev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, esp32s31_lp_misc_deregister, lp);
	if (ret)
		return ret;

	ret = devm_device_add_group(dev, &esp32s31_lp_group);
	if (ret)
		return ret;

#if IS_REACHABLE(CONFIG_I2C)
	ret = devm_add_action_or_reset(dev, esp32s31_lp_del_i2c, lp);
	if (ret)
		return ret;
#endif

#if IS_REACHABLE(CONFIG_SPI)
	mutex_init(&lp->spi_lock);
	ret = devm_add_action_or_reset(dev, esp32s31_lp_del_spi, lp);
	if (ret)
		return ret;
#endif

#if IS_ENABLED(CONFIG_TTY)
	mutex_init(&lp->uart_lock);
#endif

	rproc->state = RPROC_DETACHED;
	rproc->auto_boot = true;
	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return ret;

#if IS_ENABLED(CONFIG_TTY)
	ret = devm_add_action_or_reset(dev, esp32s31_lp_del_uart, lp);
	if (ret)
		return ret;
#endif

	if (cmpxchg(&esp32s31_lp_sleep_owner, NULL, lp))
		return dev_err_probe(dev, -EBUSY,
				     "another LP sleep controller is active\n");
	ret = devm_add_action_or_reset(dev, esp32s31_lp_clear_sleep_owner, lp);
	if (ret)
		return ret;
	ret = register_reboot_notifier(&lp->reboot_notifier);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev,
				       esp32s31_lp_unregister_reboot_notifier, lp);
	if (ret)
		return ret;
	if (IS_ENABLED(CONFIG_SUSPEND))
		s2idle_set_ops(&esp32s31_lp_s2idle_ops);

	dev_info(dev, "LP-core remoteproc and mailbox registered\n");
	return 0;
}

static const struct of_device_id esp32s31_lp_of_match[] = {
	{ .compatible = "espressif,esp32s31-lp-core" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_lp_of_match);

static struct platform_driver esp32s31_lp_driver = {
	.probe = esp32s31_lp_probe,
	.driver = {
		.name = "esp32s31-lp",
		.of_match_table = esp32s31_lp_of_match,
	},
};
module_platform_driver(esp32s31_lp_driver);

MODULE_DESCRIPTION("ESP32-S31 LP-core remoteproc and mailbox driver");
MODULE_LICENSE("GPL");
