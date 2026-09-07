// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 LP-core remoteproc and hardware mailbox driver. */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/crc32.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/ktime.h>
#include <linux/mailbox_controller.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reboot.h>
#include <linux/suspend.h>
#include <linux/iopoll.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <linux/soc/espressif/esp32s31-lp.h>

#include "remoteproc_internal.h"

#define S31_LP_SRAM_BASE                0x2e000000ULL
#define S31_LP_SRAM_SIZE                0x00008000ULL
#define S31_LP_RESET_VECTOR             (S31_LP_SRAM_BASE + 0x80)

#define S31_LP_SYS_BOOT_ADDR_REG        0x18
#define S31_LP_SYS_STORE0_REG           0x2c
#define S31_LP_SYS_STORE1_REG           0x30
#define S31_LP_SYS_STORE2_REG           0x34
#define S31_LP_SYS_STORE3_REG           0x38
#define S31_LP_SYS_STORE4_REG           0x3c
#define S31_LP_SYS_STORE5_REG           0x40
#define S31_DEEP_SLEEP_MAGIC            0x53314453U
#define S31_LP_SYS_DEBUG_PC_REG         0x190
#define S31_LP_SYS_EXCEPTION_PC_REG     0x194

#define S31_LP_CLKRST_CPU_CTRL_REG      0x04
#define S31_LP_CLKRST_INTR_CTRL_REG     0x28
#define S31_LP_CLKRST_MAILBOX_REG       0x38
#define S31_LP_CLKRST_CPU_REG           0x44
#define S31_LP_CPU_CLK_EN               BIT(30)
#define S31_LP_CPU_RESET                BIT(31)
#define S31_LP_INTR_CLK_EN              BIT(30)
#define S31_LP_INTR_RESET               BIT(31)
#define S31_LP_MAILBOX_CLK_EN           BIT(30)
#define S31_LP_DBGM_UNAVAILABLE         BIT(31)

#define S31_PMU_LP_CPU_PWR0_REG         0x1ac
#define S31_PMU_LP_CPU_PWR1_REG         0x1b0
#define S31_PMU_LP_CPU_PWR2_REG         0x1b4
#define S31_PMU_HP_LP_COMM_REG          0x1b8
#define S31_PMU_LP_CPU_STALL            BIT(29)
#define S31_PMU_LP_CPU_SLEEP_RESET      BIT(30)
#define S31_PMU_LP_CPU_SLEEP_REQ        BIT(31)
#define S31_PMU_LP_WAKE_HP_CPU          BIT(1)
#define S31_PMU_HP_TRIGGER_LP           BIT(31)

#define S31_MB_MSG(n)                   ((n) * 4)
#define S31_MB_HP_RAW                   0x50
#define S31_MB_HP_ST                    0x54
#define S31_MB_HP_ENA                   0x58
#define S31_MB_HP_CLR                   0x5c
#define S31_MB_CLK_REG                  0x60
#define S31_MB_CLK_EN                   BIT(0)
#define S31_MB_ACK                      0xe5U
#define S31_MB_RX_MASK                  (BIT(0) | BIT(2) | BIT(4) | BIT(6))
#define S31_MB_TX_ACK_MASK              (BIT(9) | BIT(11) | BIT(13) | BIT(15))
#define S31_MB_IRQ_MASK                 (S31_MB_RX_MASK | S31_MB_TX_ACK_MASK)

#define S31_LP_PERI_PMS_BASE            0x20706000
#define S31_LP_PERI_PMS_SIZE            0x100
#define S31_LP_PERI_PMS_SYSREG_CTRL     0x00
#define S31_LP_PERI_PMS_TIMER_CTRL      0x24
#define S31_LP_PERI_PMS_PERICLKRST_CTRL 0x38
#define S31_LP_PERI_PMS_IOMUX_CTRL      0x3c
#define S31_LP_PERI_PMS_MAILBOX_CTRL    0x64
#define S31_LP_PERI_PMS_READ_REE_MASK   GENMASK(3, 1)
#define S31_LP_PERI_PMS_WRITE_REE_MASK  GENMASK(7, 5)
#define S31_LP_PERI_PMS_LOCK            BIT(8)
#define S31_LP_PERI_PMS_REE_ACCESS      (S31_LP_PERI_PMS_READ_REE_MASK | \
					 S31_LP_PERI_PMS_WRITE_REE_MASK)

#define S31_LP_RX_DEPTH                 64
#define S31_LP_TIMEOUT_MS               1000

#define S31_RTC_TIMER_UPDATE_REG        0x10
#define S31_RTC_TIMER_BUF0_LOW_REG      0x14
#define S31_RTC_TIMER_BUF0_HIGH_REG     0x18
#define S31_RTC_TIMER_DATE_REG          0x3fc
#define S31_RTC_TIMER_MAIN_UPDATE       BIT(27)
#define S31_RTC_TIMER_MAIN_STALL        BIT(30)
#define S31_RTC_TIMER_MAIN_RESET        BIT(31)
#define S31_RTC_TIMER_CLK_EN            BIT(31)
#define S31_RTC_CAL_FRACT               19
#define S31_RTC_CAL_SAMPLE_US           100000

struct esp32s31_lp {
	struct device *dev;
	struct rproc *rproc;
	void __iomem *sram;
	void __iomem *mailbox;
	void __iomem *lp_pms;
	void __iomem *lp_sys;
	void __iomem *lp_clkrst;
	void __iomem *rtc_timer;
	struct regmap *pmu;
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

static u64 esp32s31_lp_rtc_count(struct esp32s31_lp *lp)
{
	u32 high;

	writel(S31_RTC_TIMER_MAIN_UPDATE,
	       lp->rtc_timer + S31_RTC_TIMER_UPDATE_REG);
	high = readl(lp->rtc_timer + S31_RTC_TIMER_BUF0_HIGH_REG) & 0xffff;
	return ((u64)high << 32) |
		readl(lp->rtc_timer + S31_RTC_TIMER_BUF0_LOW_REG);
}

static int esp32s31_lp_calibrate_rtc_slow(struct esp32s31_lp *lp)
{
	u64 started_ns, elapsed_us, first, delta, cal, hz;
	u32 update;

	writel(readl(lp->rtc_timer + S31_RTC_TIMER_DATE_REG) |
	       S31_RTC_TIMER_CLK_EN,
	       lp->rtc_timer + S31_RTC_TIMER_DATE_REG);
	update = readl(lp->rtc_timer + S31_RTC_TIMER_UPDATE_REG);
	writel(update | S31_RTC_TIMER_MAIN_RESET,
	       lp->rtc_timer + S31_RTC_TIMER_UPDATE_REG);
	writel(update & ~(S31_RTC_TIMER_MAIN_RESET | S31_RTC_TIMER_MAIN_STALL),
	       lp->rtc_timer + S31_RTC_TIMER_UPDATE_REG);

	first = esp32s31_lp_rtc_count(lp);
	started_ns = ktime_get_raw_ns();
	usleep_range(S31_RTC_CAL_SAMPLE_US, S31_RTC_CAL_SAMPLE_US + 1000);
	delta = esp32s31_lp_rtc_count(lp) - first;
	elapsed_us = div_u64(ktime_get_raw_ns() - started_ns, NSEC_PER_USEC);
	if (!delta || !elapsed_us)
		return -EIO;

	cal = div64_u64(elapsed_us << S31_RTC_CAL_FRACT, delta);
	hz = div64_u64(delta * USEC_PER_SEC, elapsed_us);
	/* Reject a stopped counter and an accidentally selected fast clock. */
	if (hz < 20000 || hz > 300000 || !cal || cal > U32_MAX)
		return dev_err_probe(lp->dev, -ERANGE,
				     "invalid RTC slow clock calibration: %llu Hz, cal=%llu\n",
				     hz, cal);

	writel((u32)cal, lp->lp_sys + S31_LP_SYS_STORE1_REG);
	dev_info(lp->dev,
		 "RTC slow clock calibrated at %llu Hz (Q13.19 period %llu)\n",
		 hz, cal);
	return 0;
}

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

		WRITE_ONCE(lp->last_message, message);
		if (message == S31_LP_MSG_READY)
			WRITE_ONCE(lp->ready, true);
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

static int esp32s31_lp_grant_peripheral(struct esp32s31_lp *lp, u32 offset,
					const char *name, u32 *value)
{
	u32 pms = readl(lp->lp_pms + offset);

	if ((pms & S31_LP_PERI_PMS_REE_ACCESS) == S31_LP_PERI_PMS_REE_ACCESS)
		goto out;
	if (pms & S31_LP_PERI_PMS_LOCK) {
		dev_err(lp->dev, "LP %s PMS locked without REE access (0x%08x)\n",
			name, pms);
		return -EACCES;
	}

	writel(pms | S31_LP_PERI_PMS_REE_ACCESS, lp->lp_pms + offset);
	pms = readl(lp->lp_pms + offset);
	if ((pms & S31_LP_PERI_PMS_REE_ACCESS) != S31_LP_PERI_PMS_REE_ACCESS) {
		dev_err(lp->dev, "failed to enable LP %s REE access (0x%08x)\n",
			name, pms);
		return -EACCES;
	}

out:
	if (value)
		*value = pms;
	return 0;
}

static int esp32s31_lp_start(struct rproc *rproc)
{
	struct esp32s31_lp *lp = rproc->priv;
	u64 bootaddr = rproc->bootaddr;
	u32 pmu_pwr0;
	u32 pmu_pwr1;
	u32 pmu_pwr2;
	u32 pmu_comm;
	u32 status;
	u32 value;
	u32 pms;
	int ret;

	if (bootaddr < S31_LP_SRAM_BASE || bootaddr >= S31_LP_SRAM_BASE + S31_LP_SRAM_SIZE)
		return -EINVAL;

	ret = esp32s31_lp_grant_peripheral(lp, S31_LP_PERI_PMS_SYSREG_CTRL,
					  "system registers", NULL);
	if (ret)
		return ret;
	ret = esp32s31_lp_grant_peripheral(lp, S31_LP_PERI_PMS_TIMER_CTRL,
					  "RTC timer", NULL);
	if (ret)
		return ret;
	ret = esp32s31_lp_calibrate_rtc_slow(lp);
	if (ret)
		return ret;
	ret = esp32s31_lp_grant_peripheral(lp, S31_LP_PERI_PMS_PERICLKRST_CTRL,
					  "peripheral clock/reset", NULL);
	if (ret)
		return ret;
	ret = esp32s31_lp_grant_peripheral(lp, S31_LP_PERI_PMS_IOMUX_CTRL,
					  "IOMUX", NULL);
	if (ret)
		return ret;
	ret = esp32s31_lp_grant_peripheral(lp, S31_LP_PERI_PMS_MAILBOX_CTRL,
					  "mailbox", &pms);
	if (ret)
		return ret;

	writel((u32)bootaddr, lp->lp_sys + S31_LP_SYS_BOOT_ADDR_REG);
	writel(0, lp->lp_sys + S31_LP_SYS_STORE0_REG);

	/* Mirror ESP-IDF lp_core_ll_reset_register() and bus-clock setup. */
	value = readl(lp->lp_clkrst + S31_LP_CLKRST_CPU_CTRL_REG);
	writel(value | S31_LP_CPU_RESET,
	       lp->lp_clkrst + S31_LP_CLKRST_CPU_CTRL_REG);
	value = readl(lp->lp_clkrst + S31_LP_CLKRST_INTR_CTRL_REG);
	writel(value | S31_LP_INTR_RESET,
	       lp->lp_clkrst + S31_LP_CLKRST_INTR_CTRL_REG);
	writel(readl(lp->lp_clkrst + S31_LP_CLKRST_CPU_CTRL_REG) &
	       ~S31_LP_CPU_RESET, lp->lp_clkrst + S31_LP_CLKRST_CPU_CTRL_REG);
	writel(readl(lp->lp_clkrst + S31_LP_CLKRST_INTR_CTRL_REG) &
	       ~S31_LP_INTR_RESET, lp->lp_clkrst + S31_LP_CLKRST_INTR_CTRL_REG);
	writel(readl(lp->lp_clkrst + S31_LP_CLKRST_CPU_CTRL_REG) |
	       S31_LP_CPU_CLK_EN, lp->lp_clkrst + S31_LP_CLKRST_CPU_CTRL_REG);
	writel(readl(lp->lp_clkrst + S31_LP_CLKRST_INTR_CTRL_REG) |
	       S31_LP_INTR_CLK_EN, lp->lp_clkrst + S31_LP_CLKRST_INTR_CTRL_REG);
	writel(readl(lp->lp_clkrst + S31_LP_CLKRST_CPU_REG) &
	       ~S31_LP_DBGM_UNAVAILABLE, lp->lp_clkrst + S31_LP_CLKRST_CPU_REG);

	regmap_update_bits(lp->pmu, S31_PMU_LP_CPU_PWR0_REG,
			   S31_PMU_LP_CPU_STALL | S31_PMU_LP_CPU_SLEEP_RESET,
			   S31_PMU_LP_CPU_STALL | S31_PMU_LP_CPU_SLEEP_RESET);
	regmap_write(lp->pmu, S31_PMU_LP_CPU_PWR2_REG, S31_PMU_LP_WAKE_HP_CPU);
	regmap_write(lp->pmu, S31_PMU_HP_LP_COMM_REG, S31_PMU_HP_TRIGGER_LP);

	/*
	 * The native LP mailbox runtime resets the whole mailbox before sending
	 * READY.  That reset also clears HP interrupt enables, so restore them
	 * while waiting for the first LP-to-HP message.  Poll RAW rather than ST
	 * to close the reset/enable race, then run the normal IRQ handler to ACK
	 * READY and release the LP firmware from its synchronous send.
	 */
	ret = readl_poll_timeout(lp->mailbox + S31_MB_HP_RAW, status,
		({
			writel(S31_MB_CLK_EN, lp->mailbox + S31_MB_CLK_REG);
			writel(S31_MB_IRQ_MASK, lp->mailbox + S31_MB_HP_ENA);
			status = readl(lp->mailbox + S31_MB_HP_RAW);
			READ_ONCE(lp->ready) || (status & S31_MB_RX_MASK);
		}), 10, 500000);
	if (ret) {
		dev_err(lp->dev,
			"LP-core did not signal READY: msg0=%08x msg1=%08x lp=%08x/%08x/%08x hp=%08x/%08x/%08x clk=%08x pms=%08x\n",
			readl(lp->mailbox + S31_MB_MSG(0)),
			readl(lp->mailbox + S31_MB_MSG(1)),
			readl(lp->mailbox + 0x40),
			readl(lp->mailbox + 0x44),
			readl(lp->mailbox + 0x48),
			status,
			readl(lp->mailbox + S31_MB_HP_ST),
			readl(lp->mailbox + S31_MB_HP_ENA),
			readl(lp->mailbox + S31_MB_CLK_REG), pms);
		regmap_read(lp->pmu, S31_PMU_LP_CPU_PWR0_REG, &pmu_pwr0);
		regmap_read(lp->pmu, S31_PMU_LP_CPU_PWR1_REG, &pmu_pwr1);
		regmap_read(lp->pmu, S31_PMU_LP_CPU_PWR2_REG, &pmu_pwr2);
		regmap_read(lp->pmu, S31_PMU_HP_LP_COMM_REG, &pmu_comm);
		dev_err(lp->dev,
			"state boot=%08x stage=%08x pc=%08x exc=%08x cpu=%08x intr=%08x mbclk=%08x dbg=%08x pmu=%08x/%08x/%08x/%08x\n",
			readl(lp->lp_sys + S31_LP_SYS_BOOT_ADDR_REG),
			readl(lp->lp_sys + S31_LP_SYS_STORE0_REG),
			readl(lp->lp_sys + S31_LP_SYS_DEBUG_PC_REG),
			readl(lp->lp_sys + S31_LP_SYS_EXCEPTION_PC_REG),
			readl(lp->lp_clkrst + S31_LP_CLKRST_CPU_CTRL_REG),
			readl(lp->lp_clkrst + S31_LP_CLKRST_INTR_CTRL_REG),
			readl(lp->lp_clkrst + S31_LP_CLKRST_MAILBOX_REG),
			readl(lp->lp_clkrst + S31_LP_CLKRST_CPU_REG),
			pmu_pwr0, pmu_pwr1, pmu_pwr2, pmu_comm);
		return ret;
	}

	if (!READ_ONCE(lp->ready))
		esp32s31_lp_irq(lp->irq, lp);
	if (!READ_ONCE(lp->ready)) {
		dev_err(lp->dev, "LP-core first message was not READY (0x%08x)\n",
			READ_ONCE(lp->last_message));
		return -EPROTO;
	}

	return 0;
}

static int esp32s31_lp_stop(struct rproc *rproc)
{
	struct esp32s31_lp *lp = rproc->priv;

	regmap_write(lp->pmu, S31_PMU_LP_CPU_PWR1_REG,
		     S31_PMU_LP_CPU_SLEEP_REQ);
	return 0;
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
	sequence = (u32)atomic_inc_return(&lp->rpc_sequence) &
		   S31_LP_SEQUENCE_MASK;
	if (!sequence)
		sequence = (u32)atomic_inc_return(&lp->rpc_sequence) &
			   S31_LP_SEQUENCE_MASK;

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

static void *esp32s31_lp_da_to_va(struct rproc *rproc, u64 da, size_t len,
				  bool *is_iomem)
{
	struct esp32s31_lp *lp = rproc->priv;
	u64 offset;

	if (da < S31_LP_SRAM_BASE)
		return NULL;
	offset = da - S31_LP_SRAM_BASE;
	if (offset > S31_LP_SRAM_SIZE || len > S31_LP_SRAM_SIZE - offset)
		return NULL;

	if (is_iomem)
		*is_iomem = true;
	return (__force void *)(lp->sram + offset);
}

static int esp32s31_lp_parse_fw(struct rproc *rproc,
				const struct firmware *firmware)
{
	return 0;
}

static const struct rproc_ops esp32s31_lp_rproc_ops = {
	.start = esp32s31_lp_start,
	.stop = esp32s31_lp_stop,
	.da_to_va = esp32s31_lp_da_to_va,
	.load = rproc_elf_load_segments,
	.parse_fw = esp32s31_lp_parse_fw,
	.sanity_check = rproc_elf_sanity_check,
	.get_boot_addr = rproc_elf_get_boot_addr,
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
	struct resource *resource;
	const char *firmware = "esp32s31/s31-lp-core.elf";
	int ret;

	of_property_read_string(dev->of_node, "firmware-name", &firmware);
	rproc = devm_rproc_alloc(dev, "esp32s31-lp", &esp32s31_lp_rproc_ops,
				 firmware, sizeof(*lp));
	if (!rproc)
		return -ENOMEM;

	lp = rproc->priv;
	lp->dev = dev;
	lp->rproc = rproc;
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

	lp->lp_pms = devm_ioremap(&pdev->dev, S31_LP_PERI_PMS_BASE,
				  S31_LP_PERI_PMS_SIZE);
	if (!lp->lp_pms)
		return -ENOMEM;
	lp->lp_sys = devm_platform_ioremap_resource_byname(pdev, "lp-sys");
	if (IS_ERR(lp->lp_sys))
		return PTR_ERR(lp->lp_sys);
	/* A request latch authorizes only the immediately following shutdown.
	 * Clear any stale value left by a reset before userspace can arm a new
	 * deep-sleep transaction.  STORE2/3 are deliberately retained as the
	 * previous-wake record. */
	writel(0, lp->lp_sys + S31_LP_SYS_STORE4_REG);
	writel(0, lp->lp_sys + S31_LP_SYS_STORE5_REG);
	lp->rtc_timer = devm_platform_ioremap_resource_byname(pdev, "rtc-timer");
	if (IS_ERR(lp->rtc_timer))
		return PTR_ERR(lp->rtc_timer);
	resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lp-clkrst");
	if (!resource)
		return -EINVAL;
	/* LP clock/reset registers share the SoC clock-controller aperture. */
	lp->lp_clkrst = devm_ioremap(dev, resource->start,
				    resource_size(resource));
	if (!lp->lp_clkrst)
		return -ENOMEM;

	lp->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "espressif,pmu");
	if (IS_ERR(lp->pmu))
		return dev_err_probe(dev, PTR_ERR(lp->pmu), "failed to get PMU syscon\n");

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

	writel(readl(lp->mailbox + S31_MB_CLK_REG) | S31_MB_CLK_EN,
	       lp->mailbox + S31_MB_CLK_REG);
	writel(readl(lp->lp_clkrst + S31_LP_CLKRST_MAILBOX_REG) |
	       S31_LP_MAILBOX_CLK_EN,
	       lp->lp_clkrst + S31_LP_CLKRST_MAILBOX_REG);
	writel(~0U, lp->mailbox + S31_MB_HP_CLR);
	writel(S31_MB_IRQ_MASK, lp->mailbox + S31_MB_HP_ENA);

	lp->irq = platform_get_irq(pdev, 0);
	if (lp->irq < 0)
		return lp->irq;
	ret = devm_request_irq(dev, lp->irq, esp32s31_lp_irq, 0,
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

	/* The firmware lives in the SquashFS root, which is mounted after probe. */
	rproc->auto_boot = false;
	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return ret;

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
