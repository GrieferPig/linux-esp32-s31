// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 LP-core remoteproc and hardware mailbox driver. */

#include <linux/bitops.h>
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
#include <linux/irq.h>
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
#include <linux/rtc.h>
#include <linux/suspend.h>
#include <linux/iopoll.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <linux/soc/espressif/esp32s31-lp.h>
#include <linux/soc/espressif/esp32s31-gpio.h>

#include "remoteproc_internal.h"

#define S31_LP_SRAM_BASE                0x2e000000ULL
#define S31_LP_SRAM_SIZE                0x00008000ULL

#define S31_LP_SYS_BOOT_ADDR_REG        0x18
#define S31_LP_SYS_STORE1_REG           0x30
#define S31_LP_SYS_DEBUG_PC_REG         0x190
#define S31_LP_SYS_EXCEPTION_PC_REG     0x194

#define S31_LP_CLKRST_CPU_CTRL_REG      0x04
#define S31_LP_CLKRST_INTR_CTRL_REG     0x28
#define S31_LP_CLKRST_MAILBOX_REG       0x38
#define S31_LP_CLKRST_CPU_REG           0x44
#define S31_LP_AON_HP0_RESET_CAUSE      0x20701030
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

#define S31_RTC_TIMER_TARGET0_LOW_REG    0x00
#define S31_RTC_TIMER_TARGET0_HIGH_REG   0x04
#define S31_RTC_TIMER_INT_RAW_REG        0x28
#define S31_RTC_TIMER_INT_ENA_REG        0x30
#define S31_RTC_TIMER_INT_CLR_REG        0x34
#define S31_RTC_TIMER_TARGET_EN          BIT(31)
#define S31_RTC_TIMER_SOC_WAKE           BIT(31)
#define S31_RTC_TIMER_MASK               GENMASK_ULL(47, 0)
#define S31_RTC_ALARM_MAX_SECONDS        (366U * 24 * 60 * 60)

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
	void __iomem *reset_cause;
	void __iomem *rtc_timer;
	struct regmap *pmu;
	int irq;
	struct rtc_device *rtc;
	int rtc_irq;
	bool rtc_irq_wake;
	spinlock_t rtc_lock;
	u32 rtc_cal;
	u64 rtc_base_ticks;
	time64_t rtc_base_time;
	time64_t rtc_alarm_time;
	u64 rtc_alarm_ticks;
	bool rtc_valid;
	bool rtc_alarm_enabled;
	bool rtc_alarm_pending;
	bool rtc_alarm_handoff;

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
	struct s31_lp_sleep_control mem_control;
	bool mem_armed;
};

static u64 esp32s31_lp_rtc_count(struct esp32s31_lp *lp)
{
	u32 high, low, check;

	writel(S31_RTC_TIMER_MAIN_UPDATE,
	       lp->rtc_timer + S31_RTC_TIMER_UPDATE_REG);
	/* LP can take another snapshot concurrently. Retry across a low wrap. */
	do {
		high = readl(lp->rtc_timer + S31_RTC_TIMER_BUF0_HIGH_REG) & 0xffff;
		low = readl(lp->rtc_timer + S31_RTC_TIMER_BUF0_LOW_REG);
		check = readl(lp->rtc_timer + S31_RTC_TIMER_BUF0_HIGH_REG) & 0xffff;
	} while (high != check);
	return ((u64)high << 32) | low;
}

static int esp32s31_lp_calibrate_rtc_slow(struct esp32s31_lp *lp)
{
	u64 started_ns, elapsed_us, first, delta, cal, hz;
	u32 update;

	writel(readl(lp->rtc_timer + S31_RTC_TIMER_DATE_REG) |
	       S31_RTC_TIMER_CLK_EN,
	       lp->rtc_timer + S31_RTC_TIMER_DATE_REG);
	update = readl(lp->rtc_timer + S31_RTC_TIMER_UPDATE_REG);
	/* The RTC is shared with Linux timekeeping: never reset on LP restart. */
	writel(update & ~(S31_RTC_TIMER_MAIN_RESET | S31_RTC_TIMER_MAIN_STALL),
	       lp->rtc_timer + S31_RTC_TIMER_UPDATE_REG);

	first = esp32s31_lp_rtc_count(lp);
	started_ns = ktime_get_raw_ns();
	usleep_range(S31_RTC_CAL_SAMPLE_US, S31_RTC_CAL_SAMPLE_US + 1000);
	delta = (esp32s31_lp_rtc_count(lp) - first) & S31_RTC_TIMER_MASK;
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

	lp->rtc_cal = cal;
	writel((u32)cal, lp->lp_sys + S31_LP_SYS_STORE1_REG);
	dev_info(lp->dev,
		 "RTC slow clock calibrated at %llu Hz (Q13.19 period %llu)\n",
		 hz, cal);
	return 0;
}

#include "esp32s31_lp_rtc.inc"

static struct esp32s31_lp *esp32s31_lp_sleep_owner;

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

	if (bootaddr < S31_LP_SRAM_BASE ||
	    bootaddr >= S31_LP_SRAM_BASE + S31_LP_FIRMWARE_SIZE)
		return -EINVAL;

	ret = esp32s31_lp_grant_peripheral(lp, S31_LP_PERI_PMS_SYSREG_CTRL,
					  "system registers", NULL);
	if (ret)
		return ret;
	ret = esp32s31_lp_grant_peripheral(lp, S31_LP_PERI_PMS_TIMER_CTRL,
					  "RTC timer", NULL);
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
			"state boot=%08x pc=%08x exc=%08x cpu=%08x intr=%08x mbclk=%08x dbg=%08x pmu=%08x/%08x/%08x/%08x\n",
			readl(lp->lp_sys + S31_LP_SYS_BOOT_ADDR_REG),
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

	WRITE_ONCE(lp->ready, false);
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

static int esp32s31_lp_sleep_prepare(struct s31_lp_sleep_control *control)
{
	return esp32s31_lp_sleep_rpc(S31_LP_CMD_SLEEP_PREPARE,
		S31_LP_RSP_SLEEP_PREPARED, control, true);
}

static int esp32s31_lp_sleep_arm(struct s31_lp_sleep_control *control)
{
	return esp32s31_lp_sleep_rpc(S31_LP_CMD_SLEEP_ARM,
		S31_LP_RSP_SLEEP_ARMED, control, true);
}

static int esp32s31_lp_sleep_abort(struct s31_lp_sleep_control *control)
{
	return esp32s31_lp_sleep_rpc(S31_LP_CMD_SLEEP_ABORT,
		S31_LP_RSP_SLEEP_ABORTED, control, false);
}

static int esp32s31_lp_sleep_query(struct s31_lp_sleep_control *control)
{
	return esp32s31_lp_sleep_rpc(S31_LP_CMD_SLEEP_QUERY,
		S31_LP_RSP_SLEEP_STATUS, control, false);
}

static int esp32s31_lp_sleep_reclaim(struct s31_lp_sleep_control *control)
{
	return esp32s31_lp_sleep_rpc(S31_LP_CMD_SLEEP_RECLAIM,
		S31_LP_RSP_SLEEP_RECLAIMED, control, false);
}

int esp32s31_lp_system_suspend_prepare(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);
	struct esp32s31_gpio_wake gpio;
	int ret;

	if (!lp || !READ_ONCE(lp->ready))
		return -ENODEV;
	/* Normal device suspend has enabled native IRQ wake. Snapshot before
	 * noirq/CPU-off phases while both GPIO mappings and LP RPC IRQs are live.
	 */
	ret = esp32s31_gpio_get_lp_wake(&gpio);
	if (ret)
		return ret;
	memset(&lp->mem_control, 0, sizeof(lp->mem_control));
	lp->mem_control.flags = S31_LP_SLEEP_F_MEM;
	ret = esp32s31_lp_rtc_handoff(lp, &lp->mem_control);
	if (ret)
		return ret;
	if (lp->mem_control.flags & S31_LP_SLEEP_F_RTC_ABSOLUTE)
		lp->mem_control.wake_mask |= S31_LP_WAKE_TIMER;
	if (gpio.mask) {
		lp->mem_control.wake_mask |= S31_LP_WAKE_GPIO;
		lp->mem_control.gpio_mask_lo = gpio.mask;
		lp->mem_control.gpio_types = gpio.types;
		lp->mem_control.gpio_pullup_mask = gpio.pullup;
		lp->mem_control.gpio_pulldown_mask = gpio.pulldown;
	}
	if (!lp->mem_control.wake_mask) {
		ret = -ENXIO;
		goto restore_alarm;
	}
	/* Conservative stage one: retain every domain until measured otherwise. */
	lp->mem_control.retention_mask = ~0U;
	lp->mem_control.domain_mask = ~0U;
	lp->mem_control.clock_mask = ~0U;
	lp->mem_armed = false;

	ret = esp32s31_lp_sleep_prepare(&lp->mem_control);
	if (ret)
		goto restore_alarm;
	if (!(lp->mem_control.capabilities & S31_LP_CAP_RETENTION_DESC) ||
	    (gpio.mask && !(lp->mem_control.capabilities & S31_LP_CAP_GPIO_WAKE)) ||
	    ((lp->mem_control.flags & S31_LP_SLEEP_F_RTC_ABSOLUTE) &&
	     !(lp->mem_control.capabilities & S31_LP_CAP_RTC_ABSOLUTE))) {
		ret = -EOPNOTSUPP;
		goto abort_sleep;
	}
	ret = esp32s31_lp_sleep_arm(&lp->mem_control);
	if (ret)
		goto abort_sleep;
	if (lp->mem_control.state != S31_LP_SLEEP_ARMED) {
		pm_wakeup_event(lp->dev, 0);
		ret = -EBUSY;
		goto abort_sleep;
	}
	lp->mem_armed = true;
	return 0;

abort_sleep:
	esp32s31_lp_sleep_abort(&lp->mem_control);
restore_alarm:
	esp32s31_lp_rtc_restore(lp);
	return ret;
}
EXPORT_SYMBOL_GPL(esp32s31_lp_system_suspend_prepare);

int esp32s31_lp_system_suspend_validate(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);
	struct s31_lp_sleep_control control;
	struct esp32s31_gpio_wake gpio;
	int ret;

	if (!lp || !lp->mem_armed)
		return -ENODEV;
	/* A driver which changes wake selection after ordinary device suspend
	 * cannot silently invalidate an already-armed LP request. No mailbox RPC
	 * is issued after device IRQs have been suspended.
	 */
	ret = esp32s31_gpio_get_lp_wake(&gpio);
	if (ret)
		return ret;
	if (gpio.mask != lp->mem_control.gpio_mask_lo ||
	    gpio.types != lp->mem_control.gpio_types ||
	    gpio.pullup != lp->mem_control.gpio_pullup_mask ||
	    gpio.pulldown != lp->mem_control.gpio_pulldown_mask) {
		dev_err(lp->dev, "GPIO wake changed after LP preparation\n");
		return -EAGAIN;
	}
	ret = esp32s31_lp_read_control(lp, &control);
	if (ret)
		return ret;
	if (control.state != S31_LP_SLEEP_ARMED) {
		pm_wakeup_event(lp->dev, 0);
		return -EBUSY;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(esp32s31_lp_system_suspend_validate);

void esp32s31_lp_system_suspend_wake(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);
	struct s31_lp_sleep_control control;

	if (!lp || !lp->mem_armed)
		return;
	/* No mailbox transaction before resume_device_irqs(). The LP response
	 * is checksummed in retained SRAM and its GPIO edge must reach the
	 * consumer even when the physical pulse ended before HP resumed.
	 */
	if (esp32s31_lp_read_control(lp, &control) ||
	    control.sequence != lp->mem_control.sequence ||
	    control.state != S31_LP_SLEEP_WAKING ||
	    !(control.wake_reason & S31_LP_WAKE_GPIO))
		return;
	esp32s31_gpio_replay_lp_wake(control.wake_raw &
				    lp->mem_control.gpio_mask_lo);
}
EXPORT_SYMBOL_GPL(esp32s31_lp_system_suspend_wake);

void esp32s31_lp_system_suspend_finish(void)
{
	struct esp32s31_lp *lp = READ_ONCE(esp32s31_lp_sleep_owner);
	int ret;

	if (!lp)
		return;
	if (!lp->mem_armed)
		goto restore_alarm;
	ret = esp32s31_lp_sleep_query(&lp->mem_control);
	if (ret)
		dev_warn(lp->dev, "failed to query suspend wake: %d\n", ret);
	ret = esp32s31_lp_sleep_reclaim(&lp->mem_control);
	if (ret)
		dev_warn(lp->dev, "failed to reclaim suspend state: %d\n", ret);
	lp->mem_armed = false;
restore_alarm:
	esp32s31_lp_rtc_restore(lp);
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
	/* HP retention and PM records are not firmware load/dump storage. */
	if (offset >= S31_LP_FIRMWARE_SIZE ||
	    len > S31_LP_FIRMWARE_SIZE - offset)
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

static_assert(sizeof(struct s31_lp_poweroff) == 32);
static_assert(S31_LP_SLEEP_CONTROL_OFFSET + sizeof(struct s31_lp_sleep_control) <=
	      S31_LP_POWEROFF_OFFSET);
static_assert(S31_LP_POWEROFF_OFFSET + sizeof(struct s31_lp_poweroff) <=
	      S31_LP_RTC_ANCHOR_OFFSET);
static_assert(S31_LP_RTC_ANCHOR_OFFSET + sizeof(struct s31_lp_rtc_anchor) <=
	      S31_LP_SRAM_SIZE);

static int esp32s31_lp_poweroff_prepare(struct sys_off_data *data)
{
	struct esp32s31_lp *lp = data->cb_data;
	struct s31_lp_poweroff request = {
		.magic = S31_LP_POWEROFF_MAGIC,
		.version = S31_LP_POWEROFF_VERSION,
		.size = sizeof(request),
	};
	void __iomem *handoff = lp->sram + S31_LP_POWEROFF_OFFSET;
	unsigned long flags;

	/* Runs after the clock provider's default-priority XTAL handoff. */
	writel(0, handoff);
	spin_lock_irqsave(&lp->rtc_lock, flags);
	/* A previously delivered alarm is not a new wake request. Capture only
	 * the currently enabled native RTC alarm, preserving its calibrated
	 * absolute deadline across the rest of shutdown.
	 */
	if (lp->rtc_alarm_enabled && device_may_wakeup(lp->dev)) {
		request.flags = S31_LP_POWEROFF_RTC;
		request.deadline_lo = lower_32_bits(lp->rtc_alarm_ticks);
		request.deadline_hi = upper_32_bits(lp->rtc_alarm_ticks);
	}
	lp->rtc_alarm_handoff = true;
	esp32s31_lp_rtc_disarm(lp);
	spin_unlock_irqrestore(&lp->rtc_lock, flags);

	request.crc = crc32_le(~0U, (const u8 *)&request,
			       offsetof(struct s31_lp_poweroff, crc)) ^ ~0U;
	memcpy_toio(handoff + sizeof(request.magic),
		    (const u8 *)&request + sizeof(request.magic),
		    sizeof(request) - sizeof(request.magic));
	wmb();
	writel(request.magic, handoff);
	return NOTIFY_DONE;
}

static struct attribute *esp32s31_lp_attrs[] = {
	&dev_attr_ready.attr,
	&dev_attr_last_message.attr,
	&dev_attr_mailbox_stats.attr,
	&dev_attr_tx_message.attr,
	&dev_attr_ping.attr,
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
	platform_set_drvdata(pdev, lp);

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
	/* A reset must never authorize a later poweroff with a stale deadline. */
	writel(0, lp->sram + S31_LP_POWEROFF_OFFSET);
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
	/* HP reset cause belongs to LP_AONCLKRST, not LP_PERICLKRST. */
	lp->reset_cause = devm_ioremap(dev, S31_LP_AON_HP0_RESET_CAUSE, sizeof(u32));
	if (!lp->reset_cause)
		return -ENOMEM;


	lp->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "espressif,pmu");
	if (IS_ERR(lp->pmu))
		return dev_err_probe(dev, PTR_ERR(lp->pmu), "failed to get PMU syscon\n");

	spin_lock_init(&lp->rtc_lock);
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

	ret = esp32s31_lp_calibrate_rtc_slow(lp);
	if (ret)
		return ret;
	ret = esp32s31_lp_rtc_register(pdev, lp);
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
	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
					SYS_OFF_PRIO_LOW,
					esp32s31_lp_poweroff_prepare, lp);
	if (ret)
		return ret;

	dev_info(dev, "LP-core remoteproc and mailbox registered\n");
	return 0;
}

/* Suspend-to-idle keeps both HP harts and the RTC counter powered. Let
 * genirq wake Linux from the actual target-0 interrupt; no LP dry-run,
 * polling loop or platform wake predicate is needed. MEM still hands that
 * target to LP/SBI through esp32s31_lp_system_suspend_prepare().
 */
static int esp32s31_lp_suspend_late(struct device *dev)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	unsigned long flags;
	bool alarm;
	int ret;

	if (pm_suspend_target_state != PM_SUSPEND_TO_IDLE ||
	    !device_may_wakeup(dev))
		return 0;
	spin_lock_irqsave(&lp->rtc_lock, flags);
	alarm = lp->rtc_alarm_enabled;
	spin_unlock_irqrestore(&lp->rtc_lock, flags);
	if (!alarm)
		return 0;
	ret = enable_irq_wake(lp->rtc_irq);
	if (!ret)
		lp->rtc_irq_wake = true;
	return ret;
}

static int esp32s31_lp_resume_early(struct device *dev)
{
	struct esp32s31_lp *lp = dev_get_drvdata(dev);
	int ret;

	if (!lp->rtc_irq_wake)
		return 0;
	ret = disable_irq_wake(lp->rtc_irq);
	if (!ret)
		lp->rtc_irq_wake = false;
	return ret;
}

static const struct dev_pm_ops esp32s31_lp_pm_ops = {
	LATE_SYSTEM_SLEEP_PM_OPS(esp32s31_lp_suspend_late,
				esp32s31_lp_resume_early)
};

static const struct of_device_id esp32s31_lp_of_match[] = {
	{ .compatible = "espressif,esp32s31-lp-core" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_lp_of_match);

static struct platform_driver esp32s31_lp_driver = {
	.probe = esp32s31_lp_probe,
	.driver = {
		.name = "esp32s31-lp",
		.pm = pm_sleep_ptr(&esp32s31_lp_pm_ops),
		.of_match_table = esp32s31_lp_of_match,
	},
};
module_platform_driver(esp32s31_lp_driver);

MODULE_DESCRIPTION("ESP32-S31 LP-core remoteproc and mailbox driver");
MODULE_LICENSE("GPL");
