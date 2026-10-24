/* SPDX-License-Identifier: GPL-2.0-only */
/* Used only for the relocatable vmlinux.o link. */
SECTIONS {
	.s31_rx_ram.text : {
		*tcp_input.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*tcp.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*gro.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*skbuff.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		/* Measured RX allocation, IRQ, wakeup and network-core paths. */
		*lib/genalloc.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*lib/find_bit.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*arch/riscv/lib/uaccess.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*mm/slub.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*kernel/sched/core.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*kernel/sched/fair.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*kernel/locking/spinlock.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*kernel/softirq.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*kernel/time/hrtimer.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*kernel/irq/chip.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*kernel/irq/handle.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*kernel/irq/manage.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/core/dev.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/core/sock.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		/* IPv4 GRO, ACK output, page fragments and cross-hart RX kicks. */
		*net/ipv4/tcp_offload.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/ipv4/tcp_ipv4.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/ipv4/ip_input.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/ipv4/ip_output.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/ipv4/af_inet.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/ipv4/inet_connection_sock.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*mm/page_alloc.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*mm/page_frag_cache.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*kernel/smp.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		/* CPU1 RX wakeups cross the SoC doorbell and virtual-IPI mux on
		 * every coalesced kick. Keep their small steady-state wrappers
		 * beside the existing CSD/IRQ-flow code; cold setup stays XIP. */
		*drivers/irqchip/irq-esp32s31-smp.o(.text.esp32s31_ipi_chained
			.text.esp32s31_ipi_send .text.esp32s31_tlb_flush_poll)
		*kernel/irq/ipi-mux.o(.text.ipi_mux_process .text.ipi_mux_pending
			.text.ipi_mux_send_mask)
		*kernel/irq/irqdesc.o(.text.handle_irq_desc)
		/* Remaining sampled allocator, wakeup and TCP ACK paths. */
		*lib/bitmap.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*kernel/sched/build_utility.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*kernel/workqueue.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*kernel/time/time.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/ipv4/tcp_output.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		/* RX checksum and socket copying; early memcpy/memset stay in Flash. */
		*lib/checksum.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*arch/riscv/lib/csum.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*lib/iov_iter.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/core/datagram.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/ipv4/tcp_timer.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		/* Keep the SoftMAC data transmit, completion, receive and CCMP
		 * paths in executable PSRAM; station setup stays in Flash XIP. */
		*net/mac80211/tx.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/mac80211/status.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/mac80211/rx.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/mac80211/wpa.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/mac80211/aead_api.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		*net/mac80211/agg-rx.o(.text .text.* .sched.text .spinlock.text .irqentry.text .softirqentry.text)
		/* Station setup/teardown is cold on a connected STA. Keep only
		 * lookup helpers used by the RX path; leave the rest in Flash XIP. */
		*net/mac80211/sta_info.o(.text.sta_info_get_by_addrs
			.text.link_sta_info_get_bss
			.text.sta_info_get_by_idx
			.text.ieee80211_find_sta_by_link_addrs)

        /* Timer PC sampling: frequently called small RX helpers remain in
         * Flash despite the protocol stack being in PSRAM. Select only the
         * measured functions; early memcpy/memset aliases stay in Flash. */
        *net/wireless/util.o(.text.ieee80211_hdrlen .text.ieee80211_channel_to_freq_khz)
        *net/ethernet/eth.o(.text.eth_type_trans .text.eth_skb_pkt_type .text.eth_skb_pkt_type.*)
        *lib/string.o(.text.memcmp .text.bcmp)

		/* Hardware/timer-PC profiles also find these small per-IRQ and
		 * per-wakeup helpers in Flash. Keep their normal cached execution
		 * in PSRAM alongside the IRQ flow and network callers above. This
		 * does not make them cache-off safe or consume radio internal SRAM.
		 * Select individual sections: boot, PM and cold policy code stay XIP.
		 */
		*kernel/sched/build_policy.o(.text.update_irq_load_avg
			.text.irqtime_account_irq .text.irqtime_account_delta
			.text.update_rt_rq_load_avg .text.__update_load_avg_se
			.text.decay_load .text.dequeue_rt_stack)
		*drivers/clocksource/timer-esp32s31-systimer.o(
			.text.s31_systimer_read_counter_locked)
		*arch/riscv/kernel/s31-blob-fp-adapter.o(
			.text.esp32s31_radio_task_current_data)
		*kernel/irq/irqdesc.o(.text.generic_handle_domain_irq)
		*kernel/irq/spurious.o(.text.note_interrupt)
		*kernel/time/timekeeping.o(.text.ktime_get)
		*kernel/time/clockevents.o(.text.clockevents_program_event)
		*net/mac80211/util.o(.text.ieee80211_get_bssid)

		/* BT wait-operation profile: scheduling/resumption is a large CPU
		 * envelope. Try its remaining Flash RT/FP helper code in cached
		 * executable PSRAM. No scheduling/FP protocol or SRAM change. */
		*kernel/sched/build_policy.o(.text.__accumulate_pelt_segments .text._double_lock_balance.isra.0 .text.add_nr_running
			.text.balance_rt .text.cpufreq_update_util.constprop.0 .text.dequeue_pushable_task
			.text.dequeue_task_rt .text.div_u64 .text.double_unlock_balance
			.text.enqueue_pushable_task .text.enqueue_task_rt .text.enqueue_top_rt_rq
			.text.find_lock_lowest_rq .text.find_lowest_rq .text.get_push_task
			.text.move_queued_task_locked .text.pick_next_pushable_task .text.pick_next_rt_entity
			.text.pick_task_rt .text.pull_rt_task .text.push_rt_task
			.text.push_rt_tasks .text.put_prev_task_rt .text.requeue_task_rt.isra.0
			.text.rq_of_rt_se .text.rt_clear_overload.part.0 .text.rt_queue_push_tasks.part.0
			.text.rt_set_overload.part.0 .text.rto_next_cpu .text.sched_find_first_bit
			.text.select_task_rq_rt .text.set_next_task_rt .text.task_tick_rt
			.text.task_woken_rt .text.test_tsk_need_resched .text.update_curr_rt
			.text.wakeup_preempt_rt)
		*arch/riscv/kernel/s31-blob-fp-adapter.o(.text.esp32s31_blob_fp_pre_switch .text.esp32s31_blob_fp_post_switch .text.bank_save
			.text.bank_restore .text.bank_fs .text.require_transition)
		*arch/riscv/kernel/s31-blob-fp-core.o(.text.s31_fp_pre_switch .text.s31_fp_post_switch)
		*arch/riscv/kernel/s31-blob-fp-bank.o(.text)
		/* The timeout wrappers share one __sched input section. */
		*kernel/time/sleep_timeout.o(.sched.text)

	}
}
