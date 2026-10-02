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

	}
}
