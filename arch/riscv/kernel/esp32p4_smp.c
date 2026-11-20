// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/memblock.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <asm/smp.h>

#define P4_CORE1_CLK_REG	0x500e6014u
#define P4_CORE1_CLK_EN		BIT(4)
#define P4_CORE1_RST_REG	0x500e60c0u
#define P4_CORE1_RESET		BIT(8)
#define P4_CORE_STALL_REG	0x50115200u
#define P4_CORE1_STALL_MASK	GENMASK(23, 16)
#define P4_CORE_STALLED_REG	0x500e5064u
#define P4_CORE1_STALLED	BIT(1)
#define P4_ROM_APPBOOT		0x4fc000a8u
#define P4_SECONDARY_LPRAM	0x50108000u
#define P4_SECONDARY_ARGS	0x50108100u

extern const u8 esp32p4_secondary_stub[];
extern const u8 esp32p4_secondary_stub_end[];
extern void esp32p4_secondary_start(void);

/*
 * TRM ch.9 System and Memory - LP SRAM is executable on the HP CPUs.
 * The rev1.3 ROM enters this temporary Linux trampoline on hart 1.
 * Install its PSRAM PMP entry before entering the Linux spinwait path.
 */
asm(".pushsection .init.text,\"ax\"\n"
    ".balign 4\n"
    ".global esp32p4_secondary_stub, esp32p4_secondary_stub_end\n"
    "esp32p4_secondary_stub:\n"
    "csrci mstatus, 8\n"
    "li t0, 0x50108100\n"
    "lw t1, 0(t0)\n"
    "csrr t2, 0x3a1\n"
    "srli t2, t2, 24\n"
    "bnez t2, 1f\n"
    "csrw 0x3b7, t1\n"
    "li t3, 0x1f000000\n"
    "csrs 0x3a1, t3\n"
    "1: csrr t2, 0x3b7\n"
    "bne t1, t2, 2f\n"
    "csrr t2, 0x3a1\n"
    "srli t2, t2, 24\n"
    "andi t2, t2, 0x7f\n"
    "li t3, 0x1f\n"
    "bne t2, t3, 2f\n"
    "lw t1, 4(t0)\n"
    "li t2, 1\n"
    "sw t2, 8(t0)\n"
    "fence iorw, iorw\n"
    "fence.i\n"
    "jr t1\n"
    "2: li t2, 2\n"
    "sw t2, 8(t0)\n"
    "fence iorw, iorw\n"
    "3: wfi\n"
    "j 3b\n"
    "esp32p4_secondary_stub_end:\n"
    ".popsection\n");

int __ref esp32p4_boot_secondary(unsigned int hart)
{
	void __iomem *args = (void __iomem *)P4_SECONDARY_ARGS;
	void __iomem *entry = (void __iomem *)P4_SECONDARY_LPRAM;
	void __iomem *stall = (void __iomem *)P4_CORE_STALL_REG;
	void __iomem *clock = (void __iomem *)P4_CORE1_CLK_REG;
	void __iomem *reset = (void __iomem *)P4_CORE1_RST_REG;
	void (*rom_boot_addr)(u32) = (void *)P4_ROM_APPBOOT;
	phys_addr_t base = memblock_start_of_DRAM();
	phys_addr_t size = memblock_end_of_DRAM() - base;
	size_t len = esp32p4_secondary_stub_end - esp32p4_secondary_stub;
	u32 status;
	int ret;

	if (hart != 1 || !len || len > P4_SECONDARY_ARGS - P4_SECONDARY_LPRAM ||
	    base != 0x48000000 || (size != SZ_16M && size != SZ_32M && size != SZ_64M))
		return -EINVAL;

	memcpy_toio(entry, esp32p4_secondary_stub, len);
	writel((base >> 2) | ((size >> 3) - 1), args);
	writel((u32)esp32p4_secondary_start, args + 4);
	writel(0, args + 8);
	asm volatile("fence iorw, iorw" ::: "memory");

	/*
	 * TRM ch.12 Reset and Clock - release HP CPU1 clock and global reset.
	 * IDF start_other_core() and cpu_utility_ll_enable_clock_and_reset_app_cpu()
	 * use the same ROM appcpu hook after releasing the PMU stall.
	 */
	writel(readl(stall) | P4_CORE1_STALL_MASK, stall);
	ret = readl_poll_timeout_atomic((void __iomem *)P4_CORE_STALLED_REG, status,
				       !(status & P4_CORE1_STALLED), 1, 100000);
	if (ret)
		return ret;

	writel(readl(clock) | P4_CORE1_CLK_EN, clock);
	rom_boot_addr(P4_SECONDARY_LPRAM);
	asm volatile("fence iorw, iorw" ::: "memory");
	writel(readl(reset) & ~P4_CORE1_RESET, reset);
	ret = readl_poll_timeout_atomic(args + 8, status, status, 1, 100000);
	rom_boot_addr(0);
	if (ret)
		return ret;
	return status == 1 ? 0 : -EIO;
}
