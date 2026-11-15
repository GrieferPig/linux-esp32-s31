// SPDX-License-Identifier: GPL-2.0-only
/*
 * Failure-path observation for the retained ESP32-P4 v1.3 handoff only.
 * Register contracts: ESP-IDF a602e67b, soc/esp32p4/register/hw_ver1 and
 * esp_hal_uart/esp32p4/include/hal/uart_ll.h.
 *
 * No DebugAssist writes, clock enabling, pending clears or extra IPIs.
 * Recorded PC/SP/history are non-atomic samples, NOT live architectural CSRs.
 * Raw UART is bounded best-effort output and may interleave with the console.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <asm/esp32p4_debug.h>

#define P4_BUS_CLOCK		0x500e6014UL
#define P4_UART_SYS_CLOCK	0x500e6018UL
#define P4_UART_APB_CLOCK	0x500e601cUL
#define P4_UART_SCLK		0x500e6068UL
#define P4_DEBUG_BASE		0x3ff06000UL
#define P4_DEBUG_GATE		(P4_DEBUG_BASE + 0x108)
#define P4_UART_FIFO		0x500ca000UL
#define P4_UART_STATUS		0x500ca01cUL
#define P4_UART_FSM		0x500ca070UL
#define P4_UART_CLK_CONF		0x500ca088UL
#define P4_UART_READ_BUDGET	(1U << 20)

static bool p4_diag_ready __ro_after_init;

struct p4_pc_sample {
	u32 record, pc1, sp, pc2, history;
};

struct p4_diag_line {
	char data[192];
	unsigned int length;
};

static int __init p4_diag_init(void)
{
	struct device_node *chosen;
	u32 revision = 0;

	chosen = of_find_node_by_path("/chosen");
	if (chosen)
		of_property_read_u32(chosen, "espressif,chip-revision", &revision);
	of_node_put(chosen);
	p4_diag_ready = revision == 103;
	return 0;
}
early_initcall(p4_diag_init);

static u32 p4_diag_read(unsigned long address)
{
	return readl((void __iomem *)address);
}

static bool p4_diag_uart_ready(void)
{
	if (!(p4_diag_read(P4_UART_SYS_CLOCK) & BIT(18)) ||
	    !(p4_diag_read(P4_UART_APB_CLOCK) & BIT(7)) ||
	    !(p4_diag_read(P4_UART_SCLK) & BIT(26)))
		return false;
	return p4_diag_read(P4_UART_CLK_CONF) & BIT(24);
}

static bool p4_diag_uart_read(unsigned long address, unsigned int *budget,
			      u32 *value)
{
	if (!*budget)
		return false;
	--*budget;
	*value = p4_diag_read(address);
	return true;
}

static bool p4_diag_uart_drain(unsigned int *budget)
{
	u32 status, fsm;

	while (p4_diag_uart_read(P4_UART_STATUS, budget, &status)) {
		if ((status >> 16) & 0xff)
			continue;
		if (!p4_diag_uart_read(P4_UART_FSM, budget, &fsm))
			return false;
		if (!(fsm & 0xf0))
			return true;
	}
	return false;
}

static bool p4_diag_uart_write(const char *data, unsigned int length,
			       unsigned int *budget)
{
	unsigned int i;
	u32 status;

	for (i = 0; i < length; i++) {
		do {
			if (!p4_diag_uart_read(P4_UART_STATUS, budget, &status))
				return false;
		} while (((status >> 16) & 0xff) >= 120);
		/* Full 32-bit write only: byte access may read/consume RX FIFO. */
		/* No MMIO lock is held: avoid per-CPU mmiowb bookkeeping. */
		RISCV_FENCE(w, o);
		writel_relaxed((u8)data[i], (void __iomem *)P4_UART_FIFO);
	}
	return true;
}

static void p4_diag_text(struct p4_diag_line *line, const char *text)
{
	while (*text && line->length < sizeof(line->data))
		line->data[line->length++] = *text++;
}

static void p4_diag_hex(struct p4_diag_line *line, u32 value)
{
	static const char digits[] = "0123456789abcdef";
	int shift;

	for (shift = 28; shift >= 0 && line->length < sizeof(line->data); shift -= 4)
		line->data[line->length++] = digits[(value >> shift) & 0xf];
}

static void p4_diag_field(struct p4_diag_line *line, const char *name, u32 value)
{
	p4_diag_text(line, name);
	p4_diag_hex(line, value);
}

void esp32p4_backtrace_snapshot(bool after)
{
	struct p4_pc_sample samples[2] = {};
	struct p4_diag_line line = {};
	const char *phase = after ? "POST" : "PRE";
	unsigned int budget = P4_UART_READ_BUDGET;
	unsigned int cpu;
	u32 bus, gate = 0;

	BUILD_BUG_ON(IS_ENABLED(CONFIG_MMU));
	BUILD_BUG_ON(IS_ENABLED(CONFIG_SERIAL_ESP32_UHCI));

	if (!READ_ONCE(p4_diag_ready) || !p4_diag_uart_ready())
		return;

	/*
	 * Emit and drain a boundary before the first DebugAssist access.
	 * A finite poll budget does not guarantee a bus transaction can finish.
	 */
	p4_diag_text(&line, "\r\nP4PC ");
	p4_diag_text(&line, phase);
	p4_diag_text(&line, " begin recorded/non-atomic\r\n");
	if (!p4_diag_uart_write(line.data, line.length, &budget) ||
	    !p4_diag_uart_drain(&budget))
		return;

	bus = p4_diag_read(P4_BUS_CLOCK);
	if (bus & BIT(6)) {
		gate = p4_diag_read(P4_DEBUG_GATE);
		if (gate & BIT(0)) {
			for (cpu = 0; cpu < 2; cpu++) {
				unsigned long base = P4_DEBUG_BASE + cpu * 0x80;
				struct p4_pc_sample *sample = &samples[cpu];

				sample->record = p4_diag_read(base + 0x44);
				if ((sample->record & 3) != 3)
					continue;
				sample->pc1 = p4_diag_read(base + 0x48);
				sample->sp = p4_diag_read(base + 0x4c);
				sample->pc2 = p4_diag_read(base + 0x48);
				sample->history = p4_diag_read(base + 0x70);
			}
		}
	}

	for (cpu = 0; cpu < 2; cpu++) {
		struct p4_pc_sample *sample = &samples[cpu];

		line.length = 0;
		p4_diag_text(&line, "P4PC ");
		p4_diag_text(&line, phase);
		p4_diag_text(&line, cpu ? " cpu=1" : " cpu=0");
		if (!(bus & BIT(6))) {
			p4_diag_text(&line, " skip=bus");
			p4_diag_field(&line, " bus=", bus);
		} else if (!(gate & BIT(0))) {
			p4_diag_text(&line, " skip=gate");
			p4_diag_field(&line, " gate=", gate);
		} else if ((sample->record & 3) != 3) {
			p4_diag_text(&line, " skip=record");
			p4_diag_field(&line, " rec=", sample->record);
		} else {
			p4_diag_field(&line, " rec=", sample->record);
			p4_diag_field(&line, " pc1=", sample->pc1);
			p4_diag_field(&line, " sp=", sample->sp);
			p4_diag_field(&line, " pc2=", sample->pc2);
			p4_diag_field(&line, " hist=", sample->history);
		}
		p4_diag_text(&line, "\r\n");
		if (!p4_diag_uart_write(line.data, line.length, &budget))
			return;
	}
	/* Best effort only; never wait for another hart or acquire console locks. */
	p4_diag_uart_drain(&budget);
}
