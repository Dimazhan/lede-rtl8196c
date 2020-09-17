/*
 *  Realtek RLX based SoC specific interrupt handling
 *
 *  Copyright (C) 2017 Weijie Gao <hackpascal@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>

#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <asm/c-lexra.h>
#include <asm/lxregs.h>

#include <asm/mach-realtek/realtek.h>
#include <asm/mach-realtek/platform.h>
#include "common.h"

static void __iomem *realtek_intctl_base;

static u32 mips_chip_bits;

static void realtek_soc_irq_handler(struct irq_desc *desc)
{
	u32 pending;

	pending = __raw_readl(realtek_intctl_base + REALTEK_INTCTL_REG_MASK) &
		  __raw_readl(realtek_intctl_base + REALTEK_INTCTL_REG_STATUS);

	if (pending & mips_chip_bits) {
		/*
		 * interrupts routed to mips core found here
		 * clear these bits as they can't be handled here
		 */
		__raw_writel(mips_chip_bits, realtek_intctl_base + REALTEK_INTCTL_REG_STATUS);
		pending &= ~mips_chip_bits;

		if (!pending)
			return;
	}

	if (!pending) {
		spurious_interrupt();
		return;
	}

	while (pending) {
		int bit = __ffs(pending);

		generic_handle_irq(REALTEK_SOC_IRQ(bit));
		pending &= ~BIT(bit);
	}
}

static void realtek_soc_irq_unmask(struct irq_data *d)
{
	unsigned int irq = d->irq - REALTEK_SOC_IRQ_BASE;
	u32 t;

	t = __raw_readl(realtek_intctl_base + REALTEK_INTCTL_REG_MASK);
	__raw_writel(t | (1 << irq), realtek_intctl_base + REALTEK_INTCTL_REG_MASK);

	/* flush write */
	__raw_readl(realtek_intctl_base + REALTEK_INTCTL_REG_MASK);
}

static void realtek_soc_irq_mask(struct irq_data *d)
{
	unsigned int irq = d->irq - REALTEK_SOC_IRQ_BASE;
	u32 t;

	t = __raw_readl(realtek_intctl_base + REALTEK_INTCTL_REG_MASK);
	__raw_writel(t & ~(1 << irq), realtek_intctl_base + REALTEK_INTCTL_REG_MASK);

	/* flush write */
	__raw_readl(realtek_intctl_base + REALTEK_INTCTL_REG_MASK);
}

static void realtek_soc_irq_ack(struct irq_data *d)
{
}

static struct irq_chip realtek_misc_irq_chip = {
	.name		= "soc",
	.irq_unmask	= realtek_soc_irq_unmask,
	.irq_mask	= realtek_soc_irq_mask,
	.irq_ack	= realtek_soc_irq_ack,
};

static void __init rtl8196c_irq_init(void)
{
	mips_chip_bits = RTL8196C_INTCTL_IRQ_TC0 | RTL8196C_INTCTL_IRQ_USB |
		RTL8196C_INTCTL_IRQ_PCIE | RTL8196C_INTCTL_IRQ_SWCORE;

	__raw_writel(mips_chip_bits, realtek_intctl_base + REALTEK_INTCTL_REG_MASK);

	__raw_writel(RTL8196C_INTCTL_RS_UART << RTL8196C_INTCTL_IRR0_UART_SHIFT,
		realtek_intctl_base + REALTEK_INTCTL_REG_IRR0);
		
	__raw_writel((RTL8196C_INTCTL_RS_TC0 << RTL8196C_INTCTL_IRR1_TC0_SHIFT) |
		(RTL8196C_INTCTL_RS_GPIO << RTL8196C_INTCTL_IRR1_GPIO_SHIFT) |
		(RTL8196C_INTCTL_RS_SWCORE << RTL8196C_INTCTL_IRR1_SWCORE_SHIFT) |
		(RTL8196C_INTCTL_RS_PCIE << RTL8196C_INTCTL_IRR1_PCIE_SHIFT),
		realtek_intctl_base + REALTEK_INTCTL_REG_IRR1);

	__raw_writel(RTL8196C_INTCTL_RS_USB_HOST << RTL8196C_INTCTL_IRR2_USB_HOST_SHIFT,
		realtek_intctl_base + REALTEK_INTCTL_REG_IRR2);
}

static void __init rtl819xd_irq_init(void)
{
	mips_chip_bits = RTL819XD_INTCTL_IRQ_TC0 | RTL819XD_INTCTL_IRQ_TC1 | RTL819XD_INTCTL_IRQ_SWCORE |
		RTL819XD_INTCTL_IRQ_UART0 | RTL819XD_INTCTL_IRQ_USB | RTL819XD_INTCTL_IRQ_PCIE0 | RTL819XD_INTCTL_IRQ_PCIE1 |
		RTL819XD_INTCTL_IRQ_GPIO_ABCD | RTL819XD_INTCTL_IRQ_GPIO_EFGH;

	__raw_writel(mips_chip_bits, realtek_intctl_base + REALTEK_INTCTL_REG_MASK);

	__raw_writel(
		RTL819XD_INTCTL_RS_SWCORE << RTL819XD_INTCTL_IRR1_SWCORE_SHIFT |
		//RTL819XD_INTCTL_RS_NONE << RTL819XD_INTCTL_IRR1_NONE_SHIFT |
		//RTL819XD_INTCTL_RS_UART1 << RTL819XD_INTCTL_IRR1_UART1_SHIFT |
		RTL819XD_INTCTL_RS_UART0 << RTL819XD_INTCTL_IRR1_UART0_SHIFT |
		//RTL819XD_INTCTL_RS_OTG << RTL819XD_INTCTL_IRR1_OTG_SHIFT |
		RTL819XD_INTCTL_RS_USB_H << RTL819XD_INTCTL_IRR1_USB_SHIFT |
		RTL819XD_INTCTL_RS_TC1 << RTL819XD_INTCTL_IRR1_TC1_SHIFT |
		RTL819XD_INTCTL_RS_TC0 << RTL819XD_INTCTL_IRR1_TC0_SHIFT,
		realtek_intctl_base + REALTEK_INTCTL_REG_IRR1);

	__raw_writel(
		RTL819XD_INTCTL_RS_GPIO_ABCD << RTL819XD_INTCTL_IRR2_GPIO_ABCD_SHIFT |
		RTL819XD_INTCTL_RS_GPIO_EFGH << RTL819XD_INTCTL_IRR2_GPIO_EFGH_SHIFT |
		RTL819XD_INTCTL_RS_PCIE0 << RTL819XD_INTCTL_IRR2_PCIE0_SHIFT |
		RTL819XD_INTCTL_RS_PCIE1 << RTL819XD_INTCTL_IRR2_PCIE1_SHIFT,
		realtek_intctl_base + REALTEK_INTCTL_REG_IRR2);
}

static void __init realtek_soc_irq_init(void)
{
	int i;

	realtek_intctl_base = ioremap_nocache(REALTEK_INTCTL_BASE, REALTEK_INTCTL_SIZE);

	__raw_writel(0, realtek_intctl_base + REALTEK_INTCTL_REG_MASK);
	
	if (soc_is_rtl8196c())
		rtl8196c_irq_init();
	else if (soc_is_rtl819xd())
		rtl819xd_irq_init();

	for (i = 0; i < REALTEK_SOC_IRQ_COUNT; i++)
		irq_set_chip_and_handler(REALTEK_SOC_IRQ(i), &realtek_misc_irq_chip, handle_level_irq);

	irq_set_chained_handler(REALTEK_CPU_IRQ(2), realtek_soc_irq_handler);
	/*for (i = 2; i < 8; i++)
		irq_set_chained_handler(REALTEK_CPU_IRQ(i), realtek_soc_irq_handler);*/
}

asmlinkage void plat_irq_dispatch(void)
{
	unsigned long pending;
	int irq;

	pending = read_c0_status() & read_c0_cause() & ST0_IM;

	if (!pending) {
		spurious_interrupt();
		return;
	}

	pending >>= CAUSEB_IP;
	while (pending) {
		irq = fls(pending) - 1;
		do_IRQ(MIPS_CPU_IRQ_BASE + irq);
		pending &= ~BIT(irq);
	}
}

static inline void unmask_realtek_vec_irq(struct irq_data *d)
{
	set_lxc0_estatus(0x10000 << (d->irq - REALTEK_LOPI_IRQ_BASE));
	irq_enable_hazard();
}

static inline void mask_realtek_vec_irq(struct irq_data *d)
{
	clear_lxc0_estatus(0x10000 << (d->irq - REALTEK_LOPI_IRQ_BASE));
	irq_disable_hazard();
}

static struct irq_chip realtek_vec_irq_controller = {
	.name		= "RLX LOPI",
	.irq_ack	= mask_realtek_vec_irq,
	.irq_mask	= mask_realtek_vec_irq,
	.irq_mask_ack	= mask_realtek_vec_irq,
	.irq_unmask	= unmask_realtek_vec_irq,
	.irq_eoi	= unmask_realtek_vec_irq,
};

static struct irq_desc *realtek_vec_irq_desc;

void __init realtek_vec_irq_init(void)
{
	int i;
	extern char handle_vec;

	/* Mask interrupts. */
	clear_lxc0_estatus(EST0_IM);
	clear_lxc0_ecause(ECAUSEF_IP);

	realtek_vec_irq_desc = irq_desc + REALTEK_LOPI_IRQ_BASE;

	for (i = 0; i < REALTEK_LOPI_IRQ_COUNT; i++)
		irq_set_chip_and_handler(REALTEK_LOPI_IRQ(i), &realtek_vec_irq_controller,
					 handle_percpu_irq);

	write_lxc0_intvec(&handle_vec);
}

asmlinkage void realtek_do_lopi_IRQ(int irq_offset)
{
	do_IRQ(REALTEK_LOPI_IRQ(irq_offset));
}

void __init arch_init_irq(void)
{
	mips_cpu_irq_init();
	if (soc_is_rtl819xd())
		realtek_vec_irq_init();
	realtek_soc_irq_init();
}
