/*
 *  Copyright (C) 2017 Weijie Gao <hackpascal@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */
#ifndef __ASM_MACH_REALTEK_IRQ_H
#define __ASM_MACH_REALTEK_IRQ_H

#ifdef CONFIG_CPU_RX5281
#define NR_IRQS					48
#else
#define NR_IRQS					40
#endif

#define MIPS_CPU_IRQ_BASE		0
#define MIPS_CPU_IRQ_COUNT		8
#define REALTEK_CPU_IRQ(_x)		(MIPS_CPU_IRQ_BASE + (_x))

#ifdef CONFIG_CPU_RX5281
#define REALTEK_LOPI_IRQ_BASE	(MIPS_CPU_IRQ_BASE + MIPS_CPU_IRQ_COUNT)
#define REALTEK_LOPI_IRQ_COUNT	8
#define REALTEK_LOPI_IRQ(_x)	(REALTEK_LOPI_IRQ_BASE + (_x))

#define REALTEK_SOC_IRQ_BASE	(REALTEK_LOPI_IRQ_BASE + REALTEK_LOPI_IRQ_COUNT)
#else
#define REALTEK_SOC_IRQ_BASE	(MIPS_CPU_IRQ_BASE + MIPS_CPU_IRQ_COUNT)
#endif
#define REALTEK_SOC_IRQ_COUNT	32
#define REALTEK_SOC_IRQ(_x)		(REALTEK_SOC_IRQ_BASE + (_x))

#include_next <irq.h>

#endif /* __ASM_MACH_REALTEK_IRQ_H */
