/*
 *  Realtek RLX based SoC register definitions
 *
 *  Copyright (C) 2017 Weijie Gao <hackpascal@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REALTEK_REGS_H
#define __ASM_MACH_REALTEK_REGS_H

#include <linux/types.h>
#include <linux/io.h>
#include <linux/bitops.h>

/* Register bases */
#define REALTEK_SYS_BASE			0x18000000
#define REALTEK_SYS_SIZE			0x1000
#define REALTEK_MC_BASE				0x18001000
#define REALTEK_MC_SIZE				0x100
#define REALTEK_SPI_BASE			0x18001200
#define REALTEK_SPI_SIZE			0x10
#define REALTEK_UART0_BASE			0x18002000
#define REALTEK_UART0_SIZE			0x20
#define REALTEK_UART1_BASE			0x18002100
#define REALTEK_UART1_SIZE			0x20
#define REALTEK_INTCTL_BASE			0x18003000
#define REALTEK_INTCTL_SIZE			0x100
#define REALTEK_TC_BASE				0x18003100
#define REALTEK_TC_SIZE				0x20
#define REALTEK_GPIO_ABCD_BASE			0x18003500
#define REALTEK_GPIO_ABCD_SIZE			0x1c
#define REALTEK_GPIO_EFGH_BASE			0x1800351c
#define REALTEK_GPIO_EFGH_SIZE			0x1c
#define REALTEK_NIC_BASE			0x18010000
#define REALTEK_NIC_SIZE			0x100
#define REALTEK_USB_OHCI_BASE			0x18020000
#define REALTEK_USB_OHCI_SIZE			0x1000
#define REALTEK_USB_EHCI_BASE			0x18021000
#define REALTEK_USB_EHCI_SIZE			0xf000
#define REALTEK_SWTABLE_BASE			0x1b000000
#define REALTEK_SWTABLE_SIZE			0x140000
#define REALTEK_SWCORE_BASE			0x1b800000
#define REALTEK_SWCORE_SIZE			0x10000

#define REALTEK_PCIE0_RC_CFG_BASE		0x18b00000
#define REALTEK_PCIE0_RC_CFG_SIZE		0x1000
#define REALTEK_PCIE0_RC_EXT_BASE		0x18b01000
#define REALTEK_PCIE0_RC_EXT_SIZE		0x1000
#define REALTEK_PCIE0_EP_CFG_BASE		0x18b10000
#define REALTEK_PCIE0_EP_CFG_SIZE		0x1000
#define REALTEK_PCIE0_EP_EXT_BASE		0x18b11000
#define REALTEK_PCIE0_EP_EXT_SIZE		0x1000

#define REALTEK_PCIE1_RC_CFG_BASE		0x18b20000
#define REALTEK_PCIE1_RC_CFG_SIZE		0x1000
#define REALTEK_PCIE1_RC_EXT_BASE		0x18b21000
#define REALTEK_PCIE1_RC_EXT_SIZE		0x1000
#define REALTEK_PCIE1_EP_CFG_BASE		0x18b30000
#define REALTEK_PCIE1_EP_CFG_SIZE		0x1000
#define REALTEK_PCIE1_EP_EXT_BASE		0x18b31000
#define REALTEK_PCIE1_EP_EXT_SIZE		0x1000

#define REALTEK_PCIE0_IO_BASE			0x18c00000
#define REALTEK_PCIE0_IO_SIZE			0x200000

#define REALTEK_PCIE1_IO_BASE			0x18e00000
#define REALTEK_PCIE1_IO_SIZE			0x200000

#define REALTEK_PCIE0_MEM_BASE			0x19000000
#define REALTEK_PCIE0_MEM_SIZE			0x1000000

#define REALTEK_PCIE1_MEM_BASE			0x1a000000
#define REALTEK_PCIE1_MEM_SIZE			0x1000000


/* Register definitions */
#define REALTEK_SYS_REG_REVISION		0x00
#define REALTEK_SYS_REG_BOOTSTRAP		0x08
#define REALTEK_SYS_REG_CLK_MANAGE		0x10
#define REALTEK_SYS_REG_USB_SIE			0x34
#define REALTEK_SYS_REG_GPIO_MUX		0x40
#define REALTEK_SYS_REG_GPIO_MUX2		0x44
#define REALTEK_SYS_REG_PCIE0_PHY		0x50
#define REALTEK_SYS_REG_PCIE1_PHY		0x54
#define REALTEK_SYS_REG_USB_PHY			0x90

#define REALTEK_MC_REG_DRAM_CFG			0x04

#define REALTEK_INTCTL_REG_MASK			0x00
#define REALTEK_INTCTL_REG_STATUS		0x04
#define REALTEK_INTCTL_REG_IRR0			0x08
#define REALTEK_INTCTL_REG_IRR1			0x0c
#define REALTEK_INTCTL_REG_IRR2			0x10
#define REALTEK_INTCTL_REG_IRR3			0x14
#define REALTEK_INTCTL_REG_IRR4			0x18

#define REALTEK_TC_REG_DATA0			0x00
#define REALTEK_TC_REG_DATA1			0x04
#define REALTEK_TC_REG_COUNT0			0x08
#define REALTEK_TC_REG_COUNT1			0x0c
#define REALTEK_TC_REG_CTRL			0x10
#define REALTEK_TC_REG_IR			0x14
#define REALTEK_TC_REG_CLOCK_DIV		0x18
#define REALTEK_TC_REG_WATCHDOG			0x1c

#define REALTEK_GPIO_REG_CTRL			0x00
#define REALTEK_GPIO_REG_DIR			0x08
#define REALTEK_GPIO_REG_DATA			0x0c
#define REALTEK_GPIO_REG_IR_STATUS		0x10
#define REALTEK_GPIO_REG_IR_MASK_MODE0		0x14
#define REALTEK_GPIO_REG_IR_MASK_MODE1		0x18

#define REALTEK_PCIE_RC_EXT_REG_MDIO		0x00
#define REALTEK_PCIE_RC_EXT_REG_INT_STS		0x04
#define REALTEK_PCIE_RC_EXT_REG_PWR_CR		0x08
#define REALTEK_PCIE_RC_EXT_REG_IP_CFG		0x0c
#define REALTEK_PCIE_RC_EXT_REG_BISTFAIL	0x10


/* SoC identifier */
#define SOC_ID_RTL8196C_REV_A			0x80000001
#define SOC_ID_RTL8196C_REV_B			0x80000002
#define SOC_ID_RTL8197DN				0x8197c003

/* Register bits definitions */
/* Common */

/* REALTEK_SYS_REG_PCIE_PHY */
#define REALTEK_SYS_PCIE_PHY_RESET_L			BIT(0)
#define REALTEK_SYS_PCIE_PHY_LOAD_DONE			BIT(1)
#define REALTEK_SYS_PCIE_PHY_UNK3			BIT(3)

/* REALTEK_MC_REG_DRAM_CFG */
#define REALTEK_DRAM_DBUS_WIDTH_SHIFT			28
#define REALTEK_DRAM_DBUS_WIDTH_MASK			0x3

#define REALTEK_DRAM_MODE_SEL_SHIFT			BIT(27)

#define REALTEK_DRAM_ROW_WIDTH_SHIFT			25
#define REALTEK_DRAM_ROW_WIDTH_MASK			0x3

#define REALTEK_DRAM_COL_WIDTH_SHIFT			22
#define REALTEK_DRAM_COL_WIDTH_MASK			0x7

#define REALTEK_DRAM_BANK_WIDTH_SHIFT			19
#define REALTEK_DRAM_BANK_WIDTH_MASK			1

/* REALTEK_TC_REG_CTRL */
#define REALTEK_TC_CTRL_TC0_EN				BIT(31)
#define REALTEK_TC_CTRL_TC0_MODE			BIT(30)
#define REALTEK_TC_CTRL_TC1_EN				BIT(29)
#define REALTEK_TC_CTRL_TC1_MODE			BIT(28)

/* REALTEK_TC_REG_IR */
#define REALTEK_TC_IR_TC0_EN				BIT(31)
#define REALTEK_TC_IR_TC1_EN				BIT(30)
#define REALTEK_TC_IR_TC0_PENDING			BIT(29)
#define REALTEK_TC_IR_TC1_PENDING			BIT(28)

/* REALTEK_TC_REG_CLOCK_DIV */
#define REALTEK_TC_CLOCK_DIV_FACTOR_SHIFT		16
#define REALTEK_TC_CLOCK_DIV_FACTOR_MASK		0xffff

/* REALTEK_TC_REG_WATCHDOG */
#define REALTEK_TC_WATCHDOG_EN_SHIFT			24
#define REALTEK_TC_WATCHDOG_EN_MASK			0xff

#define REALTEK_TC_WATCHDOG_FEED			BIT(23)

#define REALTEK_TC_WATCHDOG_OVSEL_L_SHIFT		21
#define REALTEK_TC_WATCHDOG_OVSEL_L_MASK		0x3

#define REALTEK_TC_WATCHDOG_IND				BIT(20)

#define REALTEK_TC_WATCHDOG_NRF_TYPE			BIT(19)

#define REALTEK_TC_WATCHDOG_OVSEL_H_SHIFT		17
#define REALTEK_TC_WATCHDOG_OVSEL_H_MASK		0x3

/* REALTEK_PCIE_RC_EXT_REG_MDIO */
#define REALTEK_PCIE_RC_EXT_MDIO_DATA_SHIFT		16
#define REALTEK_PCIE_RC_EXT_MDIO_DATA_MASK		0xffff

#define REALTEK_PCIE_RC_EXT_MDIO_PHYADDR_SHIFT		13
#define REALTEK_PCIE_RC_EXT_MDIO_PHYADDR_MASK		0x7

#define REALTEK_PCIE_RC_EXT_MDIO_REGADDR_SHIFT		8
#define REALTEK_PCIE_RC_EXT_MDIO_REGADDR_MASK		0x1f

#define REALTEK_PCIE_RC_EXT_MDIO_STATUS_SHIFT		5
#define REALTEK_PCIE_RC_EXT_MDIO_STATUS_MASK		0x3

#define REALTEK_PCIE_RC_EXT_MDIO_READY			BIT(4)

#define REALTEK_PCIE_RC_EXT_MDIO_CLK_RATE_SHIFT		2
#define REALTEK_PCIE_RC_EXT_MDIO_CLK_RATE_MASK		0x3

#define REALTEK_PCIE_RC_EXT_MDIO_RESET			BIT(1)
#define REALTEK_PCIE_RC_EXT_MDIO_WR			BIT(0)

/* REALTEK_PCIE_RC_EXT_REG_PWR_CR */
#define REALTEK_PCIE_RC_EXT_PWR_APP_UNLOCK		BIT(10)
#define REALTEK_PCIE_RC_EXT_PWR_APP_PME_TURN_OFF	BIT(9)
#define REALTEK_PCIE_RC_EXT_PWR_APP_INIT_RST		BIT(8)
#define REALTEK_PCIE_RC_EXT_PWR_PHY_SRST_L		BIT(7)
#define REALTEK_PCIE_RC_EXT_PWR_P1_CLK_REQ_EN		BIT(6)
#define REALTEK_PCIE_RC_EXT_PWR_LOW_POWER_EN		BIT(5)
#define REALTEK_PCIE_RC_EXT_PWR_SYS_AUX_PWR_DETECT	BIT(4)
#define REALTEK_PCIE_RC_EXT_PWR_APP_READY_ENTER_L23	BIT(3)
#define REALTEK_PCIE_RC_EXT_PWR_APP_REQ_EXIT_L1		BIT(2)
#define REALTEK_PCIE_RC_EXT_PWR_APP_REQ_ENTER_L1	BIT(1)
#define REALTEK_PCIE_RC_EXT_PWR_APP_LTSSM_EN		BIT(0)

/* REALTEK_PCIE_RC_EXT_REG_IP_CFG */
#define REALTEK_PCIE_RC_EXT_BUS_NUM_SHIFT		8
#define REALTEK_PCIE_RC_EXT_BUS_NUM_MASK		0xff

#define REALTEK_PCIE_RC_EXT_DEV_NUM_SHIFT		3
#define REALTEK_PCIE_RC_EXT_DEV_NUM_MASK		0x1f

#define REALTEK_PCIE_RC_EXT_FUN_NUM_SHIFT		0
#define REALTEK_PCIE_RC_EXT_FUN_NUM_MASK		0x7



/* RTL8196C */

/* Interrupt bits */
#define RTL8196C_INTCTL_IRQ_USB				BIT(16)	/* OR 17? */
#define RTL8196C_INTCTL_IRQ_TC1				BIT(15)
#define RTL8196C_INTCTL_IRQ_TC0				BIT(14)
#define RTL8196C_INTCTL_IRQ_GDMA			BIT(11)
#define RTL8196C_INTCTL_IRQ_PCIE			BIT(10)
#define RTL8196C_INTCTL_IRQ_GPIO			BIT(9)
#define RTL8196C_INTCTL_IRQ_SWCORE			BIT(8)
#define RTL8196C_INTCTL_IRQ_UART0			BIT(7)

/* Interrupt Routing Selection */
#define RTL8196C_INTCTL_RS_UART				2
#define RTL8196C_INTCTL_RS_GPIO				2
#define RTL8196C_INTCTL_RS_USB_HOST			4
#define RTL8196C_INTCTL_RS_PCIE				5
#define RTL8196C_INTCTL_RS_SWCORE			6
#define RTL8196C_INTCTL_RS_TC0				7

/* REALTEK_SYS_REG_BOOTSTRAP */
#define RTL8196C_BOOTSTRAP_AP_ROUTER_MODE		BIT(20)

#define RTL8196C_BOOTSTRAP_CPU_FREQ_DIV			BIT(19)

#define RTL8196C_BOOTSTRAP_CPU_FREQ_SHIFT		14
#define RTL8196C_BOOTSTRAP_CPU_FREQ_MASK		0x7

#define RTL8196C_BOOTSTRAP_SDRAM_CLK_SEL_SHIFT		10
#define RTL8196C_BOOTSTRAP_SDRAM_CLK_SEL_MASK		0x7

#define RTL8196C_BOOTSTRAP_CLKLX_FROM_CLKM		BIT(7)

#define RTL8196C_BOOTSTRAP_BOOT_SEL			BIT(2)

/* REALTEK_SYS_REG_CLK_MANAGE */
#define RTL8196C_SYS_CLK_USB_HOST_EN			BIT(17)
#define RTL8196C_SYS_CLK_PCIE0_DEV_RST_L		BIT(12)
#define RTL8196C_SYS_CLK_PCIE0_EN			BIT(11)

/* REALTEK_SYS_REG_GPIO_MUX */
#define RTL8196C_GPIO_MUX_PCIE_RST			BIT(23)

#define RTL8196C_GPIO_MUX_UART				BIT(22)

#define RTL8196C_GPIO_MUX_JTAG_SHIFT			20
#define RTL8196C_GPIO_MUX_JTAG_MASK			0x3

#define RTL8196C_GPIO_MUX_MEM_SHIFT			18
#define RTL8196C_GPIO_MUX_MEM_MASK			0x3

#define RTL8196C_GPIO_MUX_GPIOC0_SHIFT			12
#define RTL8196C_GPIO_MUX_GPIOC0_MASK			0x3

#define RTL8196C_GPIO_MUX_RESET_SHIFT			10
#define RTL8196C_GPIO_MUX_RESET_MASK			0x3

#define RTL8196C_GPIO_MUX_PORT4_SHIFT			8
#define RTL8196C_GPIO_MUX_PORT4_MASK			0x3

#define RTL8196C_GPIO_MUX_PORT3_SHIFT			6
#define RTL8196C_GPIO_MUX_PORT3_MASK			0x3

#define RTL8196C_GPIO_MUX_PORT2_SHIFT			4
#define RTL8196C_GPIO_MUX_PORT2_MASK			0x3

#define RTL8196C_GPIO_MUX_PORT1_SHIFT			2
#define RTL8196C_GPIO_MUX_PORT1_MASK			0x3

#define RTL8196C_GPIO_MUX_PORT0_SHIFT			0
#define RTL8196C_GPIO_MUX_PORT0_MASK			0x3

/* REALTEK_INTCTL_REG_IRR0 */
#define RTL8196C_INTCTL_IRR0_UART_SHIFT			28
#define RTL8196C_INTCTL_IRR0_UART_MASK			0x0f

/* REALTEK_INTCTL_REG_IRR1 */
#define RTL8196C_INTCTL_IRR1_TC1_SHIFT			28
#define RTL8196C_INTCTL_IRR1_TC1_MASK			0x0f

#define RTL8196C_INTCTL_IRR1_TC0_SHIFT			24
#define RTL8196C_INTCTL_IRR1_TC0_MASK			0x0f

#define RTL8196C_INTCTL_IRR1_GDMA_SHIFT			12
#define RTL8196C_INTCTL_IRR1_GDMA_MASK			0x0f

#define RTL8196C_INTCTL_IRR1_PCIE_SHIFT			8
#define RTL8196C_INTCTL_IRR1_PCIE_MASK			0x0f

#define RTL8196C_INTCTL_IRR1_GPIO_SHIFT			4
#define RTL8196C_INTCTL_IRR1_GPIO_MASK			0x0f

#define RTL8196C_INTCTL_IRR1_SWCORE_SHIFT		0
#define RTL8196C_INTCTL_IRR1_SWCORE_MASK		0x0f

/* REALTEK_INTCTL_REG_IRR2 */
#define RTL8196C_INTCTL_IRR2_USB_HOST_SHIFT		0
#define RTL8196C_INTCTL_IRR2_USB_HOST_MASK		0x0f

/* REALTEK_TC_REG_DATA / REALTEK_TC_REG_COUNT */
#define RTL8196C_TC_DATA_REVA_SHIFT			8
#define RTL8196C_TC_DATA_REVA_MASK			0xffffff

#define RTL8196C_TC_DATA_REVB_SHIFT			4
#define RTL8196C_TC_DATA_REVB_MASK			0xffffffff


/* RTL819xD */

/* REALTEK_SYS_REG_BOOTSTRAP */
#define RTL819XD_BOOTSTRAP_CPU_FREQ_DIV_SHIFT	18
#define RTL819XD_BOOTSTRAP_CPU_FREQ_DIV_MASK	0x3

#define RTL819XD_BOOTSTRAP_CPU_FREQ_SHIFT		13
#define RTL819XD_BOOTSTRAP_CPU_FREQ_MASK		0xf

#define RTL819XD_BOOTSTRAP_SDRAM_CLK_SEL_SHIFT	10
#define RTL819XD_BOOTSTRAP_SDRAM_CLK_SEL_MASK	0x7

#define RTL819XD_BOOTSTRAP_CLKLX_FROM_CLKM		BIT(7)

#define RTL819XD_BOOTSTRAP_BOOT_SEL_SHIFT		0
#define RTL819XD_BOOTSTRAP_BOOT_SEL_MASK		0x7

/* REALTEK_TC_REG_DATA / REALTEK_TC_REG_COUNT */
#define RTL819XD_TC_DATA_SHIFT					4
#define RTL819XD_TC_DATA_MASK					0xffffff

/* Interrupt bits */
#define RTL819XD_INTCTL_IRQ_TC0					BIT(8)
#define RTL819XD_INTCTL_IRQ_TC1					BIT(9)
#define RTL819XD_INTCTL_IRQ_USB					BIT(10)
#define RTL819XD_INTCTL_IRQ_OTG					BIT(11)
#define RTL819XD_INTCTL_IRQ_UART0				BIT(12)
#define RTL819XD_INTCTL_IRQ_UART1				BIT(13)
#define RTL819XD_INTCTL_IRQ_PCI					BIT(14)
#define RTL819XD_INTCTL_IRQ_SWCORE				BIT(15)
#define RTL819XD_INTCTL_IRQ_GPIO_ABCD			BIT(16)
#define RTL819XD_INTCTL_IRQ_GPIO_EFGH			BIT(17)
#define RTL819XD_INTCTL_IRQ_NFBI				BIT(18)
#define RTL819XD_INTCTL_IRQ_PCM					BIT(19)
#define RTL819XD_INTCTL_IRQ_CRYPTO				BIT(20)
#define RTL819XD_INTCTL_IRQ_PCIE				BIT(21)
#define RTL819XD_INTCTL_IRQ_PCIE2				BIT(22)
#define RTL819XD_INTCTL_IRQ_GDMA				BIT(23)
#define RTL819XD_INTCTL_IRQ_I2S					BIT(26)
#define RTL819XD_INTCTL_IRQ_WLAN_MAC			BIT(29)

/* Interrupt Routing Selection */
#define RTL819XD_INTCTL_RS_TC0					13
#define RTL819XD_INTCTL_RS_TC1					2
#define RTL819XD_INTCTL_RS_USB_H				10
#define RTL819XD_INTCTL_RS_USB_D				2
#define RTL819XD_INTCTL_RS_OTG					15
#define RTL819XD_INTCTL_RS_UART0				8
#define RTL819XD_INTCTL_RS_UART1				2
#define RTL819XD_INTCTL_RS_NONE					2
#define RTL819XD_INTCTL_RS_SWCORE				12
#define RTL819XD_INTCTL_RS_GPIO_ABCD			(16+16)
#define RTL819XD_INTCTL_RS_GPIO_EFGH			(16+17)
#define RTL819XD_INTCTL_RS_PCIE					11
#define RTL819XD_INTCTL_RS_PCIE2				14

/* REALTEK_INTCTL_REG_IRR1 */
#define RTL819XD_INTCTL_IRR1_TC0_SHIFT			0
#define RTL819XD_INTCTL_IRR1_TC1_SHIFT			4
#define RTL819XD_INTCTL_IRR1_USB_SHIFT			8
#define RTL819XD_INTCTL_IRR1_OTG_SHIFT			12
#define RTL819XD_INTCTL_IRR1_UART0_SHIFT		16
#define RTL819XD_INTCTL_IRR1_UART1_SHIFT		20
#define RTL819XD_INTCTL_IRR1_NONE_SHIFT			24
#define RTL819XD_INTCTL_IRR1_SWCORE_SHIFT		28

/* REALTEK_INTCTL_REG_IRR2 */
#define RTL819XD_INTCTL_IRR1_GPIO_ABCD_SHIFT	0
#define RTL819XD_INTCTL_IRR1_GPIO_EFGH_SHIFT	4
#define RTL819XD_INTCTL_IRR1_PCIE_SHIFT			20
#define RTL819XD_INTCTL_IRR1_PCIE2_SHIFT		24

/* REALTEK_SYS_REG_CLK_MANAGE */
#define RTL819XD_SYS_CLK_PCIE0_EN				BIT(14)
#define RTL819XD_SYS_CLK_PCIE1_EN				BIT(16)
#define RTL819XD_SYS_CLK_PCIE0_DEV_RST_L		BIT(26)

/* REALTEK_SYS_REG_REVISION */
#define RTL819XD_SYS_REV_RTL8198_REVISION_B		0xC0000001
#define RTL819XD_SYS_REV_RTL8197D				0x8197C000

/* REALTEK_SYS_REG_GPIO_MUX */
#define RTL819XD_GPIO_MUX_PCIE_RST				BIT(6)
#define RTL819XD_GPIO_MUX_PCIE_RST_SHIFT		6
#define RTL819XD_GPIO_MUX_PCIE_RST_MASK			0x3

#define RTL819XD_GPIO_MUX_UART0					BIT(5)

#define RTL819XD_GPIO_MUX_JTAG_SHIFT			0
#define RTL819XD_GPIO_MUX_JTAG_CLEAR_MASK		0x7
#define RTL819XD_GPIO_MUX_JTAG_SET_MASK			0x6

#endif /* __ASM_MACH_REALTEK_REGS_H */
