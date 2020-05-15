/*
 *  Realtek RLX based SoC PCI bus controller initialization
 *
 *  Copyright (C) 2017 Weijie Gao <hackpascal@gmail.com>
 *
 *  Parts of this file are based on Linux kernel of Realtek RSDK 1.3.6
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <asm/mach-realtek/realtek.h>
#include <asm/mach-realtek/platform.h>
#include "common.h"
#include "irq.h"
#include "gpio.h"

static struct resource realtek_pcie0_resource[] = {
	{
		.name	= "rc_cfg_base",
		.flags	= IORESOURCE_MEM,
		.start	= REALTEK_PCIE0_RC_CFG_BASE,
		.end	= REALTEK_PCIE0_RC_CFG_BASE + REALTEK_PCIE0_RC_CFG_SIZE - 1,
	},
	{
		.name	= "dev_cfg_base",
		.flags	= IORESOURCE_MEM,
		.start	= REALTEK_PCIE0_EP_CFG_BASE,
		.end	= REALTEK_PCIE0_EP_CFG_BASE + REALTEK_PCIE0_EP_CFG_SIZE - 1,
	},
	{
		.name	= "io_base",
		.flags	= IORESOURCE_IO,
		.start	= REALTEK_PCIE0_IO_BASE,
		.end	= REALTEK_PCIE0_IO_BASE + REALTEK_PCIE0_IO_SIZE - 1,
	},
	{
		.name	= "mem_base",
		.flags	= IORESOURCE_MEM,
		.start	= REALTEK_PCIE0_MEM_BASE,
		.end	= REALTEK_PCIE0_MEM_BASE + REALTEK_PCIE0_MEM_SIZE - 1,
	}
};

static struct resource realtek_pcie1_resource[] = {
	{
		.name	= "rc_cfg_base",
		.flags	= IORESOURCE_MEM,
		.start	= REALTEK_PCIE1_RC_CFG_BASE,
		.end	= REALTEK_PCIE1_RC_CFG_BASE + REALTEK_PCIE1_RC_CFG_SIZE - 1,
	},
	{
		.name	= "dev_cfg_base",
		.flags	= IORESOURCE_MEM,
		.start	= REALTEK_PCIE1_EP_CFG_BASE,
		.end	= REALTEK_PCIE1_EP_CFG_BASE + REALTEK_PCIE1_EP_CFG_SIZE - 1,
	},
	{
		.name	= "io_base",
		.flags	= IORESOURCE_IO,
		.start	= REALTEK_PCIE1_IO_BASE,
		.end	= REALTEK_PCIE1_IO_BASE + REALTEK_PCIE1_IO_SIZE - 1,
	},
	{
		.name	= "mem_base",
		.flags	= IORESOURCE_MEM,
		.start	= REALTEK_PCIE1_MEM_BASE,
		.end	= REALTEK_PCIE1_MEM_BASE + REALTEK_PCIE1_MEM_SIZE - 1,
	}
};

struct realtek_pcie_reset_controller {
	void __iomem *rc_ext_base;
	u32 rc_phy_reg;
};

static void __init realtek_pcie_mdio_write(struct realtek_pcie_reset_controller *rprc, u32 reg, u32 data)
{
	__raw_writel(
		((reg & REALTEK_PCIE_RC_EXT_MDIO_REGADDR_MASK) << REALTEK_PCIE_RC_EXT_MDIO_REGADDR_SHIFT) |
		((data & REALTEK_PCIE_RC_EXT_MDIO_DATA_MASK) << REALTEK_PCIE_RC_EXT_MDIO_DATA_SHIFT) |
		REALTEK_PCIE_RC_EXT_MDIO_WR,
		rprc->rc_ext_base + REALTEK_PCIE_RC_EXT_REG_MDIO);

	mdelay(1);
}

static void __init realtek_pcie_mdio_reset(struct realtek_pcie_reset_controller *rprc)
{
	realtek_sys_write(rprc->rc_phy_reg, REALTEK_SYS_PCIE_PHY_UNK3);
	realtek_sys_write(rprc->rc_phy_reg, REALTEK_SYS_PCIE_PHY_UNK3 | REALTEK_SYS_PCIE_PHY_RESET_L);
	realtek_sys_write(rprc->rc_phy_reg, REALTEK_SYS_PCIE_PHY_UNK3 | REALTEK_SYS_PCIE_PHY_LOAD_DONE | REALTEK_SYS_PCIE_PHY_RESET_L);
}

static void __init realtek_pcie_phy_reset(struct realtek_pcie_reset_controller *rprc)
{
	__raw_writel(REALTEK_PCIE_RC_EXT_PWR_APP_LTSSM_EN, rprc->rc_ext_base + REALTEK_PCIE_RC_EXT_REG_PWR_CR);
	__raw_writel(REALTEK_PCIE_RC_EXT_PWR_PHY_SRST_L | REALTEK_PCIE_RC_EXT_PWR_APP_LTSSM_EN, rprc->rc_ext_base + REALTEK_PCIE_RC_EXT_REG_PWR_CR);
}

static void __init rtl8196c_pcie_reset(struct realtek_pcie_reset_controller *rprc, int pcie_xtal_40mhz)
{
	u32 val;

	/* Enable PCIe controller */
	val = realtek_sys_read(REALTEK_SYS_REG_CLK_MANAGE);
	val |= RTL8196C_SYS_CLK_PCIE0_EN;
	realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);

#if 0
	/* Is it OK not to set this? */
	__raw_writel((1 << REALTEK_PCIE_RC_EXT_DEV_NUM_SHIFT), rprc->rc_ext_base + REALTEK_PCIE_RC_EXT_REG_IP_CFG);
#endif

	realtek_pcie_mdio_reset(rprc);
	realtek_pcie_phy_reset(rprc);

	realtek_pcie_mdio_write(rprc, 0x00, 0xD087);
	realtek_pcie_mdio_write(rprc, 0x01, 0x0003);
	realtek_pcie_mdio_write(rprc, 0x02, 0x4d18);

	if (pcie_xtal_40mhz) {
		realtek_pcie_mdio_write(rprc, 0x05, 0x0BCB);
		realtek_pcie_mdio_write(rprc, 0x06, 0xF148);
	} else {
		realtek_pcie_mdio_write(rprc, 0x06, 0xf848);
	}

	realtek_pcie_mdio_write(rprc, 0x07, 0x31ff);
	realtek_pcie_mdio_write(rprc, 0x08, 0x18d7);
	realtek_pcie_mdio_write(rprc, 0x09, 0x539c);
	realtek_pcie_mdio_write(rprc, 0x0a, 0x20eb);
	realtek_pcie_mdio_write(rprc, 0x0d, 0x1764);
	realtek_pcie_mdio_write(rprc, 0x0b, 0x0511);
	realtek_pcie_mdio_write(rprc, 0x0f, 0x0a00);
	realtek_pcie_mdio_write(rprc, 0x19, 0xFCE0); 
	realtek_pcie_mdio_write(rprc, 0x1e, 0xC280);

	/* Reset PCIe device */
	val = realtek_sys_read(REALTEK_SYS_REG_CLK_MANAGE);
	val &= ~RTL8196C_SYS_CLK_PCIE0_DEV_RST_L;
	realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
	mdelay(1);

	val |= RTL8196C_SYS_CLK_PCIE0_DEV_RST_L;
	realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
	mdelay(1);

	realtek_pcie_phy_reset(rprc);
}

static void __init rtl819xd_pcie_reset(struct realtek_pcie_reset_controller *rprc, int pcie_xtal_40mhz)
{
	u32 val;
#define PCIE1_GPIO	44

	// Enable PCIe controller
	val = realtek_sys_read(REALTEK_SYS_REG_CLK_MANAGE);
	if (rprc->rc_phy_reg == REALTEK_SYS_REG_PCIE0_PHY)
		val |= RTL819XD_SYS_CLK_PCIE0_EN;
	else
		val |= RTL819XD_SYS_CLK_PCIE1_EN;
	realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);

	// Enable PCIe device
	if (rprc->rc_phy_reg == REALTEK_SYS_REG_PCIE0_PHY) {
		val = realtek_sys_read(REALTEK_SYS_REG_CLK_MANAGE);
		val &= ~RTL819XD_SYS_CLK_PCIE0_DEV_RST_L;
		realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
		val |= RTL819XD_SYS_CLK_PCIE0_DEV_RST_L;
		realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
	} else {
		realtek_set_gpio_mux(0,RTL819XD_GPIO_MUX_PCIE_RST_MASK << RTL819XD_GPIO_MUX_PCIE_RST_SHIFT);
		realtek_set_gpio_control(PCIE1_GPIO, true);//port F bit 4 0x1000
		realtek_set_gpio_direction_output(PCIE1_GPIO, 1);
	}
//#ifdef CONFIG_RTL_819XD
	val = realtek_sys_read(REALTEK_SYS_REG_CLK_MANAGE);
	val |= (1<<12)|(1<<13)|(1<<18);
	realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
//#endif

	realtek_pcie_mdio_reset(rprc);

//#if defined(CONFIG_RTL8198_REVISION_B) || defined(CONFIG_RTL_819XD)
	val = realtek_sys_read(REALTEK_SYS_REG_REVISION);
	if ((val >= SOC_ID_RTL8198_REV_B) || ((val&0xfffff000) == SOC_ID_RTL8197D))
	{
		realtek_pcie_mdio_write(rprc, 0x00, 0xD087);
		realtek_pcie_mdio_write(rprc, 0x01, 0x0003);
		realtek_pcie_mdio_write(rprc, 0x02, 0x4d18);

		if (pcie_xtal_40mhz) {
			realtek_pcie_mdio_write(rprc, 0x05, 0x0BCB);
			realtek_pcie_mdio_write(rprc, 0x06, 0xF148);
		} else {
			realtek_pcie_mdio_write(rprc, 0x06, 0xf848);
		}

		realtek_pcie_mdio_write(rprc, 0x07, 0x31ff);
		realtek_pcie_mdio_write(rprc, 0x08, 0x18d5);
		realtek_pcie_mdio_write(rprc, 0x09, 0x539c);
		realtek_pcie_mdio_write(rprc, 0x0a, 0x20eb);
		realtek_pcie_mdio_write(rprc, 0x0d, 0x1766);
//#ifdef CONFIG_RTL_819XD
		realtek_pcie_mdio_write(rprc, 0x0b, 0x0711);
//#else
//		realtek_pcie_mdio_write(rprc, 0x0b, 0x0511);
//#endif
		realtek_pcie_mdio_write(rprc, 0x0f, 0x0a00);
		realtek_pcie_mdio_write(rprc, 0x19, 0xFCE0);
		realtek_pcie_mdio_write(rprc, 0x1a, 0x7e4f);
		realtek_pcie_mdio_write(rprc, 0x1b, 0xFC01);
		realtek_pcie_mdio_write(rprc, 0x1e, 0xC280);
	} else
//#endif
	{
		realtek_pcie_mdio_write(rprc, 0x00, 0xD087);
		realtek_pcie_mdio_write(rprc, 0x01, 0x0003);
		realtek_pcie_mdio_write(rprc, 0x06, 0xf448);
		realtek_pcie_mdio_write(rprc, 0x06, 0x408);
		realtek_pcie_mdio_write(rprc, 0x07, 0x31ff);
		realtek_pcie_mdio_write(rprc, 0x08, 0x18d5);
		realtek_pcie_mdio_write(rprc, 0x09, 0x531c);
		realtek_pcie_mdio_write(rprc, 0x0d, 0x1766);
		realtek_pcie_mdio_write(rprc, 0x0f, 0x0010);
		realtek_pcie_mdio_write(rprc, 0x19, 0xFCE0);
		realtek_pcie_mdio_write(rprc, 0x1e, 0xC280);
	}

	// Reset PCIe device
	if (rprc->rc_phy_reg == REALTEK_SYS_REG_PCIE0_PHY) {
		val = realtek_sys_read(REALTEK_SYS_REG_CLK_MANAGE);
		val &= ~RTL819XD_SYS_CLK_PCIE0_DEV_RST_L;
		realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
		mdelay(1);
		val |= RTL819XD_SYS_CLK_PCIE0_DEV_RST_L;
		realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
		mdelay(1);
	} else {
		realtek_set_gpio_direction_output(PCIE1_GPIO, 0);
		mdelay(1);
		realtek_set_gpio_direction_output(PCIE1_GPIO, 1);
		mdelay(1);
	}

	realtek_pcie_phy_reset(rprc);
}

static void __init rtl819xd_pcie_only_one_reset(struct realtek_pcie_reset_controller *rprc0,
	struct realtek_pcie_reset_controller *rprc1, int pcie_xtal_40mhz)
{
	u32 val;
	int i;
	struct realtek_pcie_reset_controller *rprc;

//#ifdef CONFIG_RTL_819XD
	val = realtek_sys_read(REALTEK_SYS_REG_CLK_MANAGE);
	val |= (1<<12)|(1<<13)|(1<<16)|(1<<18)|(1<<19)|(1<<20);
	realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
//#endif

	/* Enable PCIe controllers */
	val = realtek_sys_read(REALTEK_SYS_REG_CLK_MANAGE);
	val &= ~RTL819XD_SYS_CLK_PCIE0_EN;
	realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
	val |= RTL819XD_SYS_CLK_PCIE0_EN;
	realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
	val = realtek_sys_read(REALTEK_SYS_REG_PCIE_PAD_CONTROL);
	val |= REALTEK_SYS_PCIE_PC_SWITCH_TO_RC;	//switch to rc
	realtek_sys_write(REALTEK_SYS_REG_PCIE_PAD_CONTROL, val);
	val = realtek_sys_read(REALTEK_SYS_REG_CLK_MANAGE);
	val &= ~RTL819XD_SYS_CLK_PCIE1_EN;
	realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
	val |= RTL819XD_SYS_CLK_PCIE1_EN;
	realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);

	realtek_pcie_mdio_reset(rprc0);
	realtek_pcie_mdio_reset(rprc1);
	mdelay(500);

	for (i=0; i<2; i++) {
		if (i==0)
			rprc = rprc0;
		else
			rprc = rprc1;

		realtek_pcie_mdio_write(rprc, 0x00, 0xD087);
		realtek_pcie_mdio_write(rprc, 0x01, 0x0003);
		realtek_pcie_mdio_write(rprc, 0x02, 0x4d18);

		if (pcie_xtal_40mhz) {
			realtek_pcie_mdio_write(rprc, 0x05, 0x0BCB);
			realtek_pcie_mdio_write(rprc, 0x06, 0xF148);
		} else {
			realtek_pcie_mdio_write(rprc, 0x06, 0xf848);
		}

		realtek_pcie_mdio_write(rprc, 0x07, 0x31ff);
//#ifdef CONFIG_RTL_819XD
		realtek_pcie_mdio_write(rprc, 0x08, 0x18d6);
/*#else
		realtek_pcie_mdio_write(rprc, 0x08, 0x18d7);
#endif*/
		realtek_pcie_mdio_write(rprc, 0x09, 0x539c);
		realtek_pcie_mdio_write(rprc, 0x0a, 0x20eb);
		realtek_pcie_mdio_write(rprc, 0x0d, 0x1766);
		realtek_pcie_mdio_write(rprc, 0x0b, 0x0511);
		realtek_pcie_mdio_write(rprc, 0x0f, 0x0a00);
		realtek_pcie_mdio_write(rprc, 0x19, 0xFCE0);
		realtek_pcie_mdio_write(rprc, 0x1a, 0x7e4f);
		realtek_pcie_mdio_write(rprc, 0x1b, 0xFC01);
		realtek_pcie_mdio_write(rprc, 0x1e, 0xC280);
	}

	/* Reset PCIe devices */
	val = realtek_sys_read(REALTEK_SYS_REG_CLK_MANAGE);
	val &= ~RTL819XD_SYS_CLK_PCIE0_DEV_RST_L;
	realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
	mdelay(500);
	val |= RTL819XD_SYS_CLK_PCIE0_DEV_RST_L;
	realtek_sys_write(REALTEK_SYS_REG_CLK_MANAGE, val);
	mdelay(500);

	realtek_pcie_phy_reset(rprc0);
	realtek_pcie_phy_reset(rprc1);
}

int pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	pr_info("pcibios_map_irq: %d %d %d\n",dev->bus->number, slot, pin);
	switch (dev->bus->number) {
	case 0:
		if (soc_is_rtl8196c())
			return REALTEK_SOC_IRQ(5);
		else if (soc_is_rtl819xd())
			return RTL819XD_INTCTL_RS_PCIE;
		break;
	case 1:
		if (soc_is_rtl819xd())
			return RTL819XD_INTCTL_RS_PCIE2;
		break;
	}

	return -1;
}

int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}

static void __init __realtek_pci_register(int n)
{
	struct platform_device *pdev;

	switch (n) {
	case 0:
		pdev = platform_device_register_simple("realtek-pci", 0,
					       realtek_pcie0_resource, ARRAY_SIZE(realtek_pcie0_resource));
		break;
	case 1:
		pdev = platform_device_register_simple("realtek-pci", 1,
					       realtek_pcie1_resource, ARRAY_SIZE(realtek_pcie1_resource));
		break;
	default:
		return;
	}
}

void __init realtek_register_pci(void)
{
	struct realtek_pcie_reset_controller rprc0, rprc1;

	/* Reference: https://www.linux-mips.org/wiki/PCI_Subsystem#I.2FO_ports_in_PCI */
	//pr_info("ioport: %x %x\niomem: %x %x\n",ioport_resource.start, ioport_resource.end, iomem_resource.start, iomem_resource.end);
	//set_io_port_base(KSEG1);
	//ioport_resource.end = 0xffffffff;
	//pr_info("KSEG1: %x\n",KSEG1);

	rprc0.rc_ext_base = ioremap_nocache(REALTEK_PCIE0_RC_EXT_BASE, REALTEK_PCIE0_RC_EXT_SIZE);
	rprc0.rc_phy_reg = REALTEK_SYS_REG_PCIE0_PHY;

	rprc1.rc_ext_base = ioremap_nocache(REALTEK_PCIE1_RC_EXT_BASE, REALTEK_PCIE1_RC_EXT_SIZE);
	rprc1.rc_phy_reg = REALTEK_SYS_REG_PCIE1_PHY;

	if (soc_is_rtl8196c()) {
		rtl8196c_pcie_reset(&rprc0, 1);
		__realtek_pci_register(0);
	} else if (soc_is_rtl819xd()) {
		if (0) {//#ifdef CONFIG_RTL_DUAL_PCIESLOT_BIWLAN_D
			rtl819xd_pcie_only_one_reset(&rprc0, &rprc1, 1);
			__realtek_pci_register(1);
			__realtek_pci_register(0);
		} else {
			rtl819xd_pcie_reset(&rprc0, 1);
			rtl819xd_pcie_reset(&rprc1, 1);
			__realtek_pci_register(0);
			__realtek_pci_register(1);
		}
	} else
		BUG();

	iounmap(rprc0.rc_ext_base);
	iounmap(rprc1.rc_ext_base);
}
