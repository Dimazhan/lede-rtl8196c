/*
 *  TOTOLINK A3002RU board support
 *
 *  Copyright (C) 2018 Dimazhan <dimazhan@list.ru>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/mach-realtek/realtek.h>
#include <asm/mach-realtek/platform.h>

#include "machtypes.h"
#include "dev-gpio-buttons.h"
#include "dev-leds-gpio.h"
#include "dev-m25p80.h"
#include "dev-eth.h"
#include "gpio.h"

#define TL_GPIO_LED_USB				4 //BSP_GPIO_PIN_H7?
#define TL_GPIO_LED_PWR				6 //BSP_GPIO_PIN_A6

#define TL_GPIO_BTN_WPS				3 //BSP_GPIO_PIN_H6
#define TL_GPIO_BTN_RESET			5 //BSP_GPIO_PIN_E7 or BSP_GPIO_PIN_H5

#define TL_KEYS_POLL_INTERVAL		100	/* msecs */
#define TL_KEYS_DEBOUNCE_INTERVAL	(3 * TL_KEYS_POLL_INTERVAL)

static struct gpio_led tl_leds_gpio[] __initdata = {
	{
		.name		= "tl:orange:usb",
		.gpio		= TL_GPIO_LED_USB,
		.active_low	= 1,
	}, {
		.name		= "tl:orange:pwr",
		.gpio		= TL_GPIO_LED_PWR,
		.active_low	= 1,
	},
};

static struct gpio_keys_button tl_gpio_keys[] __initdata = {
	{
		.desc		= "reset",
		.type		= EV_KEY,
		.code		= KEY_RESTART,
		.debounce_interval = TL_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= TL_GPIO_BTN_RESET,
		.active_low	= 1,
	} , {
		.desc		= "wps",
		.type		= EV_KEY,
		.code		= KEY_WPS_BUTTON,
		.debounce_interval = TL_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= TL_GPIO_BTN_WPS,
		.active_low	= 1,
	}
};

static void __init a3002ru_init(void)
{
	realtek_register_leds_gpio(-1, ARRAY_SIZE(tl_leds_gpio),
				 tl_leds_gpio);
	realtek_register_gpio_keys_polled(-1, TL_KEYS_POLL_INTERVAL,
					ARRAY_SIZE(tl_gpio_keys),
					tl_gpio_keys);

	realtek_register_m25p80(NULL);
	realtek_register_eth();

	realtek_set_gpio_control(TL_GPIO_LED_USB, true);
	realtek_set_gpio_control(TL_GPIO_LED_PWR, true);
	realtek_set_gpio_control(TL_GPIO_BTN_WPS, true);
	realtek_set_gpio_control(TL_GPIO_BTN_RESET, true);

	/*realtek_set_gpio_mux(0xFFFFFFFF, 0);
	realtek_set_gpio_mux2(0xFFFFFFFF, 0);*/
	/*301e
	0011 0000 0001 1110
	0000 0000 0000 0110
	0
	0
	*/
	realtek_set_gpio_mux(
		/*RTL819XD_GPIO_MUX_UART0 |
		(RTL819XD_GPIO_MUX_PCIE_RST_MASK << RTL819XD_GPIO_MUX_PCIE_RST_SHIFT),
		(RTL8196C_GPIO_MUX_PORT0_MASK << RTL8196C_GPIO_MUX_PORT0_SHIFT) |
		(RTL8196C_GPIO_MUX_PORT4_MASK << RTL8196C_GPIO_MUX_PORT4_SHIFT) |*/
		(RTL819XD_GPIO_MUX_JTAG_CLEAR_MASK << RTL819XD_GPIO_MUX_JTAG_SHIFT),
		/*(RTL8196C_GPIO_MUX_PORT2_MASK << RTL8196C_GPIO_MUX_PORT2_SHIFT) |
		(RTL8196C_GPIO_MUX_PORT3_MASK << RTL8196C_GPIO_MUX_PORT3_SHIFT) |
		(RTL8196C_GPIO_MUX_RESET_MASK << RTL8196C_GPIO_MUX_RESET_SHIFT) |
		(RTL8196C_GPIO_MUX_GPIOC0_MASK << RTL8196C_GPIO_MUX_GPIOC0_SHIFT) |*/
		(RTL819XD_GPIO_MUX_JTAG_SET_MASK << RTL819XD_GPIO_MUX_JTAG_SHIFT));
}

MIPS_MACHINE(REALTEK_MACH_A3002RU, "A3002RU", "TOTOLINK A3002RU",
	     a3002ru_init);
