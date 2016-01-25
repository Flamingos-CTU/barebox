/*
 * (C) Copyright 2016 Michal Vokáč <vokac.m@gmail.com>
 * (C) Copyright 2008
 * Texas Instruments, <www.ti.com>
 * Raghavendra KH <r-khandenahally@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <common.h>
#include <console.h>
#include <init.h>
#include <driver.h>
#include <linux/sizes.h>
#include <io.h>
#include <bbu.h>
#include <filetype.h>
#include <ns16550.h>
#include <envfs.h>
#include <asm/armlinux.h>
#include <generated/mach-types.h>
#include <mach/gpmc.h>
#include <mach/gpmc_nand.h>
#include <mach/ehci.h>
#include <mach/omap3-devices.h>
#include <i2c/i2c.h>
#include <linux/err.h>
#include <usb/ehci.h>
#include <asm/barebox-arm.h>
#include <gpio.h>
#include <printk.h>
#include "overo.h"

#ifdef CONFIG_DRIVER_SERIAL_NS16550
static int overo_console_init(void)
{
	if (barebox_arm_machine() != MACH_TYPE_OVERO)
		return 0;

	barebox_set_model("Gumstix Overo COM");
	barebox_set_hostname("overo");

	omap3_add_uart3();

	return 0;
}
console_initcall(overo_console_init);
#endif /* CONFIG_DRIVER_SERIAL_NS16550 */

#ifdef CONFIG_USB_EHCI_OMAP
static struct omap_hcd omap_ehci_pdata = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.phy_reset  = 1,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 147,
	.reset_gpio_port[2]  = -EINVAL
};

static struct ehci_platform_data ehci_pdata = {
	.flags = 0,
};
#endif /* CONFIG_USB_EHCI_OMAP */

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
	},
};

static struct gpmc_nand_platform_data nand_plat = {
	.device_width = 16,
	.ecc_mode = OMAP_ECC_HAMMING_CODE_HW_ROMCODE,
	.nand_cfg = &omap3_nand_cfg,
};

#ifdef CONFIG_DRIVER_NET_SMC911X
static struct gpmc_config smsc_cfg = {
	.cfg = {
		0x00001000,	/*CONF1 */
		0x00060700,	/*CONF2 */
		0x00020201,	/*CONF3 */
		0x06000700,	/*CONF4 */
		0x0006090A,	/*CONF5 */
		0x87030000,	/*CONF6 */
	},
	.base = SMC911X_BASE,
	/* GPMC address map as small as possible */
	.size = GPMC_SIZE_16M,
};

static void overo_net_init(void)
{
	gpmc_cs_config(5, &smsc_cfg);
}
#endif /* CONFIG_DRIVER_NET_SMC911X */

static int overo_board_revision(void)
{
	int revision;
	pr_info("board-revision: ");

	if (!gpio_request(112, "") &&
            !gpio_request(113, "") &&
            !gpio_request(115, "")) {

                gpio_direction_input(112);
                gpio_direction_input(113);
                gpio_direction_input(115);

                revision = gpio_get_value(115) << 2 |
                           gpio_get_value(113) << 1 |
                           gpio_get_value(112);

		pr_info("%d\n", revision);
        } else {
		pr_info("unable to acquire board revision GPIOs\n");
		revision = -1;
	}
	return revision;
}

static int overo_mem_init(void)
{
	if (barebox_arm_machine() != MACH_TYPE_OVERO)
		return 0;

	omap3_add_sram0();
	omap_add_ram0(SDRAM_SIZE);

	return 0;
}
mem_initcall(overo_mem_init);

static int overo_devices_init(void)
{
	if (barebox_arm_machine() != MACH_TYPE_OVERO)
		return 0;

	overo_board_revision();
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
	omap3_add_i2c1(NULL);

#ifdef CONFIG_USB_EHCI_OMAP
	if (ehci_omap_init(&omap_ehci_pdata) >= 0)
		omap3_add_ehci(&ehci_pdata);
#endif /* CONFIG_USB_EHCI_OMAP */
#ifdef CONFIG_OMAP_GPMC
	/* WP is made high and WAIT1 active Low */
	gpmc_generic_init(0x10);
#endif
	omap_add_gpmc_nand_device(&nand_plat);

	omap3_add_mmc1(NULL);

#ifdef CONFIG_DRIVER_NET_SMC911X
	overo_net_init();
	add_generic_device("smc911x", DEVICE_ID_DYNAMIC, NULL, SMC911X_BASE, SZ_4K,
			   IORESOURCE_MEM, NULL);
#endif

	armlinux_set_architecture(MACH_TYPE_OVERO);

	defaultenv_append_directory(env);

	return 0;
}
device_initcall(overo_devices_init);
