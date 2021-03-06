/*
 * Copyright (C) 2013-2014 Alexander Shiyan <shc_work@mail.ru>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <init.h>
#include <common.h>
#include <malloc.h>
#include <linux/err.h>
#include <linux/basic_mmio_gpio.h>

static int clps711x_gpio_probe(struct device_d *dev)
{
	int err, id = dev->id;
	void __iomem *dat, *dir = NULL, *dir_inv = NULL;
	struct bgpio_chip *bgc;

	if (dev->device_node)
		id = of_alias_get_id(dev->device_node, "gpio");

	if (id < 0 || id > 4)
		return -ENODEV;

	dat = dev_request_mem_region(dev, 0);
	if (IS_ERR(dat))
		return PTR_ERR(dat);

	switch (id) {
	case 3:
		dir_inv = dev_request_mem_region(dev, 1);
		if (IS_ERR(dir_inv))
			return PTR_ERR(dir_inv);
		break;
	default:
		dir = dev_request_mem_region(dev, 1);
		if (IS_ERR(dir))
			return PTR_ERR(dir);
		break;
	}

	bgc = xzalloc(sizeof(struct bgpio_chip));
	if (!bgc)
		return -ENOMEM;

	err = bgpio_init(bgc, dev, 1, dat, NULL, NULL, dir, dir_inv, 0);
	if (err)
		goto out_err;

	bgc->gc.base = id * 8;
	switch (id) {
	case 4:
		bgc->gc.ngpio = 3;
		break;
	default:
		break;
	}

	err = gpiochip_add(&bgc->gc);

out_err:
	if (err)
		free(bgc);

	return err;
}

static struct of_device_id __maybe_unused clps711x_gpio_dt_ids[] = {
	{ .compatible = "cirrus,clps711x-gpio", },
};

static struct driver_d clps711x_gpio_driver = {
	.name		= "clps711x-gpio",
	.probe		= clps711x_gpio_probe,
	.of_compatible	= DRV_OF_COMPAT(clps711x_gpio_dt_ids),
};

static __init int clps711x_gpio_register(void)
{
	return platform_driver_register(&clps711x_gpio_driver);
}
coredevice_initcall(clps711x_gpio_register);
