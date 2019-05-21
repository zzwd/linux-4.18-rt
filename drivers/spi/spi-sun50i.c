/*
 * Copyright (C) 2012 - 2014 Allwinner Tech
 * Pan Nan <pannan@allwinnertech.com>
 *
 * Copyright (C) 2014 Maxime Ripard
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>


static int sun50i_spi_runtime_suspend(struct device *dev)
{

	return 0;
}

static int sun50i_spi_runtime_resume(struct device *dev)
{

	return 0;
}


static int sun50i_spi_probe(struct platform_device *pdev)
{

	printk(KERN_WARNING "sun50i spi probe success\n");
	return 0;
}

static int sun50i_spi_remove(struct platform_device *pdev)
{
	printk(KERN_WARNING "sun50i spi remove success\n");
	return 0;
}


static const struct of_device_id sun50i_spi_match[] = {
	{ .compatible = "allwinner,sun8i-h3-spi", },
	{}
};
MODULE_DEVICE_TABLE(of, sun50i_spi_match);

static const struct dev_pm_ops sun50i_spi_pm_ops = {
	.runtime_resume		= sun50i_spi_runtime_resume,
	.runtime_suspend	= sun50i_spi_runtime_suspend,
};

static struct platform_driver sun50i_spi_driver = {
	.probe	= sun50i_spi_probe,
	.remove	= sun50i_spi_remove,
	.driver	= {
		.name		= "sun50i-spi",
		.of_match_table	= sun50i_spi_match,
		.pm		= &sun50i_spi_pm_ops,
	},
};
module_platform_driver(sun50i_spi_driver);

MODULE_AUTHOR("zzw, email<1600284146@qq.com>");
MODULE_DESCRIPTION("Allwinner A64 SPI controller driver");
MODULE_LICENSE("GPL");
