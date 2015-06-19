/*
 * Driver for Atmel Flexcom
 *
 * Copyright (C) 2015 Atmel Corporation
 *
 * Author: Cyrille Pitchen <cyrille.pitchen@atmel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>

#define FX_MR		0x0
#define FX_RHR		0x10
#define FX_THR		0x20
#define FX_VERSION	0xfc

#define FX_MR_NO_COM	0
#define FX_MR_USART	1
#define FX_MR_SPI	2
#define FX_MR_TWI	3


static int atmel_flexcom_probe(struct platform_device *pdev)
{
	struct clk *clk;
	struct resource *res;
	unsigned char __iomem *map;
	unsigned int version, mr;
	const char *mode;
	int err;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	map = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(map))
		return PTR_ERR(map);

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	err = of_property_read_string(pdev->dev.of_node,
				      "atmel,flexcom-mode", &mode);
	if (err)
		return err;

	if (!strcmp(mode, "usart"))
		mr = FX_MR_USART;
	else if (!strcmp(mode, "spi"))
		mr = FX_MR_SPI;
	else if (!strcmp(mode, "twi") || !strcmp(mode, "i2c"))
		mr = FX_MR_TWI;
	else
		return -EINVAL;

	clk_prepare_enable(clk);
	version = readl(map + FX_VERSION);
	writel(mr, map + FX_MR);
	clk_disable_unprepare(clk);

	dev_info(&pdev->dev, "version: %#x, mode: %s\n", version, mode);

	return of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
}

static const struct of_device_id atmel_flexcom_of_match[] = {
	{ .compatible = "atmel,sama5d2-flexcom" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, atmel_flexcom_of_match);

static struct platform_driver atmel_flexcom_driver = {
	.driver		= {
		.name	= "atmel_flexcom",
		.of_match_table = atmel_flexcom_of_match,
	},
	.probe		= atmel_flexcom_probe,
};

module_platform_driver(atmel_flexcom_driver);

MODULE_AUTHOR("Cyrille Pitchen <cyrille.pitchen@atmel.com>");
MODULE_DESCRIPTION("Atmel Flexcom MFD driver");
MODULE_LICENSE("GPL");
