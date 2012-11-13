/*
 * Atmel Nand flash Controller (NFC) - System peripherals regsters.
 * Based on AT91SAMA5D3 datasheet.
 *
 * Copyright (C) 2012 Atmel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include "atmel_nand_nfc.h"
#include <linux/delay.h>

static int atmel_nfc_init(struct platform_device *pdev, struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct atmel_nand_host *host = nand_chip->priv;
	struct atmel_nfc *nfc = &host->nfc;
	struct resource *nfc_cmd_regs, *nfc_hsmc_regs, *nfc_sram;

	dev_info(host->dev, "Using NFC\n");
	nfc_cmd_regs = platform_get_resource(pdev, IORESOURCE_MEM, 4);
	nfc_hsmc_regs = platform_get_resource(pdev, IORESOURCE_MEM, 5);
	nfc_sram = platform_get_resource(pdev, IORESOURCE_MEM, 6);

	if (nfc_cmd_regs && nfc_hsmc_regs && nfc_sram) {
		nfc->base_cmd_regs = devm_request_and_ioremap(&pdev->dev, nfc_cmd_regs);
		nfc->hsmc_regs = devm_request_and_ioremap(&pdev->dev, nfc_hsmc_regs);
		nfc->sram_bank0 = devm_request_and_ioremap(&pdev->dev, nfc_sram);
	}

	if (!nfc->base_cmd_regs || !nfc->hsmc_regs || !nfc->sram_bank0) {
		dev_err(host->dev,
			"Can not get I/O resource for Nand Flash Controller!\n");
		return -EIO;
	}

	return 0;
}

