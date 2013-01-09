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

#define NFC_TIME_OUT_MS		100

static void pmecc_enable(struct atmel_nand_host *host, enum pmecc_op op);
static int atmel_nand_dma_op(struct mtd_info *mtd, void *buf, int len,
			       int is_read);

static int nfc_wait_interrupt(struct atmel_nand_host *host, u32 flag)
{
	unsigned long timeout;
	init_completion(&host->comp_nfc);

	/* Enable interrupt that need to wait for */
	nfc_writel(host->nfc.hsmc_regs, IER, flag);

	timeout = wait_for_completion_timeout(&host->comp_nfc,
			msecs_to_jiffies(NFC_TIME_OUT_MS));
	if (timeout == 0) {
		dev_err(host->dev, "interrupt time out???? flag is 0x%08x\n", flag);
		return -ETIMEDOUT;
	}

	return 0;
}

static int nfc_send_command(struct atmel_nand_host *host,
	unsigned int cmd, unsigned int addr, unsigned char cycle0)
{
	unsigned long timeout;
	dev_dbg(host->dev,
		"nfc_cmd: 0x%08x, addr1234: 0x%08x, cycle0: 0x%02x\n",
		cmd, addr, cycle0);

	timeout = jiffies + msecs_to_jiffies(NFC_TIME_OUT_MS);
	while (nfc_cmd_readl(NFCADDR_CMD_NFCBUSY, host->nfc.base_cmd_regs)
			& NFCADDR_CMD_NFCBUSY) {
		if (time_after(jiffies, timeout)) {
			dev_err(host->dev,
				"Time out to wait CMD_NFCBUSY ready!\n");
			break;
		}
	}
	nfc_writel(host->nfc.hsmc_regs, CYCLE0, cycle0);
	nfc_cmd_addr1234_writel(cmd, addr, host->nfc.base_cmd_regs);
	return nfc_wait_interrupt(host, ATMEL_HSMC_NFC_CMD_DONE);
}

static int nfc_device_ready(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct atmel_nand_host *host = nand_chip->priv;
	if (!nfc_wait_interrupt(host, ATMEL_HSMC_NFC_RB_EDGE))
		return 1;
	return 0;
}

static void nfc_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct atmel_nand_host *host = nand_chip->priv;

	if (chip == -1)
		nfc_writel(host->nfc.hsmc_regs, CTRL, ATMEL_HSMC_NFC_DISABLE);
	else
		nfc_writel(host->nfc.hsmc_regs, CTRL, ATMEL_HSMC_NFC_ENABLE);
}

static void* get_bank_sram_base(struct atmel_nand_host *host)
{
	if (nfc_readl(host->nfc.hsmc_regs, BANK) & ATMEL_HSMC_NFC_BANK1)
		return host->nfc.sram_bank1;
	else
		return host->nfc.sram_bank0;
}

static dma_addr_t get_bank_sram_phys(struct atmel_nand_host *host)
{
	if (nfc_readl(host->nfc.hsmc_regs, BANK) & ATMEL_HSMC_NFC_BANK1)
		return host->nfc.sram_bank1_phys;
	else
		return host->nfc.sram_bank0_phys;
}

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

	nfc->sram_bank1 = nfc->sram_bank0 + 0x1200;

	nfc->sram_bank0_phys = (dma_addr_t)nfc_sram->start;
	nfc->sram_bank1_phys = nfc->sram_bank0_phys + 0x1200;
	return 0;
}

static int make_addr(struct mtd_info *mtd, int column, int page_addr,
		unsigned int *addr1234, unsigned int *cycle0)
{
	struct nand_chip *chip = mtd->priv;

	int acycle = 0;
	unsigned char addr_bytes[8];
	int index = 0, bit_shift;

	BUG_ON(addr1234 == NULL || cycle0 == NULL);

	*cycle0 = 0;
	*addr1234 = 0;

	if (column != -1) {
		if (chip->options & NAND_BUSWIDTH_16)
			column >>= 1;
		addr_bytes[acycle++] = column & 0xff;
		if (mtd->writesize > 512)
			addr_bytes[acycle++] = (column >> 8) & 0xff;
	}

	if (page_addr != -1) {
		addr_bytes[acycle++] = page_addr & 0xff;
		addr_bytes[acycle++] = (page_addr >> 8) & 0xff;
		if (chip->chipsize > (128 << 20))
			addr_bytes[acycle++] = (page_addr >> 16) & 0xff;
	}

	if (acycle > 4)
		*cycle0 = addr_bytes[index++];

	for (bit_shift = 0; index < acycle; bit_shift += 8)
		*addr1234 += addr_bytes[index++] << bit_shift;

	return acycle << 19;	/* return acycle in cmd register */
}

static void nfc_nand_command(struct mtd_info *mtd, unsigned int command,
				int column, int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct atmel_nand_host *host = chip->priv;
	unsigned long timeout;
	unsigned int nfc_addr_cmd = 0;

	unsigned int cmd1 = command << 2;

	/* Set default settings: no cmd2, no addr cycle. read from nand */
	unsigned int cmd2 = 0;
	unsigned int vcmd2 = 0;
	int acycle = NFCADDR_CMD_ACYCLE_NONE;
	int csid = NFCADDR_CMD_CSID_3;
	int dataen = NFCADDR_CMD_DATADIS;
	int nfcwr = NFCADDR_CMD_NFCRD;
	unsigned int addr1234 = 0;
	unsigned int cycle0 = 0;
	bool do_addr = true;
	host->nfc.data_in_sram = NULL;

	dev_dbg(host->dev, "%s: cmd = 0x%02x, col = 0x%08x, page = 0x%08x\n",
	     __func__, command, column, page_addr);

	switch (command) {
	case NAND_CMD_RESET:
		nfc_addr_cmd = cmd1 | acycle | csid | dataen | nfcwr;
		nfc_send_command(host, nfc_addr_cmd, addr1234, cycle0);
		udelay(chip->chip_delay);

		nfc_nand_command(mtd, NAND_CMD_STATUS, -1, -1);
		timeout = jiffies + msecs_to_jiffies(NFC_TIME_OUT_MS);
		while (!(chip->read_byte(mtd) & NAND_STATUS_READY)) {
			if (time_after(jiffies, timeout)) {
				dev_err(host->dev,
					"Time out to wait status ready!\n");
				break;
			}
		}
		return;
	case NAND_CMD_STATUS:
		do_addr = false;
		break;
	case NAND_CMD_PARAM:
	case NAND_CMD_READID:
		do_addr = false;
		acycle = NFCADDR_CMD_ACYCLE_1;
		if (column != -1)
			addr1234 = column;
		break;
	case NAND_CMD_RNDOUT:
		cmd2 = NAND_CMD_RNDOUTSTART << 10;
		vcmd2 = NFCADDR_CMD_VCMD2;
		break;
	case NAND_CMD_READ0:
	case NAND_CMD_READOOB:
		if (command == NAND_CMD_READOOB) {
			column += mtd->writesize;
			command = NAND_CMD_READ0; /* only READ0 is valid */
			cmd1 = command << 2;
		}
		if (host->use_nfc_sram) {
			/* Enable Data transfer to sram */
			dataen = NFCADDR_CMD_DATAEN;

			/* Need enable PMECC now, since NFC will transfer
			 * data in bus after sending nfc read command.
			 */
			if (chip->ecc.mode == NAND_ECC_HW && host->has_pmecc)
				pmecc_enable(host, PMECC_READ);
		}

		cmd2 = NAND_CMD_READSTART << 10;
		vcmd2 = NFCADDR_CMD_VCMD2;
		break;
	/* For prgramming command, the cmd need set to write enable */
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
		nfcwr = NFCADDR_CMD_NFCWR;
		if (host->nfc.will_write_sram && command == NAND_CMD_SEQIN)
			dataen = NFCADDR_CMD_DATAEN;
		break;
	default:
		break;
	}

	if (do_addr)
		acycle = make_addr(mtd, column, page_addr, &addr1234, &cycle0);
	nfc_addr_cmd = cmd1 | cmd2 | vcmd2 | acycle | csid | dataen | nfcwr;
	nfc_send_command(host, nfc_addr_cmd, addr1234, cycle0);

	if (dataen == NFCADDR_CMD_DATAEN)
		if (nfc_wait_interrupt(host, ATMEL_HSMC_NFC_XFR_DONE))
			printk("something wrong, No XFR_DONE interrupt comes.\n");

	/*
	 * Program and erase have their own busy handlers status, sequential
	 * in, and deplete1 need no delay.
	 */
	switch (command) {
	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
	case NAND_CMD_DEPLETE1:
	case NAND_CMD_RNDOUT:
	case NAND_CMD_SEQIN:
	case NAND_CMD_READID:
		return;

	case NAND_CMD_STATUS_ERROR:
	case NAND_CMD_STATUS_ERROR0:
	case NAND_CMD_STATUS_ERROR1:
	case NAND_CMD_STATUS_ERROR2:
	case NAND_CMD_STATUS_ERROR3:
		/* Read error status commands require only a short delay */
		udelay(chip->chip_delay);
		return;

	case NAND_CMD_READ0:
		if (dataen == NFCADDR_CMD_DATAEN) {
			host->nfc.data_in_sram = get_bank_sram_base(host);
			return;
		}
		/* fall through */
	default:
		nfc_wait_interrupt(host, ATMEL_HSMC_NFC_RB_EDGE);
	}
}

static int nfc_sram_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			   const uint8_t *buf, int oob_required, int page,
			   int cached, int raw)
{
	int cfg, len;
	int status = 0;
	struct atmel_nand_host *host = chip->priv;
	char *sram = get_bank_sram_base(host);

	cfg = nfc_readl(host->nfc.hsmc_regs, CFG);
	len = mtd->writesize;

	if (unlikely(raw)) {
		len += mtd->oobsize;
		nfc_writel(host->nfc.hsmc_regs, CFG, cfg | ATMEL_HSMC_WSPARE);
	} else
		nfc_writel(host->nfc.hsmc_regs, CFG, cfg & ~ATMEL_HSMC_WSPARE);

	/* Copy page data to sram that will write to nand via NFC */
	if (use_dma) {
		if (atmel_nand_dma_op(mtd, (void *)buf, len, 0) != 0)
			/* Fall back to use cpu copy */
			memcpy(sram, buf, len);
	} else {
		memcpy(sram, buf, len);
	}

	if (chip->ecc.mode == NAND_ECC_HW && host->has_pmecc)
		/*
		 * When use NFC sram, need set up PMECC before send
		 * NAND_CMD_SEQIN command. Since when the nand command
		 * is sent, nfc will do transfer from sram and nand.
		 */
		pmecc_enable(host, PMECC_WRITE);

	host->nfc.will_write_sram = true;
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);
	host->nfc.will_write_sram = false;

	if (likely(!raw))
		/* Need to write ecc into oob */
		status = chip->ecc.write_page(mtd, chip, buf, oob_required);

	if (status < 0)
		return status;

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	status = chip->waitfunc(mtd, chip);

	if ((status & NAND_STATUS_FAIL) && (chip->errstat))
		status = chip->errstat(mtd, chip, FL_WRITING, status, page);

	if (status & NAND_STATUS_FAIL)
		return -EIO;

	return 0;
}

static int atmel_nfc_sram_init(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct atmel_nand_host *host = chip->priv;
	int res = 0;

	/* Initialize the NFC CFG register */
	unsigned int cfg_nfc = 0;

	/* set page size and oob layout */
	switch (mtd->writesize) {
	case 512:
		cfg_nfc = ATMEL_HSMC_PAGESIZE_512;
		break;
	case 1024:
		cfg_nfc = ATMEL_HSMC_PAGESIZE_1024;
		break;
	case 2048:
		cfg_nfc = ATMEL_HSMC_PAGESIZE_2048;
		break;
	case 4096:
		cfg_nfc = ATMEL_HSMC_PAGESIZE_4096;
		break;
	case 8192:
		cfg_nfc = ATMEL_HSMC_PAGESIZE_8192;
		break;
	default:
		printk(KERN_ERR "Unsupported page size for NFC.\n");
		res = -ENXIO;
		return res;
	}

	cfg_nfc |= ((mtd->oobsize / 4) - 1) << 24;
	cfg_nfc |= ATMEL_HSMC_RSPARE |
			ATMEL_HSMC_NFC_DTOCYC | ATMEL_HSMC_NFC_DTOMUL;

	nfc_writel(host->nfc.hsmc_regs, CFG, cfg_nfc);

	host->nfc.will_write_sram = false;

	dev_info(host->dev, "Using NFC Sram\n");

	/* Use Write page with NFC SRAM only for PMECC or ECC NONE. */
	if ((chip->ecc.mode == NAND_ECC_HW && host->has_pmecc) ||
			chip->ecc.mode == NAND_ECC_NONE)
		chip->write_page = nfc_sram_write_page;

	return 0;
}
