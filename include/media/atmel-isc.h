/*
 * Register definitions for the Atmel Image Sensor Controller (ISC).
 *
 * Copyright (C) 2015 Atmel Corporation
 * Josh Wu, <josh.wu@atmel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ATMEL_ISC_H__
#define __ATMEL_ISC_H__

#include <linux/types.h>

/* register offsets */
#define ISC_CTRLEN				0x0000
#define ISC_CTRLDIS				0x0004
#define ISC_CTRLSR				0x0008
#define ISC_PFE_CFG0				0x000c
#define ISC_PFE_CFG1				0x0010
#define ISC_PFE_CFG2				0x0014
#define ISC_CLKEN				0x0018
#define ISC_CLKDIS				0x001C
#define ISC_CLKSR				0x0020
#define ISC_CLKCFG				0x0024
#define ISC_INTEN				0x0028
#define ISC_INTDIS				0x002C
#define ISC_INTMASK				0x0030
#define ISC_INTSR				0x0034

#define ISC_CFA_CTRL				0x0070
#define ISC_CFA_CFG				0x0074

#define ISC_GAM_CTRL				0x0094
#define ISC_GAM_BENTRY0				0x0098
#define ISC_GAM_GENTRY0				(ISC_GAM_BENTRY0 + 4 * 64)
#define ISC_GAM_RENTRY0				(ISC_GAM_GENTRY0 + 4 * 64)

#define ISC_RLP_CFG				0x03d0

#define ISC_DCFG				0x03e0
#define ISC_DCTRL				0x03e4
#define ISC_DNDA				0x03e8
#define ISC_DAD0				0x03ec
#define ISC_DST0				0x03f0
#define ISC_DAD1				0x03f4
#define ISC_DST1				0x03f8
#define ISC_DAD2				0x03fc
#define ISC_DST2				0x0400

/* Bitfields in ISC_CTRLEN */
#define ISC_CTRLEN_CAPTURE			(1 << 0)
#define ISC_CTRLEN_UPPRO			(1 << 1)	/* update profile */
#define ISC_CTRLEN_HISREQ			(1 << 2)	/* update histogram table */
#define ISC_CTRLEN_HISCLR			(1 << 3)	/* clear histogram table */
/* Bitfields in ISC_CTRLDIS */
#define ISC_CTRLDIS_CAPTURE			(1 << 0)
#define ISC_CTRLDIS_SWRST			(1 << 8)
/* Bitfields in ISC_CTRLSR */
#define ISC_CTRLSR_CAPTURE			(1 << 0)	/* capture pending */
#define ISC_CTRLSR_UPPRO			(1 << 1)	/* update profile pending */
#define ISC_CTRLSR_HISREQ			(1 << 2)	/* update histogram table pending */
#define ISC_CTRLSR_FIELD			(1 << 4)	/* field status */
#define ISC_CTRLSR_SIP				(1 << 31)	/* Synchronization In Progress */

/* Bitfields in ISC_PFE */
#define ISC_PFE_HSYNC_ACTIVE_HIGH		(0 << 0)
#define ISC_PFE_HSYNC_ACTIVE_LOW		(1 << 0)
#define ISC_PFE_VSYNC_ACTIVE_HIGH		(0 << 1)
#define ISC_PFE_VSYNC_ACTIVE_LOW		(1 << 1)
#define ISC_PFE_PIX_CLK_RISING_EDGE		(0 << 2)
#define ISC_PFE_PIX_CLK_FALLING_EDGE		(1 << 2)
#define ISC_PFE_FPOL				(1 << 3)
#define ISC_PFE_MODE_PROGRESSIVE		(0 << 4)
#define ISC_PFE_CONT_SINGLE_SHOT		(0 << 7)
#define ISC_PFE_CONT_VIDEO			(1 << 7)
#define ISC_PFE_GATED_PIX_CLK			(1 << 8)
#define ISC_PFE_COL_CROP			(1 << 12)
#define ISC_PFE_ROW_CROP			(1 << 13)

#define ISC_PFE_BPS_12_BIT			(0 << 28)
#define ISC_PFE_BPS_11_BIT			(1 << 28)
#define ISC_PFE_BPS_10_BIT			(2 << 28)
#define ISC_PFE_BPS_9_BIT			(3 << 28)
#define ISC_PFE_BPS_8_BIT			(4 << 28)

/* Bitfields in ISC_CLKEN/CLKDIS/CLKSR */
#define ISC_CLK_ISP				(1 << 0)
#define ISC_CLK_MASTER				(1 << 1)
#define ISC_CLK_ISP_RESET			(1 << 8)
#define ISC_CLK_MASTER_RESET			(1 << 9)
#define ISC_CLK_SIP				(1 << 31)

/* Bitfields in ISC_CLKCFG */
#define ISC_CLKCFG_ICDIV_OFFSET			(0)
#define ISC_CLKCFG_ICDIV_MASK			(0xFF << ISC_CLKCFG_ICDIV_OFFSET)
#define ISC_CLKCFG_ICDIV(n)			((n) << ISC_CLKCFG_ICDIV_OFFSET)
#define ISC_CLKCFG_ISP_SEL_HCLOCK		(0 << 8)
#define ISC_CLKCFG_ISP_SEL_GCK			(1 << 8)
#define ISC_CLKCFG_MCDIV_OFFSET			(16)
#define ISC_CLKCFG_MCDIV_MASK			(0xFF << ISC_CLKCFG_MCDIV_OFFSET)
#define ISC_CLKCFG_MCDIV(n)			((n) << ISC_CLKCFG_MCDIV_OFFSET)
#define ISC_CLKCFG_MASTER_SEL_HCLOCK		(0 << 24)
#define ISC_CLKCFG_MASTER_SEL_GCK		(1 << 24)
#define ISC_CLKCFG_MASTER_SEL_480M		(2 << 24)

/* Bitfields in ISC_GAM_CTRL */
#define ISC_GAM_CTRL_ENABLE			BIT(0)
#define ISC_GAM_CTRL_B_ENABLE			BIT(1)
#define ISC_GAM_CTRL_G_ENABLE			BIT(2)
#define ISC_GAM_CTRL_R_ENABLE			BIT(3)
#define ISC_GAM_CTRL_ENABLE_ALL_CHAN		(ISC_GAM_CTRL_B_ENABLE | \
						 ISC_GAM_CTRL_G_ENABLE | \
						 ISC_GAM_CTRL_R_ENABLE)

/* Bitfields in ISC_RLP_CFG */
#define ISC_RLP_CFG_MODE_DAT8			(0 << 0)
#define ISC_RLP_CFG_MODE_DAT9			(1 << 0)
#define ISC_RLP_CFG_MODE_DAT10			(2 << 0)
#define ISC_RLP_CFG_MODE_DAT11			(3 << 0)
#define ISC_RLP_CFG_MODE_DAT12			(4 << 0)
#define ISC_RLP_CFG_MODE_DAT_Y8			(5 << 0)
#define ISC_RLP_CFG_MODE_DAT_Y10		(6 << 0)
#define ISC_RLP_CFG_MODE_ARGB444		(7 << 0)
#define ISC_RLP_CFG_MODE_ARGB555		(8 << 0)
#define ISC_RLP_CFG_MODE_RGB565			(9 << 0)
#define ISC_RLP_CFG_MODE_ARGB32			(10 << 0)
#define ISC_RLP_CFG_MODE_YCC			(11 << 0)
#define ISC_RLP_CFG_MODE_YCC_LIMITED		(12 << 0)
#define ISC_RLP_CFG_ALPHA_OFFSET		(8)
#define ISC_RLP_CFG_ALPHA_MASK			(0xFF << ISC_RLP_CFG_ALPHA_OFFSET)

/* Bitfields in ISC_INTEN/INTDIS/INTMASK/INTSR */
#define ISC_INT_VSYNC				(1 << 0)
#define ISC_INT_HSYNC				(1 << 1)
#define ISC_INT_SWRST_COMPLETE			(1 << 4)
#define ISC_INT_DISABLE_COMPLETE		(1 << 5)
#define ISC_INT_DMA_DONE			(1 << 8)
#define ISC_INT_DMA_LIST_DONE			(1 << 9)
#define ISC_INT_HISTOGRAM_DONE			(1 << 12)
#define ISC_INT_HISTOGRAM_CLEAR			(1 << 13)

/* Bitfields in ISC_DCFG (DMA config)*/
#define ISC_DCFG_IMODE_PACKED8			(0 << 0)
#define ISC_DCFG_IMODE_PACKED16			(1 << 0)
#define ISC_DCFG_IMODE_PACKED32			(2 << 0)
#define ISC_DCFG_IMODE_YC422SP			(3 << 0)
#define ISC_DCFG_IMODE_YC422P			(4 << 0)
#define ISC_DCFG_IMODE_YC420SP			(5 << 0)
#define ISC_DCFG_IMODE_YC420P			(6 << 0)
#define		ISC_DCFG_MBSIZE_SINGLE		(0)
#define		ISC_DCFG_MBSIZE_BEATS_4		(1)
#define		ISC_DCFG_MBSIZE_BEATS_8		(2)
#define		ISC_DCFG_MBSIZE_BEATS_16	(3)
#define		ISC_DCFG_MBSIZE_CHAN_Y_OFFSET	(4)
#define		ISC_DCFG_MBSIZE_CHAN_C_OFFSET	(8)
#define ISC_DCFG_YMBSIZE(x)	(x << ISC_DCFG_MBSIZE_CHAN_Y_OFFSET)
#define ISC_DCFG_CMBSIZE(x)	(x << ISC_DCFG_MBSIZE_CHAN_C_OFFSET)

/* Bitfields in ISC_DCTRL (DMA control)*/
#define ISC_DCTRL_DESC_DISABLE			(0 << 0)
#define ISC_DCTRL_DESC_ENABLE			(1 << 0)
#define ISC_DCTRL_DVIEW_PACKED			(0 << 1)
#define ISC_DCTRL_DVIEW_SEMIPLANAR		(1 << 1)
#define ISC_DCTRL_DVIEW_PLANAR			(2 << 1)
#define ISC_DCTRL_DMA_DONE_INT_DISABLE		(0 << 4)
#define ISC_DCTRL_DMA_DONE_INT_ENABLE		(1 << 4)
#define ISC_DCTRL_WRITE_BACK_DISABLE		(0 << 5)
#define ISC_DCTRL_WRITE_BACK_ENABLE		(1 << 5)

/* Definition for isc_platform_data */
#define ISC_DATAWIDTH_8				0x01
#define ISC_DATAWIDTH_10			0x02

struct v4l2_async_subdev;

struct isc_platform_data {
	u8 has_emb_sync;
	u8 emb_crc_sync;
	u8 hsync_act_low;
	u8 vsync_act_low;
	u8 pclk_act_falling;
	u8 full_mode;
	u32 data_width_flags;
	/* Using for ISI_CFG1 */
	u32 frate;
	/* Using for ISI_MCK */
	u32 mck_hz;
	struct v4l2_async_subdev **asd;	/* Flat array, arranged in groups */
	int *asd_sizes;		/* 0-terminated array of asd group sizes */
};

#endif /* __ATMEL_ISC_H__ */
