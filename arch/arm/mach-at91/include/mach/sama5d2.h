/*
 * Chip-specific header file for the SAMA5D2 family
 *
 *  Copyright (C) 2015 Atmel,
 *                2015 Ludovic Desroches <ludovic.desroches@atmel.com>
 *
 * Common definitions.
 * Based on SAMA5D2 datasheet.
 *
 * Licensed under GPLv2 or later.
 */

#ifndef SAMA5D2_H
#define SAMA5D2_H

/*
 * Peripheral identifiers/interrupts.
 */
#define SAMA5D2_ID_WDT		 4	/* Watchdog Timer Interrupt */
#define SAMA5D2_ID_GMAC0	 5
#define SAMA5D2_ID_DMA0		 6
#define SAMA5D2_ID_AES		 9
#define SAMA5D2_ID_AESOTF	10
#define SAMA5D2_ID_SHA		12
#define SAMA5D2_ID_DDR		13
#define SAMA5D2_ID_HMATRIX64	15
#define SAMA5D2_ID_FLEXCOM0	19
#define SAMA5D2_ID_FLEXCOM1	20
#define SAMA5D2_ID_UART0	24
#define SAMA5D2_ID_SDHC0	31
#define SAMA5D2_ID_SPI0		33
#define SAMA5D2_ID_TC0		35
#define SAMA5D2_ID_ISI		46
#define SAMA5D2_ID_QSPI0	52
#define SAMA5D2_ID_I2SC0	54
#define SAMA5D2_ID_L2CC		63
#define SAMA5D2_ID_GMACQ1	66
#define SAMA5D2_ID_GMACQ2	67

/*
 * Physical base addresses.
 */
#define SAMA5D2_BASE_PMC	0xf0014000
#define SAMA5D2_BASE_USART0	0xf801c000
#define SAMA5D2_BASE_UART1	0xf8020000
#define SAMA5D2_BASE_SYS	0xf8048000

/*
 * Internal Memory
 */
#define SAMA5D2_SRAM_BASE	0x00200000	/* Internal SRAM base address */
#define SAMA5D2_SRAM_SIZE	(64 * SZ_1K)	/* Internal SRAM size (128Kb) */

#endif
