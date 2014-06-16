/*
 * Chip-specific header file for the SAMA5D4 family
 *
 *  Copyright (C) 2013 Atmel Corporation,
 *                     Nicolas Ferre <nicolas.ferre@atmel.com>
 *
 * Common definitions.
 * Based on SAMA5D4 datasheet.
 *
 * Licensed under GPLv2 or later.
 */

#ifndef SAMA5D4_H
#define SAMA5D4_H

/*
 * Peripheral identifiers/interrupts.
 */
#define SAMA5D4_ID_PIT		 3
#define SAMA5D4_ID_WDT		 4
#define SAMA5D4_ID_PIOD		 5
#define SAMA5D4_ID_USART0	 6
#define SAMA5D4_ID_USART1	 7
#define SAMA5D4_ID_DMA0		 8
#define SAMA5D4_ID_ICM		 9
#define SAMA5D4_ID_PKCC		10
#define SAMA5D4_ID_SCI		11
#define SAMA5D4_ID_AES		12
#define SAMA5D4_ID_AESB		13
#define SAMA5D4_ID_TDES		14
#define SAMA5D4_ID_SHA		15
#define SAMA5D4_ID_MPDDRC	16
#define SAMA5D4_ID_MATRIX1	17
#define SAMA5D4_ID_MATRIX0	18
#define SAMA5D4_ID_VDEC		19
#define SAMA5D4_ID_SECUMOD	20
#define SAMA5D4_ID_MSADCC	21
#define SAMA5D4_ID_HSMC		22
#define SAMA5D4_ID_PIOA		23
#define SAMA5D4_ID_PIOB		24
#define SAMA5D4_ID_PIOC		25
#define SAMA5D4_ID_PIOE		26
#define SAMA5D4_ID_UART0	27
#define SAMA5D4_ID_UART1	28
#define SAMA5D4_ID_USART2	29
#define SAMA5D4_ID_USART3	30
#define SAMA5D4_ID_USART4	31
#define SAMA5D4_ID_TWI0		32
#define SAMA5D4_ID_TWI1		33
#define SAMA5D4_ID_TWI2		34
#define SAMA5D4_ID_HSMCI0	35
#define SAMA5D4_ID_HSMCI1	36
#define SAMA5D4_ID_SPI0		37
#define SAMA5D4_ID_SPI1		38
#define SAMA5D4_ID_SPI2		39
#define SAMA5D4_ID_TC0		40
#define SAMA5D4_ID_TC1		41
#define SAMA5D4_ID_TC2		42
#define SAMA5D4_ID_PWM		43
#define SAMA5D4_ID_ADC		44
#define SAMA5D4_ID_DBGU		45
#define SAMA5D4_ID_UHPHS	46
#define SAMA5D4_ID_UDPHS	47
#define SAMA5D4_ID_SSC0		48
#define SAMA5D4_ID_SSC1		49
#define SAMA5D4_ID_DMA1		50
#define SAMA5D4_ID_LCDC		51
#define SAMA5D4_ID_ISI		52
#define SAMA5D4_ID_TRNG		53
#define SAMA5D4_ID_GMAC0	54
#define SAMA5D4_ID_IRQ		56
#define SAMA5D4_ID_IRQ		56
#define SAMA5D4_ID_SFC		57
#define SAMA5D4_ID_SECURAM	59
#define SAMA5D4_ID_CTB		60
#define SAMA5D4_ID_SMD		61
#define SAMA5D4_ID_TWI3		62
#define SAMA5D4_ID_CATB		63
#define SAMA5D4_ID_SFR		64
#define SAMA5D4_ID_AIC		65
#define SAMA5D4_ID_SAIC		66
#define SAMA5D4_ID_L2CC		67


/*
 * User Peripheral physical base addresses.
 */
#define SAMA5D4_BASE_AIC	0xfc06e000 /* (AIC non-secure) Base Address */
#define SAMA5D4_BASE_USART3	0xfc00c000 /* (USART3 non-secure) Base Address */
#define SAMA5D4_BASE_PMC	0xf0018000 /* (PMC) Base Address */
#define SAMA5D4_BASE_MPDDRC	0xf0010000 /* (MPDDRC) Base Address */
#define SAMA5D4_BASE_PIOD	0xfc068000 /* (PIOD) Base Address */
#define SAMA5D4_BASE_PIOE	0xfc06d000 /* (PIOE) Base Address */

/* Some other peripherals */
#define SAMA5D4_BASE_SYS2	SAMA5D4_BASE_PIOD

/*
 * Internal Memory.
 */
#define SAMA5D4_NS_SRAM_BASE     0x00210000      /* Internal SRAM base address Non-Secure */
#define SAMA5D4_NS_SRAM_SIZE     (64 * SZ_1K)   /* Internal SRAM size Non-Secure part (64Kb) */

#endif
