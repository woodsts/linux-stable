/*
 * Chip-specific header file for the SAMA5D3 family
 *
 *  Copyright (C) 2009-2012 Atmel Corporation.
 *
 * Common definitions.
 * Based on SAMA5D3 datasheet.
 *
 * Licensed under GPLv2 or later.
 */

#ifndef SAMA5D3_H
#define SAMA5D3_H

/*
 * Peripheral identifiers/interrupts.
 */
#define AT91_ID_FIQ            	 0	/* Advanced Interrupt Controller (FIQ) */
#define AT91_ID_SYS            	 1	/* System Peripherals */
#define SAMA5D3_ID_DBGU       	 2	/* debug Unit (usually no special interrupt line) */
#define AT91_ID_PIT		 3	/* PIT */
#define SAMA5D3_ID_HSMC		 5	/* Static Memory Controller */
#define SAMA5D3_ID_PIOA		 6	/* PIOA */
#define SAMA5D3_ID_PIOB		 7	/* PIOB */
#define SAMA5D3_ID_PIOC		 8	/* PIOC */
#define SAMA5D3_ID_PIOD		 9	/* PIOD */
#define SAMA5D3_ID_PIOE		10	/* PIOE */
#define SAMA5D3_ID_USART0	12	/* USART0 */
#define SAMA5D3_ID_USART1	13	/* USART1 */
#define SAMA5D3_ID_USART2	14	/* USART2 */
#define SAMA5D3_ID_USART3	15	/* USART3 */
#define SAMA5D3_ID_UART0	16	/* UART0 */
#define SAMA5D3_ID_UART1	17	/* UART1 */
#define SAMA5D3_ID_TWI0		18	/* Two-Wire Interface 0 */
#define SAMA5D3_ID_TWI1		19	/* Two-Wire Interface 1 */
#define SAMA5D3_ID_TWI2		20	/* Two-Wire Interface 2 */
#define SAMA5D3_ID_HSMCI0	21	/* MCI */
#define SAMA5D3_ID_HSMCI1	22	/* MCI */
#define SAMA5D3_ID_HSMCI2	23	/* MCI */
#define SAMA5D3_ID_SPI0		24	/* Serial Peripheral Interface 0 */
#define SAMA5D3_ID_SPI1		25	/* Serial Peripheral Interface 1 */
#define SAMA5D3_ID_TC0        	26      /* Timer Counter 0 */
#define SAMA5D3_ID_TC1        	27      /* Timer Counter 2 */
#define SAMA5D3_ID_ADC		29	/* Touch Screen ADC Controller */
#define SAMA5D3_ID_DMA0		30	/* DMA Controller 0 */
#define SAMA5D3_ID_DMA1		31	/* DMA Controller 1 */
#define SAMA5D3_ID_UHPHS	32	/* USB Host High Speed */
#define SAMA5D3_ID_UDPHS	33	/* USB Device High Speed */
#define SAMA5D3_ID_GMAC		34	/* Gigabit Ethernet MAC */
#define SAMA5D3_ID_EMAC		35	/* Ethernet MAC */
#define SAMA5D3_ID_LCDC     	36      /* LCD Controller */
#define SAMA5D3_ID_ISI		37	/* Image Sensor Interface */
#define SAMA5D3_ID_CAN0		40	/* CAN Controller 0 */
#define SAMA5D3_ID_CAN1		41	/* CAN Controller 1 */
#define SAMA5D3_ID_SSC0		38	/* Synchronous Serial Controller 0 */
#define SAMA5D3_ID_SSC1		39	/* Synchronous Serial Controller 1 */
#define SAMA5D3_ID_SHA		42	/* AES */
#define SAMA5D3_ID_AES		43	/* AES */
#define SAMA5D3_ID_TDES		44	/* TDES */
#define SAMA5D3_ID_IRQ0		41	/* Advanced Interrupt Controller (IRQ0) */

/*
 * User Peripheral physical base addresses.
 */
#define SAMA5D3_BASE_TC0	0xf0010000 /* (TC0) Base Address */
#define SAMA5D3_BASE_TC1	0xf0010040 /* (TC1) Base Address */
#define SAMA5D3_BASE_GMAC	0xf0028000 /* (GMAC) Base Address */
#define SAMA5D3_BASE_LCDC	0xf0030000 /* (HLCDC5) Base Address */
#define SAMA5D3_BASE_HSMCI0	0xf0000000 /* (MMCI) Base Address */
#define SAMA5D3_BASE_EMAC	0xf802c000 /* (EMAC) Base Address */
#define SAMA5D3_BASE_UDPHS	0xf8030000
#define AT91_BASE_SYS		0xffffc000

/*
 * System Peripherals (offset from AT91_BASE_SYS)
 */
#define AT91_DMA0	(0xffffe600 - AT91_BASE_SYS)
#define AT91_DMA1	(0xffffe800 - AT91_BASE_SYS)
#define AT91_MATRIX	(0xffffec00 - AT91_BASE_SYS)
#define AT91_DBGU       AT91_BASE_DBGU1
#define AT91_PIOA	(0xfffff200 - AT91_BASE_SYS)
#define AT91_PIOB	(0xfffff400 - AT91_BASE_SYS)
#define AT91_PIOC	(0xfffff600 - AT91_BASE_SYS)
#define AT91_PIOD	(0xfffff800 - AT91_BASE_SYS)
#define AT91_PIOE	(0xfffffA00 - AT91_BASE_SYS)
#define AT91_PIT        (0xfffffe30 - AT91_BASE_SYS)
#define AT91_WDT        (0xfffffe40 - AT91_BASE_SYS)
#define AT91_GPBR       (0xfffffe60 - AT91_BASE_SYS) // KO OAR_TEMP, NO GPBR, error while building in "drivers/rtc/rtc-at91sam9.c"

/*
 * Internal Memory.
 */
#define SAMA5D3_SRAM_BASE     0x00300000      /* Internal SRAM base address */
#define SAMA5D3_SRAM_SIZE     (128 * SZ_1K)   /* Internal SRAM size (128Kb) */

#define SAMA5D3_UDPHS_FIFO	0x00500000
#define SAMA5D3_OHCI_BASE	0x00600000	/* USB Host controller (OHCI) */
#define SAMA5D3_EHCI_BASE	0x00700000	/* USB Host controller (EHCI) */

/*
 * DMA0 peripheral identifiers
 * for hardware handshaking interface
 */
#define SAMA5_DMA_ID_MCI0		 0
#define SAMA5_DMA_ID_SPI0_TX	 1
#define SAMA5_DMA_ID_SPI0_RX	 2
#define SAMA5_DMA_ID_USART0_TX	 3
#define SAMA5_DMA_ID_USART0_RX	 4
#define SAMA5_DMA_ID_USART1_TX	 5
#define SAMA5_DMA_ID_USART1_RX	 6
#define SAMA5_DMA_ID_TWI0_TX	 7
#define SAMA5_DMA_ID_TWI0_RX	 8
#define SAMA5_DMA_ID_TWI1_TX	 9
#define SAMA5_DMA_ID_TWI1_RX	10
#define SAMA5_DMA_ID_UART0_TX	11
#define SAMA5_DMA_ID_UART0_RX	12
#define SAMA5_DMA_ID_SSC0_TX	13
#define SAMA5_DMA_ID_SSC0_RX	14
#define SAMA5_DMA_ID_SMD_TX	15
#define SAMA5_DMA_ID_SMD_RX	16

/*
 * DMA1 peripheral identifiers
 * for hardware handshaking interface
 */
#define SAMA5_DMA_ID_MCI1           0

#endif
