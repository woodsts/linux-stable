/*
 *  Chip-specific setup code for the SAMA5D4 family
 *
 *  Copyright (C) 2013 Atmel Corporation,
 *                     Nicolas Ferre <nicolas.ferre@atmel.com>
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>

#include <asm/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include "clock.h"
#include <mach/sama5d4.h>
#include <mach/at91_pmc.h>
#include <mach/cpu.h>

#include "soc.h"
#include "generic.h"
#include "sam9_smc.h"

/* --------------------------------------------------------------------
 *  Clocks
 * -------------------------------------------------------------------- */

/*
 * The peripheral clocks.
 */

static struct clk pioA_clk = {
	.name		= "pioA_clk",
	.pid		= SAMA5D4_ID_PIOA,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk pioB_clk = {
	.name		= "pioB_clk",
	.pid		= SAMA5D4_ID_PIOB,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk pioC_clk = {
	.name		= "pioC_clk",
	.pid		= SAMA5D4_ID_PIOC,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk pioD_clk = {
	.name		= "pioD_clk",
	.pid		= SAMA5D4_ID_PIOD,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk pioE_clk = {
	.name		= "pioE_clk",
	.pid		= SAMA5D4_ID_PIOE,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk usart0_clk = {
	.name		= "usart0_clk",
	.pid		= SAMA5D4_ID_USART0,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk usart2_clk = {
	.name		= "usart2_clk",
	.pid		= SAMA5D4_ID_USART2,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk usart3_clk = {
	.name		= "usart3_clk",
	.pid		= SAMA5D4_ID_USART3,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk usart4_clk = {
	.name		= "usart4_clk",
	.pid		= SAMA5D4_ID_USART4,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk mmc0_clk = {
	.name		= "mci0_clk",
	.pid		= SAMA5D4_ID_HSMCI0,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk mmc1_clk = {
	.name		= "mci1_clk",
	.pid		= SAMA5D4_ID_HSMCI1,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk tcb0_clk = {
	.name		= "tcb0_clk",
	.pid		= SAMA5D4_ID_TC0,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk tcb1_clk = {
	.name		= "tcb1_clk",
	.pid		= SAMA5D4_ID_TC1,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk dma0_clk = {
	.name		= "dma0_clk",
	.pid		= SAMA5D4_ID_DMA0,
	.type		= CLK_TYPE_PERIPHERAL | CLK_TYPE_PERIPH_H64MX,
};
static struct clk dma1_clk = {
	.name		= "dma1_clk",
	.pid		= SAMA5D4_ID_DMA1,
	.type		= CLK_TYPE_PERIPHERAL | CLK_TYPE_PERIPH_H64MX,
};
static struct clk uhphs_clk = {
	.name		= "uhphs",
	.pid		= SAMA5D4_ID_UHPHS,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk udphs_clk = {
	.name		= "udphs_clk",
	.pid		= SAMA5D4_ID_UDPHS,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk lcdc_clk = {
	.name		= "lcdc_clk",
	.pid		= SAMA5D4_ID_LCDC,
	.type		= CLK_TYPE_PERIPHERAL | CLK_TYPE_PERIPH_H64MX,
};
static struct clk macb0_clk = {
	.name		= "macb0_clk",
	.pid		= SAMA5D4_ID_GMAC0,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk twi0_clk = {
	.name		= "twi0_clk",
	.pid		= SAMA5D4_ID_TWI0,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk twi2_clk = {
	.name		= "twi2_clk",
	.pid		= SAMA5D4_ID_TWI2,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk spi0_clk = {
	.name		= "spi0_clk",
	.pid		= SAMA5D4_ID_SPI0,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk smd_clk = {
	.name		= "smd_clk",
	.pid		= SAMA5D4_ID_SMD,
	.type		= CLK_TYPE_PERIPHERAL,
};

static struct clk *periph_clocks[] __initdata = {
	&pioA_clk,
	&pioB_clk,
	&pioC_clk,
	&pioD_clk,
	&pioE_clk,
	&usart0_clk,
	&usart2_clk,
	&usart3_clk,
	&usart4_clk,
	&mmc0_clk,
	&mmc1_clk,
	&tcb0_clk,
	&tcb1_clk,
	&dma0_clk,
	&dma1_clk,
	&uhphs_clk,
	&udphs_clk,
	&lcdc_clk,
	&macb0_clk,
	&twi0_clk,
	&twi2_clk,
	&spi0_clk,
	&smd_clk,
};

static struct clk_lookup periph_clocks_lookups[] = {
	/* lookup table for DT entries */
	CLKDEV_CON_DEV_ID("pclk", "400000.gadget", &udphs_clk),
	CLKDEV_CON_DEV_ID("hclk", "400000.gadget", &utmi_clk),
	CLKDEV_CON_DEV_ID("hclk", "500000.ohci", &uhphs_clk),
	CLKDEV_CON_DEV_ID("ohci_clk", "500000.ohci", &uhphs_clk),
	CLKDEV_CON_DEV_ID("ehci_clk", "600000.ehci", &uhphs_clk),
	CLKDEV_CON_DEV_ID("dma_clk", "f0014000.dma-controller", &dma0_clk),
	CLKDEV_CON_DEV_ID("dma_clk", "f0004000.dma-controller", &dma1_clk),
	CLKDEV_CON_DEV_ID("mci_clk", "f8000000.mmc", &mmc0_clk),
	CLKDEV_CON_DEV_ID("mci_clk", "fc000000.mmc", &mmc1_clk),
	CLKDEV_CON_DEV_ID(NULL, "f8014000.i2c", &twi0_clk),
	CLKDEV_CON_DEV_ID(NULL, "f8024000.i2c", &twi2_clk),
	CLKDEV_CON_DEV_ID("spi_clk", "f8010000.spi", &spi0_clk),
	CLKDEV_CON_DEV_ID("t0_clk", "f801c000.timer", &tcb0_clk),
	CLKDEV_CON_DEV_ID("hclk", "f8020000.ethernet", &macb0_clk),
	CLKDEV_CON_DEV_ID("pclk", "f8020000.ethernet", &macb0_clk),
	CLKDEV_CON_DEV_ID("usart", "f802c000.serial", &usart0_clk),
	CLKDEV_CON_DEV_ID("usart", "fc008000.serial", &usart2_clk),
	CLKDEV_CON_DEV_ID("usart", "fc00c000.serial", &usart3_clk),
	CLKDEV_CON_DEV_ID("usart", "fc010000.serial", &usart4_clk),
	CLKDEV_CON_DEV_ID("t0_clk", "fc020000.timer", &tcb1_clk),
	CLKDEV_CON_DEV_ID(NULL, "fc068000.gpio", &pioD_clk),
	CLKDEV_CON_DEV_ID("usart", "fc069000.serial", &mck),
	CLKDEV_CON_DEV_ID(NULL, "fc06a000.gpio", &pioA_clk),
	CLKDEV_CON_DEV_ID(NULL, "fc06b000.gpio", &pioB_clk),
	CLKDEV_CON_DEV_ID(NULL, "fc06c000.gpio", &pioC_clk),
	CLKDEV_CON_DEV_ID(NULL, "fc06d000.gpio", &pioE_clk),
};

static void __init sama5d4_register_clocks(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(periph_clocks); i++)
		clk_register(periph_clocks[i]);

	clkdev_add_table(periph_clocks_lookups,
			 ARRAY_SIZE(periph_clocks_lookups));
}

/* --------------------------------------------------------------------
 *  Processor initialization
 * -------------------------------------------------------------------- */

static void __init sama5d4_map_io(void)
{
	at91_init_sram(0, SAMA5D4_NS_SRAM_BASE, SAMA5D4_NS_SRAM_SIZE);
}

AT91_SOC_START(sama5d4)
	.map_io = sama5d4_map_io,
	.register_clocks = sama5d4_register_clocks,
AT91_SOC_END
