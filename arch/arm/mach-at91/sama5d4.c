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
static struct clk adc_clk = {
	.name		= "adc_clk",
	.pid		= SAMA5D4_ID_ADC,
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
static struct clk isi_clk = {
	.name		= "isi_clk",
	.pid		= SAMA5D4_ID_ISI,
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
static struct clk ssc0_clk = {
	.name		= "ssc0_clk",
	.pid		= SAMA5D4_ID_SSC0,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk ssc1_clk = {
	.name		= "ssc1_clk",
	.pid		= SAMA5D4_ID_SSC1,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk sha_clk = {
	.name		= "sha_clk",
	.pid		= SAMA5D4_ID_SHA,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk aes_clk = {
	.name		= "aes_clk",
	.pid		= SAMA5D4_ID_AES,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk tdes_clk = {
	.name		= "tdes_clk",
	.pid		= SAMA5D4_ID_TDES,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk pwm_clk = {
	.name		= "pwm_clk",
	.pid		= SAMA5D4_ID_PWM,
	.type		= CLK_TYPE_PERIPHERAL,
};
static struct clk vdec_clk= {
	.name		= "vdec_clk",
	.pid		= SAMA5D4_ID_VDEC,
	.type		= CLK_TYPE_PERIPHERAL | CLK_TYPE_PERIPH_H64MX,
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
	&adc_clk,
	&dma0_clk,
	&dma1_clk,
	&uhphs_clk,
	&udphs_clk,
	&lcdc_clk,
	&isi_clk,
	&macb0_clk,
	&twi0_clk,
	&twi2_clk,
	&spi0_clk,
	&smd_clk,
	&ssc0_clk,
	&ssc1_clk,
	&sha_clk,
	&aes_clk,
	&tdes_clk,
	&pwm_clk,
	&vdec_clk,
};

static struct clk pck0 = {
	.name		= "pck0",
	.pmc_mask	= AT91_PMC_PCK0,
	.type		= CLK_TYPE_PROGRAMMABLE,
	.id		= 0,
};

static struct clk pck1 = {
	.name		= "pck1",
	.pmc_mask	= AT91_PMC_PCK1,
	.type		= CLK_TYPE_PROGRAMMABLE,
	.id		= 1,
};

static struct clk pck2 = {
	.name		= "pck2",
	.pmc_mask	= AT91_PMC_PCK2,
	.type		= CLK_TYPE_PROGRAMMABLE,
	.id		= 2,
};

static struct clk_lookup periph_clocks_lookups[] = {
	/* lookup table for DT entries */
	CLKDEV_CON_DEV_ID("vdec_clk", "300000.vdec", &vdec_clk),
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
	CLKDEV_CON_DEV_ID("adc_clk", "fc034000.adc", &adc_clk),
	CLKDEV_CON_DEV_ID("aes_clk", "fc044000.aes", &aes_clk),
	CLKDEV_CON_DEV_ID("tdes_clk", "fc04c000.tdes", &tdes_clk),
	CLKDEV_CON_DEV_ID("sha_clk", "fc050000.sha", &sha_clk),
	CLKDEV_CON_DEV_ID(NULL, "fc068000.gpio", &pioD_clk),
	CLKDEV_CON_DEV_ID("usart", "fc069000.serial", &mck),
	CLKDEV_CON_DEV_ID(NULL, "fc06a000.gpio", &pioA_clk),
	CLKDEV_CON_DEV_ID(NULL, "fc06b000.gpio", &pioB_clk),
	CLKDEV_CON_DEV_ID(NULL, "fc06c000.gpio", &pioC_clk),
	CLKDEV_CON_DEV_ID(NULL, "fc06d000.gpio", &pioE_clk),
	CLKDEV_CON_DEV_ID(NULL, "f8008000.ssc", &ssc0_clk),
	CLKDEV_CON_DEV_ID(NULL, "fc014000.ssc", &ssc1_clk),
	CLKDEV_CON_DEV_ID(NULL, "f800c000.pwm", &pwm_clk),
};

static void __init sama5d4_register_clocks(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(periph_clocks); i++)
		clk_register(periph_clocks[i]);

	clkdev_add_table(periph_clocks_lookups,
			 ARRAY_SIZE(periph_clocks_lookups));

	clk_register(&pck0);
	clk_register(&pck1);
	clk_register(&pck2);
}

/* --------------------------------------------------------------------
 *  Processor initialization
 * -------------------------------------------------------------------- */

static struct map_desc at91_io_desc[] __initdata = {
	{
	.virtual	= (unsigned long)AT91_ALT_IO_P2V(SAMA5D4_BASE_MPDDRC),
	.pfn		= __phys_to_pfn(SAMA5D4_BASE_MPDDRC),
	.length		= SZ_512,
	.type		= MT_DEVICE,
	},
	{
	.virtual	= (unsigned long)AT91_ALT_IO_P2V(SAMA5D4_BASE_PMC),
	.pfn		= __phys_to_pfn(SAMA5D4_BASE_PMC),
	.length		= SZ_512,
	.type		= MT_DEVICE,
	},
	{ /* On sama5d4, we use USART3 as serial console */
	.virtual	= (unsigned long)AT91_ALT_IO_P2V(SAMA5D4_BASE_USART3),
	.pfn		= __phys_to_pfn(SAMA5D4_BASE_USART3),
	.length		= SZ_256,
	.type		= MT_DEVICE,
	},
	{ /* A bunch of peripheral with fine grained IO space */
	.virtual	= (unsigned long)AT91_ALT_IO_P2V(SAMA5D4_BASE_SYS2),
	.pfn		= __phys_to_pfn(SAMA5D4_BASE_SYS2),
	.length		= SZ_2K,
	.type		= MT_DEVICE,
	},
};

static void __init sama5d4_map_io(void)
{
	iotable_init(at91_io_desc, ARRAY_SIZE(at91_io_desc));
	at91_init_sram(0, SAMA5D4_NS_SRAM_BASE, SAMA5D4_NS_SRAM_SIZE);
}

AT91_SOC_START(sama5d4)
	.map_io = sama5d4_map_io,
	.register_clocks = sama5d4_register_clocks,
AT91_SOC_END
