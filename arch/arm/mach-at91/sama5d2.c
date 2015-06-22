/*
 *  Chip-specific setup code for the SAMA5D2 family
 *
 *  Copyright (C) 2015 Atmel Corporation,
 *                     Nicolas Ferre <nicolas.ferre@atmel.com>
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/clk/at91_pmc.h>

#include <asm/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/sama5d2.h>
#include <mach/cpu.h>
#include <mach/hardware.h>

#include "soc.h"
#include "generic.h"
#include "sam9_smc.h"

/* --------------------------------------------------------------------
 *  Processor initialization
 * -------------------------------------------------------------------- */
static struct map_desc at91_io_desc[] __initdata = {
	/*
	{
	.virtual        = (unsigned long)AT91_ALT_IO_P2V(SAMA5D2_BASE_MPDDRC),
	.pfn            = __phys_to_pfn(SAMA5D2_BASE_MPDDRC),
	.length         = SZ_512,
	.type           = MT_DEVICE,
	}, */
	{
	.virtual        = (unsigned long)AT91_ALT_IO_P2V(SAMA5D2_BASE_PMC),
	.pfn            = __phys_to_pfn(SAMA5D2_BASE_PMC),
	.length         = SZ_512,
	.type           = MT_DEVICE,
	},
	{ /* On sama5d2, we use UART1 as serial console */
	.virtual        = (unsigned long)AT91_ALT_IO_P2V(SAMA5D2_BASE_UART1),
	.pfn            = __phys_to_pfn(SAMA5D2_BASE_UART1),
	.length         = SZ_256,
	.type           = MT_DEVICE,
	},
	{ /* A bunch of peripheral with fine grained IO space */
	.virtual        = (unsigned long)AT91_ALT_IO_P2V(SAMA5D2_BASE_SYS),
	.pfn            = __phys_to_pfn(SAMA5D2_BASE_SYS),
	.length         = SZ_2K,
	.type           = MT_DEVICE,
	},
};


static void __init sama5d2_map_io(void)
{
	iotable_init(at91_io_desc, ARRAY_SIZE(at91_io_desc));
}

AT91_SOC_START(sama5d2)
	.map_io = sama5d2_map_io,
AT91_SOC_END
