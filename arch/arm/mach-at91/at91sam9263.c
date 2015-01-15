/*
 * arch/arm/mach-at91/at91sam9263.c
 *
 *  Copyright (C) 2007 Atmel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <asm/system_misc.h>
#include <mach/hardware.h>

#include "soc.h"
#include "generic.h"

/* --------------------------------------------------------------------
 *  AT91SAM9263 processor initialization
 * -------------------------------------------------------------------- */

static void __init at91sam9263_initialize(void)
{
	arm_pm_idle = at91sam9_idle;

	at91_sysirq_mask_rtt(AT91SAM9263_BASE_RTT0);
	at91_sysirq_mask_rtt(AT91SAM9263_BASE_RTT1);
}

AT91_SOC_START(at91sam9263)
	.init = at91sam9263_initialize,
AT91_SOC_END
