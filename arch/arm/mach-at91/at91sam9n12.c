/*
 * SoC specific setup code for the AT91SAM9N12
 *
 * Copyright (C) 2012 Atmel Corporation.
 *
 * Licensed under GPLv2 or later.
 */

#include <asm/system_misc.h>
#include <mach/hardware.h>

#include "soc.h"
#include "generic.h"

/* --------------------------------------------------------------------
 *  AT91SAM9N12 processor initialization
 * -------------------------------------------------------------------- */

static void __init at91sam9n12_initialize(void)
{
	at91_sysirq_mask_rtc(AT91SAM9N12_BASE_RTC);
}

AT91_SOC_START(at91sam9n12)
	.init = at91sam9n12_initialize,
AT91_SOC_END
