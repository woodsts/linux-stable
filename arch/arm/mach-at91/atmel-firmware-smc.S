/*
 * Copyright (C) 2013 Atmel,
 *                    Nicolas Ferre <nicolas.ferre@atmel.com>
 *
 * Copied from omap-smc.S Copyright (C) 2010 Texas Instruments, Inc.
 *
 * This program is free software,you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/linkage.h>

/*
 * Function signature: void atmel_smc(u32 cmd, u32 arg1, u32 arg2, u32 arg3)
 */

ENTRY(atmel_smc)
	stmfd	sp!, {r4-r11, lr}
	dsb
	smc	#0
	ldmfd	sp!, {r4-r11, pc}
ENDPROC(atmel_smc)
