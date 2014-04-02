/*
 * Copyright (C) 2013 Atmel,
 *                    Nicolas Ferre <nicolas.ferre@atmel.com>
 *
 * SMC Calls
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ATMEL_FIRMWARE_SMC_H
#define __ATMEL_FIRMWARE_SMC_H

/* For Power Management */
#define SMC_CMD_PMC_READ		0x24
#define SMC_CMD_PMC_PERIPH_CLK		0x25
#define SMC_CMD_PMC_SYS_CLK		0x26
#define SMC_CMD_PMC_UCKR_CLK		0x27
#define SMC_CMD_PMC_USB_SETUP		0x28
#define SMC_CMD_PMC_SMD_SETUP		0x50
/* For RSTC */
#define SMC_CMD_PM_RESTART		0x29
/* For CP15 Access */
/* For L2 Cache Access */
#define SMC_CMD_L2CC_ENABLE		0x42
#define SMC_CMD_L2CC_DISABLE		0x43
/* watchdog */
#define SMC_CMD_WDT_SET_COUNTER		0x60
#define SMC_CMD_WDT_RELOAD_COUNTER	0x61

/* watchdog */
#define SMC_CMD_WDT_SET_COUNTER		0x60
#define SMC_CMD_WDT_RELOAD_COUNTER	0x61

#ifndef __ASSEMBLY__

extern int atmel_smc(u32 cmd, u32 arg1, u32 arg2, u32 arg3);

#endif
#endif
