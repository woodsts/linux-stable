/*
 * Copyright (C) 2012 Samsung Electronics.
 * Kyungmin Park <kyungmin.park@samsung.com>
 * Tomasz Figa <t.figa@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARM_FIRMWARE_H
#define __ASM_ARM_FIRMWARE_H

#include <linux/bug.h>

/*
 * struct firmware_ops
 *
 * A structure to specify available firmware operations.
 *
 * A filled up structure can be registered with register_firmware_ops().
 */
struct firmware_ops {
	/*
	 * Enters CPU idle mode
	 */
	int (*do_idle)(void);
	/*
	 * Sets boot address of specified physical CPU
	 */
	int (*set_cpu_boot_addr)(int cpu, unsigned long boot_addr);
	/*
	 * Boots specified physical CPU
	 */
	int (*cpu_boot)(int cpu);
	/*
	 * Initializes L2 cache
	 */
	int (*l2x0_init)(void);
	/*
	 * Disables L2 cache
	 */
	void (*l2x0_disable)(void);
	/*
	 * PMC
	 */
	int (*pmc_read_reg)(u32 *reg_value, u32 reg_offset);
	int (*pmc_periph_clk)(u32 periph_id, u32 is_on);
	int (*pmc_sys_clk)(u32 sys_clk_mask, u32 is_on);
	int (*pmc_uckr_clk)(u32 is_on);
	int (*pmc_usb_setup)(void);
	int (*pmc_smd_setup)(u32 reg_value);
	/*
	 * Restart SoC
	 */
	int (*pm_restart)(void);

	/*
	 * Watchdog
	 */
	int (*wdt_set_counter)(u32 count);
	int (*wdt_reload_counter)(void);
};

/* Global pointer for current firmware_ops structure, can't be NULL. */
extern const struct firmware_ops *firmware_ops;

/*
 * call_firmware_op(op, ...)
 *
 * Checks if firmware operation is present and calls it,
 * otherwise returns -ENOSYS
 */
#define call_firmware_op(op, ...)					\
	((firmware_ops->op) ? firmware_ops->op(__VA_ARGS__) : (-ENOSYS))

/*
 * register_firmware_ops(ops)
 *
 * A function to register platform firmware_ops struct.
 */
static inline void register_firmware_ops(const struct firmware_ops *ops)
{
	BUG_ON(!ops);

	firmware_ops = ops;
}

#endif
