/*
 * Copyright (C) 2014 Atmel,
 *
 * Refer to arch/arm/mach-omap2/sleep44xx.S
 		Copyright (C) 2011 Texas Instruments, Inc
 *
 * This program is free software,you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/linkage.h>
#include <asm/hardware/cache-l2x0.h>
#include <mach/at91_pmc.h>
#include <mach/at91sam9_ddrsdr.h>

/*
 * void at91_cortexa5_do_wfi(void)
 *
 * This function is to disable the Processor Clock
 * and put the core to the WFI state.
 */
ENTRY(at91_cortexa5_do_wfi)
	stmfd	sp!, {lr}

	/*
	 * Execute an ISB instruction to ensure that all of the
	 * CP15 register changes have been committed.
	 */
	isb

	/*
	 * Execute a barrier instruction to ensure that all cache,
	 * TLB and branch predictor maintenance operations issued
	 * by any CPU in the cluster have completed.
	 */
	dsb
	dmb

	/* Disable the Processor Clock */
	bl	at91_get_pmc_base
	mov	r2, r0
	mov	r1, #AT91_PMC_PCK
	str	r1, [r2, #AT91_PMC_SCDR]

	/*
	 * Execute a WFI instruction and wait until the
	 * STANDBYWFI output is asserted to indicate that the
	 * CPU is in idle and low power state. CPU can specualatively
	 * prefetch the instructions so add NOPs after WFI. Sixteen
	 * NOPs as per Cortex-A5 pipeline.
	 */
	wfi	@ Wait For Interrupt
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	ldmfd	sp!, {pc}
ENDPROC(at91_cortexa5_do_wfi)

/*
 * void at91_cortexa5_disable_cache(void)
 *
 * This function is to flush the L1 data cache and disable the L1 data cache.
 * and clean and invalidate the L2 cache, then disable the L2 cache.
 */
ENTRY(at91_cortexa5_disable_cache)
	stmfd	sp!, {r4 - r12, lr}

	/*
	 * Flush all data from the L1 data cache before disabling
	 * SCTLR.C bit.
	 */
	bl	v7_flush_dcache_all

	/*
	 * Clear the SCTLR.C bit to prevent further data cache
	 * allocation. Clearing SCTLR.C would make all the data accesses
	 * strongly ordered and would not hit the cache.
	 */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #(1 << 2)		@ Disable the C bit
	mcr	p15, 0, r0, c1, c0, 0
	isb

	/*
	 * Invalidate L1 data cache. Even though only invalidate is
	 * necessary exported flush API is used here. Doing clean
	 * on already clean cache would be almost NOP.
	 */
	bl	v7_flush_dcache_all

	/*
	 * Clean and invalidate the L2 cache.
	 * Common cache-l2x0.c functions can't be used here since it
	 * uses spinlocks. We are out of coherency here with data cache
	 * disabled. The spinlock implementation uses exclusive load/store
	 * instruction which can fail without data cache being enabled.
	 * Because of this, CPU can lead to deadlock.
	 */
	bl	at91_get_l2cc_base
	mov	r2, r0
	cmp	r2, #0
	beq	skip_l2disable
	mov	r0, #0xff
	str	r0, [r2, #L2X0_CLEAN_INV_WAY]
wait:
	ldr	r0, [r2, #L2X0_CLEAN_INV_WAY]
	mov	r1, #0xff
	ands	r0, r0, r1
	bne	wait

	mov	r0, #0
	str	r0, [r2, #L2X0_CTRL]

l2x_sync:
	ldr	r0, [r2, #L2X0_CACHE_SYNC]
	bic	r0, r0, #0x1
	str	r0, [r2, #L2X0_CACHE_SYNC]
sync:
	ldr	r0, [r2, #L2X0_CACHE_SYNC]
	ands	r0, r0, #0x1
	bne	sync

skip_l2disable:
	ldmfd	sp!, {r4 - r12, pc}
ENDPROC(at91_cortexa5_disable_cache)

/*
 * void at91_cortexa5_enable_cache(void)
 *
 * The function is to enable the L1 data cache and L2 cache.
 */
ENTRY(at91_cortexa5_enable_cache)
	stmfd	sp!, {r4 - r12, lr}

	/*
	 * Enable the L2 cache
	 */
	bl	at91_get_l2cc_base
	mov	r2, r0
	cmp	r2, #0
	beq	skip_l2en
	ldr	r0, [r2, #L2X0_CTRL]
	ands	r0, r0, #L2X0_CTRL_EN
	bne	skip_l2en			@ Skip if already enabled
	mov	r0, #L2X0_CTRL_EN
	str	r0, [r2, #L2X0_CTRL]
skip_l2en:

	/*
	 * CPU is here when it failed to enter OFF/DORMANT or
	 * no low power state was attempted.
	 */
	mrc	p15, 0, r0, c1, c0, 0
	tst	r0, #(1 << 2)			@ Check C bit enabled?
	orreq	r0, r0, #(1 << 2)		@ Enable the C bit
	mcreq	p15, 0, r0, c1, c0, 0
	isb

	ldmfd	sp!, {r4 - r12, pc}
ENDPROC(at91_cortexa5_enable_cache)

/*
 * unsigned int at91_ddrc_enter_self_refresh(void)
 *
 * The function is to put the DDR SDRAM to enter the self-refresh mode
 *  @return: DDRC low-power register setting.
 */
ENTRY(at91_ddrc_enter_self_refresh)
	stmfd	sp!, {r4 - r12, lr}

	bl	at91_get_ramc0_base
	mov	r2, r0
	ldr	r1, [r2, #AT91_DDRSDRC_LPR]
	mov	r0, r1
	bic	r1, #AT91_DDRSDRC_LPCB
	orr	r1, #AT91_DDRSDRC_LPCB_SELF_REFRESH
	str	r1, [r2, #AT91_DDRSDRC_LPR]

	ldmfd	sp!, {r4 - r12, pc}
ENDPROC(at91_ddrc_enter_self_refresh)

/*
 * void at91_ddrc_exit_self_refresh(unsigned int lpr_setting)
 *
 * The funtion is to make the DDR SDRAM exit the self-refresh mode.
 * @lpr_setting: DDRC low-power register setting.
 */

ENTRY(at91_ddrc_exit_self_refresh)
	stmfd	sp!, {r4 - r12, lr}

	mov	r1, r0
	bl	at91_get_ramc0_base
	mov	r2, r0
	str	r1, [r2, #AT91_DDRSDRC_LPR]

	ldmfd	sp!, {r4 - r12, pc}
ENDPROC(at91_ddrc_exit_self_refresh)

/*
 * void at91_cortexa5_standby(void)
 *
 * The function is to make the core perform suspend to standby mode.
 */
ENTRY(at91_cortexa5_standby)
	stmfd	sp!, {r4 - r12, lr}

	bl	at91_cortexa5_disable_cache

	bl	at91_ddrc_enter_self_refresh
	push	{r0}

	bl	at91_cortexa5_do_wfi

	pop	{r0}
	bl	at91_ddrc_exit_self_refresh

	bl	at91_cortexa5_enable_cache

	ldmfd	sp!, {r4 - r12, pc}
ENDPROC(at91_cortexa5_standby)
