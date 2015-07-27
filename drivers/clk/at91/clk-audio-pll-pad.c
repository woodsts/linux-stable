/*
 *  Copyright (C) 2015 Atmel Corporation,
 *                     Nicolas Ferre <nicolas.ferre@atmel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#define DEBUG 12

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/clk/at91_pmc.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>

#include "pmc.h"

#define AUDIO_PLL_FOUT_MIN		600000000
#define AUDIO_PLL_FOUT_MAX		800000000
#define AUDIO_PLL_REFERENCE_FOUT	700000000

#define AUDIO_PLL_QDPAD_MAX	((AT91_PMC_AUDIO_PLL_QDPAD_DIV_MASK >> \
				    AT91_PMC_AUDIO_PLL_QDPAD_DIV_OFFSET) * \
					AT91_PMC_AUDIO_PLL_QDPAD_EXTDIV_MAX)
#define AUDIO_PLL_QDPAD_EXTDIV_OFFSET	(AT91_PMC_AUDIO_PLL_QDPAD_EXTDIV_OFFSET - \
					     AT91_PMC_AUDIO_PLL_QDPAD_OFFSET)
#define AUDIO_PLL_DIV2QD(div, ext_div)	((div) | ((ext_div) << \
						  AUDIO_PLL_QDPAD_EXTDIV_OFFSET))
#define AUDIO_PLL_QD2DIV(qd)	((qd) & (AT91_PMC_AUDIO_PLL_QDPAD_DIV_MASK >> \
					  AT91_PMC_AUDIO_PLL_QDPAD_DIV_OFFSET))
#define AUDIO_PLL_QD2EXTDIV(qd)	(((qd) >> AUDIO_PLL_QDPAD_EXTDIV_OFFSET) \
				 & (AT91_PMC_AUDIO_PLL_QDPAD_EXTDIV_MASK >> \
					  AT91_PMC_AUDIO_PLL_QDPAD_EXTDIV_OFFSET))

struct clk_audio_pad {
	struct clk_hw hw;
	struct at91_pmc *pmc;
	u8 qdpad;
};

#define to_clk_audio_pad(hw) container_of(hw, struct clk_audio_pad, hw)

static int clk_audio_pll_pad_enable(struct clk_hw *hw)
{
	struct clk_audio_pad *apad_ck = to_clk_audio_pad(hw);
	struct at91_pmc *pmc = apad_ck->pmc;
	u32 tmp;

	pmc_lock(pmc);
	tmp = pmc_read(pmc, AT91_PMC_AUDIO_PLL1) & ~AT91_PMC_AUDIO_PLL_QDPAD_MASK;
	tmp |= AT91_PMC_AUDIO_PLL_QDPAD(apad_ck->qdpad);
	pmc_write(pmc, AT91_PMC_AUDIO_PLL1, tmp);
	tmp = pmc_read(pmc, AT91_PMC_AUDIO_PLL0);
	pmc_write(pmc, AT91_PMC_AUDIO_PLL0, tmp | AT91_PMC_AUDIO_PLL_PADEN);
	pmc_unlock(pmc);
	return 0;
}

static void clk_audio_pll_pad_disable(struct clk_hw *hw)
{
	struct clk_audio_pad *apad_ck = to_clk_audio_pad(hw);
	struct at91_pmc *pmc = apad_ck->pmc;
	u32 tmp;

	pmc_lock(pmc);
	tmp = pmc_read(pmc, AT91_PMC_AUDIO_PLL0) & ~AT91_PMC_AUDIO_PLL_PADEN;
	pmc_write(pmc, AT91_PMC_AUDIO_PLL0, tmp);
	pmc_unlock(pmc);
}

static unsigned long clk_audio_pll_pad_recalc_rate(struct clk_hw *hw,
					           unsigned long parent_rate)
{
	struct clk_audio_pad *apad_ck = to_clk_audio_pad(hw);
	unsigned long apad_rate = 0;
	u8 tmp_div = AUDIO_PLL_QD2DIV(apad_ck->qdpad);
	u8 tmp_ext_div = AUDIO_PLL_QD2EXTDIV(apad_ck->qdpad);

	if (tmp_div && tmp_ext_div)
		apad_rate = parent_rate / (tmp_div * tmp_ext_div);

	pr_debug("A PLL/PAD: %s, apad_rate = %lu (div = %u, ext_div = %u)\n" ,
		 __func__ , apad_rate, tmp_div, tmp_ext_div);

	return apad_rate;
}

static int clk_audio_pll_compute_qdpad(unsigned long q_rate, unsigned long rate,
				       unsigned long *qd, u8 *div, u8 *ext_div)
{
	unsigned long tmp_qd;
	unsigned long rem2, rem3;
	unsigned long ldiv, lext_div;;

	if (!rate)
		return -EINVAL;

	tmp_qd = q_rate / rate;
	if (!tmp_qd || tmp_qd > AUDIO_PLL_QDPAD_MAX)
		return -EINVAL;

	if (tmp_qd <= AT91_PMC_AUDIO_PLL_QDPAD_EXTDIV_MAX) {
		ldiv = 1;
		lext_div = tmp_qd;
	} else {
		rem2 = tmp_qd % 2;
		rem3 = tmp_qd % 3;

		if (rem3 == 0 ||
		    tmp_qd > AT91_PMC_AUDIO_PLL_QDPAD_EXTDIV_MAX * 2 ||
		    rem3 < rem2) {
			ldiv = 3;
			lext_div = tmp_qd / 3;
		} else {
			ldiv = 2;
			lext_div = tmp_qd >> 1;
		}
	}

	pr_debug("A PLL/PAD: %s, qd = %lu (div = %lu, ext_div = %lu)\n" ,
		 __func__ , ldiv * lext_div, ldiv, lext_div);

	/* if we were given variable to store, we can provide them */
	if (qd)
		*qd = ldiv * lext_div;
	if (div && ext_div) {
		/* we can cast here as we verified the bounds just above */
		*div = (u8)ldiv;
		*ext_div = (u8)lext_div;
	}

	return 0;
}

static long clk_audio_pll_pad_round_rate(struct clk_hw *hw, unsigned long rate,
					 unsigned long *parent_rate)
{
	struct clk *pclk = __clk_get_parent(hw->clk);
	long best_rate = -EINVAL;
	unsigned long best_parent_rate = 0;
	unsigned long tmp_qd;

	pr_debug("A PLL/PAD: %s, rate = %lu (parent_rate = %lu)\n" ,
		 __func__ , rate, *parent_rate);

	if (clk_audio_pll_compute_qdpad(AUDIO_PLL_REFERENCE_FOUT, rate,
				        &tmp_qd, NULL, NULL))
		return -EINVAL;

	best_parent_rate = __clk_round_rate(pclk, rate * tmp_qd);
	best_rate = best_parent_rate / tmp_qd;

	pr_debug("A PLL/PAD: %s, best_rate = %ld, best_parent_rate = %lu\n",
		 __func__, best_rate, best_parent_rate);

	*parent_rate = best_parent_rate;
	return best_rate;
}

static int clk_audio_pll_pad_set_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long parent_rate)
{
	struct clk_audio_pad *apad_ck = to_clk_audio_pad(hw);
	u8 tmp_div, tmp_ext_div;
	int ret;

	pr_debug("A PLL/PAD: %s, rate = %lu (parent_rate = %lu)\n" ,
		 __func__ , rate, parent_rate);

	ret = clk_audio_pll_compute_qdpad(parent_rate, rate, NULL,
					  &tmp_div, &tmp_ext_div);
	if (!ret)
		apad_ck->qdpad = AUDIO_PLL_DIV2QD(tmp_div, tmp_ext_div);

	return ret;
}

static const struct clk_ops audio_pll_pad_ops = {
	.enable = clk_audio_pll_pad_enable,
	.disable = clk_audio_pll_pad_disable,
	.recalc_rate = clk_audio_pll_pad_recalc_rate,
	.round_rate = clk_audio_pll_pad_round_rate,
	.set_rate = clk_audio_pll_pad_set_rate,
};

void __init of_sama5d2_clk_audio_pll_pad_setup(struct device_node *np,
					       struct at91_pmc *pmc)
{
	struct clk_audio_pad *apad_ck;
	struct clk *clk = NULL;
	struct clk_init_data init;
	const char *parent_name;
	const char *name = np->name;

	parent_name = of_clk_get_parent_name(np, 0);

	of_property_read_string(np, "clock-output-names", &name);

	apad_ck = kzalloc(sizeof(*apad_ck), GFP_KERNEL);
	if (!apad_ck)
		return;

	init.name = name;
	init.ops = &audio_pll_pad_ops;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = 1;
	init.flags = CLK_SET_RATE_GATE |
		     CLK_SET_PARENT_GATE | CLK_SET_RATE_PARENT;

	apad_ck->hw.init = &init;
	apad_ck->pmc = pmc;

	clk = clk_register(NULL, &apad_ck->hw);
	if (IS_ERR(clk))
		kfree(apad_ck);
	else
		of_clk_add_provider(np, of_clk_src_simple_get, clk);

	return;
}
