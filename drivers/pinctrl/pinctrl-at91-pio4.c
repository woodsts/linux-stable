/*
 * Driver for the Atmel PIO4 controller
 *
 * Copyright (C) 2015 Atmel Corporation
 *
 * Author: Ludovic Desroches <ludovic.desroches@atmel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include "core.h"
#include "pinctrl-utils.h"


#define ATMEL_PIO4_BANK_OFFSET	0x0040
#define ATMEL_PIO4_MSKR		0x0000
#define ATMEL_PIO4_CFGR		0x0004
#define		ATMEL_PIO4_CFGR_FUNC_MASK	(0x7)
#define		ATMEL_PIO4_DIR_MASK		(0x1 << 8)
#define		ATMEL_PIO4_PUEN_MASK		(0x1 << 9)
#define		ATMEL_PIO4_PDEN_MASK		(0x1 << 10)
#define		ATMEL_PIO4_IFEN_MASK		(0x1 << 12)
#define		ATMEL_PIO4_IFSCEN_MASK		(0x1 << 13)
#define		ATMEL_PIO4_OPD_MASK		(0x1 << 14)
#define		ATMEL_PIO4_SCHMITT_MASK		(0x1 << 15)
#define ATMEL_PIO4_PDSR		0x0008
#define ATMEL_PIO4_LOCKSR	0x000C
#define ATMEL_PIO4_SODR		0x0010
#define ATMEL_PIO4_CODR		0x0014
#define ATMEL_PIO4_ODSR		0x0018
#define ATMEL_PIO4_IER		0x0020
#define ATMEL_PIO4_IDR		0x0024
#define ATMEL_PIO4_IMR		0x0028
#define ATMEL_PIO4_ISR		0x002C
#define ATMEL_PIO4_IOFR		0x003C

#define ATMEL_GET_IOSET(pin)	((pin >> 16) & 0xff)


struct atmel_pinctrl {
	struct pinctrl_dev 	*pctrl;
	struct regmap		*regmap_base;
	unsigned int 		nbanks;
	unsigned int		npins_per_bank;
	struct atmel_group	*groups;
	unsigned int		ngroups;
	const char* const 	*funcs;
	unsigned int		nfuncs;
	struct pinctrl_desc	*pctrl_desc;
};

struct atmel_pinctrl_data {
	const struct pinctrl_ops	*pctlops;
	const struct pinmux_ops		*pmxops;
	const struct pinconf_ops	*confops;
	bool				complex_pin_desc;
	unsigned			nbanks;
	struct atmel_pinctrl 		*atmel_pinctrl;
};

struct atmel_group {
	const char *name;
	u32 *pins;
	unsigned npins;
};

static int atmel_pio4_pin_config_read(struct pinctrl_dev *pctldev,
				      unsigned pin_id, u32 *res)
{
	struct atmel_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	unsigned	bank, pin;
	u32		mask;

	bank = pin_id / pctrl->npins_per_bank;
	pin = pin_id % pctrl->npins_per_bank;
	mask = 1 << pin;
	dev_vdbg(pctldev->dev, "%s: bank %u, pin %u\n", __func__, bank, pin);

	regmap_write(pctrl->regmap_base,
		     bank * ATMEL_PIO4_BANK_OFFSET + ATMEL_PIO4_MSKR,
		     mask);

	return regmap_read(pctrl->regmap_base,
			   bank * ATMEL_PIO4_BANK_OFFSET + ATMEL_PIO4_CFGR,
			   res);
}

static void atmel_pio4_pin_config_write(struct pinctrl_dev *pctldev,
					unsigned pin_id, u32 conf)
{
	struct atmel_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	unsigned	bank, pin;
	u32		mask;

	bank = pin_id / pctrl->npins_per_bank;
	pin = pin_id % pctrl->npins_per_bank;
	mask = 1 << pin;
	dev_vdbg(pctldev->dev, "%s: bank %u, pin %u\n", __func__, bank, pin);

	regmap_write(pctrl->regmap_base,
		     bank * ATMEL_PIO4_BANK_OFFSET + ATMEL_PIO4_MSKR,
		     mask);

	regmap_write(pctrl->regmap_base,
		     bank * ATMEL_PIO4_BANK_OFFSET + ATMEL_PIO4_CFGR,
		     conf);
}

/* ----- pinctrl part ----- */

static int atmel_pctl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct atmel_pinctrl *atmel_pinctrl = pinctrl_dev_get_drvdata(pctldev);

	return atmel_pinctrl->ngroups;
}

static const char *atmel_pctl_get_group_name(struct pinctrl_dev *pctldev,
					     unsigned selector)
{
	struct atmel_pinctrl *atmel_pinctrl = pinctrl_dev_get_drvdata(pctldev);

	return atmel_pinctrl->groups[selector].name;
}

static int atmel_pctl_get_group_pins(struct pinctrl_dev *pctldev,
				     unsigned selector, const unsigned **pins,
				     unsigned *num_pins)
{
	struct atmel_pinctrl *atmel_pinctrl = pinctrl_dev_get_drvdata(pctldev);

	*pins = atmel_pinctrl->groups[selector].pins;
	*num_pins = atmel_pinctrl->groups[selector].npins;

	return 0;
}

static const struct pinctrl_ops atmel_pctlops = {
	.get_groups_count = atmel_pctl_get_groups_count,
	.get_group_name = atmel_pctl_get_group_name,
	.get_group_pins = atmel_pctl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinctrl_utils_dt_free_map,
};


/* ----- pinmux part ----- */

static int atmel_pmux_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct atmel_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->nfuncs;
}

static const char *atmel_pmux_get_function_name(struct pinctrl_dev *pctldev,
						unsigned selector)
{
	struct atmel_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->funcs[selector];
}

static int atmel_pmux_get_function_groups(struct pinctrl_dev *pctldev,
					   unsigned selector,
					   const char * const **groups,
					   unsigned * const num_groups)
{
	return -EINVAL;
}

static int atmel_pio4_set_mux(struct pinctrl_dev *pctldev,
			      unsigned function,
			      unsigned group)
{
	struct atmel_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	int ret, i;
	u32 conf;
	unsigned *pin;

	dev_dbg(pctldev->dev, "enable function %s group %s\n",
		pctrl->funcs[function], pctrl->groups[group].name);

	/*
	 * Unfortunately, we can't set the function for several pins in one
	 * operation. Inside a group, pins can have different configurations.
	 */
	pin = pctrl->groups[group].pins;
	for (i = 0 ; i < pctrl->groups[group].npins ; i++) {
		ret = atmel_pio4_pin_config_read(pctldev, pin[i], &conf);
		conf &= (~ATMEL_PIO4_CFGR_FUNC_MASK);
		conf |= (function & ATMEL_PIO4_CFGR_FUNC_MASK);
		dev_dbg(pctldev->dev, "pin: %u, conf: 0x%08x\n", pin[i], conf);
		atmel_pio4_pin_config_write(pctldev, pin[i], conf);
	}

	return 0;
}

static const struct pinmux_ops atmel_pio4_pmxops = {
	.mux_per_pin = true,
	.get_functions_count = atmel_pmux_get_functions_count,
	.get_function_name = atmel_pmux_get_function_name,
	.get_function_groups = atmel_pmux_get_function_groups,
	.set_mux = atmel_pio4_set_mux,
	/* TODO */
	.gpio_request_enable = NULL,
	.gpio_disable_free = NULL,
	.gpio_set_direction = NULL,
};


/* ----- pinconf part ----- */

static int atmel_pio4_pin_config_get(struct pinctrl_dev *pctldev,
				     unsigned pin,
				     unsigned long *config)
{
	unsigned int arg = 0;
	unsigned int param = pinconf_to_config_param(*config);
	u32 res;
	int ret;

	ret = atmel_pio4_pin_config_read(pctldev, pin, &res);
	if (ret)
		return -EIO;

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
		if (!(res & ATMEL_PIO4_PUEN_MASK))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if ((res & ATMEL_PIO4_PUEN_MASK)
		    || (!(res & ATMEL_PIO4_PDEN_MASK)))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		if ((res & ATMEL_PIO4_PUEN_MASK)
		    || ((res & ATMEL_PIO4_PDEN_MASK)))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		if (!(res & ATMEL_PIO4_OPD_MASK))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		if (!(res & ATMEL_PIO4_SCHMITT_MASK))
			return -EINVAL;
		arg = 1;
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int atmel_pio4_pin_config_set(struct pinctrl_dev *pctldev,
				     unsigned pin_id,
				     unsigned long *configs,
				     unsigned num_configs)
{
	struct atmel_pinctrl	*pctrl = pinctrl_dev_get_drvdata(pctldev);
	int 			i, ret;
	u32			mask, conf = 0;
	unsigned		bank, pin;

	ret = atmel_pio4_pin_config_read(pctldev, pin_id, &conf);
	if (ret)
		return -EIO;

	for (i = 0; i < num_configs; i++) {
		unsigned int param = pinconf_to_config_param(configs[i]);
		unsigned int arg = pinconf_to_config_argument(configs[i]);

		dev_dbg(pctldev->dev, "%s: pin=%u, config=0x%lx\n",
			__func__, pin_id, configs[i]);

		switch(param) {
		case PIN_CONFIG_BIAS_DISABLE:
			conf &= (~ATMEL_PIO4_PUEN_MASK);
			conf &= (~ATMEL_PIO4_PDEN_MASK);
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			conf |= ATMEL_PIO4_PUEN_MASK;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			conf |= ATMEL_PIO4_PDEN_MASK;
			break;
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			if (arg == 0)
				conf &= (~ATMEL_PIO4_OPD_MASK);
			else
				conf |= ATMEL_PIO4_OPD_MASK;
			break;
		case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
			if (arg == 0)
				conf |= ATMEL_PIO4_SCHMITT_MASK;
			else
				conf &= (~ATMEL_PIO4_SCHMITT_MASK);
			break;
		case PIN_CONFIG_OUTPUT:
			conf |= ATMEL_PIO4_DIR_MASK;
			bank = pin_id / pctrl->npins_per_bank;
			pin = pin_id % pctrl->npins_per_bank;
			mask = 1 << pin;

			if (arg == 0)
				regmap_write(pctrl->regmap_base,
				     bank * ATMEL_PIO4_BANK_OFFSET + ATMEL_PIO4_CODR,
				     mask);
			else
				regmap_write(pctrl->regmap_base,
				     bank * ATMEL_PIO4_BANK_OFFSET + ATMEL_PIO4_SODR,
				     mask);
			break;
		default:
			dev_warn(pctldev->dev,
				 "unsupported configuration parameter: %u\n",
				 param);
			continue;
		}
	}

	atmel_pio4_pin_config_write(pctldev, pin_id, conf);

	return 0;
}

static void atmel_pio4_pin_config_dbg_show(struct pinctrl_dev *pctldev,
					   struct seq_file *s, unsigned pin_id)
{
	/* TODO */

	return;
}

static const struct pinconf_ops atmel_pio4_confops = {
	.is_generic = true,
	.pin_config_get = atmel_pio4_pin_config_get,
	.pin_config_set = atmel_pio4_pin_config_set,
	.pin_config_dbg_show = atmel_pio4_pin_config_dbg_show,
};

static struct pinctrl_desc atmel_pinctrl_desc = {
	.name = "atmel_pinctrl",
	.owner = THIS_MODULE,
};

static const char * const atmel_pio4_functions[] = {
	"GPIO", "A", "B", "C", "D", "E", "F", "G"
};

/*
 * Set only values which are specific to PIO4. nbanks is related to the device
 * so take this information from atmel_pinctrl_data. Groups will be parsed
 * from the device tree.
 */
struct atmel_pinctrl atmel_pio4_pinctrl = {
	.npins_per_bank	= 32,
	.funcs		= atmel_pio4_functions,
	.nfuncs		= ARRAY_SIZE(atmel_pio4_functions),
};

static const struct atmel_pinctrl_data atmel_sama5d2_pctrl_data = {
	.pctlops		= &atmel_pctlops,
	.pmxops			= &atmel_pio4_pmxops,
	.confops		= &atmel_pio4_confops,
	.complex_pin_desc	= true,
	.nbanks 		= 4,
	.atmel_pinctrl 		= &atmel_pio4_pinctrl,
};

static const struct of_device_id atmel_pinctrl_of_match[] = {
	{
		.compatible = "atmel,sama5d2-pinctrl",
		.data = &atmel_sama5d2_pctrl_data,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, atmel_pinctrl_of_match);

/*
 * Groups are used to check if all the pins used are from the same ioset and
 * for readability. The controller has no knowledge about groups, they are a
 * virtual concept. Groups are defined in a subnode called group_defs.
 */
static int atmel_pinctrl_parse_groups(struct platform_device *pdev,
				      struct atmel_pinctrl *atmel_pinctrl)
{
	struct device_node	*np = pdev->dev.of_node, *groups_np, *group_np;
	struct atmel_group	*group;
	int			ret, i, ioset;
	u32			pin;

	groups_np = of_find_node_by_name(np, "group_defs");
	if (!groups_np) {
		dev_err(&pdev->dev, "group_defs node not found\n");
		return -EINVAL;
	}

	atmel_pinctrl->ngroups = of_get_child_count(groups_np);
	if (!atmel_pinctrl->ngroups)
		return -EINVAL;

	dev_dbg(&pdev->dev, "%u group(s)\n", atmel_pinctrl->ngroups);

	atmel_pinctrl->groups = devm_kzalloc(&pdev->dev,
		sizeof(*atmel_pinctrl->groups) * atmel_pinctrl->ngroups,
		GFP_KERNEL);
	if (!atmel_pinctrl->groups)
		return -ENOMEM;

	group = atmel_pinctrl->groups;
	for_each_child_of_node(groups_np, group_np) {
		ioset = -1;
		group->name = group_np->name;
		dev_dbg(&pdev->dev, "%s:\n", group->name);
		group->npins = of_property_count_u32_elems(group_np, "pins");
		if (!group->npins) {
			dev_err(&pdev->dev, "no pins for group %s\n", group->name);
			return -EINVAL;
		}
		group->pins = devm_kzalloc(&pdev->dev,
					   sizeof(*group->pins) * group->npins,
					   GFP_KERNEL);
		if (!group->pins) {
			dev_err(&pdev->dev,
				"can't allocate pin table for group %s\n",
				group->name);
			return -ENOMEM;
		}
		for (i = 0; i < group->npins; i++) {
			ret = of_property_read_u32_index(group_np, "pins", i, &pin);
			if (ret) {
				dev_err(&pdev->dev,
					"can't get pins for group %s\n",
				        group->name);
				return ret;
			}
			group->pins[i] = pin & PINCTRL_PIN_MASK;
			dev_dbg(&pdev->dev, "%u\n", group->pins[i]);
			/*
			 * All the pins of a group should have the same ioset
			 * because validation has been done in this way. It
			 * means you can have an unexpected behaviour if you
			 * mix signals from several iosets.
			 */
			if (ioset < 0)
				ioset = ATMEL_GET_IOSET(pin);
			if (ATMEL_GET_IOSET(pin) != ioset)
				dev_warn(&pdev->dev,
				         "/!\\ pins from group %s are not using the same ioset /!\\\n",
					 group->name);
		}
		group++;
	}

	return 0;
}

static struct atmel_pinctrl *atmel_pinctrl_get_data(struct platform_device *pdev) {
	struct atmel_pinctrl		*atmel_pinctrl;
	const struct of_device_id	*match;
	const struct atmel_pinctrl_data *data;

	atmel_pinctrl = devm_kzalloc(&pdev->dev, sizeof(*atmel_pinctrl),
				     GFP_KERNEL);
	if (!atmel_pinctrl) {
		dev_err(&pdev->dev, "can't allocate atmel_pinctrl structure\n");
		return ERR_PTR(-ENOMEM);
	}

	match = of_match_node(atmel_pinctrl_of_match, pdev->dev.of_node);
	if (!match) {
		dev_err(&pdev->dev, "unknow compatible string\n");
		return ERR_PTR(-ENODEV);
	}

	data = match->data;
	atmel_pinctrl = data->atmel_pinctrl;
	atmel_pinctrl->nbanks = data->nbanks;
	atmel_pinctrl_desc.pctlops = data->pctlops;
	atmel_pinctrl_desc.pmxops = data->pmxops;
	atmel_pinctrl_desc.confops = data->confops;
	atmel_pinctrl_desc.complex_pin_desc = data->complex_pin_desc;
	atmel_pinctrl_desc.npins = data->nbanks * atmel_pinctrl->npins_per_bank;
	dev_dbg(&pdev->dev, "%u pins: %u banks, %u pins per bank\n",
		atmel_pinctrl_desc.npins, atmel_pinctrl->nbanks,
		atmel_pinctrl->npins_per_bank);

	return atmel_pinctrl;
}

static int atmel_pinctrl_probe(struct platform_device *pdev)
{
	struct device_node	*np = pdev->dev.of_node, *regmap_np;
	struct atmel_pinctrl	*atmel_pinctrl;
	struct pinctrl_pin_desc	*pin_desc;
	int 			ret, i;

	atmel_pinctrl = atmel_pinctrl_get_data(pdev);
	if (!atmel_pinctrl)
		return PTR_ERR(atmel_pinctrl);

	pin_desc = devm_kzalloc(&pdev->dev,
		sizeof(*atmel_pinctrl_desc.pins) * atmel_pinctrl_desc.npins,
		GFP_KERNEL);
	if (!pin_desc) {
		dev_err(&pdev->dev, "can't allocate pins structure\n");
		return -ENOMEM;
	}

	/* Pin naming convention is P(bank_name)(bank_pin_number). */
	for (i = 0 ; i < atmel_pinctrl_desc.npins; i++) {
		pin_desc[i].number = i;
		pin_desc[i].name = kasprintf(GFP_KERNEL, "P%c%d",
			(i / atmel_pinctrl->npins_per_bank) + 'A',
			i % atmel_pinctrl->npins_per_bank);
	}
	atmel_pinctrl_desc.pins = pin_desc;

	ret = atmel_pinctrl_parse_groups(pdev, atmel_pinctrl);
	if (ret)
		return ret;

	/* Some registers are shared with the gpio controller driver. */
	regmap_np = of_parse_phandle(np, "atmel,pio_reg", 0);
	if (regmap_np) {
		atmel_pinctrl->regmap_base = syscon_node_to_regmap(regmap_np);
		if (IS_ERR(atmel_pinctrl->regmap_base)) {
			dev_err(&pdev->dev, "can't get regmap\n");
			return PTR_ERR(atmel_pinctrl->regmap_base);
		}
	} else {
		dev_err(&pdev->dev, "atmel,pio_reg property is missing\n");
		return -EINVAL;
	}

	atmel_pinctrl->pctrl = pinctrl_register(&atmel_pinctrl_desc, &pdev->dev, atmel_pinctrl);
	if (!atmel_pinctrl->pctrl) {
		dev_err(&pdev->dev, "pinctrl registration failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, atmel_pinctrl);

	dev_info(&pdev->dev, "atmel pinctrl initialized\n");

	return 0;
}

int atmel_pinctrl_remove(struct platform_device *pdev)
{
	struct atmel_pinctrl *atmel_pinctrl = platform_get_drvdata(pdev);

	pinctrl_unregister(atmel_pinctrl->pctrl);

	return 0;
}

static struct platform_driver atmel_pinctrl_driver = {
	.driver = {
		.name = "pinctrl-at91-pio4",
		.of_match_table = atmel_pinctrl_of_match,
	},
	.probe = atmel_pinctrl_probe,
	.remove = atmel_pinctrl_remove,
};

module_platform_driver(atmel_pinctrl_driver);

MODULE_AUTHOR(Ludovic Desroches <ludovic.desroches@atmel.com>);
MODULE_DESCRIPTION("Atmel PIO4 pinctrl driver");
MODULE_LICENSE("GPL v2");
