/*
 * Atmel SSC driver
 *
 * Copyright (C) 2007 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/atmel-ssc.h>
#include <linux/slab.h>
#include <linux/module.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_data/dma-atmel.h>

/* Serialize access to ssc_list and user count */
static DEFINE_SPINLOCK(user_lock);
static LIST_HEAD(ssc_list);

struct ssc_device *ssc_request(unsigned int ssc_num)
{
	int ssc_valid = 0;
	struct ssc_device *ssc;

	spin_lock(&user_lock);
	list_for_each_entry(ssc, &ssc_list, list) {
		if (ssc->pdev->dev.of_node) {
			if (of_alias_get_id(ssc->pdev->dev.of_node, "ssc")
				== ssc_num) {
				ssc_valid = 1;
				break;
			}
		} else if (ssc->pdev->id == ssc_num) {
			ssc_valid = 1;
			break;
		}
	}

	if (!ssc_valid) {
		spin_unlock(&user_lock);
		pr_err("ssc: ssc%d platform device is missing\n", ssc_num);
		return ERR_PTR(-ENODEV);
	}

	if (ssc->user) {
		spin_unlock(&user_lock);
		dev_dbg(&ssc->pdev->dev, "module busy\n");
		return ERR_PTR(-EBUSY);
	}
	ssc->user++;
	spin_unlock(&user_lock);

	clk_enable(ssc->clk);

	return ssc;
}
EXPORT_SYMBOL(ssc_request);

void ssc_free(struct ssc_device *ssc)
{
	spin_lock(&user_lock);
	if (ssc->user) {
		ssc->user--;
		clk_disable(ssc->clk);
	} else {
		dev_dbg(&ssc->pdev->dev, "device already free\n");
	}
	spin_unlock(&user_lock);
}
EXPORT_SYMBOL(ssc_free);

static struct atmel_ssc_platform_data at91rm9200_config = {
	.use_dma = 0,
};

static struct atmel_ssc_platform_data at91sam9g45_config = {
	.use_dma = 1,
};

static const struct platform_device_id atmel_ssc_devtypes[] = {
	{
		.name = "at91rm9200_ssc",
		.driver_data = (unsigned long) &at91rm9200_config,
	}, {
		.name = "at91sam9g45_ssc",
		.driver_data = (unsigned long) &at91sam9g45_config,
	}, {
		/* sentinel */
	}
};

#ifdef CONFIG_OF
static const struct of_device_id atmel_ssc_dt_ids[] = {
	{
		.compatible = "atmel,at91rm9200-ssc",
		.data = &at91rm9200_config,
	}, {
		.compatible = "atmel,at91sam9g45-ssc",
		.data = &at91sam9g45_config,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, atmel_ssc_dt_ids);
#endif

static inline const struct atmel_ssc_platform_data * __init
	atmel_ssc_get_driver_data(struct platform_device *pdev)
{
	if (pdev->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_node(atmel_ssc_dt_ids, pdev->dev.of_node);
		if (match == NULL)
			return NULL;
		return match->data;
	}

	return (struct atmel_ssc_platform_data *)
		platform_get_device_id(pdev)->driver_data;
}

static int atmel_dma_of_init(struct device_node *np,
                            struct at_dma_slave *atslave)
{
       struct of_phandle_args  dma_spec;
       struct device_node      *dmac_np;
       struct platform_device  *dmac_pdev;
       const __be32            *nbcells;
       int                     ret;

       ret = of_parse_phandle_with_args(np, "dma", "#dma-cells", 0, &dma_spec);
       if (ret || !dma_spec.np) {
               pr_err("%s: can't parse dma property (%d)\n", np->full_name, ret);
               goto err0;
       }
       dmac_np = dma_spec.np;

       /* check property format */
       nbcells = of_get_property(dmac_np, "#dma-cells", NULL);
       if (!nbcells) {
               pr_err("%s: #dma-cells property is required\n", np->full_name);
               ret = -EINVAL;
               goto err1;
       }

       if (dma_spec.args_count != be32_to_cpup(nbcells)
               || dma_spec.args_count != 1) {
               pr_err("%s: wrong #dma-cells for %s\n",
                       np->full_name, dmac_np->full_name);
               ret = -EINVAL;
               goto err1;
       }

       /* retreive DMA controller information */
       dmac_pdev = of_find_device_by_node(dmac_np);
       if (!dmac_pdev) {
               pr_err("%s: unable to find pdev from DMA controller\n",
                       dmac_np->full_name);
               ret = -EINVAL;
               goto err1;
       }

       /* now fill in the at_dma_slave structure */
       atslave->dma_dev = &dmac_pdev->dev;
       atslave->cfg = dma_spec.args[0];

err1:
       of_node_put(dma_spec.np);
err0:
       pr_debug("%s exited with status %d\n", __func__, ret);
       return ret;
}

static int ssc_probe(struct platform_device *pdev)
{
	struct resource *regs;
	struct ssc_device *ssc;
	const struct atmel_ssc_platform_data *plat_dat;
	struct pinctrl *pinctrl;
	struct at_dma_slave *atslave;
	struct device_node *np = pdev->dev.of_node;

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "failed to request pinctrl\n");
		return PTR_ERR(pinctrl);
	}

	ssc = devm_kzalloc(&pdev->dev, sizeof(struct ssc_device), GFP_KERNEL);
	if (!ssc) {
		dev_err(&pdev->dev, "out of memory\n");
		return -ENOMEM;
	}

	ssc->pdev = pdev;

	plat_dat = atmel_ssc_get_driver_data(pdev);
	if (!plat_dat) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -ENODEV;
	}
	ssc->pdata = (struct atmel_ssc_platform_data *)plat_dat;

	if (plat_dat->use_dma) {
		atslave = devm_kzalloc(&pdev->dev, sizeof(struct at_dma_slave),
				GFP_KERNEL);
		if (!atslave) {
			dev_err(&pdev->dev, "failed alloc memory for dma\n");
			devm_kfree(&pdev->dev, atslave);
			return -ENOMEM;
		}

		if (atmel_dma_of_init(np, atslave)) {
			dev_err(&pdev->dev, "could not find DMA parameters\n");
			devm_kfree(&pdev->dev, atslave);
			return -EINVAL;
		}

		pdev->dev.platform_data = atslave;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "no mmio resource defined\n");
		return -ENXIO;
	}

	ssc->regs = devm_request_and_ioremap(&pdev->dev, regs);
	if (!ssc->regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -EINVAL;
	}

	ssc->phybase = regs->start;

	ssc->clk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(ssc->clk)) {
		dev_err(&pdev->dev, "no pclk clock defined\n");
		return -ENXIO;
	}

	/* disable all interrupts */
	clk_enable(ssc->clk);
	ssc_writel(ssc->regs, IDR, ~0UL);
	ssc_readl(ssc->regs, SR);
	clk_disable(ssc->clk);

	ssc->irq = platform_get_irq(pdev, 0);
	if (!ssc->irq) {
		dev_err(&pdev->dev, "could not get irq\n");
		return -ENXIO;
	}

	spin_lock(&user_lock);
	list_add_tail(&ssc->list, &ssc_list);
	spin_unlock(&user_lock);

	platform_set_drvdata(pdev, ssc);

	dev_info(&pdev->dev, "Atmel SSC device at 0x%p (irq %d)\n",
			ssc->regs, ssc->irq);

	return 0;
}

static int __devexit ssc_remove(struct platform_device *pdev)
{
	struct ssc_device *ssc = platform_get_drvdata(pdev);

	spin_lock(&user_lock);
	list_del(&ssc->list);
	spin_unlock(&user_lock);

	return 0;
}

static struct platform_driver ssc_driver = {
	.driver		= {
		.name		= "ssc",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(atmel_ssc_dt_ids),
	},
	.id_table	= atmel_ssc_devtypes,
	.probe		= ssc_probe,
	.remove		= __devexit_p(ssc_remove),
};
module_platform_driver(ssc_driver);

MODULE_AUTHOR("Hans-Christian Egtvedt <hcegtvedt@atmel.com>");
MODULE_DESCRIPTION("SSC driver for Atmel AVR32 and AT91");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ssc");
