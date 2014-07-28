#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include "sdhci-pltfm.h"

struct sdhci_at91_priv {
	struct clk *clk;
};

static const struct sdhci_ops sdhci_at91_sama5d2_ops = {
	.set_clock		= sdhci_set_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.reset			= sdhci_reset,
	.set_uhs_signaling	= sdhci_set_uhs_signaling,
};

static const struct sdhci_pltfm_data soc_data_sama5d2 = {
	.ops = &sdhci_at91_sama5d2_ops,
};

static const struct of_device_id sdhci_at91_dt_match[] = {
	{ .compatible = "atmel,sama5d2-sdhci", .data = &soc_data_sama5d2 },
	{}
};

static int sdhci_at91_probe(struct platform_device *pdev)
{
	const struct of_device_id	*match;
	const struct sdhci_pltfm_data	*soc_data;
	struct sdhci_host		*host;
	struct sdhci_pltfm_host		*pltfm_host;
	struct sdhci_at91_priv		*priv;
	int				ret;

	match = of_match_device(sdhci_at91_dt_match, &pdev->dev);
	if (!match)
		return -EINVAL;
	soc_data = match->data;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "unable to allocate private data\n");
		return -ENOMEM;
	}

	priv->clk = devm_clk_get(&pdev->dev, NULL);

	host = sdhci_pltfm_init(pdev, soc_data, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);
	
	pltfm_host = sdhci_priv(host);
	pltfm_host->priv = priv;

	if (!IS_ERR(priv->clk))
		clk_prepare_enable(priv->clk);

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_sdhci_add;

	ret = sdhci_add_host(host);
	if (ret)
		goto err_sdhci_add;

	return 0;

err_sdhci_add:
	if (!IS_ERR(priv->clk))
		clk_disable_unprepare(priv->clk);
	sdhci_pltfm_free(pdev);
	return ret;
}

static int sdhci_at91_remove(struct platform_device *pdev)
{
	struct sdhci_host	*host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host	*pltfm_host = sdhci_priv(host);
	struct sdhci_at91_priv	*priv = pltfm_host->priv;

	sdhci_pltfm_unregister(pdev);

	if (!IS_ERR(priv->clk))
		clk_disable_unprepare(priv->clk);

	return 0;
}

static struct platform_driver sdhci_at91_driver = {
	.driver		= {
		.name	= "sdhci-at91",
		.owner	= THIS_MODULE,
		.of_match_table = sdhci_at91_dt_match,
		.pm	= SDHCI_PLTFM_PMOPS,
	},
	.probe		= sdhci_at91_probe,
	.remove		= sdhci_at91_remove,
};

module_platform_driver(sdhci_at91_driver);

MODULE_DESCRIPTION("SDHCI driver for at91");
MODULE_AUTHOR("Ludovic Desroches <ludovic.desroches@atmel.com>");
MODULE_LICENSE("GPL v2");
