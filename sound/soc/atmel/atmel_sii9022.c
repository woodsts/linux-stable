/*
 * atmel_sii9022 - Atmel ASoC driver for boards with HDMI encoder SiI9022
 *
 * Copyright (C) 2014 Atmel
 *
 * Author: Bo Shen <voice.shen@atmel.com>
 *
 * GPLv2 or later
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <sound/soc.h>

#include "atmel_ssc_dai.h"

#define MCLK_RATE 12000000

static struct clk *mclk;

static const struct snd_soc_dapm_widget atmel_asoc_sii9022_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("TX"),
};

static int atmel_asoc_sii9022_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct clk *mclk;
	unsigned long mclk_rate;
	int bclk_rate, bclk_div, period;
	int ret;

	mclk = clk_get(NULL, "mck");
	if (IS_ERR(mclk)) {
		pr_err("%s - Failed to get mck\n", __func__);
		return -ENODEV;
	}

	mclk_rate = clk_get_rate(mclk);

	bclk_rate = snd_soc_params_to_bclk(params);
	if (bclk_rate < 0) {
		pr_err("%s - Failed to get bclk\n", __func__);
		return -EINVAL;
	}

	bclk_div = (mclk_rate / 4) / bclk_rate;
	ret = snd_soc_dai_set_clkdiv(cpu_dai, ATMEL_SSC_CMR_DIV, bclk_div);
	if (ret < 0) {
		pr_err("%s - Failed to set cpu dai clk divider\n", __func__);
		return ret;
	}

	period = (bclk_rate / params_rate(params)) / 2 - 1;
	ret = snd_soc_dai_set_clkdiv(cpu_dai, ATMEL_SSC_TCMR_PERIOD, period);
	if (ret < 0) {
		pr_err("%s - Failed to set cpu dai lrclk divider\n", __func__);
		return ret;
	}
	ret = snd_soc_dai_set_clkdiv(cpu_dai, ATMEL_SSC_RCMR_PERIOD, period);
	if (ret < 0) {
		pr_err("%s - failed to set sii9022 codec PLL.", __func__);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops atmel_asoc_sii9022_ops = {
	.hw_params = atmel_asoc_sii9022_hw_params,
};

static int atmel_set_bias_level(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	if (dapm->bias_level == SND_SOC_BIAS_STANDBY) {
		switch (level) {
		case SND_SOC_BIAS_PREPARE:
			clk_prepare_enable(mclk);
			break;
		case SND_SOC_BIAS_OFF:
			clk_disable_unprepare(mclk);
			break;
		default:
			break;
		}
	}

	return 0;
};

static struct snd_soc_dai_link atmel_asoc_sii9022_dailink = {
	.name = "SiI9022",
	.stream_name = "SiI9022 PCM",
	.codec_dai_name = "hdmi-hifi",
	.dai_fmt = SND_SOC_DAIFMT_I2S
		| SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS,
	.ops = &atmel_asoc_sii9022_ops,
};

static struct snd_soc_card atmel_asoc_sii9022_card = {
	.name = "atmel_asoc_sii9022",
	.owner = THIS_MODULE,
	.set_bias_level = atmel_set_bias_level,
	.dai_link = &atmel_asoc_sii9022_dailink,
	.num_links = 1,
	.dapm_widgets = atmel_asoc_sii9022_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(atmel_asoc_sii9022_dapm_widgets),
	.fully_routed = true,
};

static int atmel_asoc_sii9022_dt_init(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *codec_np, *cpu_np;
	struct snd_soc_card *card = &atmel_asoc_sii9022_card;
	struct snd_soc_dai_link *dailink = &atmel_asoc_sii9022_dailink;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "only device tree supported\n");
		return -EINVAL;
	}

	ret = snd_soc_of_parse_card_name(card, "atmel,model");
	if (ret) {
		dev_err(&pdev->dev, "failed to parse card name\n");
		return ret;
	}

	ret = snd_soc_of_parse_audio_routing(card, "atmel,audio-routing");
	if (ret) {
		dev_err(&pdev->dev, "failed to parse audio routing\n");
		return ret;
	}

	cpu_np = of_parse_phandle(np, "atmel,ssc-controller", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "failed to get dai and pcm info\n");
		ret = -EINVAL;
		return ret;
	}
	dailink->cpu_of_node = cpu_np;
	dailink->platform_of_node = cpu_np;
	of_node_put(cpu_np);

	codec_np = of_parse_phandle(np, "atmel,audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "failed to get codec info\n");
		ret = -EINVAL;
		return ret;
	}
	dailink->codec_of_node = codec_np;
	of_node_put(codec_np);

	return 0;
}

static int atmel_asoc_sii9022_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &atmel_asoc_sii9022_card;
	struct snd_soc_dai_link *dailink = &atmel_asoc_sii9022_dailink;
	int id, ret;

	card->dev = &pdev->dev;
	ret = atmel_asoc_sii9022_dt_init(pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to init dt info\n");
		return ret;
	}

	id = of_alias_get_id((struct device_node *)dailink->cpu_of_node, "ssc");
	ret = atmel_ssc_set_audio(id);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to set SSC %d for audio\n", id);
		return ret;
	}

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed\n");
		goto err_set_audio;
	}

	return 0;

err_set_audio:
	atmel_ssc_put_audio(id);
	return ret;
}

static int atmel_asoc_sii9022_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct snd_soc_dai_link *dailink = &atmel_asoc_sii9022_dailink;
	int id;

	id = of_alias_get_id((struct device_node *)dailink->cpu_of_node, "ssc");

	snd_soc_unregister_card(card);
	atmel_ssc_put_audio(id);

	return 0;
}

static const struct of_device_id atmel_asoc_sii9022_dt_ids[] = {
	{ .compatible = "atmel,asoc-sii9022", },
	{ }
};

static struct platform_driver atmel_asoc_sii9022_driver = {
	.driver = {
		.name = "atmel-sii9022-audio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(atmel_asoc_sii9022_dt_ids),
	},
	.probe = atmel_asoc_sii9022_probe,
	.remove = atmel_asoc_sii9022_remove,
};
module_platform_driver(atmel_asoc_sii9022_driver);

/* Module information */
MODULE_AUTHOR("Bo Shen <voice.shen@atmel.com>");
MODULE_DESCRIPTION("ALSA SoC machine driver for Atmel EK with SiI9022");
MODULE_LICENSE("GPL");
