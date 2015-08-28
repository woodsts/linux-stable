/* Atmel ALSA SoC Audio Class D Amplifier (CLASSD) driver
 *
 * Copyright (C) 2015 Atmel
 *
 * Author: Songjun Wu <songjun.wu@atmel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or later
 * as published by the Free Software Foundation.
 */

#include <linux/of.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include "atmel-classd.h"

struct atmel_classd_pdata {
	bool non_overlap_enable;
	int non_overlap_time;
	int pwm_type;
};

struct atmel_classd {
	dma_addr_t phy_base;
	struct regmap *regmap;
	struct clk *pclk;
	struct clk *gclk;
	struct clk *aclk;
	int irq;
	const struct atmel_classd_pdata *pdata;
};

#ifdef CONFIG_OF
static const struct of_device_id atmel_classd_of_match[] = {
	{
		.compatible = "atmel,sama5d2-classd",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, atmel_classd_of_match);

static struct atmel_classd_pdata *atmel_classd_dt_init(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct atmel_classd_pdata *pdata;
	const char *pwm_type;
	int ret;

	if (!np) {
		dev_err(dev, "device node not found\n");
		return ERR_PTR(-EINVAL);
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_string(np, "atmel,pwm-type", &pwm_type);
	if ((ret == 0) && (strcmp(pwm_type, "diff") == 0))
		pdata->pwm_type = CLASSD_MR_PWMTYP_DIFF;
	else
		pdata->pwm_type = CLASSD_MR_PWMTYP_SINGLE;

	ret = of_property_read_u32(np,
			"atmel,non-overlap-time", &pdata->non_overlap_time);
	if (ret != 0)
		pdata->non_overlap_enable = false;
	else
		pdata->non_overlap_enable = true;

	return pdata;
}
#else
static inline struct atmel_classd_pdata *
atmel_classd_dt_init(struct device *dev)
{
	return ERR_PTR(-EINVAL);
}
#endif

#define ATMEL_CLASSD_RATES (SNDRV_PCM_RATE_8000 \
			| SNDRV_PCM_RATE_16000	| SNDRV_PCM_RATE_22050 \
			| SNDRV_PCM_RATE_32000	| SNDRV_PCM_RATE_44100 \
			| SNDRV_PCM_RATE_48000	| SNDRV_PCM_RATE_88200 \
			| SNDRV_PCM_RATE_96000)

static const struct snd_pcm_hardware atmel_classd_hw = {
	.info			= SNDRV_PCM_INFO_MMAP
				| SNDRV_PCM_INFO_MMAP_VALID
				| SNDRV_PCM_INFO_INTERLEAVED
				| SNDRV_PCM_INFO_RESUME
				| SNDRV_PCM_INFO_PAUSE,
	.formats		= (SNDRV_PCM_FMTBIT_S16_LE),
	.rates			= ATMEL_CLASSD_RATES,
	.rate_min		= 8000,
	.rate_max		= 96000,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= 64 * 1024,
	.period_bytes_min	= 256,
	.period_bytes_max	= 32 * 1024,
	.periods_min		= 2,
	.periods_max		= 256,
};

#define ATMEL_CLASSD_PREALLOC_BUF_SIZE  (64 * 1024)

/* cpu dai component */
static int atmel_classd_cpu_dai_startup(struct snd_pcm_substream *substream,
					struct snd_soc_dai *cpu_dai)
{
	struct atmel_classd *dd = snd_soc_dai_get_drvdata(cpu_dai);

	regmap_write(dd->regmap, CLASSD_THR, 0x0);

	return clk_prepare_enable(dd->pclk);
}

static void atmel_classd_cpu_dai_shutdown(struct snd_pcm_substream *substream,
					struct snd_soc_dai *cpu_dai)
{
	struct atmel_classd *dd = snd_soc_dai_get_drvdata(cpu_dai);

	clk_disable_unprepare(dd->pclk);
}

static const struct snd_soc_dai_ops atmel_classd_cpu_dai_ops = {
	.startup	= atmel_classd_cpu_dai_startup,
	.shutdown	= atmel_classd_cpu_dai_shutdown,
};

static struct snd_soc_dai_driver atmel_classd_cpu_dai = {
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = ATMEL_CLASSD_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &atmel_classd_cpu_dai_ops,
};

static const struct snd_soc_component_driver atmel_classd_cpu_dai_component = {
	.name = "atmel-classd",
};

/* platform */
static int
atmel_classd_platform_configure_dma(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct dma_slave_config *slave_config)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct atmel_classd *dd = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	int bits;

	bits = params_physical_width(params);
	if (bits != 16) {
		dev_err(rtd->platform->dev,
			"only supports 16-bit audio data\n");
		return -EINVAL;
	}

	slave_config->direction = DMA_MEM_TO_DEV;
	slave_config->dst_addr = dd->phy_base + CLASSD_THR;
	slave_config->dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	slave_config->dst_maxburst = 1;
	slave_config->src_maxburst = 1;
	slave_config->device_fc = false;

	return 0;
}

static const struct snd_dmaengine_pcm_config
atmel_classd_dmaengine_pcm_config = {
	.prepare_slave_config = atmel_classd_platform_configure_dma,
	.pcm_hardware = &atmel_classd_hw,
	.prealloc_buffer_size = ATMEL_CLASSD_PREALLOC_BUF_SIZE,
};

/* codec component */
static const char * const swap_text[] = {
	"No Swap", "Swap"
};

static SOC_ENUM_SINGLE_DECL(classd_swap_enum,
			CLASSD_INTPMR, CLASSD_INTPMR_SWAP_SHIFT,
			swap_text);

static const char * const mono_text[] = {
	"stereo", "mono"
};

static SOC_ENUM_SINGLE_DECL(classd_mono_enum,
			CLASSD_INTPMR, CLASSD_INTPMR_MONO_SHIFT,
			mono_text);

static const char * const mono_mode_text[] = {
	"mix", "sat", "left", "right"
};

static SOC_ENUM_SINGLE_DECL(classd_mono_mode_enum,
			CLASSD_INTPMR, CLASSD_INTPMR_MONO_MODE_SHIFT,
			mono_mode_text);

static const char * const deemp_text[] = {
	"disabled", "enabled"
};

static SOC_ENUM_SINGLE_DECL(classd_deemp_enum,
			CLASSD_INTPMR, CLASSD_INTPMR_DEEMP_SHIFT,
			deemp_text);

static const char * const eqcfg_bass_text[] = {
	"-12 dB", "-6 dB", "0 dB", "+6 dB", "+12 dB"
};

static const unsigned int eqcfg_bass_value[] = {
	CLASSD_INTPMR_EQCFG_B_CUT_12,
	CLASSD_INTPMR_EQCFG_B_CUT_6, CLASSD_INTPMR_EQCFG_FLAT,
	CLASSD_INTPMR_EQCFG_B_BOOST_6, CLASSD_INTPMR_EQCFG_B_BOOST_12
};

static SOC_VALUE_ENUM_SINGLE_DECL(classd_eqcfg_bass_enum,
		CLASSD_INTPMR, CLASSD_INTPMR_EQCFG_SHIFT, 0xf,
		eqcfg_bass_text, eqcfg_bass_value);

static const char * const eqcfg_medium_text[] = {
	"-8 dB", "-3 dB", "0 dB", "+3 dB", "+8 dB"
};

static const unsigned int eqcfg_medium_value[] = {
	CLASSD_INTPMR_EQCFG_M_CUT_8,
	CLASSD_INTPMR_EQCFG_M_CUT_3, CLASSD_INTPMR_EQCFG_FLAT,
	CLASSD_INTPMR_EQCFG_M_BOOST_3, CLASSD_INTPMR_EQCFG_M_BOOST_8
};

static SOC_VALUE_ENUM_SINGLE_DECL(classd_eqcfg_medium_enum,
		CLASSD_INTPMR, CLASSD_INTPMR_EQCFG_SHIFT, 0xf,
		eqcfg_medium_text, eqcfg_medium_value);

static const char * const eqcfg_treble_text[] = {
	"-12 dB", "-6 dB", "0 dB", "+6 dB", "+12 dB",
};

static const unsigned int eqcfg_treble_value[] = {
	CLASSD_INTPMR_EQCFG_T_CUT_12,
	CLASSD_INTPMR_EQCFG_T_CUT_6, CLASSD_INTPMR_EQCFG_FLAT,
	CLASSD_INTPMR_EQCFG_T_BOOST_6, CLASSD_INTPMR_EQCFG_T_BOOST_12
};

static SOC_VALUE_ENUM_SINGLE_DECL(classd_eqcfg_treble_enum,
		CLASSD_INTPMR, CLASSD_INTPMR_EQCFG_SHIFT, 0xf,
		eqcfg_treble_text, eqcfg_treble_value);

static int classd_get_eq_enum(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val, item;
	unsigned int reg_val;
	int ret;

	ret = snd_soc_component_read(component, e->reg, &reg_val);
	if (ret)
		return ret;
	val = (reg_val >> e->shift_l) & e->mask;

	if (strcmp(kcontrol->id.name, "EQ Bass") == 0) {
		if (val > 4)
			val = 0;
	} else if (strcmp(kcontrol->id.name, "EQ Medium") == 0) {
		if ((val > 8) || (val < 5))
			val = 0;
	} else if (strcmp(kcontrol->id.name, "EQ Treble") == 0) {
		if ((val > 12) || (val < 9))
			val = 0;
	} else
		return -EINVAL;

	item = snd_soc_enum_val_to_item(e, val);
	ucontrol->value.enumerated.item[0] = item;

	return 0;
}

static const DECLARE_TLV_DB_SCALE(classd_digital_tlv, -7800, 100, 1);

static const struct snd_kcontrol_new atmel_classd_snd_controls[] = {
SOC_SINGLE_TLV("Left Volume", CLASSD_INTPMR,
		CLASSD_INTPMR_ATTL_SHIFT, 78, 1, classd_digital_tlv),

SOC_SINGLE_TLV("Right Volume", CLASSD_INTPMR,
		CLASSD_INTPMR_ATTR_SHIFT, 78, 1, classd_digital_tlv),

SOC_ENUM("De-emphasis", classd_deemp_enum),

SOC_ENUM("Stereo", classd_mono_enum),

SOC_ENUM("Swap", classd_swap_enum),

SOC_ENUM("Mono Mode", classd_mono_mode_enum),

SOC_ENUM_EXT("EQ Bass", classd_eqcfg_bass_enum,
		classd_get_eq_enum, snd_soc_put_enum_double),

SOC_ENUM_EXT("EQ Medium", classd_eqcfg_medium_enum,
		classd_get_eq_enum, snd_soc_put_enum_double),

SOC_ENUM_EXT("EQ Treble", classd_eqcfg_treble_enum,
		classd_get_eq_enum, snd_soc_put_enum_double),
};

static const char * const pwm_type[] = {
	"single-ended", "differential"
};

static int atmel_classd_codec_probe(struct snd_soc_codec *codec)
{
	struct atmel_classd *dd  = snd_soc_codec_get_drvdata(codec);
	const struct atmel_classd_pdata *pdata = dd->pdata;
	u32 mask, val;

	mask = CLASSD_MR_PWMTYP_MASK;
	val = pdata->pwm_type << CLASSD_MR_PWMTYP_SHIFT;

	mask |= CLASSD_MR_NON_OVERLAP_MASK;
	if (pdata->non_overlap_enable) {
		val |= (CLASSD_MR_NON_OVERLAP_EN
			<< CLASSD_MR_NON_OVERLAP_SHIFT);

		mask |= CLASSD_MR_NOVR_VAL_MASK;
		switch (pdata->non_overlap_time) {
		case 5:
			val |= (CLASSD_MR_NOVR_VAL_5NS
				<< CLASSD_MR_NOVR_VAL_SHIFT);
			break;
		case 10:
			val |= (CLASSD_MR_NOVR_VAL_10NS
				<< CLASSD_MR_NOVR_VAL_SHIFT);
			break;
		case 15:
			val |= (CLASSD_MR_NOVR_VAL_15NS
				<< CLASSD_MR_NOVR_VAL_SHIFT);
			break;
		case 20:
			val |= (CLASSD_MR_NOVR_VAL_20NS
				<< CLASSD_MR_NOVR_VAL_SHIFT);
			break;
		default:
			val |= (CLASSD_MR_NOVR_VAL_10NS
				<< CLASSD_MR_NOVR_VAL_SHIFT);
			break;
		}
	}

	snd_soc_update_bits(codec, CLASSD_MR, mask, val);

	dev_info(codec->dev,
		"PWM modulation type is %s, non-overlapping is %s\n",
		pwm_type[pdata->pwm_type],
		pdata->non_overlap_enable?"enabled":"disabled");

	return 0;
}

static struct regmap *atmel_classd_codec_get_remap(struct device *dev)
{
	struct atmel_classd *dd = dev_get_drvdata(dev);

	return dd->regmap;
}

static struct snd_soc_codec_driver soc_codec_dev_classd = {
	.probe = atmel_classd_codec_probe,
	.controls = atmel_classd_snd_controls,
	.num_controls = ARRAY_SIZE(atmel_classd_snd_controls),
	.get_regmap = atmel_classd_codec_get_remap,
};

static int atmel_classd_codec_dai_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *codec_dai)
{
	struct atmel_classd *dd = snd_soc_dai_get_drvdata(codec_dai);

	clk_prepare_enable(dd->aclk);
	clk_prepare_enable(dd->gclk);

	return 0;
}

static int atmel_classd_codec_dai_digital_mute(struct snd_soc_dai *codec_dai,
	int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u32 mask, val;

	mask = CLASSD_MR_LMUTE_MASK | CLASSD_MR_RMUTE_MASK;

	if (mute)
		val = mask;
	else
		val = 0;

	snd_soc_update_bits(codec, CLASSD_MR, mask, val);

	return 0;
}

#define CLASSD_ACLK_RATE_11M2896_MPY_8 (112896 * 100 * 8)
#define CLASSD_ACLK_RATE_12M288_MPY_8  (12228 * 1000 * 8)

static struct {
	int rate;
	int sample_rate;
	int dsp_clk;
	unsigned long aclk_rate;
} const sample_rates[] = {
	{ 8000,  CLASSD_INTPMR_FRAME_8K,
	CLASSD_INTPMR_DSP_CLK_FREQ_12M288, CLASSD_ACLK_RATE_12M288_MPY_8 },
	{ 16000, CLASSD_INTPMR_FRAME_16K,
	CLASSD_INTPMR_DSP_CLK_FREQ_12M288, CLASSD_ACLK_RATE_12M288_MPY_8 },
	{ 32000, CLASSD_INTPMR_FRAME_32K,
	CLASSD_INTPMR_DSP_CLK_FREQ_12M288, CLASSD_ACLK_RATE_12M288_MPY_8 },
	{ 48000, CLASSD_INTPMR_FRAME_48K,
	CLASSD_INTPMR_DSP_CLK_FREQ_12M288, CLASSD_ACLK_RATE_12M288_MPY_8 },
	{ 96000, CLASSD_INTPMR_FRAME_96K,
	CLASSD_INTPMR_DSP_CLK_FREQ_12M288, CLASSD_ACLK_RATE_12M288_MPY_8 },
	{ 22050, CLASSD_INTPMR_FRAME_22K,
	CLASSD_INTPMR_DSP_CLK_FREQ_11M2896, CLASSD_ACLK_RATE_11M2896_MPY_8 },
	{ 44100, CLASSD_INTPMR_FRAME_44K,
	CLASSD_INTPMR_DSP_CLK_FREQ_11M2896, CLASSD_ACLK_RATE_11M2896_MPY_8 },
	{ 88200, CLASSD_INTPMR_FRAME_88K,
	CLASSD_INTPMR_DSP_CLK_FREQ_11M2896, CLASSD_ACLK_RATE_11M2896_MPY_8 },
};

static int
atmel_classd_codec_dai_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct atmel_classd *dd = snd_soc_dai_get_drvdata(codec_dai);
	int fs;
	int i, best, best_val, cur_val, ret;
	u32 mask, val;

	fs = params_rate(params);

	best = 0;
	best_val = abs(fs - sample_rates[0].rate);
	for (i = 1; i < ARRAY_SIZE(sample_rates); i++) {
		/* Closest match */
		cur_val = abs(fs - sample_rates[i].rate);
		if (cur_val < best_val) {
			best = i;
			best_val = cur_val;
		}
	}

	dev_dbg(codec->dev,
		"Selected SAMPLE_RATE of %dHz, ACLK_RATE of %ldHz\n",
		sample_rates[best].rate, sample_rates[best].aclk_rate);

	clk_disable_unprepare(dd->gclk);
	clk_disable_unprepare(dd->aclk);

	ret = clk_set_rate(dd->aclk, sample_rates[best].aclk_rate);
	if (ret)
		return ret;

	mask = CLASSD_INTPMR_DSP_CLK_FREQ_MASK | CLASSD_INTPMR_FRAME_MASK;
	val = (sample_rates[best].dsp_clk << CLASSD_INTPMR_DSP_CLK_FREQ_SHIFT)
	| (sample_rates[best].sample_rate << CLASSD_INTPMR_FRAME_SHIFT);

	snd_soc_update_bits(codec, CLASSD_INTPMR, mask, val);

	clk_prepare_enable(dd->aclk);
	clk_prepare_enable(dd->gclk);

	return 0;
}

static void
atmel_classd_codec_dai_shutdown(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *codec_dai)
{
	struct atmel_classd *dd = snd_soc_dai_get_drvdata(codec_dai);

	clk_disable_unprepare(dd->gclk);
	clk_disable_unprepare(dd->aclk);
}

static int atmel_classd_codec_dai_prepare(struct snd_pcm_substream *substream,
					struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	snd_soc_update_bits(codec, CLASSD_MR,
				CLASSD_MR_LEN_MASK | CLASSD_MR_REN_MASK,
				(CLASSD_MR_LEN_DIS << CLASSD_MR_LEN_SHIFT)
				|(CLASSD_MR_REN_DIS << CLASSD_MR_REN_SHIFT));

	return 0;
}

static int atmel_classd_codec_dai_trigger(struct snd_pcm_substream *substream,
					int cmd, struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u32 mask, val;

	mask = CLASSD_MR_LEN_MASK | CLASSD_MR_REN_MASK;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		val = mask;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		val = (CLASSD_MR_LEN_DIS << CLASSD_MR_LEN_SHIFT)
			| (CLASSD_MR_REN_DIS << CLASSD_MR_REN_SHIFT);
		break;
	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec, CLASSD_MR, mask, val);

	return 0;
}

static const struct snd_soc_dai_ops atmel_classd_codec_dai_ops = {
	.digital_mute	= atmel_classd_codec_dai_digital_mute,
	.startup	= atmel_classd_codec_dai_startup,
	.shutdown	= atmel_classd_codec_dai_shutdown,
	.hw_params	= atmel_classd_codec_dai_hw_params,
	.prepare	= atmel_classd_codec_dai_prepare,
	.trigger	= atmel_classd_codec_dai_trigger,
};

#define ATMEL_CLASSD_CODEC_DAI_NAME  "atmel-classd-hifi"

static struct snd_soc_dai_driver atmel_classd_codec_dai = {
	.name = ATMEL_CLASSD_CODEC_DAI_NAME,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ATMEL_CLASSD_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &atmel_classd_codec_dai_ops,
};

/* regmap configuration */
static const struct reg_default atmel_classd_reg_defaults[] = {
	{ CLASSD_INTPMR,   0x00302727 },
};

#define ATMEL_CLASSD_REG_MAX    0xE4
static const struct regmap_config atmel_classd_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = ATMEL_CLASSD_REG_MAX,

	.cache_type = REGCACHE_FLAT,
	.reg_defaults = atmel_classd_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(atmel_classd_reg_defaults),
};

static int atmel_classd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct atmel_classd *dd;
	struct resource *res;
	void __iomem *io_base;
	const struct atmel_classd_pdata *pdata;
	int ret;

	pdata = dev_get_platdata(dev);
	if (!pdata) {
		pdata = atmel_classd_dt_init(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	dd = devm_kzalloc(&pdev->dev, sizeof(*dd), GFP_KERNEL);
	if (!dd)
		return -ENOMEM;

	dd->pdata = pdata;

	platform_set_drvdata(pdev, dd);

	dd->irq = platform_get_irq(pdev, 0);
	if (dd->irq < 0) {
		ret = dd->irq;
		dev_err(&pdev->dev, "failed to could not get irq: %d\n", ret);
		return ret;
	}

	dd->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(dd->pclk)) {
		ret = PTR_ERR(dd->pclk);
		dev_err(dev, "failed to get peripheral clock: %d\n", ret);
		return ret;
	}

	dd->gclk = devm_clk_get(dev, "gclk");
	if (IS_ERR(dd->aclk)) {
		ret = PTR_ERR(dd->gclk);
		dev_err(dev, "failed to get GCK clock: %d\n", ret);
		return ret;
	}

	dd->aclk = devm_clk_get(dev, "aclk");
	if (IS_ERR(dd->aclk)) {
		ret = PTR_ERR(dd->aclk);
		dev_err(dev, "failed to get audio clock: %d\n", ret);
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource\n");
		return -ENXIO;
	}

	io_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(io_base)) {
		ret =  PTR_ERR(io_base);
		dev_err(dev, "failed to remap register memory: %d\n", ret);
		return ret;
	}

	dd->phy_base = res->start;

	dd->regmap = devm_regmap_init_mmio(dev, io_base,
					&atmel_classd_regmap_config);
	if (IS_ERR(dd->regmap)) {
		ret = PTR_ERR(dd->regmap);
		dev_err(dev, "failed to init register map: %d\n", ret);
		return ret;
	}

	ret = devm_snd_soc_register_component(&pdev->dev,
					&atmel_classd_cpu_dai_component,
					&atmel_classd_cpu_dai, 1);
	if (ret) {
		dev_err(dev, "Could not register CPU DAI: %d\n", ret);
		return ret;
	}

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev,
					&atmel_classd_dmaengine_pcm_config,
					SND_DMAENGINE_PCM_FLAG_NO_RESIDUE);
	if (ret) {
		dev_err(dev, "Could not register platform: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_codec(dev, &soc_codec_dev_classd,
					&atmel_classd_codec_dai, 1);
	if (ret) {
		dev_err(dev, "Could not register codec: %d\n", ret);
		return ret;
	}

	dev_info(dev,
		"Atmel Class D Amplifier (CLASSD) device at 0x%p (irq %d)\n",
		io_base, dd->irq);

	return 0;
}

static int atmel_classd_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver classd_driver = {
	.driver		= {
		.name		= "atmel-classd",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(atmel_classd_of_match),
	},
	.probe		= atmel_classd_probe,
	.remove		= atmel_classd_remove,
};
module_platform_driver(classd_driver);

static struct snd_soc_dai_link atmel_asoc_classd_dailink = {
	.name = "CLASSD",
	.stream_name = "CLASSD PCM",
	.codec_dai_name = ATMEL_CLASSD_CODEC_DAI_NAME,
};

static struct snd_soc_card atmel_asoc_classd_card = {
	.owner = THIS_MODULE,
	.dai_link = &atmel_asoc_classd_dailink,
	.num_links = 1,
};

#ifdef CONFIG_OF
static const struct of_device_id atmel_asoc_dt_ids[] = {
	{ .compatible = "atmel,asoc-classd", },
	{ }
};
#endif

static int atmel_asoc_classd_dt_init(struct platform_device *pdev,
				struct snd_soc_card *card)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *codec_np, *platform_np;
	const char *cpu_dai_name;
	struct snd_soc_dai_link *dailink = card->dai_link;
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

	of_property_read_string(np, "atmel,audio-cpu-dai-name", &cpu_dai_name);
	dailink->cpu_dai_name = cpu_dai_name;

	platform_np = of_parse_phandle(np, "atmel,audio-platform", 0);
	if (!platform_np) {
		dev_err(&pdev->dev, "failed to get platform info\n");
		ret = -EINVAL;
		return ret;
	}
	dailink->platform_of_node = platform_np;
	of_node_put(platform_np);

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

static int atmel_asoc_classd_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &atmel_asoc_classd_card;
	int ret;

	card->dev = &pdev->dev;
	ret = atmel_asoc_classd_dt_init(pdev, card);
	if (ret) {
		dev_err(&pdev->dev, "failed to init dt info\n");
		return ret;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "failed to register asoc card\n");
		return ret;
	}

	return 0;
}

static struct platform_driver atmel_asoc_classd_driver = {
	.driver = {
		.name = "atmel-asoc-audio",
		.of_match_table = of_match_ptr(atmel_asoc_dt_ids),
		.pm	= &snd_soc_pm_ops,
	},
	.probe = atmel_asoc_classd_probe,
};

module_platform_driver(atmel_asoc_classd_driver);

MODULE_DESCRIPTION("Atmel ClassD driver under ALSA SoC architecture");
MODULE_AUTHOR("Songjun Wu <songjun.wu@atmel.com>");
MODULE_LICENSE("GPL");
