/*
 * Copyright (C) 2014 Atmel
 *		      Bo Shen <voice.shen@atmel.com>
 *
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/hdmi.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/regmap.h>

#include "encoder-sii9022.h"

static const struct regmap_config sii9022_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static ssize_t sii902x_show_state(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sii902x_data *sii9022x = dev_get_drvdata(dev);

	if (sii9022x->cable_plugin == 0)
		strcpy(buf, "plugout\n");
	else
		strcpy(buf, "plugin\n");

	return strlen(buf);
}
static DEVICE_ATTR(cable_state, S_IRUGO, sii902x_show_state, NULL);

static ssize_t sii902x_show_edid(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sii902x_data *sii9022x = dev_get_drvdata(dev);
	int i, j, len = 0;

	for (j = 0; j < SII9022_EDID_LEN / 16; j++) {
		for (i = 0; i < 16; i++)
			len += sprintf(buf+len, "0x%02X ",
				       sii9022x->edid[j * 16 + i]);
		len += sprintf(buf+len, "\n");
	}

	return len;
}
static DEVICE_ATTR(edid, S_IRUGO, sii902x_show_edid, NULL);

static void sii902x_poweron(struct sii902x_data *sii9022x)
{
	/* Turn on DVI or HDMI */
	if (sii9022x->edid_cfg.hdmi_cap)
		regmap_write(sii9022x->regmap, SII9022_SYS_CTRL_DATA_REG, 0x01);
	else
		regmap_write(sii9022x->regmap, SII9022_SYS_CTRL_DATA_REG, 0x00);
}

static void sii902x_poweroff(struct sii902x_data *sii9022x)
{
	/* disable tmds before changing resolution */
	if (sii9022x->edid_cfg.hdmi_cap)
		regmap_write(sii9022x->regmap, SII9022_SYS_CTRL_DATA_REG, 0x11);
	else
		regmap_write(sii9022x->regmap, SII9022_SYS_CTRL_DATA_REG, 0x10);
}

static void sii902x_reset(struct sii902x_data *sii9022x)
{
	gpio_direction_output(sii9022x->reset_pin, 0);
	mdelay(100);
	gpio_direction_output(sii9022x->reset_pin, 1);
}

static int sii902x_set_avi_infoframe(struct sii902x_data *sii9022x)
{
	struct i2c_client *client = sii9022x->client;
	struct hdmi_avi_infoframe infoframe;
	u8 infoframe_buf[HDMI_INFOFRAME_HEADER_SIZE + HDMI_AVI_INFOFRAME_SIZE];
	int ret;

	hdmi_avi_infoframe_init(&infoframe);

	infoframe.colorspace = HDMI_COLORSPACE_RGB;
	infoframe.active_info_valid  = true;
	infoframe.horizontal_bar_valid = false;
	infoframe.vertical_bar_valid = false;
	infoframe.scan_mode = HDMI_SCAN_MODE_NONE;
	infoframe.colorimetry = HDMI_COLORIMETRY_NONE;
	infoframe.picture_aspect = HDMI_PICTURE_ASPECT_16_9;
	infoframe.active_aspect = HDMI_ACTIVE_ASPECT_PICTURE;
	infoframe.quantization_range = HDMI_QUANTIZATION_RANGE_FULL;

	ret = hdmi_avi_infoframe_pack(&infoframe, infoframe_buf,
				      sizeof(infoframe_buf));
	if (ret < 0) {
		dev_err(&client->dev, "failed to pack avi infoframe\n");
		return ret;
	}

	regmap_bulk_write(sii9022x->regmap, SII9022_AVI_INFOFRAME_BASE_REG,
			  &infoframe_buf[3], SII9022_AVI_INFOFRAME_LEN);

	return 0;
}

static void sii902x_setup(struct sii902x_data *sii9022x)
{
	u16 data[4];
	u8 *tmp;
	int i;

	dev_dbg(&sii9022x->client->dev, "Sii902x: setup..\n");

	/* Power up */
	regmap_write(sii9022x->regmap, SII9022_POWER_STATE_CTRL_REG, 0x00);

	/* set TPI video mode */
	switch (sii9022x->resolution) {
	case 1080: /* 1080P30 timing */
		data[0] = 7425;
		data[1] = 6000;
		data[2] = 2200;
		data[3] = 1125;
		break;
	case 900: /* 1440 x 900 timing */
		data[0] = 8875;
		data[1] = 6000;
		data[2] = 1600;
		data[3] = 926;
		break;
	default: /* 720P60 timing */
		data[0] = 7425;
		data[1] = 6000;
		data[2] = 1650;
		data[3] = 750;
		break;
	}

	tmp = (u8 *)data;
	for (i = 0; i < 8; i++)
		regmap_write(sii9022x->regmap, i, tmp[i]);

	/* input bus/pixel: full pixel wide (24bit), rising edge */
	regmap_write(sii9022x->regmap, SII9022_PIXEL_REPETITION_REG, 0x60);
	/* Set input format to RGB */
	regmap_write(sii9022x->regmap, SII9022_AVI_IN_FORMAT_REG, 0x00);
	/* set output format to RGB */
	regmap_write(sii9022x->regmap, SII9022_AVI_OUT_FORMAT_REG, 0x10);

	sii902x_set_avi_infoframe(sii9022x);
}

static void sii902x_edid_parse_ext_blk(unsigned char *edid,
				       struct sii902x_edid_cfg *cfg)
{
	unsigned char index = 0x0;
	u8 detail_timing_offset, tag_code, data_payload;
	int i;

	if (edid[index++] != 0x2) /* only support cea ext block now */
		return;
	if (edid[index++] != 0x3) /* only support version 3 */
		return;

	detail_timing_offset = edid[index++];

	cfg->cea_underscan = (edid[index] >> 7) & 0x1;
	cfg->cea_basicaudio = (edid[index] >> 6) & 0x1;
	cfg->cea_ycbcr444 = (edid[index] >> 5) & 0x1;
	cfg->cea_ycbcr422 = (edid[index] >> 4) & 0x1;

	/* Parse data block */
	while (++index < detail_timing_offset) {
		tag_code = (edid[index] >> 5) & 0x7;
		data_payload = edid[index] & 0x1f;

		if (tag_code == 0x2) {
			for (i = 0; i < data_payload; i++)
				cfg->video_cap[i] = edid[index + 1 + i];
		}

		/* Find vendor block to check HDMI capable */
		if (tag_code == 0x3) {
			if ((edid[index + 1] == 0x03) &&
			    (edid[index + 2] == 0x0c) &&
			    (edid[index + 3] == 0x00))
				cfg->hdmi_cap = true;
		}

		index += data_payload;
	}
}

static int sii902x_edid_read(struct i2c_adapter *adp, unsigned short addr,
			     unsigned char *edid, struct sii902x_edid_cfg *cfg)
{
	u8 buf[2] = {0, 0};
	int dat = 0;
	struct i2c_msg msg[2] = {
		{
			.addr	= addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		}, {
			.addr	= addr,
			.flags	= I2C_M_RD,
			.len	= ONE_BLOCK_EDID_LEN,
			.buf	= edid,
		},
	};

	if (adp == NULL)
		return -EINVAL;

	memset(edid, 0, SII9022_EDID_LEN);

	buf[0] = 0x00;
	dat = i2c_transfer(adp, msg, 2);

	/* need read ext block? Only support one more blk now*/
	if (edid[0x7E]) {
		if (edid[0x7E] > 1) {
			pr_debug("Edid has %d ext block, but now only support 1 ext blk\n",
				 edid[0x7E]);
			return -ENODEV;
		}

		/* Add a delay to read extension block */
		msleep(20);

		buf[0] = ONE_BLOCK_EDID_LEN;
		msg[1].buf = edid + ONE_BLOCK_EDID_LEN;
		dat = i2c_transfer(adp, msg, 2);
		if (dat < 0)
			return dat;

		/* edid ext block parsing */
		sii902x_edid_parse_ext_blk(edid + ONE_BLOCK_EDID_LEN, cfg);
	}

	return 0;
}

static int sii902x_read_edid(struct sii902x_data *sii9022x)
{
	struct i2c_client *client = sii9022x->client;
	int old, dat, ret, cnt = 100;

	/* Request DDC bus */
	regmap_read(sii9022x->regmap, SII9022_SYS_CTRL_DATA_REG, &old);

	regmap_write(sii9022x->regmap, SII9022_SYS_CTRL_DATA_REG, old | 0x4);
	do {
		cnt--;
		msleep(20);
		regmap_read(sii9022x->regmap, SII9022_SYS_CTRL_DATA_REG, &dat);
	} while ((!(dat & 0x2)) && cnt);

	if (!cnt) {
		ret = -1;
		goto done;
	}

	regmap_write(sii9022x->regmap, SII9022_SYS_CTRL_DATA_REG, old | 0x06);

	/* edid reading */
	ret = sii902x_edid_read(client->adapter, HDMI_I2C_MONITOR_ADDRESS,
				sii9022x->edid, &sii9022x->edid_cfg);
	if (ret) {
		ret = -1;
		goto done;
	}

	/* Release DDC bus */
	cnt = 100;
	do {
		cnt--;
		regmap_write(sii9022x->regmap, SII9022_SYS_CTRL_DATA_REG,
			     old & ~0x6);
		msleep(20);
		regmap_read(sii9022x->regmap, SII9022_SYS_CTRL_DATA_REG, &dat);
	} while ((dat & 0x6) && cnt);

	if (!cnt)
		ret = -1;

done:
	regmap_write(sii9022x->regmap, SII9022_SYS_CTRL_DATA_REG, old);
	return ret;
}

static void det_worker(struct work_struct *work)
{
	struct sii902x_data *sii9022x;

	sii9022x = container_of(work, struct sii902x_data, work);

	if (sii902x_read_edid(sii9022x) < 0) {
		dev_err(&sii9022x->client->dev,
			"Sii902x: read edid fail\n");
	} else {
		int i;

		for (i = 0; i < sizeof(sii9022x->edid); i++) {
			if (i % 16 == 0)
				pr_debug("\n");
			pr_debug("%02x ", sii9022x->edid[i]);
		}
		pr_debug("\n");

		sii902x_setup(sii9022x);
		sii902x_poweron(sii9022x);
	}
}

static int sii902x_handle_hpd(struct sii902x_data *sii9022x)
{
	struct i2c_client *client = sii9022x->client;
	int dat, ret;

	ret = regmap_read(sii9022x->regmap, SII9022_IRQ_STATUS_REG, &dat);
	if (ret < 0) {
		dev_err(&client->dev, "failed read irq status register\n");
		return -EINVAL;
	}

	if (dat & 0x1) {
		/* cable connection changes */
		if (dat & 0x4) {
			sii9022x->cable_plugin = 1;
			schedule_work(&sii9022x->work);
		} else {
			sii902x_poweroff(sii9022x);
			sii9022x->cable_plugin = 0;
		}
	}

	ret = regmap_write(sii9022x->regmap, SII9022_IRQ_STATUS_REG, dat);
	if (ret < 0) {
		dev_err(&client->dev, "failed clean irq status register\n");
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t sii902x_detect_handler(int irq, void *data)
{
	struct sii902x_data *sii9022x = data;
	struct i2c_client *client = sii9022x->client;
	int ret;

	ret = sii902x_handle_hpd(sii9022x);
	if (ret < 0) {
		dev_err(&client->dev, "failed deal with irq\n");
		sii902x_reset(sii9022x);
	}

	return IRQ_HANDLED;
}

static int sii902x_detect_version(struct sii902x_data *sii9022x)
{
	struct i2c_client *client = sii9022x->client;
	int product_id, device_id, rev_id, tpi_id, hdcp_rev;
	int dat, ret;

	ret = regmap_write(sii9022x->regmap, HDMI_IND_SET_PAGE, 0x01);
	if (ret < 0) {
		dev_err(&client->dev, "can not set page register\n");
		return -EINVAL;
	}

	ret = regmap_write(sii9022x->regmap, HDMI_IND_OFFSET, 0x03);
	if (ret < 0) {
		dev_err(&client->dev, "can not set offset register\n");
		return -EINVAL;
	}

	ret = regmap_read(sii9022x->regmap, HDMI_IND_VALUE, &dat);
	if (ret < 0) {
		dev_err(&client->dev, "can not read value register\n");
		return -EINVAL;
	}

	product_id = dat << 8;

	ret = regmap_write(sii9022x->regmap, HDMI_IND_SET_PAGE, 0x01);
	if (ret < 0) {
		dev_err(&client->dev, "can not set page register\n");
		return -EINVAL;
	}

	ret = regmap_write(sii9022x->regmap, HDMI_IND_OFFSET, 0x02);
	if (ret < 0) {
		dev_err(&client->dev, "can not set offset register\n");
		return -EINVAL;
	}

	ret = regmap_read(sii9022x->regmap, HDMI_IND_VALUE, &dat);
	if (ret < 0) {
		dev_err(&client->dev, "can not read value register\n");
		return -EINVAL;
	}

	product_id |= dat;
	dev_info(&client->dev, "product id = %x\n", product_id);

	ret = regmap_read(sii9022x->regmap, SII9022_DEVICE_ID_REG, &device_id);
	if (ret < 0) {
		dev_err(&client->dev, "can not read device id register\n");
		return -EINVAL;
	}

	ret = regmap_read(sii9022x->regmap, SII9022_DEVICE_REV_ID_REG, &rev_id);
	if (ret < 0) {
		dev_err(&client->dev, "can not read rev id register\n");
		return -EINVAL;
	}

	ret = regmap_read(sii9022x->regmap, SII9022_DEVICE_TPI_ID_REG, &tpi_id);
	if (ret < 0) {
		dev_err(&client->dev, "can not read tpi id register\n");
		return -EINVAL;
	}

	ret = regmap_read(sii9022x->regmap, SII9022_DEVICE_HDCP_REV_REG,
			  &hdcp_rev);
	if (ret < 0) {
		dev_err(&client->dev, "can not read hdcp revision register\n");
		return -EINVAL;
	}

	dev_info(&client->dev, "hardware version %02X-%02X-%02X-%02X",
		 device_id, rev_id, tpi_id, hdcp_rev);

	return 0;
}

static int sii902x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct sii902x_data *sii9022x;
	struct regmap *regmap;
	int ret;

	sii9022x = devm_kzalloc(&client->dev, sizeof(sii9022x), GFP_KERNEL);
	if (sii9022x == NULL)
		return -ENOMEM;

	regmap = devm_regmap_init_i2c(client, &sii9022_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "failed to init regmap\n");
		return PTR_ERR(regmap);
	}

	sii9022x->regmap = regmap;
	sii9022x->client = client;

	if (client->dev.of_node) {
		of_property_read_u32(client->dev.of_node, "resolution",
				     &sii9022x->resolution);
		sii9022x->reset_pin = of_get_gpio(client->dev.of_node, 0);
		if (gpio_is_valid(sii9022x->reset_pin)) {
			ret = gpio_request(sii9022x->reset_pin, "reset");
			if (ret < 0) {
				dev_err(&client->dev,
					"can not request reset pin\n");
				return -ENODEV;
			}
		}
	}

	/*
	 * The following is the Initialization process
	 * Take reference on SiI9022A PR page 8
	 */

	/* Step 1.1: hardware reset */
	sii902x_reset(sii9022x);

	/* Set termination to default */
	ret = regmap_write(sii9022x->regmap, SII9022_TMDS_CONT_REG, 0x25);
	if (ret < 0) {
		dev_err(&client->dev, "failed set termination to default\n");
		return -EINVAL;
	}

	/* Set hardware debounce to 64 ms */
	ret = regmap_write(sii9022x->regmap, SII9022_HPD_DELAY_DEBOUNCE, 0x14);
	if (ret < 0) {
		dev_err(&client->dev, "failed set hw debounce to 64 ms\n");
		return -EINVAL;
	}

	/* Step 1.2: enable TPI mode */
	ret = regmap_write(sii9022x->regmap, SII9022_TPI_RQB_REG, 0x00);
	if (ret < 0) {
		dev_err(&client->dev, "can not enable TPI mode\n");
		return -EINVAL;
	}

	/* Step 2: detect product id and version */
	ret = sii902x_detect_version(sii9022x);
	if (ret < 0) {
		dev_err(&client->dev, "detect sii902x failed\n");
		return -EINVAL;
	}

	INIT_WORK(&(sii9022x->work), det_worker);

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, sii902x_detect_handler,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						"SiI902x_det", sii9022x);
		if (ret) {
			dev_err(&client->dev, "failed to request det irq\n");
		} else {
			/* Enable cable hot plug irq */
			regmap_write(sii9022x->regmap, SII9022_IRQ_ENABLE_REG,
				     0x01);
		}

		ret = device_create_file(&client->dev, &dev_attr_cable_state);
		if (ret < 0)
			dev_warn(&client->dev,
				 "cound not create sys node for cable state\n");
		ret = device_create_file(&client->dev, &dev_attr_edid);
		if (ret < 0)
			dev_warn(&client->dev,
				 "cound not create sys node for edid\n");
	}

	i2c_set_clientdata(client, sii9022x);

#ifdef CONFIG_SND_ATMEL_SOC_SII9022
	sii9022_hdmi_codec_register(&client->dev);
#endif

	return 0;
}

static int sii902x_remove(struct i2c_client *client)
{
	struct sii902x_data *sii9022x = i2c_get_clientdata(client);

	sii902x_poweroff(sii9022x);

	return 0;
}

static int sii902x_suspend(struct device *dev)
{
	/*TODO*/
	return 0;
}

static int sii902x_resume(struct device *dev)
{
	/*TODO*/
	return 0;
}

SIMPLE_DEV_PM_OPS(sii902x_pm_ops, sii902x_suspend, sii902x_resume);

static const struct i2c_device_id sii902x_id[] = {
	{ "sii902x", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sii902x_id);

static struct i2c_driver sii902x_i2c_driver = {
	.driver = {
		.name = "sii902x",
		.pm = &sii902x_pm_ops,
		},
	.probe = sii902x_probe,
	.remove = sii902x_remove,
	.id_table = sii902x_id,
};
module_i2c_driver(sii902x_i2c_driver);

MODULE_AUTHOR("Bo Shen <voice.shen@atmel.com>");
MODULE_DESCRIPTION("SII902x DVI/HDMI driver");
MODULE_LICENSE("GPL");
