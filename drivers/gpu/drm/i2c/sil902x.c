
#include <linux/component.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>

#define SIL902X_TPI_VIDEO_DATA			0x0

#define SIL902X_TPI_PIXEL_REPETITION		0x8
#define SIL902X_TPI_AVI_PIXEL_REP_BUS_24BIT     BIT(5)
#define SIL902X_TPI_AVI_PIXEL_REP_RISING_EDGE   BIT(4)
#define SIL902X_TPI_AVI_PIXEL_REP_4X		3
#define SIL902X_TPI_AVI_PIXEL_REP_2X		1
#define SIL902X_TPI_AVI_PIXEL_REP_NONE		0
#define SIL902X_TPI_CLK_RATIO_HALF		(0 << 6)
#define SIL902X_TPI_CLK_RATIO_1X		(1 << 6)
#define SIL902X_TPI_CLK_RATIO_2X		(2 << 6)
#define SIL902X_TPI_CLK_RATIO_4X		(3 << 6)

#define SIL902X_TPI_AVI_IN_FORMAT		0x9
#define SIL902X_TPI_AVI_INPUT_BITMODE_12BIT	BIT(7)
#define SIL902X_TPI_AVI_INPUT_DITHER		BIT(6)
#define SIL902X_TPI_AVI_INPUT_RANGE_LIMITED	(2 << 2)
#define SIL902X_TPI_AVI_INPUT_RANGE_FULL	(1 << 2)
#define SIL902X_TPI_AVI_INPUT_RANGE_AUTO	(0 << 2)
#define SIL902X_TPI_AVI_INPUT_COLORSPACE_BLACK	(3 << 0)
#define SIL902X_TPI_AVI_INPUT_COLORSPACE_YUV422	(2 << 0)
#define SIL902X_TPI_AVI_INPUT_COLORSPACE_YUV444	(1 << 0)
#define SIL902X_TPI_AVI_INPUT_COLORSPACE_RGB	(0 << 0)


#define SIL902X_SYS_CTRL_DATA			0x1a
#define SIL902X_SYS_CTRL_PWR_DWN		BIT(4)
#define SIL902X_SYS_CTRL_AV_MUTE		BIT(3)
#define SIL902X_SYS_CTRL_DDC_BUS_REQ		BIT(2)
#define SIL902X_SYS_CTRL_DDC_BUS_GRTD		BIT(1)
#define SIL902X_SYS_CTRL_OUTPUT_MODE		BIT(0)
#define SIL902X_SYS_CTRL_OUTPUT_HDMI		1
#define SIL902X_SYS_CTRL_OUTPUT_DVI		0

#define SIL902X_REG_CHIPID(n)			(0x1b + (n))

#define SIL902X_PWR_STATE_CTRL			0x1e
#define SIL902X_AVI_POWER_STATE_MSK		GENMASK(1, 0)
#define SIL902X_AVI_POWER_STATE_D(l)		((l) & SIL902X_AVI_POWER_STATE_MSK)

#define SI902X_INT_ENABLE			0x3c
#define SI902X_INT_STATUS			0x3d
#define SI902X_HOTPLUG_EVENT			BIT(0)
#define SI902X_PLUGGED_STATUS			BIT(2)

#define SIL902X_REG_TPI_RQB			0xc7

struct sil902x {
	struct i2c_client *i2c;
	struct regmap *regmap;
	struct drm_encoder encoder;
	struct drm_connector connector;
	int reset_gpio;
	bool reset_active_low;
	enum of_gpio_flags reset_gpio_flags;
	struct work_struct hotplug_work;
};

static inline struct sil902x *encoder_to_sil902x(struct drm_encoder *encoder)
{
	return container_of(encoder, struct sil902x, encoder);
}

static inline struct sil902x *connector_to_sil902x(struct drm_connector *con)
{
	return container_of(con, struct sil902x, connector);
}

static void sil902x_reset(struct sil902x *sil902x)
{
	if (!gpio_is_valid(sil902x->reset_gpio))
		return;

	gpio_set_value(sil902x->reset_gpio,
		       sil902x->reset_active_low ? 0 : 1);

	msleep(100);

	gpio_set_value(sil902x->reset_gpio,
		       sil902x->reset_active_low ? 1 : 0);
}

static void sil902x_encoder_reset(struct drm_encoder *encoder)
{
	struct sil902x *sil902x = encoder_to_sil902x(encoder);

	sil902x_reset(sil902x);
}

static void sil902x_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs sil902x_encoder_funcs = {
	.reset = sil902x_encoder_reset,
	.destroy = sil902x_encoder_destroy,
};

static void sil902x_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct sil902x *sil902x = encoder_to_sil902x(encoder);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		regmap_update_bits(sil902x->regmap, SIL902X_PWR_STATE_CTRL,
				   SIL902X_AVI_POWER_STATE_MSK,
				   SIL902X_AVI_POWER_STATE_D(0));
		regmap_update_bits(sil902x->regmap, SIL902X_SYS_CTRL_DATA,
				   SIL902X_SYS_CTRL_PWR_DWN, 0);
		break;
	case DRM_MODE_DPMS_OFF:
		regmap_update_bits(sil902x->regmap, SIL902X_SYS_CTRL_DATA,
				   SIL902X_SYS_CTRL_PWR_DWN,
				   SIL902X_SYS_CTRL_PWR_DWN);
		break;
	}
}

static void sil902x_encoder_save(struct drm_encoder *encoder)
{
	/* TODO: save registers values */
}

static void sil902x_encoder_restore(struct drm_encoder *encoder)
{
	/* TODO: restore registers values */
}

static bool sil902x_encoder_mode_fixup(struct drm_encoder *encoder,
				       const struct drm_display_mode *mode,
				       struct drm_display_mode *adjusted_mode)
{
	/* TODO: see if we can implement a proper mode fixup here */

	return true;
}

static void sil902x_encoder_prepare(struct drm_encoder *encoder)
{
	sil902x_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void sil902x_encoder_commit(struct drm_encoder *encoder)
{
	sil902x_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void sil902x_encoder_mode_set(struct drm_encoder *encoder,
				     struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode)
{
	struct sil902x *sil902x = encoder_to_sil902x(encoder);
	struct regmap * regmap = sil902x->regmap;
	u8 buf[10];
	int ret;

	buf[0] = adjusted_mode->clock;
	buf[1] = adjusted_mode->clock >> 8;
	buf[2] = 0x3c;
	buf[3] = 0x60;
	buf[4] = adjusted_mode->hdisplay;
	buf[5] = adjusted_mode->hdisplay >> 8;
	buf[6] = adjusted_mode->vdisplay;
	buf[7] = adjusted_mode->vdisplay >> 8;
	buf[8] = SIL902X_TPI_CLK_RATIO_1X | SIL902X_TPI_AVI_PIXEL_REP_NONE |
		 SIL902X_TPI_AVI_PIXEL_REP_BUS_24BIT;
	buf[9] = SIL902X_TPI_AVI_INPUT_RANGE_AUTO |
		 SIL902X_TPI_AVI_INPUT_COLORSPACE_RGB;

	ret = regmap_bulk_write(regmap, SIL902X_TPI_VIDEO_DATA, buf, 10);
	if (ret)
		return;
}

static const struct drm_encoder_helper_funcs sil902x_encoder_helper_funcs = {
	.dpms = sil902x_encoder_dpms,
	.save = sil902x_encoder_save,
	.restore = sil902x_encoder_restore,
	.mode_fixup = sil902x_encoder_mode_fixup,
	.prepare = sil902x_encoder_prepare,
	.commit = sil902x_encoder_commit,
	.mode_set = sil902x_encoder_mode_set,
};

static void sil902x_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static enum drm_connector_status
sil902x_connector_detect(struct drm_connector *connector, bool force)
{
	struct sil902x *sil902x = connector_to_sil902x(connector);
	unsigned int status;

	regmap_read(sil902x->regmap, SI902X_INT_STATUS, &status);

	return (status & SI902X_PLUGGED_STATUS) ?
	       connector_status_connected : connector_status_disconnected;
}

static const struct drm_connector_funcs sil902x_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = sil902x_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = sil902x_connector_destroy,
};

static int sil902x_get_modes(struct drm_connector *connector)
{
	struct sil902x *sil902x = connector_to_sil902x(connector);
	struct regmap *regmap = sil902x->regmap;
	u32 bus_fomart = MEDIA_BUS_FMT_RGB888_1X24;
	unsigned int status;
	struct edid *edid;
	int num = 0;
	int ret;
	int i;

	ret = regmap_update_bits(regmap, SIL902X_PWR_STATE_CTRL,
				 SIL902X_AVI_POWER_STATE_MSK,
				 SIL902X_AVI_POWER_STATE_D(2));
	if (ret)
		return ret;

	ret = regmap_write(regmap, SIL902X_SYS_CTRL_DATA,
			   SIL902X_SYS_CTRL_OUTPUT_HDMI |
			   SIL902X_SYS_CTRL_PWR_DWN);
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, SIL902X_SYS_CTRL_DATA,
				 SIL902X_SYS_CTRL_DDC_BUS_REQ,
				 SIL902X_SYS_CTRL_DDC_BUS_REQ);
	if (ret)
		return ret;

	i = 0;
	do {
		ret = regmap_read(regmap, SIL902X_SYS_CTRL_DATA, &status);
		if (ret)
			return ret;
		i++;
	} while (!(status & SIL902X_SYS_CTRL_DDC_BUS_GRTD));

	ret = regmap_write(regmap, SIL902X_SYS_CTRL_DATA, status);
	if (ret)
		return ret;

	edid = drm_get_edid(connector, sil902x->i2c->adapter);
	drm_mode_connector_update_edid_property(connector, edid);
	if (edid) {
		num += drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_fomart, 1);
	if (ret)
		return ret;

	regmap_read(regmap, SIL902X_SYS_CTRL_DATA, &status);
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, SIL902X_SYS_CTRL_DATA,
				 SIL902X_SYS_CTRL_DDC_BUS_REQ |
				 SIL902X_SYS_CTRL_DDC_BUS_GRTD, 0);
	if (ret)
		return ret;

	i = 0;
	do {
		ret = regmap_read(regmap, SIL902X_SYS_CTRL_DATA, &status);
		if (ret)
			return ret;
		i++;
	} while (status & (SIL902X_SYS_CTRL_DDC_BUS_REQ |
			   SIL902X_SYS_CTRL_DDC_BUS_GRTD));

	return num;
}

static enum drm_mode_status sil902x_mode_valid(struct drm_connector *connector,
					       struct drm_display_mode *mode)
{
	/* TODO: check mode */

	return MODE_OK;
}

static struct drm_encoder *sil902x_best_encoder(struct drm_connector *connector)
{
	struct sil902x *sil902x = connector_to_sil902x(connector);

	return &sil902x->encoder;
}

static const struct drm_connector_helper_funcs sil902x_connector_helper_funcs = {
	.get_modes = sil902x_get_modes,
	.mode_valid = sil902x_mode_valid,
	.best_encoder = sil902x_best_encoder,
};

static const struct regmap_range sil902x_volatile_ranges[] = {
        {
                .range_min = 0,
                .range_max = 0xff,
        },
};

static const struct regmap_access_table sil902x_volatile_table = {
        .yes_ranges = sil902x_volatile_ranges,
        .n_yes_ranges = ARRAY_SIZE(sil902x_volatile_ranges),
};


static const struct regmap_config sil902x_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.volatile_table = &sil902x_volatile_table,
	.cache_type = REGCACHE_NONE,
};

static int sil902x_bind(struct device *dev, struct device *master, void *data)
{
	struct sil902x *sil902x = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	int ret;

	sil902x->encoder.possible_crtcs = 1;
	drm_encoder_helper_add(&sil902x->encoder, &sil902x_encoder_helper_funcs);
	ret = drm_encoder_init(drm, &sil902x->encoder, &sil902x_encoder_funcs,
			       DRM_MODE_ENCODER_TMDS);
	if (ret)
		return ret;

	drm_connector_helper_add(&sil902x->connector,
				 &sil902x_connector_helper_funcs);
	ret = drm_connector_init(drm, &sil902x->connector,
				 &sil902x_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret)
		goto err_cleanup_encoder;

	if (sil902x->i2c->irq > 0)
		sil902x->connector.polled = DRM_CONNECTOR_POLL_HPD;
	else
		sil902x->connector.polled = DRM_CONNECTOR_POLL_CONNECT;

	drm_mode_connector_attach_encoder(&sil902x->connector, &sil902x->encoder);

	return 0;

err_cleanup_encoder:
	drm_encoder_cleanup(&sil902x->encoder);

	return ret;
}

static void sil902x_unbind(struct device *dev, struct device *master, void *data)
{
	struct sil902x *sil902x = dev_get_drvdata(dev);

	drm_connector_cleanup(&sil902x->connector);
	drm_encoder_cleanup(&sil902x->encoder);
}

static const struct component_ops sil902x_ops = {
	.bind = sil902x_bind,
	.unbind = sil902x_unbind,
};

static irqreturn_t sil902x_interrupt(int irq, void *data)
{
	struct sil902x *sil902x = data;
	unsigned int status = 0;

	regmap_read(sil902x->regmap, SI902X_INT_STATUS, &status);
	regmap_write(sil902x->regmap, SI902X_INT_STATUS, status);

	if ((status & SI902X_HOTPLUG_EVENT) && sil902x->encoder.dev)
		drm_helper_hpd_irq_event(sil902x->encoder.dev);

	return IRQ_HANDLED;
}

static int sil902x_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	enum of_gpio_flags gpio_flags;
	unsigned int status = 0;
	struct sil902x *sil902x;
	u8 chipid[4];
	int ret;

	sil902x = devm_kzalloc(dev, sizeof(*sil902x), GFP_KERNEL);
	if (!sil902x)
		return -ENOMEM;

	sil902x->i2c = client;
	sil902x->regmap = devm_regmap_init_i2c(client, &sil902x_regmap_config);
	if (IS_ERR(sil902x->regmap))
		return PTR_ERR(sil902x->regmap);

	sil902x->reset_gpio = of_get_named_gpio_flags(dev->of_node,
						      "reset-gpios", 0,
						       &gpio_flags);
	if (gpio_flags & OF_GPIO_ACTIVE_LOW)
		sil902x->reset_active_low = true;

	if (gpio_is_valid(sil902x->reset_gpio)) {
		ret = devm_gpio_request(dev, sil902x->reset_gpio, "reset-gpio");
		if (ret)
			return ret;
		sil902x->reset_active_low = true;

		ret = gpio_direction_output(sil902x->reset_gpio,
				sil902x->reset_active_low ? 1 : 0);
		if (ret)
			return ret;

		sil902x_reset(sil902x);
	}

	ret = regmap_write(sil902x->regmap, SIL902X_REG_TPI_RQB, 0x0);
	if (ret)
		return ret;

	ret = regmap_bulk_read(sil902x->regmap, SIL902X_REG_CHIPID(0),
			       &chipid, 4);
	if (ret) {
		dev_err(dev, "regmap_read failed %d\n", ret);
		return ret;
	}

	if (chipid[0] != 0xb0) {
		dev_err(dev, "Invalid chipid: %02x (expecting 0xb0)\n",
			chipid[0]);
		return -EINVAL;
	}

	/* Clear all pending interrupts */
	regmap_read(sil902x->regmap, SI902X_INT_STATUS, &status);
	regmap_write(sil902x->regmap, SI902X_INT_STATUS, status);

	if (client->irq > 0) {
		regmap_write(sil902x->regmap, SI902X_INT_ENABLE,
			     SI902X_HOTPLUG_EVENT);

		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sil902x_interrupt,
						IRQF_ONESHOT, dev_name(dev),
						sil902x);
		if (ret)
			return ret;
	}

	i2c_set_clientdata(client, sil902x);

	return component_add(&client->dev, &sil902x_ops);
}


static int sil902x_i2c_remove(struct i2c_client *client)

{
	component_del(&client->dev, &sil902x_ops);

	return 0;
}

static int sil902x_encoder_init(struct i2c_client *client,
				struct drm_device *dev,
				struct drm_encoder_slave *encoder)
{
	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id sil902x_i2c_dt_ids[] = {
	{ .compatible = "sil,sil9022", },
	{ }
};
MODULE_DEVICE_TABLE(of, sil902x_i2c_dt_ids);
#endif

static const struct i2c_device_id sil902x_i2c_ids[] = {
	{ "sil9022", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sil9022_id);

static struct drm_i2c_encoder_driver sil902x_i2c_driver = {
	.i2c_driver = {
		.probe = sil902x_i2c_probe,
		.remove = sil902x_i2c_remove,
		.driver = {
			.name = "sil902x",
			.of_match_table = of_match_ptr(sil902x_i2c_dt_ids),
		},
		.id_table = sil902x_i2c_ids,
	},

	.encoder_init = sil902x_encoder_init,
};

static int __init
sil902x_init(void)
{
	return drm_i2c_encoder_register(THIS_MODULE, &sil902x_i2c_driver);
}
module_init(sil902x_init);

static void __exit
sil902x_exit(void)
{
	drm_i2c_encoder_unregister(&sil902x_i2c_driver);
}
module_exit(sil902x_exit);

MODULE_AUTHOR("Boris Brezillon <boris.brezillon@free-electrons.com>");
MODULE_DESCRIPTION("SIL902X TMDS encoders");
MODULE_LICENSE("GPL");
