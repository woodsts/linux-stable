// SPDX-License-Identifier: GPL-2.0-only
/*
 * tmp006.c - Support for TI TMP006 IR thermopile sensor
 *
 * Copyright (c) 2013 Peter Meerwald <pmeerw@pmeerw.net>
 *
 * Driver for the Texas Instruments I2C 16-bit IR thermopile sensor
 *
 * (7-bit I2C slave address 0x40, changeable via ADR pins)
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/pm.h>
#include <linux/bitops.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define TMP006_VOBJECT 0x00
#define TMP006_TAMBIENT 0x01
#define TMP006_CONFIG 0x02
#define TMP006_MANUFACTURER_ID 0xfe
#define TMP006_DEVICE_ID 0xff

#define TMP006_TAMBIENT_SHIFT 2

#define TMP006_CONFIG_RESET BIT(15)
#define TMP006_CONFIG_DRDY_EN BIT(8)
#define TMP006_CONFIG_DRDY BIT(7)

#define TMP006_CONFIG_MOD_MASK GENMASK(14, 12)

#define TMP006_CONFIG_CR_MASK GENMASK(11, 9)
#define TMP006_CONFIG_CR_SHIFT 9

#define TMP006_MANUFACTURER_MAGIC 0x5449
#define TMP006_DEVICE_MAGIC 0x0067

struct tmp006_data {
	struct i2c_client *client;
	u16 config;
	struct iio_trigger *drdy_trig;
};

static int tmp006_read_measurement(struct tmp006_data *data, u8 reg)
{
	s32 ret;
	int tries = 50;

	while (tries-- > 0) {
		ret = i2c_smbus_read_word_swapped(data->client,
			TMP006_CONFIG);
		if (ret < 0)
			return ret;
		if (ret & TMP006_CONFIG_DRDY)
			break;
		msleep(100);
	}

	if (tries < 0)
		return -EIO;

	return i2c_smbus_read_word_swapped(data->client, reg);
}

static const int tmp006_freqs[5][2] = { {4, 0}, {2, 0}, {1, 0},
					{0, 500000}, {0, 250000} };

static int tmp006_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *channel, int *val,
			    int *val2, long mask)
{
	struct tmp006_data *data = iio_priv(indio_dev);
	s32 ret;
	int cr;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (channel->type == IIO_VOLTAGE) {
			/* LSB is 156.25 nV */
			if (!iio_device_claim_direct(indio_dev))
				return -EBUSY;

			ret = tmp006_read_measurement(data, TMP006_VOBJECT);
			iio_device_release_direct(indio_dev);
			if (ret < 0)
				return ret;

			*val = sign_extend32(ret, 15);
		} else if (channel->type == IIO_TEMP) {
			/* LSB is 0.03125 degrees Celsius */
			if (!iio_device_claim_direct(indio_dev))
				return -EBUSY;

			ret = tmp006_read_measurement(data, TMP006_TAMBIENT);
			iio_device_release_direct(indio_dev);
			if (ret < 0)
				return ret;

			*val = sign_extend32(ret, 15) >> TMP006_TAMBIENT_SHIFT;
		} else {
			break;
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (channel->type == IIO_VOLTAGE) {
			*val = 0;
			*val2 = 156250;
		} else if (channel->type == IIO_TEMP) {
			*val = 31;
			*val2 = 250000;
		} else {
			break;
		}
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		cr = (data->config & TMP006_CONFIG_CR_MASK)
			>> TMP006_CONFIG_CR_SHIFT;
		*val = tmp006_freqs[cr][0];
		*val2 = tmp006_freqs[cr][1];
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		break;
	}

	return -EINVAL;
}

static int tmp006_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct tmp006_data *data = iio_priv(indio_dev);
	int ret, i;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(tmp006_freqs); i++)
		if ((val == tmp006_freqs[i][0]) &&
		    (val2 == tmp006_freqs[i][1])) {
			if (!iio_device_claim_direct(indio_dev))
				return -EBUSY;

			data->config &= ~TMP006_CONFIG_CR_MASK;
			data->config |= i << TMP006_CONFIG_CR_SHIFT;

			ret = i2c_smbus_write_word_swapped(data->client,
							   TMP006_CONFIG,
							   data->config);

			iio_device_release_direct(indio_dev);
			return ret;
		}
	return -EINVAL;
}

static IIO_CONST_ATTR(sampling_frequency_available, "4 2 1 0.5 0.25");

static struct attribute *tmp006_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group tmp006_attribute_group = {
	.attrs = tmp006_attributes,
};

static const struct iio_chan_spec tmp006_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_BE,
		}
	},
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 14,
			.storagebits = 16,
			.shift = TMP006_TAMBIENT_SHIFT,
			.endianness = IIO_BE,
		}
	},
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static const struct iio_info tmp006_info = {
	.read_raw = tmp006_read_raw,
	.write_raw = tmp006_write_raw,
	.attrs = &tmp006_attribute_group,
};

static bool tmp006_check_identification(struct i2c_client *client)
{
	int mid, did;

	mid = i2c_smbus_read_word_swapped(client, TMP006_MANUFACTURER_ID);
	if (mid < 0)
		return false;

	did = i2c_smbus_read_word_swapped(client, TMP006_DEVICE_ID);
	if (did < 0)
		return false;

	return mid == TMP006_MANUFACTURER_MAGIC && did == TMP006_DEVICE_MAGIC;
}

static int tmp006_power(struct device *dev, bool up)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct tmp006_data *data = iio_priv(indio_dev);

	if (up)
		data->config |= TMP006_CONFIG_MOD_MASK;
	else
		data->config &= ~TMP006_CONFIG_MOD_MASK;

	return i2c_smbus_write_word_swapped(data->client, TMP006_CONFIG,
		data->config);
}

static void tmp006_powerdown_cleanup(void *dev)
{
	tmp006_power(dev, false);
}

static irqreturn_t tmp006_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct tmp006_data *data = iio_priv(indio_dev);
	struct {
		s16 channels[2];
		aligned_s64 ts;
	} scan = { };
	s32 ret;

	ret = i2c_smbus_read_word_data(data->client, TMP006_VOBJECT);
	if (ret < 0)
		goto err;
	scan.channels[0] = ret;

	ret = i2c_smbus_read_word_data(data->client, TMP006_TAMBIENT);
	if (ret < 0)
		goto err;
	scan.channels[1] = ret;

	iio_push_to_buffers_with_ts(indio_dev, &scan, sizeof(scan),
				    iio_get_time_ns(indio_dev));
err:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int tmp006_set_trigger_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct tmp006_data *data = iio_priv(indio_dev);

	if (state)
		data->config |= TMP006_CONFIG_DRDY_EN;
	else
		data->config &= ~TMP006_CONFIG_DRDY_EN;

	return i2c_smbus_write_word_swapped(data->client, TMP006_CONFIG,
					    data->config);
}

static const struct iio_trigger_ops tmp006_trigger_ops = {
	.set_trigger_state = tmp006_set_trigger_state,
};

static const unsigned long tmp006_scan_masks[] = { 0x3, 0 };

static int tmp006_probe(struct i2c_client *client)
{
	struct iio_dev *indio_dev;
	struct tmp006_data *data;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	if (!tmp006_check_identification(client)) {
		dev_err(&client->dev, "no TMP006 sensor\n");
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	indio_dev->name = dev_name(&client->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &tmp006_info;

	indio_dev->channels = tmp006_channels;
	indio_dev->num_channels = ARRAY_SIZE(tmp006_channels);
	indio_dev->available_scan_masks = tmp006_scan_masks;

	ret = i2c_smbus_read_word_swapped(data->client, TMP006_CONFIG);
	if (ret < 0)
		return ret;
	data->config = ret;

	if ((ret & TMP006_CONFIG_MOD_MASK) != TMP006_CONFIG_MOD_MASK) {
		ret = tmp006_power(&client->dev, true);
		if (ret < 0)
			return ret;
	}

	ret = devm_add_action_or_reset(&client->dev, tmp006_powerdown_cleanup,
				       &client->dev);
	if (ret < 0)
		return ret;

	if (client->irq > 0) {
		data->drdy_trig = devm_iio_trigger_alloc(&client->dev,
							 "%s-dev%d",
							 indio_dev->name,
							 iio_device_id(indio_dev));
		if (!data->drdy_trig)
			return -ENOMEM;

		data->drdy_trig->ops = &tmp006_trigger_ops;
		iio_trigger_set_drvdata(data->drdy_trig, indio_dev);
		ret = iio_trigger_register(data->drdy_trig);
		if (ret)
			return ret;

		indio_dev->trig = iio_trigger_get(data->drdy_trig);

		ret = devm_request_threaded_irq(&client->dev, client->irq,
						iio_trigger_generic_data_rdy_poll,
						NULL,
						IRQF_ONESHOT,
						"tmp006_irq",
						data->drdy_trig);
		if (ret < 0)
			return ret;
	}

	ret = devm_iio_triggered_buffer_setup(&client->dev, indio_dev, NULL,
					      tmp006_trigger_handler, NULL);
	if (ret < 0)
		return ret;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static int tmp006_suspend(struct device *dev)
{
	return tmp006_power(dev, false);
}

static int tmp006_resume(struct device *dev)
{
	return tmp006_power(dev, true);
}

static DEFINE_SIMPLE_DEV_PM_OPS(tmp006_pm_ops, tmp006_suspend, tmp006_resume);

static const struct of_device_id tmp006_of_match[] = {
	{ .compatible = "ti,tmp006" },
	{ }
};
MODULE_DEVICE_TABLE(of, tmp006_of_match);

static const struct i2c_device_id tmp006_id[] = {
	{ "tmp006" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tmp006_id);

static struct i2c_driver tmp006_driver = {
	.driver = {
		.name	= "tmp006",
		.of_match_table = tmp006_of_match,
		.pm	= pm_sleep_ptr(&tmp006_pm_ops),
	},
	.probe = tmp006_probe,
	.id_table = tmp006_id,
};
module_i2c_driver(tmp006_driver);

MODULE_AUTHOR("Peter Meerwald <pmeerw@pmeerw.net>");
MODULE_DESCRIPTION("TI TMP006 IR thermopile sensor driver");
MODULE_LICENSE("GPL");
