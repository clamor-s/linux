// SPDX-License-Identifier: GPL-2.0-only
/*
 * AL3000a - Dyna Image Ambient Light Sensor
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define AL3000A_DRV_NAME		"al3000a"
#define AL3000A_REG_SYSTEM		0x00
#define AL3000A_REG_DATA		0x05

#define AL3000A_CONFIG_ENABLE		0x00
#define AL3000A_CONFIG_DISABLE		0x0B
#define AL3000A_CONFIG_RESET		0x0F

/*
 * This are pre-calculated lux values based on possible output
 * of sensor (range 0x00 - 0x3F). Actual formula is exponetial:
 *
 * 	illuminance = 1.20051 ^ (raw sensor output) 
 *
 * Such calculation can cause unneeded cpu load since illuminance
 * may change constantly and rapitly.
 */
static const u32 lux_table[64] = {
	1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 6, 7, 9, 11, 13, 16,
	19, 22, 27, 32, 39, 46, 56, 67, 80, 96, 116, 139,
	167, 200, 240, 289, 347, 416, 499, 600, 720, 864,
	1037, 1245, 1495, 1795, 2155, 2587, 3105, 3728,	4475,
	5373, 6450, 7743, 9296, 11160, 13397, 16084, 19309,
	23180, 27828, 33408, 40107, 48148, 57803, 69393,
	83306, 100000
};

struct al3000a_data {
	struct i2c_client *client;
	struct regulator *vdd_supply;
};

static const struct iio_chan_spec al3000a_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	}
};

static int al3000a_set_pwr(struct al3000a_data *data, bool pwr)
{
	struct device *dev = &data->client->dev;
	u8 val = pwr ? AL3000A_CONFIG_ENABLE : AL3000A_CONFIG_DISABLE;
	int ret;

	if (pwr) {
		ret = regulator_enable(data->vdd_supply);
		if (ret < 0) {
			dev_err(dev, "failed to enable vdd power supply\n");
			return ret;
		}
	}

	ret = i2c_smbus_write_byte_data(data->client, AL3000A_REG_SYSTEM, val);
	if (ret < 0) {
		dev_err(dev, "failed to write system register\n");
		return ret;
	}

	if (!pwr) {
		ret = regulator_disable(data->vdd_supply);
		if (ret < 0) {
			dev_err(dev, "failed to disable vdd power supply\n");
			return ret;
		}
	}

	return 0;
}

static void al3000a_set_pwr_off(void *_data)
{
	struct al3000a_data *data = _data;

	al3000a_set_pwr(data, false);
}

static int al3000a_init(struct al3000a_data *data)
{
	int ret;

	ret = al3000a_set_pwr(data, true);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(data->client, AL3000A_REG_SYSTEM,
					AL3000A_CONFIG_RESET);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(data->client, AL3000A_REG_SYSTEM,
					AL3000A_CONFIG_ENABLE);
	if (ret < 0)
		return ret;

	return 0;
}

static int al3000a_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct al3000a_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = i2c_smbus_read_byte_data(data->client,
					       AL3000A_REG_DATA);
		if (ret < 0)
			return ret;

		*val = lux_table[ret & 0x3F];

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 1;

		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static const struct iio_info al3000a_info = {
	.read_raw	= al3000a_read_raw,
};

static int al3000a_probe(struct i2c_client *client)
{
	struct al3000a_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	data->vdd_supply = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(data->vdd_supply))
		return dev_err_probe(&client->dev, PTR_ERR(data->vdd_supply),
				     "failed to get vdd regulator\n");

	indio_dev->info = &al3000a_info;
	indio_dev->name = AL3000A_DRV_NAME;
	indio_dev->channels = al3000a_channels;
	indio_dev->num_channels = ARRAY_SIZE(al3000a_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = al3000a_init(data);
	if (ret < 0)
		return dev_err_probe(&client->dev, ret,
				     "failed to init ALS\n");

	ret = devm_add_action_or_reset(&client->dev, al3000a_set_pwr_off,
				       data);
	if (ret < 0)
		return dev_err_probe(&client->dev, ret,
				     "failed to add action\n");

	return devm_iio_device_register(&client->dev, indio_dev);
}

static int al3000a_suspend(struct device *dev)
{
	struct al3000a_data *data = iio_priv(dev_get_drvdata(dev));

	return al3000a_set_pwr(data, false);
}

static int al3000a_resume(struct device *dev)
{
	struct al3000a_data *data = iio_priv(dev_get_drvdata(dev));

	return al3000a_set_pwr(data, true);
}

static DEFINE_SIMPLE_DEV_PM_OPS(al3000a_pm_ops, al3000a_suspend, al3000a_resume);

static const struct of_device_id al3000a_of_match[] = {
	{ .compatible = "dynaimage,al3000a" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, al3000a_of_match);

static struct i2c_driver al3000a_driver = {
	.driver = {
		.name = AL3000A_DRV_NAME,
		.of_match_table = al3000a_of_match,
		.pm = pm_sleep_ptr(&al3000a_pm_ops),
	},
	.probe_new = al3000a_probe,
};
module_i2c_driver(al3000a_driver);

MODULE_AUTHOR("Svyatolsav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("al3000a Ambient Light Sensor driver");
MODULE_LICENSE("GPL v2");
