// SPDX-License-Identifier: GPL-2.0-only
/*
 * Raydium touchscreen I2C driver.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#define RM_INPUT_RESOLUTION_X		4096
#define RM_INPUT_RESOLUTION_Y		4096
#define RM_MAX_TOUCH_NUM		10

#define RM31080_REG_01 0x01
#define RM31080_REG_02 0x02
#define RM31080_REG_09 0x09
#define RM31080_REG_0E 0x0E
#define RM31080_REG_10 0x10
#define RM31080_REG_11 0x11
#define RM31080_REG_1F 0x1F
#define RM31080_REG_40 0x40
#define RM31080_REG_41 0x41
#define RM31080_REG_80 0x80
#define RM31080_REG_F2 0xF2

struct rm31080_priv {
	struct spi_device *spi;
	struct device *dev;
	struct input_dev *input;

	struct clk *ref_clk;

	struct gpio_desc *reset_gpio;

	struct regulator *vdd;
	struct regulator *vio;
};

static int rm31080_spi_raw_read(struct rm31080_priv *priv,
				u8 addr, u8 *rxbuf, size_t len)
{
	struct spi_device *spi = priv->spi;
	struct spi_message msg;
	struct spi_transfer xfer[2];
	int ret;

	spi_message_init(&msg);
	memset(&xfer, 0, sizeof(xfer));

	addr |= BIT(7);

	xfer[0].tx_buf = &addr;
	xfer[0].len = 1;
	spi_message_add_tail(&xfer[0], &msg);

	xfer[1].rx_buf = rxbuf;
	xfer[1].len = len;
	spi_message_add_tail(&xfer[1], &msg);

	ret = spi_sync(spi, &msg);
	if (ret)
		dev_err(&spi->dev, "spi_sync_read failed %d\n", ret);

	return ret;
}

static int rm31080_spi_byte_read(struct rm31080_priv *priv, u8 addr)
{
	int ret;
	u8 data;

	ret = rm31080_spi_raw_read(priv, addr, &data, 1);
	if (ret) {
		dev_err(&priv->spi->dev, "get data from 0x%x failed with %d\n",
			addr, ret);
		return ret;
	}

	return data;
}

static int rm31080_spi_byte_write(struct rm31080_priv *priv,
				  u8 addr, u8 data)
{
	struct spi_device *spi = priv->spi;
	u8 buf[2];
	int ret;

	buf[0] = addr;
	buf[1] = data;

	ret = spi_write(spi, buf, 2);
	if (ret) {
		dev_err(&priv->spi->dev, "write 0x%x > 0x%x failed with %d\n",
			data, addr, ret);
		return ret;
	}

	return 0;
}

static void rm31080_ctrl_enter_auto_mode(struct rm31080_priv *priv)
{
	rm31080_spi_byte_write(priv, RM31080_REG_09, BIT(4) | BIT(6));
}

static void rm31080_ctrl_leave_auto_mode(struct rm31080_priv *priv)
{
	rm31080_spi_byte_write(priv, RM31080_REG_09, 0x00);
}

static void rm31080_ctrl_scan_start(struct rm31080_priv *priv)
{
	rm31080_spi_byte_write(priv, RM31080_REG_11, 0x17);
}

static irqreturn_t rm31080_irq(int irq, void *_dev)
{
	struct rm31080_priv *priv = _dev;

	dev_err(priv->dev, "INTERRUPT CALLED!\n");

	return IRQ_HANDLED;
}

static int rm31080_power_on(struct rm31080_priv *priv)
{
	int error;

	error = regulator_enable(priv->vio);
	if (error) {
		dev_err(priv->dev, "failed to enable vio regulator: %d\n", error);
		return error;
	}
	usleep_range(5000, 6000);

	error = regulator_enable(priv->vdd);
	if (error) {
		regulator_disable(priv->vio);
		dev_err(priv->dev, "failed to enable vdd regulator: %d\n", error);
		return error;
	}
	usleep_range(5000, 6000);

	error = clk_prepare_enable(priv->ref_clk);
	if (error < 0) {
		regulator_disable(priv->vdd);
		regulator_disable(priv->vio);
		dev_err(priv->dev, "error enabling tx_clk (%d)\n", error);
		return error;
	}
	usleep_range(5000, 6000);

	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	msleep(50);

	return 0;
}

static void rm31080_power_off(void *_data)
{
	struct rm31080_priv *priv = _data;

	if (priv->reset_gpio)
		gpiod_set_value_cansleep(priv->reset_gpio, 1);

	regulator_disable(priv->vio);
	regulator_disable(priv->vdd);
}

static int rm31080_probe(struct spi_device *spi)
{
	struct rm31080_priv *priv;
	struct device *dev = &spi->dev;
	int error;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	error = spi_setup(spi);
	if (error)
		return dev_err_probe(dev, error,
				     "spi_setup failed\n");

	priv->spi = spi;
	spi_set_drvdata(spi, priv);

	priv->dev = dev;
	dev_set_drvdata(dev, priv);

	priv->ref_clk = devm_clk_get_optional(dev, "ref_clk");
	if (IS_ERR(priv->ref_clk))
		return dev_err_probe(dev, PTR_ERR(priv->ref_clk),
				     "can't retrieve ref_clk\n");

	priv->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(priv->vdd))
		return dev_err_probe(dev, PTR_ERR(priv->vdd),
				     "failed to get vdd supply\n");

	priv->vio = devm_regulator_get(dev, "vio");
	if (IS_ERR(priv->vio))
		return dev_err_probe(dev, PTR_ERR(priv->vio),
				     "failed to get vio supply\n");

	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						   GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "failed to get reset GPIO\n");

	error = rm31080_power_on(priv);
	if (error)
		return error;

	error = devm_add_action_or_reset(dev, rm31080_power_off, priv);
	if (error)
		return dev_err_probe(dev, error,
				     "failed to install power off action\n");

	priv->input = devm_input_allocate_device(dev);
	if (IS_ERR(priv->input))
		return dev_err_probe(dev, PTR_ERR(priv->input),
				     "failed to allocate input device\n");

	priv->input->name = "Raydium RM31080";
	priv->input->id.bustype = BUS_SPI;

	__set_bit(EV_ABS, priv->input->evbit);
	__set_bit(ABS_X, priv->input->absbit);
	__set_bit(ABS_Y, priv->input->absbit);
	__set_bit(ABS_PRESSURE, priv->input->absbit);

	__set_bit(EV_KEY, priv->input->evbit);
	__set_bit(BTN_TOUCH, priv->input->keybit);

	/* For single touch */
	input_set_abs_params(priv->input, ABS_X, 0,
			     RM_INPUT_RESOLUTION_X - 1, 0, 0);
	input_set_abs_params(priv->input, ABS_Y, 0,
			     RM_INPUT_RESOLUTION_Y - 1, 0, 0);
	input_set_abs_params(priv->input, ABS_PRESSURE, 0,
			     1, 0, 0);

	/* For multi touch */
	input_set_abs_params(priv->input, ABS_MT_TOUCH_MAJOR, 0,
			     0xFF, 0, 0);
	input_set_abs_params(priv->input, ABS_MT_POSITION_X, 0,
			     RM_INPUT_RESOLUTION_X - 1, 0, 0);
	input_set_abs_params(priv->input, ABS_MT_POSITION_Y, 0,
			     RM_INPUT_RESOLUTION_Y - 1, 0, 0);
	input_set_abs_params(priv->input, ABS_MT_TRACKING_ID, 0,
			     32, 0, 0);

	error = input_mt_init_slots(priv->input, RM_MAX_TOUCH_NUM,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return dev_err_probe(dev, error,
				     "failed to initialize MT slots\n");

	error = input_register_device(priv->input);
	if (error)
		return dev_err_probe(dev, error,
				     "unable to register input device\n");

	error = devm_request_threaded_irq(dev, spi->irq,
					  NULL, rm31080_irq,
					  IRQF_ONESHOT, "rm31080_ts", priv);
	if (error)
		return dev_err_probe(dev, error,
				     "error requesting irq\n");

	/* clear interrupt register */
	rm31080_spi_byte_read(priv, RM31080_REG_F2);

	rm31080_ctrl_enter_auto_mode(priv);

	return 0;
}

static int __maybe_unused rm31080_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused rm31080_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(rm31080_pm_ops,
			 rm31080_suspend, rm31080_resume);

static const struct of_device_id rm31080_of_match[] = {
	{ .compatible = "raydium,rm31080", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rm31080_of_match);

static struct spi_driver rm31080_driver = {
	.driver = {
		.name = "rm31080",
		.of_match_table = rm31080_of_match,
		.pm = &rm31080_pm_ops,
	},
	.probe = rm31080_probe,
//	.remove = rm31080_remove,
};
module_spi_driver(rm31080_driver);

MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("Raydium RM31080 Touchscreen driver");
MODULE_LICENSE("GPL v2");
