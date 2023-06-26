// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2011 NVIDIA Corporation
 * Copyright (C) 2023 Svyatoslav Ryhel <clamor95@gmail.com>
 */

#include <linux/devm-helpers.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

#define ENUM_REPEAT_TRY_CNT 3
#define MODEM_ENUM_TIMEOUT_500MS 16 /* 8 sec */
#define MODEM_ENUM_TIMEOUT_200MS 25 /* 5 sec */

enum ipc_ap_wake_state {
	IPC_AP_WAKE_IRQ_READY,
	IPC_AP_WAKE_INIT1,
	IPC_AP_WAKE_INIT2,
	IPC_AP_WAKE_L,
	IPC_AP_WAKE_H,
	IPC_AP_WAKE_UNINIT,
};

struct baseband_xmm_power_data {
	struct device *dev;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;

	struct gpio_desc *ipc_cp_gpio;
	struct gpio_desc *ipc_ap_gpio;

	struct regulator *vbat_supply;

	struct device_node *hsic_node;
	struct work_struct modem_work;

	int irq_hostwake;
	enum ipc_ap_wake_state ap_state;
};

static void baseband_xmm_reset(struct baseband_xmm_power_data *priv)
{
	int ret;

	ret = regulator_enable(priv->vbat_supply);
	if (ret < 0)
		dev_err(priv->dev, "failed to enable vbat power supply\n");

	/* reset / power on sequence */
	gpiod_set_value_cansleep(priv->enable_gpio, 0);
	msleep(50);

	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	msleep(200);
	gpiod_set_value_cansleep(priv->reset_gpio, 0);

	msleep(50);
	
	gpiod_set_value_cansleep(priv->enable_gpio, 1);
	udelay(60);
	gpiod_set_value_cansleep(priv->enable_gpio, 0);
	msleep(20);
}

static void baseband_xmm_power_off(struct baseband_xmm_power_data *priv)
{
	gpiod_set_value_cansleep(priv->enable_gpio, 0);
	gpiod_set_value_cansleep(priv->reset_gpio, 1);

	regulator_disable(priv->vbat_supply);
}

static void baseband_xmm_power_work(struct work_struct *work)
{
	struct baseband_xmm_power_data *priv =
		container_of(work, struct baseband_xmm_power_data, modem_work);
	struct platform_device *hsic_line;
	int timeout_500ms = MODEM_ENUM_TIMEOUT_500MS;
	int timeout_200ms = 0;
	bool enum_success = false;

	dev_err(priv->dev, "%s: registering controller\n", __func__);
	hsic_line = of_platform_device_create(priv->hsic_node, NULL, priv->dev);

	/* waiting ap_wake up */
	while (priv->ap_state == IPC_AP_WAKE_INIT1 && timeout_500ms--) {
		dev_err(priv->dev, "%s: waiting loop # %d\n", __func__, timeout_500ms);
		msleep(500);
	}

	if (priv->ap_state != IPC_AP_WAKE_INIT2) {
		dev_err(priv->dev, "%s: ap_state is not IPC_AP_WAKE_INIT2\n", __func__);
		timeout_200ms = MODEM_ENUM_TIMEOUT_200MS;
	}

	/* check if enumeration succeeded */
	/* waiting ttyACM dev to be created */
	do {
		struct file *filp;
		filp = filp_open("/dev/ttyACM0",
			O_RDONLY, 0);
		if (filp && !IS_ERR(filp)) {
			pr_info("ttyACM0 created OK\n");
			enum_success = true;
			filp_close(filp, NULL);
			break;
		}

		dev_err(priv->dev, "%s: no /dev/ttyACM0 # %d\n", __func__, timeout_200ms);
		msleep(200);
	} while (++timeout_200ms <= MODEM_ENUM_TIMEOUT_200MS);

	if (!enum_success) {
		dev_err(priv->dev, "%s: enum_success is false\n", __func__);
		priv->ap_state = IPC_AP_WAKE_IRQ_READY;

		/* unregister usb host controller */
		dev_err(priv->dev, "%s: deregistering controller\n", __func__);
		of_platform_device_destroy(&hsic_line->dev, NULL);
		msleep(500);
	}
}

static irqreturn_t baseband_hostwake_interrupt(int irq, void *dev_id)
{
	struct baseband_xmm_power_data *priv = dev_id;
	int state;

	state = gpiod_get_value(priv->ipc_ap_gpio);
	dev_err(priv->dev, "hostwake: %d, ap_state: %d\n", state, priv->ap_state);

	switch (priv->ap_state) {
	case IPC_AP_WAKE_IRQ_READY:
		if (!state) {
			priv->ap_state = IPC_AP_WAKE_INIT1;
			schedule_work(&priv->modem_work);
		}

		dev_err(priv->dev, "hostwake: %d, ap_state: %d\n", state, priv->ap_state);
		break;

	case IPC_AP_WAKE_INIT1:
		if (state)
			priv->ap_state = IPC_AP_WAKE_INIT2;

		dev_err(priv->dev, "hostwake: %d, ap_state: %d\n", state, priv->ap_state);
		break;

	default:
		if (state)
			priv->ap_state = IPC_AP_WAKE_H;
		else
			priv->ap_state = IPC_AP_WAKE_L;

		dev_err(priv->dev, "hostwake: %d, ap_state: %d\n", state, priv->ap_state);
		break;
	}

	return IRQ_HANDLED;
}

static int baseband_xmm_power_probe(struct platform_device *pdev)
{
	struct baseband_xmm_power_data *priv;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	platform_set_drvdata(pdev, priv);

	/* There should be only one child - usb line in HSIC */
	for_each_child_of_node(np, child) {
		priv->hsic_node = child;
	}

	priv->vbat_supply = devm_regulator_get_optional(dev, "vbat");
	if (IS_ERR(priv->vbat_supply))
		return dev_err_probe(dev, PTR_ERR(priv->vbat_supply),
				     "failed to get vbat regulator\n");

	/* Own modem gpios */
	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						  GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "failed to get reset GPIO\n");

	priv->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						  GPIOD_OUT_LOW);
	if (IS_ERR(priv->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->enable_gpio),
				     "failed to get enable GPIO\n");

	/* CP - AP connections */
	priv->ipc_cp_gpio = devm_gpiod_get_optional(dev, "link-slavewake",
						  GPIOD_OUT_HIGH);
	if (IS_ERR(priv->ipc_cp_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->ipc_cp_gpio),
				     "failed to get CLIENT WAKE GPIO\n");

	priv->ipc_ap_gpio = devm_gpiod_get_optional(dev, "link-hostwake",
						  GPIOD_IN);
	if (IS_ERR(priv->ipc_ap_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->ipc_ap_gpio),
				     "failed to get HOST WAKE GPIO\n");

	/* ver 1145 or later starts in READY state */
	/* ap_wake keeps low util CP starts to initiate hsic hw. */
	/* ap_wake goes up during cp hsic init and then */
	/* it goes down when cp hsic ready */
	priv->ap_state = IPC_AP_WAKE_IRQ_READY;

	baseband_xmm_reset(priv);

	devm_work_autocancel(dev, &priv->modem_work, baseband_xmm_power_work);

	priv->irq_hostwake = platform_get_irq(pdev, 0);
	if (priv->irq_hostwake < 0)
		return dev_err_probe(&pdev->dev, priv->irq_hostwake,
				     "failed to get IRQ %d\n", priv->irq_hostwake);

	ret = devm_request_irq(dev, priv->irq_hostwake,
			       baseband_hostwake_interrupt,
			       IRQF_NO_SUSPEND,
			       "modem-hostwake", priv);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register IRQ %d\n", priv->irq_hostwake);

	dev_err(dev, "Infineon Technologies Flash Loader device attached\n");

	return 0;
}

static int baseband_xmm_power_remove(struct platform_device *pdev)
{
	struct baseband_xmm_power_data *priv = platform_get_drvdata(pdev);

	baseband_xmm_power_off(priv);

	return 0;
}

static const struct of_device_id baseband_xmm_power_match[] = {
	{ .compatible = "infineon,xmm6260-power" },
	{ }
};
MODULE_DEVICE_TABLE(of, baseband_xmm_power_match);

static struct platform_driver baseband_xmm_power_driver = {
	.driver = {
		.name		= "baseband-xmm-power",
		.of_match_table	= baseband_xmm_power_match,
	},
	.probe = baseband_xmm_power_probe,
	.remove = baseband_xmm_power_remove,
};
module_platform_driver(baseband_xmm_power_driver);

MODULE_DESCRIPTION("Baseband power supply driver");
MODULE_LICENSE("GPL");
