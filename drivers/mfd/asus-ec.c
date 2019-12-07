// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ASUS EC driver
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/mfd/asus-ec.h>

#define ASUSEC_SMI_HANDSHAKE		0x50
#define ASUSEC_SMI_RESET		0x5F

#define ASUSEC_CTL_SUSB_MODE		BIT_ULL(9)
#define ASUSEC_CTL_FACTORY_MODE		BIT_ULL(38)

#define ASUSEC_RSP_BUFFER_SIZE		8

struct asus_ec_data {
	struct asusec_info	info;
	struct mutex		ecreq_lock;
	struct gpio_desc	*ecreq;
	struct i2c_client	*self;
	u8			ec_data[DOCKRAM_ENTRY_BUFSIZE];
	bool			logging_disabled;
};

#define to_ec_data(ec) \
	container_of(ec, struct asus_ec_data, info)

static void asusec_remove_notifier(struct device *dev, void *res)
{
	struct asusec_info *ec = dev_get_drvdata(dev->parent);
	struct notifier_block **nb = res;

	blocking_notifier_chain_unregister(&ec->notify_list, *nb);
}

int devm_asusec_register_notifier(struct platform_device *pdev,
				  struct notifier_block *nb)
{
	struct asusec_info *ec = dev_get_drvdata(pdev->dev.parent);
	struct notifier_block **res;
	int ret;

	res = devres_alloc(asusec_remove_notifier, sizeof(*res), GFP_KERNEL);
	if (!res)
		return -ENOMEM;

	*res = nb;
	ret = blocking_notifier_chain_register(&ec->notify_list, nb);
	if (ret) {
		devres_free(res);
		return ret;
	}

	devres_add(&pdev->dev, res);
	return 0;
}
EXPORT_SYMBOL_GPL(devm_asusec_register_notifier);

static int asus_ec_signal_request(const struct asusec_info *ec)
{
	struct asus_ec_data *priv = to_ec_data(ec);

	mutex_lock(&priv->ecreq_lock);

	dev_dbg(&priv->self->dev, "EC request\n");

	gpiod_set_value_cansleep(priv->ecreq, 1);
	msleep(50);

	gpiod_set_value_cansleep(priv->ecreq, 0);
	msleep(200);

	mutex_unlock(&priv->ecreq_lock);

	return 0;
}

static int asus_ec_write(struct asus_ec_data *priv, u16 data)
{
	int ret = i2c_smbus_write_word_data(priv->self, 0x64, data);

	dev_dbg(&priv->self->dev, "EC write: %04x, ret = %d\n", data, ret);
	return ret;
}

static int asus_ec_read(struct asus_ec_data *priv, bool in_irq)
{
	int ret = i2c_smbus_read_i2c_block_data(priv->self, 0x6A,
						sizeof(priv->ec_data),
						priv->ec_data);

	dev_dbg(&priv->self->dev, "EC read: %*ph, ret = %d%s\n",
		sizeof(priv->ec_data), priv->ec_data,
		ret, in_irq ? "; in irq" : "");
	return ret;
}

int asusec_i2c_command(const struct asusec_info *ec, u16 data)
{
	return asus_ec_write(to_ec_data(ec), data);
}
EXPORT_SYMBOL_GPL(asusec_i2c_command);

static void asus_ec_clear_buffer(struct asus_ec_data *priv)
{
	int retry = ASUSEC_RSP_BUFFER_SIZE;

	while (retry--) {
		if (asus_ec_read(priv, false) < 0)
			continue;

		if (priv->ec_data[1] & ASUSEC_OBF_MASK)
			continue;

		break;
	}
}

static int asus_ec_log_info(struct asus_ec_data *priv, unsigned int reg,
			    const char *name, char **out)
{
	char buf[DOCKRAM_ENTRY_BUFSIZE];
	int ret;

	ret = asus_dockram_read(priv->info.dockram, reg, buf);
	if (ret < 0)
		return ret;

	if (!priv->logging_disabled)
		dev_info(&priv->self->dev, "%-14s: %.*s\n", name, buf[0], buf + 1);

	if (out)
		*out = kstrndup(buf + 1, buf[0], GFP_KERNEL);

	return 0;
}

static int asus_ec_reset(struct asus_ec_data *priv)
{
	int retry, ret;

	for (retry = 0; retry < 3; ++retry) {
		ret = asus_ec_write(priv, 0);
		if (!ret)
			return 0;

		msleep(300);
	}

	return ret;
}

static int asus_ec_magic_debug(struct asus_ec_data *priv)
{
	u64 flag;
	int ret;

	ret = asusec_get_ctl(&priv->info, &flag);
	if (ret < 0)
		return ret;

	flag &= ASUSEC_CTL_SUSB_MODE;
	dev_info(&priv->self->dev, "EC FW behaviour: %s\n",
		 flag ? "susb on when receive ec_req" : "susb on when system wakeup");

	return 0;
}

static int asus_ec_set_factory_mode(struct asus_ec_data *priv, bool on)
{
	dev_info(&priv->self->dev, "Entering %s mode.\n", on ? "factory" : "normal");
	return asusec_update_ctl(&priv->info, ASUSEC_CTL_FACTORY_MODE,
				 on ? ASUSEC_CTL_FACTORY_MODE : 0);
}

static void asus_ec_handle_smi(struct asus_ec_data *priv, unsigned int code);

static irqreturn_t asus_ec_interrupt(int irq, void *dev_id)
{
	struct asus_ec_data *priv = dev_id;
	unsigned long notify_action;
	int ret;

	ret = asus_ec_read(priv, true);

        if (ret <= 0 || !(priv->ec_data[1] & ASUSEC_OBF_MASK))
		return IRQ_NONE;

	notify_action = priv->ec_data[1];
	if (notify_action & ASUSEC_SMI_MASK) {
		unsigned int code = priv->ec_data[2];

		asus_ec_handle_smi(priv, code);

		notify_action |= code << 8;
		dev_dbg(&priv->self->dev, "SMI code: 0x%02x\n", code);
	}

	blocking_notifier_call_chain(&priv->info.notify_list,
				     notify_action, priv->ec_data);

	return IRQ_HANDLED;
}

static void asus_ec_remove_subdev(void *subdev)
{
	platform_device_unregister(subdev);
}

static int asus_ec_init_components(struct asus_ec_data *priv)
{
	struct device *dev = &priv->self->dev;
	struct platform_device_info subdev_info;
	struct platform_device *subdev;
	struct fwnode_handle *child;
	int ret;

	memset(&subdev_info, 0, sizeof(subdev_info));
	subdev_info.parent = dev;
	subdev_info.id = PLATFORM_DEVID_AUTO;

	device_for_each_child_node(dev, child) {
		if (!fwnode_device_is_available(child))
			continue;

		subdev_info.fwnode = child;
		subdev_info.name = fwnode_get_name(child);
		subdev = platform_device_register_full(&subdev_info);
		if (IS_ERR(subdev))
			return dev_err_probe(dev, PTR_ERR(subdev),
					     "register subdev for %s",
					     subdev_info.name);

		ret = devm_add_action_or_reset(dev, asus_ec_remove_subdev, subdev);
		if (ret)
			return dev_err_probe(dev, ret,
					     "register subdev cleanup action for %s",
					     subdev_info.name);
	}

	return 0;
}

static int asus_ec_detect(struct asus_ec_data *priv)
{
	char *model = NULL;
	int ret;

	ret = asus_ec_reset(priv);
	if (ret)
		goto err_exit;

	asus_ec_clear_buffer(priv);

	ret = asus_ec_log_info(priv, 0x01, "model", &model);
	if (ret)
		goto err_exit;

	ret = asus_ec_log_info(priv, 0x02, "FW version", NULL);
	if (ret)
		goto err_exit;

	ret = asus_ec_log_info(priv, 0x03, "Config format", NULL);
	if (ret)
		goto err_exit;

	ret = asus_ec_log_info(priv, 0x04, "HW version", NULL);
	if (ret)
		goto err_exit;

	priv->logging_disabled = true;

	ret = asus_ec_magic_debug(priv);
	if (ret)
		goto err_exit;

	priv->info.model = model;
	priv->info.name = of_device_get_match_data(&priv->self->dev);

	if (device_property_read_bool(&priv->self->dev, "asus,clear-factory-mode"))
		asus_ec_set_factory_mode(priv, false);

err_exit:
	if (ret)
		dev_err(&priv->self->dev, "failed to access EC: %d\n", ret);

	return ret;
}

static void asus_ec_handle_smi(struct asus_ec_data *priv, unsigned int code)
{
	dev_dbg(&priv->self->dev, "SMI interrupt: 0x%02x\n", code);

	switch (code) {
	case ASUSEC_SMI_HANDSHAKE:
	case ASUSEC_SMI_RESET:
		asus_ec_detect(priv);
		break;
	}
}

static ssize_t ec_request_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct asusec_info *ec = dev_get_drvdata(dev);

	asus_ec_signal_request(ec);

	return count;
}

static ssize_t ec_irq_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct asusec_info *ec = dev_get_drvdata(dev);
	struct asus_ec_data *priv = to_ec_data(ec);

	irq_wake_thread(priv->self->irq, priv);

	return count;
}

static DEVICE_ATTR_WO(ec_request);
static DEVICE_ATTR_WO(ec_irq);

static struct attribute *asus_ec_attributes[] = {
	&dev_attr_ec_request.attr,
	&dev_attr_ec_irq.attr,
	NULL
};

static const struct attribute_group asus_ec_attr_group = {
	.attrs = asus_ec_attributes,
};

static void asus_ec_sysfs_release(void *data)
{
	struct i2c_client *client = data;
	sysfs_remove_link(&client->dev.kobj, "dockram");
}

static int asus_ec_probe(struct i2c_client *client)
{
	struct asus_ec_data *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->self = client;

	priv->info.dockram = devm_asus_dockram_get(&client->dev);
	if (IS_ERR(priv->info.dockram))
		return dev_err_probe(&client->dev, PTR_ERR(priv->info.dockram),
				     "failed to get dockram\n");

	priv->ecreq = devm_gpiod_get(&client->dev, "request", GPIOD_OUT_LOW);
	if (IS_ERR(priv->ecreq))
		return dev_err_probe(&client->dev, PTR_ERR(priv->ecreq),
				     "failed to get request GPIO\n");

	BLOCKING_INIT_NOTIFIER_HEAD(&priv->info.notify_list);
	mutex_init(&priv->ecreq_lock);

	ret = devm_device_add_group(&client->dev, &asus_ec_attr_group);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to create sysfs attributes\n");

	ret = sysfs_create_link(&client->dev.kobj,
				&priv->info.dockram->dev.kobj,
				"dockram");
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to create sysfs link 'dockram'\n");

	ret = devm_add_action_or_reset(&client->dev, asus_ec_sysfs_release,
				       client);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to register sysfs release\n");

	asus_ec_signal_request(&priv->info);

	ret = asus_ec_detect(priv);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to detect EC version\n");

	ret = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, &asus_ec_interrupt,
					IRQF_ONESHOT | IRQF_SHARED,
					client->name, priv);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to register IRQ\n");

	ret = asus_ec_init_components(priv);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to init components\n");

	return 0;
}

static const struct of_device_id asus_ec_match[] = {
	{ .compatible = "asus,ec-pad", .data = "pad" },
	{ .compatible = "asus,ec-dock", .data = "dock" },
	{ }
};
MODULE_DEVICE_TABLE(of, asus_ec_match);

static struct i2c_driver asus_ec_driver = {
	.driver	= {
		.name = "asus-ec",
		.of_match_table = asus_ec_match,
	},
	.probe_new = asus_ec_probe,
};
module_i2c_driver(asus_ec_driver);

MODULE_AUTHOR("Michał Mirosław <mirq-linux@rere.qmqm.pl>");
MODULE_DESCRIPTION("ASUS Transformer's EC driver");
MODULE_LICENSE("GPL");
