// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ASUS i2c ELAN touchpad
 */

#include <linux/i2c.h>
#include <linux/i8042.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/serio.h>
#include <linux/slab.h>

struct asus_i2c_serio_data {
	struct i2c_client		*client;
	struct serio 			*sdev;

//	struct notifier_block		nb;
//	struct blocking_notifier_head	notify_list;
};

enum etd_i2c_command {
	ETP_HID_WAKE_UP_CMD,
	ETP_HID_READ_DATA_CMD
};

const u8 etp_hid_wake_up_cmd[] = { 0x22, 0x00, 0x00, 0x08 };
const u8 etp_hid_read_data_cmd[] = { 0x24, 0x00 };

static int elantech_i2c_command(struct i2c_client *client, int command,
				unsigned char *buf_recv, int data_len)
{
	const u8 *cmd = NULL;
	unsigned char *rec_buf = buf_recv;
	int ret;
	int tries = 3;
	int length = 0;
	struct i2c_msg msg[2];
	int msg_num = 0;

	switch (command) {
	case ETP_HID_WAKE_UP_CMD:
		cmd = etp_hid_wake_up_cmd;
		length = sizeof(etp_hid_wake_up_cmd);
		msg_num = 1;
		break;
	case ETP_HID_READ_DATA_CMD:
		cmd = etp_hid_read_data_cmd;
		length = sizeof(etp_hid_read_data_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
	default:
		return -1;
	}

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = length;
	msg[0].buf = (char *) cmd;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].buf = rec_buf;

	do {
		ret = i2c_transfer(client->adapter, msg, msg_num);
		if (ret > 0)
			break;
		tries--;
		msleep(10);
	} while (tries > 0);

	return ret;
}

#define TP_RET_FA_OFFSET	2
#define PSMOUSE_RET_ACK		0xfa
#define PSMOUSE_RET_NAK		0xfe

static irqreturn_t asus_i2c_serio_irq(int irq, void *dev_id)
{
	struct asus_i2c_serio_data *priv = dev_id;
	struct i2c_client *client = priv->client;
	u8 i2c_data[16];
	unsigned int n;
	int ret = 0;

	dev_err(&client->dev, "asus_i2c_serio_irq: called\n");

	ret = elantech_i2c_command(client, ETP_HID_READ_DATA_CMD, i2c_data, 10);
	if (ret < 0) {
		dev_err(&client->dev, "Fail to read data, status %d\n", ret);
		return IRQ_NONE;
	}

	dev_err(&client->dev, "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		i2c_data[0], i2c_data[1], i2c_data[2], i2c_data[3],
		i2c_data[4], i2c_data[5], i2c_data[6], i2c_data[7]);
	dev_err(&client->dev, "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		i2c_data[8], i2c_data[9], i2c_data[10], i2c_data[11],
		i2c_data[12], i2c_data[13], i2c_data[14], i2c_data[15]);

//	if (i2c_data[4] == PSMOUSE_RET_ACK) {
//		n = i2c_data[0] - 1;
//		i2c_data += 4;
//
//		while (n--)
//			serio_interrupt(priv->sdev, *i2c_data++, 0);
//	};

	return IRQ_NONE;
}

#if 0
static int asus_i2c_serio_notify(struct notifier_block *nb,
				 unsigned long action, void *data_)
{
	struct asus_i2c_serio_data *priv =
			container_of(nb, struct asus_i2c_serio_data, nb);
	struct i2c_client *client = priv->client;
	u8 i2c_data[16];
	u8 *data = data_;

	dev_err(&client->dev, "asus_i2c_serio_irq: called\n");

	ret = elantech_i2c_command(client, ETP_HID_READ_DATA_CMD, i2c_data, 10);
	if (ret < 0) {
		dev_err(&client->dev, "Fail to read data, status %d\n", ret);
		return NOTIFY_DONE;
	}

	n = data[0] - 1;
	data += 2;

	while (n--)
		serio_interrupt(priv->sdev[port_idx], *data++, 0);

	return NOTIFY_DONE;
}
#endif

static int asus_i2c_serio_write(struct serio *port, unsigned char data)
{
	struct asus_i2c_serio_data *priv = port->port_data;
	struct i2c_client *client = priv->client;

	u8 i2c_payload[16] = { 0x00, 0x05, 0x00, 0x22, port->id.extra, data };

	return i2c_smbus_write_i2c_block_data(client, 0x25, 7, i2c_payload);
}

static int asus_i2c_serio_probe(struct i2c_client *client)
{
	struct asus_i2c_serio_data *priv;
	struct serio *port;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->client = client;

	port = devm_kzalloc(&client->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	priv->sdev = port;
	port->dev.parent = &client->dev;
	port->id.type = SERIO_8042;
	port->id.extra = I8042_CMD_AUX_SEND & 0xFF;
	port->write = asus_i2c_serio_write;
	port->port_data = (void *)priv;
	snprintf(port->name, sizeof(port->name), "%s", "Touchpad");
	snprintf(port->phys, sizeof(port->phys), "i2c-%u-%04x/serio%d",
		 i2c_adapter_id(client->adapter), client->addr, 1);

	serio_register_port(port);

	/* tp power on */
	elantech_i2c_command(client, ETP_HID_WAKE_UP_CMD, NULL, 0);
	msleep(50);


	ret = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, &asus_i2c_serio_irq,
					IRQF_ONESHOT | IRQF_SHARED,
					client->name, priv);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to register IRQ\n");

#if 0
	priv->nb.notifier_call = asus_i2c_serio_notify;
	ret = blocking_notifier_chain_register(&priv->notify_list, &priv->nb);
#endif
	return ret;
}

static void asus_i2c_serio_remove(struct i2c_client *client)
{
	struct asus_i2c_serio_data *priv = i2c_get_clientdata(client);

//	blocking_notifier_chain_unregister(&priv->notify_list, &priv->nb);
	serio_unregister_port(priv->sdev);
}

static const struct of_device_id asus_i2c_serio_match[] = {
	{ .compatible = "elan,ekth1059" },
	{ }
};
MODULE_DEVICE_TABLE(of, asus_i2c_serio_match);

static struct i2c_driver asus_i2c_serio_driver = {
	.driver	= {
		.name = "asus-i2c-serio",
		.of_match_table = asus_i2c_serio_match,
	},
	.probe_new = asus_i2c_serio_probe,
	.remove = asus_i2c_serio_remove,
};
module_i2c_driver(asus_i2c_serio_driver);

MODULE_AUTHOR("Michał Mirosław <mirq-linux@rere.qmqm.pl>");
MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("ASUS Transformer's TF701T Dock touchpad controller driver");
MODULE_LICENSE("GPL");
