// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012 Synaptics Incorporated
 */

#include <linux/kernel.h>
#include <linux/rmi.h>
#include <linux/input.h>
#include <linux/property.h>
#include <linux/slab.h>
#include "rmi_driver.h"

#define QUERY_BASE_INDEX 1
#define MAX_NAME_LEN 256
#define MAX_BUFFER_LEN 80

#define FILTER_MODE_MIN				0
#define FILTER_MODE_MAX				3
#define MULTI_BUTTON_REPORT_MIN			0
#define MULTI_BUTTON_REPORT_MAX			3
#define TX_RX_BUTTON_MIN			0
#define TX_RX_BUTTON_MAX			255
#define THREADHOLD_BUTTON_MIN			0
#define THREADHOLD_BUTTON_MAX			255
#define RELEASE_THREADHOLD_BUTTON_MIN		0
#define RELEASE_THREADHOLD_BUTTON_MAX		255
#define STRONGEST_BUTTON_HYSTERESIS_MIN		0
#define STRONGEST_BUTTON_HYSTERESIS_MAX		255
#define FILTER_STRENGTH_MIN			0
#define FILTER_STRENGTH_MAX			255

union f1a_0d_query {
	struct {
		u8 max_button_count:3;
		u8 reserved:5;
		u8 has_general_control:1;
		u8 has_interrupt_enable:1;
		u8 has_multibutton_select:1;
		u8 has_tx_rx_map:1;
		u8 has_perbutton_threshold:1;
		u8 has_release_threshold:1;
		u8 has_strongestbtn_hysteresis:1;
		u8 has_filter_strength:1;
	} __attribute__((__packed__));
	struct {
		u8 regs[2];
		u16 address;
	} __attribute__((__packed__));
};

union f1a_0d_control_0 {
	struct {
		u8 multibutton_report:2;
		u8 filter_mode:2;
	} __attribute__((__packed__));
	struct {
		u8 regs[1];
		u16 address;
	} __attribute__((__packed__));
};

struct f1a_0d_control_1n {
	u8 interrupt_enabled_button;
};

struct f1a_0d_control_1 {
	struct f1a_0d_control_1n *regs;
	u16 address;
	u8 length;
} __attribute__((__packed__));

struct f1a_0d_control_2n {
	u8 multi_button;
};

struct f1a_0d_control_2 {
	struct f1a_0d_control_2n *regs;
	u16 address;
	u8 length;
} __attribute__((__packed__));

struct f1a_0d_control_3_4n {
	u8 transmitterbtn;
	u8 receiverbtn;
} __attribute__((__packed__));

struct f1a_0d_control_3_4 {
	struct f1a_0d_control_3_4n *regs;
	u16 address;
	u8 length;
} __attribute__((__packed__));

struct f1a_0d_control_5n {
	u8 threshold_button;
};

struct f1a_0d_control_5 {
	struct f1a_0d_control_5n *regs;
	u16 address;
	u8 length;
} __attribute__((__packed__));

union f1a_0d_control_6 {
	struct {
		u8 button_release_threshold;
	} __attribute__((__packed__));
	struct {
		u8 regs[1];
		u16 address;
	} __attribute__((__packed__));
};

union f1a_0d_control_7 {
	struct {
		u8 strongest_button_hysteresis;
	} __attribute__((__packed__));
	struct {
		u8 regs[1];
		u16 address;
	} __attribute__((__packed__));
};

union f1a_0d_control_8 {
	struct {
		u8 filter_strength;
	} __attribute__((__packed__));
	struct {
		u8 regs[1];
		u16 address;
	} __attribute__((__packed__));
};

struct f1a_0d_control {
	union f1a_0d_control_0 *reg_0;
	struct f1a_0d_control_1 *reg_1;
	struct f1a_0d_control_2 *reg_2;
	struct f1a_0d_control_3_4 *reg_3_4;
	struct f1a_0d_control_5 *reg_5;
	union f1a_0d_control_6 *reg_6;
	union f1a_0d_control_7 *reg_7;
	union f1a_0d_control_8 *reg_8;
};

/* data specific to fn $1a that needs to be kept around */
struct f1a_data {
	struct f1a_0d_control control;
	union f1a_0d_query query;
	u8 sensor_button_count;
	int button_bitmask_size;
	u8 *button_data_buffer;
	u8 button_map_size;
	u8 *button_map;
	char input_phys[MAX_NAME_LEN];
	struct input_dev *input;
};

static int rmi_f1a_read_control_parameters(struct rmi_device *rmi_dev,
	struct f1a_data *f1a)
{
	union f1a_0d_query *query = &f1a->query;
	struct f1a_0d_control *control = &f1a->control;
	int retval = 0;

	if (query->has_general_control) {
		retval = rmi_read_block(rmi_dev, control->reg_0->address,
				(u8 *)control->reg_0->regs,
				sizeof(control->reg_0->regs));
		if (retval < 0) {
			dev_err(&rmi_dev->dev, "Could not read control reg0 to %#06x.\n",
					control->reg_0->address);
			return retval;
		}
	}

	if (query->has_interrupt_enable) {
		retval = rmi_read_block(rmi_dev, control->reg_1->address,
			(u8 *)control->reg_1->regs, f1a->button_bitmask_size);
		if (retval < 0) {
			dev_err(&rmi_dev->dev, "Could not read control reg1 to %#06x.\n",
				 control->reg_1->address);
			return retval;
		}
	}

	if (query->has_multibutton_select) {
		retval = rmi_read_block(rmi_dev, control->reg_2->address,
			(u8 *)control->reg_2->regs, f1a->button_bitmask_size);
		if (retval < 0) {
			dev_err(&rmi_dev->dev, "Could not read control reg2 to %#06x.\n",
				 control->reg_2->address);
			return retval;
		}
	}

	if (query->has_tx_rx_map) {
		retval = rmi_read_block(rmi_dev, control->reg_3_4->address,
			(u8 *)control->reg_3_4->regs,
			sizeof(struct f1a_0d_control_3_4n) *
					f1a->sensor_button_count);
		if (retval < 0) {
			dev_err(&rmi_dev->dev, "Could not read control reg 3_4 to %#06x.\n",
				 control->reg_3_4->address);
			return retval;
		}
	}

	if (query->has_perbutton_threshold) {
		retval = rmi_read_block(rmi_dev, control->reg_5->address,
			(u8 *)control->reg_5->regs, f1a->sensor_button_count);
		if (retval < 0) {
			dev_err(&rmi_dev->dev, "Could not read control reg 5 to %#06x.\n",
				 control->reg_5->address);
			return retval;
		}
	}

	if (query->has_release_threshold) {
		retval = rmi_read_block(rmi_dev, control->reg_6->address,
				(u8 *)control->reg_6->regs,
				sizeof(control->reg_6->regs));
		if (retval < 0) {
			dev_err(&rmi_dev->dev, "Could not read control reg 6 to %#06x.\n",
					control->reg_6->address);
			return retval;
		}
	}

	if (query->has_strongestbtn_hysteresis) {
		retval = rmi_read_block(rmi_dev, control->reg_7->address,
				(u8 *)control->reg_7->regs,
				sizeof(control->reg_7->regs));
		if (retval < 0) {
			dev_err(&rmi_dev->dev, "Could not read control reg 7 to %#06x.\n",
					control->reg_7->address);
			return retval;
		}
	}

	if (query->has_filter_strength) {
		retval = rmi_read_block(rmi_dev, control->reg_8->address,
				(u8 *)control->reg_8->regs,
				sizeof(control->reg_8->regs));
		if (retval < 0) {
			dev_err(&rmi_dev->dev, "Could not read control reg 8 to %#06x.\n",
					control->reg_8->address);
			return retval;
		}
	}

	return 0;
}

static int rmi_f1a_alloc_memory(struct rmi_function *fn)
{
	struct f1a_data *f1a;
	int rc, regSize;
	u16 ctrl_base_addr;

	f1a = devm_kzalloc(&fn->dev, sizeof(struct f1a_data), GFP_KERNEL);
	if (!f1a) {
		dev_err(&fn->dev, "Failed to allocate function data.\n");
		return -ENOMEM;
	}
	dev_set_drvdata(&fn->dev, f1a);

	rc = rmi_read_block(fn->rmi_dev, fn->fd.query_base_addr,
			f1a->query.regs, sizeof(f1a->query.regs));
	if (rc < 0) {
		dev_err(&fn->dev, "Failed to read query register.\n");
		return rc;
	}
	f1a->sensor_button_count = f1a->query.max_button_count + 1;
	f1a->button_bitmask_size =
			sizeof(u8)*(f1a->sensor_button_count + 7) / 8;

	f1a->button_data_buffer = devm_kzalloc(&fn->dev,
			f1a->button_bitmask_size, GFP_KERNEL);
	if (!f1a->button_data_buffer) {
		dev_err(&fn->dev, "Failed to allocate button data buffer.\n");
		return -ENOMEM;
	}

	f1a->button_map = devm_kzalloc(&fn->dev,
				f1a->sensor_button_count, GFP_KERNEL);
	if (!f1a->button_map) {
		dev_err(&fn->dev, "Failed to allocate button map.\n");
		return -ENOMEM;
	}

	/* allocate memory for control reg */
	/* reg 0 */
	ctrl_base_addr = fn->fd.control_base_addr;
	if (f1a->query.has_general_control) {
		f1a->control.reg_0 = devm_kzalloc(&fn->dev,
				sizeof(f1a->control.reg_0->regs), GFP_KERNEL);
		if (!f1a->control.reg_0) {
			dev_err(&fn->dev, "Failed to allocate reg_0 control registers.");
			return -ENOMEM;
		}
		f1a->control.reg_0->address = ctrl_base_addr;
		ctrl_base_addr += sizeof(f1a->control.reg_0->regs);
	}

	/* reg 1 */
	if (f1a->query.has_interrupt_enable) {
		f1a->control.reg_1 = devm_kzalloc(&fn->dev,
				sizeof(struct f1a_0d_control_1), GFP_KERNEL);
		if (!f1a->control.reg_1) {
			dev_err(&fn->dev, "Failed to allocate reg_1 control registers.");
			return -ENOMEM;
		}
		f1a->control.reg_1->regs = devm_kzalloc(&fn->dev,
				f1a->button_bitmask_size, GFP_KERNEL);
		if (!f1a->control.reg_1->regs) {
			dev_err(&fn->dev, "Failed to allocate reg_1->regs control registers.");
			return -ENOMEM;
		}
		f1a->control.reg_1->address = ctrl_base_addr;
		f1a->control.reg_1->length = f1a->button_bitmask_size;
		ctrl_base_addr += f1a->button_bitmask_size;
	}

	/* reg 2 */
	if (f1a->query.has_multibutton_select) {
		f1a->control.reg_2 = devm_kzalloc(&fn->dev,
				sizeof(struct f1a_0d_control_2), GFP_KERNEL);
		if (!f1a->control.reg_2) {
			dev_err(&fn->dev, "Failed to allocate reg_2 control registers.");
			return -ENOMEM;
		}
		f1a->control.reg_2->regs = devm_kzalloc(&fn->dev,
				f1a->button_bitmask_size, GFP_KERNEL);
		if (!f1a->control.reg_2->regs) {
			dev_err(&fn->dev, "Failed to allocate reg_2->regs control registers.");
			return -ENOMEM;
		}
		f1a->control.reg_2->address = ctrl_base_addr;
		f1a->control.reg_2->length = f1a->button_bitmask_size;
		ctrl_base_addr += f1a->button_bitmask_size;
	}

	/* reg 3_4 */
	if (f1a->query.has_tx_rx_map) {
		f1a->control.reg_3_4 = devm_kzalloc(&fn->dev,
			sizeof(struct f1a_0d_control_3_4), GFP_KERNEL);
		if (!f1a->control.reg_3_4) {
			dev_err(&fn->dev, "Failed to allocate reg_3_4 control registers.");
			return -ENOMEM;
		}
		regSize = sizeof(struct f1a_0d_control_3_4n);
		f1a->control.reg_3_4->regs = devm_kzalloc(&fn->dev,
				regSize*f1a->sensor_button_count, GFP_KERNEL);
		if (!f1a->control.reg_3_4->regs) {
			dev_err(&fn->dev, "Failed to allocate reg_3_4->regs control registers.");
			return -ENOMEM;
		}
		f1a->control.reg_3_4->address = ctrl_base_addr;
		f1a->control.reg_3_4->length =
					regSize * f1a->sensor_button_count;
		ctrl_base_addr += regSize * f1a->sensor_button_count;
	}

	/* reg 5 */
	if (f1a->query.has_perbutton_threshold) {
		f1a->control.reg_5 = devm_kzalloc(&fn->dev,
				sizeof(struct f1a_0d_control_5), GFP_KERNEL);
		if (!f1a->control.reg_5) {
			dev_err(&fn->dev, "Failed to allocate reg_5 control registers.");
			return -ENOMEM;
		}
		f1a->control.reg_5->regs = devm_kzalloc(&fn->dev,
				f1a->sensor_button_count, GFP_KERNEL);
		if (!f1a->control.reg_5->regs) {
			dev_err(&fn->dev, "Failed to allocate reg_5->regs control registers.");
			return -ENOMEM;
		}
		f1a->control.reg_5->address = ctrl_base_addr;
		f1a->control.reg_5->length = f1a->sensor_button_count;
		ctrl_base_addr += f1a->sensor_button_count;
	}

	/* reg 6 */
	if (f1a->query.has_release_threshold) {
		f1a->control.reg_6 = devm_kzalloc(&fn->dev,
			sizeof(f1a->control.reg_6->regs), GFP_KERNEL);
		if (!f1a->control.reg_6) {
			dev_err(&fn->dev, "Failed to allocate reg_6 control registers.");
			return -ENOMEM;
		}
		f1a->control.reg_6->address = ctrl_base_addr;
		ctrl_base_addr += sizeof(f1a->control.reg_6->regs);
	}

	/* reg 7 */
	if (f1a->query.has_strongestbtn_hysteresis) {
		f1a->control.reg_7 = devm_kzalloc(&fn->dev,
			sizeof(f1a->control.reg_7->regs), GFP_KERNEL);
		if (!f1a->control.reg_7) {
			dev_err(&fn->dev, "Failed to allocate reg_7 control registers.");
			return -ENOMEM;
		}
		f1a->control.reg_7->address = ctrl_base_addr;
		ctrl_base_addr += sizeof(f1a->control.reg_7->regs);
	}

	/* reg 8 */
	if (f1a->query.has_filter_strength) {
		f1a->control.reg_8 = devm_kzalloc(&fn->dev,
				sizeof(f1a->control.reg_8->regs), GFP_KERNEL);
		if (!f1a->control.reg_8) {
			dev_err(&fn->dev, "Failed to allocate reg_8 control registers.");
			return -ENOMEM;
		}
		f1a->control.reg_8->address = ctrl_base_addr;
		ctrl_base_addr += sizeof(f1a->control.reg_8->regs);
	}

	return 0;
}

static int rmi_f1a_initialize(struct rmi_function *fn)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f1a_data *f1a = dev_get_drvdata(&fn->dev);
	struct fwnode_handle *node;
	int retval, key_code;

	dev_dbg(&fn->dev, "Intializing F1A values.\n");

	f1a->button_map_size = 0;

	device_for_each_child_node(&fn->dev, node) {
		fwnode_property_read_u32(node, "linux,code", &key_code);
		f1a->button_map[f1a->button_map_size] = key_code;
		f1a->button_map_size++;
	}

	retval = rmi_f1a_read_control_parameters(rmi_dev, f1a);
	if (retval < 0) {
		dev_err(&fn->dev,
			"Failed to initialize F1a control params.\n");
		return retval;
	}

	return 0;
}

static int rmi_f1a_register_device(struct rmi_function *fn)
{
	struct input_dev *input_dev;
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f1a_data *f1a = dev_get_drvdata(&fn->dev);
	struct rmi_driver *driver = fn->rmi_dev->driver;
	int i, rc;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&fn->dev, "Failed to allocate input device.\n");
		return -ENOMEM;
	}

	f1a->input = input_dev;
	if (driver->set_input_params) {
		rc = driver->set_input_params(rmi_dev, input_dev);
		if (rc < 0) {
			dev_err(&fn->dev, "%s: Error in setting input device.\n",
			__func__);
			goto error_free_device;
		}
	}

	sprintf(f1a->input_phys, "%s/input0", dev_name(&fn->dev));
	input_dev->phys = f1a->input_phys;
	input_dev->dev.parent = &rmi_dev->dev;
	input_set_drvdata(input_dev, f1a);

	/* Set up any input events. */
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	/* manage button map using input subsystem */
	input_dev->keycode = f1a->button_map;
	input_dev->keycodesize = sizeof(f1a->button_map);
	input_dev->keycodemax = f1a->button_map_size;

	/* set bits for each button. */
	for (i = 0; i < f1a->button_map_size; i++) {
		set_bit(f1a->button_map[i], input_dev->keybit);
		input_set_capability(input_dev, EV_KEY, f1a->button_map[i]);
	}

	rc = input_register_device(input_dev);
	if (rc < 0) {
		dev_err(&fn->dev, "Failed to register input device.\n");
		goto error_free_device;
	}

	return 0;

error_free_device:
	input_free_device(input_dev);

	return rc;
}

static int rmi_f1a_config(struct rmi_function *fn)
{
	struct f1a_data *f1a = dev_get_drvdata(&fn->dev);
	union f1a_0d_query *query = &f1a->query;
	struct f1a_0d_control *control = &f1a->control;
	int retval;

	if (query->has_general_control) {
		retval = rmi_write_block(fn->rmi_dev,
					control->reg_0->address,
					control->reg_0->regs,
					sizeof(control->reg_0->regs));
		if (retval < 0) {
			dev_err(&fn->dev, "%s : Could not write reg 0 to 0x%x\n",
					__func__, control->reg_0->address);
			return retval;
		}
	}

	if (query->has_interrupt_enable) {
		retval = rmi_write_block(fn->rmi_dev,
					control->reg_1->address,
					control->reg_1->regs,
					f1a->button_bitmask_size);
		if (retval < 0) {
			dev_err(&fn->dev, "%s : Could not write reg 1 to 0x%x\n",
				__func__, control->reg_1->address);
			return retval;
		}
	}

	if (query->has_multibutton_select) {
		retval = rmi_write_block(fn->rmi_dev,
				control->reg_2->address, control->reg_2->regs,
				f1a->button_bitmask_size);
		if (retval < 0) {
			dev_err(&fn->dev, "%s : Could not write reg 2 to 0x%x\n",
				__func__, control->reg_2->address);
			return -EINVAL;
		}
	}

	if (query->has_tx_rx_map) {
		retval = rmi_write_block(fn->rmi_dev,
					control->reg_3_4->address,
					control->reg_3_4->regs,
					f1a->sensor_button_count);
		if (retval < 0) {
			dev_err(&fn->dev, "%s : Could not write reg_3_4 to 0x%x\n",
					__func__, control->reg_3_4->address);
			return -EINVAL;
		}
	}

	if (query->has_perbutton_threshold) {
		retval = rmi_write_block(fn->rmi_dev,
				control->reg_5->address, control->reg_5->regs,
				f1a->sensor_button_count);
		if (retval < 0) {
			dev_err(&fn->dev, "%s : Could not write reg 5 to 0x%x\n",
				__func__, control->reg_5->address);
			return retval;
		}
	}

	if (query->has_release_threshold) {
		retval = rmi_write_block(fn->rmi_dev,
				control->reg_6->address, control->reg_6->regs,
				sizeof(control->reg_6->regs));
		if (retval < 0) {
			dev_err(&fn->dev, "%s : Could not write reg 6 to 0x%x\n",
				__func__, control->reg_6->address);
			return -EINVAL;
		}
	}

	if (query->has_strongestbtn_hysteresis) {
		retval = rmi_write_block(fn->rmi_dev,
				control->reg_7->address, control->reg_7->regs,
				sizeof(control->reg_7->regs));
		if (retval < 0) {
			dev_err(&fn->dev, "%s : Could not write reg 7 to 0x%x\n",
				__func__, control->reg_7->address);
			return -EINVAL;
		}
	}

	if (query->has_filter_strength) {
		retval = rmi_write_block(fn->rmi_dev,
				control->reg_8->address, control->reg_8->regs,
				sizeof(control->reg_8->regs));
		if (retval < 0) {
			dev_err(&fn->dev, "%s : Could not write reg 8 to 0x%x\n",
				__func__, control->reg_8->address);
			return -EINVAL;
		}
	}

	return 0;
}

static irqreturn_t rmi_f1a_attention(int irq, void *ctx)
{
	struct rmi_function *fn = ctx;
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f1a_data *f1a = dev_get_drvdata(&fn->dev);
	u16 data_base_addr = fn->fd.data_base_addr;
	int error, button;

	/* Read the button data. */
	error = rmi_read_block(rmi_dev, data_base_addr, f1a->button_data_buffer,
			f1a->button_bitmask_size);
	if (error < 0) {
		dev_err(&fn->dev, "%s: Failed to read button data registers.\n",
			__func__);
		return IRQ_RETVAL(error);
	}

	/* Generate events for buttons that change state. */
	for (button = 0; button < f1a->sensor_button_count; button++) {
		int button_reg;
		int button_shift;
		bool button_status;

		/* determine which data byte the button status is in */
		button_reg = button / 8;

		/* bit shift to get button's status */
		button_shift = button % 8;
		button_status =
		    ((f1a->button_data_buffer[button_reg] >> button_shift)
			& 0x01) != 0;
		dev_err(&fn->dev, "%s: Button %d (code %d) -> %d.\n",
			__func__, button, f1a->button_map[button],
				button_status);

		/* Generate an event here. */
		input_report_key(f1a->input, f1a->button_map[button],
					button_status);
	}

	input_sync(f1a->input); /* sync after groups of events */

	return IRQ_HANDLED;
}

static int rmi_f1a_probe(struct rmi_function *fn)
{
	int rc;

	rc = rmi_f1a_alloc_memory(fn);
	if (rc < 0)
		return rc;

	rc = rmi_f1a_initialize(fn);
	if (rc < 0)
		return rc;

	rc = rmi_f1a_register_device(fn);
	if (rc < 0)
		return rc;

	return 0;
}

static void rmi_f1a_remove(struct rmi_function *fn)
{
	struct f1a_data *f1a = dev_get_drvdata(&fn->dev);
	input_unregister_device(f1a->input);
}

struct rmi_function_handler rmi_f1a_handler = {
	.driver = {
		.name	= "rmi4_f1a",
	},
	.func		= 0x1a,
	.probe		= rmi_f1a_probe,
	.remove		= rmi_f1a_remove,
	.config		= rmi_f1a_config,
	.attention	= rmi_f1a_attention,
};
