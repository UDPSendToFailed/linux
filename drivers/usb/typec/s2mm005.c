// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S2MM005 USB Type-C CC/PD controller driver.
 *
 * The S2MM005 is a companion chip found on Samsung SDM450-based devices
 * (e.g. Galaxy Tab A 10.5). It sits on I2C and manages the USB Type-C
 * CC lines and USB PD negotiation autonomously. The host only needs to
 * read its state register to know the current attach/detach and role
 * status.
 *
 * This minimal mainline driver:
 *  - Reads HW/SW version at probe for chip identification
 *  - Handles the IRQ (falling edge on the interrupt GPIO)
 *  - Reads FUNC_STATE (register 0x0020) to detect attach/detach,
 *    source/sink role, and DFP/UFP data role
 *  - Reports state changes via the kernel typec subsystem
 *
 * I2C protocol: 16-bit big-endian register addresses, multi-byte data.
 *
 * Copyright (c) 2025, The Linux Foundation. All rights reserved.
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/usb/typec.h>

/* 16-bit register addresses (big-endian on the wire) */
#define S2MM005_REG_HW_VER		0x0000
#define S2MM005_REG_SW_VER		0x0008
#define S2MM005_REG_I2C_SLV_CMD		0x0010
#define S2MM005_REG_FUNC_STATE		0x0020
#define S2MM005_REG_INT_STATUS		0x0030

/* FUNC_STATE bit definitions (32-bit LE value) */
#define FUNC_PD_STATE_MASK		GENMASK(7, 0)
#define FUNC_CC1_MASK			GENMASK(10, 8)
#define FUNC_CC2_MASK			GENMASK(14, 12)
#define FUNC_ATTACH_DONE		BIT(24)
#define FUNC_IS_SOURCE			BIT(25)
#define FUNC_IS_DFP			BIT(26)
#define FUNC_RP_CURLVL_MASK		GENMASK(28, 27)

/* PD state == 0 means initial detach */
#define PD_STATE_INITIAL_DETACH		0

struct s2mm005 {
	struct i2c_client	*client;
	struct typec_port	*port;
	struct typec_partner	*partner;
	bool			attached;
};

/* I2C read: 16-bit register address, then read `len` bytes of data */
static int s2mm005_read(struct i2c_client *client, u16 reg, void *buf, int len)
{
	u8 addr[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[] = {
		{ .addr = client->addr, .flags = 0,        .len = 2,   .buf = addr },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = len, .buf = buf  },
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2)
		return ret < 0 ? ret : -EIO;

	return 0;
}

/* I2C write: 16-bit register address, then 1 byte of data */
static int s2mm005_write_byte(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 3,
		.buf = buf,
	};
	int ret;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1)
		return ret < 0 ? ret : -EIO;

	return 0;
}

static void s2mm005_update_state(struct s2mm005 *data)
{
	struct device *dev = &data->client->dev;
	u8 raw[4];
	u32 func;
	u8 pd_state;
	bool is_source, is_dfp, attached;
	int ret;

	ret = s2mm005_read(data->client, S2MM005_REG_FUNC_STATE, raw, 4);
	if (ret) {
		dev_err(dev, "failed to read FUNC_STATE: %d\n", ret);
		return;
	}

	func = le32_to_cpup((__le32 *)raw);
	pd_state = func & FUNC_PD_STATE_MASK;
	is_source = !!(func & FUNC_IS_SOURCE);
	is_dfp = !!(func & FUNC_IS_DFP);
	attached = !!(func & FUNC_ATTACH_DONE) &&
		   pd_state != PD_STATE_INITIAL_DETACH;

	dev_info(dev, "FUNC_STATE=0x%08x pd=%d src=%d dfp=%d attached=%d\n",
		 func, pd_state, is_source, is_dfp, attached);

	if (attached && !data->attached) {
		struct typec_partner_desc desc = {};

		desc.accessory = TYPEC_ACCESSORY_NONE;

		typec_set_pwr_role(data->port,
				   is_source ? TYPEC_SOURCE : TYPEC_SINK);
		typec_set_data_role(data->port,
				    is_dfp ? TYPEC_HOST : TYPEC_DEVICE);
		typec_set_pwr_opmode(data->port, TYPEC_PWR_MODE_USB);

		data->partner = typec_register_partner(data->port, &desc);
		if (IS_ERR(data->partner)) {
			dev_err(dev, "failed to register partner: %ld\n",
				PTR_ERR(data->partner));
			data->partner = NULL;
		}

		data->attached = true;
		dev_info(dev, "attached: %s/%s pd_state=%d\n",
			 is_source ? "SRC" : "SNK",
			 is_dfp ? "DFP" : "UFP", pd_state);
	} else if (!attached && data->attached) {
		if (data->partner) {
			typec_unregister_partner(data->partner);
			data->partner = NULL;
		}

		typec_set_pwr_role(data->port, TYPEC_SINK);
		typec_set_data_role(data->port, TYPEC_DEVICE);
		typec_set_pwr_opmode(data->port, TYPEC_PWR_MODE_USB);

		data->attached = false;
		dev_info(dev, "detached\n");
	} else if (attached) {
		/* Update roles if they changed (DR_Swap / PR_Swap) */
		typec_set_pwr_role(data->port,
				   is_source ? TYPEC_SOURCE : TYPEC_SINK);
		typec_set_data_role(data->port,
				    is_dfp ? TYPEC_HOST : TYPEC_DEVICE);
	}
}

static irqreturn_t s2mm005_irq_handler(int irq, void *devid)
{
	struct s2mm005 *data = devid;
	u8 int_status[48];

	/* Read interrupt status to unblock CCIC firmware */
	if (!s2mm005_read(data->client, S2MM005_REG_INT_STATUS, int_status, 48))
		dev_dbg(&data->client->dev, "IRQ0:0x%02x IRQ1:0x%02x\n",
			int_status[0], int_status[1]);

	s2mm005_update_state(data);

	/* Clear interrupt */
	s2mm005_write_byte(data->client, S2MM005_REG_I2C_SLV_CMD, 0x01);

	return IRQ_HANDLED;
}

static int s2mm005_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct typec_capability cap = {};
	struct s2mm005 *data;
	u8 hw_ver[8], sw_ver[8], int_status[48];
	int ret;

	dev_info(dev, "s2mm005_probe: enter (addr=0x%02x, irq=%d)\n",
		 client->addr, client->irq);

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);

	/* Read chip identification */
	ret = s2mm005_read(client, S2MM005_REG_HW_VER, hw_ver, 8);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read HW version\n");

	ret = s2mm005_read(client, S2MM005_REG_SW_VER, sw_ver, 8);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read SW version\n");

	dev_info(dev, "S2MM005 HW:%02x.%02x.%02x.%02x SW:%02x.%02x.%02x.%02x\n",
		 hw_ver[3], hw_ver[2], hw_ver[1], hw_ver[0],
		 sw_ver[3], sw_ver[2], sw_ver[1], sw_ver[0]);

	/* Register typec port */
	cap.revision = USB_TYPEC_REV_1_2;
	cap.pd_revision = 0x0300;
	cap.prefer_role = TYPEC_SINK;
	cap.type = TYPEC_PORT_DRP;
	cap.data = TYPEC_PORT_DRD;

	data->port = typec_register_port(dev, &cap);
	if (IS_ERR(data->port))
		return dev_err_probe(dev, PTR_ERR(data->port),
				     "failed to register typec port\n");

	/* Read initial state and clear pending interrupts before enabling IRQ */
	if (!s2mm005_read(client, S2MM005_REG_INT_STATUS, int_status, 48))
		dev_info(dev, "initial IRQ0:0x%02x IRQ1:0x%02x\n",
			 int_status[0], int_status[1]);
	s2mm005_update_state(data);
	s2mm005_write_byte(client, S2MM005_REG_I2C_SLV_CMD, 0x01);

	/* Downstream runs an explicit initial-detect path after probe. */
	usleep_range(1000, 2000);
	s2mm005_update_state(data);

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					s2mm005_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"s2mm005", data);
	if (ret) {
		typec_unregister_port(data->port);
		return dev_err_probe(dev, ret, "failed to request IRQ\n");
	}

	return 0;
}

static void s2mm005_remove(struct i2c_client *client)
{
	struct s2mm005 *data = i2c_get_clientdata(client);

	if (data->partner)
		typec_unregister_partner(data->partner);
	typec_unregister_port(data->port);
}

static const struct of_device_id s2mm005_of_match[] = {
	{ .compatible = "samsung,s2mm005" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mm005_of_match);

static const struct i2c_device_id s2mm005_id[] = {
	{ "s2mm005" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s2mm005_id);

static struct i2c_driver s2mm005_driver = {
	.driver = {
		.name		= "s2mm005",
		.of_match_table	= s2mm005_of_match,
	},
	.probe		= s2mm005_probe,
	.remove		= s2mm005_remove,
	.id_table	= s2mm005_id,
};
module_i2c_driver(s2mm005_driver);

MODULE_DESCRIPTION("Samsung S2MM005 USB Type-C CC/PD controller");
MODULE_LICENSE("GPL");
