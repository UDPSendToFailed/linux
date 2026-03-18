// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics FTS1BA90A Touchscreen Driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 * Copyright (c) 2026 The LineageOS Project
 *
 * Ported from downstream Samsung kernel driver to modern kernel APIs.
 * Original downstream driver: drivers/input/touchscreen/stm/fts1ba90a/
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

/* Chip identification */
#define FTS1BA90A_ID0				0x39
#define FTS1BA90A_ID1				0x36

/* FIFO / event constants */
#define FTS1BA90A_FIFO_MAX			32
#define FTS1BA90A_EVENT_SIZE			8
#define FTS1BA90A_MAX_FINGERS			10

/* I2C commands */
#define FTS1BA90A_CMD_SENSE_ON			0x10
#define FTS1BA90A_CMD_SENSE_OFF			0x11
#define FTS1BA90A_CMD_SW_RESET			0x12
#define FTS1BA90A_CMD_FORCE_CALIBRATION		0x13

#define FTS1BA90A_READ_DEVICE_ID		0x22
#define FTS1BA90A_READ_FW_VERSION		0x24

#define FTS1BA90A_CMD_SET_TOUCHTYPE		0x30
#define FTS1BA90A_CMD_SET_SCANMODE		0xA0

#define FTS1BA90A_READ_ONE_EVENT		0x60
#define FTS1BA90A_READ_ALL_EVENT		0x61
#define FTS1BA90A_CMD_CLEAR_ALL_EVENT		0x62

#define FTS1BA90A_CMD_IC_INTERRUPT		0xA4

/* Status event byte 0: report ID */
#define FTS1BA90A_EVENT_STATUS_REPORT		0x43
#define FTS1BA90A_EVENT_ERROR_REPORT		0xF3

/* Event ID (lower 2 bits of byte 0) */
#define FTS1BA90A_COORDINATE_EVENT		0
#define FTS1BA90A_STATUS_EVENT			1

/* Status event sub-types */
#define FTS1BA90A_STATUSTYPE_CMDDRIVEN		0
#define FTS1BA90A_STATUSTYPE_ERROR		1
#define FTS1BA90A_STATUSTYPE_INFORMATION	2

/* Status IDs */
#define FTS1BA90A_INFO_READY_STATUS		0x00

/* Error event IDs */
#define FTS1BA90A_ERR_EVENT_QUEUE_FULL		0x01
#define FTS1BA90A_ERR_EVENT_ESD			0x02

/* Coordinate actions */
#define FTS1BA90A_ACTION_NONE			0
#define FTS1BA90A_ACTION_PRESS			1
#define FTS1BA90A_ACTION_MOVE			2
#define FTS1BA90A_ACTION_RELEASE		3

/* Touch types */
#define FTS1BA90A_TOUCHTYPE_NORMAL		0
#define FTS1BA90A_TOUCHTYPE_PALM		5
#define FTS1BA90A_TOUCHTYPE_WET			6
#define FTS1BA90A_TOUCHTYPE_GLOVE		3

/* Touch type bitmask for CMD 0x30 */
#define FTS1BA90A_TOUCHTYPE_BIT_TOUCH		BIT(0)
#define FTS1BA90A_TOUCHTYPE_BIT_PALM		BIT(5)
#define FTS1BA90A_TOUCHTYPE_BIT_WET		BIT(6)
#define FTS1BA90A_TOUCHTYPE_DEFAULT \
	(FTS1BA90A_TOUCHTYPE_BIT_TOUCH | FTS1BA90A_TOUCHTYPE_BIT_PALM | \
	 FTS1BA90A_TOUCHTYPE_BIT_WET)

/* Scan modes for CMD 0xA0 */
#define FTS1BA90A_SCAN_MODE_MS_SS		BIT(0)
#define FTS1BA90A_SCAN_MODE_DEFAULT		FTS1BA90A_SCAN_MODE_MS_SS

/* System reset magic sequence */
#define FTS1BA90A_SYSRESET_LEN			6

/* I2C retry count */
#define FTS1BA90A_I2C_RETRY			3

/* Ready-wait retries and polling interval (ms) */
#define FTS1BA90A_READY_RETRIES			10
#define FTS1BA90A_READY_POLL_MS			20

/* Echo-wait retries and polling interval (ms) */
#define FTS1BA90A_ECHO_RETRIES			300
#define FTS1BA90A_ECHO_POLL_MS			20

/* Post-reset delay (ms) */
#define FTS1BA90A_RESET_DELAY_MS		10

enum fts1ba90a_regulators {
	FTS1BA90A_REGULATOR_VDD,
	FTS1BA90A_REGULATOR_AVDD,
};

/*
 * 8-byte coordinate event, packed as sent by the controller.
 */
struct fts1ba90a_event_coord {
	u8 eid:2;
	u8 tid:4;
	u8 action:2;
	u8 x_11_4;
	u8 y_11_4;
	u8 y_3_0:4;
	u8 x_3_0:4;
	u8 major;
	u8 minor;
	u8 z:6;
	u8 ttype_3_2:2;
	u8 left_event:6;
	u8 ttype_1_0:2;
} __packed;

/*
 * 8-byte status event, packed as sent by the controller.
 */
struct fts1ba90a_event_status {
	u8 eid:2;
	u8 stype:4;
	u8 sf:2;
	u8 status_id;
	u8 status_data[5];
	u8 left_event:6;
	u8 reserved:2;
} __packed;

struct fts1ba90a_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct regulator_bulk_data regulators[2];
	struct touchscreen_properties prop;

	struct mutex mutex;

	u8 event_buf[FTS1BA90A_FIFO_MAX * FTS1BA90A_EVENT_SIZE];

	u16 fw_version;
	u16 config_version;
	u16 fw_main_version;
	u8 finger_action[FTS1BA90A_MAX_FINGERS];
	int touch_count;
};

static int fts1ba90a_i2c_write(struct fts1ba90a_data *ts, u8 *data, u16 len)
{
	struct i2c_msg msg = {
		.addr = ts->client->addr,
		.len = len,
		.buf = data,
	};
	int ret, retry = FTS1BA90A_I2C_RETRY;

	do {
		ret = i2c_transfer(ts->client->adapter, &msg, 1);
		if (ret == 1)
			return 0;
		usleep_range(10000, 11000);
	} while (--retry > 0);

	dev_err(&ts->client->dev, "i2c write failed after %d retries: %d\n",
		FTS1BA90A_I2C_RETRY, ret);
	return ret < 0 ? ret : -EIO;
}

static int fts1ba90a_i2c_read(struct fts1ba90a_data *ts, u8 *reg, int reg_len,
			      u8 *buf, int buf_len)
{
	struct i2c_msg msgs[2] = {
		{
			.addr = ts->client->addr,
			.len = reg_len,
			.buf = reg,
		},
		{
			.addr = ts->client->addr,
			.flags = I2C_M_RD,
			.len = buf_len,
			.buf = buf,
		},
	};
	int ret, retry = FTS1BA90A_I2C_RETRY;

	do {
		ret = i2c_transfer(ts->client->adapter, msgs, 2);
		if (ret == 2)
			return 0;
		usleep_range(10000, 11000);
	} while (--retry > 0);

	dev_err(&ts->client->dev, "i2c read failed after %d retries: %d\n",
		FTS1BA90A_I2C_RETRY, ret);
	return ret < 0 ? ret : -EIO;
}

static int fts1ba90a_command(struct fts1ba90a_data *ts, u8 cmd)
{
	return fts1ba90a_i2c_write(ts, &cmd, 1);
}

static void fts1ba90a_systemreset(struct fts1ba90a_data *ts)
{
	u8 cmd[FTS1BA90A_SYSRESET_LEN] = { 0xFA, 0x20, 0x00, 0x00, 0x24, 0x81 };

	fts1ba90a_i2c_write(ts, cmd, sizeof(cmd));
	msleep(FTS1BA90A_RESET_DELAY_MS);
}

static int fts1ba90a_wait_for_ready(struct fts1ba90a_data *ts)
{
	u8 reg = FTS1BA90A_READ_ONE_EVENT;
	u8 data[FTS1BA90A_EVENT_SIZE];
	struct fts1ba90a_event_status *ev;
	int retry, err_cnt = 0;

	for (retry = 0; retry <= FTS1BA90A_READY_RETRIES; retry++) {
		if (fts1ba90a_i2c_read(ts, &reg, 1, data, sizeof(data)))
			break;

		ev = (struct fts1ba90a_event_status *)data;

		if (ev->stype == FTS1BA90A_STATUSTYPE_INFORMATION &&
		    ev->status_id == FTS1BA90A_INFO_READY_STATUS)
			return 0;

		if (data[0] == FTS1BA90A_EVENT_ERROR_REPORT) {
			if (++err_cnt > 32)
				return -EIO;
			continue;
		}

		msleep(FTS1BA90A_READY_POLL_MS);
	}

	dev_err(&ts->client->dev, "timeout waiting for ready event\n");
	return -ETIMEDOUT;
}

static int fts1ba90a_wait_for_echo(struct fts1ba90a_data *ts, u8 *cmd,
				   u8 cmd_len)
{
	u8 reg = FTS1BA90A_READ_ONE_EVENT;
	u8 data[FTS1BA90A_EVENT_SIZE];
	int retry, i;
	bool matched;

	for (retry = 0; retry < FTS1BA90A_ECHO_RETRIES; retry++) {
		if (fts1ba90a_i2c_read(ts, &reg, 1, data, sizeof(data)))
			break;

		/* Check for command echo: status_report(0x43), sub=0x01 */
		if (data[0] == FTS1BA90A_EVENT_STATUS_REPORT && data[1] == 0x01) {
			matched = true;
			for (i = 0; i < cmd_len; i++) {
				if (data[i + 2] != cmd[i]) {
					matched = false;
					break;
				}
			}
			if (matched)
				return 0;
		}

		if (data[0] == FTS1BA90A_EVENT_ERROR_REPORT)
			break;

		msleep(FTS1BA90A_ECHO_POLL_MS);
	}

	dev_err(&ts->client->dev, "timeout waiting for echo\n");
	return -ETIMEDOUT;
}

static int fts1ba90a_command_echo(struct fts1ba90a_data *ts, u8 cmd)
{
	int ret;

	ret = fts1ba90a_command(ts, cmd);
	if (ret)
		return ret;

	return fts1ba90a_wait_for_echo(ts, &cmd, 1);
}

static int fts1ba90a_read_chip_id(struct fts1ba90a_data *ts)
{
	u8 reg = FTS1BA90A_READ_DEVICE_ID;
	u8 val[5];
	int ret;

	ret = fts1ba90a_i2c_read(ts, &reg, 1, val, sizeof(val));
	if (ret)
		return ret;

	dev_info(&ts->client->dev, "chip id: %c %c %02X %02X %02X\n",
		 val[0], val[1], val[2], val[3], val[4]);

	if (val[2] != FTS1BA90A_ID0 || val[3] != FTS1BA90A_ID1) {
		dev_err(&ts->client->dev, "invalid chip id\n");
		return -ENODEV;
	}

	return 0;
}

static int fts1ba90a_get_version(struct fts1ba90a_data *ts)
{
	u8 reg = FTS1BA90A_READ_FW_VERSION;
	u8 data[FTS1BA90A_EVENT_SIZE];
	int ret;

	ret = fts1ba90a_i2c_read(ts, &reg, 1, data, sizeof(data));
	if (ret)
		return ret;

	ts->fw_version = (data[0] << 8) | data[1];
	ts->config_version = (data[2] << 8) | data[3];
	ts->fw_main_version = data[4] | (data[5] << 8);

	dev_info(&ts->client->dev,
		 "FW ver: 0x%04X, Config ver: 0x%04X, Main ver: 0x%04X\n",
		 ts->fw_version, ts->config_version, ts->fw_main_version);

	return 0;
}

static int fts1ba90a_set_scanmode(struct fts1ba90a_data *ts, u8 mode)
{
	u8 cmd[3] = { FTS1BA90A_CMD_SET_SCANMODE, 0x00, mode };
	int ret;

	ret = fts1ba90a_i2c_write(ts, cmd, sizeof(cmd));
	if (ret)
		return ret;

	return fts1ba90a_wait_for_echo(ts, cmd, sizeof(cmd));
}

static void fts1ba90a_release_all_fingers(struct fts1ba90a_data *ts)
{
	int i;

	for (i = 0; i < FTS1BA90A_MAX_FINGERS; i++) {
		input_mt_slot(ts->input, i);
		input_mt_report_slot_inactive(ts->input);
		ts->finger_action[i] = FTS1BA90A_ACTION_NONE;
	}

	ts->touch_count = 0;
	input_report_key(ts->input, BTN_TOUCH, 0);
	input_report_key(ts->input, BTN_TOOL_FINGER, 0);
	input_sync(ts->input);
}

static void fts1ba90a_handle_coordinate(struct fts1ba90a_data *ts,
					struct fts1ba90a_event_coord *ev)
{
	unsigned int tid = ev->tid;
	unsigned int x, y, major, minor, z;
	u8 action, ttype;

	if (tid >= FTS1BA90A_MAX_FINGERS)
		return;

	action = ev->action;
	ttype = (ev->ttype_3_2 << 2) | ev->ttype_1_0;

	/* Only handle normal, palm, wet, and glove touch types */
	if (ttype != FTS1BA90A_TOUCHTYPE_NORMAL &&
	    ttype != FTS1BA90A_TOUCHTYPE_PALM &&
	    ttype != FTS1BA90A_TOUCHTYPE_WET &&
	    ttype != FTS1BA90A_TOUCHTYPE_GLOVE)
		return;

	x = (ev->x_11_4 << 4) | ev->x_3_0;
	y = (ev->y_11_4 << 4) | ev->y_3_0;
	z = ev->z & 0x3F;
	major = ev->major;
	minor = ev->minor;

	if (z == 0)
		z = 1;

	switch (action) {
	case FTS1BA90A_ACTION_RELEASE:
		input_mt_slot(ts->input, tid);
		input_mt_report_slot_inactive(ts->input);

		if (ts->touch_count > 0)
			ts->touch_count--;
		if (ts->touch_count == 0) {
			input_report_key(ts->input, BTN_TOUCH, 0);
			input_report_key(ts->input, BTN_TOOL_FINGER, 0);
		}
		ts->finger_action[tid] = FTS1BA90A_ACTION_NONE;
		break;

	case FTS1BA90A_ACTION_PRESS:
		ts->touch_count++;
		ts->finger_action[tid] = FTS1BA90A_ACTION_PRESS;

		input_mt_slot(ts->input, tid);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		input_report_key(ts->input, BTN_TOUCH, 1);
		input_report_key(ts->input, BTN_TOOL_FINGER, 1);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, major);
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, minor);
		input_report_abs(ts->input, ABS_MT_PRESSURE, z);
		break;

	case FTS1BA90A_ACTION_MOVE:
		if (ts->touch_count == 0 ||
		    ts->finger_action[tid] == FTS1BA90A_ACTION_NONE)
			break;

		ts->finger_action[tid] = FTS1BA90A_ACTION_MOVE;

		input_mt_slot(ts->input, tid);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		input_report_key(ts->input, BTN_TOUCH, 1);
		input_report_key(ts->input, BTN_TOOL_FINGER, 1);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, major);
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, minor);
		input_report_abs(ts->input, ABS_MT_PRESSURE, z);
		break;
	}
}

static void fts1ba90a_handle_status(struct fts1ba90a_data *ts,
				    struct fts1ba90a_event_status *ev)
{
	if (ev->stype == FTS1BA90A_STATUSTYPE_ERROR &&
	    ev->status_id == FTS1BA90A_ERR_EVENT_QUEUE_FULL) {
		dev_warn(&ts->client->dev, "event queue full\n");
		fts1ba90a_release_all_fingers(ts);
	}

	if (ev->stype == FTS1BA90A_STATUSTYPE_ERROR &&
	    ev->status_id == FTS1BA90A_ERR_EVENT_ESD)
		dev_err(&ts->client->dev, "ESD detected\n");
}

static irqreturn_t fts1ba90a_irq_handler(int irq, void *data)
{
	struct fts1ba90a_data *ts = data;
	u8 reg;
	int left_event_count, i;
	u8 *evt;
	u8 event_id;

	mutex_lock(&ts->mutex);

	do {
		reg = FTS1BA90A_READ_ONE_EVENT;
		if (fts1ba90a_i2c_read(ts, &reg, 1, ts->event_buf,
				       FTS1BA90A_EVENT_SIZE))
			break;

		/* Controller returns 0xFF when FIFO is empty */
		if (ts->event_buf[0] == 0xFF)
			break;

		left_event_count = ts->event_buf[7] & 0x3F;
		if (left_event_count >= FTS1BA90A_FIFO_MAX)
			left_event_count = FTS1BA90A_FIFO_MAX - 1;

		if (left_event_count > 0) {
			reg = FTS1BA90A_READ_ALL_EVENT;
			if (fts1ba90a_i2c_read(ts, &reg, 1,
					       &ts->event_buf[FTS1BA90A_EVENT_SIZE],
					       FTS1BA90A_EVENT_SIZE * left_event_count))
				break;
		}

		for (i = 0; i <= left_event_count; i++) {
			evt = &ts->event_buf[i * FTS1BA90A_EVENT_SIZE];
			event_id = evt[0] & 0x03;

			switch (event_id) {
			case FTS1BA90A_COORDINATE_EVENT:
				fts1ba90a_handle_coordinate(ts,
					(struct fts1ba90a_event_coord *)evt);
				break;
			case FTS1BA90A_STATUS_EVENT:
				fts1ba90a_handle_status(ts,
					(struct fts1ba90a_event_status *)evt);
				break;
			}
		}
	} while (left_event_count > 0);

	input_sync(ts->input);

	mutex_unlock(&ts->mutex);
	return IRQ_HANDLED;
}

static int fts1ba90a_hw_init(struct fts1ba90a_data *ts)
{
	u8 cmd[3];
	int ret, retry = 3;

	/* Reset and wait for controller ready */
	do {
		fts1ba90a_systemreset(ts);

		ret = fts1ba90a_wait_for_ready(ts);
		if (ret == 0)
			break;

		msleep(20);
	} while (--retry > 0);

	if (retry == 0) {
		dev_err(&ts->client->dev, "controller not ready after reset\n");
		return -ETIMEDOUT;
	}

	ret = fts1ba90a_get_version(ts);
	if (ret)
		return ret;

	ret = fts1ba90a_read_chip_id(ts);
	if (ret)
		return ret;

	/* Set default touch type */
	cmd[0] = FTS1BA90A_CMD_SET_TOUCHTYPE;
	cmd[1] = (u8)(FTS1BA90A_TOUCHTYPE_DEFAULT & 0xFF);
	cmd[2] = (u8)(FTS1BA90A_TOUCHTYPE_DEFAULT >> 8);
	fts1ba90a_i2c_write(ts, cmd, 3);
	msleep(10);

	/* Force calibration */
	fts1ba90a_command_echo(ts, FTS1BA90A_CMD_FORCE_CALIBRATION);

	/* Clear event stack */
	fts1ba90a_command_echo(ts, FTS1BA90A_CMD_CLEAR_ALL_EVENT);

	/* Set default scan mode */
	fts1ba90a_set_scanmode(ts, FTS1BA90A_SCAN_MODE_DEFAULT);

	return 0;
}

static int fts1ba90a_input_open(struct input_dev *dev)
{
	struct fts1ba90a_data *ts = input_get_drvdata(dev);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ts->regulators), ts->regulators);
	if (ret)
		return ret;

	msleep(5);

	ret = fts1ba90a_hw_init(ts);
	if (ret) {
		regulator_bulk_disable(ARRAY_SIZE(ts->regulators),
				       ts->regulators);
		return ret;
	}

	enable_irq(ts->client->irq);
	return 0;
}

static void fts1ba90a_input_close(struct input_dev *dev)
{
	struct fts1ba90a_data *ts = input_get_drvdata(dev);

	disable_irq(ts->client->irq);
	fts1ba90a_release_all_fingers(ts);
	regulator_bulk_disable(ARRAY_SIZE(ts->regulators), ts->regulators);
}

static int fts1ba90a_probe(struct i2c_client *client)
{
	struct fts1ba90a_data *ts;
	struct input_dev *input;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	mutex_init(&ts->mutex);

	ts->regulators[FTS1BA90A_REGULATOR_VDD].supply = "vdd";
	ts->regulators[FTS1BA90A_REGULATOR_AVDD].supply = "avdd";
	ret = devm_regulator_bulk_get(&client->dev, ARRAY_SIZE(ts->regulators),
				      ts->regulators);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to get regulators\n");

	input = devm_input_allocate_device(&client->dev);
	if (!input)
		return -ENOMEM;

	ts->input = input;
	input->name = "STM FTS1BA90A Touchscreen";
	input->phys = "stm_fts1ba90a/input0";
	input->id.bustype = BUS_I2C;
	input->open = fts1ba90a_input_open;
	input->close = fts1ba90a_input_close;
	input_set_drvdata(input, ts);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, 65535, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 65535, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 63, 0, 0);

	touchscreen_parse_properties(input, true, &ts->prop);
	if (!ts->prop.max_x || !ts->prop.max_y) {
		dev_err(&client->dev, "touchscreen-size-x/y not specified\n");
		return -EINVAL;
	}

	ret = input_mt_init_slots(input, FTS1BA90A_MAX_FINGERS,
				  INPUT_MT_DIRECT);
	if (ret)
		return ret;

	i2c_set_clientdata(client, ts);

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					fts1ba90a_irq_handler,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"stm_fts1ba90a", ts);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to request IRQ\n");

	ret = input_register_device(input);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to register input device\n");

	return 0;
}

static int fts1ba90a_suspend(struct device *dev)
{
	struct fts1ba90a_data *ts = dev_get_drvdata(dev);

	guard(mutex)(&ts->input->mutex);

	if (input_device_enabled(ts->input)) {
		disable_irq(ts->client->irq);
		fts1ba90a_release_all_fingers(ts);
		regulator_bulk_disable(ARRAY_SIZE(ts->regulators),
				       ts->regulators);
	}

	return 0;
}

static int fts1ba90a_resume(struct device *dev)
{
	struct fts1ba90a_data *ts = dev_get_drvdata(dev);

	guard(mutex)(&ts->input->mutex);

	if (input_device_enabled(ts->input)) {
		int ret;

		ret = regulator_bulk_enable(ARRAY_SIZE(ts->regulators),
					    ts->regulators);
		if (ret)
			return ret;

		msleep(5);
		fts1ba90a_hw_init(ts);
		enable_irq(ts->client->irq);
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(fts1ba90a_pm_ops,
				fts1ba90a_suspend, fts1ba90a_resume);

static const struct of_device_id fts1ba90a_of_match[] = {
	{ .compatible = "st,fts1ba90a" },
	{ }
};
MODULE_DEVICE_TABLE(of, fts1ba90a_of_match);

static const struct i2c_device_id fts1ba90a_id[] = {
	{ "fts1ba90a" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fts1ba90a_id);

static struct i2c_driver fts1ba90a_driver = {
	.driver = {
		.name = "stm_fts1ba90a",
		.of_match_table = fts1ba90a_of_match,
		.pm = pm_sleep_ptr(&fts1ba90a_pm_ops),
	},
	.probe = fts1ba90a_probe,
	.id_table = fts1ba90a_id,
};
module_i2c_driver(fts1ba90a_driver);

MODULE_DESCRIPTION("STMicroelectronics FTS1BA90A Touchscreen Driver");
MODULE_LICENSE("GPL");
