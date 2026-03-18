// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S2MU004 fuel gauge driver.
 *
 * The S2MU004 fuel gauge is a coulomb-counting battery monitor found
 * in Samsung devices such as the Galaxy Tab A 10.5 (gta2xlwifi).
 * It sits on a dedicated I2C bus at address 0x3B, separate from
 * the S2MU004 MUIC/charger PMIC at 0x3D.
 *
 * Key 16-bit little-endian registers:
 *   0x04  RVBAT    – Battery voltage
 *   0x06  RCUR_CC  – Coulomb counter current
 *   0x08  RSOC     – Raw state of charge
 *   0x0A  MONOUT   – Multiplex monitor output
 *   0x0C  MONOUT_SEL – Monitor output selector
 *   0x1A  IRQ_LVL  – Alert thresholds
 *   0x1E  START    – FG start / POR flag
 *   0x25  CTRL0    – Control register 0
 *   0x48  FG_ID    – Chip / model ID
 *
 * Voltage:  mV = (RVBAT * 1000) >> 13
 * Current:  mA = (|RCUR_CC| * 1000) >> 12  (bit 15 = charging flag)
 * SoC:      %  = (RSOC & 0x7FFF) / 100
 * Temp:     °C = ((MONOUT[0x10] * 100) >> 8) / 10  (2's complement)
 *
 * Copyright (c) 2025
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

/* Register addresses (8-bit, data is 16-bit LE pairs) */
#define S2MU004_FG_RVBAT	0x04
#define S2MU004_FG_RCUR_CC	0x06
#define S2MU004_FG_RSOC	0x08
#define S2MU004_FG_MONOUT	0x0A
#define S2MU004_FG_MONOUT_SEL	0x0C
#define S2MU004_FG_START	0x1E
#define S2MU004_FG_CTRL0	0x25
#define S2MU004_FG_ID		0x48

/* MONOUT_SEL values */
#define MONOUT_SEL_TEMP		0x10
#define MONOUT_SEL_AVG_CUR	0x26
#define MONOUT_SEL_AVG_VBAT	0x27

struct s2mu004_fg {
	struct i2c_client *client;
	struct regmap *regmap;
	struct power_supply *psy;
	struct mutex lock;	/* protects MONOUT mux sequences */
};

/* ---- low-level register helpers ---- */

static int s2mu004_fg_read16(struct regmap *map, u8 reg, u16 *val)
{
	u8 data[2];
	int ret;

	ret = regmap_bulk_read(map, reg, data, 2);
	if (ret)
		return ret;

	*val = data[0] | (data[1] << 8);
	return 0;
}

/* Read a MONOUT value — caller must hold fg->lock */
static int s2mu004_fg_read_monout(struct s2mu004_fg *fg, u8 sel, u16 *val)
{
	int ret;

	ret = regmap_write(fg->regmap, S2MU004_FG_MONOUT_SEL, sel);
	if (ret)
		return ret;

	ret = s2mu004_fg_read16(fg->regmap, S2MU004_FG_MONOUT, val);

	/* Restore default selector */
	regmap_write(fg->regmap, S2MU004_FG_MONOUT_SEL, MONOUT_SEL_TEMP);

	return ret;
}

/* ---- measurement functions ---- */

/*
 * Battery voltage in microvolts.
 * Vendor formula: mV = (raw * 1000) >> 13
 */
static int s2mu004_fg_get_voltage(struct s2mu004_fg *fg, int *uv)
{
	u16 raw;
	int ret;

	ret = s2mu004_fg_read16(fg->regmap, S2MU004_FG_RVBAT, &raw);
	if (ret)
		return ret;

	*uv = (((int)raw * 1000) >> 13) * 1000;
	return 0;
}

/*
 * Battery current in microamps.
 * Bit 15 set = charging (2's complement negative raw value).
 * Kernel convention: negative = charging, positive = discharging.
 */
static int s2mu004_fg_get_current(struct s2mu004_fg *fg, int *ua)
{
	u16 raw;
	int ret, mag;

	ret = s2mu004_fg_read16(fg->regmap, S2MU004_FG_RCUR_CC, &raw);
	if (ret)
		return ret;

	if (raw & BIT(15)) {
		mag = ((~raw) & 0xFFFF) + 1;
		*ua = -(int)(((u32)mag * 1000) >> 12) * 1000;
	} else {
		mag = raw & 0x7FFF;
		*ua = (int)(((u32)mag * 1000) >> 12) * 1000;
	}

	return 0;
}

/*
 * State of charge in percent (0–100).
 * Raw register: bits 14:0 in 0.01% units (0–10000 = 0–100.00%).
 * Double-read to ensure stable value.
 */
static int s2mu004_fg_get_soc(struct s2mu004_fg *fg, int *pct)
{
	u16 r1, r2;
	int ret, i;

	for (i = 0; i < 5; i++) {
		ret = s2mu004_fg_read16(fg->regmap, S2MU004_FG_RSOC, &r1);
		if (ret)
			return ret;
		ret = s2mu004_fg_read16(fg->regmap, S2MU004_FG_RSOC, &r2);
		if (ret)
			return ret;
		if (r1 == r2)
			break;
	}

	*pct = clamp((int)(r1 & 0x7FFF) / 100, 0, 100);
	return 0;
}

/*
 * Chip temperature in tenths of a degree Celsius.
 * MONOUT with selector 0x10 returns 2's complement, scaled by (100 >> 8)/10.
 */
static int s2mu004_fg_get_temp(struct s2mu004_fg *fg, int *deci_celsius)
{
	u16 raw;
	int ret, temp;

	mutex_lock(&fg->lock);
	ret = s2mu004_fg_read_monout(fg, MONOUT_SEL_TEMP, &raw);
	mutex_unlock(&fg->lock);
	if (ret)
		return ret;

	if (raw & BIT(15))
		temp = -((int)((~raw) & 0xFFFF) + 1);
	else
		temp = raw & 0x7FFF;

	*deci_celsius = ((temp * 100) >> 8) / 10;
	return 0;
}

/* ---- power_supply interface ---- */

static int s2mu004_fg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct s2mu004_fg *fg = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = s2mu004_fg_get_voltage(fg, &val->intval);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = s2mu004_fg_get_current(fg, &val->intval);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		ret = s2mu004_fg_get_soc(fg, &val->intval);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		ret = s2mu004_fg_get_temp(fg, &val->intval);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 7300000; /* 7300 mAh in µAh */
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property s2mu004_fg_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static const struct power_supply_desc s2mu004_fg_desc = {
	.name		= "s2mu004-fuelgauge",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= s2mu004_fg_props,
	.num_properties	= ARRAY_SIZE(s2mu004_fg_props),
	.get_property	= s2mu004_fg_get_property,
};

/* ---- init / probe ---- */

static void s2mu004_fg_init_regs(struct s2mu004_fg *fg)
{
	unsigned int val;

	/*
	 * Vendor init sequence — tune top-off current sensing
	 * and mixed-mode interrupt source. These are undocumented
	 * register tweaks from the Samsung downstream driver.
	 */

	/* Reduce top-off current difference (reg 0x27 bit 4) */
	if (!regmap_read(fg->regmap, 0x27, &val))
		regmap_write(fg->regmap, 0x27, val | 0x10);

	/* Interrupt source: mixed mode (reg 0x43 bits 3:2 = 10) */
	if (!regmap_read(fg->regmap, 0x43, &val))
		regmap_write(fg->regmap, 0x43, (val & 0xF3) | 0x08);

	/* Charger top-off sensing method (reg 0x49 bit 7 = 0) */
	if (!regmap_read(fg->regmap, 0x49, &val))
		regmap_write(fg->regmap, 0x49, val & 0x7F);
}

static const struct regmap_config s2mu004_fg_regmap_cfg = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= S2MU004_FG_ID,
};

static int s2mu004_fg_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct s2mu004_fg *fg;
	unsigned int chip_id;
	int ret;

	fg = devm_kzalloc(&client->dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->client = client;
	mutex_init(&fg->lock);

	fg->regmap = devm_regmap_init_i2c(client, &s2mu004_fg_regmap_cfg);
	if (IS_ERR(fg->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(fg->regmap),
				     "failed to init regmap\n");

	/* Verify the chip is present */
	ret = regmap_read(fg->regmap, S2MU004_FG_ID, &chip_id);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read FG_ID\n");

	dev_info(&client->dev, "S2MU004 fuel gauge ID: 0x%02x\n", chip_id);

	s2mu004_fg_init_regs(fg);

	psy_cfg.drv_data = fg;
	psy_cfg.fwnode = dev_fwnode(&client->dev);

	fg->psy = devm_power_supply_register(&client->dev,
					     &s2mu004_fg_desc, &psy_cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(&client->dev, PTR_ERR(fg->psy),
				     "failed to register power supply\n");

	return 0;
}

static const struct of_device_id s2mu004_fg_of_match[] = {
	{ .compatible = "samsung,s2mu004-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mu004_fg_of_match);

static const struct i2c_device_id s2mu004_fg_id_table[] = {
	{ "s2mu004-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s2mu004_fg_id_table);

static struct i2c_driver s2mu004_fg_driver = {
	.driver = {
		.name		= "s2mu004-fuelgauge",
		.of_match_table	= s2mu004_fg_of_match,
	},
	.probe		= s2mu004_fg_probe,
	.id_table	= s2mu004_fg_id_table,
};
module_i2c_driver(s2mu004_fg_driver);

MODULE_DESCRIPTION("Samsung S2MU004 fuel gauge driver");
MODULE_LICENSE("GPL");
