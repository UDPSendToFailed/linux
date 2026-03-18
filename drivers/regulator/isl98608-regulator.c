// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intersil ISL98608 LCD Display Bias Power IC regulator driver
 *
 * The ISL98608 provides VSP (positive) and VSN (negative) supply rails
 * for LCD panels via a single-inductor boost converter.  Output voltages
 * are programmed over I2C and the converter is gated by an external
 * enable GPIO.
 *
 * On this device the I2C register file is accessible regardless of the
 * EN pin state, so voltage programming is performed before the enable
 * GPIO is asserted.
 *
 * Both VSP and VSN share the same enable GPIO.  Per-regulator boolean
 * flags track the framework-requested state so that the GPIO is asserted
 * when either output is enabled and de-asserted only when both are
 * disabled.  A bare integer refcount is wrong here because the
 * regulator core can skip ops->enable (when is_enabled reports true
 * from a boot-on power-up) yet still call ops->disable later, creating
 * an unbalanced decrement.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

/* ISL98608 register map */
#define ISL98608_REG_VBST	0x06	/* Boost converter voltage */
#define ISL98608_REG_VN		0x08	/* Negative output voltage */
#define ISL98608_REG_VP		0x09	/* Positive output voltage */

/*
 * VP/VN output voltage: 5.0V to 6.0V in 100mV steps.
 * Register value 0x00 = 5.0V, 0x0A = 6.0V.
 * Bits [3:0] select the voltage.
 */
#define ISL98608_VOUT_MASK	0x0F
#define ISL98608_VOUT_MIN_UV	5000000
#define ISL98608_VOUT_STEP_UV	100000
#define ISL98608_VOUT_N_VOLTAGES 11

#define ISL98608_ID_VSP		0
#define ISL98608_ID_VSN		1

struct isl98608 {
	struct regmap *regmap;
	struct gpio_desc *enable_gpio;
	struct mutex lock;
	bool vsp_enabled;
	bool vsn_enabled;
};

static int isl98608_enable(struct regulator_dev *rdev)
{
	struct isl98608 *isl = rdev_get_drvdata(rdev);
	bool *flag = (rdev_get_id(rdev) == ISL98608_ID_VSP)
		     ? &isl->vsp_enabled : &isl->vsn_enabled;

	mutex_lock(&isl->lock);
	if (!isl->vsp_enabled && !isl->vsn_enabled) {
		regcache_sync(isl->regmap);
		gpiod_set_value_cansleep(isl->enable_gpio, 1);
		usleep_range(3000, 4000);
	}
	*flag = true;
	mutex_unlock(&isl->lock);

	return 0;
}

static int isl98608_disable(struct regulator_dev *rdev)
{
	struct isl98608 *isl = rdev_get_drvdata(rdev);
	bool *flag = (rdev_get_id(rdev) == ISL98608_ID_VSP)
		     ? &isl->vsp_enabled : &isl->vsn_enabled;

	mutex_lock(&isl->lock);
	*flag = false;
	if (!isl->vsp_enabled && !isl->vsn_enabled) {
		gpiod_set_value_cansleep(isl->enable_gpio, 0);
		usleep_range(5000, 6000);
	}
	mutex_unlock(&isl->lock);

	return 0;
}

static int isl98608_is_enabled(struct regulator_dev *rdev)
{
	struct isl98608 *isl = rdev_get_drvdata(rdev);
	bool *flag = (rdev_get_id(rdev) == ISL98608_ID_VSP)
		     ? &isl->vsp_enabled : &isl->vsn_enabled;

	return *flag;
}

static const struct regulator_ops isl98608_ops = {
	.enable = isl98608_enable,
	.disable = isl98608_disable,
	.is_enabled = isl98608_is_enabled,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

static const struct regulator_desc isl98608_vsp_desc = {
	.name = "vsp",
	.of_match = "vsp",
	.id = ISL98608_ID_VSP,
	.ops = &isl98608_ops,
	.type = REGULATOR_VOLTAGE,
	.n_voltages = ISL98608_VOUT_N_VOLTAGES,
	.min_uV = ISL98608_VOUT_MIN_UV,
	.uV_step = ISL98608_VOUT_STEP_UV,
	.vsel_reg = ISL98608_REG_VP,
	.vsel_mask = ISL98608_VOUT_MASK,
	.enable_time = 5000,
	.owner = THIS_MODULE,
};

static const struct regulator_desc isl98608_vsn_desc = {
	.name = "vsn",
	.of_match = "vsn",
	.id = ISL98608_ID_VSN,
	.ops = &isl98608_ops,
	.type = REGULATOR_VOLTAGE,
	.n_voltages = ISL98608_VOUT_N_VOLTAGES,
	.min_uV = ISL98608_VOUT_MIN_UV,
	.uV_step = ISL98608_VOUT_STEP_UV,
	.vsel_reg = ISL98608_REG_VN,
	.vsel_mask = ISL98608_VOUT_MASK,
	.enable_time = 5000,
	.owner = THIS_MODULE,
};

static const struct regmap_config isl98608_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ISL98608_REG_VP,
	.cache_type = REGCACHE_FLAT,
};

static int isl98608_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = {};
	struct regulator_dev *rdev;
	struct isl98608 *isl;
	struct regmap *regmap;

	isl = devm_kzalloc(dev, sizeof(*isl), GFP_KERNEL);
	if (!isl)
		return -ENOMEM;

	regmap = devm_regmap_init_i2c(client, &isl98608_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "Failed to init regmap\n");

	isl->regmap = regmap;
	mutex_init(&isl->lock);

	isl->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(isl->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(isl->enable_gpio),
				     "Failed to get enable GPIO\n");

	regmap_write(regmap, ISL98608_REG_VBST, 0x08);
	regmap_write(regmap, ISL98608_REG_VN, 0x08);
	regmap_write(regmap, ISL98608_REG_VP, 0x08);

	i2c_set_clientdata(client, isl);

	config.dev = dev;
	config.driver_data = isl;
	config.regmap = regmap;

	rdev = devm_regulator_register(dev, &isl98608_vsp_desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev),
				     "Failed to register VSP regulator\n");

	rdev = devm_regulator_register(dev, &isl98608_vsn_desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev),
				     "Failed to register VSN regulator\n");

	return 0;
}

static const struct of_device_id isl98608_of_match[] = {
	{ .compatible = "renesas,isl98608" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, isl98608_of_match);

static const struct i2c_device_id isl98608_id[] = {
	{ "isl98608" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, isl98608_id);

static struct i2c_driver isl98608_driver = {
	.driver = {
		.name = "isl98608",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = isl98608_of_match,
	},
	.probe = isl98608_probe,
	.id_table = isl98608_id,
};
module_i2c_driver(isl98608_driver);

MODULE_DESCRIPTION("Intersil ISL98608 LCD bias regulator driver");
MODULE_LICENSE("GPL");
