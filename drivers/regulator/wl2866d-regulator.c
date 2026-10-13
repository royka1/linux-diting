// SPDX-License-Identifier: GPL-2.0-only
/*
 * Willsemi WL2866D four channel camera LDO
 *
 * Two DVDD LDOs (0.6 V to 1.8 V in 6 mV steps, fed from VIN1) and two
 * AVDD LDOs (1.2 V to 4.3 V in 12.5 mV steps, fed from VIN2), set over
 * I2C. Register layout as used by the vendor driver of the Xiaomi 12T Pro:
 * voltage codes at 0x03 (DVDD1), 0x04 (DVDD2), 0x05 (AVDD1), 0x06 (AVDD2),
 * output enables in 0x0e (bit 0 DVDD1, 1 DVDD2, 2 AVDD1, 3 AVDD2). The
 * chip itself is enabled by its EN pin.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define WL2866D_REG_DVDD1_VOUT		0x03
#define WL2866D_REG_ENABLE		0x0e

enum wl2866d_ldo {
	WL2866D_DVDD1,
	WL2866D_DVDD2,
	WL2866D_AVDD1,
	WL2866D_AVDD2,
	WL2866D_NUM_LDOS,
};

static const struct regulator_ops wl2866d_ldo_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

#define WL2866D_LDO(_id, _name, _supply, _min, _step, _n)		\
	[_id] = {							\
		.name = _name,						\
		.of_match = of_match_ptr(_name),			\
		.regulators_node = of_match_ptr("regulators"),		\
		.supply_name = _supply,					\
		.id = _id,						\
		.ops = &wl2866d_ldo_ops,				\
		.type = REGULATOR_VOLTAGE,				\
		.owner = THIS_MODULE,					\
		.min_uV = _min,						\
		.uV_step = _step,					\
		.n_voltages = _n,					\
		.vsel_reg = WL2866D_REG_DVDD1_VOUT + (_id),		\
		.vsel_mask = 0xff,					\
		.enable_reg = WL2866D_REG_ENABLE,			\
		.enable_mask = BIT(_id),				\
		.enable_time = 1000,					\
	}

static const struct regulator_desc wl2866d_ldos[WL2866D_NUM_LDOS] = {
	WL2866D_LDO(WL2866D_DVDD1, "dvdd1", "vin1", 600000, 6000, 201),
	WL2866D_LDO(WL2866D_DVDD2, "dvdd2", "vin1", 600000, 6000, 201),
	WL2866D_LDO(WL2866D_AVDD1, "avdd1", "vin2", 1200000, 12500, 249),
	WL2866D_LDO(WL2866D_AVDD2, "avdd2", "vin2", 1200000, 12500, 249),
};

static const struct regmap_config wl2866d_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = WL2866D_REG_ENABLE,
};

static int wl2866d_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { };
	struct gpio_desc *enable_gpio;
	struct regulator_dev *rdev;
	struct regmap *regmap;
	unsigned int val;
	int i, ret;

	regmap = devm_regmap_init_i2c(client, &wl2866d_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "failed to init regmap\n");

	/* Chip enable: the outputs are switched individually over I2C */
	enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(enable_gpio))
		return dev_err_probe(dev, PTR_ERR(enable_gpio),
				     "failed to get enable GPIO\n");
	if (enable_gpio)
		usleep_range(1000, 2000);

	ret = regmap_read(regmap, WL2866D_REG_ENABLE, &val);
	if (ret)
		return dev_err_probe(dev, ret, "failed to access the chip\n");

	config.dev = dev;
	config.regmap = regmap;

	for (i = 0; i < WL2866D_NUM_LDOS; i++) {
		rdev = devm_regulator_register(dev, &wl2866d_ldos[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     wl2866d_ldos[i].name);
	}

	return 0;
}

static const struct of_device_id wl2866d_of_match[] = {
	{ .compatible = "willsemi,wl2866d" },
	{ }
};
MODULE_DEVICE_TABLE(of, wl2866d_of_match);

static const struct i2c_device_id wl2866d_id[] = {
	{ "wl2866d" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wl2866d_id);

static struct i2c_driver wl2866d_driver = {
	.driver = {
		.name = "wl2866d-regulator",
		.of_match_table = wl2866d_of_match,
	},
	.probe = wl2866d_probe,
	.id_table = wl2866d_id,
};
module_i2c_driver(wl2866d_driver);

MODULE_DESCRIPTION("Willsemi WL2866D camera LDO regulator driver");
MODULE_LICENSE("GPL");
