// SPDX-License-Identifier: GPL-2.0+
/*
 * dio8018, Multi-Output Regulators
 * Copyright (C) 2023  Motorola Mobility LLC,
 *
 * Author: ChengLong, Motorola Mobility LLC,
 */

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/debug-regulator.h>
#include "dio8018-regulator.h"

// #ifdef dev_info
// #undef dev_info
// #endif
// #define dev_info dev_err

#define MAX_REG_SUPPLY_NUM (2)

static int ldo_chipid = -1;
enum dio8018_regulators {
	DIO8018_REGULATOR_LDO1 = 0,
	DIO8018_REGULATOR_LDO2,
	DIO8018_REGULATOR_LDO3,
	DIO8018_REGULATOR_LDO4,
	DIO8018_REGULATOR_LDO5,
	DIO8018_REGULATOR_LDO6,
	DIO8018_REGULATOR_LDO7,
	DIO8018_MAX_REGULATORS,
};

struct dio8018 {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_desc *rdesc[DIO8018_MAX_REGULATORS];
	struct regulator_dev *rdev[DIO8018_MAX_REGULATORS];
	const char *reg_name[MAX_REG_SUPPLY_NUM];
	int reg_min_vol[MAX_REG_SUPPLY_NUM];
	int reg_max_vol[MAX_REG_SUPPLY_NUM];
	struct regulator *reg_list[MAX_REG_SUPPLY_NUM];
	int reg_num;
	int chip_cs_pin;
};

static const struct regmap_range dio8018_writeable_ranges[] = {
	regmap_reg_range(DIO8018_IOUT, DIO8018_RESET),
};

static const struct regmap_range dio8018_readable_ranges[] = {
	regmap_reg_range(DIO8018_PRODUCT_ID, DIO8018_MINT3),
};

static const struct regmap_range dio8018_volatile_ranges[] = {
	regmap_reg_range(DIO8018_IOUT, DIO8018_MINT3),
};

static const struct regmap_access_table dio8018_writeable_table = {
	.yes_ranges   = dio8018_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(dio8018_writeable_ranges),
};

static const struct regmap_access_table dio8018_readable_table = {
	.yes_ranges   = dio8018_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(dio8018_readable_ranges),
};

static const struct regmap_access_table dio8018_volatile_table = {
	.yes_ranges   = dio8018_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(dio8018_volatile_ranges),
};

static const struct regmap_config dio8018_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = DIO8018_MINT3,
	.wr_table = &dio8018_writeable_table,
	.rd_table = &dio8018_readable_table,
	.volatile_table = &dio8018_volatile_table,
};

static int dio8018_get_error_flags(struct regulator_dev *rdev, unsigned int *flags)
{
	struct dio8018 *chip = rdev_get_drvdata(rdev);
	uint8_t reg_dump[DIO8018_REG_NUM];
	uint8_t reg_idx;
	unsigned int val = 0;

	dev_err(chip->dev, "************ start dump dio8018 register ************\n");
	dev_err(chip->dev, "register 0x00:      chip version\n");
	dev_err(chip->dev, "register 0x01:      LDO CL\n");
	dev_err(chip->dev, "register 0x03~0x09: LDO1~LDO7 OUT Voltage\n");
	dev_err(chip->dev, "register 0x0e:      Bit[6:0] LDO7~LDO1 EN\n");

	for (reg_idx = 0; reg_idx < DIO8018_REG_NUM; reg_idx++) {
		regmap_read(chip->regmap, reg_idx, &val);
		reg_dump[reg_idx] = val;
		dev_err(chip->dev, "Reg[0x%02x] = 0x%x", reg_idx, reg_dump[reg_idx]);
	}
	dev_err(chip->dev, "************ end dump dio8018 register ************\n");

	if (flags != NULL) {
		*flags = 0;
	}

	return 0;
}

static int dio8018_get_status(struct regulator_dev *rdev)
{
	struct dio8018 *chip = rdev_get_drvdata(rdev);
	int ret, id = rdev_get_id(rdev);
	unsigned int status = 0;

	ret = regulator_is_enabled_regmap(rdev);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read enable register(%d)\n", ret);
		return ret;
	}

	if (!ret)
		return REGULATOR_STATUS_OFF;

	dio8018_get_error_flags(rdev, NULL);

	ret = regmap_read(chip->regmap, DIO8018_LDO_EN, &status);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read status register(%d)\n", ret);
		return ret;
	}

	if (status & (0x01ul << id)) {
		return REGULATOR_STATUS_ON;
	} else {
		return REGULATOR_STATUS_OFF;
	}
}

#if 0
//ONLY FOR DEBUG
int debug_regulator_map_voltage_linear_range(struct regulator_dev *rdev,
				       int min_uV, int max_uV)
{
	int ret = 0;
	ret = regulator_map_voltage_linear_range(rdev, min_uV, max_uV);
	dev_err(&rdev->dev, "map_voltage ret:%d min/max:%d,%d\n", ret, min_uV, max_uV);
	return ret;
}

int debug_regulator_list_voltage_linear_range(struct regulator_dev *rdev,
					unsigned int selector)
{
	int ret=0;
	ret = regulator_list_voltage_linear_range(rdev, selector);
	dev_err(&rdev->dev, "list_voltage ret:%d selector:%d\n", ret, selector);
	return ret;
}

int debug_regulator_enable_regmap(struct regulator_dev *rdev)
{
	int ret=0;
	ret = regulator_enable_regmap(rdev);
	dev_err(&rdev->dev, "regulator enable ret:%d, rmap:%p en reg:%d, mask:%d\n", ret, rdev->regmap, rdev->desc->enable_reg, rdev->desc->enable_mask);
	return ret;
}

int debug_regulator_disable_regmap(struct regulator_dev *rdev)
{
	int ret=0;
	ret = regulator_disable_regmap(rdev);
	dev_err(&rdev->dev, "regulator disable ret:%d\n", ret);
	return ret;
}

int debug_regulator_is_enabled_regmap(struct regulator_dev *rdev)
{
	int ret=0;
	ret = regulator_is_enabled_regmap(rdev);
	dev_err(&rdev->dev, "regulator is enable ret:%d\n", ret);
	return ret;
}
#endif


static const struct regulator_ops dio8018_regl_ops = {
	.enable  = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	// .enable  = debug_regulator_enable_regmap,
	// .disable = debug_regulator_disable_regmap,

	.is_enabled = regulator_is_enabled_regmap,
	// .is_enabled = debug_regulator_is_enabled_regmap,

	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage  = regulator_map_voltage_linear_range,
	// .list_voltage = debug_regulator_list_voltage_linear_range,
	// .map_voltage  = debug_regulator_map_voltage_linear_range,

	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,

	.get_status = dio8018_get_status,
	.get_error_flags = dio8018_get_error_flags,
};

static int dio8018_of_parse_cb(struct device_node *np, const struct regulator_desc *desc, struct regulator_config *config)
{
	int ena_gpio;

	ena_gpio = of_get_named_gpio(np, "enable-gpios", 0);
	if (gpio_is_valid(ena_gpio))
		config->ena_gpiod = gpio_to_desc(ena_gpio);

	return 0;
}

const struct linear_range dio8018_regls_range[DIO8018_MAX_REGULATORS] = {
	{.min = 504000, .min_sel=0x3E, .max_sel=0xBB, .step=8000,},
	{.min = 504000, .min_sel=0x3E, .max_sel=0xBB, .step=8000,},
	{.min = 1500000, .min_sel=0x10, .max_sel=0xFF, .step=8000,},
	{.min = 1500000, .min_sel=0x10, .max_sel=0xFF, .step=8000,},
	{.min = 1500000, .min_sel=0x10, .max_sel=0xFF, .step=8000,},
	{.min = 1500000, .min_sel=0x10, .max_sel=0xFF, .step=8000,},
	{.min = 1500000, .min_sel=0x10, .max_sel=0xFF, .step=8000,},
};

const struct linear_range wl28681c_regls_range[DIO8018_MAX_REGULATORS] = {
	{.min = 496000, .min_sel=0x3D, .max_sel=0xFF, .step=8000,},
	{.min = 496000, .min_sel=0x3D, .max_sel=0xFF, .step=8000,},
	{.min = 1372000, .min_sel=0x00, .max_sel=0xFF, .step=8000,},
	{.min = 1372000, .min_sel=0x00, .max_sel=0xFF, .step=8000,},
	{.min = 1372000, .min_sel=0x00, .max_sel=0xFF, .step=8000,},
	{.min = 1372000, .min_sel=0x00, .max_sel=0xFF, .step=8000,},
	{.min = 1372000, .min_sel=0x00, .max_sel=0xFF, .step=8000,},
};

#define DIO8018_REGL_DESC(_id, _name, _s_name, _min, _step, _min_linear_sel, _n_volt)       \
	[DIO8018_REGULATOR_##_id] = {                             \
		.name = #_name,                                    \
		.supply_name = _s_name,                            \
		.id = DIO8018_REGULATOR_##_id,                    \
		.of_match = of_match_ptr(#_name),                  \
		.of_parse_cb = dio8018_of_parse_cb,               \
		.ops = &dio8018_regl_ops,                         \
		.regulators_node = of_match_ptr("regulators"),     \
		.n_voltages = _n_volt,                                 \
		.min_uV = _min,                                    \
		.uV_step = _step,                                  \
		.linear_min_sel = _min_linear_sel,                 \
		.n_linear_ranges = 1,                    \
		.linear_ranges = &dio8018_regls_range[DIO8018_REGULATOR_##_id],   \
		.vsel_mask = DIO8018_VSEL_MASK,               \
		.vsel_reg = DIO8018_##_id##_VSEL,                 \
		.enable_reg = DIO8018_LDO_EN,       \
		.enable_mask = BIT(DIO8018_REGULATOR_##_id),     \
		.type = REGULATOR_VOLTAGE,                         \
		.owner = THIS_MODULE,                              \
	}

static struct regulator_desc dio8018_regls_desc[DIO8018_MAX_REGULATORS] = {
	DIO8018_REGL_DESC(LDO1, ldo1, "vin1", 504000,  8000, 62, 188),
	DIO8018_REGL_DESC(LDO2, ldo2, "vin1", 504000,  8000, 62, 188),
	DIO8018_REGL_DESC(LDO3, ldo3, "vin2", 1500000, 8000, 16, 256),
	DIO8018_REGL_DESC(LDO4, ldo4, "vin2", 1500000, 8000, 16, 256),
	DIO8018_REGL_DESC(LDO5, ldo5, "vin2", 1500000, 8000, 16, 256),
	DIO8018_REGL_DESC(LDO6, ldo6, "vin2", 1500000, 8000, 16, 256),
	DIO8018_REGL_DESC(LDO7, ldo7, "vin2", 1500000, 8000, 16, 256),
};

static int dio8018_regulator_init(struct dio8018 *chip)
{
	struct regulator_config config = { };
	struct regulator_desc *rdesc;
	u8 vsel_range[1];
	int id, ret = 0;

	unsigned int ldo_regs[DIO8018_MAX_REGULATORS] = {
		DIO8018_LDO1_VOUT,
		DIO8018_LDO2_VOUT,
		DIO8018_LDO3_VOUT,
		DIO8018_LDO4_VOUT,
		DIO8018_LDO5_VOUT,
		DIO8018_LDO6_VOUT,
		DIO8018_LDO7_VOUT,
	};

	unsigned int initial_voltage[DIO8018_MAX_REGULATORS] = {
		0x83,//LDO1 1.056V
		0x9A,//LDO2 1.24V
		0xB3,//LDO3 2.804V
		0xB3,//LDO4 2.804V
		0xB3,//LDO5 2.804V
		0xB3,//LDO6 2.804V
		0x36,//LDO7 1.804V
	};

	/*Disable all ldo output by default*/
	ret = regmap_write(chip->regmap, DIO8018_LDO_EN, 0x80);
	if (ret < 0) {
		dev_err(chip->dev, "Disable all LDO output failed!!!\n");
		return ret;
	}

	if(ldo_chipid == 0x0D){
		dio8018_regls_desc[DIO8018_REGULATOR_LDO1].min_uV = 496000;
		dio8018_regls_desc[DIO8018_REGULATOR_LDO1].n_voltages = 256;
		dio8018_regls_desc[DIO8018_REGULATOR_LDO1].linear_ranges = &wl28681c_regls_range[DIO8018_REGULATOR_LDO1];
		dio8018_regls_desc[DIO8018_REGULATOR_LDO2].min_uV = 496000;
		dio8018_regls_desc[DIO8018_REGULATOR_LDO2].n_voltages = 256;
		dio8018_regls_desc[DIO8018_REGULATOR_LDO2].linear_ranges = &wl28681c_regls_range[DIO8018_REGULATOR_LDO2];
		dio8018_regls_desc[DIO8018_REGULATOR_LDO3].min_uV = 1372000;
		dio8018_regls_desc[DIO8018_REGULATOR_LDO3].linear_ranges = &wl28681c_regls_range[DIO8018_REGULATOR_LDO3];
		dio8018_regls_desc[DIO8018_REGULATOR_LDO4].min_uV = 1372000;
		dio8018_regls_desc[DIO8018_REGULATOR_LDO4].linear_ranges = &wl28681c_regls_range[DIO8018_REGULATOR_LDO4];
		dio8018_regls_desc[DIO8018_REGULATOR_LDO5].min_uV = 1372000;
		dio8018_regls_desc[DIO8018_REGULATOR_LDO5].linear_ranges = &wl28681c_regls_range[DIO8018_REGULATOR_LDO5];
		dio8018_regls_desc[DIO8018_REGULATOR_LDO6].min_uV = 1372000;
		dio8018_regls_desc[DIO8018_REGULATOR_LDO6].linear_ranges = &wl28681c_regls_range[DIO8018_REGULATOR_LDO6];
		dio8018_regls_desc[DIO8018_REGULATOR_LDO7].min_uV = 1372000;
		dio8018_regls_desc[DIO8018_REGULATOR_LDO7].linear_ranges = &wl28681c_regls_range[DIO8018_REGULATOR_LDO7];
	}

	for (id = DIO8018_MAX_REGULATORS-1; id >= 0; id--)
	{
		chip->rdesc[id] = &dio8018_regls_desc[id];
		rdesc              = chip->rdesc[id];
		config.regmap      = chip->regmap;
		config.dev         = chip->dev;
		config.driver_data = chip;

		dev_info(chip->dev, "Regulator[%d] supply_name: %s, volt range:%d - %d\n", id, rdesc->supply_name,
		    rdesc->linear_ranges[0].min, rdesc->linear_ranges[0].min+(rdesc->linear_ranges[0].max_sel-rdesc->linear_ranges[0].min_sel)*rdesc->linear_ranges[0].step);

		ret = regmap_bulk_read(chip->regmap, ldo_regs[id], vsel_range, 1);
		pr_debug("dio8018_regulator_init: LDO%d, ldo_regs=0x%x default value:0x%x", (id+1),ldo_regs[id],vsel_range[0]);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to read the ldo register\n");
			return ret;
		}

		pr_debug("dio8018_regulator_init: enable_reg0x%x, enable_mask:0x%x", dio8018_regls_desc[id].enable_reg, dio8018_regls_desc[id].enable_mask);

		ret = regmap_write(chip->regmap, ldo_regs[id], initial_voltage[id]);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to write inital voltage register\n");
			return ret;
		}
		pr_debug("dio8018_regulator_init: LDO%d, initial value:0x%x", (id+1), initial_voltage[id]);

		chip->rdev[id] = devm_regulator_register(chip->dev, rdesc, &config);
		if (IS_ERR(chip->rdev[id])) {
			ret = PTR_ERR(chip->rdev[id]);
			dev_err(chip->dev, "Failed to register regulator(%s):%d\n", chip->rdesc[id]->name, ret);
			return ret;
		}

		// ret = devm_regulator_debug_register(chip->dev, chip->rdev[id]);
		// if (IS_ERR(chip->rdev[id])) {
		// 	ret = PTR_ERR(chip->rdev[id]);
		// 	dev_err(chip->dev, "Failed to register regulator debug fs(%s):%d\n", chip->rdesc[id]->name, ret);
		// 	return ret;
		// }
	}

	return 0;
}

static int dio8018_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct dio8018 *chip;
	int cs_gpio, ret, i, count;
	struct device_node *of_node = NULL;

	chip = devm_kzalloc(dev, sizeof(struct dio8018), GFP_KERNEL);
	if (!chip) {
		dev_err(chip->dev, "dio8018_i2c_probe Memory error...\n");
		return -ENOMEM;
	}

	/*Power on DIO8018 and enable pull up power supply*/
	of_node = dev->of_node;
	count = of_property_count_strings(dev->of_node, "regulator-names");
	if (count > 0) {
		chip->reg_num = count;
		for (i=0; i<chip->reg_num; i++) {
			ret = of_property_read_string_index(of_node,
				"regulator-names", i, &chip->reg_name[i]);
			dev_info(chip->dev, "reg_name[%d] = %s", i, chip->reg_name[i]);
			if (ret) {
				dev_err(chip->dev, "no regulator resource at cnt=%d", i);
				return -ENODEV;
			}
		}

		ret = of_property_read_u32_array(of_node, "rgltr-min-voltage", chip->reg_min_vol, chip->reg_num);
		if (ret) {
			dev_err(chip->dev, "No minimum volatage value found, ret=%d", ret);
			return -EINVAL;
		}

		ret = of_property_read_u32_array(of_node, "rgltr-max-voltage", chip->reg_max_vol, chip->reg_num);
		if (ret) {
			dev_err(chip->dev, "No maximum volatage value found, ret=%d", ret);
			return -EINVAL;
		}

		for (i=0; i<chip->reg_num; i++) {
			chip->reg_list[i] = devm_regulator_get(dev, chip->reg_name[i]);
			if (chip->reg_list[i] != NULL) {
				dev_info(chip->dev, "DIO8018 %s regulator get successed, volt min/max: %d - %d", chip->reg_name[i], chip->reg_min_vol[i], chip->reg_max_vol[i]);
				ret = regulator_set_voltage(chip->reg_list[i], chip->reg_min_vol[i], chip->reg_max_vol[i]);
				if (ret) {
					dev_err(chip->dev, "regulator %s set volt failed ret=%d", chip->reg_name[i], ret);
				}
				ret = regulator_enable(chip->reg_list[i]);
			} else {
				dev_warn(dev, "%s regulator get failed or absent!!!\n", chip->reg_name[i]);
			}
		}
	}

	cs_gpio = of_get_named_gpio(dev->of_node, "semi,cs-gpios", 0);
	if (cs_gpio > 0) {
		if (!gpio_is_valid(cs_gpio)) {
			dev_err(dev, "Invalid chip select pin\n");
			return -EPERM;
		}

		//ret = devm_gpio_request_one(dev, cs_gpio, GPIOF_OUT_INIT_HIGH, "dio8018_cs_pin");
		ret = devm_gpio_request(dev, cs_gpio, "dio8018_cs_pin");
		if (ret) {
			dev_err(dev, "GPIO(%d) request failed(%d)\n", cs_gpio, ret);
			return ret;
		}
		gpio_direction_output(cs_gpio, 0);
		mdelay(5);
		gpio_direction_output(cs_gpio, 1);

		chip->chip_cs_pin = cs_gpio;
	}

	dev_info(chip->dev, "dio8018_i2c_probe cs_gpio:%d...\n", cs_gpio);

	mdelay(5);

	dev_info(chip->dev, "dio8018_i2c_probe Enter...\n");

	i2c_set_clientdata(client, chip);
	chip->dev    = dev;
	chip->regmap = devm_regmap_init_i2c(client, &dio8018_regmap_config);

	ret = regmap_write(chip->regmap, DIO8018_RESET, 0xB6);
	mdelay(1);
	//Workaround for barely register access disfunction
	ret = regmap_write(chip->regmap, DIO8018_RESET, 0xB6);
	mdelay(1);

	ret = regmap_read(chip->regmap, DIO8018_PRODUCT_ID, &ldo_chipid);
	if (DIO8018_CHIP_ID == ldo_chipid || WL28681C_CHIP_ID == ldo_chipid) {
		dev_info(chip->dev, "DIO8018 chip id matched!\n");
	} else {
		dev_err(chip->dev, "DIO8018 read chipid error...chipid=:0x%x, ret:%d\n",ldo_chipid,ret);
		return -ENOMEM;
	}

	{
		int chip_rev = 0;
		ret = regmap_bulk_read(chip->regmap, DIO8018_CHIP_REV, &chip_rev, 1);

		printk("DIO8018 chip rev: %02x\n", chip_rev);
	}

	ret = dio8018_regulator_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to init regulator(%d)\n", ret);
		return ret;
	}
	dev_info(chip->dev, "dio8018_i2c_probe Exit...\n");

	return ret;
}

static void dio8018_i2c_shutdown(struct i2c_client *client)
{
	struct dio8018 *chip = i2c_get_clientdata(client);

	if (chip) {
		/* Since some platforms doesn't have BOB power supply, if no BOB power source, the BOB on schematic is stands for
		    connecting directly to battery. Most of WL286xC chip selects BOB as input power supply for LDO2~LDO7, when
		    software doesn't have shutdown handler, LD02~7's input power supply is always there, there will be high risk of
		    current leak after phone power off, so force disable all ldos before phone power off.*/
		regmap_write(chip->regmap, DIO8018_LDO_EN, 0x00);
		dev_err(chip->dev, "dio8018_i2c_shutdown");
	}
}

static int dio8018_i2c_remove(struct i2c_client *client)
{
	struct dio8018 *chip = i2c_get_clientdata(client);
	struct gpio_desc *desc;
	int ret = 0, i;

	if (chip->chip_cs_pin > 0) {
		desc = gpio_to_desc(chip->chip_cs_pin);
		ret = gpiod_direction_output_raw(desc, GPIOF_INIT_LOW);
		devm_gpiod_put(chip->dev, desc);
	}

	//Disable DIO8018 chip power supply
	for (i=0; i<chip->reg_num; i++) {
		if (chip->reg_list[i] != NULL) {
			regulator_disable(chip->reg_list[i]);
			devm_regulator_put(chip->reg_list[i]);
			chip->reg_list[i] = NULL;
		}
	}

	return ret;
}

static const struct i2c_device_id dio8018_i2c_id[] = {
	{"dio8018", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, dio8018_i2c_id);

static struct i2c_driver dio8018_regulator_driver = {
	.driver = {
		.name = "dio8018-regulator",
	},
	.probe = dio8018_i2c_probe,
	.remove = dio8018_i2c_remove,
	.shutdown = dio8018_i2c_shutdown,
	.id_table = dio8018_i2c_id,
};

module_i2c_driver(dio8018_regulator_driver);

MODULE_AUTHOR("ChengLong <chengl1@motorola.com>");
MODULE_DESCRIPTION("dio8018 regulator driver");
MODULE_LICENSE("GPL");
