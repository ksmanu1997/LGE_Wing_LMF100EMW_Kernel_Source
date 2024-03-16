/*
 * P1 DSV  MFD Driver
 *
 * Copyright 2014 LG Electronics Inc,
 *
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include <linux/mfd/sm5109.h>

static struct sm5109 *sm5109_base;
static struct mfd_cell sm5109_devs[] = {
	{ .name = "sm5109_dev" },
};


int sm5109_register_set(u8 address, u8 value)
{
	struct i2c_client *cl;
	int ret = 0;



	if (sm5109_base == NULL) {
		pr_err("[Display][%s] invalid sm5109 address \n", __func__);
		return -ENODEV;
	}

	cl = container_of(sm5109_base->dev, struct i2c_client, dev);
	if (cl == NULL) {
		pr_err("[Display][%s] invalid i2c client address \n", __func__);
		return -EINVAL;
	}

	ret = i2c_smbus_write_byte_data(cl, address, value);
	if(ret < 0)
		pr_err("[Display][%s] failed to set address %d\n", __func__, address);
	else
		pr_err("[Display][%s] success write(addr:%0x value:%0x) \n", __func__, address, value);

	return ret;
}
EXPORT_SYMBOL_GPL(sm5109_register_set);

int sm5109_set_output_voltage(uint8_t val)
{
	int ret = 0;
	struct i2c_client *cl;

	if (sm5109_base == NULL) {
		pr_err("[Display][%s] sm5109 was not probed\n", __func__);
		return -ENODEV;
	}

	cl = container_of(sm5109_base->dev, struct i2c_client, dev);
	if (cl == NULL) {
		pr_err("[Display][%s] invalid i2c client address \n", __func__);
		return -EINVAL;
	}

	ret = i2c_smbus_write_byte_data(cl, SM5109_VPOS_VOLTAGE_REG, val);
	if (ret == 0) {
		ret = i2c_smbus_read_byte_data(cl, SM5109_VPOS_VOLTAGE_REG);
		if (ret < 0) {
			pr_err("[Display][%s] SM5109_VPOS_VOLTAGE_REG read failed\n", __func__);
			return ret;
		} else {
			pr_err("[Display][%s] SM5109_VPOS_VOLTAGE_REG = 0x%02X\n", __func__, ret);
			ret = 0;
		}
	} else {
		pr_err("[Display][%s] SM5109_VPOS_VOLTAGE_REG write failed\n", __func__);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(cl, SM5109_VNEG_VOLTAGE_REG, val);
	if (ret == 0) {
		ret = i2c_smbus_read_byte_data(cl, SM5109_VNEG_VOLTAGE_REG);
		if (ret < 0) {
			pr_err("[Display][%s] SM5109_VNEG_VOLTAGE_REG read failed\n", __func__);
			return ret;
		} else {
			pr_err("[Display][%s] SM5109_VNEG_VOLTAGE_REG = 0x%02X\n", __func__, ret);
			ret = 0;
		}
	} else {
		pr_err("[Display][%s] SM5109_VNEG_VOLTAGE_REG write failed\n", __func__);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(sm5109_set_output_voltage);

static struct regmap_config sm5109_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SM5109_MAX_REGISTERS,
};

static int sm5109_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct sm5109 *sm5109;
	struct device *dev = &cl->dev;
	struct sm5109_platform_data *pdata = dev_get_platdata(dev);
	int rc = 0;

	pr_err("[Display][%s] sm5109_probe\n", __func__);

	sm5109 = devm_kzalloc(dev, sizeof(*sm5109), GFP_KERNEL);
	if (!sm5109) {
		pr_err("[Display][%s] mem alloc failed \n", __func__);
		return -ENOMEM;
	}

	sm5109->pdata = pdata;

	sm5109->regmap = devm_regmap_init_i2c(cl, &sm5109_regmap_config);
	if (IS_ERR(sm5109->regmap)) {
		pr_err("[Display][%s] Failed to allocate register map\n", __func__);
		devm_kfree(dev, sm5109);
		return PTR_ERR(sm5109->regmap);
	}

	sm5109->dev = &cl->dev;
	i2c_set_clientdata(cl, sm5109);
	sm5109_base = sm5109;

	rc = mfd_add_devices(dev, -1, sm5109_devs, ARRAY_SIZE(sm5109_devs),
			       NULL, 0, NULL);
	if (rc) {
		pr_err("[Display][%s] Failed to add sm5109 subdevice ret=%d\n", __func__, rc);
		return -ENODEV;
	}

	pr_info("[Display][%s] done \n", __func__);

	return rc;
}

static int sm5109_remove(struct i2c_client *cl)
{
	struct sm5109 *sm5109 = i2c_get_clientdata(cl);

	mfd_remove_devices(sm5109->dev);

	return 0;
}

static const struct i2c_device_id sm5109_ids[] = {
	{ "sm5109", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5109_ids);

#ifdef CONFIG_OF
static const struct of_device_id sm5109_of_match[] = {
	{ .compatible = "sm5109", },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5109_of_match);
#endif

static struct i2c_driver sm5109_driver = {
	.driver = {
		.name = "sm5109",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(sm5109_of_match),
#endif
	},
	.id_table = sm5109_ids,
	.probe = sm5109_probe,
	.remove = sm5109_remove,
};
module_i2c_driver(sm5109_driver);

MODULE_DESCRIPTION("sm5109 MFD Core");
