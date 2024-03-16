/*
 * P1 DSV MFD Driver
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

#ifndef __MFD_SM5109_H__
#define __MFD_SM5109_H__

#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

/* Refer to data sheet and set a profit mode for the panel */
#define SM5109_VPOS_VOLTAGE_REG 			0x00
#define SM5109_VNEG_VOLTAGE_REG 			0x01
#define SM5109_DISCHARGE_STATUS_CONTROL_REG 0x03

#define SM5109_MAX_REGISTERS 0x04


struct sm5109_platform_data {
	const char *name;
};


struct sm5109 {
	struct device *dev;
	struct regmap *regmap;
	struct sm5109_platform_data *pdata;
};

extern int sm5109_set_output_voltage(uint8_t val);
int sm5109_register_set(u8 address, u8 value);
#endif
