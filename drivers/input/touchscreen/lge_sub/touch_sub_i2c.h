/*
 * touch_sub_i2c.h
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef TOUCH_I2C_H
#define TOUCH_I2C_H

extern int touch_sub_i2c_read(struct i2c_client *client, struct touch_sub_bus_msg *msg);
extern int touch_sub_i2c_write(struct i2c_client *client, struct touch_sub_bus_msg *msg);
extern int touch_sub_i2c_device_init(struct touch_sub_hwif *hwif, void *driver);
extern void touch_sub_i2c_device_exit(struct touch_sub_hwif *hwif);

#if defined(CONFIG_SUB_SECURE_TOUCH)
extern void touch_sub_i2c_set(struct touch_sub_core_data *ts);
extern int touch_sub_i2c_get(struct touch_sub_core_data *ts);
#endif

#endif
