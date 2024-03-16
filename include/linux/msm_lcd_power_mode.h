/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef LCD_POWER_H
#define LCD_POWER_H

enum { /* This enum is for power control between display-touch driver*/
	DEEP_SLEEP_ENTER = 0, /* For entering deep sleep sequence */
	DEEP_SLEEP_EXIT, /* For exiting deep sleep sequence */
	DSV_TOGGLE, /* For setting DSV Toggle mode */
	DSV_ALWAYS_ON, /* For setting DSV Always on mode */
};

void lge_panel_set_power_mode(int mode);

#endif //End of LCD_POWER_H
