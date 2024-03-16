/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 */

#include <linux/thermal.h>

static const struct virtual_sensor_data sub6_lge_virtual_sensors[] = {
    /* vts = 0.12*xo_therm + 0.58*quiet_therm +8.39*/
    {
		.virt_zone_name = "vts-virt-therm",
		.num_sensors = 2,
		.sensor_names = {"xo-therm-usr",
				"quiet-therm-usr"},
		.coefficient_ct = 2,
		.coefficients = {12, 58},
		.avg_offset = 839000,
		.avg_denominator = 100,
		.logic = VIRT_WEIGHTED_AVG,
	},
};

static const struct virtual_sensor_data mmw_lge_virtual_sensors[] = {
	/* vts = 0*xo_therm + 1*quiet_therm */
	{
		.virt_zone_name = "vts-virt-therm",
		.num_sensors = 2,
		.sensor_names = {"xo-therm-usr",
				"quiet-therm-usr"},
		.coefficient_ct = 2,
		.coefficients = {0, 1},
		.avg_offset = 0,
		.avg_denominator = 1,
		.logic = VIRT_WEIGHTED_AVG,
	},
};
