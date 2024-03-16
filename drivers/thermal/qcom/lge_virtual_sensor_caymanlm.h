/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 */

#include <linux/thermal.h>

static const struct virtual_sensor_data sub6_lge_virtual_sensors[] = {
    /* vts = -0.164*xo_therm + 0.971*quiet_therm + 5.74 */
    {
        .virt_zone_name = "vts-virt-therm",
        .num_sensors = 2,
        .sensor_names = {"xo-therm-usr",
                "quiet-therm-usr"},
        .coefficient_ct = 2,
        .coefficients = {-164, 971},
        .avg_offset = 5740000,
        .avg_denominator = 1000,
        .logic = VIRT_WEIGHTED_AVG,
	},
};

static const struct virtual_sensor_data mmw_lge_virtual_sensors[] = {
	/* vts = -0.663*xo_therm + 1.519*quiet_therm + 4.034 */
	{
		.virt_zone_name = "vts-virt-therm",
		.num_sensors = 2,
		.sensor_names = {"xo-therm-usr",
				"quiet-therm-usr"},
		.coefficient_ct = 2,
		.coefficients = {-663, 1519},
		.avg_offset = 4034000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},

	/* qtm0(east) : -1.386*xo_therm + 0.497*quiet_therm + 1.734*qtm_e_therm + 5.372 */
	{
		.virt_zone_name = "qtm-0-vts-therm",
		.num_sensors = 3,
		.sensor_names = {"xo-therm-usr", "quiet-therm-usr", "qtm-e-therm-usr"},
		.coefficient_ct = 3,
		.coefficients = {-1386, 497, 1734},
		.avg_offset = 5372000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},
	/* qtm1(west) : -1.14*xo_therm + 2.005*quiet_therm – 0.023*qtm_w_therm + 5.893 */
	{
		.virt_zone_name = "qtm-1-vts-therm",
		.num_sensors = 3,
		.sensor_names = {"xo-therm-usr", "quiet-therm-usr", "qtm-w-therm-usr"},
		.coefficient_ct = 3,
		.coefficients = {-1140, 2005, -23},
		.avg_offset = 5893000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},
	/* qtm2(north) : -1.861*xo_therm + 1.705*quiet_therm + 0.97*qtm_n_therm + 7.179 */
	{
		.virt_zone_name = "qtm-2-vts-therm",
		.num_sensors = 3,
		.sensor_names = {"xo-therm-usr", "quiet-therm-usr", "qtm-n-therm-usr"},
		.coefficient_ct = 3,
		.coefficients = {-1861, 1705, 970},
		.avg_offset = 7179000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},
	// Not used
	/* qtm-modem(smr) : -1.24*quiet-therm + 2.0*qtm-e-therm + 7.38 */
	{
		.virt_zone_name = "qtm-modem-vts-therm",
		.num_sensors = 2,
		.sensor_names = {"quiet-therm-usr", "qtm-e-therm-usr"},
		.coefficient_ct = 2,
		.coefficients = {-124, 200},
		.avg_offset = 738000,
		.avg_denominator = 100,
		.logic = VIRT_WEIGHTED_AVG,
	},
};
