// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 */

#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include "lge_virtual_sensor.h"

#if defined(CONFIG_MACH_LITO_CAYMANLM)
#include "lge_virtual_sensor_caymanlm.h"
#elif defined(CONFIG_MACH_LITO_ACELM)
#include "lge_virtual_sensor_acelm.h"
#elif defined(CONFIG_MACH_LITO_WINGLM)
#include "lge_virtual_sensor_winglm.h"
#elif defined(CONFIG_MACH_LAGOON_ACEXLM)
#include "lge_virtual_sensor_acexlm.h"
#elif defined(CONFIG_MACH_LAGOON_SMASHJLM)
#include "lge_virtual_sensor_smashjlm.h"
#else
#include "lge_virtual_sensor_default.h"
#endif

#ifdef CONFIG_LGE_ONE_BINARY_SKU
#include <soc/qcom/lge/board_lge.h>
#endif

int lge_virtual_sensor_register(struct device *dev)
{
	int sens_ct = 0;
	static int idx;
	struct thermal_zone_device *tz;

#ifdef CONFIG_LGE_ONE_BINARY_SKU
	enum lge_sku_carrier_type sku_carrier = HW_SKU_MAX;
	sku_carrier = lge_get_sku_carrier();

	pr_info("operator is %s\n", sku_carrier == HW_SKU_NA_CDMA_VZW ? "VZW" : "Non-VZW");

	if (sku_carrier == HW_SKU_NA_CDMA_VZW){
		sens_ct = ARRAY_SIZE(mmw_lge_virtual_sensors);
		for (; idx < sens_ct; idx++) {
			tz = devm_thermal_of_virtual_sensor_register(dev,
					&mmw_lge_virtual_sensors[idx]);
			if (IS_ERR(tz))
				pr_err("%s: sensor:%s register error:%ld\n", __func__,
						mmw_lge_virtual_sensors[idx].virt_zone_name, PTR_ERR(tz));
			else
				pr_err("%s: sensor:%s register success\n", __func__,
					mmw_lge_virtual_sensors[idx].virt_zone_name);
		}
	} else {
		sens_ct = ARRAY_SIZE(sub6_lge_virtual_sensors);
		for (; idx < sens_ct; idx++) {
			tz = devm_thermal_of_virtual_sensor_register(dev,
					&sub6_lge_virtual_sensors[idx]);
			if (IS_ERR(tz))
				pr_err("%s: sensor:%s register error:%ld\n", __func__,
						sub6_lge_virtual_sensors[idx].virt_zone_name, PTR_ERR(tz));
			else
				pr_err("%s: sensor:%s register success\n", __func__,
						sub6_lge_virtual_sensors[idx].virt_zone_name);
		}
	}
#else
	sens_ct = ARRAY_SIZE(sub6_lge_virtual_sensors);
	for (; idx < sens_ct; idx++) {
		tz = devm_thermal_of_virtual_sensor_register(dev,
				&sub6_lge_virtual_sensors[idx]);
		if (IS_ERR(tz))
			pr_err("%s: sensor:%s register error:%ld\n", __func__,
					sub6_lge_virtual_sensors[idx].virt_zone_name, PTR_ERR(tz));
		else
			pr_err("%s: sensor:%s register success\n", __func__,
				sub6_lge_virtual_sensors[idx].virt_zone_name);
	}
#endif	//CONFIG_LGE_ONE_BINARY_SKU

	return 0;
}
