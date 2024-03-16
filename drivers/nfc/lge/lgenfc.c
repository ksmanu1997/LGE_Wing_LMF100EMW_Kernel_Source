/*
 * drivers/soc/qcom/lge/devices_lge.c
 *
 * Copyright (C) 2020 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/nfc/lgenfc.h>
#include <linux/printk.h>
#ifdef CONFIG_LGE_ONE_BINARY_SKU
#include <stdbool.h>
#include <soc/qcom/lge/board_lge.h>
#include <linux/nfc/lgenfc_matchinglist.h>

enum NfcChip getChipset() {
  enum lge_sku_carrier_type sku = lge_get_sku_carrier();
  int len = mMatchingListSize;
  int cnt = 0;
  enum NfcChip chipset = mMatchingDefault;

  for (cnt = 0; cnt < len; ++cnt) {
    if (mMatchingList[cnt].sku == sku) {
      chipset = mMatchingList[cnt].chip;
      break;
    }
  }

  return chipset;
}

int isDriverAvailable(enum NfcChip target) {
  int res = (target == getChipset());
  pr_info("%s sku:%d NfcChip:%d avail:%d", __func__, lge_get_sku_carrier(), target, res);
  return res;
}
#else // CONFIG_LGE_ONE_BINARY_SKU
int isDriverAvailable(enum NfcChip target) {
  return true;
}
#endif // CONFIG_LGE_ONE_BINARY_SKU