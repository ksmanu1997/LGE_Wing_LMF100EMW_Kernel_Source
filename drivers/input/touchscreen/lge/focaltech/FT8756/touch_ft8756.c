/* Touch_ft8756.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hyokmin.kwon@lge.com
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
#define TS_MODULE "[ft8756]"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>


/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>

/*
 *  Include to Local Header File
 */
#include "touch_ft8756.h"
#include "touch_ft8756_prd.h"


// Definitions for Debugging Failure Reason in LPWG
enum {
	TCI_DEBUG_DISABLE = 0,
	TCI_DEBUG_ALWAYS,
	TCI_DEBUG_BUFFER,
	TCI_DEBUG_BUFFER_ALWAYS,
};

static const char *tci_debug_type_str[] = {
	"Disable Type",
	"Always Report Type",
	"Buffer Type",
	"Buffer and Always Report Type"
};

#define TCI_FR_BUF_LEN	10
#define TCI_FR_NUM		7

static const char const *tci_debug_str[TCI_FR_NUM + 1] = {
	"NONE",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP",
	"MULTI_FINGER",
	"DELAY_TIME", /* It means Over Tap */
	"PALM_STATE",
	"OUT_OF_AREA"
};

#define SWIPE_FAILREASON_BUF_LEN    10
#define SWIPE_FAILREASON_DIR_NUM    4
static const char const *swipe_debug_direction_str[SWIPE_FAILREASON_DIR_NUM + 1] = {
	[0] = "SWIPE_UP",
	[1] = "SWIPE_DOWN",
	[2] = "SWIPE_LEFT2",
	[3] = "SWIPE_RIGHT2",
	[4] = "Reserved" // Invalid data
};
#define SWIPE_FAILREASON_NUM        12
static const char const *swipe_debug_str[SWIPE_FAILREASON_NUM + 1] = {
	[0] = "Reserved",
	[1] = "NORMAL_ERROR",
	[2] = "FINGER_FAST_RELEASE",
	[3] = "MULTI_FINGER",
	[4] = "FAST_SWIPE",
	[5] = "SLOW_SWIPE",
	[6] = "WRONG_DIRECTION",
	[7] = "RATIO_FAIL",
	[8] = "OUT_OF_START_AREA",
	[9] = "OUT_OF_ACTIVE_AREA",
	[10] = "INITAIL_RATIO_FAIL",
	[11] = "PALM_STATE",
	[12] = "Reserved" // Invalid data
};

/* touch irq handle according to display suspend in mfts */
bool mfts_check_shutdown = false;

#if defined (CONFIG_LGE_TOUCH_MODULE_DETECT)
int ft8756_panel_type;
#endif /* CONFIG_LGE_TOUCH_MODULE_DETECT */

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
extern void lge_mdss_report_panel_dead(void);
#endif
#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
extern void mtkfb_esd_recovery(void);
#endif

#ifdef FT8756_ESD_SKIP_WHILE_TOUCH_ON
static int finger_cnt = 0;

bool ft8756_check_finger(void)
{
	return finger_cnt==0? false:true;
}
EXPORT_SYMBOL(ft8756_check_finger);
#endif

int major_value = 0;
int minor_value = 0;

int ft8756_reg_read(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	struct touch_bus_msg msg = {0, };
	int ret = 0;

#if defined(CONFIG_SECURE_TOUCH)
		if (atomic_read(&ts->st_enabled)) {
			TOUCH_E("cannot use i2c, ownership changed!\n");
			return ret;
		}
#endif

	mutex_lock(&d->rw_lock);

	ts->tx_buf[0] = addr;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], size);
	mutex_unlock(&d->rw_lock);
	return 0;

}

int ft8756_reg_write(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	struct touch_bus_msg msg = {0, };
	int ret = 0;

#if defined(CONFIG_SECURE_TOUCH)
	if (atomic_read(&ts->st_enabled)) {
		TOUCH_E("cannot use i2c, ownership changed!\n");
		return ret;
	}
#endif

	mutex_lock(&d->rw_lock);

	ts->tx_buf[0] = addr;
	memcpy(&ts->tx_buf[1], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size + 1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	mutex_unlock(&d->rw_lock);

	return 0;
}

int ft8756_cmd_read(struct device *dev, void *cmd_data, int cmd_len, void *read_buf, int read_len)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	struct touch_bus_msg msg = {0, };
	int ret = 0;

	mutex_lock(&d->rw_lock);

	memcpy(&ts->tx_buf[0], cmd_data, cmd_len);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = cmd_len;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = read_len;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	memcpy(read_buf, &ts->rx_buf[0], read_len);
	mutex_unlock(&d->rw_lock);
	return 0;

}

int ft8756_dummy_read(struct device *dev)
{
	u8 dummy = 0x00;
	int ret = 0;

	TOUCH_TRACE();

	//In order to watch buffer failreason when turning on the screen with power key, we have to read any register twice for dummy
	ret = ft8756_reg_read(dev, 0x00, &dummy, 1);
	if (ret < 0)
		TOUCH_I("check or wakeup touch ic, bus error is normal operation\n");
	touch_msleep(40);
	return ret;
}

static int ft8756_reset_ctrl(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u8 data = 0;

	TOUCH_TRACE();

	switch (ctrl) {
	case SW_RESET:
		TOUCH_I("%s : SW Reset\n", __func__);
		data = 0x55;
		ft8756_reg_write(dev, 0xFC, &data, 1);
		touch_msleep(10);
		data = 0x66;
		ft8756_reg_write(dev, 0xFC, &data, 1);
		touch_msleep(ts->caps.sw_reset_delay);
		break;
	case HW_RESET:
		TOUCH_I("%s : HW Reset\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(5);
		touch_gpio_direction_output(ts->reset_pin, 1);
		queue_delayed_work(ts->wq, &ts->init_work,
			msecs_to_jiffies(ts->caps.hw_reset_delay));
		break;
	default:
		TOUCH_I("%s, Unknown reset ctrl!!!!\n", __func__);
		break;
	}

	return 0;
}

static int ft8756_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
//	struct ft8756_data *d = to_ft8756_data(dev);
	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
#if defined(CONFIG_SECURE_TOUCH)
		if (atomic_read(&ts->st_enabled))
			secure_touch_stop(ts, true);
#endif
		TOUCH_I("%s, off\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(5);
		break;
	case POWER_ON:
		TOUCH_I("%s, on\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(5);
		break;
	case POWER_SLEEP:
		TOUCH_I("%s, sleep\n", __func__);
		break;
	case POWER_WAKE:
		TOUCH_I("%s, wake\n", __func__);
		break;
	case POWER_HW_RESET_ASYNC:
		TOUCH_I("%s, HW Reset(%d)\n", __func__, ctrl);
		ft8756_reset_ctrl(dev, HW_RESET);
		break;
	case POWER_HW_RESET_SYNC:
		TOUCH_I("%s, HW Reset(%d)\n", __func__, ctrl);
		ft8756_reset_ctrl(dev, HW_RESET);
		break;
	case POWER_SW_RESET:
	    TOUCH_I("%s, SW Reset\n", __func__);
		ft8756_reset_ctrl(dev, SW_RESET);
		break;
	default:
		TOUCH_I("%s, Unknown Power Ctrl!!!!\n", __func__);
		break;
	}

	return 0;
}

static void ft8756_get_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 0;
	ts->tci.info[TCI_1].max_intertap = 50;
	ts->tci.info[TCI_1].touch_slop = 10;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].min_intertap = 0;
	ts->tci.info[TCI_2].max_intertap = 70;
	ts->tci.info[TCI_2].touch_slop = 10;
	ts->tci.info[TCI_2].tap_distance = 255;
	ts->tci.info[TCI_2].intr_delay = 20;
}

static void ft8756_get_swipe_info(struct device *dev)
{

	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	float mm_to_point = 15.5; // 1 mm -> about X point

	TOUCH_TRACE();

	ts->swipe[SWIPE_L2].enable = false; // true modify for test
	ts->swipe[SWIPE_L2].distance = 12;
	ts->swipe[SWIPE_L2].ratio_thres = 150;
	ts->swipe[SWIPE_L2].min_time = 4;
	ts->swipe[SWIPE_L2].max_time = 150;
	ts->swipe[SWIPE_L2].wrong_dir_thres = 5;
	ts->swipe[SWIPE_L2].init_ratio_chk_dist = 4;
	ts->swipe[SWIPE_L2].init_ratio_thres = 100;
	ts->swipe[SWIPE_L2].area.x1 = 0;									// LG Pay UI width/height
	ts->swipe[SWIPE_L2].area.y1 = 0;
	ts->swipe[SWIPE_L2].area.x2 = (ts->caps.max_x);
	ts->swipe[SWIPE_L2].area.y2 = (ts->caps.max_y);
	ts->swipe[SWIPE_L2].start_area.x1 = ((ts->caps.max_x) - (int)(mm_to_point * 10)); 		// spec 10mm
	ts->swipe[SWIPE_L2].start_area.y1 = 204;
	ts->swipe[SWIPE_L2].start_area.x2 = (ts->caps.max_x);
	ts->swipe[SWIPE_L2].start_area.y2 = 1105;

	ts->swipe[SWIPE_R2].enable = false;
	ts->swipe[SWIPE_R2].distance = 12;
	ts->swipe[SWIPE_R2].ratio_thres = 150;
	ts->swipe[SWIPE_R2].min_time = 4;
	ts->swipe[SWIPE_R2].max_time = 150;
	ts->swipe[SWIPE_R2].wrong_dir_thres = 5;
	ts->swipe[SWIPE_R2].init_ratio_chk_dist = 4;
	ts->swipe[SWIPE_R2].init_ratio_thres = 100;
	ts->swipe[SWIPE_R2].area.x1 = 0;									// LG Pay UI width/height
	ts->swipe[SWIPE_R2].area.y1 = 0;
	ts->swipe[SWIPE_R2].area.x2 = (ts->caps.max_x);
	ts->swipe[SWIPE_R2].area.y2 = (ts->caps.max_y);
	ts->swipe[SWIPE_R2].start_area.x1 = 0;
	ts->swipe[SWIPE_R2].start_area.y1 = 204;
	ts->swipe[SWIPE_R2].start_area.x2 = (int)(mm_to_point * 10);					// spec 10mm
	ts->swipe[SWIPE_R2].start_area.y2 = 1105;

	ts->swipe[SWIPE_U].enable = false;
	ts->swipe[SWIPE_U].distance = 20;
	ts->swipe[SWIPE_U].ratio_thres = 150;
	ts->swipe[SWIPE_U].min_time = 4;
	ts->swipe[SWIPE_U].max_time = 150;
	ts->swipe[SWIPE_U].wrong_dir_thres = 5;
	ts->swipe[SWIPE_U].init_ratio_chk_dist = 4;
	ts->swipe[SWIPE_U].init_ratio_thres = 100;
	ts->swipe[SWIPE_U].area.x1 = 0 + (int)(mm_to_point * 4);					// spec 4mm
	ts->swipe[SWIPE_U].area.y1 = 0;									// spec 0mm
	ts->swipe[SWIPE_U].area.x2 = ts->caps.max_x - (int)(mm_to_point * 4);				// spec 4mm
	ts->swipe[SWIPE_U].area.y2 = ts->caps.max_y;							// spec 0mm
	ts->swipe[SWIPE_U].start_area.x1 = (ts->caps.max_x / 2) - (int)(mm_to_point * 12.5);		// spec start_area_width 25mm
	ts->swipe[SWIPE_U].start_area.y1 = ts->swipe[SWIPE_U].area.y2 - (int)(mm_to_point * 14.5);	// spec start_area_height 14.5mm
	ts->swipe[SWIPE_U].start_area.x2 = (ts->caps.max_x / 2) + (int)(mm_to_point * 12.5);		// spec start_area_width 25mm
	ts->swipe[SWIPE_U].start_area.y2 = ts->swipe[SWIPE_U].area.y2;

	ts->swipe[SWIPE_D].enable = false;
	ts->swipe[SWIPE_D].distance = 15;
	ts->swipe[SWIPE_D].ratio_thres = 150;
	ts->swipe[SWIPE_D].min_time = 0;
	ts->swipe[SWIPE_D].max_time = 150;
	ts->swipe[SWIPE_D].wrong_dir_thres = 5;
	ts->swipe[SWIPE_D].init_ratio_chk_dist = 5;
	ts->swipe[SWIPE_D].init_ratio_thres = 100;
	ts->swipe[SWIPE_D].area.x1 = 0 + (int)(mm_to_point * 4);					// spec 4mm
	ts->swipe[SWIPE_D].area.y1 = 0;									// spec 0mm
	ts->swipe[SWIPE_D].area.x2 = ts->caps.max_x - (int)(mm_to_point * 4);				// spec 4mm
	ts->swipe[SWIPE_D].area.y2 = ts->caps.max_y;							// spec 0mm
	ts->swipe[SWIPE_D].start_area.x1 = 0 + (int)(mm_to_point * 4);					// spec 4mm
	ts->swipe[SWIPE_D].start_area.y1 = 0;								// spec 0mm
	ts->swipe[SWIPE_D].start_area.x2 = ts->caps.max_x - (int)(mm_to_point * 4);			// spec 4mm
	ts->swipe[SWIPE_D].start_area.y2 = 200;

	d->start_pay_area_L2.x1 = ts->swipe[SWIPE_L2].start_area.x1;
	d->start_pay_area_L2.x2 = ts->swipe[SWIPE_L2].start_area.x2;
	d->start_pay_area_L2.y1 = ts->swipe[SWIPE_L2].start_area.y1;
	d->start_pay_area_L2.y2 = ts->swipe[SWIPE_L2].start_area.y2;

	d->start_pay_area_R2.x1 = ts->swipe[SWIPE_R2].start_area.x1;
	d->start_pay_area_R2.x2 = ts->swipe[SWIPE_R2].start_area.x2;
	d->start_pay_area_R2.y1 = ts->swipe[SWIPE_R2].start_area.y1;
	d->start_pay_area_R2.y2 = ts->swipe[SWIPE_R2].start_area.y2;

}

static struct ft8756_ic_info *ext_ic_info;

int ft8756_ic_info(struct device *dev)
{
//	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0;
	u8 chip_id = 0;
	u8 chip_id_low = 0;
	u8 is_official = 0;
	u8 fw_version = 0;
	u8 fw_version_minor = 0;
	u8 fw_version_sub_minor = 0;
	u8 fw_vendor_id = 0;
	u8 lib_version_high = 0;
	u8 lib_version_low = 0;
	int i;
//	u8 rdata = 0;

	TOUCH_TRACE();

	// In LPWG, i2c can be failed because of i2c sleep mode
	if (d->state != TC_STATE_ACTIVE && d->state != TC_STATE_LPWG) {
		TOUCH_E("Cannot get ic info in NOT ACTIVE mode\n");
		return -EPERM; // Do nothing in caller
	}

	// If it is failed to get info without error, just return error
	for (i = 0; i < 2; i++) {
		ret |= ft8756_reg_read(dev, FTS_REG_ID, (u8 *)&chip_id, 1);
		ret |= ft8756_reg_read(dev, FTS_REG_ID_LOW, (u8 *)&chip_id_low, 1);
		ret |= ft8756_reg_read(dev, FTS_REG_FW_VER, (u8 *)&fw_version, 1);
		ret |= ft8756_reg_read(dev, FTS_REG_FW_VER_MINOR, (u8 *)&fw_version_minor, 1);
		ret |= ft8756_reg_read(dev, FTS_REG_FW_VER_SUB_MINOR, (u8 *)&fw_version_sub_minor, 1);
		ret |= ft8756_reg_read(dev, FTS_REG_FW_VENDOR_ID, (u8 *)&fw_vendor_id, 1);
		ret |= ft8756_reg_read(dev, FTS_REG_LIB_VER_H, (u8 *)&lib_version_high, 1);
		ret |= ft8756_reg_read(dev, FTS_REG_LIB_VER_L, (u8 *)&lib_version_low, 1);

		if (ret == 0) {
			TOUCH_I("Success to get ic info data\n");
			break;
		}
	}

	if (i >= 2) {
		TOUCH_E("Failed to get ic info data, (need to recover it?)\n");
		return -EPERM; // Do nothing in caller
	}

	is_official = (fw_version & 0x80) >> 7;
	fw_version &= 0x7F;
	d->ic_info.version.major = fw_version;
	d->ic_info.version.minor = fw_version_minor;
	d->ic_info.version.sub_minor = fw_version_sub_minor;

	d->ic_info.chip_id = chip_id; // Device ID
	d->ic_info.chip_id_low = chip_id_low;
	d->ic_info.is_official = is_official;
	d->ic_info.fw_version = fw_version; // Major
	d->ic_info.fw_vendor_id = fw_vendor_id; // Vendor ID
	d->ic_info.lib_version_high = lib_version_high;
	d->ic_info.lib_version_low = lib_version_low;

	d->ic_info.info_valid = 1;

	ext_ic_info = &d->ic_info;

	TOUCH_I("================================================\n");
	TOUCH_I("chip_id : %x, chip_id_low : %x, is_official : %d\n", chip_id, chip_id_low, is_official);
	TOUCH_I("fw_version : %d.%d.%d, fw_vendor_id : %x,\n", fw_version, fw_version_minor, fw_version_sub_minor, fw_vendor_id);
	TOUCH_I("lib_version_high : %x, lib_version_low : %x\n", lib_version_high, lib_version_low);
	TOUCH_I("================================================\n");

	return ret;
}

int ft8756_grip_suppression_ctrl(struct device *dev, u8 x, u8 y)
{
        int ret = 0;
        u8 data = 0x0;

	TOUCH_TRACE();

        data = x; // unit: 0.1mm = 1 (0~255 : 0~25.5mm)
        ret = ft8756_reg_write(dev, CONTROL_GRIP_SUPPRESSION_X, &data, sizeof(data));
        if (ret < 0) {
                TOUCH_E("Write grip suppression x cmd error\n");
                return ret;
	}

        data = y; // unit: 1mm = 1 (0~129 : 0~129mm) - panel max y size
        ret = ft8756_reg_write(dev, CONTROL_GRIP_SUPPRESSION_Y, &data, sizeof(data));
        if (ret < 0) {
                TOUCH_E("Write grip suppression y cmd error\n");
                return ret;
	}

        TOUCH_I("%s - x: %d*0.1mm, y: %dmm\n", __func__, x, y);

        return ret;
}

static int ft8756_tci_control(struct device *dev, int type)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	struct tci_info *info2 = &ts->tci.info[TCI_2];
	u8 data;
	int ret = 0;

	TOUCH_TRACE();

	switch (type) {
	case TCI_CTRL_SET:
		data = ts->tci.mode;
		ret = ft8756_reg_write(dev, 0xD0, &data, 1);
		break;

	case TCI_CTRL_CONFIG_COMMON:
		data = (((d->tci_debug_type) & 0x02) << 3) | ((d->tci_debug_type) & 0x01);
		ret = ft8756_reg_write(dev, 0xE5, &data, 1);	// Fail Reason Debug Function Enable

		//Resolution 1080x2400 ACE

		data = 0x28;
		ret |= ft8756_reg_write(dev, 0x92, &data, 1);	// Active Area LSB of X1  // MH4 CPT: FT8756 0x92
		data = 0x00;
		ret |= ft8756_reg_write(dev, 0xB5, &data, 1);	// Active Area MSB of X1 (40)
		data = 0x10;
		ret |= ft8756_reg_write(dev, 0xB6, &data, 1);	// Active Area LSB of X2
		data = 0x04;
		ret |= ft8756_reg_write(dev, 0xB7, &data, 1);	// Active Area MSB of X2 (1040)
		data = 0x28;
		ret |= ft8756_reg_write(dev, 0xB8, &data, 1);	// Active Area LSB of Y1
		data = 0x00;
		ret |= ft8756_reg_write(dev, 0xB9, &data, 1);	// Active Area MSB of Y1 (40)
		data = 0x38;
		ret |= ft8756_reg_write(dev, 0xBA, &data, 1);	// Active Area LSB of Y2
		data = 0x09;
		ret |= ft8756_reg_write(dev, 0xBB, &data, 1);	// Active Area MSB of Y2 (2360)
		break;

	case TCI_CTRL_CONFIG_TCI_1:
		data = info1->touch_slop;
		ret = ft8756_reg_write(dev, 0xBC, &data, 1);	// Touch Slop (10mm)
		data = info1->tap_distance;
		ret |= ft8756_reg_write(dev, 0xC4, &data, 1);	// Touch Distance (10mm)
		data = info1->max_intertap;
		ret |= ft8756_reg_write(dev, 0xC6, &data, 1);	// Time Gap Max (700ms)
		data = info1->tap_count;
		ret |= ft8756_reg_write(dev, 0xCA, &data, 1);	// Total Tap Count (2)
		data = info1->intr_delay;
		ret |= ft8756_reg_write(dev, 0xCC, &data, 1);	// Interrupt Delay (700ms or 0ms)
		break;

	case TCI_CTRL_CONFIG_TCI_2:
		data = info2->touch_slop;
		ret = ft8756_reg_write(dev, 0xBD, &data, 1);	// Touch Slop (10mm)
		data = info2->tap_distance;
		ret |= ft8756_reg_write(dev, 0xC5, &data, 1);	// Touch Distance (?? 200mm ??)
		data = info2->max_intertap;
		ret |= ft8756_reg_write(dev, 0xC7, &data, 1);	// Time Gap Max (700ms)
		data = info2->tap_count;
		ret |= ft8756_reg_write(dev, 0xCB, &data, 1);	// Total Tap Count (2)
		data = info2->intr_delay;
		ret |= ft8756_reg_write(dev, 0xCD, &data, 1);	// Interrupt Delay (370ms ??)
		break;

	default:
		break;
	}

	return ret;
}

static int ft8756_swipe_control(struct device *dev, u8 idx, int *swipe_mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	u8 data = 0;
	int ret = 0;

	TOUCH_TRACE();

	switch (idx) {
		case SWIPE_D:
			*swipe_mode |= 0x1 << 3;
			data = 0x1;
			ret = ft8756_reg_write(dev, 0x60, &data, 1);	// Select BankSel (which swipe config: 0x00: swipe_up / 0x01: swipe_down / 0x02: swipe_left2 / 0x03: swipe_right2)
			break;
		case SWIPE_U:
			*swipe_mode |= 0x1 << 2;
			data = 0x0;
			ret = ft8756_reg_write(dev, 0x60, &data, 1);
			break;
		case SWIPE_L2:
			*swipe_mode |= 0x1 << 4;
			data = 0x2;
			ret = ft8756_reg_write(dev, 0x60, &data, 1);
			break;
		case SWIPE_R2:
			*swipe_mode |= 0x1 << 5;
			data = 0x3;
			ret = ft8756_reg_write(dev, 0x60, &data, 1);
			break;
		default:
			TOUCH_E("Not supported swipe index : %d\n", idx);
			return -1;
	}

	data = (((d->swipe_debug_type) & 0x02) << 3) | ((d->swipe_debug_type) & 0x01);
	ret |= ft8756_reg_write(dev, 0x95, &data, 1);	// Fail Reason Debug Function Enable

	data = ts->swipe[idx].distance;
	ret |= ft8756_reg_write(dev, 0x61, &data, 1);	// Distance
	data = ts->swipe[idx].ratio_thres;
	ret |= ft8756_reg_write(dev, 0x62, &data, 1);	// Ratio
	data = (ts->swipe[idx].min_time & 0xFF00) >> 8;
	ret |= ft8756_reg_write(dev, 0x63, &data, 1);	// Min time HighBit
	data = ts->swipe[idx].min_time & 0x00FF;
	ret |= ft8756_reg_write(dev, 0x64, &data, 1);	// Min time LowBit
	data = (ts->swipe[idx].max_time & 0xFF00) >> 8;
	ret |= ft8756_reg_write(dev, 0x65, &data, 1);	// Max time HighBit
	data = ts->swipe[idx].max_time & 0x00FF;
	ret |= ft8756_reg_write(dev, 0x66, &data, 1);	// Max time LowBit
	data = ts->swipe[idx].wrong_dir_thres;
	ret |= ft8756_reg_write(dev, 0x67, &data, 1);	// Wrong Direction

	data = (ts->swipe[idx].area.x1 & 0xFF00) >> 8;
	ret |= ft8756_reg_write(dev, 0x68, &data, 1);	// Active Area x1 HighBit
	data = ts->swipe[idx].area.x1 & 0x00FF;
	ret |= ft8756_reg_write(dev, 0x69, &data, 1);	// Active Area x1 LowBit
	data = (ts->swipe[idx].area.y1 & 0xFF00) >> 8;
	ret |= ft8756_reg_write(dev, 0x6A, &data, 1);	// Active Area y1 HighBit
	data = ts->swipe[idx].area.y1 & 0x00FF;
	ret |= ft8756_reg_write(dev, 0x6B, &data, 1);	// Active Area y1 LowBit

	data = (ts->swipe[idx].area.x2 & 0xFF00) >> 8;
	ret |= ft8756_reg_write(dev, 0x6C, &data, 1);	// Active Area x2 HighBit
	data = ts->swipe[idx].area.x2 & 0x00FF;
	ret |= ft8756_reg_write(dev, 0x6D, &data, 1);	// Active Area x2 LowBit
	data = (ts->swipe[idx].area.y2 & 0xFF00) >> 8;
	ret |= ft8756_reg_write(dev, 0x6E, &data, 1);	// Active Area y2 HighBit
	data = ts->swipe[idx].area.y2 & 0x00FF;
	ret |= ft8756_reg_write(dev, 0x6F, &data, 1);	// Active Area y2 LowBit

	data = (ts->swipe[idx].start_area.x1 & 0xFF00) >> 8;
	ret |= ft8756_reg_write(dev, 0x70, &data, 1);	// Active Start Area x1 HighBit
	data = ts->swipe[idx].start_area.x1 & 0x00FF;
	ret |= ft8756_reg_write(dev, 0x71, &data, 1);	// Active Start Area x1 LowBit
	data = (ts->swipe[idx].start_area.y1 & 0xFF00) >> 8;
	ret |= ft8756_reg_write(dev, 0x72, &data, 1);	// Active Start Area y1 HighBit
	data = ts->swipe[idx].start_area.y1 & 0x00FF;
	ret |= ft8756_reg_write(dev, 0x73, &data, 1);	// Active Start Area y1 LowBit

	data = (ts->swipe[idx].start_area.x2 & 0xFF00) >> 8;
	ret |= ft8756_reg_write(dev, 0x74, &data, 1);	// Active Start Area x2 HighBit
	data = ts->swipe[idx].start_area.x2 & 0x00FF;
	ret |= ft8756_reg_write(dev, 0x75, &data, 1);	// Active Start Area x2 LowBit
	data = (ts->swipe[idx].start_area.y2 & 0xFF00) >> 8;
	ret |= ft8756_reg_write(dev, 0x76, &data, 1);	// Active Start Area y2 HighBit
	data = ts->swipe[idx].start_area.y2 & 0x00FF;
	ret |= ft8756_reg_write(dev, 0x77, &data, 1);	// Active Start Area y2 LowBit

	data = ts->swipe[idx].init_ratio_chk_dist;
	ret |= ft8756_reg_write(dev, 0x78, &data, 1);	// Initial Ratio Check Distance
	data = ts->swipe[idx].init_ratio_thres;
	ret |= ft8756_reg_write(dev, 0x79, &data, 1);	// Initial Ratio Threshold

	return ret;
}

static int ft8756_lpwg_control(struct device *dev, u8 mode, bool swipe_enable)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	//struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0;
	int swipe_mode = 0;
	u8 data = 0;
	int i = 0;

	TOUCH_TRACE();

	switch (mode) {
	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		info1->intr_delay = 0;

		ret = ft8756_tci_control(dev, TCI_CTRL_CONFIG_TCI_1);
		ret |= ft8756_tci_control(dev, TCI_CTRL_CONFIG_COMMON);
//		ret |= ft8756_tci_control(dev, TCI_CTRL_SET);

		break;

#ifdef KNOCK_CODE
	case LPWG_PASSWORD:
		ts->tci.mode = 0x03;
		info1->intr_delay = ts->tci.double_tap_check ? 70 : 0;

		ret = ft8756_tci_control(dev, TCI_CTRL_CONFIG_TCI_1);
		ret |= ft8756_tci_control(dev, TCI_CTRL_CONFIG_TCI_2);
		ret |= ft8756_tci_control(dev, TCI_CTRL_CONFIG_COMMON);
//		ret |= ft8756_tci_control(dev, TCI_CTRL_SET);

		break;

	case LPWG_PASSWORD_ONLY:
		ts->tci.mode = 0x02;
		info1->intr_delay = 0;

//		ret = ft8756_tci_control(dev, TCI_CTRL_CONFIG_TCI_1);
		ret |= ft8756_tci_control(dev, TCI_CTRL_CONFIG_TCI_2);
		ret |= ft8756_tci_control(dev, TCI_CTRL_CONFIG_COMMON);
//		ret |= ft8756_tci_control(dev, TCI_CTRL_SET);

		break;
#endif
	default:
		TOUCH_I("%s: invalid setting\n", __func__);
		break;
	}

	if (swipe_enable) {
		for (i = 0; i < sizeof(ts->swipe) / sizeof(struct swipe_ctrl); i++) {
			if (ts->swipe[i].enable) {
				ret |= ft8756_swipe_control(dev, i, &swipe_mode);
			}
		}
	} else { //swipe_disable
		swipe_mode = 0;
	}


	// Setting Enable knock on/swipe Register
	data = ts->tci.mode | swipe_mode;
	ret |= ft8756_reg_write(dev, 0xD0, &data, 1);

	TOUCH_I("ft8756_lpwg_control tci = %d, swipe = %d\n", ts->tci.mode, swipe_mode);
	return ret;
}


static int ft8756_deep_sleep(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u8 data = 0;
	int ret = 0;
	int i = 0;

	TOUCH_TRACE();

	TOUCH_I("ft8756_deep_sleep = %d, status = %d\n", mode, atomic_read(&ts->state.sleep));

	if(mode) {
		if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
			return 0;
		data = 0x03;
		atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
		ft8756_dummy_read(dev);
		for (i = 0; i < 3; i++) {
			ret = ft8756_reg_write(dev, 0xA5, &data, 1);
			if (ret == 0)
				break;
			else
				TOUCH_I("retry deep sleep command, %d\n", i + 1);
		}
		return ret;
	}
	else {
		if (atomic_read(&ts->state.sleep) == IC_NORMAL)
			return 0;
		// Do something
		atomic_set(&ts->state.sleep, IC_NORMAL);
		return 0;
	}
}

static int ft8756_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	u8 next_state;
	int ret = 0;
	int mfts_mode = 0;

	if (atomic_read(&d->init) == IC_INIT_NEED) {
		TOUCH_I("Not Ready, Need IC init\n");
		return 0;
	}

#if defined(CONFIG_SECURE_TOUCH)
	if (atomic_read(&ts->st_enabled)) {
		TOUCH_E("Secure Session enabled, don't set anything\n");
		return ret;
	}
#endif

	// Check MFTS mode to use POWER_OFF state
	mfts_mode = touch_check_boot_mode(dev);
	if ((mfts_mode >= TOUCH_MINIOS_MFTS_FOLDER && mfts_mode <= TOUCH_MINIOS_MFTS_CURVED) && !ts->mfts_lpwg) {
		TOUCH_I("MINIOS_MFTS [%d]\n", mfts_mode);
		if (d->state == TC_STATE_ACTIVE && ts->lpwg.screen == 0) { // Int disable, Touch/LCD Reset 0, DSV Off, VDDI Off
			next_state = TC_STATE_POWER_OFF;
			TOUCH_I("STATE_ACTIVE to STATE_POWER OFF in MINIOS_MFTS\n");
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			touch_gpio_direction_output(ts->reset_pin, 0);
		} else if (d->state == TC_STATE_POWER_OFF && ts->lpwg.screen == 1) { // VDDI On, 1ms, DSV On, Touch/LCD Reset 1, Int enable
			next_state = TC_STATE_ACTIVE;
			TOUCH_I("STATE_POWER_OFF to STATE_ACTIVE in MINIOS_MFTS\n");
			touch_gpio_direction_output(ts->reset_pin, 1);
			touch_msleep(105); // lmh add

			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			touch_msleep(300); // ??????? Check with LCD on delay
		} else {
			next_state = d->state;
		}
		goto RET;
	}

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if ((ts->mfts_lpwg) && (d->state == TC_STATE_ACTIVE)) {
			next_state = TC_STATE_LPWG;
			TOUCH_I("STATE_ACTIVE to STATE_LPWG (MiniOS/MFTS)\n");

			ft8756_lpwg_control(dev, LPWG_DOUBLE_TAP, false);
			goto RET;
		}
	}

	// NORMAL Case
	if (d->state == TC_STATE_ACTIVE) {
		if (ts->lpwg.screen == 0) {
			if (ts->lpwg.sensor == PROX_NEAR) {
				next_state = TC_STATE_DEEP_SLEEP;
				TOUCH_I("STATE_ACTIVE to STATE_DEEP_SLEEP (Proxi Near)\n");

				ret = ft8756_deep_sleep(dev, 1);
			} else if(ts->lpwg.mode == LPWG_NONE
					&& !ts->swipe[SWIPE_U].enable
					&& !ts->swipe[SWIPE_L2].enable
					&& !ts->swipe[SWIPE_R2].enable) {
				next_state = TC_STATE_DEEP_SLEEP;
				TOUCH_I("STATE_ACTIVE to STATE_DEEP_SLEEP (LPWG_NONE & SWIPE_NONE)\n");

				ret = ft8756_deep_sleep(dev, 1);
			} else  {
				next_state = TC_STATE_LPWG;
				TOUCH_I("STATE_ACTIVE to STATE_LPWG\n");

				ret = ft8756_lpwg_control(dev, ts->lpwg.mode,
						(ts->swipe[SWIPE_U].enable || ts->swipe[SWIPE_L2].enable || ts->swipe[SWIPE_R2].enable));
			}
		}
		else {
			next_state = TC_STATE_ACTIVE; // Do nothing
			TOUCH_I("STATE_ACTIVE to STATE_ACTIVE\n");
		}


	} else if (d->state == TC_STATE_LPWG) {
		if (ts->lpwg.screen == 0) {
			if (ts->lpwg.sensor == PROX_NEAR) {
				next_state = TC_STATE_DEEP_SLEEP; // Touch Reset, Deep Sleep, DSV Off
				TOUCH_I("STATE_LPWG to STATE_DEEP_SLEEP (Proxi Near)\n");

				ret = ft8756_deep_sleep(dev, 1);
			}
			else {
				next_state = TC_STATE_LPWG; // Do nothing
				TOUCH_I("STATE_LPWG to STATE_LPWG\n");
			}
		}
		else {
			next_state = TC_STATE_ACTIVE; // Touch Reset -> LCD Reset, SLP Out
			TOUCH_I("STATE_LPWG to STATE_ACTIVE\n");

			if(d->tci_debug_type & TCI_DEBUG_BUFFER)
				ft8756_report_tci_fr_buffer(dev); // Report fr before touch IC reset
			if(d->swipe_debug_type & TCI_DEBUG_BUFFER)
				ft8756_report_swipe_fr_buffer(dev); // Report fr before touch IC reset

			touch_gpio_direction_output(ts->reset_pin, 0);
			TOUCH_I("%s\n","STATE_ACTIVE");
			touch_msleep(5);
			touch_gpio_direction_output(ts->reset_pin, 1);
//			touch_msleep(105); //  case resume version 0.0
		}
	} else if (d->state == TC_STATE_DEEP_SLEEP) {
		if (ts->lpwg.screen == 0) {
			if(ts->lpwg.mode == LPWG_NONE
				&& !ts->swipe[SWIPE_U].enable
				&& !ts->swipe[SWIPE_L2].enable
				&& !ts->swipe[SWIPE_R2].enable) {
				next_state = TC_STATE_DEEP_SLEEP; // Do nothing
				TOUCH_I("DEEP_SLEEP to DEEP_SLEEP (LPWG_NONE & SWIPE_NONE)\n");
			}
			else if (ts->lpwg.sensor == PROX_FAR) {
				next_state = TC_STATE_LPWG;
				TOUCH_I("DEEP_SLEEP to STATE_LPWG\n");
				touch_gpio_direction_output(ts->reset_pin, 0);
				touch_msleep(5);
				touch_gpio_direction_output(ts->reset_pin, 1);
				touch_msleep(200);
				ret = ft8756_deep_sleep(dev, 0);

				ret = ft8756_lpwg_control(dev, ts->lpwg.mode,
							(ts->swipe[SWIPE_U].enable || ts->swipe[SWIPE_L2].enable || ts->swipe[SWIPE_R2].enable));
			}
			else {
				next_state = TC_STATE_DEEP_SLEEP; // Do nothing
				TOUCH_I("DEEP_SLEEP to DEEP_SLEEP\n");
			}
		} else {
			next_state = TC_STATE_ACTIVE; //  Touch DSV On, Reset
			TOUCH_I("DEEP_SLEEP to STATE_ACTIVE\n");
			touch_gpio_direction_output(ts->reset_pin, 0);

			touch_msleep(5);
			touch_gpio_direction_output(ts->reset_pin, 1);
			touch_msleep(105);

			ret = ft8756_deep_sleep(dev, 0);
			touch_msleep(15);
		}
	} else {
		next_state = d->state;
	}

RET:

	TOUCH_I("State changed from [%d] to [%d]\n", d->state, next_state);

	d->state = next_state;

	return ret;
}

static int ft8756_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int *value = (int *)param;

	TOUCH_TRACE();

	switch (code) {
	case LPWG_ACTIVE_AREA:
		ts->tci.area.x1 = value[0];
		ts->tci.area.x2 = value[1];
		ts->tci.area.y1 = value[2];
		ts->tci.area.y2 = value[3];
		TOUCH_I("LPWG_ACTIVE_AREA: x0[%d], x1[%d], x2[%d], x3[%d]\n",
			value[0], value[1], value[2], value[3]);
		break;

	case LPWG_TAP_COUNT:
		ts->tci.info[TCI_2].tap_count = value[0];
		TOUCH_I("LPWG_TAP_COUNT: [%d]\n", value[0]);
		break;

	case LPWG_DOUBLE_TAP_CHECK:
		ts->tci.double_tap_check = value[0];
		TOUCH_I("LPWG_DOUBLE_TAP_CHECK: [%d]\n", value[0]);
		break;

	case LPWG_UPDATE_ALL:
		ts->lpwg.mode = value[0];
		ts->lpwg.screen = value[1];
		ts->lpwg.sensor = value[2];
		ts->lpwg.qcover = value[3];

		TOUCH_I(
			"LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
			value[0],
			value[1] ? "ON" : "OFF",
			value[2] ? "FAR" : "NEAR",
			value[3] ? "CLOSE" : "OPEN");

		ft8756_lpwg_mode(dev);
		break;

	case LPWG_REPLY:
		break;

	}

	return 0;
}

int ft8756_lpwg_set(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);

	mutex_lock(&d->fb_lock);

	if(!ts->mfts_lpwg)
		d->state = TC_STATE_ACTIVE;

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
		ft8756_deep_sleep(dev, 0);

	ft8756_lpwg_mode(dev);

	mutex_unlock(&d->fb_lock);

	return 0;
}

static void ft8756_connect(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	int charger_state = atomic_read(&ts->state.connect);
//	int wireless_state = atomic_read(&ts->state.wireless);

	TOUCH_TRACE();

	d->charger = CONNECT_NONE;

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)

	/* wire */
	if (charger_state)
		d->charger = CONNECT_USB;

	/* wireless */
	/*
	if (wireless_state)
		d->charger |= CONNECT_WIRELESS;
	*/

	/* Distinguish just TA state or not. */
	if (d->charger)
		d->charger = 1;
	else
		d->charger = 0;
#elif defined(CONFIG_LGE_TOUCH_CORE_MTK)
	if (charger_state >= STANDARD_HOST && charger_state <= APPLE_0_5A_CHARGER) {
		d->charger = 1;
	} else {
		d->charger = 0;
	}
#endif

	/* wireless */
	/*
	if (wireless_state)
		d->charger = d->charger | CONNECT_WIRELESS;
	*/

	/* code for TA simulator */
	if (atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_0) {
		TOUCH_I("TA Simulator mode, Set CONNECT_TA\n");
		d->charger = 1;
	}

	if (ts->lpwg.screen != 0 ) {

		TOUCH_I("%s: write charger_state = 0x%02X\n", __func__, d->charger);
		if (atomic_read(&ts->state.pm) > DEV_PM_RESUME) {
			TOUCH_I("DEV_PM_SUSPEND - Don't try SPI\n");
			return;
		}

		ft8756_reg_write(dev, SPR_CHARGER_STS, &d->charger, sizeof(u8));
	}
}

static int ft8756_usb_status(struct device *dev, u32 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("TA Type: %d\n", atomic_read(&ts->state.connect));
	ft8756_connect(dev);
	return 0;
}

#if 0
static int ft8756_wireless_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Wireless charger: 0x%02X\n", atomic_read(&ts->state.wireless));
	ft8756_connect(dev);
	return 0;
}

static int ft8756_earjack_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Earjack Type: 0x%02X\n", atomic_read(&ts->state.earjack));
	return 0;
}
#endif

static int ft8756_debug_option(struct device *dev, u32 *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u32 chg_mask = data[0];
	u32 enable = data[1];
	int ret = 0;

	switch (chg_mask) {
	case DEBUG_OPTION_0: // IME
		TOUCH_I("Debug Option 0 %s\n", enable ? "Enable" : "Disable");
		break;
	case DEBUG_OPTION_1: // TA
		if (enable) {
			atomic_set(&ts->state.connect, 1);
			ret = ft8756_usb_status(dev, 1);
		} else {
			atomic_set(&ts->state.connect, 0);
			ret = ft8756_usb_status(dev, 0);
		}
		break;
	case DEBUG_OPTION_2: // debugging APK
		if (enable)
			fts_create_apk_debug_channel(dev);
		else
			fts_release_apk_debug_channel();
		break;
	default:
		TOUCH_E("Not supported debug option\n");
		break;
	}

	return ret;
}

static void ft8756_fb_notify_work_func(struct work_struct *fb_notify_work)
{
	struct ft8756_data *d =
			container_of(to_delayed_work(fb_notify_work),
				struct ft8756_data, fb_notify_work);
	int ret = 0;

	if (d->lcd_mode == LCD_MODE_U3)
		ret = FB_RESUME;
	else
		ret = FB_SUSPEND;

	touch_notifier_call_chain(NOTIFY_FB, &ret);
}

/*
static int ft8756_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *ev = (struct fb_event *)data;

	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == FB_BLANK_UNBLANK)
			TOUCH_I("FB_UNBLANK\n");
		else if (*blank == FB_BLANK_POWERDOWN)
			TOUCH_I("FB_BLANK\n");
	}

	return 0;
}
*/

static void ft8756_lcd_mode(struct device *dev, u32 mode)
{
	struct ft8756_data *d = to_ft8756_data(dev);

	d->prev_lcd_mode = d->lcd_mode;
	d->lcd_mode = mode;
	TOUCH_D(TRACE, "lcd_mode: %d (prev: %d)\n", d->lcd_mode, d->prev_lcd_mode);
}

static int ft8756_check_mode(struct device *dev)
{
	struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0;

	if (d->lcd_mode != LCD_MODE_U3) {
		if (d->lcd_mode == LCD_MODE_U2) {
			if (d->prev_lcd_mode == LCD_MODE_U2_UNBLANK) {
				TOUCH_I("U2 UNBLANK -> U2\n");
				ret = 1;
			} else {
				TOUCH_I("U2 mode change\n");
			}
		} else if (d->lcd_mode == LCD_MODE_U2_UNBLANK) {
			switch (d->prev_lcd_mode) {
			case LCD_MODE_U2:
				TOUCH_I("U2 -> U2 UNBLANK\n");
				ret = 1;
				break;
			case LCD_MODE_U0:
				TOUCH_I("U0 -> U2 UNBLANK mode change\n");
				break;
			default:
				TOUCH_I("LCD_MODE_U2_UNBLANK Mode change\n", __func__);
				break;
			}
		} else if (d->lcd_mode == LCD_MODE_U0) {
			TOUCH_I("U0 mode change\n");
		} else {
			TOUCH_I("%s - Not defined mode\n", __func__);
		}
	}

	return ret;
}

static int ft8756_notify(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	struct lge_panel_notifier *panel_data = data;
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s event=0x%x\n", __func__, (unsigned int)event);
	switch (event) {
	case NOTIFY_TOUCH_RESET:
		//temp, bringup, need to check. when init. And why not interrupt_enable.
		if (panel_data->state == LGE_PANEL_RESET_LOW) {
			TOUCH_I("NOTIFY_TOUCH_RESET_LOW!\n");
			touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
			touch_gpio_direction_output(ts->reset_pin, 0);
		} else if (panel_data->state == LGE_PANEL_RESET_HIGH) {
			TOUCH_I("NOTIFY_TOUCH_RESET_HIGH!\n");
			touch_gpio_direction_output(ts->reset_pin, 1);
			atomic_set(&d->init, IC_INIT_NEED);
		}
		break;
	case LCD_EVENT_LCD_BLANK:
		TOUCH_I("LCD_EVENT_LCD_BLANK!\n");
		break;
	case LCD_EVENT_LCD_UNBLANK:
		TOUCH_I("LCD_EVENT_LCD_UNBLANK!\n");
		break;
	case LCD_EVENT_LCD_MODE:
		TOUCH_I("LCD_EVENT_LCD_MODE!\n");
		TOUCH_I("lcd mode : %lu\n", (unsigned long)*(u32 *)data);
		ft8756_lcd_mode(dev, *(u32 *)data);
		ret = ft8756_check_mode(dev);

		if (ret == 0) {
			queue_delayed_work(ts->wq, &d->fb_notify_work, 0);
		} else {
			ret = 0;
		}
		break;
	case NOTIFY_CONNECTION:
		TOUCH_I("NOTIFY_CONNECTION!\n");
		ret = ft8756_usb_status(dev, *(u32 *)data);
		break;
	case NOTIFY_IME_STATE:
		TOUCH_I("NOTIFY_IME_STATE!\n");
#if 0
		status = atomic_read(&ts->state.ime);
		ret = ft5726_reg_write(dev, REG_IME_STATE, &status, sizeof(status));
		if (ret)
			TOUCH_E("failed to write reg_ime_state, ret : %d\n", ret);
#endif
		break;
	case NOTIFY_CALL_STATE:
		TOUCH_I("NOTIFY_CALL_STATE!\n");
#if 0
		ret = ft8756_reg_write(dev, REG_CALL_STATE,
			(u32 *)data, sizeof(u32));
#endif
		break;
	case NOTIFY_DEBUG_OPTION:
		TOUCH_I("NOTIFY_DEBUG_OPTION!\n");
		ret = ft8756_debug_option(dev, (u32 *)data);
		break;
	default:
		TOUCH_E("%lu is not supported\n", event);
		break;
	}

	return ret;
}

static int ft8756_init_works(struct ft8756_data *d)
{

	INIT_DELAYED_WORK(&d->fb_notify_work, ft8756_fb_notify_work_func);

	return 0;
}

static void ft8756_init_locks(struct ft8756_data *d)
{
	mutex_init(&d->rw_lock);
	mutex_init(&d->fb_lock);
}

static int ft8756_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = NULL;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate ft8756 data\n");
		return -ENOMEM;
	}

	d->dev = dev;
	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_gpio_init(ts->maker_id_pin, "touch_make_id");
	touch_gpio_direction_input(ts->maker_id_pin);

	touch_power_init(dev);

	touch_bus_init(dev, MAX_BUF_SIZE);

	ft8756_init_works(d);
	ft8756_init_locks(d);

	ft8756_get_tci_info(dev);
	ft8756_get_swipe_info(dev);

	if (touch_check_boot_mode(dev) == TOUCH_CHARGER_MODE) {
		touch_gpio_init(ts->reset_pin, "touch_reset");
		touch_gpio_direction_output(ts->reset_pin, 1);
		/* Deep Sleep */
		touch_msleep(100); // ???????????????????????
		ft8756_deep_sleep(dev, 1);
		return 0;
	}

	d->tci_debug_type = TCI_DEBUG_BUFFER;
	d->swipe_debug_type = TCI_DEBUG_BUFFER;
	d->noise_info.noise_log = NOISE_DISABLE;

#ifdef FTS_APK_DEBUG
	fts_create_apk_debug_channel(dev);
	atomic_set(&ts->state.debug_option_mask, DEBUG_OPTION_2);
#endif

	return 0;
}

static int ft8756_remove(struct device *dev)
{

	struct touch_core_data *ts = to_touch_core(dev);
	int debug_option_mask = 0;

	TOUCH_TRACE();

	debug_option_mask = atomic_read(&ts->state.debug_option_mask);
	if (debug_option_mask & DEBUG_OPTION_2) {
		fts_release_apk_debug_channel();
	}

	return 0;
}

static int ft8756_fwboot_upgrade(struct device *dev, const struct firmware *fw_boot)
{
	struct touch_core_data *ts = to_touch_core(dev);
	const u8 *fw_data = fw_boot->data;
	u32 fw_size = (u32)(fw_boot->size);
	u8 *fw_check_buf = NULL;
	u8 i2c_buf[FTS_PACKET_LENGTH + 12] = {0,};
	int ret;
	int packet_num, i, j, packet_addr, packet_len;
	u8 pramboot_ecc;

	TOUCH_I("%s - START\n", __func__);

	if(fw_size > 0x10000 || fw_size == 0)
		return -EIO;

	fw_check_buf = kmalloc(fw_size+1, GFP_ATOMIC);
	if(fw_check_buf == NULL)
		return -ENOMEM;

	for (i = 12; i <= 30; i++) {
		// HW Reset(Run RomBoot mode)
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(50);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(i);

		// Set Upgrade Mode(switch to upgrade procedure in RomBoot)
		ret = ft8756_reg_write(dev, 0x55, i2c_buf, 0);
		if(ret < 0) {
			TOUCH_E("set upgrade mode write error\n");
			goto FAIL;
		}
		touch_msleep(1);
		TOUCH_I("%s - Set Upgrade Mode and Check ID : %d ms\n", __func__, i);

		// Read FT8756 ID
		ret = ft8756_reg_read(dev, 0x90, i2c_buf, 2);
		if(ret < 0) {
			TOUCH_E("read ID register read error\n");
			goto FAIL;
		}

		//check if ID is valid
		TOUCH_I("Check ID : 0x%x , 0x%x\n", i2c_buf[0], i2c_buf[1]);

		if(i2c_buf[0] == 0x87 && i2c_buf[1] == 0x56){   // FT8756 ID : 0x87 0x56
			touch_msleep(5);
			break;
		}
	}

	if (i > 30) {
		TOUCH_E("timeout to set upgrade mode\n");
		goto FAIL;
	}

	// Write F/W (Pramboot) Binary to CTPM
	TOUCH_I("%s - Write F/W (Pramboot)\n", __func__);
	pramboot_ecc = 0;
	packet_num = (fw_size + FTS_PACKET_LENGTH - 1) / FTS_PACKET_LENGTH;
	for (i = 0; i < packet_num; i++) {
		packet_addr = i * FTS_PACKET_LENGTH;
		i2c_buf[0] = 0; //(u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[1] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[2] = (u8)((u32)(packet_addr) & 0x000000FF);
		if(packet_addr + FTS_PACKET_LENGTH > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = FTS_PACKET_LENGTH;
		i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
		i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);
		for (j = 0; j < packet_len; j++) {
			i2c_buf[5 + j] = fw_data[packet_addr + j];  // FT8756
			pramboot_ecc ^= i2c_buf[5 + j];
		}
		//TOUCH_I("#%d : Writing to %d , %d bytes\n", i, packet_addr, packet_len); //kjh
		ret = ft8756_reg_write(dev, 0xAE, i2c_buf, packet_len + 5);
		if(ret < 0) {
			TOUCH_E("f/w(Pramboot) binary to CTPM write error\n");
			goto FAIL;
		}
	}

	touch_msleep(20);

	// Verify F/W(Checksum)
	TOUCH_I("%s - Verify\n", __func__);
	for (i = 0; i < packet_num; i++) {
		i2c_buf[0] = 0x85;
		packet_addr = i * FTS_PACKET_LENGTH;
		i2c_buf[1] = 0; //(u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[2] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[3] = (u8)((u32)(packet_addr) & 0x000000FF);
		if(packet_addr + FTS_PACKET_LENGTH > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = FTS_PACKET_LENGTH;
		//TOUCH_I("#%d : Reading from %d , %d bytes\n", i, packet_addr, packet_len);  //kjh
		ret = ft8756_cmd_read(dev, i2c_buf, 4, fw_check_buf+packet_addr, packet_len);
		if(ret < 0) {
			TOUCH_E("checksum read error\n");
			goto FAIL;
		}
	}
	for (i = 0; i < fw_size; i++) {
		if(fw_check_buf[i] != fw_data[i]) {
			TOUCH_I("%s - Verify Failed %d %d %d!!\n", __func__,i, fw_check_buf[i], fw_data[i]); //kjh
			goto FAIL;
		}
	}

	TOUCH_I("%s -Pramboot write Verify OK !!\n", __func__);

	// Run PramBoot
	TOUCH_I("%s - Start App\n", __func__);
	ret = ft8756_reg_write(dev, 0x08, i2c_buf, 0);
	if(ret < 0) {
		TOUCH_E("Start App error\n");
		goto FAIL;
	}
	touch_msleep(10);
	if(fw_check_buf)
		kfree(fw_check_buf);

	TOUCH_I("===== Firmware (Pramboot) download Okay =====\n");

	return 0;

FAIL :

	if(fw_check_buf)
		kfree(fw_check_buf);

	TOUCH_I("===== Firmware (Pramboot) download FAIL!!! =====\n");

	return -EIO;

}

#if 1 // Flash CRC check Function.
static int ft8756_flash_crc_check(struct device *dev, const struct firmware *fw, const struct firmware *fw_boot)
{
	u8 i2c_buf[FTS_PACKET_LENGTH + 12] = {0,};
	int i;
	int ret1 = 0, ret2 = 0;
	u8 flash_crc_buf[5] = {0,};
	u8 bin_crc[5] = {0,};
	int crc_check = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("===== Flash CRC check : Start \n");
	TOUCH_I("===== Flash CRC check : Pramboot loading \n");
	ret1 = ft8756_fwboot_upgrade(dev, fw_boot);

	for (i = 0; i < 30; i++) {
		TOUCH_I("===== Flash CRC check : Check ID \n");

		ret1 |= ft8756_reg_write(dev, 0x55, i2c_buf, 0);
		touch_msleep(1);
		ret1 |= ft8756_reg_read(dev, 0x90, i2c_buf, 2);

		TOUCH_I("Check ID [%d] : 0x%x , 0x%x\n", i, i2c_buf[0], i2c_buf[1]);   // FT8756 : 0xF7, 0xA6
		if(i2c_buf[0] == 0xF7 && i2c_buf[1] == 0xA6)
			break;
		touch_msleep(10);
	}
	if (i == 30) {
		TOUCH_E("timeout to set Flash CRC check ID\n");
		goto FAIL;
	}

	for (i = 0; i < 5; i++)
	{
		// 4byte app2 CRC code read, FT8756 0x1108
		i2c_buf[0] = 0x00;
		i2c_buf[1] = 0x11;
		i2c_buf[2] = 0x08;
		ret2 |= ft8756_reg_write(dev, 0x03, i2c_buf, 3);

		msleep(10);

		i2c_buf[0] = 0x03;
		i2c_buf[1] = 0x00;
		i2c_buf[2] = 0x11;
		i2c_buf[3] = 0x08;
		ret2 |= ft8756_cmd_read(dev, i2c_buf, 4, flash_crc_buf, 4);
		TOUCH_I("===== Flash CRC check : CRC code: 0x%x, 0x%x, 0x%x, 0x%x \n", flash_crc_buf [0], flash_crc_buf [1], flash_crc_buf [2], flash_crc_buf [3]); 

		bin_crc[0] = fw->data[0x1100+0x08]; // CRC1 : FT8756
		bin_crc[1] = fw->data[0x1100+0x09]; // CRC2 : FT8756
		bin_crc[2] = fw->data[0x1100+0x0A]; // CRC3 : FT8756
		bin_crc[3] = fw->data[0x1100+0x0B]; // CRC4 : FT8756

		if((bin_crc[0]==flash_crc_buf[0]) && (bin_crc[1]==flash_crc_buf[1]) && (bin_crc[2]==flash_crc_buf[2])&& (bin_crc[3]==flash_crc_buf[3])){
			TOUCH_I("===== Flash CRC check : Compare OK =====\n");
			crc_check = 1;  // CRC comapre OK
			break;
		} else if(ret2 == 0){
			TOUCH_I("===== Flash CRC check : Compare NG =====\n");
			crc_check = 0;  // CRC comapre
			break;
		} else {
			TOUCH_E("Failed to get crc data\n");
		}
	}
	if (i == 5) {
		TOUCH_E("timeout to set Flash CRC check\n");
		goto FAIL;
	}

	/*SW Reset CMD*/
	ret2 = ft8756_reg_write(dev, 0x07, i2c_buf, 0);
	if(ret2 < 0) {
		TOUCH_E("0x07 reg_write error\n");
		goto FAIL;
	}

	return crc_check;

FAIL:
	TOUCH_I("===== Flash CRC Check FAIL!!! =====\n");

	// Reset Anyway
	ft8756_power(dev, POWER_OFF);
	ft8756_power(dev, POWER_ON);
	touch_msleep(ts->caps.hw_reset_delay);

	return -EIO;
}

#endif

static int ft8756_fw_compare(struct device *dev, const struct firmware *fw, const struct firmware *fw_boot)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	u8 ic_fw_version = d->ic_info.fw_version;
	u8 ic_is_official = d->ic_info.is_official;
	u8 bin_fw_version = 0;
	u8 bin_is_official = 0;
	int update = 0;
	int crc_check = -1;

	TOUCH_TRACE();

	if(d->ic_info.info_valid == 0) { // Failed to get ic info
		TOUCH_I("invalid ic info, skip fw upgrade\n");
		return 0;
	}

	bin_fw_version = fw->data[0x1100+0x0e]; // FT8756

	bin_is_official = (bin_fw_version & 0x80) >> 7;
	bin_fw_version &= 0x7F;

	if(d->ic_info.fw_version_bin == 0 && d->ic_info.is_official_bin == 0) { // IF fw ver of bin is not initialized
		d->ic_info.fw_version_bin = bin_fw_version;
		d->ic_info.is_official_bin = bin_is_official;
	}

//	u32 bin_ver_offset = *((u32 *)&fw->data[0xe8]);
//	u32 bin_pid_offset = *((u32 *)&fw->data[0xf0]);

//	if ((bin_ver_offset > FLASH_FW_SIZE) || (bin_pid_offset > FLASH_FW_SIZE)) {
//		TOUCH_I("%s : invalid offset\n", __func__);
//		return -1;
//	}

	if (ts->force_fwup) {
		update = 1;
	} else if ((ic_is_official != bin_is_official) || (ic_fw_version != bin_fw_version)) {
		update = 1;
	} else if ( (ic_is_official == bin_is_official) && (ic_fw_version == bin_fw_version)) {
		crc_check = ft8756_flash_crc_check(dev, fw, fw_boot);
		if (crc_check == 0)
		{
			update = 1;
			TOUCH_I("CRC_CHECK = %d, version is same, but crc is different", crc_check);
		}
		else if (crc_check == 1)
			TOUCH_I("CRC_CHECK = %d, version and crc are same", crc_check);
	}

	TOUCH_I("%s : binary[V%d.%d] device[V%d.%d]" \
		" -> update: %d, force: %d, crc_check : %d\n", __func__,
		bin_is_official, bin_fw_version, ic_is_official, ic_fw_version,
		update, ts->force_fwup, crc_check);

	return update;
}

static int ft8756_fw_upgrade(struct device *dev, const struct firmware *fw)
{
//	struct touch_core_data *ts = to_touch_core(dev);
	const u8 *fw_data = fw->data;
	u32 fw_size = (u32)(fw->size);
	u8 i2c_buf[FTS_PACKET_LENGTH + 12] = {0,};
	int ret;
	int packet_num, retry, i, j, packet_addr, packet_len;
	u8 fw_ecc;

	TOUCH_I("%s - Firmware(all.bin ) download START\n", __func__);

	for (i = 0; i < 30; i++) {
		// Enter Upgrade Mode
		TOUCH_I("%s - Enter Upgrade Mode and Check ID\n", __func__);

		ret = ft8756_reg_write(dev, 0x55, i2c_buf, 0);  //Enter upgrade mode of ParamBoot
		if(ret < 0) {
			TOUCH_E("enter upgrade mode error\n");
			return -EIO;
		}
		touch_msleep(1);

		// Read FT8756 ID
		ret = ft8756_reg_read(dev, 0x90, i2c_buf, 2);
		if(ret < 0) {
			TOUCH_E("read ID error\n");
			return -EIO;
		}

		//check if ID is correnct
		TOUCH_I("Check ID [%d] : 0x%x , 0x%x\n", i, i2c_buf[0], i2c_buf[1]);   // FT8756 : 0xF7, 0xA6
		if(i2c_buf[0] == 0xF7 && i2c_buf[1] == 0xA6)
			break;

		touch_msleep(10);
	}

	if (i == 30) {
		TOUCH_E("timeout to set upgrade mode\n");
		goto FAIL;
	}

	// Send flash erase type command(0x0A:All, 0x0b: App, 0x0C: Lcd)
	i2c_buf[0] = 0x0A;
	ret = ft8756_reg_write(dev, 0x09, i2c_buf, 1);

	if(ret < 0) {
		TOUCH_E("flash erase type command write error\n");
		goto FAIL;
	}
//	touch_msleep(50);

	// Send flash erase command
	TOUCH_I("%s - Erase App and Panel Parameter Area\n", __func__);

	ret = ft8756_reg_write(dev, 0x61, i2c_buf, 0);
	if(ret < 0) {
		TOUCH_E("flash erase command error\n");
		goto FAIL;
	}

	touch_msleep(1000);

	//Send 0x6A to check the flash erase status,erase operation is Finished if the return value is 0xF0 0xAA
	retry = 300;
	i = 0;
	do {
		ret = ft8756_reg_read(dev, 0x6A, i2c_buf, 2);
		if(ret < 0) {
			TOUCH_E("flash erase status check error\n");
			goto FAIL;
		}

		if(i2c_buf[0] == 0xF0 && i2c_buf[1] == 0xAA)
		{
			TOUCH_I("Erase Done : %d \n", i);
			break;
		}
		i++;
		touch_msleep(10);
	} while (--retry);


	// Write the total length of data expected to be written to Flash
	i2c_buf[0] = (u8)(((u32)(fw_size) & 0x00FF0000) >> 16);
	i2c_buf[1] = (u8)(((u32)(fw_size) & 0x0000FF00) >> 8);
	i2c_buf[2] = (u8)((u32)(fw_size) & 0x000000FF);
	ret = ft8756_reg_write(dev, 0x7A, i2c_buf, 3);
	TOUCH_I(" FW total length  [0x%02x] , [0x%02x] , [0x%02x]\n", i2c_buf[0], i2c_buf[1], i2c_buf[2]);
	touch_msleep(5);

	//  Write F/W (All or App) Binary to CTPM
	TOUCH_I("%s - Write F/W (App)\n", __func__);
	fw_ecc = 0;
	packet_num = (fw_size + FTS_PACKET_LENGTH - 1) / FTS_PACKET_LENGTH;
	for (i = 0; i < packet_num; i++) {
		packet_addr = i * FTS_PACKET_LENGTH;
		i2c_buf[0] = (u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[1] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[2] = (u8)((u32)(packet_addr) & 0x000000FF);
		if(packet_addr + FTS_PACKET_LENGTH > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = FTS_PACKET_LENGTH;
		i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
		i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);
		for (j = 0; j < packet_len; j++) {
			i2c_buf[5 + j] = fw_data[packet_addr + j];
			fw_ecc ^= i2c_buf[5 + j];
		}
#if 0
		TOUCH_I("#%d : Writing to %d , %d bytes..[0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", i, packet_addr, packet_len, \
				i2c_buf[0], i2c_buf[1], i2c_buf[2], i2c_buf[3], i2c_buf[4]);
#endif
		//Sending writing flash command(0xBF [ADD] [LEN] [DATA])
		ret = ft8756_reg_write(dev, 0xBF, i2c_buf, packet_len + 5);
		if(ret < 0) {
			TOUCH_E("write f/w(app) binary to CTPM error\n");
			goto FAIL;
		}

		// Waiting
		retry = 100;
		do {
			//Read 2bytes of 6A register to check if the flash write operation is finished
			ret = ft8756_reg_read(dev, 0x6A, i2c_buf, 2);
			if(ret < 0) {
				TOUCH_E("wating error\n");
				goto FAIL;
			}
			if((u32)(i + 0x1000) == (((u32)(i2c_buf[0]) << 8) | ((u32)(i2c_buf[1]))))
			{
				if((i & 0x007F) == 0) {
					TOUCH_I("Write Done : %d / %d\n", i+1, packet_num);
				}
				break;
			}
			touch_msleep(1);

		} while (--retry);

		if(retry == 0) {
			if((i+1)== packet_num)
				TOUCH_I("Write Done, Last packet. : %d / %d\n", i+1, packet_num);
			else
				TOUCH_I("Write Fail : %d / %d :	[0x%02x] , [0x%02x]\n", i+1, packet_num, i2c_buf[0], i2c_buf[1]);
		}
	}

	TOUCH_I("Write Finished : Total %d\n", packet_num);

	touch_msleep(50);

	// Checksum
	TOUCH_I("%s - Read out checksum (App) for %d bytes\n", __func__, fw_size);
	//Send clear checksum value command (0x64)
	ret = ft8756_reg_write(dev, 0x64, i2c_buf, 0);
	if(ret < 0) {
		TOUCH_E("clear checksum value command error\n");
		goto FAIL;
	}
	touch_msleep(300);

	packet_num = (fw_size + LEN_FLASH_ECC_MAX - 1) / LEN_FLASH_ECC_MAX;
	for (i = 0; i < packet_num; i++) {
		packet_addr = i * LEN_FLASH_ECC_MAX;
		i2c_buf[0] = (u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[1] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[2] = (u8)((u32)(packet_addr) & 0x000000FF);
		if(packet_addr + LEN_FLASH_ECC_MAX > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = LEN_FLASH_ECC_MAX;
		i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
		i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);

		//send checksum start address and length command(0x65 [Addr] [Len])
		ret = ft8756_reg_write(dev, 0x65, i2c_buf, 5);
		if(ret < 0) {
			TOUCH_E("checksum start address and length command error\n");
			goto FAIL;
		}

		TOUCH_I("cheecksum read range, packet num :  %d\n", packet_num);
		TOUCH_I("cheecksum read range  [0x%02x] ,[0x%02x] , [0x%02x] , [0x%02x]\n", i2c_buf[1], i2c_buf[2], i2c_buf[3], i2c_buf[4]);

		touch_msleep(fw_size/256);

		retry = 200;
		do {
			//run checksum operation, if 0xF0 0x55 means, checksum operation is finished
			ret = ft8756_reg_read(dev, 0x6A, i2c_buf, 2);
			if(ret < 0) {
				TOUCH_E("run checksum error\n");
				goto FAIL;
			}
			if(i2c_buf[0] == 0xF0 && i2c_buf[1] == 0x55)
			{
				TOUCH_I("Checksum read range, Calc.  [0x%02x] , [0x%02x]\n", i2c_buf[0], i2c_buf[1]);
				break;
			}
			else
			{
				TOUCH_I("Checksum read range, Calc.  [0x%02x] , [0x%02x]\n", i2c_buf[0], i2c_buf[1]);
			}
			touch_msleep(1);
		} while (--retry);
	}
	//Send read checksum command
	ret = ft8756_reg_read(dev, 0x66, i2c_buf, 1);
	if(ret < 0) {
		TOUCH_E("sned read checksum command error\n");
		goto FAIL;
	}
	TOUCH_I("Reg 0x66 : 0x%x\n", i2c_buf[0]);

	if(i2c_buf[0] != fw_ecc)
	{
		TOUCH_E("Checksum ERROR : Reg 0x66 [0x%x] , fw_ecc [0x%x]\n", i2c_buf[0], fw_ecc);
		goto FAIL;
	}

	TOUCH_I("Checksum OK : Reg 0x66 [0x%x] , fw_ecc [0x%x]\n", i2c_buf[0], fw_ecc);

	TOUCH_I("===== Firmware download OK!!! =====\n");

	//Exit download mode
	ret = ft8756_reg_write(dev, 0x07, i2c_buf, 0);
	if(ret < 0) {
		TOUCH_E("Exit download mode write fail, ret = %d\n", ret);
		goto FAIL;
	}

	/*After SW Reset, not need HW_Reset_Delay becuz it wiil take in CORE side*/
/*
	// Do something for recovering LCD
	ret = tianma_ft860x_firmware_recovery();
	touch_msleep(300);

	if(ret != 0){
		TOUCH_I("tianma_ft860x_firmware_recovery didn't LCD reset : ret = %d\n", ret);
	}
*/
	return 0;

FAIL:

	TOUCH_I("===== Firmware download FAIL!!! =====\n");

	// Exit download mode
	ret = ft8756_reg_write(dev, 0x07, i2c_buf, 0);
	if(ret < 0) {
		TOUCH_E("Exit download mode write fail, ret = %d\n", ret);
	}

	return -EIO;

}

static int ft8756_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fw = NULL;
	const struct firmware *fw_boot = NULL;
	int ret = 0;
	int retry_upgrade = 0;
	char fwpath[256] = {0};
//	int i = 0;
	int boot_mode = TOUCH_NORMAL_BOOT;

	TOUCH_TRACE();

	boot_mode = touch_check_boot_mode(dev);

	if ((boot_mode == TOUCH_CHARGER_MODE)
			|| (boot_mode == TOUCH_LAF_MODE)
			|| (boot_mode == TOUCH_RECOVERY_MODE)) {
		TOUCH_I("%s: boot_mode = %d(CHARGER:5/LAF:6/RECOVER_MODE:7)\n", __func__, boot_mode);
		return -EPERM;
	}

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		return -EPERM;
	}

	if (ts->test_fwpath[0] && (ts->test_fwpath[0] != 't' && ts->test_fwpath[1] != 0)) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n", &ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
#if 0//defined(CONFIG_LGE_TOUCH_MODULE_DETECT)
		if (ft8756_panel_type == LCE_FT8756) {
			strncpy(fwpath, ts->def_fwpath[1], sizeof(fwpath) - 1);
			TOUCH_I("Get Touch Firmware for LCE-BOE\n");
			TOUCH_I("get fwpath from def_fwpath : %s\n", fwpath);
		}
		else {
			strncpy(fwpath, ts->def_fwpath[2], sizeof(fwpath) - 1);
			TOUCH_I("Get Touch Firmware for TXD-CPT\n");
			TOUCH_I("get fwpath from def_fwpath : %s\n", fwpath);
		}
#else
    	strncpy(fwpath, ts->def_fwpath[1], sizeof(fwpath) - 1);
		TOUCH_I("get fwpath from def_fwpath : %s\n", fwpath);
#endif
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	if (fwpath == NULL) {
		TOUCH_E("error get fw path\n");
		return -EPERM;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);

	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", fwpath, ret);
		release_firmware(fw);
		return ret;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	TOUCH_I("fwpath_boot[%s]\n", ts->def_fwpath[0]);  // PARAMBOOT.BIN
	ret = request_firmware(&fw_boot, ts->def_fwpath[0], dev);
	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", ts->def_fwpath[0], ret);
		release_firmware(fw);
		release_firmware(fw_boot);
		return ret;
	}

	for (retry_upgrade = 0; retry_upgrade < 3; retry_upgrade++) { //fw_check function need
		if (ft8756_fw_compare(dev, fw, fw_boot))  {

			ret = ft8756_fwboot_upgrade(dev, fw_boot);
			if(ret < 0) {
				TOUCH_E("fail to upgrade f/w (pramboot) : %d\n", ret);
				continue;
			}

			ret = ft8756_fw_upgrade(dev, fw);
			if(ret < 0) {
				TOUCH_E("fail to upgrade f/w : %d\n", ret);
				continue;
			}

			TOUCH_I("f/w upgrade complete\n");
			break;
		} else {
			TOUCH_I("f/w no need upgrade\n");
			break;
		}
	}

	release_firmware(fw);
	release_firmware(fw_boot);

	if (retry_upgrade >= 3)
		return -EPERM;

	return 0;
}

#if defined(CONFIG_LGE_TOUCH_APP_FW_UPGRADE)
static int ft8756_app_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fw = NULL;
	const struct firmware *fw_boot = NULL;
	struct firmware temp_fw;
	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		return -EPERM;
	}

	if ((ts->app_fw_upgrade.offset == 0) || (ts->app_fw_upgrade.data == NULL)) {
		TOUCH_E("app_fw_upgrade.offset = %d, app_fw_upgrade.data = %p\n",
				(int)ts->app_fw_upgrade.offset, ts->app_fw_upgrade.data);
		return -EPERM;
	}

	memset(&temp_fw, 0, sizeof(temp_fw));
	temp_fw.size = ts->app_fw_upgrade.offset;
	temp_fw.data = ts->app_fw_upgrade.data;

	fw = &temp_fw;

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	TOUCH_I("fwpath_boot[%s]\n", ts->def_fwpath[0]);  // PARAMBOOT.BIN
	ret = request_firmware(&fw_boot, ts->def_fwpath[0], dev);
	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", ts->def_fwpath[0], ret);
		return ret;
	}

	if (ft8756_fw_compare(dev, fw, fw_boot)) {
		ret = ft8756_fwboot_upgrade(dev, fw_boot);
		if(ret < 0) {
			TOUCH_E("fail to upgrade f/w (pramboot) : %d\n", ret);
			release_firmware(fw_boot);
			return -EPERM;
		}

		ret = ft8756_fw_upgrade(dev, fw);
		if(ret < 0) {
			TOUCH_E("fail to upgrade f/w : %d\n", ret);
			release_firmware(fw_boot);
			return -EPERM;
		}

		TOUCH_I("f/w upgrade complete\n");
	}

	release_firmware(fw_boot);

	return 0;
}
#endif

static int ft8756_suspend(struct device *dev)
{
//	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	//int mfts_mode = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (touch_check_boot_mode(dev) == TOUCH_CHARGER_MODE)
		return -EPERM;
/*
	mfts_mode = touch_check_boot_mode(dev);
	if ((mfts_mode >= TOUCH_MINIOS_MFTS_FOLDER) && !ts->mfts_lpwg) {
		TOUCH_I("%s : touch_suspend - MFTS\n", __func__);
		touch_interrupt_control(dev, INTERRUPT_DISABLE);
		ft8756_power(dev, POWER_OFF);
		return -EPERM;
	} else {
		TOUCH_I("%s : touch_suspend start\n", __func__);
//		if (d->lcd_mode == LCD_MODE_U2 &&
//			atomic_read(&d->watch.state.rtc_status) == RTC_RUN &&
//			d->watch.ext_wdata.time.disp_waton)
//				ext_watch_get_current_time(dev, NULL, NULL);
	}
*/
	if (atomic_read(&d->init) == IC_INIT_DONE)
		ft8756_lpwg_mode(dev);
	else /* need init */
		ret = 1;

	return ret;
}

static int ft8756_resume(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	int mfts_mode = 0;

	TOUCH_TRACE();

	mfts_mode = touch_check_boot_mode(dev);
	if ((mfts_mode >= TOUCH_MINIOS_MFTS_FOLDER && mfts_mode <= TOUCH_MINIOS_MFTS_CURVED) && !ts->mfts_lpwg) {
		#if 0
		ft8756_power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
		ft8756_ic_info(dev);
		if (ft8756_upgrade(dev) == 0) {
			ft8756_power(dev, POWER_OFF);
			ft8756_power(dev, POWER_ON);
			touch_msleep(ts->caps.hw_reset_delay);
		}
		//return -EPERM;
		#endif
		// Activate IC is done in fb callback EARLY BLANK (POWER_OFF => ACTIVE)
		// Return 0 to do init in touch_resume
		if (d->state == TC_STATE_ACTIVE)
			return 0;
		else
			return -EPERM;
	}

	if (touch_check_boot_mode(dev) == TOUCH_CHARGER_MODE) {
		ft8756_deep_sleep(dev, 1);
		return -EPERM;
	}


//	if (atomic_read(&d->init) == IC_INIT_DONE)
//		ft8756_lpwg_mode(dev);
//	else /* need init */
//		ret = 1;

	return 0;
}

static int ft8756_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
//	u32 data = 1;
	int ret = 0;

	TOUCH_TRACE();

/*
	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		TOUCH_I("ft8756 fb_notif register\n");
		fb_unregister_client(&ts->fb_notif);
		ts->fb_notif.notifier_call = ft8756_fb_notifier_callback;
		fb_register_client(&ts->fb_notif);
	}
*/
	touch_msleep(50);
	TOUCH_I("%s: charger_state = 0x%02X\n", __func__, d->charger);

//	if (atomic_read(&ts->state.debug_tool) == DEBUG_TOOL_ENABLE)
//		ft8756_sic_abt_init(dev);

	ret = ft8756_ic_info(dev);
	if (ret < 0) {
		TOUCH_I("failed to get ic_info, ret:%d", ret);
		atomic_set(&d->init, IC_INIT_DONE); // Nothing to init, anyway DONE init

		return 0;
		//touch_interrupt_control(dev, INTERRUPT_DISABLE);
		//ft8756_power(dev, POWER_OFF);
		//ft8756_power(dev, POWER_ON);
		//touch_msleep(ts->caps.hw_reset_delay);
	}
#if 0
	ret = ft8756_reg_write(dev, tc_device_ctl, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'tc_device_ctrl\', ret:%d\n", ret);

	ret = ft8756_reg_write(dev, tc_interrupt_ctl, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'tc_interrupt_ctrl\', ret:%d\n", ret);
#endif
	TOUCH_I("%s: charger_state = 0x%02X\n", __func__, d->charger);
	ret = ft8756_reg_write(dev, SPR_CHARGER_STS, &d->charger, sizeof(u8));
	if (ret)
		TOUCH_E("failed to write \'spr_charger_sts\', ret:%d\n", ret);

	ret = ft8756_reg_write(dev, SPR_QUICKCOVER_STS, &ts->lpwg.qcover, sizeof(u8));
	if (ret)
		TOUCH_E("failed to write SPR_QUICKCOVER_STS, ret:%d\n", ret);
#if 0
	data = atomic_read(&ts->state.ime);
	TOUCH_I("%s: ime_state = %d\n", __func__, data);
	ret = ft8756_reg_write(dev, REG_IME_STATE, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'reg_ime_state\', ret:%d\n", ret);
#endif

	/* TODO Control grip suppresion value */
	ft8756_grip_suppression_ctrl(dev, 15, 40);

	atomic_set(&d->init, IC_INIT_DONE);
	atomic_set(&ts->state.sleep, IC_NORMAL);
	d->state = TC_STATE_ACTIVE;

	ft8756_lpwg_mode(dev);
	if (ret)
		TOUCH_E("failed to lpwg_control, ret:%d\n", ret);

	return 0;
}

static int ft8756_noise_log(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);

	int ret = 0;
	u8 data = 0;
	u8 rebase = 0;
	u8 frequency = 0;
	u8 noise_level = 0;

	TOUCH_TRACE();

	ret = ft8756_reg_read(dev, FTS_REG_NOISE_HOPPING_REBASE, &data, 1);
	if (ret < 0) {
		TOUCH_E("0xEF(noise & hopping & rebase)register read error\n");
		return ret;
	}

	noise_level = (data & 0x0F); //noise level (0(00) ~ 9(1001))
	frequency = (data & 0x30) >> 4; //frequency (00 = freq1, 01 = freq2, 10 = freq3)
	rebase = (data & 0xC0) >> 6;// rebase (00 = no rebase, 01 = rebase)

	d->noise_info.noise_level = noise_level;
	d->noise_info.frequency = frequency;
	d->noise_info.rebase = rebase;

	if (d->noise_info.noise_log == NOISE_ENABLE) {
		if (ts->old_mask != ts->new_mask) {
			TOUCH_I("Rebase[%2d] Frequency_mode[%2d] Noise_level[%2d]\n", rebase, frequency, noise_level);
		}
	}


	return 0;
}

static int ft8756_irq_abs_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	struct ft8756_touch_data *data = d->info.data;
	struct touch_data *tdata;
	u32 touch_count = 0;
	u8 finger_index = 0;
	int ret = 0;
	int i = 0;
	u8 touch_id, event, palm;

	touch_count = d->info.touch_cnt;
	ts->new_mask = 0;

	TOUCH_TRACE();

	for (i = 0; i < FTS_MAX_POINTS; i++) {
		touch_id = (u8)(data[i].yh) >> 4;
		if (touch_id >= MAX_FINGER) {
			break; // ??
		}

		event = (u8)(data[i].xh) >> 6;
		palm = ((u8)(data[i].xh) >> 4) & 0x01;

		if (palm) {
			if (event == FTS_TOUCH_CONTACT) { // FTS_TOUCH_DOWN
				ts->is_cancel = 1;
				TOUCH_I("Palm Detected\n");
			}
			else if (event == FTS_TOUCH_UP) {
				ts->is_cancel = 0;
				TOUCH_I("Palm Released\n");
			}
			ts->tcount = 0;
			ts->intr_status = TOUCH_IRQ_FINGER;
			return 0;
		}

		if(event == FTS_TOUCH_DOWN || event == FTS_TOUCH_CONTACT) {
			ts->new_mask |= (1 << touch_id);
			tdata = ts->tdata + touch_id;

			tdata->id = touch_id;
			tdata->type = MT_TOOL_FINGER;
			tdata->x = ((u16)(data[i].xh & 0x0F))<<8 | (u16)(data[i].xl);
			tdata->y = ((u16)(data[i].yh & 0x0F))<<8 | (u16)(data[i].yl);
			tdata->pressure = (u8)(data[i].weight);

			if (major_value == 10)
				major_value = 11;
			else
				major_value = 10;

			if (minor_value == 10)
				minor_value = 11;
			else
				minor_value = 10;

			tdata->width_major = major_value;
			tdata->width_minor = minor_value;

			tdata->orientation = 0;

			finger_index++;

			TOUCH_D(ABS, "tdata [id:%d e:%d x:%d y:%d z:%d - %d,%d,%d]\n",\
					tdata->id,\
					event,\
					tdata->x,\
					tdata->y,\
					tdata->pressure,\
					tdata->width_major,\
					tdata->width_minor,\
					tdata->orientation);

		}
	}

#ifdef FT8756_ESD_SKIP_WHILE_TOUCH_ON
	if (finger_index != finger_cnt) {
//		TOUCH_I("finger cnt changed from %d to %d\n", finger_cnt, finger_index);
		finger_cnt = finger_index;
	}
#endif
	if (d->noise_info.noise_log == NOISE_ENABLE) {
		if (ts->old_mask != ts->new_mask) {
			ret  = ft8756_noise_log(dev);
			if (ret < 0)
				TOUCH_I("failed to print rebase & frequency & rebase log (ret : %d)\n", ret);
		}
	}

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return 0;

}

int ft8756_irq_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	struct ft8756_touch_data *data = d->info.data;
	u8 point_buf[POINT_READ_BUF] = { 0, };
	int ret = -1;

	TOUCH_TRACE();

	ret = ft8756_reg_read(dev, 0x02, point_buf+2, POINT_READ_BUF-2);

	if (ret < 0) {
		TOUCH_E("Fail to read point regs.\n");
		return ret;
	}

	/* check if touch cnt is valid */
	if (/*point_buf[FTS_TOUCH_P_NUM] == 0 || */point_buf[FTS_TOUCH_P_NUM] > ts->caps.max_id) {
		TOUCH_I("%s : touch cnt is invalid - %d\n",
			__func__, point_buf[FTS_TOUCH_P_NUM]);
		return -ERANGE;
	}

	d->info.touch_cnt = point_buf[FTS_TOUCH_P_NUM];

	memcpy(data, point_buf+FTS_TOUCH_EVENT_POS, FTS_ONE_TCH_LEN * FTS_MAX_POINTS);

	return ft8756_irq_abs_data(dev);
}

int ft8756_irq_lpwg(struct device *dev, int tci)
{
	struct touch_core_data *ts = to_touch_core(dev);
	//struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0, tap_count = 0, i = 0, j = 0;
	int ignore_event_area = 90; // caution, only ace project ft8756!!

	u8 tci_data_buf[MAX_TAP_COUNT*4 + 2];

	TOUCH_TRACE();

	if(ts->lpwg.mode == LPWG_NONE || (ts->lpwg.mode == LPWG_DOUBLE_TAP && tci == TCI_2)) {
		TOUCH_I("lpwg irq is invalid!!\n");
		return -1;
	}

	ret = ft8756_reg_read(dev, 0xD3, tci_data_buf, 2);
	if (ret < 0) {
		TOUCH_E("Fail to read tci data\n");
		return ret;
	}

	TOUCH_I("TCI Data : TCI[%d], Result[%d], TapCount[%d]\n", tci, tci_data_buf[0], tci_data_buf[1]);

	// Validate tci data
	if (!((tci_data_buf[0] == 0x01 && tci == TCI_1) || (tci_data_buf[0] == 0x02 && tci == TCI_2))
		|| tci_data_buf[1] == 0 || tci_data_buf[1] > MAX_TAP_COUNT) {
		TOUCH_I("tci data is invalid!!\n");
		return -1;
	}

	tap_count = tci_data_buf[1];

	ret = ft8756_reg_read(dev, 0xD3, tci_data_buf, tap_count*4 + 2);
	if (ret < 0) {
		TOUCH_E("Fail to read tci data\n");
		return ret;
	}

	ts->lpwg.code_num = tap_count;
	for (i = 0; i < tap_count; i++) {
		j = i*4+2;
		ts->lpwg.code[i].x = ((int)tci_data_buf[j] << 8) | (int)tci_data_buf[j+1];
		ts->lpwg.code[i].y = ((int)tci_data_buf[j+2] << 8) | (int)tci_data_buf[j+3];

		/* left top, right top, left bottom, right bottom is no node */
		if (ts->lpwg.code[i].x < ignore_event_area ||
				ts->lpwg.code[i].x > ts->caps.max_x - ignore_event_area) {
			if (ts->lpwg.code[i].y < ignore_event_area ||
					ts->lpwg.code[i].y > ts->caps.max_y - ignore_event_area) {
				TOUCH_I("skip knock on event, because invalid area");
				ts->intr_status = TOUCH_IRQ_NONE;
			}
		}

		if (ts->lpwg.mode >= LPWG_PASSWORD)
			TOUCH_I("LPWG data xxxx, xxxx\n");
		else
			TOUCH_I("LPWG data %d, %d\n", ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[tap_count].x = -1;
	ts->lpwg.code[tap_count].y = -1;

	return ret;
}

int ft8756_get_swipe_data(struct device *dev, int swipe) {
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	u8 swipe_data_buf[10] = {0, };
	u16 start_x, start_y, end_x, end_y, time;

	TOUCH_TRACE();

	if (ts->swipe[SWIPE_D].enable == false &&
			ts->swipe[SWIPE_U].enable == false &&
			ts->swipe[SWIPE_L2].enable == false &&
			ts->swipe[SWIPE_R2].enable == false) {
		TOUCH_I("swipe irq is invalid!!\n");
		return -1;
	}

	ret = ft8756_reg_read(dev, 0x02, swipe_data_buf, sizeof(swipe_data_buf));
	if (ret < 0) {
		TOUCH_E("Fail to read swipe data\n");
		return ret;
	}

	start_x = swipe_data_buf[0] | ((u16)swipe_data_buf[1] << 8);
	start_y = swipe_data_buf[2] | ((u16)swipe_data_buf[3] << 8);
	end_x = swipe_data_buf[4] | ((u16)swipe_data_buf[5] << 8);
	end_y = swipe_data_buf[6] | ((u16)swipe_data_buf[7] << 8);
	time = swipe_data_buf[8] | ((u16)swipe_data_buf[9] << 8);

	TOUCH_I("SWIPE Data : SWIPE_DIR[%d], start(%d, %d), end(%d, %d), time(%dms)\n",
			swipe, start_x, start_y, end_x, end_y, time);

	return 0;
}


int ft8756_irq_report_tci_fr(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0;
	u8 data, tci_1_fr;
#ifdef KNOCK_CODE
        u8 tci_2_fr;
#endif

	if (d->tci_debug_type != TCI_DEBUG_ALWAYS && d->tci_debug_type != TCI_DEBUG_BUFFER_ALWAYS) {
		TOUCH_I("tci debug in real time is disabled!!\n");
		return 0;
	}

	ret = ft8756_reg_read(dev, 0xE6, &data, 1);
	if (ret < 0) {
		TOUCH_E("0xE6 register read error\n");
		return ret;
	}

	tci_1_fr = data & 0x0F; // TCI_1

	if (tci_1_fr < TCI_FR_NUM) {
		TOUCH_I("Knock-on Failure Reason Reported : [%s]\n", tci_debug_str[tci_1_fr]);
	}
	else {
		TOUCH_I("Knock-on Failure Reason Reported : [%s]\n", tci_debug_str[TCI_FR_NUM]);
	}

#ifdef KNOCK_CODE
	tci_2_fr = (data & 0xF0) >> 4; // TCI_2

	if (tci_2_fr < TCI_FR_NUM) {
		TOUCH_I("Knock-code Failure Reason Reported : [%s]\n", tci_debug_str[tci_2_fr]);
	}
	else {
		TOUCH_I("Knock-code Failure Reason Reported : [%s]\n", tci_debug_str[TCI_FR_NUM]);
	}
#endif

	return ret;
}

int ft8756_irq_report_swipe_fr(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0;
	u8 data, swipe_dir, swipe_fr;

	TOUCH_TRACE();

	if (d->swipe_debug_type != TCI_DEBUG_ALWAYS && d->swipe_debug_type != TCI_DEBUG_BUFFER_ALWAYS) {
		TOUCH_I("swipe debug in real time is disabled!!\n");
		return 0;
	}

	ret = ft8756_reg_read(dev, 0x96, &data, 1);
	if (ret < 0) {
		TOUCH_E("0x96 register read error\n");
		return ret;
	}

	swipe_dir = (data & 0xF0) >> 4; // Direction
	swipe_fr = data & 0x0F; // Error code

	if (swipe_dir < SWIPE_FAILREASON_DIR_NUM && swipe_fr < SWIPE_FAILREASON_NUM) {
		TOUCH_I("Swipe Failure Reason Reported : [%s][%s]\n",
				swipe_debug_direction_str[swipe_dir],
				swipe_debug_str[swipe_fr]);
	}
	else {
		TOUCH_I("Swipe Failure Reason Reported : [%s][%s]\n",
				swipe_debug_direction_str[SWIPE_FAILREASON_DIR_NUM],
				swipe_debug_str[SWIPE_FAILREASON_NUM]);
	}

	return ret;
}

int ft8756_report_tci_fr_buffer(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0, i;
	u8 tci_fr_buffer[1 + TCI_FR_BUF_LEN], tci_fr_cnt, tci_fr;

	TOUCH_TRACE();

	if (d->tci_debug_type != TCI_DEBUG_BUFFER && d->tci_debug_type != TCI_DEBUG_BUFFER_ALWAYS) {
		TOUCH_I("tci debug in buffer is disabled!!\n");
		return 0;
	}

	ft8756_dummy_read(dev);

	// Knock-on
	for (i = 0; i < 25; i++) {
		ret = ft8756_reg_read(dev, 0xE7, &tci_fr_buffer, sizeof(tci_fr_buffer));
		if (!ret) {
			break;
		}
		msleep(2);
	}
	if (i == 25) {
		TOUCH_E("0xE7 register 10 byte read error\n");
		return ret;
	}

	tci_fr_cnt = tci_fr_buffer[0];
	if (tci_fr_cnt > TCI_FR_BUF_LEN) {
		TOUCH_I("Knock-on Failure Reason Buffer Count Invalid\n");
	}
	else if (tci_fr_cnt == 0) {
		TOUCH_I("Knock-on Failure Reason Buffer NONE\n");
	}
	else {
		for (i = 0; i < tci_fr_cnt; i++) {
			tci_fr = tci_fr_buffer[1 + i];
			if (tci_fr < TCI_FR_NUM) {
				TOUCH_I("Knock-on Failure Reason Buffer [%02d] : [%s]\n", i+1, tci_debug_str[tci_fr]);
			}
			else {
				TOUCH_I("Knock-on Failure Reason Buffer [%02d] : [%s]\n", i+1, tci_debug_str[TCI_FR_NUM]);
			}
		}
	}

#ifdef KNOCK_CODE
	ft8756_dummy_read(dev);

	// Knock-code (Same as knock-on case except for reg addr)
	for (i = 0; i < 25; i++) {
		ret = ft8756_reg_read(dev, 0xE9, &tci_fr_buffer, sizeof(tci_fr_buffer));
		if (!ret) {
			break;
		}
		msleep(2);
	}
	if (i == 25) {
		TOUCH_E("0xE9 register 10 byte read error\n");
		return ret;
	}

	tci_fr_cnt = tci_fr_buffer[0];
	if (tci_fr_cnt > TCI_FR_BUF_LEN) {
		TOUCH_I("Knock-code Failure Reason Buffer Count Invalid\n");
	}
	else if (tci_fr_cnt == 0) {
		TOUCH_I("Knock-code Failure Reason Buffer NONE\n");
	}
	else {
		for (i = 0; i < tci_fr_cnt; i++) {
			tci_fr = tci_fr_buffer[1 + i];
			if (tci_fr < TCI_FR_NUM) {
				TOUCH_I("Knock-code Failure Reason Buffer [%02d] : [%s]\n", i+1, tci_debug_str[tci_fr]);
			}
			else {
				TOUCH_I("Knock-code Failure Reason Buffer [%02d] : [%s]\n", i+1, tci_debug_str[TCI_FR_NUM]);
			}
		}
	}

#endif
	return ret;
}

int ft8756_report_swipe_fr_buffer(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0, i;
	u8 swipe_fr_buffer[1 + SWIPE_FAILREASON_BUF_LEN];
	u8 swipe_fr_cnt;
	u8 data, swipe_dir, swipe_fr;

	TOUCH_TRACE();

	if (d->swipe_debug_type != TCI_DEBUG_BUFFER && d->swipe_debug_type != TCI_DEBUG_BUFFER_ALWAYS) {
		TOUCH_I("swipe debug in buffer is disabled!!\n");
		return 0;
	}

	ft8756_dummy_read(dev);

	// Knock-on
	for (i = 0; i < 25; i++) {
		ret = ft8756_reg_read(dev, 0x93, &swipe_fr_buffer, sizeof(swipe_fr_buffer));
		if (!ret) {
			break;
		}
		msleep(2);
	}
	if (i == 25) {
		TOUCH_E("0x93 register 10 byte read error\n");
		return ret;
	}

	swipe_fr_cnt = swipe_fr_buffer[0];
	if (swipe_fr_cnt > SWIPE_FAILREASON_BUF_LEN) {
		TOUCH_I("Swipe Failure Reason Buffer Count Invalid\n");
	}
	else if (swipe_fr_cnt == 0) {
		TOUCH_I("Swipe Failure Reason Buffer NONE\n");
	}
	else {
		for (i = 0; i < swipe_fr_cnt; i++) {
			data = swipe_fr_buffer[1 + i];
			swipe_dir = (data & 0xF0) >> 4; // Direction
			swipe_fr = data & 0x0F; // Error code

			if (swipe_dir < SWIPE_FAILREASON_DIR_NUM && swipe_fr < SWIPE_FAILREASON_NUM) {
				TOUCH_I("Swipe Failure Reason Buffer [%02d] : [%s][%s]\n",
						i+1,
						swipe_debug_direction_str[swipe_dir],
						swipe_debug_str[swipe_fr]);
			}
			else {
				TOUCH_I("Swipe Failure Reason Buffer [%02d] : [%s][%s]\n",
						i+1,
						swipe_debug_direction_str[SWIPE_FAILREASON_DIR_NUM],
						swipe_debug_str[SWIPE_FAILREASON_NUM]);
			}
		}
	}

	return ret;
}

int ft8756_irq_handler(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct touch_core_data *ts =(struct touch_core_data *)dev->driver_data;
	//struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0;
	u8 int_status = 0;

	ret = ft8756_reg_read(dev, 0x01, &int_status, 1);
	if (ret < 0)
		return ret;

	switch (int_status) {
		case 0x01: //Finger
			ret = ft8756_irq_abs(dev);
			break;
		case 0x02: //TCI_1
			ts->intr_status = TOUCH_IRQ_KNOCK;
			ret = ft8756_irq_lpwg(dev, TCI_1);
			break;
#ifdef KNOCK_CODE
		case 0x03: //TCI_2
			ts->intr_status = TOUCH_IRQ_PASSWD;
			ret = ft8756_irq_lpwg(dev, TCI_2);
			break;
#endif
		case 0x04: //LPWG Fail Reason Report(RealTime)
			ret = ft8756_irq_report_tci_fr(dev);
			break;
		case 0x05: //ESD
			TOUCH_I("ESD interrupt !!\n");
			ret = -EGLOBALRESET;
			break;
		case 0x06: // Swipe Up
			ts->intr_status = TOUCH_IRQ_SWIPE_UP;
			ret = ft8756_get_swipe_data(dev, SWIPE_U);
			break;
		case 0x07: // Swipe Down
			ts->intr_status = TOUCH_IRQ_SWIPE_DOWN;
			ret = ft8756_get_swipe_data(dev, SWIPE_D);
			break;
		case 0x08: // Swipe Left
			ts->intr_status = TOUCH_IRQ_SWIPE_UP;
			ret = ft8756_get_swipe_data(dev, SWIPE_L2);
			break;
		case 0x09: // Swipe Right
			ts->intr_status = TOUCH_IRQ_SWIPE_UP;
			ret = ft8756_get_swipe_data(dev, SWIPE_R2);
			break;
		case 0x0A: // Swipe Fail Reason Report (RealTime)
			ret = ft8756_irq_report_swipe_fr(dev);
			break;
		case 0x0B:
			TOUCH_I("Palm Detected\n");
			ts->new_mask = 0;
			ts->is_cancel = 1;
			ts->tcount = 0;
			ts->intr_status = TOUCH_IRQ_FINGER;
			break;
		case 0x0C:
			TOUCH_I("Palm Released\n");
			break;
		case 0x00:
			TOUCH_I("No interrupt status\n");
			break;
		default:
			TOUCH_E("Invalid interrupt status : %d\n", int_status);
			ret = -EGLOBALRESET;
			break;
	}

	return ret;
}

static ssize_t store_reg_ctrl(struct device *dev,
				const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	char command[6] = {0};
	u32 reg = 0;
	u32 value = 0;
	u8 data = 0;
	u8 reg_addr;

	TOUCH_TRACE();

	if (sscanf(buf, "%5s %x %x", command, &reg, &value) <= 0)
		return count;

	mutex_lock(&ts->lock);

	reg_addr = (u8)reg;
	if (!strcmp(command, "write")) {
		data = (u8)value;
		if (ft8756_reg_write(dev, reg_addr, &data, sizeof(u8)) < 0)
			TOUCH_E("reg addr 0x%x write fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else if (!strcmp(command, "read")) {
		if (ft8756_reg_read(dev, reg_addr, &data, sizeof(u8)) < 0)
			TOUCH_E("reg addr 0x%x read fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else {
		TOUCH_D(BASE_INFO, "Usage\n");
		TOUCH_D(BASE_INFO, "Write reg value\n");
		TOUCH_D(BASE_INFO, "Read reg\n");
	}

	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t show_tci_debug(struct device *dev, char *buf)
{
	struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = touch_snprintf(buf + ret, PAGE_SIZE,
			"Current TCI Debug Type = %s\n",
			(d->tci_debug_type < 4) ? tci_debug_type_str[d->tci_debug_type] : "Invalid");

	TOUCH_I("Current TCI Debug Type = %s\n",
			(d->tci_debug_type < 4) ? tci_debug_type_str[d->tci_debug_type] : "Invalid");

	return ret;
}

static ssize_t store_tci_debug(struct device *dev, const char *buf, size_t count)
{
	struct ft8756_data *d = to_ft8756_data(dev);
	int value = 0;

	TOUCH_TRACE();
	if (sscanf(buf, "%d", &value) <= 0 || value < 0 || value > 3) {
		TOUCH_I("Invalid TCI Debug Type, please input 0~3\n");
		return count;
	}

	d->tci_debug_type = (u8)value;

	TOUCH_I("Set TCI Debug Type = %s\n", (d->tci_debug_type < 4) ? tci_debug_type_str[d->tci_debug_type] : "Invalid");

	return count;
}

static ssize_t show_swipe_debug(struct device *dev, char *buf)
{
	struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = touch_snprintf(buf + ret, PAGE_SIZE,
			"Current SWIPE Debug Type = %s\n",
			(d->swipe_debug_type < 4) ? tci_debug_type_str[d->swipe_debug_type] : "Invalid");

	TOUCH_I("Current Swipe Debug Type = %s\n",
			(d->swipe_debug_type < 4) ? tci_debug_type_str[d->swipe_debug_type] : "Invalid");

	return ret;
}

static ssize_t store_swipe_debug(struct device *dev, const char *buf, size_t count)
{
	struct ft8756_data *d = to_ft8756_data(dev);
	int value = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0 || value < 0 || value > 3) {
		TOUCH_I("Invalid SWIPE Debug Type, please input 0~3\n");
		return count;
	}

	d->swipe_debug_type = (u8)value;

	TOUCH_I("Set SWIPE Debug Type = %s\n", (d->swipe_debug_type < 4) ? tci_debug_type_str[d->swipe_debug_type] : "Invalid");

	return count;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	int value = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value == 1) {
		touch_interrupt_control(dev, INTERRUPT_DISABLE);
		ft8756_reset_ctrl(dev, value);

		ft8756_init(dev);
		touch_interrupt_control(dev, INTERRUPT_ENABLE);
	} else if (value == 2) {
		ft8756_esd_recovery(dev);
	} else {
		TOUCH_I("no test!\n");
	}

	return count;
}

static ssize_t show_pinstate(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = touch_snprintf(buf, PAGE_SIZE, "RST:%d, INT:%d\n",
			gpio_get_value(ts->reset_pin), gpio_get_value(ts->int_pin));
	TOUCH_I("%s() buf:%s",__func__, buf);
	return ret;
}

static ssize_t store_noise_log(struct device *dev, const char *buf, size_t count)
{
	struct ft8756_data *d = to_ft8756_data(dev);

	int value = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if ((value > 1) || (value < 0)) {
		TOUCH_I("Set Noise log mode wrong, 0, 1 only\n");
		return count;
	}

	d->noise_info.noise_log = value ? NOISE_ENABLE : NOISE_DISABLE;

	TOUCH_I("noise_log = %s\n", (d->noise_info.noise_log == NOISE_ENABLE)
			? "NOISE_LOG_ENABLE" : "NOISE_LOG_DISABLE");

	return count;
}

static ssize_t show_noise_log(struct device *dev, char *buf)
{
	struct ft8756_data *d = to_ft8756_data(dev);

	int ret = 0;

	TOUCH_TRACE();

	ret = touch_snprintf(buf + ret, PAGE_SIZE, "noise_log = %d\n",
				d->noise_info.noise_log);

	TOUCH_I("noise_log = %s\n", (d->noise_info.noise_log == NOISE_ENABLE)
			? "NOISE_LOG_ENABLE" : "NOISE_LOG_DISABLE");

	return ret;
}

static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(tci_debug, show_tci_debug, store_tci_debug);
static TOUCH_ATTR(swipe_debug, show_swipe_debug, store_swipe_debug);
static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);
static TOUCH_ATTR(pinstate, show_pinstate, NULL);
static TOUCH_ATTR(noise_log, show_noise_log, store_noise_log);

static struct attribute *ft8756_attribute_list[] = {
	&touch_attr_reg_ctrl.attr,
	&touch_attr_tci_debug.attr,
	&touch_attr_swipe_debug.attr,
	&touch_attr_reset_ctrl.attr,
	&touch_attr_pinstate.attr,
	&touch_attr_noise_log.attr,
	NULL,
};

static const struct attribute_group ft8756_attribute_group = {
	.attrs = ft8756_attribute_list,
};

static int ft8756_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &ft8756_attribute_group);
	if (ret < 0)
		TOUCH_E("ft8756 sysfs register failed\n");

	ft8756_prd_register_sysfs(dev);

	return 0;
}

static int ft8756_get_cmd_version(struct device *dev, char *buf)
{
	struct ft8756_data *d = to_ft8756_data(dev);
	int offset = 0;

	TOUCH_TRACE();

	offset = touch_snprintf(buf + offset, PAGE_SIZE - offset, "IC firmware info\n");
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset, "Version : V%d.%02d\n\n",
		d->ic_info.is_official, d->ic_info.fw_version);

	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset, "Bin firmware info\n");
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset, "Version : V%d.%02d\n\n",
		d->ic_info.is_official_bin, d->ic_info.fw_version_bin);

	if (d->ic_info.chip_id == 0x80) {
		offset += touch_snprintf(buf + offset, PAGE_SIZE - offset, "Product-id : [FT800%xm]\n\n", d->ic_info.chip_id_low);
	}
	else if ((d->ic_info.chip_id == 0x87)&&(d->ic_info.chip_id_low == 0x56)){       //FT8756 ID_HGH 0X87, ID_LOW 0X56

		offset += touch_snprintf(buf + offset, PAGE_SIZE - offset, "Product-id : [FT8756]\n\n");

		}
	else {
		offset += touch_snprintf(buf + offset, PAGE_SIZE - offset, "Product-id : [Unknown]\n\n");
	}

	return offset;
}

static int ft8756_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct ft8756_data *d = to_ft8756_data(dev);
	int offset = 0;

	TOUCH_TRACE();

	offset = touch_snprintf(buf, PAGE_SIZE, "V%d.%02d\n",
		d->ic_info.is_official, d->ic_info.fw_version);

	return offset;
}

int ft8756_esd_recovery(struct device *dev)
{
	struct ft8756_data *d = to_ft8756_data(dev);
	int boot_mode = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	boot_mode = touch_check_boot_mode(dev);
	if (boot_mode == TOUCH_NORMAL_BOOT || d->lcd_mode != LCD_MODE_U0) {
#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
		lge_mdss_report_panel_dead();
#endif

#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
		mtkfb_esd_recovery();
#endif
	} else {
		TOUCH_I("esd skip, boot_mode : %d, lcd_mode : %d\n", boot_mode, d->lcd_mode);
	}

	return 0;
}

static int ft8756_swipe_enable(struct device *dev, bool enable)
{

	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0;
	int i = 0;
	bool retry_flag = false;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("IC status is deep_sleep\n");
		return 0;
	}

	if (d->state == TC_STATE_LPWG) {
		for (i = 0; i < 3; i++) {

			if (ts->swipe[SWIPE_L2].enable) {
				if (retry_flag || d->start_pay_area_L2.x1 != ts->swipe[SWIPE_L2].start_area.x1 ||
						d->start_pay_area_L2.x2 != ts->swipe[SWIPE_L2].start_area.x2 ||
						d->start_pay_area_L2.y1 != ts->swipe[SWIPE_L2].start_area.y1 ||
						d->start_pay_area_L2.y2 != ts->swipe[SWIPE_L2].start_area.y2) {

					d->start_pay_area_L2.x1 = ts->swipe[SWIPE_L2].start_area.x1;
					d->start_pay_area_L2.x2 = ts->swipe[SWIPE_L2].start_area.x2;
					d->start_pay_area_L2.y1 = ts->swipe[SWIPE_L2].start_area.y1;
					d->start_pay_area_L2.y2 = ts->swipe[SWIPE_L2].start_area.y2;

					ft8756_dummy_read(dev);

					ret = ft8756_lpwg_control(dev, ts->lpwg.mode, ts->swipe[SWIPE_L2].enable);
					TOUCH_I("L2 pay area setting, x1(%d), x2(%d), y1(%d), y2(%d)\n",
							d->start_pay_area_L2.x1, d->start_pay_area_L2.x2,
							d->start_pay_area_L2.y1, d->start_pay_area_L2.y2);
				}
			} else if(ts->swipe[SWIPE_R2].enable) {
				if (retry_flag || d->start_pay_area_R2.x1 != ts->swipe[SWIPE_R2].start_area.x1 ||
							d->start_pay_area_R2.x2 != ts->swipe[SWIPE_R2].start_area.x2 ||
							d->start_pay_area_R2.y1 != ts->swipe[SWIPE_R2].start_area.y1 ||
							d->start_pay_area_R2.y2 != ts->swipe[SWIPE_R2].start_area.y2) {

					d->start_pay_area_R2.x1 = ts->swipe[SWIPE_R2].start_area.x1;
					d->start_pay_area_R2.x2 = ts->swipe[SWIPE_R2].start_area.x2;
					d->start_pay_area_R2.y1 = ts->swipe[SWIPE_R2].start_area.y1;
					d->start_pay_area_R2.y2 = ts->swipe[SWIPE_R2].start_area.y2;

					ft8756_dummy_read(dev);

					ret = ft8756_lpwg_control(dev, ts->lpwg.mode, ts->swipe[SWIPE_R2].enable);
					TOUCH_I("R2 pay area setting, x1(%d), x2(%d), y1(%d), y2(%d)\n",
						d->start_pay_area_L2.x1, d->start_pay_area_L2.x2,
						d->start_pay_area_L2.y1, d->start_pay_area_L2.y2);
				}
			}

			if (ret == 0) {
				TOUCH_I("%s : setting Done!\n", __func__);
				break;
			} else {
				retry_flag = true;
				TOUCH_I("%s : retry(%d)!\n", __func__, i + 1);
			}
		}
	}

	return 0;
}

static int ft8756_init_pm(struct device *dev)
{
#if 0
	//Not used in add-on type (Only use in-cell model)

#if defined(CONFIG_FB)
	struct touch_core_data *ts = to_touch_core(dev);
#endif

	TOUCH_TRACE();

#if defined(CONFIG_DRM_MSM) && defined(CONFIG_FB) && defined(CONFIG_LGE_TOUCH_USE_PANEL_NOTIFY)
	TOUCH_I("%s: drm_notif change\n", __func__);
	ts->drm_notif.notifier_call = ili9881h_drm_notifier_callback;
#elif defined(CONFIG_FB)
	TOUCH_I("%s: fb_notif change\n", __func__);
	fb_unregister_client(&ts->fb_notif);
	ts->fb_notif.notifier_call = ili9881h_fb_notifier_callback;
	*/
#endif
#endif
	return 0;
}

static int ft8756_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int ft8756_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = ft8756_get_cmd_version(dev, (char *)output);
		break;

	case CMD_ATCMD_VERSION:
		ret = ft8756_get_cmd_atcmd_version(dev, (char *)output);
		break;

	default:
		break;
	}

	return ret;
}
static int ft8756_shutdown(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static struct touch_driver touch_driver = {
	.probe = ft8756_probe,
	.remove = ft8756_remove,
	.suspend = ft8756_suspend,
	.shutdown = ft8756_shutdown,
	.resume = ft8756_resume,
	.init = ft8756_init,
	.irq_handler = ft8756_irq_handler,
	.power = ft8756_power,
	.upgrade = ft8756_upgrade,
#if defined(CONFIG_LGE_TOUCH_APP_FW_UPGRADE)
	.app_upgrade = ft8756_app_upgrade,
#endif
	.esd_recovery = ft8756_esd_recovery,
	.lpwg = ft8756_lpwg,
	.swipe_enable = ft8756_swipe_enable,
	.notify = ft8756_notify,
	.init_pm = ft8756_init_pm,
	.register_sysfs = ft8756_register_sysfs,
	.set = ft8756_set,
	.get = ft8756_get,
};

#define MATCH_NAME			"focaltech_lge,ft8756_lge"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{},
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(touch_match_ids),
};

static int __init touch_device_init(void)
{
	TOUCH_I("FT8756__[%s] touch_bus_device_init\n", __func__);
/*
	if(is_lcm_name("AUO-FT8756")) //MH45, DH30
	{
		TOUCH_I("%s, FT8756 found!!lcm_name = %s\n",__func__,lge_get_lcm_name());
		return touch_bus_device_init(&hwif, &touch_driver);
	}

	TOUCH_I("%s, FT8756 not found!!\n",__func__);

	return 0;
*/
	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("BSP-TOUCH@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");
