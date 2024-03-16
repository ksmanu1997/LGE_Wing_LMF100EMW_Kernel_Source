/* touch_sw49107.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: BSP-TOUCH@lge.com
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#include <soc/qcom/lge/board_lge.h>
#endif
#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
#include <soc/mediatek/lge/board_lge.h>
#endif
#include <touch_core.h>
#include <touch_hwif.h>

#include "touch_sw49107.h"
#include "touch_sw49107_prd.h"

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#include <linux/msm_lcd_power_mode.h>
#endif
#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
#include <linux/lcd_power_mode.h>
#endif
#if defined(CONFIG_LGE_TOUCH_MODULE_DETECT)
int panel_id = 0;
#endif

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY)
#include <linux/msm_lcd_recovery.h>
#endif
#endif
#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
extern void mtkfb_esd_recovery(void);
#endif

static void project_param_set(struct device *dev)
{
	struct sw49107_data *d = to_sw49107_data(dev);

	d->p_param.chip_id = VCHIP_VAL;
	d->p_param.protocol = VPROTO_VAL;

	/* if (F/W IMAGE = Code area + CFG area) = FUNC_ON */
	d->p_param.fw_cfg_use_en = FUNC_ON;

	/* flash_fw_size = F/W IMAGE SIZE - CFG SIZE */
	d->p_param.flash_fw_size = (84 * 1024);

	d->p_param.used_mode = (1<<LCD_MODE_U0)|(1<<LCD_MODE_U3)|(1<<LCD_MODE_U3_QUICKCOVER)|
			       (1<<LCD_MODE_STOP)|(1<<LCD_MODE_U3_PARTIAL);

	d->p_param.osc_clock_ctrl = FUNC_OFF;

	/* incell touch driver have no authority to control vdd&vio power pin */
	d->p_param.touch_power_control_en = FUNC_OFF;

	/* (AT_TOUCH_FW:tci settings are in Touch F/W.)  (AT_AP:tci settings are in AP.) */
	//d->p_param.tci_setting = AT_TOUCH_FW; //49107 tci settings are in the Touch F/W.

	/* Read flexible I2C data when N-finger come in.  */
	d->p_param.flex_report = FUNC_ON;

	/* Use two normalize table for differnece in panel sensitiviy issue
	 * Board rev above 1.1		: Use NEW_SAMPLE_TUNE_TABLE
	 * Board rev A, B, C, 1.0	: Use OLD_SAMPLE_TUNE_TABLE */
	d->p_param.dynamic_tune_table = FUNC_ON;

	/* Use DSV Toggle mode */
	d->p_param.dsv_toggle = FUNC_ON;

	/* Use Power control when deep sleep */
	d->p_param.deep_sleep_power_control = FUNC_OFF;

	/* Use Palm Log */
	d->p_param.palm_log = FUNC_OFF;
}

#define LPWG_FAILREASON_TCI_NUM 10
static const char const *lpwg_failreason_tci_str[LPWG_FAILREASON_TCI_NUM] = {
	[0] = "SUCCESS",
	[1] = "DISTANCE_INTER_TAP",
	[2] = "DISTANCE_TOUCHSLOP",
	[3] = "MINTIMEOUT_INTER_TAP",
	[4] = "MAXTIMEOUT_INTER_TAP",
	[5] = "LONGPRESS_TIME_OUT",
	[6] = "MULTI_FINGER",
	[7] = "DELAY_TIME", /* Over Tap */
	[8] = "PALM_STATE",
	[9] = "OUTOF_AREA",
};

#define LPWG_FAILREASON_SWIPE_NUM 11
static const char const *lpwg_failreason_swipe_str[LPWG_FAILREASON_SWIPE_NUM] = {
	[0] = "ERROR",
	[1] = "FINGER_FAST_RELEASE",
	[2] = "MULTI_FINGER",
	[3] = "FAST_SWIPE",
	[4] = "SLOW_SWIPE",
	[5] = "WRONG_DIRECTION",
	[6] = "RATIO_FAIL",
	[7] = "OUT_OF_START_AREA",
	[8] = "OUT_OF_ACTIVE_AREA",
	[9] = "INITAIL_RATIO_FAIL",
	[10] = "PALM_STATE",
};

#define DEBUG_INFO_NUM 32
static const char *debug_info_str[DEBUG_INFO_NUM] = {
	[0] = "NONE",
	[1] = "DBG_TG_FAULT",
	[2] = "DBG_ESD_FAULT",
	[3] = "DBG_WATDOG_TIMEOUT",
	[4] = "DBG_TC_DRV_MISMATCH",
	[5] = "DBG_TC_INVALID_TIME_DRV_REQ",
	[6] = "DBG_AFE_TUNE_FAIL",
	[7] = "DBG_DBG_MSG_FULL",
	[8] = "DBG_PRE_MA_OVF_ERR",
	[9] = "DBG_ADC_OVF_ERR",
	[10] = "DBG_CM3_FAULT",			// 0x0A
	[11] = "DBG_UNKNOWN_TEST_MSG [0x0B]",	// 0x0B
	[12] = "DBG_FLASH_EDTECT_ERR",		// 0x0C
	[13] = "DBG_MEM_ACCESS_ISR",		// 0x0D
	[14] = "DBG_DISPLAY_CHANGE_IRQ",	// 0x0E
	[15] = "DBG_PT_CHKSUM_ERR",		// 0x0F
	[16] = "DBG_UNKNOWN_CMD",		// 0x10
	[17] = "DBG_TE_FREQ_REPORT",		// 0x11
	[18] = "DBG_STACK_OVERFLOW_ERR",	// 0x12
	[19] = "DBG_ABNORMAL_ACCESS_ERR",	// 0x13
	[20] = "DBG_UNKNOWN_TEST_MSG [0x14]",	// 0x14
	[21] = "DBG_UNKNOWN_TEST_MSG [0x15]",	// 0x15
	[22] = "DBG_CG_CTL_INT",		// 0x16
	[23] = "DBG_DCS_IRQ",			// 0x17
	[24] = "DBG_DISPLAY_IRQ",		// 0x18
	[25] = "DBG_USER1_IRQ",			// 0x19
	[26] = "DBG_CMD_Q_FULL",		// 0x1A
	[27] = "DBG_TC_DRV_START_SKIP",		// 0x1B
	[28] = "DBG_TC_DRV_CMD_INVALID",	// 0x1C
	[29] = "DBG_UNKNOWN_TEST_MSG [0x1D]",	// 0x1D
	[30] = "DBG_CFG_S_IDX",			// 0x1E
	[31] = "DBG_UNKNOWN_TEST_MSG [0x1F]",	// 0x1F
};

#define IC_STATUS_INFO_NUM 13
static const int ic_status_info_idx[IC_STATUS_INFO_NUM] = {1, 2, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const char *ic_status_info_str[32] = {
	[1] = "ESD Right Side Detection",
	[2] = "ESD Left Side Detection",
	[5] = "Watchdog Timer Expired",
	[7] = "CM3 Fault Status",
	[8] = "DIC MIPI Interface Error",
	[9] = "DIC Check Sum Error",
	[10] = "DSC Decoder Input Buffer Overflow",
	[11] = "DSC Decoder Input Buffer Underflow",
	[12] = "DSC Decoder Input Buffer Chunk Size Mismatch",
	[13] = "DSC Decoder RC Buffer Overflow",
	[14] = "Flash Magic Number Check Error",
	[15] = "Flash Code Dump Error",
	[16] = "Display APOD Signal Detection",
};

#define TC_STATUS_INFO_NUM 13
static const int tc_status_info_idx[TC_STATUS_INFO_NUM] = {5, 6, 7, 9, 10, 13, 15, 20, 21, 22, 27, 28, 31};
static const char *tc_status_info_str[32] = {
	[5] = "Device Check Failed",
	[6] = "Code CRC Invalid",
	[7] = "Config CRC Invalid",
	[9] = "Abnormal Status Detected",
	[10] = "ESD System Error Detected",
	[13] = "Display Mode Mismatched",
	[15] = "Low Active Interrupt Pin is High",
	[20] = "Touch Interrupt Disabled",
	[21] = "Touch Memory Crashed",
	[22] = "TC Driving Invalid",
	[27] = "Model ID Not Loaded",
	[28] = "Production Test info checksum error",
	[31] = "Display Abnormal Status",
};

#define LCD_POWER_MODE_NUM 4
static const char const *lcd_power_mode_str[LCD_POWER_MODE_NUM] = {
	[0] = "DEEP_SLEEP_ENTER",
	[1] = "DEEP_SLEEP_EXIT",
	[2] = "DSV_TOGGLE",
	[3] = "DSV_ALWAYS_ON",
};

int sw49107_reg_read(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	struct touch_bus_msg msg = {0, };
	int ret = 0;

/*Temp*/
	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		TOUCH_I("%s : FB_SUSPEMD", __func__);
		return 0;
	}

	mutex_lock(&d->io_lock);
	ts->tx_buf[0] = ((size > 4) ? 0x20 : 0x00);
	ts->tx_buf[0] |= ((addr >> 8) & 0x0f);
	ts->tx_buf[1] = (addr & 0xff);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = W_HEADER_SIZE;
	msg.rx_buf = ts->rx_buf;
	msg.rx_size = R_HEADER_SIZE + size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus error : %d\n", ret);
		mutex_unlock(&d->io_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[R_HEADER_SIZE], size);
	mutex_unlock(&d->io_lock);
	return 0;
}

int sw49107_reg_write(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	struct touch_bus_msg msg = {0, };
	int ret = 0;

	/*Temp*/
	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		TOUCH_I("%s : FB_SUSPEMD", __func__);
		return 0;
	}

	mutex_lock(&d->io_lock);
	ts->tx_buf[0] = ((size > 4) ? 0x60 : 0x40);
	ts->tx_buf[0] |= ((addr >> 8) & 0x0f);
	ts->tx_buf[1] = (addr  & 0xff);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = W_HEADER_SIZE + size;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	memcpy(&ts->tx_buf[W_HEADER_SIZE], data, size);

	ret = touch_bus_write(dev, &msg);
	mutex_unlock(&d->io_lock);

	if (ret < 0) {
		TOUCH_E("touch bus error : %d\n", ret);
		return ret;
	}

	return 0;
}

static int sw49107_sw_reset(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0;

	TOUCH_I("%s : SW Reset(mode%d)\n", __func__, mode);

	if(mode == SW_RESET) {
		/* Touch F/W jump reset vector and do not code flash dump */
		sw49107_write_value(dev, sys_rst_ctl, 2);
		touch_msleep(20);
		sw49107_write_value(dev, sys_rst_ctl, 0);
	} else if(mode == SW_RESET_CODE_DUMP) {
		/* Touch F/W resister reset and do code flash dump */
		sw49107_write_value(dev, sys_rst_ctl, 1);
		touch_msleep(20);
		sw49107_write_value(dev, sys_rst_ctl, 0);
	} else {
		TOUCH_E("%s Invalid SW reset mode!!\n", __func__);
	}

	atomic_set(&d->init, IC_INIT_NEED);

	queue_delayed_work(ts->wq, &ts->init_work, msecs_to_jiffies(ts->caps.sw_reset_delay));

	return ret;
}

int sw49107_hw_reset(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);

	TOUCH_I("%s : HW Reset(mode:%d)\n", __func__, mode);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_msleep(1);

	touch_gpio_direction_output(ts->reset_pin, 1);
	atomic_set(&d->init, IC_INIT_NEED);

	if (mode == POWER_HW_RESET_ASYNC){
		queue_delayed_work(ts->wq, &ts->init_work, msecs_to_jiffies(ts->caps.hw_reset_delay));
	} else if(mode == POWER_HW_RESET_SYNC) {
		touch_msleep(ts->caps.hw_reset_delay);
		ts->driver->init(dev);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	} else {
		TOUCH_E("%s Invalid HW reset mode!!\n", __func__);
	}

	return 0;
}

int sw49107_reset_ctrl(struct device *dev, int ctrl)
{
	TOUCH_TRACE();

	switch (ctrl) {
		default :
		case SW_RESET:
		case SW_RESET_CODE_DUMP:
			sw49107_sw_reset(dev, ctrl);
			break;

		case POWER_HW_RESET_ASYNC:
		case POWER_HW_RESET_SYNC:
			sw49107_hw_reset(dev, ctrl);
			break;
	}

	return 0;
}

int sw49107_display_power_ctrl(struct device *dev, int ctrl)
{
	TOUCH_TRACE();

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
	//lge_panel_set_power_mode(ctrl); //bring up
#elif defined(CONFIG_LGE_TOUCH_CORE_MTK)
	primary_display_set_power_mode(ctrl);
#endif

	if (ctrl >= DEEP_SLEEP_ENTER && ctrl <= DSV_ALWAYS_ON) {
		TOUCH_I("%s, mode : %s\n", __func__, lcd_power_mode_str[ctrl]);
	}

	return 0;
}

static int sw49107_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	TOUCH_TRACE();

	switch (ctrl) {
		case POWER_OFF:
			if(d->p_param.touch_power_control_en == FUNC_ON) {
				atomic_set(&d->init, IC_INIT_NEED);
				touch_gpio_direction_output(ts->reset_pin, 0);
				touch_power_1_8_vdd(dev, 0);
				touch_power_3_3_vcl(dev, 0);
				touch_msleep(1);
			} else {
				TOUCH_I("%s, off Not Supported\n", __func__);
			}
			break;

		case POWER_ON:
			if(d->p_param.touch_power_control_en == FUNC_ON) {
				touch_power_3_3_vcl(dev, 1);
				touch_power_1_8_vdd(dev, 1);
				touch_gpio_direction_output(ts->reset_pin, 1);
			} else {
				TOUCH_I("%s, on Not Supported\n", __func__);
				if (atomic_read(&ts->state.core) == CORE_UPGRADE) {
					TOUCH_I("%s, Need to H/W reset after upgrade\n", __func__);
					sw49107_reset_ctrl(dev, POWER_HW_RESET_SYNC);
				}
			}
			break;
		case POWER_HW_RESET_SYNC:
			TOUCH_I("%s, reset\n", __func__);
			sw49107_reset_ctrl(dev, POWER_HW_RESET_SYNC);
			break;
		case POWER_SW_RESET:
			sw49107_reset_ctrl(dev, SW_RESET);
			break;
#if 0 //bring up
		case POWER_SLEEP:
			if (d->p_param.deep_sleep_power_control == FUNC_ON) {
				sw49107_display_power_ctrl(dev, DEEP_SLEEP_ENTER);
			}
			break;
		case POWER_WAKE:
			if (d->p_param.deep_sleep_power_control == FUNC_ON) {
				sw49107_display_power_ctrl(dev, DEEP_SLEEP_EXIT);
			}
			break;
		case POWER_DSV_TOGGLE:
			if (d->p_param.dsv_toggle == FUNC_ON) {
				sw49107_display_power_ctrl(dev, DSV_TOGGLE);
			}
			break;
		case POWER_DSV_ALWAYS_ON:
			if (d->p_param.dsv_toggle == FUNC_ON) {
				sw49107_display_power_ctrl(dev, DSV_ALWAYS_ON);
			}
			break;
		default:
			TOUCH_I("%s, Not Supported. case: %d\n", __func__, ctrl);
			break;
#endif
	}
	return 0;
}

static void sw49107_get_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int mm_to_point = 15; // 1 mm -> about X point

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 6;
	ts->tci.info[TCI_1].max_intertap = 70;
	ts->tci.info[TCI_1].touch_slop = 100;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].min_intertap = 6;
	ts->tci.info[TCI_2].max_intertap = 70;
	ts->tci.info[TCI_2].touch_slop = 100;
	ts->tci.info[TCI_2].tap_distance = 255;
	ts->tci.info[TCI_2].intr_delay = 20;

	ts->tci.area.x1 = 0 + (mm_to_point * 5);				// spec 3~5mm
	ts->tci.area.y1 = 0 + (mm_to_point * 5);				// spec 3~5mm
	ts->tci.area.x2 = ts->caps.max_x - (mm_to_point * 5);			// spec 3~5mm
	ts->tci.area.y2 = ts->caps.max_y - (mm_to_point * 5);			// spec 3~5mm

/* check build error (not use)
	ts->tci.cover_area.x1 = 0 + (mm_to_point * 5);
	ts->tci.cover_area.y1 = 0 + (mm_to_point * 5);
	ts->tci.cover_area.x2 = ts->caps.max_x - (mm_to_point * 5);
	ts->tci.cover_area.y2 = ts->caps.max_y - (mm_to_point * 5);
*/
}

static void sw49107_get_swipe_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int mm_to_point = 15; // 1 mm -> about X point

	ts->swipe[SWIPE_L].enable = false;
	ts->swipe[SWIPE_L].distance = 7;
	ts->swipe[SWIPE_L].ratio_thres = 100;
	ts->swipe[SWIPE_L].min_time = 0;
	ts->swipe[SWIPE_L].max_time = 150;
	ts->swipe[SWIPE_L].wrong_dir_thres = 2;
	ts->swipe[SWIPE_L].init_ratio_chk_dist = 2;
	ts->swipe[SWIPE_L].init_ratio_thres = 100;
	ts->swipe[SWIPE_L].area.x1 = 0;
	ts->swipe[SWIPE_L].area.y1 = 305;
	ts->swipe[SWIPE_L].area.x2 = (ts->caps.max_x);
	ts->swipe[SWIPE_L].area.y2 = 1655;
	ts->swipe[SWIPE_L].start_area.x1 = ((ts->caps.max_x) - (mm_to_point * 4));
	ts->swipe[SWIPE_L].start_area.y1 = 305;
	ts->swipe[SWIPE_L].start_area.x2 = (ts->caps.max_x);
	ts->swipe[SWIPE_L].start_area.y2 = 1655;

	ts->swipe[SWIPE_R].enable = false;
	ts->swipe[SWIPE_R].distance = 7;
	ts->swipe[SWIPE_R].ratio_thres = 100;
	ts->swipe[SWIPE_R].min_time = 0;
	ts->swipe[SWIPE_R].max_time = 150;
	ts->swipe[SWIPE_R].wrong_dir_thres = 2;
	ts->swipe[SWIPE_R].init_ratio_chk_dist = 2;
	ts->swipe[SWIPE_R].init_ratio_thres = 100;
	ts->swipe[SWIPE_R].area.x1 = 0;
	ts->swipe[SWIPE_R].area.y1 = 305;
	ts->swipe[SWIPE_R].area.x2 = (ts->caps.max_x);
	ts->swipe[SWIPE_R].area.y2 = 1655;
	ts->swipe[SWIPE_R].start_area.x1 = 0;
	ts->swipe[SWIPE_R].start_area.y1 = 305;
	ts->swipe[SWIPE_R].start_area.x2 = (mm_to_point * 4);
	ts->swipe[SWIPE_R].start_area.y2 = 1655;

	ts->swipe[SWIPE_U].enable = false;
	ts->swipe[SWIPE_U].distance = 20;
	ts->swipe[SWIPE_U].ratio_thres = 150;
	ts->swipe[SWIPE_U].min_time = 4;
	ts->swipe[SWIPE_U].max_time = 150;
	ts->swipe[SWIPE_U].wrong_dir_thres = 5;
	ts->swipe[SWIPE_U].init_ratio_chk_dist = 4;
	ts->swipe[SWIPE_U].init_ratio_thres = 100;
	ts->swipe[SWIPE_U].area.x1 = 0 + (mm_to_point * 4);					// spec 4mm
	ts->swipe[SWIPE_U].area.y1 = 0;							// spec 0mm
	ts->swipe[SWIPE_U].area.x2 = ts->caps.max_x - (mm_to_point * 4);			// spec 4mm
	ts->swipe[SWIPE_U].area.y2 = ts->caps.max_y;					// spec 0mm
	ts->swipe[SWIPE_U].start_area.x1 = (ts->caps.max_x / 2) - (int)(mm_to_point * 12.5);	// spec start_area_width 25mm
	ts->swipe[SWIPE_U].start_area.y1 = ts->swipe[SWIPE_U].area.y2 - (int)(mm_to_point * 14.5);	// spec start_area_height 14.5mm
	ts->swipe[SWIPE_U].start_area.x2 = (ts->caps.max_x / 2) + (int)(mm_to_point * 12.5);	// spec start_area_width 25mm
	ts->swipe[SWIPE_U].start_area.y2 = ts->swipe[SWIPE_U].area.y2;

	ts->swipe[SWIPE_D].enable = false;
	ts->swipe[SWIPE_D].distance = 15;
	ts->swipe[SWIPE_D].ratio_thres = 150;
	ts->swipe[SWIPE_D].min_time = 0;
	ts->swipe[SWIPE_D].max_time = 150;
	ts->swipe[SWIPE_D].wrong_dir_thres = 5;
	ts->swipe[SWIPE_D].init_ratio_chk_dist = 5;
	ts->swipe[SWIPE_D].init_ratio_thres = 100;
	ts->swipe[SWIPE_D].area.x1 = 0 + (mm_to_point * 4);					// spec 4mm
	ts->swipe[SWIPE_D].area.y1 = 0;							// spec 0mm
	ts->swipe[SWIPE_D].area.x2 = ts->caps.max_x - (mm_to_point * 4);			// spec 4mm
	ts->swipe[SWIPE_D].area.y2 = ts->caps.max_y;					// spec 0mm
	ts->swipe[SWIPE_D].start_area.x1 = 0 + (mm_to_point * 4);					// spec 4mm
	ts->swipe[SWIPE_D].start_area.y1 = 0;							// spec 0mm
	ts->swipe[SWIPE_D].start_area.x2 = ts->caps.max_x - (mm_to_point * 4);			// spec 4mm
	ts->swipe[SWIPE_D].start_area.y2 = 200;

}

int sw49107_ic_info(struct device *dev)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	u32 bootmode = 0;
	u32 ic_status = 0;

	memset(&d->ic_info, 0, sizeof(d->ic_info));
	ret = sw49107_reg_read(dev, TC_VERSION, &d->ic_info.version, sizeof(d->ic_info.version));
	if (ret < 0) {
		TOUCH_E("TC_VERSION read failed %d, version: v%d.%02d\n", ret,
				d->ic_info.version.major_ver, d->ic_info.version.major_ver);
		return ret;
	}

	ret = sw49107_reg_read(dev, TC_PRODUCT_ID1, &d->ic_info.product_id, sizeof(d->ic_info.product_id));
	if (ret < 0) {
		TOUCH_E("TC_PRODUCT_ID1 read failed : %d.\n", ret);
		return ret;
	}
	d->ic_info.product_id[sizeof(d->ic_info.product_id) - 1]  = '\0';
	ret = sw49107_reg_read(dev, PT_INFO_LCM_TYPE, &d->ic_info.pt_info, sizeof(d->ic_info.pt_info));
	if (ret < 0) {
		TOUCH_E("PT_INFO_LCM_TYPE read failed : %d.\n", ret);
		return ret;
	}
	ret = sw49107_reg_read(dev, SPR_BOOT_ST, &bootmode, sizeof(bootmode));
	if (ret < 0) {
		TOUCH_E("SPR_BOOT_ST read failed : %d.\n", ret);
		return ret;
	}

	TOUCH_I("==================== Version Info ====================\n");
	TOUCH_I("version: v%d.%02d, build: %d, chip id: %d, protocol: %d\n",
			d->ic_info.version.major_ver, d->ic_info.version.minor_ver, d->ic_info.version.build_ver,
			d->ic_info.version.chip_id, d->ic_info.version.protocol_ver);
	TOUCH_I("product id: [%s], chip rev: %x\n", d->ic_info.product_id, d->ic_info.pt_info.chip_rev);
	TOUCH_I("lcm type: %d, lot_num: %d, fpc_type: %d\n",
			d->ic_info.pt_info.lcm_type, d->ic_info.pt_info.lot_num, d->ic_info.pt_info.fpc_type);
	TOUCH_I("date: %04d.%02d.%02d, time: %02d:%02d:%02d, site: %d\n",
			d->ic_info.pt_info.pt_date_year, d->ic_info.pt_info.pt_date_month, d->ic_info.pt_info.pt_date_day,
			d->ic_info.pt_info.pt_time_hour, d->ic_info.pt_info.pt_time_min, d->ic_info.pt_info.pt_time_sec,
			d->ic_info.pt_info.pt_site);
	TOUCH_I("flash boot : %s, %s, crc : %s\n",
			(bootmode >> 0 & 0x1) ? "BUSY" : "idle",
			(bootmode >> 1 & 0x1) ? "done" : "booting",
			(bootmode >> 2 & 0x1) ? "ERROR" : "ok");
	TOUCH_I("======================================================\n");

	// Error handling (Required code)
	if (((d->ic_info.version.chip_id != d->p_param.chip_id) || (d->ic_info.version.protocol_ver != d->p_param.protocol))
			&& (atomic_read(&ts->state.core) != CORE_PROBE)) {
		if(d->boot_err_cnt > 5) {
			TOUCH_I("FW is in abnormal state because of ESD or something. MAX Retry Over (cnt:%d)\n"
					, d->boot_err_cnt);
			return -1;
		} else {
			d->boot_err_cnt++;
			TOUCH_I("FW is in abnormal state because of ESD or something. (cnt:%d)\n"
					, d->boot_err_cnt);

			sw49107_reset_ctrl(dev, POWER_HW_RESET_ASYNC);
		}
	}

	// Error handling when interrupt is low after ic init
	if (gpio_get_value(ts->int_pin) == 0) {
		ret = sw49107_reg_read(dev, IC_STATUS, &ic_status, sizeof(ic_status));
		if (ret < 0) {
			TOUCH_E("ic_status read failed %d\n", ret);
			return ret;
		}
		// IC_STATUS : ESD, Watchdog, DIC MIPI Interface, APOD -> Reset
		if (ic_status & 0x10126) {
			TOUCH_I("Call reset - ic_status t %x, lcd_mode : %d\n", ic_status, d->lcd_mode);
			if (atomic_read(&ts->state.fb) == FB_SUSPEND)
				sw49107_reset_ctrl(dev, POWER_HW_RESET_SYNC);
			else
				queue_delayed_work(ts->wq, &ts->panel_reset_work, 0);

			return -1;
		}
	}

	return ret;
}

static int sw49107_setup_q_sensitivity(struct device *dev, int enable)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	d->q_sensitivity = enable; /* 1=enable touch, 0=disable touch */

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
		goto out;

	ret = sw49107_reg_write(dev, COVER_SENSITIVITY_CTRL, &d->q_sensitivity, sizeof(u32));

out:
	TOUCH_I("%s : %s(%d)\n", __func__,
			(d->q_sensitivity) ? "SENSITIVE" : "NORMAL", (d->q_sensitivity));

	return ret;
}

static int sw49107_get_tci_data(struct device *dev, int count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	u8 i = 0;
	u32 rdata[MAX_LPWG_CODE];

	if (!count)
		return 0;

	ts->lpwg.code_num = count;

	memcpy(&rdata, d->info.data, sizeof(u32) * count);

	for (i = 0; i < count; i++) {
		ts->lpwg.code[i].x = rdata[i] & 0xffff;
		ts->lpwg.code[i].y = (rdata[i] >> 16) & 0xffff;

		if (ts->lpwg.mode >= LPWG_PASSWORD)
			TOUCH_I("LPWG data xxxx, xxxx\n");
		else
			TOUCH_I("LPWG data %d, %d\n", ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	return 0;
}

static int sw49107_get_swipe_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	u32 rdata[3];

	/* start (X, Y), end (X, Y), time = 2bytes * 5 = 10 bytes */
	memcpy(&rdata, d->info.data, sizeof(u32) * 3);

	TOUCH_I("Swipe Gesture: start(%4d,%4d) end(%4d,%4d) swipe_time(%dms)\n",
			rdata[0] & 0xffff, rdata[0] >> 16,
			rdata[1] & 0xffff, rdata[1] >> 16,
			rdata[2] & 0xffff);

	ts->lpwg.code_num = 1;
	ts->lpwg.code[0].x = rdata[1] & 0xffff;
	ts->lpwg.code[0].y = rdata[1] >> 16;

	ts->lpwg.code[1].x = -1;
	ts->lpwg.code[1].y = -1;

	return 0;
}

static int sw49107_lpwg_failreason_ctrl(struct device *dev, int onoff)
{
	int ret = 0;

	TOUCH_I("%s - Failreason %s\n", __func__, onoff ? "Enable" : "Disable");

	ret = sw49107_reg_write(dev, LPWG_FAILREASON_ON_CTRL, &onoff, sizeof(onoff));

	return ret;
}

static int sw49107_tci_active_area(struct device *dev,
		u32 x1, u32 y1, u32 x2, u32 y2)
{
	int ret = 0, i;
	u32 active_area[4] = {x1, y1, x2, y2};

	TOUCH_I("%s - active_area: x1[%d], y1[%d], x2[%d], y2[%d]\n", __func__,
			active_area[0], active_area[1], active_area[2], active_area[3]);

	for (i=0; i < sizeof(active_area)/sizeof(u32); i++)
		active_area[i] = (active_area[i]) | (active_area[i] << 16);

	ret = sw49107_reg_write(dev, TCI_ACTIVE_AREA_X1_CTRL, &active_area[0], sizeof(active_area));

	return ret;
}

static int sw49107_tci_active_area_set(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (ts->lpwg.qcover) {
		TOUCH_I("%s - QUICKCOVER_CLOSE\n", __func__);

/* check build error (not use)
		ret = sw49107_tci_active_area(dev,
				ts->tci.cover_area.x1, ts->tci.cover_area.y1,
				ts->tci.cover_area.x2, ts->tci.cover_area.y2);
*/
	} else {
		TOUCH_I("%s - LPWG Active Area - NORMAL\n", __func__);

		ret = sw49107_tci_active_area(dev,
				ts->tci.area.x1, ts->tci.area.y1,
				ts->tci.area.x2, ts->tci.area.y2);
	}

	return ret;
}

static int sw49107_tci_command(struct device *dev, int cmd)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	struct tci_info *info2 = &ts->tci.info[TCI_2];
	u32 lpwg_data;
	int ret = 0;

	TOUCH_I("%s - cmd : %d\n", __func__, cmd);

	switch (cmd) {
		case ENABLE_CTRL:
			lpwg_data = ts->tci.mode;
			ret = sw49107_reg_write(dev, TCI_ENABLE_CTRL, &lpwg_data, sizeof(lpwg_data));
			break;

		case TAP_COUNT_CTRL:
			lpwg_data = info1->tap_count | (info2->tap_count << 16);
			ret = sw49107_reg_write(dev, TCI_TOTAL_TAP_COUNT_CTRL, &lpwg_data, sizeof(lpwg_data));
			break;

		case MIN_INTERTAP_CTRL:
			lpwg_data = info1->min_intertap | (info2->min_intertap << 16);
			ret = sw49107_reg_write(dev, TCI_INTER_TAP_TIME_MIN_CTRL, &lpwg_data, sizeof(lpwg_data));
			break;

		case MAX_INTERTAP_CTRL:
			lpwg_data = info1->max_intertap | (info2->max_intertap << 16);
			ret = sw49107_reg_write(dev, TCI_INTER_TAP_TIME_MAX_CTRL, &lpwg_data, sizeof(lpwg_data));
			break;

		case TOUCH_SLOP_CTRL:
			lpwg_data = info1->touch_slop | (info2->touch_slop << 16);
			ret = sw49107_reg_write(dev, TCI_INNER_TAP_DIST_MAX_CTRL, &lpwg_data, sizeof(lpwg_data));
			break;

		case TAP_DISTANCE_CTRL:
			lpwg_data = info1->tap_distance | (info2->tap_distance << 16);
			ret = sw49107_reg_write(dev, TCI_INTER_TAP_DISP_MAX_CTRL, &lpwg_data, sizeof(lpwg_data));
			break;

		case INTERRUPT_DELAY_CTRL:
			lpwg_data = info1->intr_delay | (info2->intr_delay << 16);
			ret = sw49107_reg_write(dev, TCI_INTERRUPT_DELAY_TIME_CTRL, &lpwg_data, sizeof(lpwg_data));
			break;

		case ACTIVE_AREA_CTRL:
			ret = sw49107_tci_active_area_set(dev);
			break;

		default:
			break;
	}

	return ret;
}

static int sw49107_tci_knock(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	struct tci_info *info2 = &ts->tci.info[TCI_2];
	u32 lpwg_data[7];
	int ret = 0;

	lpwg_data[0] = ts->tci.mode;
	lpwg_data[1] = info1->tap_count | (info2->tap_count << 16);
	lpwg_data[2] = info1->min_intertap | (info2->min_intertap << 16);
	lpwg_data[3] = info1->max_intertap | (info2->max_intertap << 16);
	lpwg_data[4] = info1->touch_slop | (info2->touch_slop << 16);
	lpwg_data[5] = info1->tap_distance | (info2->tap_distance << 16);
	lpwg_data[6] = info1->intr_delay | (info2->intr_delay << 16);
	ret = sw49107_reg_write(dev, TCI_ENABLE_CTRL, &lpwg_data[0], sizeof(lpwg_data));

	ret = sw49107_tci_command(dev, ACTIVE_AREA_CTRL);
	ret = sw49107_lpwg_failreason_ctrl(dev, d->lpwg_failreason_ctrl);

	return ret;
}

#ifdef LG_KNOCK_CODE
static int sw49107_tci_password(struct device *dev)
{
	return sw49107_tci_knock(dev);
}
#endif

static int sw49107_tci_control(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	int ret = 0;
	char* mode_str[4] = {"None", "Knock-On", "Knock-On/Code", "Knock-Code"};

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("Skip tci control in deep sleep\n");
		return 0;
	}

	if (mode <= LPWG_PASSWORD_ONLY)
		TOUCH_I("sw49107_tci_control mode = %s\n", mode_str[mode]);

	switch (mode) {
		case LPWG_NONE:
			ts->tci.mode = 0;
			ret = sw49107_tci_command(dev, ENABLE_CTRL);
			break;
		case LPWG_DOUBLE_TAP:
			ts->tci.mode = 0x01;
			info1->intr_delay = 0;
			info1->tap_distance = 10;

			ret = sw49107_tci_knock(dev);
			break;
#ifdef LG_KNOCK_CODE
		case LPWG_PASSWORD:
			ts->tci.mode = 0x01 | (0x01 << 16);
			info1->intr_delay = ts->tci.double_tap_check ? 68 : 0;
			info1->tap_distance = 7;

			ret = sw49107_tci_password(dev);
			break;
		case LPWG_PASSWORD_ONLY:
			ts->tci.mode = 0x01 << 16;
			info1->intr_delay = 0;
			info1->tap_distance = 10;

			ret = sw49107_tci_password(dev);
			break;
#endif
		default:
			TOUCH_I("Unknown tci control case\n");
			ret = -1;
			break;
	}

	return ret;
}

static int sw49107_swipe_control(struct device *dev, u8 lcd_mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	struct swipe_ctrl *ctrl[4] = {	/* L, R, U, D */
		&ts->swipe[SWIPE_L],
		&ts->swipe[SWIPE_R],
		&ts->swipe[SWIPE_U],
		&ts->swipe[SWIPE_D],
	};
	struct swipe_buf {
		u8 enable[4];
		u8 distance[4];
		u8 ratio_thres[4];
		u16 min_time[4];
		u16 max_time[4];
		u16 area_hori[8];
		u16 area_verti[8];
		u16 start_hori[8];
		u16 start_verti[8];
		u8 wrong_dir_thres[4];
		u8 init_ratio_chk_dist[4];
		u8 init_ratio_thres[4];
	} __packed;
	struct swipe_buf buf;
	int i, ret = 0;

	TOUCH_I("%s: SWIPE L(%d), R(%d), U(%d), D(%d)\n", __func__,
		ts->swipe[SWIPE_L].enable, ts->swipe[SWIPE_R].enable,
		ts->swipe[SWIPE_U].enable, ts->swipe[SWIPE_D].enable);

	if (lcd_mode == LCD_MODE_U3) {
		TOUCH_I("%s swipe[SWIPE_L/R/U/D] = %s\n", __func__, "Disable");
		memset(&(buf.enable), 0, sizeof(buf.enable));

		ret = sw49107_reg_write(dev, SWIPE_ON_CTRL,
				&(buf.enable), sizeof(buf.enable));
		if (ret < 0)
			TOUCH_E("failed to clear SWIPE_ENABLE register (ret = %d)\n", ret);
	} else {
		memset(&buf, 0, sizeof(buf));

		TOUCH_I("%s swipe[SWIPE_L/R/U/D] = %s\n", __func__,  "Enable");

		for (i = 0;i < 4;i++) {	/* L, R, U, D */
			buf.enable[i] = ctrl[i]->enable;
			buf.distance[i] = ctrl[i]->distance;
			buf.ratio_thres[i] = ctrl[i]->ratio_thres;
			buf.min_time[i] = ctrl[i]->min_time;
			buf.max_time[i] = ctrl[i]->max_time;
			buf.wrong_dir_thres[i] = ctrl[i]->wrong_dir_thres;
			buf.init_ratio_chk_dist[i] = ctrl[i]->init_ratio_chk_dist;
			buf.init_ratio_thres[i] = ctrl[i]->init_ratio_thres;
		}

		/* swipe_active_area_x1y1-x2y2 Horizontal */
		buf.area_hori[0] = ts->swipe[SWIPE_L].area.x1;
		buf.area_hori[1] = ts->swipe[SWIPE_R].area.x1;
		buf.area_hori[2] = ts->swipe[SWIPE_L].area.y1;
		buf.area_hori[3] = ts->swipe[SWIPE_R].area.y1;
		buf.area_hori[4] = ts->swipe[SWIPE_L].area.x2;
		buf.area_hori[5] = ts->swipe[SWIPE_R].area.x2;
		buf.area_hori[6] = ts->swipe[SWIPE_L].area.y2;
		buf.area_hori[7] = ts->swipe[SWIPE_R].area.y2;
		/* swipe_active_area_x1y1-x2y2 Vertical */
		buf.area_verti[0] = ts->swipe[SWIPE_U].area.x1;
		buf.area_verti[1] = ts->swipe[SWIPE_D].area.x1;
		buf.area_verti[2] = ts->swipe[SWIPE_U].area.y1;
		buf.area_verti[3] = ts->swipe[SWIPE_D].area.y1;
		buf.area_verti[4] = ts->swipe[SWIPE_U].area.x2;
		buf.area_verti[5] = ts->swipe[SWIPE_D].area.x2;
		buf.area_verti[6] = ts->swipe[SWIPE_U].area.y2;
		buf.area_verti[7] = ts->swipe[SWIPE_D].area.y2;
		/* swipe_start_area_x1y1-x2y2 Horizontal */
		buf.start_hori[0] = ts->swipe[SWIPE_L].start_area.x1;
		buf.start_hori[1] = ts->swipe[SWIPE_R].start_area.x1;
		buf.start_hori[2] = ts->swipe[SWIPE_L].start_area.y1;
		buf.start_hori[3] = ts->swipe[SWIPE_R].start_area.y1;
		buf.start_hori[4] = ts->swipe[SWIPE_L].start_area.x2;
		buf.start_hori[5] = ts->swipe[SWIPE_R].start_area.x2;
		buf.start_hori[6] = ts->swipe[SWIPE_L].start_area.y2;
		buf.start_hori[7] = ts->swipe[SWIPE_R].start_area.y2;
		/* swipe_start_area_x1y1-x2y2 Vertical */
		buf.start_verti[0] = ts->swipe[SWIPE_U].start_area.x1;
		buf.start_verti[1] = ts->swipe[SWIPE_D].start_area.x1;
		buf.start_verti[2] = ts->swipe[SWIPE_U].start_area.y1;
		buf.start_verti[3] = ts->swipe[SWIPE_D].start_area.y1;
		buf.start_verti[4] = ts->swipe[SWIPE_U].start_area.x2;
		buf.start_verti[5] = ts->swipe[SWIPE_D].start_area.x2;
		buf.start_verti[6] = ts->swipe[SWIPE_U].start_area.y2;
		buf.start_verti[7] = ts->swipe[SWIPE_D].start_area.y2;

		ret = sw49107_reg_write(dev, SWIPE_ON_CTRL, &buf, sizeof(buf));
		if (ret < 0)
			TOUCH_E("failed to write swipe registers (ret = %d)\n", ret);

	}

	return ret;
}

static int sw49107_swipe_enable(struct device *dev, bool enable)
{
	struct sw49107_data *d = to_sw49107_data(dev);

	TOUCH_TRACE();

	if (enable)
		sw49107_swipe_control(dev, d->lcd_mode);

	return 0;
}

static int sw49107_clock(struct device *dev, int state)
{
	int ret = 0;
	int onoff = (state == IC_NORMAL) ? 1 : 0;

	ret = sw49107_reg_write(dev, SPI_OSC_CTL, &onoff, sizeof(onoff));
	ret = sw49107_reg_write(dev, SPI_CLK_CTL, &onoff, sizeof(onoff));

	TOUCH_I("IC Clock = %s\n", (onoff == 0) ? "0 (off)" : "1 (on)");

	return ret;
}

int sw49107_tc_driving(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	u32 ctrl = 0;
	u32 rdata = 0;
	int i = 0;
	int ret = 0;

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("Skip tc driving in deep sleep\n");
		return 0;
	}

	if(!(d->p_param.used_mode & (1<<mode))) {
		TOUCH_E("Not Support Mode! tc_driving canceled (mode:%d)\n", mode);
		goto out;
	}

	switch (mode) {
		case LCD_MODE_U0:
			ctrl = 0x01;
			break;

		case LCD_MODE_U2_UNBLANK:
			ctrl = 0x101;
			break;

		case LCD_MODE_U2:
			ctrl = 0x101;
			break;

		case LCD_MODE_U3:
			ctrl = 0x181;
			break;

		case LCD_MODE_U3_PARTIAL:
			ctrl = 0x381;
			break;

		case LCD_MODE_U3_QUICKCOVER:
			ctrl = 0x581;
			break;

		case LCD_MODE_STOP:
			ctrl = 0x02;
			break;
		default:
			TOUCH_E("Not Support Mode! tc_driving canceled (mode:%d)\n", mode);
			return -1;
	}

	d->driving_mode = mode;

	ret = sw49107_reg_read(dev, SPR_SUBDISP_ST, &rdata, sizeof(rdata));
	TOUCH_I("DDI Display Mode = 0x%04x\n", rdata);
	ret = sw49107_reg_write(dev, TC_DRIVING_CTL, &ctrl, sizeof(ctrl));
	TOUCH_I("sw49107_tc_driving = 0x%x, 0x%x\n", mode, ctrl);

	//tc_status check
	if(mode != LCD_MODE_STOP){
		for(i = 0; i < 10; i++) {
			ret = sw49107_reg_read(dev, TC_STATUS, (u8 *)&rdata, sizeof(u32));
			TOUCH_I("read tc_status(0x201) = %x\n", rdata);
			if((rdata & 0x1F) == 0x7) {
				goto out;
			}
			touch_msleep(50);
		}
	}

out:
	return ret;
}

static int sw49107_deep_sleep_ctrl(struct device *dev, int state)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0;

	if (!atomic_read(&ts->state.incoming_call)) { /* IDLE status */
		if (state == IC_NORMAL) {
			sw49107_power(dev, POWER_WAKE);
			if (d->p_param.deep_sleep_power_control == FUNC_OFF)
				/* Defend touch init fail */
				sw49107_power(dev, POWER_DSV_ALWAYS_ON);
			mod_delayed_work(ts->wq, &ts->init_work, 0);
		} else { // IC_DEEP_SLEEP
			ret = sw49107_tc_driving(dev, LCD_MODE_STOP);
			touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
			atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
			sw49107_power(dev, POWER_SLEEP);
		}

		if (d->p_param.osc_clock_ctrl == FUNC_ON) {
			ret = sw49107_clock(dev, state);
		}

		TOUCH_I("%s - %s\n", __func__, (state == IC_NORMAL) ? "Lpwg Mode" : "Deep Sleep");
	} else { /* RINGING or OFFHOOK status */
		if (state == IC_NORMAL) {
			sw49107_power(dev, POWER_DSV_ALWAYS_ON);
			touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
			atomic_set(&ts->state.sleep, IC_NORMAL);
			ret = sw49107_setup_q_sensitivity(dev, 0);
			ret = sw49107_tci_control(dev, ts->lpwg.mode);
			ret = sw49107_swipe_enable(dev, true);
			ret = sw49107_tc_driving(dev, d->lcd_mode);
		} else { // IC_DEEP_SLEEP
			ret = sw49107_tc_driving(dev, LCD_MODE_STOP);
			atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
			touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
		}
		TOUCH_I("%s - %s But avoid deep sleep Power Sequence during Call\n",
				__func__,  (state == IC_NORMAL) ? "Lpwg Mode" : "Deep Sleep");
	}

	return ret;
}

static void sw49107_lpwg_failreason(struct device *dev)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	u32 type = 0;
	u32 buf = 0;

	if (!d->lpwg_failreason_ctrl)
		return;

	sw49107_reg_read(dev, LPWG_FAILREASON_STS_CTRL, &type, sizeof(type));
	TOUCH_I("[LPWG_FAILREASON_STS_CTRL = [0x%x]\n", type);

	if (type & 0x1) { //TCI-0 Knock-on
		sw49107_reg_read(dev, TCI_FAILREASON_BUF, &buf, sizeof(buf));
		buf = (buf & 0xFFFF);
		if (buf < LPWG_FAILREASON_TCI_NUM)
			TOUCH_I("[TCI-0 Knock-on] TCI_FAILREASON_BUF = [0X%x]%s\n", buf, lpwg_failreason_tci_str[buf]);
	}

	if (type & (0x1 << 1)) { //TCI-1 Knock-code
		sw49107_reg_read(dev, TCI_FAILREASON_BUF, &buf, sizeof(buf));
		buf = ((buf >> 16) & 0xFFFF);
		if (buf < LPWG_FAILREASON_TCI_NUM)
			TOUCH_I("[TCI-1 Knock-code] TCI_FAILREASON_BUF = [0X%x]%s\n", buf, lpwg_failreason_tci_str[buf]);
	}

	if (type & (0x1 << 2)) { //Swipe
		sw49107_reg_read(dev, SWIPE_FAILREASON_BUF, &buf, sizeof(buf));
		buf = (buf & 0xFF);
		if (buf < LPWG_FAILREASON_SWIPE_NUM)
			TOUCH_I("[SWIPE] SWIPE_FAILREASON_BUF = [0X%x]%s\n", buf, lpwg_failreason_swipe_str[buf]);
	}
}

static int sw49107_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0;

	if (atomic_read(&d->init) == IC_INIT_NEED) {
		TOUCH_I("Skip lpwg mode in IC_INIT_NEED status\n");
		return 0;
	}

	if (atomic_read(&d->hw_reset) == LCD_EVENT_TOUCH_RESET_START) {
		TOUCH_I("Skip lpwg mode during ic reset\n");
		return 0;
	}

	if (atomic_read(&d->changing_display_mode) == CHANGING_DISPLAY_MODE_NOT_READY) {
		TOUCH_I("Skip lpwg mode during changing display mode\n");
		return 0;
	}

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->mfts_lpwg) {
			ret = sw49107_tci_control(dev, LPWG_DOUBLE_TAP);
			ret = sw49107_swipe_enable(dev, true);
			ret = sw49107_tc_driving(dev, d->lcd_mode);
			sw49107_power(dev, POWER_DSV_TOGGLE);
			return 0;
		}

		if (ts->lpwg.screen) {
			TOUCH_I("Skip lpwg setting\n");
			sw49107_power(dev, POWER_DSV_ALWAYS_ON);
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep */
			TOUCH_I("suspend sensor == PROX_NEAR\n");
			if (atomic_read(&ts->state.sleep) != IC_DEEP_SLEEP)
				ret = sw49107_deep_sleep_ctrl(dev, IC_DEEP_SLEEP);
		} else if (ts->lpwg.qcover == HALL_NEAR) {
			/* Deep Sleep same as Prox near  */
			TOUCH_I("Qcover == HALL_NEAR\n");
			if (atomic_read(&ts->state.sleep) != IC_DEEP_SLEEP)
				ret = sw49107_deep_sleep_ctrl(dev, IC_DEEP_SLEEP);
		} else if (ts->swipe[SWIPE_U].enable == SWIPE_NONE && ts->lpwg.mode == LPWG_NONE
				&& d->lcd_mode == LCD_MODE_U0) {
			/* knock on/code disable */
			TOUCH_I("Knock-on mode == LPWG_NONE & swipe[SWIPE_U].enable == SWIPE_NONE & lcd mode == LCD_MODE_U0\n");
			ret = sw49107_deep_sleep_ctrl(dev, IC_DEEP_SLEEP);
		} else {
			/* knock on/code */
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
				ret = sw49107_deep_sleep_ctrl(dev, IC_NORMAL);
			} else {
				ret = sw49107_setup_q_sensitivity(dev, 0);
				ret = sw49107_tci_control(dev, ts->lpwg.mode);
				ret = sw49107_swipe_enable(dev, true);
				ret = sw49107_tc_driving(dev, d->lcd_mode);
				sw49107_power(dev, POWER_DSV_TOGGLE);
			}

		}
		return ret;
	}

	touch_report_all_event(ts);

	/* resume */
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("resume ts->lpwg.screen on\n");

		ret = sw49107_tci_control(dev, LPWG_NONE);
		ret = sw49107_swipe_enable(dev, true);
		ret = sw49107_setup_q_sensitivity(dev, 0);
		if (ts->lpwg.qcover == HALL_NEAR)
			ret = sw49107_tc_driving(dev, LCD_MODE_U3_QUICKCOVER);
		else
			ret = sw49107_tc_driving(dev, d->lcd_mode);
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		TOUCH_I("resume ts->lpwg.sensor == PROX_NEAR\n");
		ret = sw49107_tc_driving(dev, LCD_MODE_STOP);
	} else {
		/* partial */
		TOUCH_I("resume Partial\n");
		ret = sw49107_tci_control(dev, ts->lpwg.mode);
		ret = sw49107_swipe_enable(dev, true);
		ret = sw49107_tc_driving(dev, LCD_MODE_U3_PARTIAL);
	}

	return ret;
}

static int sw49107_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int *value = (int *)param;
	int ret = 0;

	switch (code) {
		case LPWG_TAP_COUNT:
			ts->tci.info[TCI_2].tap_count = value[0];
			break;

		case LPWG_DOUBLE_TAP_CHECK:
			ts->tci.double_tap_check = value[0];
			break;

		case LPWG_UPDATE_ALL:
			ts->lpwg.mode = value[0];
			ts->lpwg.screen = value[1];
			ts->lpwg.sensor = value[2];
			ts->lpwg.qcover = value[3];

			TOUCH_I("LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
					ts->lpwg.mode,
					ts->lpwg.screen ? "ON" : "OFF",
					ts->lpwg.sensor ? "FAR" : "NEAR",
					ts->lpwg.qcover ? "CLOSE" : "OPEN");

			ret = sw49107_lpwg_mode(dev);
			if (ret < 0)
				TOUCH_E("failed to lpwg_mode, ret:%d", ret);
			break;

		case LPWG_REPLY:
			break;

		default:
			TOUCH_I("%s - Unknown Lpwg Code : %d\n", __func__, code);
			break;
	}

	return ret;
}

static void sw49107_connect(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	int charger_state = atomic_read(&ts->state.connect);
	int wireless_state = atomic_read(&ts->state.wireless);

	TOUCH_TRACE();

	d->charger = 0;
	/* wire */
	if (charger_state == CONNECT_INVALID)
		d->charger = CONNECT_NONE;
	else if ((charger_state == CONNECT_DCP)	|| (charger_state == CONNECT_PROPRIETARY))
		d->charger = CONNECT_TA;
	else if (charger_state == CONNECT_HUB)
		d->charger = CONNECT_OTG;
	else
		d->charger = CONNECT_USB;

	/* wireless */
	if (wireless_state)
		d->charger = d->charger | CONNECT_WIRELESS;

	TOUCH_I("%s: write charger_state = 0x%02X\n", __func__, d->charger);
	if (atomic_read(&ts->state.pm) > DEV_PM_RESUME) {
		TOUCH_I("DEV_PM_SUSPEND - Don't try I2C\n");
		return;
	}
	sw49107_reg_write(dev, SPECIAL_CHARGER_INFO_CTRL, &d->charger, sizeof(u32));
}

static void sw49107_lcd_mode(struct device *dev, u32 mode)
{
	struct sw49107_data *d = to_sw49107_data(dev);

	TOUCH_I("lcd_mode: %d (prev: %d)\n", mode, d->lcd_mode);

	if (mode == LCD_MODE_U2_UNBLANK)
		mode = LCD_MODE_U2;

	d->lcd_mode = mode;
}

static int sw49107_usb_status(struct device *dev, u32 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("TA Type: %d\n", atomic_read(&ts->state.connect));
	sw49107_connect(dev);
	return 0;
}

static int sw49107_wireless_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Wireless charger: 0x%02X\n", atomic_read(&ts->state.wireless));
	sw49107_connect(dev);
	return 0;
}

static int sw49107_earjack_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Earjack Type: 0x%02X\n", atomic_read(&ts->state.earjack));
	return 0;
}

static void sw49107_fb_notify_work_func(struct work_struct *fb_notify_work)
{
	struct sw49107_data *d = container_of(to_delayed_work(fb_notify_work),
			struct sw49107_data, fb_notify_work);
	int ret = 0;

	if (d->lcd_mode == LCD_MODE_U0 || d->lcd_mode == LCD_MODE_U2)
		ret = FB_SUSPEND;
	else
		ret = FB_RESUME;

	touch_notifier_call_chain(NOTIFY_FB, &ret);
}

static int sw49107_notify_etc(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	switch (event) {
		case LCD_EVENT_TOUCH_RESET_START:
			TOUCH_I("LCD_EVENT_TOUCH_RESET_START!\n");

			atomic_set(&d->hw_reset, event);
			touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
			touch_gpio_direction_output(ts->reset_pin, 0);
			break;

		case LCD_EVENT_TOUCH_RESET_END:
			TOUCH_I("LCD_EVENT_TOUCH_RESET_END!\n");

			atomic_set(&d->hw_reset, event);
			touch_gpio_direction_output(ts->reset_pin, 1);
			break;

		default:
			TOUCH_E("%lu is not supported in charger mode\n", event);
			break;
	}

	return ret;
}

static int sw49107_notify_normal(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	switch (event) {
		case NOTIFY_TOUCH_RESET:
			if(atomic_read(&ts->state.debug_option_mask) & DEBUG_OPTION_1)
				ret = 1;
			else
				ret = 0;
			TOUCH_I("NOTIFY_TOUCH_RESET! return = %d\n", ret);
			break;

		case LCD_EVENT_TOUCH_RESET_START:
			TOUCH_I("LCD_EVENT_TOUCH_RESET_START!\n");

			atomic_set(&d->hw_reset, event);
			touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
			touch_gpio_direction_output(ts->reset_pin, 0);
			break;

		case LCD_EVENT_TOUCH_RESET_END:
			TOUCH_I("LCD_EVENT_TOUCH_RESET_END!\n");

			atomic_set(&d->hw_reset, event);
			touch_gpio_direction_output(ts->reset_pin, 1);
			break;

		case LCD_EVENT_LCD_BLANK:
			TOUCH_I("LCD_EVENT_LCD_BLANK! - Skip lpwg mode - START\n");
			atomic_set(&ts->state.fb, FB_SUSPEND);
			atomic_set(&d->changing_display_mode, CHANGING_DISPLAY_MODE_NOT_READY);
			break;

		case LCD_EVENT_LCD_UNBLANK:
			TOUCH_I("LCD_EVENT_LCD_UNBLANK! - Skip lpwg mode - START\n");
			/* Change DSV ENM Mode (Enter -> Exit) after MiniOS LPWG PT Test */
			if (ts->mfts_lpwg && (atomic_read(&ts->state.fb) == FB_SUSPEND))
				sw49107_power(dev, POWER_DSV_ALWAYS_ON);
			atomic_set(&ts->state.fb, FB_RESUME);
			atomic_set(&d->changing_display_mode, CHANGING_DISPLAY_MODE_NOT_READY);
			break;

		case LCD_EVENT_LCD_MODE:
			TOUCH_I("LCD_EVENT_LCD_MODE!\n");
			sw49107_lcd_mode(dev, *(u32 *)data);
			queue_delayed_work(ts->wq, &d->fb_notify_work, 0);
			break;

		case NOTIFY_CONNECTION:
			TOUCH_I("NOTIFY_CONNECTION!\n");
			ret = sw49107_usb_status(dev, *(u32 *)data);
			break;

		case NOTIFY_WIRELESS:
			TOUCH_I("NOTIFY_WIRELESS!\n");
			ret = sw49107_wireless_status(dev, *(u32 *)data);
			break;

		case NOTIFY_EARJACK:
			TOUCH_I("NOTIFY_EARJACK!\n");
			ret = sw49107_earjack_status(dev, *(u32 *)data);
			break;

		case NOTIFY_IME_STATE:
			TOUCH_I("NOTIFY_IME_STATE!\n");
			ret = sw49107_reg_write(dev, SPECIAL_IME_STATUS_CTRL, (u32*)data, sizeof(u32));
			break;

		case NOTIFY_CALL_STATE:
			/* Notify Touch IC only for GSM call and idle state */
			if (*(u32*)data >= INCOMING_CALL_IDLE && *(u32*)data <= INCOMING_CALL_LTE_OFFHOOK) {
				TOUCH_I("NOTIFY_CALL_STATE!\n");
				ret = sw49107_reg_write(dev, SPECIAL_CALL_INFO_CTRL, (u32*)data, sizeof(u32));
			}
			break;

		case NOTIFY_QMEMO_STATE:
			TOUCH_I("NOTIFY_QMEMO_STATE!\n");
			break;

		default:
			TOUCH_E("%lu is not supported\n", event);
			break;
	}

	return ret;
}

static int sw49107_notify(struct device *dev, ulong event, void *data)
{
	int boot_mode = TOUCH_NORMAL_BOOT;

	TOUCH_TRACE();

	boot_mode = touch_check_boot_mode(dev);
	if (boot_mode == TOUCH_CHARGER_MODE
			|| boot_mode == TOUCH_LAF_MODE
			|| boot_mode == TOUCH_RECOVERY_MODE) {
		TOUCH_I("Notify Skip MODE notify. %d\n", boot_mode);
		return sw49107_notify_etc(dev,event,data);
	}

	return sw49107_notify_normal(dev,event,data);
}

#if defined(CONFIG_DRM_MSM) && defined(CONFIG_FB)
#if defined(CONFIG_LGE_TOUCH_USE_PANEL_NOTIFY)
static int sw49107_drm_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct msm_drm_notifier *ev = (struct msm_drm_notifier *)data;

	TOUCH_TRACE();

	if (ev && ev->data && event == MSM_DRM_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == MSM_DRM_BLANK_UNBLANK)
			TOUCH_I("DRM_UNBLANK\n");
		else if (*blank == MSM_DRM_BLANK_POWERDOWN)
			TOUCH_I("DRM_POWERDOWN\n");
	}

	return 0;
}
#endif
#elif defined(CONFIG_FB)
static int sw49107_fb_notifier_callback(struct notifier_block *self,
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
#endif

static int sw49107_init_pm(struct device *dev)
{
#if defined(CONFIG_FB)
	struct touch_core_data *ts = to_touch_core(dev);
#endif

	TOUCH_TRACE();

#if defined(CONFIG_DRM_MSM) && defined(CONFIG_FB)
#if defined(CONFIG_LGE_TOUCH_USE_PANEL_NOTIFY)
	TOUCH_I("%s: drm_notif change\n", __func__);
	ts->drm_notif.notifier_call = sw49107_drm_notifier_callback;
#endif
#elif defined(CONFIG_FB)
	TOUCH_I("%s: fb_notif change\n", __func__);
	fb_unregister_client(&ts->fb_notif);
	ts->fb_notif.notifier_call = sw49107_fb_notifier_callback;
#endif

	return 0;
}

static void sw49107_init_works(struct sw49107_data *d)
{
	INIT_DELAYED_WORK(&d->fb_notify_work, sw49107_fb_notify_work_func);
}

static void sw49107_init_locks(struct sw49107_data *d)
{
	mutex_init(&d->io_lock);
}

static int sw49107_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = NULL;
	int ret = 0;
	int boot_mode = TOUCH_NORMAL_BOOT;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate ic data\n");
		return -ENOMEM;
	}

	d->dev = dev;
	touch_set_device(ts, d);

	project_param_set(dev);

	ret = touch_gpio_init(ts->reset_pin, "touch_reset");
	if (ret < 0) {
		TOUCH_E("failed to touch gpio init\n");
		return ret;
	}

	ret = touch_gpio_direction_output(ts->reset_pin, 1);
	if (ret < 0) {
		TOUCH_E("failed to touch gpio direction output\n");
		return ret;
	}

	ret = touch_gpio_init(ts->int_pin, "touch_int");
	if (ret < 0) {
		TOUCH_E("failed to touch gpio init\n");
		return ret;
	}
	ret = touch_gpio_direction_input(ts->int_pin);
	if (ret < 0) {
		TOUCH_E("failed to touch gpio direction input\n");
		return ret;
	}

	if (0) {
		ret = touch_gpio_init(ts->maker_id_pin, "touch_make_id");
		if (ret < 0) {
			TOUCH_E("failed to touch gpio init\n");
			return ret;
		}
		ret = touch_gpio_direction_input(ts->maker_id_pin);
		if (ret < 0) {
			TOUCH_E("failed to touch gpio direction input\n");
			return ret;
		}
	}
	/*******************************************************
	 * Display driver does control the power in sw49107 IC *
	 * due to its design from INCELL 1-chip. Here we skip  *
	 * the control power.                                  *
	 *******************************************************/
	if(d->p_param.touch_power_control_en == FUNC_ON) {
		ret = touch_power_init(dev);
		if (ret < 0) {
			TOUCH_E("failed to touch power init\n");
			return ret;
		}
	}
	ret = touch_bus_init(dev, MAX_XFER_BUF_SIZE);
	if (ret < 0) {
		TOUCH_E("failed to touch bus init\n");
		return ret;
	}

	sw49107_init_works(d);
	sw49107_init_locks(d);

	boot_mode = touch_check_boot_mode(dev);
	if (boot_mode == TOUCH_CHARGER_MODE
			|| boot_mode == TOUCH_LAF_MODE
			|| boot_mode == TOUCH_RECOVERY_MODE) {
		TOUCH_I("%s: boot_mode = %d\n", __func__, boot_mode);
		ret = touch_gpio_init(ts->reset_pin, "touch_reset");
		if (ret < 0) {
			TOUCH_E("failed to touch gpio init\n");
			return ret;
		}
		ret = touch_gpio_direction_output(ts->reset_pin, 1);
		if (ret < 0) {
			TOUCH_E("failed to touch gpio direction output\n");
			return ret;
		}
		/* Deep Sleep */
		ret = sw49107_deep_sleep_ctrl(dev, IC_DEEP_SLEEP);
		if (ret < 0) {
			TOUCH_E("failed to deep sleep ctrl\n");
			return ret;
		}
		return ret;
	}

	sw49107_get_tci_info(dev);
	sw49107_get_swipe_info(dev);

	d->lcd_mode = LCD_MODE_U3;
	d->lpwg_failreason_ctrl = LPWG_FAILREASON_ENABLE;
	atomic_set(&d->water_old_mode, MODE_NORMAL);
	if (d->p_param.dynamic_tune_table == FUNC_ON) {
#if 0
		if (lge_get_board_revno() <= HW_REV_1_0)
			d->select_tune_table = OLD_SAMPLE_TUNE_TABLE;
		else
			d->select_tune_table = NEW_SAMPLE_TUNE_TABLE;
#endif
		d->select_tune_table = NEW_SAMPLE_TUNE_TABLE;	// always new table.
	}

	return ret;
}

static int sw49107_remove(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static int sw49107_shutdown(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static int sw49107_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	u8 ic_major = d->ic_info.version.major_ver;
	u8 ic_minor = d->ic_info.version.minor_ver;
	u32 bin_ver_offset = *((u32 *)&fw->data[0xe8]);
	u32 bin_pid_offset = *((u32 *)&fw->data[0xf0]);
	char pid[12] = {0};
	u8 bin_major;
	u8 bin_minor;
	int update = 0;
	int flash_fw_size = d->p_param.flash_fw_size;

	if ((bin_ver_offset > flash_fw_size) || (bin_pid_offset > flash_fw_size)) {
		TOUCH_I("INVALID OFFSET\n");
		return -1;
	}

	bin_major = fw->data[bin_ver_offset];
	bin_minor = fw->data[bin_ver_offset + 1];
	memcpy(pid, &fw->data[bin_pid_offset], 8);

	if ((ts->force_fwup) || (bin_major != ic_major) || (bin_minor != ic_minor))
		update = 1;

	TOUCH_I("bin-ver: %d.%02d (%s), ic-ver: %d.%02d -> update: %d, force_fwup: %d\n",
			bin_major, bin_minor, pid, ic_major, ic_minor, update, ts->force_fwup);

	return update;
}

static int sw49107_condition_wait(struct device *dev,
		u16 addr, u32 *value, u32 expect, u32 mask, u32 delay, u32 retry)
{
	u32 data = 0;

	do {
		touch_msleep(delay);
		sw49107_read_value(dev, addr, &data);

		if ((data & mask) == expect) {
			if (value)
				*value = data;
			TOUCH_I("%d, addr[%04x] data[%08x], mask[%08x], expect[%08x]\n",
					retry, addr, data, mask, expect);
			return 0;
		}
	} while (--retry);

	if (value)
		*value = data;

	TOUCH_I("%s addr[%04x], expect[%x], mask[%x], data[%x]\n",
			__func__, addr, expect, mask, data);

	return -EPERM;
}

int sw49107_specific_header_verify(unsigned char *header, int i)
{
	t_cfg_s_header_def *head = (t_cfg_s_header_def *)header;
	char tmp[8] = {0, };

	if (head->cfg_specific_info1.b.chip_rev <= 0 &&
			head->cfg_specific_info1.b.chip_rev > 10) {
		TOUCH_I("Invalid Chip revision id %8.8X\n", head->cfg_specific_info1.b.chip_rev);
		return -2;
	}

	memset(tmp, 0, 8);
	memcpy((void*)tmp, (void *)&head->cfg_model_name, 4);

	TOUCH_I("==================== SPECIFIC #%d =====================\n", i +1);
	TOUCH_I("chip_rev           : %d\n", head->cfg_specific_info1.b.chip_rev);
	TOUCH_I("fpcb_id            : %d\n", head->cfg_specific_info1.b.fpcb_id);
	TOUCH_I("lcm_id             : %d\n", head->cfg_specific_info1.b.lcm_id);
	TOUCH_I("model_id           : %d\n", head->cfg_specific_info1.b.model_id);
	TOUCH_I("model_name         : %s\n", tmp);
	TOUCH_I("lot_id             : %d\n", head->cfg_specific_info2.b.lot_id);
	TOUCH_I("ver                : %d\n", head->cfg_specific_version);

	return 1;
}

int sw49107_common_header_verify(t_cfg_info_def *header)
{
	t_cfg_info_def *head = (t_cfg_info_def *)header;
	t_cfg_c_header_def *common_head = (t_cfg_c_header_def *)(header + sizeof(t_cfg_info_def));

	if (head->cfg_magic_code != CFG_MAGIC_CODE) {
		TOUCH_I("Invalid CFG_MAGIC_CODE. %8.8X\n", head->cfg_magic_code);
		return -1;
	}

	if (head->cfg_chip_id != CFG_CHIP_ID) {
		TOUCH_I("Invalid Chip ID. (49107 != %d)\n", head->cfg_chip_id);
		return -2;
	}

	if (head->cfg_struct_version <= 0) {
		TOUCH_I("Invalid cfg_struct_version. %8.8X\n", head->cfg_struct_version);
		return -3;
	}

	if (head->cfg_specific_cnt <= 0) {
		TOUCH_I("No Specific Data. %8.8X\n", head->cfg_specific_cnt);
		return -4;
	}

	if (head->cfg_size.b.common_cfg_size > CFG_C_MAX_SIZE) {
		TOUCH_I("Over CFG COMMON MAX Size (%d). %8.8X\n",
				CFG_C_MAX_SIZE, head->cfg_size.b.common_cfg_size);
		return -5;
	}

	if (head->cfg_size.b.specific_cfg_size > CFG_S_MAX_SIZE) {
		TOUCH_I("Over CFG SPECIFIC MAX Size (%d). %8.8X\n",
				CFG_S_MAX_SIZE, head->cfg_size.b.specific_cfg_size);
		return -6;
	}

	TOUCH_I("==================== COMMON ====================\n");
	TOUCH_I("magic code         : 0x%8.8X\n", head->cfg_magic_code);
	TOUCH_I("chip id            : %d\n", head->cfg_chip_id);
	TOUCH_I("struct_ver         : %d\n", head->cfg_struct_version);
	TOUCH_I("specific_cnt       : %d\n", head->cfg_specific_cnt);
	TOUCH_I("cfg_c size         : %d\n", head->cfg_size.b.common_cfg_size);
	TOUCH_I("cfg_s size         : %d\n", head->cfg_size.b.specific_cfg_size);
	TOUCH_I("date               : 0x%8.8X\n", head->cfg_global_date);
	TOUCH_I("time               : 0x%8.8X\n", head->cfg_global_time);
	TOUCH_I("common_ver         : %d\n", common_head->cfg_common_ver);

	return 1;
}

static int sw49107_img_binary_verify(struct device *dev, unsigned char *imgBuf)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	int flash_fw_size = d->p_param.flash_fw_size;
	unsigned char *specific_ptr;
	unsigned char *cfg_buf_base = &imgBuf[flash_fw_size];
	int i;
	t_cfg_info_def *head = (t_cfg_info_def *)cfg_buf_base;

	u32 *fw_crc = (u32 *)&imgBuf[flash_fw_size -4];
	u32 *fw_size = (u32 *)&imgBuf[flash_fw_size -8];

	if (*fw_crc == 0x0 || *fw_crc == 0xFFFFFFFF || *fw_size > flash_fw_size) {
		TOUCH_I("Firmware Size Invalid READ : 0x%X\n", *fw_size);
		TOUCH_I("Firmware CRC Invalid READ : 0x%X\n", *fw_crc);
		return E_FW_CODE_SIZE_ERR;
	} else {
		TOUCH_I("Firmware Size READ : 0x%X\n", *fw_size);
		TOUCH_I("Firmware CRC READ : 0x%X\n", *fw_crc);
	}

	if (sw49107_common_header_verify(head) < 0) {
		TOUCH_I("No Common CFG! Firmware Code Only\n");
		return E_FW_CODE_ONLY_VALID;
	}

	specific_ptr = cfg_buf_base + head->cfg_size.b.common_cfg_size;
	for (i = 0; i < head->cfg_specific_cnt; i++) {
		if (sw49107_specific_header_verify(specific_ptr, i) < 0) {
			TOUCH_I("specific CFG invalid!\n");
			return -2;
		}
		specific_ptr += head->cfg_size.b.specific_cfg_size;
	}

	return E_FW_CODE_AND_CFG_VALID;
}

static int sw49107_fw_upgrade(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	u8 *fwdata = (u8 *) fw->data;
	u32 data;
	u32 conf_specific_dn_index;
	u32 cfg_c_size;
	u32 cfg_s_size;
	u32 sys_sram_val;
	t_cfg_info_def *head;
	int ret = 0;
	int i = 0;
	int img_check_result;
	int flash_fw_size = d->p_param.flash_fw_size;

	/* [Start] Binary Check Verification */
	TOUCH_I("%s - Checking FW Image before flashing\n", __func__);

	if(fw->size > FLASH_SIZE) {
		TOUCH_I("%s - FW Image Size(0x%8.8x) is not correct\n", __func__, (unsigned int)fw->size);
		return -EPERM;
	} else {
		TOUCH_I("%s - FW Image Size(0x%8.8x) flash_fw_size= 0x%x\n",__func__, (unsigned int)fw->size, flash_fw_size);
	}

	//CFG area used or NOT
	if(d->p_param.fw_cfg_use_en == FUNC_ON) {
		img_check_result = sw49107_img_binary_verify(dev, (unsigned char*)fwdata);

		switch (img_check_result) {
			case E_FW_CODE_AND_CFG_VALID:
				TOUCH_I("%s - FW Image Verification success!!\n", __func__);
				break;
			case E_FW_CODE_CFG_ERR:
			case E_FW_CODE_SIZE_ERR:
			case E_FW_CODE_ONLY_VALID:
			default:
				TOUCH_I("%s - FW Image Verification fail!!\n", __func__);
				return -EPERM;
		}
	}
	/* [End] Binary Check Verification */

	/* enable SPI between RAM and ROM */
	//sw49107_write_value(dev, 0x15, 0);

	/* Reset Touch CM3 core and put system on hold */
	sw49107_write_value(dev, sys_rst_ctl, 2);

	/* sram write enable */
	sw49107_reg_read(dev, sys_sram_ctl, &sys_sram_val, sizeof(sys_sram_val));
	sys_sram_val = (sys_sram_val | 0x1);
	sw49107_write_value(dev, sys_sram_ctl, sys_sram_val);

	/* Write F/W Code to CODE SRAM (80KB) */
	for (i = 0 ; i < flash_fw_size ; i += MAX_RW_SIZE) {
		/* Set code sram base address write */
		sw49107_write_value(dev, spr_code_offset, i / 4);

		/* firmware image download to code sram */
		sw49107_reg_write(dev, code_access_addr, &fwdata[i], MAX_RW_SIZE);
	}

	/* init boot code write */
	sw49107_write_value(dev, fw_boot_code_addr, FW_BOOT_LOADER_INIT);

	/* Start CM3 Boot after Code Dump */
	sw49107_write_value(dev, sys_boot_ctl, 1);

	/* Release Touch CM3 core reset*/
	sw49107_write_value(dev, sys_rst_ctl, 0);

	/* sram write disable */
	//sw49107_write_value(dev, sys_sram_ctl, 0);

	/* Check F/W Boot Done Status */
	ret = sw49107_condition_wait(dev, fw_boot_code_addr, NULL, FW_BOOT_LOADER_CODE, 0xFFFFFFFF, 10, 200);

	if (ret < 0) {
		TOUCH_E("failed : \'boot check\'\n");
		return -EPERM;
	} else {
		TOUCH_I("success : boot check\n");
	}

	/* [Start] F/W Code Flash Download */
	/* Dump F/W Code with Flash DMA */
	sw49107_write_value(dev,tc_flash_dn_ctl,(FLASH_KEY_CODE_CMD << 16) | 1);
	touch_msleep(ts->caps.hw_reset_delay);

	/* Check F/W Code Flash Download Status */
	ret = sw49107_condition_wait(dev, tc_flash_dn_sts, &data,
			FLASH_CODE_DNCHK_VALUE, 0xFFFFFFFF, 10, 200);
	if (ret < 0) {
		TOUCH_E("failed : \'code check\'\n");
		return -EPERM;
	} else {
		TOUCH_I("success : code check\n");
	}
	/* [End] F/W Code Flash Download */

	/* [Start] Config Data Flash Download */
	if(d->p_param.fw_cfg_use_en == FUNC_ON) {
		if (img_check_result == E_FW_CODE_AND_CFG_VALID) {
			head = (t_cfg_info_def *)&fwdata[flash_fw_size];

			cfg_c_size = head->cfg_size.b.common_cfg_size;
			cfg_s_size = head->cfg_size.b.specific_cfg_size;

			/* conf index count read */
			sw49107_reg_read(dev, rconf_dn_index, (u8 *)&conf_specific_dn_index, sizeof(u32));
			TOUCH_I("conf_specific_dn_index : %08x  \n", conf_specific_dn_index);
			if (conf_specific_dn_index == 0 ||
					((conf_specific_dn_index * cfg_s_size) > (fw->size - flash_fw_size - cfg_c_size))) {
				TOUCH_I("Invalid Specific CFG Index => 0x%8.8X\n", conf_specific_dn_index);

				return -EPERM;
			}

			/* cfg_c sram base address write */
			sw49107_write_value(dev, spr_data_offset, rcfg_c_sram_oft);

			/* Conf data download to conf sram */
			sw49107_reg_write(dev, data_access_addr, &fwdata[flash_fw_size],cfg_c_size);

			/* cfg_s sram base address write */
			sw49107_write_value(dev, spr_data_offset, rcfg_s_sram_oft);

			// CFG Specific Download to CFG Download buffer (SRAM)
			sw49107_reg_write(dev, data_access_addr, &fwdata[flash_fw_size + cfg_c_size +
					(conf_specific_dn_index - 1) * cfg_s_size], cfg_s_size);

			/* Conf Download Start */
			sw49107_write_value(dev,tc_flash_dn_ctl,(FLASH_KEY_CONF_CMD << 16) | 2);

			/* Conf check */
			ret = sw49107_condition_wait(dev, tc_flash_dn_sts,&data,
					FLASH_CONF_DNCHK_VALUE,0xFFFFFFFF, 10, 200);
			if (ret < 0) {
				TOUCH_E("failed : \'cfg check\'\n");
				return -EPERM;
			} else {
				TOUCH_I("success : cfg_check\n");
			}

			ret = sw49107_specific_header_verify(&fwdata[flash_fw_size + cfg_c_size + (conf_specific_dn_index - 1)*cfg_s_size],
					conf_specific_dn_index - 1);
			if(ret < 0) {
				TOUCH_I("specific header invalid!\n");
				return -EPERM;
			}
			/* [End] Config Data Flash down */
		}
	}

	TOUCH_I("===== Firmware download Okay =====\n");

	d->boot_err_cnt = 0;

	return 0;
}

static int sw49107_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	const struct firmware *fw = NULL;
	char fwpath[256] = {0};
	int ret = 0;
	int i = 0;

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n", &ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
#if defined(CONFIG_LGE_TOUCH_MODULE_DETECT)
		if (panel_id == LGD_INCELL_SW49107) {
			memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
		} else if (panel_id == TOVIS_INCELL_SW49107) {
			memcpy(fwpath, ts->def_fwpath[1], sizeof(fwpath));
		}
		TOUCH_I("get fwpath from def_fwpath : %s", fwpath);
#endif
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	fwpath[sizeof(fwpath) - 1] = '\0';

	if (strlen(fwpath) <= 0) {
		TOUCH_E("error get fw path\n");
		return -EPERM;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);

	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", fwpath, ret);
		return ret;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	if (sw49107_fw_compare(dev, fw)) {
		ret = -EINVAL;
		touch_msleep(200);

		for (i = 0; i < 2 && ret < 0; i++)
			ret = sw49107_fw_upgrade(dev, fw);

		if (!ret) {
			d->err_cnt = 0;
			TOUCH_I("FW upgrade retry err_cnt clear\n");
		}
	} else {
		ret = -EPERM;
	}

	release_firmware(fw);

	return ret;
}

static int sw49107_esd_recovery(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY) || IS_ENABLED(CONFIG_LGE_TOUCH_PANEL_GLOBAL_RESET)
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	lge_mdss_report_panel_dead(PANEL_HW_RESET);
#endif
#endif

#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mtkfb_esd_recovery();
#endif

	TOUCH_I("Called %s (%d). !!\n", __func__, ts->lpwg.mode);

	return 0;
}

static int sw49107_suspend(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	int mfts_mode = 0;
	int ret = 0;
	u32 tc_status_tmp = 0;
	int boot_mode = TOUCH_NORMAL_BOOT;

	TOUCH_TRACE();

	boot_mode = touch_check_boot_mode(dev);
	if (boot_mode == TOUCH_CHARGER_MODE
			|| boot_mode == TOUCH_LAF_MODE
			|| boot_mode == TOUCH_RECOVERY_MODE) {
		TOUCH_I("%s: boot_mode = %d\n", __func__, boot_mode);
		return -EPERM;
	}

	mfts_mode = touch_check_boot_mode(dev);
	if ((mfts_mode >= TOUCH_MINIOS_MFTS_FOLDER) && !ts->mfts_lpwg) {
		TOUCH_I("%s : touch_suspend - MFTS\n", __func__);
		sw49107_power(dev, POWER_OFF);
		return -EPERM;
	} else {
		TOUCH_I("%s : touch_suspend start\n", __func__);
	}

	atomic_set(&d->changing_display_mode, CHANGING_DISPLAY_MODE_READY);
	TOUCH_I("Skip lpwg mode - END (suspend)\n");

	if (atomic_read(&d->init) == IC_INIT_DONE) {
		ret = sw49107_lpwg_mode(dev);
		if (ret < 0)
			TOUCH_E("failed to lpwg_mode, ret:%d", ret);
	} else { /* need init */
		return 1;
	}

	// Error handling (Required code)
	if (d->lcd_mode == LCD_MODE_U0 && (ts->swipe[SWIPE_U].enable > SWIPE_NONE || ts->lpwg.mode > LPWG_NONE)
			&& atomic_read(&ts->state.sleep) == IC_NORMAL
			&& atomic_read(&d->hw_reset) == LCD_EVENT_TOUCH_RESET_END) {
			ret = sw49107_reg_read(dev, TC_STATUS, &tc_status_tmp, sizeof(tc_status_tmp));
			if (ret < 0) {
				TOUCH_E("tc status read failed %d\n", ret);
				return -1;
			}
			// TC_STATUS : Abnormal_sts -> T-Reset
			if (tc_status_tmp & 0x200) {
				sw49107_reset_ctrl(dev, POWER_HW_RESET_SYNC);
				TOUCH_E("Call T-Reset - tc_status : %x\n", tc_status_tmp);
				return -1;
			}
	}

	return ret;
}

static int sw49107_resume(struct device *dev)
{
#if 0 /* FW-upgrade not working at MFTS mode */
	struct touch_core_data *ts = to_touch_core(dev);
	int mfts_mode = 0;
#endif
	int boot_mode = TOUCH_NORMAL_BOOT;

	TOUCH_TRACE();

#if 0 /* FW-upgrade not working at MFTS mode */
	mfts_mode = touch_check_boot_mode(dev);
	if ((mfts_mode >= TOUCH_MINIOS_MFTS_FOLDER) && !ts->mfts_lpwg) {
		sw49107_power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
		sw49107_ic_info(dev);
		if (sw49107_upgrade(dev) == 0) {
			sw49107_power(dev, POWER_OFF);
			sw49107_power(dev, POWER_ON);
			touch_msleep(ts->caps.hw_reset_delay);
		}
	}
#endif
	boot_mode = touch_check_boot_mode(dev);
	if (boot_mode == TOUCH_CHARGER_MODE
			|| boot_mode == TOUCH_LAF_MODE
			|| boot_mode == TOUCH_RECOVERY_MODE) {
		TOUCH_I("%s: boot_mode = %d\n", __func__, boot_mode);
		return -EPERM;
	}

	return 0;
}

static int sw49107_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	u32 data = 1;
	int ret = 0;

	TOUCH_TRACE();

/* check move to init_pm function
	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		TOUCH_I("fb_notif change\n");
		fb_unregister_client(&ts->fb_notif);
		ts->fb_notif.notifier_call = sw49107_fb_notifier_callback;
		fb_register_client(&ts->fb_notif);
	}
*/

	TOUCH_I("%s: charger_state = 0x%02X\n", __func__, d->charger);


	ret = sw49107_ic_info(dev);
	if (ret < 0) {
		TOUCH_E("sw49107_ic_info failed, ret:%d\n", ret);
		return ret;
	}

	ret = sw49107_reg_write(dev, TC_DEVICE_CTL, &data, sizeof(data));
	if (ret < 0) {
		TOUCH_E("failed to write \'tc_device_ctrl\', ret:%d\n", ret);
		return ret;
	}

	ret = sw49107_reg_write(dev, TC_INTERRUPT_CTL, &data, sizeof(data));
	if (ret < 0) {
		TOUCH_E("failed to write \'tc_interrupt_ctrl\', ret:%d\n", ret);
		return ret;
	}

	if (d->p_param.dynamic_tune_table == FUNC_ON) {
		ret = sw49107_reg_write(dev, SPECIAL_DYNAMIC_TUNE_TABLE_CTRL, &d->select_tune_table, sizeof(u8));
		if (ret < 0)
			TOUCH_E("failed to write \'SPECIAL_DYNAMIC_TUNE_TABLE_CTRL\', ret:%d\n", ret);
		TOUCH_I("Select tune table = %s\n",
				d->select_tune_table ? "OLD_SAMPLE_TUNE_TABLE" : "NEW_SAMPLE_TUNE_TABLE");
	}

	ret = sw49107_reg_write(dev, SPECIAL_CHARGER_INFO_CTRL, &d->charger, sizeof(u32));
	if (ret < 0)
		TOUCH_E("failed to write \'SPECIAL_CHARGER_CTRL\', ret:%d\n", ret);

	data = atomic_read(&ts->state.ime);
	ret = sw49107_reg_write(dev, SPECIAL_IME_STATUS_CTRL, &data, sizeof(data));
	if (ret < 0)
		TOUCH_E("failed to write \'SPECIAL_IME_STATUS_CTRL\', ret:%d\n", ret);

	TOUCH_D(QUICKCOVER,"%s : %s(%d)\n", __func__,
			(d->q_sensitivity) ? "SENSITIVE" : "NORMAL", (d->q_sensitivity));

	ret = sw49107_reg_write(dev, COVER_SENSITIVITY_CTRL, &d->q_sensitivity, sizeof(u32));
	if (ret < 0)
		TOUCH_E("failed to write \'QCOVER_SENSITIVITY\', ret:%d\n", ret);


	data = atomic_read(&ts->state.incoming_call);
	if (data > INCOMING_CALL_IDLE) {
		TOUCH_I("Change noise filter at call state (RINGING/OFFHOOK) : %d\n", data);
		ret = sw49107_reg_write(dev, SPECIAL_CALL_INFO_CTRL, &data, sizeof(data));
		if (ret < 0)
			TOUCH_E("failed to write \'SPECIAL_CALL_INFO_CTRL\', ret:%d\n", ret);
	}

	atomic_set(&d->init, IC_INIT_DONE);
	atomic_set(&ts->state.sleep, IC_NORMAL);
	atomic_set(&d->changing_display_mode, CHANGING_DISPLAY_MODE_READY);
	TOUCH_I("Skip lpwg mode - END (init)\n");
	d->err_cnt = 0;

	ret = sw49107_lpwg_mode(dev);
	if (ret < 0)
		TOUCH_E("failed to lpwg_mode, ret:%d", ret);

	return 0;
}

int sw49107_check_status(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0;
	int i = 0, idx = 0;
	u64 status = (((u64)d->info.ic_status) << 32) | ((u64)d->info.tc_status); //0x200~201 HW/FW Status
	u64 status_mask = 0x0;

	TOUCH_D(ABS, "%s : status [0x%016llx] ic_status [0x%08x], tc_status [0x%08x]",
			__func__, (u64)status, d->info.ic_status, d->info.tc_status);

	/* Status Checking */
	status_mask = status ^ STATUS_NORMAL_MASK;
	if (status_mask & STATUS_GLOBAL_RESET_BIT) {
		TOUCH_E("Need Global Reset\n");
		d->err_cnt++;
		if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
			TOUCH_E("Change Global to HW Reset in FB_SUSPEND\n");
			ret = -EHWRESET_SYNC;
		} else
			ret = -EGLOBALRESET;
	} else if (status_mask & STATUS_HW_RESET_BIT) {
		TOUCH_E("Need Touch HW Reset\n");
		d->err_cnt++;
		ret = -EHWRESET_SYNC;
	} else if (status_mask & STATUS_SW_RESET_BIT) {
		TOUCH_E("Need Touch SW Reset\n");
		d->err_cnt++;
		ret = -ESWRESET;
	} else if (status_mask & STATUS_FW_UPGRADE_BIT) {
		TOUCH_E("Need FW Upgrade\n");
		d->err_cnt++;
		ret = -EUPGRADE;
	} else if (status_mask & STATUS_LOGGING_BIT) {
		TOUCH_E("Need Logging\n");
		/* Start - kiwoo.han 2019-1-29 */
		//ret = -ERANGE;
		/* End - kiwoo.han 2019-1-29 */
	}

	if (d->err_cnt > 3) {
		TOUCH_E("But skip err handling, err_cnt = %d\n", d->err_cnt);
		ret = -ERANGE;
	}

	/* Status Logging */
	if (ret != 0) {
		for (i = 0, idx = 0; i < IC_STATUS_INFO_NUM; i++) {
			idx = ic_status_info_idx[i];
			if (((status_mask >> 32) & 0xFFFFFFFF) & (1 << idx)) {
				if (ic_status_info_str[idx] != NULL) {
					TOUCH_E("[IC_STATUS_INFO][%d]%s, status = %016llx, ic_status = 0x%08x\n",
							idx, ic_status_info_str[idx],
							(u64)status, d->info.ic_status);
				}
			}
		}
		for (i = 0, idx = 0; i < TC_STATUS_INFO_NUM; i++) {
			idx = tc_status_info_idx[i];
			if ((status_mask & 0xFFFFFFFF) & (1 << idx)) {
				if (tc_status_info_str[idx] != NULL) {
					TOUCH_E("[TC_STATUS_INFO][%d]%s, status = %016llx, tc_status = 0x%08x\n",
							idx, tc_status_info_str[idx],
							(u64)status, d->info.tc_status);
				}
			}
		}
	}

	return ret;
}

static void sw49107_palm_log(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	u16 old_mask = d->palm_old_mask;
	u16 new_mask = d->palm_new_mask;
	u16 press_mask = 0;
	u16 release_mask = 0;
	u16 change_mask = 0;
	int i;

	TOUCH_TRACE();

	change_mask = old_mask ^ new_mask;
	press_mask = new_mask & change_mask;
	release_mask = old_mask & change_mask;

	TOUCH_D(ABS, "palm_mask [new: %04x, old: %04x]\n",
			new_mask, old_mask);
	TOUCH_D(ABS, "palm_mask [change: %04x, press: %04x, release: %04x]\n",
			change_mask, press_mask, release_mask);

	if(!change_mask)
		goto end;

	for (i = 0; i < MAX_FINGER; i++) {
		if (new_mask & (1 << i)) {
			if (press_mask & (1 << i)) {
				TOUCH_I("%d  Palm detected:<%d>(%4d,%4d,%4d)\n",
						d->pcount,
						i,
						ts->tdata[i].x,
						ts->tdata[i].y,
						ts->tdata[i].pressure);
			}
		} else if (release_mask & (1 << i)) {
			TOUCH_I("   Palm released:<%d>(%4d,%4d,%4d)\n",
					i,
					ts->tdata[i].x,
					ts->tdata[i].y,
					ts->tdata[i].pressure);
		}
	}

end:
	d->palm_old_mask = new_mask;
	return ;
}

static int sw49107_irq_abs_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	struct sw49107_touch_data *data = d->info.data;
	struct touch_data *tdata;
	u8 finger_index = 0;
	u8 palm_index = 0;
	int ret = 0;
	int i = 0;

	ts->new_mask = 0;
	d->palm_new_mask = 0;

	/* check q cover status */
	if (d->driving_mode == LCD_MODE_U3_QUICKCOVER && !d->q_sensitivity) {
		TOUCH_I("Interrupt in Qcover closed\n");
		ts->is_cancel = 1;
		ts->tcount = 0;
		ts->intr_status = TOUCH_IRQ_FINGER;
		return ret;
	}

	for (i = 0; i < d->info.touch_cnt; i++) {
		if ((data[i].track_id >= MAX_FINGER) || (data[i].track_id < 0))
			continue;

		tdata = ts->tdata + data[i].track_id;
		tdata->id = data[i].track_id;
		tdata->type = data[i].tool_type;
		tdata->x = data[i].x;
		tdata->y = data[i].y;
		tdata->pressure = data[i].pressure;
		tdata->width_major = data[i].width_major;
		tdata->width_minor = data[i].width_minor;
		tdata->orientation = (s8)(data[i].angle);

		TOUCH_D(ABS, "tdata [id:%d t:%d x:%d y:%d z:%d-%d,%d,%d]\n",
				tdata->id,
				tdata->type,
				tdata->x,
				tdata->y,
				tdata->pressure,
				tdata->width_major,
				tdata->width_minor,
				tdata->orientation);

		switch (data[i].event) {
			case TOUCHSTS_DOWN:
			case TOUCHSTS_MOVE:
				if (d->p_param.palm_log == FUNC_ON && tdata->pressure == Z_MAX_VALUE) { // Handling Palm
					d->palm_new_mask |= (1 << data[i].track_id);
					palm_index++;
				} else {
					ts->new_mask |= (1 << data[i].track_id);
					finger_index++;
				}
				break;
		}
	}

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	if(d->p_param.palm_log == FUNC_ON) {
		d->pcount = palm_index;
		sw49107_palm_log(dev);
	}

	return ret;
}

int sw49107_irq_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);

	/* check water mode status -- start */
	int check_water_mode = d->info.current_mode;
	int check_water_old_mode = atomic_read(&d->water_old_mode);
	int change_mask = 0;

	change_mask = check_water_mode ^ check_water_old_mode;
	if (change_mask && check_water_mode == MODE_IN_WATER) {
		TOUCH_I("Water Mode On\n");
		touch_send_uevent(ts, TOUCH_UEVENT_WATER_MODE_ON);
	} else if (change_mask && check_water_mode == MODE_NORMAL) {
		TOUCH_I("Water Mode Off\n");
		touch_send_uevent(ts, TOUCH_UEVENT_WATER_MODE_OFF);
	}
	atomic_set(&d->water_old_mode, check_water_mode);
	/* check water mode status -- end */


	/* check if touch cnt is valid */
	if (d->info.touch_cnt == 0 || d->info.touch_cnt > ts->caps.max_id) {
		TOUCH_I("%s : touch cnt is invalid - %d\n", __func__, d->info.touch_cnt);
		return -ERANGE;
	}

	return sw49107_irq_abs_data(dev);
}

int sw49107_irq_lpwg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0;

	if (d->info.wakeup_type == KNOCK_1) {
		if (ts->lpwg.mode != LPWG_NONE) {
			sw49107_get_tci_data(dev, ts->tci.info[TCI_1].tap_count);
			ts->intr_status = TOUCH_IRQ_KNOCK;
		}
	} else if (d->info.wakeup_type == KNOCK_2) {
		if (ts->lpwg.mode >= LPWG_PASSWORD) {
			sw49107_get_tci_data(dev, ts->tci.info[TCI_2].tap_count);
			ts->intr_status = TOUCH_IRQ_PASSWD;
		}
	} else if (d->info.wakeup_type == SWIPE_UP) {
		TOUCH_I("SWIPE_UP\n");
		sw49107_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_UP;
	} else if (d->info.wakeup_type == SWIPE_DOWN) {
		TOUCH_I("SWIPE_DOWN\n");
		sw49107_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_DOWN;
	} else if (d->info.wakeup_type == SWIPE_LEFT) {
		TOUCH_I("SWIPE_LEFT\n");
		sw49107_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_LEFT;
	} else if (d->info.wakeup_type == SWIPE_RIGHT) {
		TOUCH_I("SWIPE_RIGHT\n");
		sw49107_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_RIGHT;
	} else if (d->info.wakeup_type == KNOCK_OVERTAP) {
		TOUCH_I("Overtap\n");
		sw49107_get_tci_data(dev, 1);
		ts->intr_status = TOUCH_IRQ_PASSWD;
	} else if (d->info.wakeup_type == CUSTOM_DEBUG) {
		TOUCH_I("CUSTOM_DEBUG\n");
		sw49107_lpwg_failreason(dev);
	} else {
		TOUCH_I("not supported LPWG wakeup_type [%d]\n", d->info.wakeup_type);
	}

	return ret;
}

int sw49107_irq_debug(struct device *dev) {
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0, i = 0;
	int count = 0;

	ret = sw49107_reg_read(dev, DEBUG_INFO_ADDR, &d->debug_info, sizeof(d->debug_info));
	if (ret < 0)
		goto error;

	TOUCH_D(ABS, "%s : debug_info.type: %x, debug_info.length: %x\n", __func__, d->debug_info.type, d->debug_info.length);

	if (d->debug_info.type < DEBUG_INFO_NUM)
		TOUCH_E("[DEBUG_TYPE] [%d]%s \n", d->debug_info.type, debug_info_str[d->debug_info.type]);

	if (d->debug_info.length > 0 && d->debug_info.length <= 12) {
		count = d->debug_info.length / 4;
		for (i = 0; i < count; i++) {
			TOUCH_E("[DEBUG_INFO] Info[%d]: %x", 2 - i, d->debug_info.info[2 - i]);
		}
	}

error:
	return ret;

}

int sw49107_irq_read_data(struct device *dev) {
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0;
	u8 intr_type_temp = 0;

	if(d->p_param.flex_report) {
		ret = sw49107_reg_read(dev, IC_STATUS, &d->info,
				sizeof(d->info) - (REPORT_PACKET_SIZE * REPORT_PACKET_EXTRA_COUNT));
		TOUCH_D(ABS, "Base Read - Addr: %x, ic: %x, tc:%x, Dst: %p, Touch_cnt: %d, Size: %lu\n",
				IC_STATUS, d->info.ic_status, d->info.tc_status, &d->info, d->info.touch_cnt,
				sizeof(d->info) - (REPORT_PACKET_SIZE * REPORT_PACKET_EXTRA_COUNT));
		if(ret < 0) {
			TOUCH_E("Register read fail\n");
			d->err_cnt++;
			ret = -EGLOBALRESET;
			goto error;
		}

		intr_type_temp = ((d->info.tc_status >> 16) & 0xF);

		if (intr_type_temp == INTR_TYPE_REPORT_PACKET) {
			if (d->info.touch_cnt > REPORT_PACKET_BASE_COUNT && d->info.touch_cnt <= MAX_FINGER) {
				ret = sw49107_reg_read(dev, REPORT_PACKET_EXTRA_DATA,
						&d->info.data[REPORT_PACKET_BASE_COUNT],
						REPORT_PACKET_SIZE * (d->info.touch_cnt - REPORT_PACKET_BASE_COUNT));
				TOUCH_D(ABS, "Extra Read - Addr: %x, Dst: %p, Touch_cnt: %d, Size: %d\n",
						REPORT_PACKET_EXTRA_DATA, &d->info.data[REPORT_PACKET_BASE_COUNT], d->info.touch_cnt,
						REPORT_PACKET_SIZE * (d->info.touch_cnt - REPORT_PACKET_BASE_COUNT));
				if(ret < 0) {
					TOUCH_E("Register read fail\n");
					d->err_cnt++;
					ret = -EGLOBALRESET;
					goto error;
				}
			}
		} else {
			ret = sw49107_reg_read(dev, REPORT_PACKET_EXTRA_DATA,
					&d->info.data[REPORT_PACKET_BASE_COUNT],
					REPORT_PACKET_SIZE * (MAX_FINGER - REPORT_PACKET_BASE_COUNT));
			TOUCH_D(ABS, "Not ReportType Extra Read - Addr: %x, Dst: %p, Touch_cnt: %d, Size: %d\n",
					REPORT_PACKET_EXTRA_DATA, &d->info.data[REPORT_PACKET_BASE_COUNT], MAX_FINGER,
					REPORT_PACKET_SIZE * (MAX_FINGER - REPORT_PACKET_BASE_COUNT));
			if(ret < 0) {
				TOUCH_E("Register read fail\n");
				d->err_cnt++;
				ret = -EGLOBALRESET;
				goto error;
			}
		}


	} else {
		ret = sw49107_reg_read(dev, IC_STATUS, &d->info, sizeof(d->info));
		if(ret < 0) {
			TOUCH_E("Register read fail\n");
			d->err_cnt++;
			ret = -EGLOBALRESET;
			goto error;
		}
	}

	return ret;

error:
	if (d->err_cnt > 3) {
		TOUCH_I("%s : But skip err handling, err_cnt = %d\n", __func__, d->err_cnt);
		ret = -ERANGE;
	}

	return ret;
}

int sw49107_irq_handler(struct device *dev)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0;

	ret = sw49107_irq_read_data(dev);
	if (ret < 0) {
		goto error;
	}

	ret = sw49107_check_status(dev);

	d->intr_type = ((d->info.tc_status >> 16) & 0xF);
	switch (d->intr_type) {
		TOUCH_D(ABS, "%s : intr_type: %x\n", __func__, (int)d->intr_type);
		case INTR_TYPE_REPORT_PACKET:
			if (d->info.wakeup_type == ABS_MODE)
				sw49107_irq_abs(dev);
			else
				sw49107_irq_lpwg(dev);
			break;
		case INTR_TYPE_ABNORMAL_ERROR_REPORT:
		case INTR_TYPE_DEBUG_REPORT:
			sw49107_irq_debug(dev);
			break;
		case INTR_TYPE_INIT_COMPLETE:
			TOUCH_I("Init Complete Interrupt!\n");
			break;
		case INTR_TYPE_BOOT_UP_DONE:
			TOUCH_I("Boot Up Done Interrupt!\n");
			break;
		default:
			TOUCH_E("Unknown Interrupt, type: %d\n", d->intr_type);
			break;
	}

error:
	return ret;
}

static ssize_t store_reg_ctrl(struct device *dev, const char *buf, size_t count)
{
	char command[6] = {0};
	u32 reg = 0;
	int value = 0;
	u32 data = 1;
	u16 reg_addr;

	if (sscanf(buf, "%5s %x %d", command, &reg, &value) <= 0)
		return count;

	reg_addr = reg;
	if (!strcmp(command, "write")) {
		data = value;
		if (sw49107_reg_write(dev, reg_addr, &data, sizeof(u32)) < 0)
			TOUCH_E("reg addr 0x%x write fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else if (!strcmp(command, "read")) {
		if (sw49107_reg_read(dev, reg_addr, &data, sizeof(u32)) < 0)
			TOUCH_E("reg addr 0x%x read fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else {
		TOUCH_D(BASE_INFO, "Usage\n");
		TOUCH_D(BASE_INFO, "Write reg value\n");
		TOUCH_D(BASE_INFO, "Read reg\n");
	}
	return count;
}

static ssize_t show_lpwg_failreason(struct device *dev, char *buf)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0;
	u32 rdata = -1;

	if (sw49107_reg_read(dev, LPWG_FAILREASON_ON_CTRL, (u8 *)&rdata, sizeof(rdata)) < 0) {
		TOUCH_I("Fail to Read Failreason On Ctrl\n");
		return ret;
	}

	ret = snprintf(buf + ret, PAGE_SIZE,
			"Failreason Ctrl[IC] = %s\n", (rdata & 0x1) ? "Enable" : "Disable");
	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Failreason Ctrl[Driver] = %s\n", d->lpwg_failreason_ctrl ? "Enable" : "Disable");
	TOUCH_I("Failreason Ctrl[IC] = %s\n", (rdata & 0x1) ? "Enable" : "Disable");
	TOUCH_I("Failreason Ctrl[Driver] = %s\n", d->lpwg_failreason_ctrl ? "Enable" : "Disable");

	return ret;
}

static ssize_t store_lpwg_failreason(struct device *dev, const char *buf, size_t count)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value > 1 || value < 0) {
		TOUCH_I("Set Lpwg Failreason Ctrl - 0(disable), 1(enable) only\n");
		return count;
	}

	d->lpwg_failreason_ctrl = (u8)value;
	TOUCH_I("Set Lpwg Failreason Ctrl = %s\n", value ? "Enable" : "Disable");

	return count;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	sw49107_reset_ctrl(dev, value);

	return count;
}

static ssize_t store_q_sensitivity(struct device *dev, const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	mutex_lock(&ts->lock);

	TOUCH_D(QUICKCOVER, "%s: change sensitivity %d -> %d", __func__, d->q_sensitivity, (value));
	d->q_sensitivity = (value); /* 1=enable touch, 0=disable touch */

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
		goto out;

	sw49107_reg_write(dev, COVER_SENSITIVITY_CTRL, &d->q_sensitivity, sizeof(u32));

out:
	mutex_unlock(&ts->lock);

	TOUCH_I("%s : %s(%d)\n", __func__,
			(d->q_sensitivity) ? "SENSITIVE" : "NORMAL", (d->q_sensitivity));

	return count;
}

static ssize_t show_pinstate(struct device *dev, char *buf)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	ret = snprintf(buf, PAGE_SIZE, "RST:%d, INT:%d\n",
			gpio_get_value(ts->reset_pin), gpio_get_value(ts->int_pin));
	TOUCH_I("%s() buf:%s",__func__, buf);
	return ret;
}

static ssize_t show_pen_support(struct device *dev, char *buf)
{
	int ret = 0;

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "%d\n", 1);

	return ret;
}

static ssize_t show_select_tune_table(struct device *dev, char *buf)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0;

/* Start - kiwoo.han 2018-11-23 */
#if 0
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "Board Rev Info = %d / Read tune table = %s\n",
			lge_get_board_revno(),
			d->select_tune_table ?"OLD_SAMPLE_TUNE_TABLE":"NEW_SAMPLE_TUNE_TABLE");
#endif
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "Read tune table = %s\n", d->select_tune_table ? "OLD_SAMPLE_TUNE_TABLE":"NEW_SAMPLE_TUNE_TABLE");
/* End - kiwoo.han 2018-11-23 */

	return ret;
}

static ssize_t store_select_tune_table(struct device *dev, const char *buf, size_t count)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	int value = NEW_SAMPLE_TUNE_TABLE;
	int ret = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;


	if (value >= NEW_SAMPLE_TUNE_TABLE && value <= OLD_SAMPLE_TUNE_TABLE) {
		ret = sw49107_reg_write(dev, SPECIAL_DYNAMIC_TUNE_TABLE_CTRL, &value, sizeof(value));
		if (ret < 0) {
			TOUCH_E("failed to write \'SPECIAL_DYNAMIC_TUNE_TABLE_CTRL\', ret:%d\n", ret);
			return count;
		}
		TOUCH_I("Write tune table = %s\n",
				value == NEW_SAMPLE_TUNE_TABLE ?"NEW_SAMPLE_TUNE_TABLE":"OLD_SAMPLE_TUNE_TABLE");
		d->select_tune_table = (value);
	}

	return count;
}

static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(lpwg_failreason, show_lpwg_failreason, store_lpwg_failreason);
static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);
static TOUCH_ATTR(q_sensitivity, NULL, store_q_sensitivity);
static TOUCH_ATTR(pinstate, show_pinstate, NULL);
static TOUCH_ATTR(pen_support, show_pen_support, NULL);
static TOUCH_ATTR(select_tune_table, show_select_tune_table, store_select_tune_table);


static struct attribute *sw49107_attribute_list[] = {
	&touch_attr_reg_ctrl.attr,
	&touch_attr_lpwg_failreason.attr,
	&touch_attr_reset_ctrl.attr,
	&touch_attr_q_sensitivity.attr,
	&touch_attr_pinstate.attr,
	&touch_attr_pen_support.attr,
	&touch_attr_select_tune_table.attr,
	NULL,
};

static const struct attribute_group sw49107_attribute_group = {
	.attrs = sw49107_attribute_list,
};

static int sw49107_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &sw49107_attribute_group);
	if (ret < 0) {
		TOUCH_E("sw49107 sysfs register failed\n");

		goto error;
	}

	sw49107_prd_register_sysfs(dev);
	if (ret < 0) {
		TOUCH_E("sw49107 register failed\n");

		goto error;
	}

	return 0;

error:
	kobject_del(&ts->kobj);

	return ret;
}

static int sw49107_get_cmd_version(struct device *dev, char *buf)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	int offset = 0;

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"version: v%d.%02d, build: %d, chip id: %d, protocol: %d\n",
			d->ic_info.version.major_ver, d->ic_info.version.minor_ver, d->ic_info.version.build_ver,
			d->ic_info.version.chip_id, d->ic_info.version.protocol_ver);
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"product id: [%s], chip rev: %x\n", d->ic_info.product_id, d->ic_info.pt_info.chip_rev);
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"lcm type: %d, lot_num: %d, fpc_type: %d\n",
			d->ic_info.pt_info.lcm_type, d->ic_info.pt_info.lot_num, d->ic_info.pt_info.fpc_type);
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"date: %04d.%02d.%02d, time: %02d:%02d:%02d, site: %d\n",
			d->ic_info.pt_info.pt_date_year, d->ic_info.pt_info.pt_date_month, d->ic_info.pt_info.pt_date_day,
			d->ic_info.pt_info.pt_time_hour, d->ic_info.pt_info.pt_time_min, d->ic_info.pt_info.pt_time_sec,
			d->ic_info.pt_info.pt_site);

	return offset;
}

static int sw49107_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	int offset = 0;

	offset = snprintf(buf + offset, PAGE_SIZE - offset,
			"v%d.%02d\n", d->ic_info.version.major_ver, d->ic_info.version.minor_ver);

	return offset;
}

static int sw49107_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int sw49107_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
		case CMD_VERSION:
			ret = sw49107_get_cmd_version(dev, (char *)output);
			break;

		case CMD_ATCMD_VERSION:
			ret = sw49107_get_cmd_atcmd_version(dev, (char *)output);
			break;

		default:
			break;
	}

	return ret;
}

static struct touch_driver touch_driver = {
	.probe = sw49107_probe,
	.remove = sw49107_remove,
	.shutdown = sw49107_shutdown,
	.suspend = sw49107_suspend,
	.resume = sw49107_resume,
	.init = sw49107_init,
	.irq_handler = sw49107_irq_handler,
	.power = sw49107_power,
	.upgrade = sw49107_upgrade,
	.esd_recovery = sw49107_esd_recovery,
	.lpwg = sw49107_lpwg,
	.swipe_enable = sw49107_swipe_enable,
	.notify = sw49107_notify,
	.init_pm = sw49107_init_pm,
	.register_sysfs = sw49107_register_sysfs,
	.set = sw49107_set,
	.get = sw49107_get,
};

#define MATCH_NAME	"lge,sw49107"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{ },
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(touch_match_ids),
};

#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
extern int lge_get_maker_id(void);
#endif
#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
extern int lge_get_panel_id(void);
#endif

extern bool panel_type_touch(char *panel_name);

static int __init touch_device_init(void)
{

	TOUCH_TRACE();

	TOUCH_I("touch_device_init func\n");

	if(!panel_type_touch("lcd")) {
		TOUCH_I("panel_type_touch not found\n");
		return 0;
	}

#if defined(CONFIG_LGE_TOUCH_MODULE_DETECT)
#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
	panel_id = lge_get_panel_id();

	switch (panel_id) {
		case LGD_INCELL_SW49107:
		case TOVIS_INCELL_SW49107:
			TOUCH_I("%s, sw49107 found!\n", __func__);
			break;
		default:
			TOUCH_I("%s, sw49107 not found! : %d\n", __func__, panel_id);
			return 0;
	}
#elif defined(CONFIG_LGE_TOUCH_CORE_MTK)
	panel_id = lge_get_maker_id(); //0: LGD panel, 1: TOVIS panel

	switch (panel_id) {
		case LGD_INCELL_SW49107:
		case TOVIS_INCELL_SW49107:
			TOUCH_I("%s, sw49107 found!\n", __func__);
			break;
		default:
			TOUCH_I("%s, sw49107 not found.\n", __func__);
			return 0;
	}
#endif
#endif

	return touch_bus_device_init(&hwif, &touch_driver);
}



static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

late_initcall(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("BSP-TOUCH@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");
