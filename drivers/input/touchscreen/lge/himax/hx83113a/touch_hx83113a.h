/*  Himax Android Driver Sample Code for QCT platform

    Copyright (C) 2018 Himax Corporation.

    This software is licensed under the terms of the GNU General Public
    License version 2, as published by the Free Software Foundation, and
    may be copied, distributed, and modified under those terms.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*/

#ifndef HIMAX_PLATFORM_H
#define HIMAX_PLATFORM_H

#include <asm/segment.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/async.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_OF
	#include <linux/of_gpio.h>
#endif
#if defined(CONFIG_HMX_DB)
	#include <linux/regulator/consumer.h>
#endif

#define HIMAX_DRIVER_VER "1.2.2.3_LGE_ACE_06"

#define FLASH_DUMP_FILE "/sdcard/HX_Flash_Dump.bin"

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
#define HX_TP_PROC_2T2R
#endif
/*===========Himax Option function=============*/
#define HX_FIX_TOUCH_INFO	/* if open, you need to change the touch info in the fix_touch_info*/
#define HX_P_SENSOR
#define CUST_TP_SIZE 100
#define CUST_TP_SHAPE_SIZE 44

#define HX_KEY_MAX_COUNT             4
#define DEFAULT_RETRY_CNT            3

#define HX_85XX_A_SERIES_PWON		"HX85xxA"
#define HX_85XX_B_SERIES_PWON		"HX85xxB"
#define HX_85XX_C_SERIES_PWON		"HX85xxC"
#define HX_85XX_D_SERIES_PWON		"HX85xxD"
#define HX_85XX_E_SERIES_PWON		"HX85xxE"
#define HX_85XX_ES_SERIES_PWON		"HX85xxES"
#define HX_85XX_F_SERIES_PWON		"HX85xxF"
#define HX_85XX_H_SERIES_PWON		"HX85xxH"
#define HX_83100A_SERIES_PWON		"HX83100A"
#define HX_83102A_SERIES_PWON		"HX83102A"
#define HX_83102B_SERIES_PWON		"HX83102B"
#define HX_83102D_SERIES_PWON		"HX83102D"
#define HX_83103A_SERIES_PWON		"HX83103A"
#define HX_83110A_SERIES_PWON		"HX83110A"
#define HX_83110B_SERIES_PWON		"HX83110B"
#define HX_83111B_SERIES_PWON		"HX83111B"
#define HX_83112A_SERIES_PWON		"HX83112A"
#define HX_83112B_SERIES_PWON		"HX83112B"
#define HX_83112D_SERIES_PWON		"HX83112D"
#define HX_83112E_SERIES_PWON		"HX83112E"
#define HX_83113A_SERIES_PWON		"HX83113A"
#define HX_83191A_SERIES_PWON		"HX83191A"

#define HX_TP_BIN_CHECKSUM_SW		1
#define HX_TP_BIN_CHECKSUM_HW		2
#define HX_TP_BIN_CHECKSUM_CRC		3

#define SHIFTBITS 5

#define  FW_SIZE_32k 	32768
#define  FW_SIZE_60k 	61440
#define  FW_SIZE_64k 	65536
#define  FW_SIZE_124k 	126976
#define  FW_SIZE_128k 	131072

#define NO_ERR 0
#define READY_TO_SERVE 1
#define WORK_OUT	2
#define I2C_FAIL -1
#define MEM_ALLOC_FAIL -2
#define CHECKSUM_FAIL -3
#define GESTURE_DETECT_FAIL -4
#define INPUT_REGISTER_FAIL -5
#define FW_NOT_READY -6
#define LENGTH_FAIL -7
#define OPEN_FILE_FAIL -8
#define ERR_WORK_OUT	-10
#define ERR_STS_WRONG	-11
#define HW_CRC_FAIL 1

#define HX_FINGER_ON	1
#define HX_FINGER_LEAVE	2

#define LPWG_SENSELESS_AREA_W		70	/* pixel */

enum {
	BOE_INCELL_ILI9881H = 0,
	CSOT_INCELL_FT8736,
	MH4_BOE_hx83113a,
};

enum {
    SW_RESET = 0,
    HW_RESET,
};

enum HX_TS_PATH {
	HX_REPORT_COORD = 1,
	HX_REPORT_SMWP_EVENT,
	HX_REPORT_COORD_RAWDATA,
};

/* swipe */
enum {
	SWIPE_ENABLE_CTRL = 0,
	SWIPE_DISABLE_CTRL,
	SWIPE_DIST_CTRL,
	SWIPE_RATIO_THR_CTRL,
	SWIPE_RATIO_PERIOD_CTRL,
	SWIPE_RATIO_DIST_CTRL,
	SWIPE_TIME_MIN_CTRL,
	SWIPE_TIME_MAX_CTRL,
	SWIPE_AREA_CTRL,
	SWIPE_WRONG_DIRECTION_THRESHOLD,
	SWIPE_INITIAL_RATIO_CHECK_DIST,
	SWIPE_INITIAL_RATIO_THRESHOLD,
};

enum {
	SWIPE_DISABLE = 0,
	SWIPE_ENABLE,
};

enum {
	HX83113A_SWIPE_U = 0,
	HX83113A_SWIPE_D = 1,
	HX83113A_SWIPE_L = 2,
	HX83113A_SWIPE_R = 3,
	HX83113A_SWIPE_NUM = 4,
};

struct hx83113a_active_area {
	s16 x1;
	s16 y1;
	s16 x2;
	s16 y2;
} __packed;

struct hx83113a_swipe_data {
	u8 enable;
	u8 distance;
	u8 ratio_thres;
	u16 min_time;
	u16 max_time;
	struct hx83113a_active_area area;
	struct hx83113a_active_area start_area;
	u8 wrong_dir_thres;
	u8 init_ratio_chk_dist;
	u8 init_ratio_thres;
} __packed;

struct hx83113a_swipe_buf {
	struct hx83113a_swipe_data data[HX83113A_SWIPE_NUM];
} __packed;

enum HX_TS_STATUS {
	HX_TS_GET_DATA_FAIL = -11,
	HX_ESD_EVENT,
	HX_CHKSUM_FAIL,
	HX_PATH_FAIL,
	HX_ESD_REC_OK,
	HX_READY_SERVE,
	HX_ESD_WARNING,
	HX_IC_RUNNING,
	HX_ZERO_EVENT_COUNT,
	HX_RST_OK,
	HX_TS_NORMAL_END,
	HX_REPORT_DATA = 0,
};

enum {
    LCD_MODE_U0 = 0,
    LCD_MODE_U2_UNBLANK,
    LCD_MODE_U2,
    LCD_MODE_U3,
    LCD_MODE_U3_PARTIAL,
    LCD_MODE_U3_QUICKCOVER,
    LCD_MODE_STOP,
};

enum cell_type {
	CHIP_IS_ON_CELL,
	CHIP_IS_IN_CELL
};
#ifdef HX_FIX_TOUCH_INFO
enum fix_touch_info {
	FIX_HX_RX_NUM = 36,
	FIX_HX_TX_NUM = 16,
	FIX_HX_BT_NUM = 0,
	FIX_HX_X_RES = 1080,
	FIX_HX_Y_RES = 2400,
	FIX_HX_MAX_PT = 10,
	FIX_HX_XY_REVERSE = false,
	FIX_HX_INT_IS_EDGE = true,
#ifdef HX_TP_PROC_2T2R
	FIX_HX_RX_NUM_2 = 0,
	FIX_HX_TX_NUM_2 = 0,
#endif
};
#endif

struct himax_ic_info {
	int ic_official_ver;
	int ic_fw_ver;
	int bin_official_ver;
	int bin_fw_ver;
	int vendor_fw_ver;
	int vendor_config_ver;
	int vendor_touch_cfg_ver;
	int vendor_display_cfg_ver;
	int vendor_cid_maj_ver;
	int vendor_cid_min_ver;
	int vendor_panel_ver;
	int vendor_sensor_id;
};

struct himax_ic_data {
	int		HX_RX_NUM;
	int		HX_TX_NUM;
	int		HX_BT_NUM;
	int		HX_X_RES;
	int		HX_Y_RES;
	int		HX_MAX_PT;
	bool	HX_XY_REVERSE;
	bool	HX_INT_IS_EDGE;
#ifdef HX_TP_PROC_2T2R
	int HX_RX_NUM_2;
	int HX_TX_NUM_2;
#endif
};

struct himax_target_report_data {
	int *x;
	int *y;
	int *w;
	int *width_major;
	int *width_minor;
	int *orientation;
	int *finger_id;
	int finger_on;
	int finger_num;
	int SMWP_event_chk;
};

struct himax_report_data {
	int touch_all_size;
	int raw_cnt_max;
	int raw_cnt_rmd;
	int touch_info_size;
	uint8_t	finger_num;
	uint8_t	finger_on;
	uint8_t *hx_coord_buf;
	uint8_t *hx_width_buf;
	uint8_t hx_state_info[2];
	int event_size;
	uint8_t *hx_event_buf;
	int hx_palm;
	int rawdata_size;
	uint8_t diag_cmd;
	uint8_t *hx_rawdata_buf;
};

struct himax_ts_data {
	bool suspended;
	atomic_t suspend_mode;
	uint8_t x_channel;
	uint8_t y_channel;
	uint8_t useScreenRes;
	uint8_t diag_cmd;
	char chip_name[30];
	uint8_t chip_cell_type;

	uint8_t protocol_type;
	uint8_t first_pressed;
	uint8_t coord_data_size;
	uint8_t area_data_size;
	uint8_t coordInfoSize;
	uint8_t nFinger_support;
	uint8_t irq_enabled;

	uint16_t pre_finger_mask;
	uint16_t old_finger;
	int hx_point_num;

	uint32_t debug_log_level;
	uint32_t widthFactor;
	uint32_t heightFactor;
	uint32_t tw_x_min;
	uint32_t tw_x_max;
	uint32_t tw_y_min;
	uint32_t tw_y_max;
	uint32_t pl_x_min;
	uint32_t pl_x_max;
	uint32_t pl_y_min;
	uint32_t pl_y_max;

	int rst_gpio;
	int pre_finger_data[10][2];

	struct device *dev;
	struct i2c_client *client;
	struct himax_i2c_platform_data *pdata;
	struct mutex rw_lock;
	struct himax_ic_info ic_info;

	struct workqueue_struct 			*flash_wq;
	struct work_struct 					flash_work;

	struct workqueue_struct *himax_diag_wq;
	struct delayed_work himax_diag_delay_wrok;

	uint8_t SMWP_enable;
	//struct wake_lock ts_SMWP_wake_lock;

	uint8_t cable_config[2];
	u8 lcd_mode;
	struct delayed_work fb_notify_work;
};

struct himax_debug {
	bool flash_dump_going;
	void (*fp_ts_dbg_func)(struct himax_ts_data *d, int start);
	int (*fp_set_diag_cmd)(struct himax_ic_data *ic_data, struct himax_report_data *hx_touch_data);
};

enum input_protocol_type {
	PROTOCOL_TYPE_A	= 0x00,
	PROTOCOL_TYPE_B	= 0x01,
};

#define GEST_PTLG_ID_LEN	(4)
#define GEST_PTLG_HDR_LEN	(4)
#define GEST_PTLG_HDR_ID1	(0xCC)
#define GEST_PTLG_HDR_ID2	(0x44)
#define GEST_PT_MAX_NUM 	(128)
#define GEST_PT_STATE_FR	(45)

#define GEST_FAIL_TYPE_MSG		(8) //TYPE ID + Fail msg
#define GEST_PT_COUNT		(9)
#define GEST_FAIL_STATUS		(10) //TCI status or SWIPE time
#define GEST_STATE_COOR	(12)
//#define EV_GESTURE_FR		(0x81)
#define HX_MAX_MSG_NUM 10
#define HX_MSG_SIZE 20
#define HX_ADDR_DBG_KNOCK_ON 0x100007A8
#define HX_ADDR_DBG_SWIPE 0x100006E0

#define HIMAX_I2C_RETRY_TIMES 10
#define HIMAX_common_NAME 				"himax_tp"

struct himax_i2c_platform_data {
	int abs_x_min;
	int abs_x_max;
	int abs_x_fuzz;
	int abs_y_min;
	int abs_y_max;
	int abs_y_fuzz;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_pressure_fuzz;
	int abs_width_min;
	int abs_width_max;
	int screenWidth;
	int screenHeight;
	uint8_t fw_version;
	uint8_t tw_id;
	//uint8_t cable_config[2];
	uint8_t protocol_type;
	int gpio_irq;
	int gpio_reset;
	int gpio_3v3_en;

	int hx_config_size;
#if defined(CONFIG_HMX_DB)
	bool	i2c_pull_up;
	bool	digital_pwr_regulator;
	int reset_gpio;
	u32 reset_gpio_flags;
	int irq_gpio;
	u32 irq_gpio_flags;

	struct regulator *vcc_ana; /* For Dragon Board */
	struct regulator *vcc_dig; /* For Dragon Board */
	struct regulator *vcc_i2c; /* For Dragon Board */
#endif
};

static inline struct himax_ts_data *to_himax_data(struct device *dev)
{
	return (struct himax_ts_data *)touch_get_device(to_touch_core(dev));
}

extern int hx83113a_report_data_init(void);
extern uint8_t hx83113a_int_gpio_read(int pinnum);
extern void hx83113a_burst_enable(struct device *dev, uint8_t auto_add_4_byte);
extern int hx83113a_register_read(struct device *dev, uint32_t addr, uint32_t read_length, uint8_t *read_data, uint8_t addr_len);
extern int hx83113a_register_write(struct device *dev, uint32_t addr, uint32_t write_length, uint8_t *read_data, uint8_t addr_len);
extern void hx83113a_interface_on(struct device *dev);
extern void hx83113a_sense_on(struct device *dev, uint8_t FlashMode);
extern bool hx83113a_sense_off(struct device *dev, bool check_en);
extern void hx83113a_set_SMWP_enable(struct device *dev, uint8_t SMWP_enable);
extern int hx83113a_read_FW_ver(struct device *dev);
extern void hx83113a_print_FW_ver(struct device *dev);
extern bool hx83113a_calculateChecksum(struct device *dev);
extern int hx83113a_assign_sorting_mode(struct device *dev, uint8_t *tmp_data);
extern int hx83113a_fts_ctpm_fw_upgrade_with_sys_fs_128k(struct device *dev, unsigned char *fw, int len);
extern void hx83113a_ic_reset(struct device *dev, uint8_t loadconfig, uint8_t int_off);
extern void hx83113a_touch_information(struct device *dev);
extern void hx83113a_SMWP_dbg_msg(struct device *dev);
#endif
