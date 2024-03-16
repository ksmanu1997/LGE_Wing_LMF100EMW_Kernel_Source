/* touch_sw49107.h
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: PH1-BSP-Touch@lge.com
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

#ifndef LGE_TOUCH_SW49107_H
#define LGE_TOUCH_SW49107_H

#define Z_MAX_VALUE			255

#define VCHIP_VAL			16
#define VPROTO_VAL			4

/* device control */
#define TC_VERSION			(0x312)	//(0x242)
#define TC_PRODUCT_CODE			(0x313)
#define TC_PRODUCT_ID1			(0x314)	//(0x244)
#define TC_PRODUCT_ID2			(0x315)	//(0x245)
#define TC_CFG_VERSION			(0x317)
#define IC_STATUS			(0x200)	/* sw49107_touch_info base addr */
#define TC_STATUS			(0x201)
#define TC_DEVICE_CTL			(0xC00)
#define TC_INTERRUPT_CTL		(0xC01)
#define TC_INTERRUPT_CLR		(0xC02)
#define TC_DRIVING_CTL			(0xC03)

#define PT_INFO_LCM_TYPE		(0x32C) /* sw49107_pt_info base addr */
#define PT_INFO_LOT_NUM			(0x32D)
#define PT_INFO_FPC_TYPE		(0x32E)
#define PT_INFO_PT_DATE			(0x32F)
#define PT_INFO_PT_TIME			(0x330)
#define PT_INFO_CHIP_REV		(0x331)

#define SPR_SUBDISP_ST			(0x110)	//(0x01D)
#define SPR_BOOT_ST			(0x010)

#define SERIAL_SPI_EN			(0xFE4)
#define SERIAL_I2C_EN			(0xFE5)
#define SPI_TATTN_OPT			(0xFF3)

#define DEBUG_INFO_ADDR			(0x23E)
#define DEBUG_INFO_HEADER_ADDR		(0x241)

#define TCI_ENABLE_CTRL			(0xF10)
#define TCI_TOTAL_TAP_COUNT_CTRL	(0xF11)
#define TCI_INTER_TAP_TIME_MIN_CTRL	(0xF12)
#define TCI_INTER_TAP_TIME_MAX_CTRL	(0xF13)
#define TCI_INNER_TAP_DIST_MAX_CTRL	(0xF14) // Touch Slop
#define TCI_INTER_TAP_DISP_MAX_CTRL	(0xF15)
#define TCI_INTERRUPT_DELAY_TIME_CTRL	(0xF16)
#define TCI_ACTIVE_AREA_X1_CTRL		(0xF17)
#define TCI_ACTIVE_AREA_Y1_CTRL		(0xF18)
#define TCI_ACTIVE_AREA_X2_CTRL		(0xF19)
#define TCI_ACTIVE_AREA_Y2_CTRL		(0xF1A)

#define SWIPE_ON_CTRL			(0xF20)
#define SWIPE_DIST_THRESHOLD_CTRL	(0xF21)
#define SWIPE_RATIO_THRESHOLD_CTRL	(0xF22)
#define SWIPE_TIME_MIN_H_CTRL		(0xF23)
#define SWIPE_TIME_MIN_V_CTRL		(0xF24)
#define SWIPE_TIME_MAX_H_CTRL		(0xF25)
#define SWIPE_TIME_MAX_V_CTRL		(0xF26)
#define SWIPE_ACT_AREA_X1_H_CTRL	(0xF27)
#define SWIPE_ACT_AREA_Y1_H_CTRL	(0xF28)
#define SWIPE_ACT_AREA_X2_H_CTRL	(0xF29)
#define SWIPE_ACT_AREA_Y2_H_CTRL	(0xF2A)
#define SWIPE_ACT_AREA_X1_V_CTRL	(0xF2B)
#define SWIPE_ACT_AREA_Y1_V_CTRL	(0xF2C)
#define SWIPE_ACT_AREA_X2_V_CTRL	(0xF2D)
#define SWIPE_ACT_AREA_Y2_V_CTRL	(0xF2E)
#define SWIPE_START_AREA_X1_H_CTRL		(0xF2F)
#define SWIPE_START_AREA_Y1_H_CTRL		(0xF30)
#define SWIPE_START_AREA_X2_H_CTRL		(0xF31)
#define SWIPE_START_AREA_Y2_H_CTRL		(0xF32)
#define SWIPE_START_AREA_X1_V_CTRL		(0xF33)
#define SWIPE_START_AREA_Y1_V_CTRL		(0xF34)
#define SWIPE_START_AREA_X2_V_CTRL		(0xF35)
#define SWIPE_START_AREA_Y2_V_CTRL		(0xF36)
#define SWIPE_WRONG_DIRECTION_THD_CTRL	(0xF37)
#define SWIPE_INIT_RATIO_CHK_DIST_CTRL	(0xF38)
#define SWIPE_INIT_RATIO_THD_CTRL		(0xF39)

#define COVER_SENSITIVITY_CTRL		(0xF36)
#define COVER_ACTIVE_AREA_START_XY_CTRL	(0xF37)
#define COVER_ACTIVE_AREA_END_XY_CTRL	(0xF38)

#define LPWG_FAILREASON_ON_CTRL		(0xF50)
#define LPWG_FAILREASON_STS_CTRL	(0xF51)
#define TCI_FAILREASON_BUF		(0xF52)
#define SWIPE_FAILREASON_BUF		(0xF53)

#define SPECIAL_CHARGER_INFO_CTRL	(0xF40)
#define SPECIAL_IME_STATUS_CTRL		(0xF41)
#define SPECIAL_DYNAMIC_TUNE_TABLE_CTRL	(0xF42)
#define SPECIAL_CALL_INFO_CTRL		(0xF44)
#define SPECIAL_INTERPOLATION_INFO_CTRL	(0xF45)
#define R_HEADER_SIZE			(0)
#define W_HEADER_SIZE			(2)

/* interrupt type */
#define INTR_TYPE_BOOT_UP_DONE		0x01
#define INTR_TYPE_INIT_COMPLETE		0x02
#define INTR_TYPE_ABNORMAL_ERROR_REPORT	0x03
#define INTR_TYPE_DEBUG_REPORT		0x04
#define INTR_TYPE_REPORT_PACKET		0x05

/* Flexible Report */
#define REPORT_PACKET_SIZE		(12) // 12byte = sizeof(sw49107_touch_data)
#define REPORT_PACKET_BASE_COUNT	(1)
#define REPORT_PACKET_EXTRA_COUNT	(MAX_FINGER - REPORT_PACKET_BASE_COUNT)
#define REPORT_PACKET_BASE_DATA		(0x203)
#define REPORT_PACKET_EXTRA_DATA	(0x203 + ((REPORT_PACKET_SIZE/4) * REPORT_PACKET_BASE_COUNT))

/* SPR control */
#define SPI_RST_CTL			0xFE0
#define SPI_CLK_CTL			0xFE2
#define SPI_OSC_CTL			0xFE1

/* Firmware control */
#define sys_rst_ctl			(0x082)	//(0x006)
#define sys_boot_ctl			(0x004)	//(0x00E)
#define sys_sram_ctl			(0x005)	//(0x00F)
#define fw_boot_code_addr		(0x0BD)	//sys_dummy 1
#define spr_code_offset			(0x0AA)
#define spr_data_offset			(0x026)	//(0x0AF)
#define tc_flash_dn_ctl			(0xC05)
#define tc_flash_dn_sts			(0x242)	//(0x247)
#define rcfg_c_sram_oft			(0xA00)
#define rcfg_s_sram_oft			(0xA80)

#define rconf_dn_index			(0x316)
//#define tc_confdn_base_addr		(0x2F9)	//change name "rconf_dn_index"
#define code_access_addr		(0xFD0)
#define data_access_addr		(0xFD1)

#define MAX_RW_SIZE			(1 * 1024)
#define FLASH_SIZE         		(128 * 1024)	//[bringup] needs to be checked
#define FLASH_CONF_SIZE			(1 * 1024)
#define FLASH_OTP_SIZE			(4 * 1024)

#define FW_BOOT_LOADER_INIT		0x74696E69	//"init"
#define FW_BOOT_LOADER_CODE		0x544F4F42	//"BOOT"

#define FLASH_KEY_CODE_CMD		0xDFC1
#define FLASH_KEY_CONF_CMD		0xE87B
#define FLASH_KEY_OTPM_CMD      	0xAE71
#define FLASH_BOOTCHK_VALUE		0x0A0A0000

#define CFG_MAGIC_CODE			0xCACACACA
#define CFG_CHIP_ID			49107
#define CFG_C_MAX_SIZE			2048
#define CFG_S_MAX_SIZE			4048

//[0]busy check [1]code crc pass [2]flash cfg common crc [3]specific crc [7:4]step [8]otp crc(x)
#define FLASH_CODE_DNCHK_VALUE		0x42
#define FLASH_CONF_DNCHK_VALUE		0x8C	//0x84
#define FLASH_OTP_DNCHK_VALUE		0xD8

#define TCI_MAX_NUM			2
#define SWIPE_MAX_NUM			2
#define TCI_DEBUG_MAX_NUM		16
#define SWIPE_DEBUG_MAX_NUM		8
#define DISTANCE_INTER_TAP		(0x1 << 1) /* 2 */
#define DISTANCE_TOUCHSLOP		(0x1 << 2) /* 4 */
#define TIMEOUT_INTER_TAP_LONG		(0x1 << 3) /* 8 */
#define MULTI_FINGER			(0x1 << 4) /* 16 */
#define DELAY_TIME			(0x1 << 5) /* 32 */
#define PALM_STATE			(0x1 << 6) /* 64 */
#define OUTOF_AREA			(0x1 << 7) /* 128 */

/* Interrupt Status for check_status func */
#define STATUS_NORMAL_MASK		((u64)IC_STATUS_NORMAL_MASK << 32 | (u64)TC_STATUS_NORMAL_MASK)
#define STATUS_GLOBAL_RESET_BIT		((u64)IC_STATUS_GLOBAL_RESET_BIT << 32 | (u64)TC_STATUS_GLOBAL_RESET_BIT)
#define STATUS_HW_RESET_BIT		((u64)IC_STATUS_HW_RESET_BIT << 32 | (u64)TC_STATUS_HW_RESET_BIT)
#define STATUS_SW_RESET_BIT		((u64)IC_STATUS_SW_RESET_BIT << 32 | (u64)TC_STATUS_SW_RESET_BIT)
#define STATUS_FW_UPGRADE_BIT		((u64)IC_STATUS_FW_UPGRADE_BIT << 32 | (u64)TC_STATUS_FW_UPGRADE_BIT)
#define STATUS_LOGGING_BIT		((u64)IC_STATUS_LOGGING_BIT << 32 | (u64)TC_STATUS_LOGGING_BIT)

#define IC_STATUS_NORMAL_MASK		0x00060010	/* (0<<01)|(0<<02)|(0<<05)|(0<<07)|(0<<08)|
							   (0<<09)|(0<<10)|(0<<11)|(0<<12)|(0<<13)|
							   (0<<16)*/
#define IC_STATUS_GLOBAL_RESET_BIT 	0x00013FA6	/* (1<<01)|(1<<02)|(1<<05)|(1<<07)|(1<<08)|
							   (1<<09)|(1<<10)|(1<<11)|(1<<12)|(1<<13)|
							   (1<<16) */
#define IC_STATUS_HW_RESET_BIT		0x00000000	/* NULL */
#define IC_STATUS_SW_RESET_BIT		0x00000000	/* NULL */
#define IC_STATUS_FW_UPGRADE_BIT	0x00000000	/* NULL */
#define IC_STATUS_LOGGING_BIT		0x00000000	/* NULL */

#define TC_STATUS_NORMAL_MASK		0x0F5580E7	/* (1<<05)|(1<<06)|(1<<07)|(0<<09)|(0<<10)|
							   (0<<13)|(1<<15)|(1<<20)|(0<<21)|(1<<22)|
							   (0<<31) */
#define TC_STATUS_GLOBAL_RESET_BIT	0x80002620	/* (1<<05)|(1<<09)|(1<<10)|(1<<13)|(1<<31) */
#define TC_STATUS_HW_RESET_BIT		0x00000000	/* NULL */
#define TC_STATUS_SW_RESET_BIT		0x00000000	/* NULL */
#define TC_STATUS_FW_UPGRADE_BIT	0x000000C0	/* (1<<06)|(1<<07) */
#define TC_STATUS_LOGGING_BIT		0x00708000	/* (1<<15)|(1<<20)|(1<<21)(1<<22) */

#define CONNECT_NONE			(0x00)
#define CONNECT_USB			(0x01)
#define CONNECT_TA			(0x02)
#define CONNECT_OTG			(0x03)
#define CONNECT_WIRELESS		(0x10)

#define TCI_DEBUG_ALL ( DISTANCE_INTER_TAP |\
			DISTANCE_TOUCHSLOP |\
			TIMEOUT_INTER_TAP_LONG |\
			MULTI_FINGER |\
			DELAY_TIME |\
			PALM_STATE |\
			OUTOF_AREA)

#if defined(CONFIG_LGE_TOUCH_CORE_MTK) || defined(CONFIG_LGE_TOUCH_CORE_QCT)
enum {
	LGD_INCELL_SW49107 = 0,
	TOVIS_INCELL_SW49107,
};
enum {
	LGD_INCELL_SW49107_HD = 0,
	TOVIS_INCELL_SW49107_HD,
};
#endif

enum {
	FUNC_OFF = 0,
	FUNC_ON,
};

enum {
	NEW_SAMPLE_TUNE_TABLE = 0,	// Board rev above 1.1
	OLD_SAMPLE_TUNE_TABLE,		// Board rev A, B, C, 1.0
};

enum {
	S_MODE_6LHB = 0,
	S_MODE_V_BLANK,
};

enum {
	SW_RESET = 0,
	HW_RESET_ASYNC,
	HW_RESET_SYNC,
	SW_RESET_CODE_DUMP,
};

enum {
	TOUCHSTS_IDLE = 0,
	TOUCHSTS_DOWN,
	TOUCHSTS_MOVE,
	TOUCHSTS_UP,
};

enum {
	MODE_NONE = 0,
	MODE_NORMAL,
	MODE_NOISE,
	MODE_QUICKCOVER,
	MODE_IN_WATER,
	MODE_LPWG,
};

enum {
	ABS_MODE = 0,
	KNOCK_1,
	KNOCK_2,
	SWIPE_UP,
	SWIPE_DOWN,
	SWIPE_LEFT,
	SWIPE_RIGHT,
	CUSTOM_DEBUG = 200,
	KNOCK_OVERTAP = 201,
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

enum {
	SWIPE_NONE = 0,
	SWIPE_UP_BIT = 1,
};

enum {
	LPWG_FAILREASON_DISABLE = 0,
	LPWG_FAILREASON_ENABLE,
};

// Vendor tmp -> Core
enum {
	SWIPE_ENABLE_CMD = 0,
	SWIPE_DISABLE_CMD,
	SWIPE_DIST_CMD,
	SWIPE_RATIO_THR_CMD,
	SWIPE_RATIO_PERIOD_CMD,
	SWIPE_RATIO_DIST_CMD,
	SWIPE_TIME_MIN_CMD,
	SWIPE_TIME_MAX_CMD,
	SWIPE_ACTIVE_AREA_CMD,
	SWIPE_WRONG_DIRECTION_THD_CMD,
	SWIPE_INIT_RATIO_CHK_DIST_CMD,
	SWIPE_INIT_RATIO_THD_CMD,
};

enum {
	IC_INIT_NEED = 0,
	IC_INIT_DONE,
};

enum {
	E_FW_CODE_SIZE_ERR = 1,
	E_FW_CODE_ONLY_VALID = 2,
	E_FW_CODE_AND_CFG_VALID = 3,
	E_FW_CODE_CFG_ERR = 4
};

enum {
	CONNECT_INVALID = 0,
	CONNECT_SDP,
	CONNECT_DCP,
	CONNECT_CDP,
	CONNECT_PROPRIETARY,
	CONNECT_FLOATED,
	CONNECT_HUB, /* SHOULD NOT change the value */
};

enum {
	CHANGING_DISPLAY_MODE_READY = 0,
	CHANGING_DISPLAY_MODE_NOT_READY,
};

#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
enum {
	CONNECT_CHARGER_UNKNOWN = 0,
	CONNECT_STANDARD_HOST,		/* USB : 450mA */
	CONNECT_CHARGING_HOST,
	CONNECT_NONSTANDARD_CHARGER,	/* AC : 450mA~1A */
	CONNECT_STANDARD_CHARGER,	/* AC : ~1A */
	CONNECT_APPLE_2_1A_CHARGER,	/* 2.1A apple charger */
	CONNECT_APPLE_1_0A_CHARGER,	/* 1A apple charger */
	CONNECT_APPLE_0_5A_CHARGER,	/* 0.5A apple charger */
	CONNECT_WIRELESS_CHARGER,
	CONNECT_DISCONNECTED,
};
#endif

struct project_param {
	u8 chip_id;
	u8 protocol;
	u8 fw_cfg_use_en;
	u32 flash_fw_size;
	u8 used_mode;
	u8 osc_clock_ctrl;
	u8 touch_power_control_en;
	u8 tci_setting;
	u8 flex_report;
	u8 dynamic_tune_table;
	u8 dsv_toggle;
	u8 deep_sleep_power_control;
	u8 palm_log;
};

struct sw49107_touch_debug_info {
	u32 info[3]; // ic_debug_info_addr : 0x23E ~ 0x240
	u32 type:24; // ic_debug_info_header_addr : 0x241
	u32 length:8;
} __packed;

/* report packet */
struct sw49107_touch_data {
	u8 tool_type:4;
	u8 event:4;
	s8 track_id;
	u16 x;
	u16 y;
	u8 pressure;
	u8 angle;
	u16 width_major;
	u16 width_minor;
} __packed;

struct sw49107_touch_info {
	u32 ic_status;
	u32 tc_status;
	u32 wakeup_type:8;
	u32 touch_cnt:5;
	u32 button_cnt:3;
	u32 current_mode:3;
	u32 palm_bit:13;
	struct sw49107_touch_data data[10];	//120byte
} __packed;

struct sw49107_tc_version {
	u8 minor_ver;
	u8 major_ver:4;
	u8 build_ver:4;
	u8 chip_id;
	u8 protocol_ver:4;
	u8 reverved:4;
} __packed;

struct sw49107_pt_info {
	u32 lcm_type;
	u32 lot_num;
	u32 fpc_type;
	u16 pt_date_year;
	u8 pt_date_month;
	u8 pt_date_day;
	u8 pt_time_hour;
	u8 pt_time_min;
	u8 pt_time_sec;
	u8 pt_site;
	u32 chip_rev;
} __packed;

struct sw49107_ic_info {
	struct sw49107_tc_version version;
	u8 product_id[9];
	struct sw49107_pt_info pt_info;
};

struct sw49107_data {
	struct device *dev;
	struct kobject kobj;
	struct sw49107_touch_info info;
	struct sw49107_touch_debug_info debug_info;
	struct sw49107_ic_info ic_info;
	struct mutex io_lock;

	struct delayed_work fb_notify_work;
	struct delayed_work int_pin_work;

	struct project_param p_param;

	atomic_t init;
	atomic_t hw_reset;
	atomic_t changing_display_mode;

	u8 lcd_mode;
	u8 driving_mode;
	u8 intr_type;
	u8 lpwg_failreason_ctrl;
	u32 charger;
	u32 q_sensitivity;
	u8 err_cnt;
	u8 boot_err_cnt;
	atomic_t water_old_mode;
	u8 select_tune_table;

	u16 palm_new_mask;
	u16 palm_old_mask;
	int pcount;
	void *prd;
};

typedef union {
	struct {
		unsigned common_cfg_size : 16;
		unsigned specific_cfg_size : 16;
	} b;
	u32 w;
} t_cfg_size;

typedef union
{
	struct {
		unsigned chip_rev : 8;
		unsigned model_id : 8;
		unsigned lcm_id : 8;
		unsigned fpcb_id : 8;
	} b;
	uint32_t w;
} t_cfg_specific_info1;

typedef union
{
	struct {
		unsigned lot_id : 8;
		unsigned reserved : 24;
	} b;
	uint32_t w;
} t_cfg_specific_info2;

typedef struct
{
	t_cfg_specific_info1 cfg_specific_info1;/* 0x0000 */
	t_cfg_specific_info2 cfg_specific_info2;/* 0x0001 */
	uint32_t cfg_specific_version; 		/* 0x0002 */
	uint32_t cfg_model_name; 	   	/* 0x0003 */
	uint32_t cfg_header_reserved1; 		/* 0x0004 */
	uint32_t cfg_header_reserved2; 		/* 0x0005 */
	uint32_t cfg_header_reserved3; 		/* 0x0006 */
	uint32_t cfg_header_reserved4; 		/* 0x0007 */
}t_cfg_s_header_def;

typedef struct {
	u32 cfg_common_ver;
} t_cfg_c_header_def;

typedef struct
{
	uint32_t cfg_magic_code;	/* 0x0000 */
	uint32_t cfg_info_reserved0;	/* 0x0001 */
	uint32_t cfg_chip_id;		/* 0x0002 */
	uint32_t cfg_struct_version;	/* 0x0003 */
	uint32_t cfg_specific_cnt;	/* 0x0004 */
	t_cfg_size cfg_size;		/* 0x0005 */
	uint32_t cfg_global_date;	/* 0x0006 */
	uint32_t cfg_global_time;	/* 0x0007 */
	uint32_t cfg_info_reserved1;	/* 0x0008 */
	uint32_t cfg_info_reserved2;	/* 0x0009 */
	uint32_t cfg_info_reserved3;	/* 0x000A */
	uint32_t cfg_info_reserved4;	/* 0x000B */
}t_cfg_info_def;

int sw49107_reg_read(struct device *dev, u16 addr, void *data, int size);
int sw49107_reg_write(struct device *dev, u16 addr, void *data, int size);
int sw49107_ic_info(struct device *dev);
int sw49107_tc_driving(struct device *dev, int mode);
int sw49107_irq_abs(struct device *dev);
int sw49107_irq_lpwg(struct device *dev);
int sw49107_irq_handler(struct device *dev);
int sw49107_check_status(struct device *dev);
int sw49107_debug_info(struct device *dev);
int sw49107_reset_ctrl(struct device *dev, int ctrl);

static inline struct sw49107_data *to_sw49107_data(struct device *dev)
{
	return (struct sw49107_data *)touch_get_device(to_touch_core(dev));
}

static inline struct sw49107_data *to_sw49107_data_from_kobj(struct kobject *kobj)
{
	return (struct sw49107_data *)container_of(kobj,
			struct sw49107_data, kobj);
}
static inline int sw49107_read_value(struct device *dev,
		u16 addr, u32 *value)
{
	return sw49107_reg_read(dev, addr, value, sizeof(*value));
}

static inline int sw49107_write_value(struct device *dev,
		u16 addr, u32 value)
{
	return sw49107_reg_write(dev, addr, &value, sizeof(value));
}

/* extern */
#if defined(CONFIG_LGE_TOUCH_MODULE_DETECT)
extern int panel_id;
#endif

#endif /* LGE_TOUCH_SW49107_H */
