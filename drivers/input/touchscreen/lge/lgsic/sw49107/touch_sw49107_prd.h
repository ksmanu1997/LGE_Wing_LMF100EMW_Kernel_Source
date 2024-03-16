/* production_test.h
 *
 * Copyright (C) 2015 LGE.
 *
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

/* Include to touch core Header File */
#include <touch_core.h>

/* Include to Local Header File */
#include "touch_sw49107.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

/* production test */
//#define tc_test_mode_ctl		(0xC6E)
//#define cmd_test_exit			(0x0000)
//#define cmd_test_enter		(0x0001)

#define TC_PT_TEST_CTL			(0xC04)
#define PT_TEST_STS			(0x26C)
#define PT_TEST_PF_RESULT		(0x26D)
#define SERIAL_DATA_OFFSET		(0x026)
#define SERIAL_DATA_ACCESS		(0xFD1)
#define tune_code_addr			(0x600)

//#define RAWDATA_OFFSET		(0xE00)
//#define rawdata_ctl_read		(0x2A4)
//#define rawdata_ctl_write		(0xC49)

//#define Serial_Data_Offset		(0x026)	//(0x0AF) 49106 case
//#define DATA_BASE_ADDR		(0xFD1)
//#define m1_raw_data_offset		(0x02D7)// [bringup] (0x0287)
//#define m1_m2_raw_data_offset		(0x0286)// [bringup] (0x0287)
//#define prod_open1_open2_offset	(0x02D7)// [bringup] (0x0287)
//#define prod_open3_short_offset	(0x0288)// [bringup] (0x0288)
//#define M1_M2_RAWDATA_TEST_CNT_MAX	(2)
//#define M1_M2_RAWDATA_TEST_CNT	(1)
#define LINE_FILTER_OPTION		(0x40000)

enum {
	PT_FRAME_1 = 0,
	PT_FRAME_2,
	PT_FRAME_3,
	PT_FRAME_4,
};

#define PT_FRAME_1_OFFSET		(0x1240)
#define PT_FRAME_2_OFFSET		(0x1384)
#define PT_FRAME_3_OFFSET		(0x14C8)
#define PT_FRAME_4_OFFSET		(0x160C)


/* Firmware debugging */
#define AIT_RAW_DATA_OFFSET			(0x12B6)	//(0xF88) 49106 case
#define DELTA_DATA_OFFSET			(0x153E)	//(0x1110) 49106 case
#define LABLE_DATA_OFFSET			(0x16BA)	//(0x1200) 49106 case
#define AIT_BASE_DATA_OFFSET			(0x13FA)	//(0x104C) 49106 case
#define AIT_DEBUG_BUF_DATA_OFFSET		(0x102E)
#define FILTERED_DELTA_DATA_OFFSET		(0x7FD)
#define ADDR_CMD_REG_SIC_IMAGECTRL_TYPE		(0x0F0C)	//(0x0C6C) 49106 case
#define ADDR_CMD_REG_SIC_GETTER_READYSTATUS	(0x0F04)	//(0x0C64)	49106 case

#define TIME_STR_LEN		(64)


/* Chanel settings */
#define MAX_CHANNEL		(36)
#define ROW_SIZE		(36)	//28
#define COL_SIZE		(18)	//14
#define M1_COL_SIZE		(2)
#define SHORT_COL_SIZE		(4)

//#define PATH_SIZE		64
//#define BURST_SIZE		512
//#define RAWDATA_SIZE		2
//#define SHORT_ROW_SIZE		MAX_CHANNEL
//#define SHORT_COL_SIZE		4
#define LOG_BUF_SIZE		(512)
#define BUF_SIZE (PAGE_SIZE * 2)
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)
#define DEBUG_ROW_SIZE		ROW_SIZE
#define DEBUG_COL_SIZE		COL_SIZE



/* tune code */
#define tc_tune_code_size		260
typedef union {
	struct {
		unsigned r_goft_tune_m1:	4;
		unsigned r_goft_tune_m1_sign:	1;
		unsigned r_goft_tune_m2:	4;
		unsigned r_goft_tune_m2_sign:	1;
		unsigned r_goft_tune_nd:	5;
		unsigned reserved:		17;
	} b;
	u32 w;
} t_goft_tune;


#define LOFT_CH_NUM	(MAX_CHANNEL/2)

struct tune_data_format {
	u32		r_tune_code_magic;

	t_goft_tune	r_goft_tune_u3_m1m2_left;
	u16		r_loft_tune_u3_m1_left[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_g1_left[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_g2_left[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_g3_left[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_nd_left[LOFT_CH_NUM];

	t_goft_tune	r_goft_tune_u3_m1m2_right;
	u16		r_loft_tune_u3_m1_right[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_g1_right[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_g2_right[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_g3_right[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_nd_right[LOFT_CH_NUM];

	t_goft_tune	r_goft_tune_u0_m1m2_left;
	u16		r_loft_tune_u0_m1_left[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_g1_left[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_g2_left[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_g3_left[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_nd_left[LOFT_CH_NUM];

	t_goft_tune	r_goft_tune_u0_m1m2_right;
	u16		r_loft_tune_u0_m1_right[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_g1_right[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_g2_right[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_g3_right[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_nd_right[LOFT_CH_NUM];
};

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

//AIT IT_IMAGE CMD
enum {
	CMD_NONE = 0,
	CMD_RAWDATA,
	CMD_BASE_DATA,
	CMD_DELTADATA,
	CMD_LABELDATA,
	CMD_FILTERED_DELTA,    // Not used
	CMD_RESERVED,			// Not used
	CMD_DEBUGDATA,
	DONT_USE_CMD = 0xEE,
	CMD_WAIT = 0xFF,
};

enum {
	U3_PT_TEST = 0,
	OPEN_NODE_TEST,
	SHORT_NODE_TEST,
	U3_M1_RAWDATA_TEST,
	U3_M1_JITTER_TEST,
	U3_M2_RAWDATA_TEST,
	U3_M2_JITTER_TEST,
	U0_M1_RAWDATA_TEST,
	U0_M1_JITTER_TEST,
	U0_M2_RAWDATA_TEST,
	U0_M2_JITTER_TEST,
	U3_BLU_JITTER_TEST,
	AVERAGE_JITTER_TEST,
};

enum {
	NORMAL_MODE = 0,
	PRODUCTION_MODE,
};

typedef enum
{
	IT_NONE = 0,
	IT_ALGORITHM_RAW_IMAGE,
	IT_BASELINE_E_IMAGE,
	IT_BASELINE_O_IMAGE,
	IT_DELTA_IMAGE,
	IT_LABEL_IMAGE,
	IT_FILTERED_DELTA_IMAGE,
	IT_WAIT = 0xFF
} eImageType_t;

typedef enum
{
	RS_READY    = 0xA0,
	RS_NONE     = 0x05,
	RS_LOG      = 0x77,
	RS_IMAGE	= 0xAA
} eProtocolReadyStatus_t;

/*
 * Jitter Test Mode
 *
 * Test Type	| PRD_JITTER		| PRD_JITTER_DIFFER			| PRD_JITTER_DELTA
 * Rawdata	| Untuned		| Tuned					| Tuned
 * Method	| Rawdata Max - Min	| MAX(Before Rawdata - Current Rawdata) | MAX(ABS(Rawdata - baseline))
 */

enum {
	PRD_JITTER = 0,
	PRD_JITTER_DIFFER,
	PRD_JITTER_DELTA,
};

struct prd_test_param {
	u16 sd_test_set;
	u16 lpwg_sd_test_set;
	char *spec_file_path;
	char *mfts_spec_file_path;
	u8 jitter_test_type;
};

/* [Start] for siw app */
enum {
	REPORT_END_RS_NG = 0x05,
	REPORT_END_RS_OK = 0xAA,
};

enum {
	PRD_CMD_TYPE_1 = 0,		//new type: base only
	PRD_CMD_TYPE_2 = 1,		//old type: base_even, base_odd
};


enum {
	APP_REPORT_OFF = 0,
	APP_REPORT_RAW,
	APP_REPORT_BASE,
	APP_REPORT_DELTA,
	APP_REPORT_LABEL,
	APP_REPORT_DEBUG_BUF,
	APP_REPORT_MAX,
};

enum {
	PRD_DATA_NAME_SZ	= 128,
	/* */
	//PRD_RAWDATA_SZ_POW	= 1,
	//PRD_RAWDATA_SIZE	= (1<<PRD_RAWDATA_SZ_POW),
	/* */
	PRD_ROW_SIZE		= ROW_SIZE,
	PRD_COL_SIZE		= COL_SIZE,
	PRD_COL_ADD			= 1,
	PRD_CH				= MAX_CHANNEL,
	PRD_M1_COL			= M1_COL_SIZE,
	PRD_CMD_TYPE		= PRD_CMD_TYPE_1,
	SECOND_SCR_BOUND_I	= 0,
	SECOND_SCR_BOUND_J	= 0,
	//PRD_M1_COL_SIZE		= (1<<1),
	/* */
	//PRD_LOG_BUF_SIZE	= (1<<10),		//1K
	//PRD_BUF_SIZE		= (PAGE_SIZE<<1),
	/* */
	PRD_SHOW_FLAG_DISABLE_PRT_RAW	= (1<<0),
	PRD_APP_INFO_SIZE	= 32,
};

enum {
	PRD_M2_ROW_COL_SIZE		= (PRD_ROW_SIZE * PRD_COL_SIZE),
	PRD_M2_ROW_COL_BUF_SIZE	= (PRD_ROW_SIZE * (PRD_COL_SIZE + PRD_COL_ADD)),
	//PRD_M1_ROW_COL_SIZE		= (PRD_ROW_SIZE * PRD_M1_COL_SIZE),
	/* */
	//PRD_M2_FRAME_SIZE		= (PRD_M2_ROW_COL_BUF_SIZE<<PRD_RAWDATA_SZ_POW),
	//PRD_M1_FRAME_SIZE		= (PRD_M1_ROW_COL_SIZE<<PRD_RAWDATA_SZ_POW),
	/* */
	PRD_DELTA_SIZE			= ((PRD_ROW_SIZE+2)*(PRD_COL_SIZE+2)),
	/* */
	PRD_LABEL_TMP_SIZE		= ((PRD_ROW_SIZE+2)*(PRD_COL_SIZE+2)),
	PRD_DEBUG_BUF_SIZE		= (PRD_M2_ROW_COL_SIZE),
};

struct siw_hal_prd_data {
	struct device *dev;
	char name[PRD_DATA_NAME_SZ];
	/* */
	int prd_app_mode;
	/* */
	int16_t	m2_buf_rawdata[PRD_M2_ROW_COL_BUF_SIZE];
	//int16_t	m2_buf_even_rawdata[PRD_M2_ROW_COL_BUF_SIZE];
	/* */
	//int16_t	m1_buf_odd_rawdata[PRD_M1_ROW_COL_SIZE];
	//int16_t	m1_buf_even_rawdata[PRD_M1_ROW_COL_SIZE];
	//int16_t m1_buf_tmp[PRD_M1_ROW_COL_SIZE];
	/* */
	//int image_lower;
	//int image_upper;
	/* */
	int16_t	buf_delta[PRD_DELTA_SIZE];
	int16_t	buf_debug[PRD_DEBUG_BUF_SIZE];
	u8	buf_label_tmp[PRD_LABEL_TMP_SIZE];
	u8	buf_label[PRD_M2_ROW_COL_SIZE];
	/* */
	struct prd_test_param prd_param;
};

enum {
	PRD_SYS_EN_IDX_SD = 0,
	PRD_SYS_EN_IDX_DELTA,
	PRD_SYS_EN_IDX_LABEL,
	PRD_SYS_EN_IDX_RAWDATA_PRD,
	PRD_SYS_EN_IDX_RAWDATA_TCM,
	PRD_SYS_EN_IDX_RAWDATA_AIT,
	PRD_SYS_EN_IDX_BASE,
	PRD_SYS_EN_IDX_DEBUG_BUF,
	PRD_SYS_EN_IDX_LPWG_SD,
	PRD_SYS_EN_IDX_FILE_TEST,
	PRD_SYS_EN_IDX_APP_RAW,
	PRD_SYS_EN_IDX_APP_BASE,
	PRD_SYS_EN_IDX_APP_LABEL,
	PRD_SYS_EN_IDX_APP_DELTA,
	PRD_SYS_EN_IDX_APP_DEBUG_BUF,
	PRD_SYS_EN_IDX_APP_END,
	PRD_SYS_EN_IDX_APP_INFO,
	PRD_SYS_ATTR_MAX,
};

enum {
	PRD_SYS_EN_SD			= (1<<PRD_SYS_EN_IDX_SD),
	PRD_SYS_EN_DELTA		= (1<<PRD_SYS_EN_IDX_DELTA),
	PRD_SYS_EN_LABEL		= (1<<PRD_SYS_EN_IDX_LABEL),
	PRD_SYS_EN_RAWDATA_PRD		= (1<<PRD_SYS_EN_IDX_RAWDATA_PRD),
	PRD_SYS_EN_RAWDATA_TCM		= (1<<PRD_SYS_EN_IDX_RAWDATA_TCM),
	PRD_SYS_EN_RAWDATA_AIT		= (1<<PRD_SYS_EN_IDX_RAWDATA_AIT),
	PRD_SYS_EN_BASE			= (1<<PRD_SYS_EN_IDX_BASE),
	PRD_SYS_EN_DEBUG_BUF		= (0<<PRD_SYS_EN_IDX_DEBUG_BUF),
	PRD_SYS_EN_LPWG_SD		= (1<<PRD_SYS_EN_IDX_LPWG_SD),
	PRD_SYS_EN_FILE_TEST		= (1<<PRD_SYS_EN_IDX_FILE_TEST),
	PRD_SYS_EN_APP_RAW		= (1<<PRD_SYS_EN_IDX_APP_RAW),
	PRD_SYS_EN_APP_BASE		= (1<<PRD_SYS_EN_IDX_APP_BASE),
	PRD_SYS_EN_APP_LABEL		= (1<<PRD_SYS_EN_IDX_APP_LABEL),
	PRD_SYS_EN_APP_DELTA		= (1<<PRD_SYS_EN_IDX_APP_DELTA),
	PRD_SYS_EN_APP_DEBUG_BUF	= (1<<PRD_SYS_EN_IDX_APP_DEBUG_BUF),
	PRD_SYS_EN_APP_END		= (1<<PRD_SYS_EN_IDX_APP_END),
	PRD_SYS_EN_APP_INFO		= (1<<PRD_SYS_EN_IDX_APP_INFO),
};

#define PRD_SYS_ATTR_EN_FLAG	(0		|\
		PRD_SYS_EN_SD			|\
		PRD_SYS_EN_DELTA		|\
		PRD_SYS_EN_LABEL		|\
		PRD_SYS_EN_RAWDATA_PRD		|\
		PRD_SYS_EN_RAWDATA_TCM		|\
		PRD_SYS_EN_RAWDATA_AIT		|\
		PRD_SYS_EN_BASE			|\
		PRD_SYS_EN_DEBUG_BUF		|\
		PRD_SYS_EN_LPWG_SD		|\
		PRD_SYS_EN_FILE_TEST		|\
		PRD_SYS_EN_APP_RAW		|\
		PRD_SYS_EN_APP_BASE		|\
		PRD_SYS_EN_APP_LABEL		|\
		PRD_SYS_EN_APP_DELTA		|\
		PRD_SYS_EN_APP_DEBUG_BUF	|\
		PRD_SYS_EN_APP_END		|\
		PRD_SYS_EN_APP_INFO		|\
		0)
/* [End] for siw app */


extern void touch_msleep(unsigned int msecs);
int sw49107_prd_register_sysfs(struct device *dev);

/* For BLU test. We need to control backlight level. */
#if defined(CONFIG_TOUCHSCREEN_MTK)
extern unsigned int mt_get_bl_brightness(void);
extern int mt65xx_leds_brightness_set(int, int);
#elif defined(CONFIG_LGE_TOUCH_CORE_QCT)
extern int mdss_fb_get_bl_brightness_extern(void);
extern void mdss_fb_set_bl_brightness_extern(int);
#endif

#endif


