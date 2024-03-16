/*  Himax Android Driver Sample Code for debug nodes

    Copyright (C) 2018 Himax Corporation.

    This software is licensed under the terms of the GNU General Public
    License version 2, as published by the Free Software Foundation, and
    may be copied, distributed, and modified under those terms.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*/
#ifndef H_HIMAX_DEBUG
#define H_HIMAX_DEBUG

#include "touch_hx83113a.h"
#include <linux/syscalls.h> //LGE for sys file

extern 	int hx_EB_event_flag;
extern 	int hx_EC_event_flag;
extern 	int hx_ED_event_flag;

#define HIMAX_PROC_DEBUG_LEVEL_FILE	"debug_level"
#define HIMAX_PROC_VENDOR_FILE		"vendor"
#define HIMAX_PROC_ATTN_FILE		"attn"
#define HIMAX_PROC_INT_EN_FILE		"int_en"
#define HIMAX_PROC_LAYOUT_FILE		"layout"
#define HIMAX_PROC_CRC_TEST_FILE		"CRC_test"

static struct proc_dir_entry *himax_proc_debug_level_file;
static struct proc_dir_entry *himax_proc_vendor_file;
static struct proc_dir_entry *himax_proc_attn_file;
static struct proc_dir_entry *himax_proc_int_en_file;
static struct proc_dir_entry *himax_proc_layout_file;
static struct proc_dir_entry *himax_proc_CRC_test_file;

#define HIMAX_PROC_REGISTER_FILE	"register"
struct proc_dir_entry *himax_proc_register_file = NULL;
uint8_t byte_length = 0;
uint8_t register_command[4];
uint8_t cfg_flag = 0;

#define HIMAX_PROC_DIAG_FILE	"diag"
struct proc_dir_entry *himax_proc_diag_file = NULL;
#define HIMAX_PROC_DIAG_ARR_FILE	"diag_arr"
struct proc_dir_entry *himax_proc_diag_arrange_file = NULL;
struct file *diag_sram_fn;
uint8_t write_counter = 0;
uint8_t write_max_count = 30;
#define IIR_DUMP_FILE "/sdcard/HX_IIR_Dump.txt"
#define DC_DUMP_FILE "/sdcard/HX_DC_Dump.txt"
#define BANK_DUMP_FILE "/sdcard/HX_BANK_Dump.txt"
#ifdef HX_TP_PROC_2T2R
static uint32_t *diag_mutual_2;
#endif
int32_t *diag_mutual = NULL;
int32_t *diag_mutual_new = NULL;
int32_t *diag_mutual_old = NULL;
uint8_t diag_max_cnt = 0;
uint8_t hx_state_info[2] = {0};
uint8_t diag_coor[128];
int32_t diag_self[100] = {0};
int32_t diag_self_new[100] = {0};
int32_t diag_self_old[100] = {0};

#define HIMAX_PROC_DEBUG_FILE	"debug"
struct proc_dir_entry *himax_proc_debug_file = NULL;
#define HIMAX_PROC_FW_DEBUG_FILE	"FW_debug"
struct proc_dir_entry *himax_proc_fw_debug_file = NULL;
#define HIMAX_PROC_DD_DEBUG_FILE	"DD_debug"
struct proc_dir_entry *himax_proc_dd_debug_file = NULL;
bool	fw_update_complete = false;
int handshaking_result = 0;
unsigned char debug_level_cmd = 0;
uint8_t cmd_set[8];
uint8_t mutual_set_flag = 0;

#define HIMAX_PROC_FLASH_DUMP_FILE	"flash_dump"
struct proc_dir_entry *himax_proc_flash_dump_file = NULL;
static int Flash_Size = 131072;
static uint8_t *flash_buffer;
static uint8_t flash_command;
static uint8_t flash_read_step;
static uint8_t flash_progress;
static uint8_t flash_dump_complete;
static uint8_t flash_dump_fail;
static uint8_t sys_operation;
static bool    flash_dump_going;

uint32_t **raw_data_array;
uint8_t X_NUM = 0, Y_NUM = 0;
uint8_t sel_type = 0x0D;

#define HIMAX_PROC_SMWP_FILE "SMWP"
struct proc_dir_entry *himax_proc_SMWP_file = NULL;
uint8_t HX_SMWP_EN = 0;

#define HIMAX_PROC_RESET_FILE		"reset"
struct proc_dir_entry *himax_proc_reset_file 		= NULL;

#define HIMAX_PROC_SENSE_ON_OFF_FILE "SenseOnOff"
struct proc_dir_entry *himax_proc_SENSE_ON_OFF_file = NULL;

#define HIMAX_PROC_ESD_CNT_FILE "ESD_cnt"
struct proc_dir_entry *himax_proc_ESD_cnt_file = NULL;

/*-LGE PRD----------------------------*/
#define HX_RSLT_OUT_PATH "/sdcard/"
#define HX_RSLT_OUT_FILE "hx_test_result.txt"
char *g_file_path;
char *g_rslt_data;

#define CUT_CC_EN 1 /*1 = cut CC, 0 = no cut CC*/

#define BS_RAWDATANOISE	10
#define BS_OPENSHORT	0

#define	BS_LPWG	1
#define	BS_LPWG_dile	1

#define	NOISEFRAME	50
#define	UNIFMAX	500


/*Himax MP Password*/
#define	PWD_OPEN_START	0x77
#define	PWD_OPEN_END	0x88
#define	PWD_SHORT_START	0x11
#define	PWD_SHORT_END	0x33
#define	PWD_RAWDATA_START	0x00
#define	PWD_RAWDATA_END	0x99
#define	PWD_NOISE_START	0x00
#define	PWD_NOISE_END	0x99
#define	PWD_SORTING_START	0xAA
#define	PWD_SORTING_END	0xCC

#define PWD_LPWG_START	0x55
#define PWD_LPWG_END	0x66

#define PWD_LPWG_IDLE_START	0x50
#define PWD_LPWG_IDLE_END	0x60

/*Himax DataType*/
#define DATA_SORTING	0x0A
#define DATA_OPEN	0x0B
#define DATA_MICRO_OPEN	0x0C
#define DATA_SHORT	0x0A
#define DATA_RAWDATA	0x0A
#define DATA_NOISE	0x0F
#define DATA_BACK_NORMAL	0x00
#define DATA_LPWG_RAWDATA	0x0C
#define DATA_LPWG_NOISE	0x0F
#define DATA_ACT_IDLE_RAWDATA	0x0A
#define DATA_ACT_IDLE_NOISE	0x0F
#define DATA_LPWG_IDLE_RAWDATA	0x0A
#define DATA_LPWG_IDLE_NOISE	0x0F

/*Himax Data Ready Password*/
#define	Data_PWD0	0xA5
#define	Data_PWD1	0x5A

typedef enum {
	HIMAX_INSPECTION_OPEN,
	HIMAX_INSPECTION_MICRO_OPEN,
	HIMAX_INSPECTION_SHORT,
	HIMAX_INSPECTION_RAWDATA,
	HIMAX_INSPECTION_NOISE,
	HIMAX_INSPECTION_SORTING,
	HIMAX_INSPECTION_BACK_NORMAL,
	HIMAX_INSPECTION_LPWG_RAWDATA,
	HIMAX_INSPECTION_LPWG_NOISE,
	HIMAX_INSPECTION_LPWG_IDLE_RAWDATA,
	HIMAX_INSPECTION_LPWG_IDLE_NOISE,
} THP_INSPECTION_ENUM;

char *g_himax_inspection_mode[] = {
	"HIMAX_INSPECTION_OPEN",
	"HIMAX_INSPECTION_MICRO_OPEN",
	"HIMAX_INSPECTION_SHORT",
	"HIMAX_INSPECTION_RAWDATA",
	"HIMAX_INSPECTION_NOISE",
	"HIMAX_INSPECTION_SORTING",
	"HIMAX_INSPECTION_BACK_NORMAL",
	"HIMAX_INSPECTION_LPWG_RAWDATA",
	"HIMAX_INSPECTION_LPWG_NOISE",
	"HIMAX_INSPECTION_LPWG_IDLE_RAWDATA",
	"HIMAX_INSPECTION_LPWG_IDLE_NOISE",
};

/* Error code of Inspection */
typedef enum {
	HX_INSPECT_OK	= 0,               /* OK */
	HX_INSPECT_ESPI = 1 << 1,        /* SPI communication error */
	HX_INSPECT_ERAW = 2 << 1,        /* Raw data error */
	HX_INSPECT_ENOISE = 3 << 1,        /* Noise error */
	HX_INSPECT_EOPEN = 4 << 1,        /* Sensor open error */
	HX_INSPECT_EMOPEN = 5 << 1,        /* Sensor micro open error */
	HX_INSPECT_ESHORT = 6 << 1,        /* Sensor short error */
	HX_INSPECT_ELPWG_RAW = 11 << 1,		   /* LPWG RAW ERROR */
	HX_INSPECT_ELPWG_NOISE = 12 << 1,		   /* LPWG NOISE ERROR */
	HX_INSPECT_ELPWG_IDLE_RAW = 13 << 1,		   /* LPWG IDLE RAW ERROR */
	HX_INSPECT_ELPWG_IDLE_NOISE = 14 << 1,		   /* LPWG IDLE NOISE ERROR */
	HX_INSPECT_EFILE = 15 << 1,					/*Criteria file error*/
	HX_INSPECT_EOTHER = 16 << 1,	/* All other errors */
} HX_INSPECT_ERR_ENUM;

/***********LGE sysfs Start***********/
#define ROW_SIZE		16
#define COL_SIZE		36
//#define M1_COL_SIZE		2
#define LOG_BUF_SIZE		256
#define BUF_SIZE (PAGE_SIZE * 2)
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)
//#define MUTUAL_NUM (ROW_SIZE * COL_SIZE)
#define MUL_OPEN_TEST_THX_UP 1000
#define MUL_OPEN_TEST_THX_LOW 0
#define MUL_MICRO_OPEN_THX_UP 1000
#define MUL_MICRO_OPEN_THX_LOW 0
#define MUL_SHORT_THX_UP 1000
#define MUL_SHORT_THX_LOW 0
#define MUL_RAW_DATA_THX_UP 20000
#define MUL_RAW_DATA_THX_LOW 1000
#define MUL_LPWG_RAW_DATA_THX_UP 20000
#define MUL_LPWG_RAW_DATA_THX_LOW 1000
//#define MUL_OS_THX_UP 30000
//#define MUL_OS_THX_LOW 10000
#define JITTER_THX 50

enum {
	OPEN_TEST = 0,
	MICRO_OPEN_TEST,
	SHORT_TEST,
	RAW_DATA_TEST,
	JITTER_TEST,
	LPWG_RAW_DATA_TEST,
	LPWG_JITTER_TEST,
};

enum{
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

uint8_t g_os_fail_num[4]; /* mut_o,mut_s,self_o,self_s */
/***********LGE sysfs End***********/
#endif
