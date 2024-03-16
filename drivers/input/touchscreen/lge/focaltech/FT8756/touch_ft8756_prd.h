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

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_ft8756.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

/* Normal Mode SET SPEC */
#define RAW_DATA_MARGIN		(4000)
#define RAW_DATA_MAX		((6000) + RAW_DATA_MARGIN)
#define RAW_DATA_MIN		((6000) - RAW_DATA_MARGIN)

#define CB_MAX			(50)
#define CB_MIN			(1)
#define NOISE_MAX		(250)
#define NOISE_MIN		(0)
#define JITTER_MAX		(250)
#define JITTER_MIN		(0)
#define SHORT_RES_MIN 	(200)
#define SHORT_RES_MAX 	(0xFFFF)
#define OPEN_CB_MIN 	(40)
#define OPEN_CB_MAX 	(255)

/* Production Test Mode */
#define FACTORY_REG_K1 			0x31
#define FACTORY_REG_K2 			0x32
#define FACTORY_REG_CLB 		0x04
#define OPENTEST_K1THRESHOLD 	(30)
#define OPENTEST_K2THRESHOLD 	(1)
#define BYTES_PER_TIME 			(32)  /* max:128 */

/* LPWG Mode SET SPEC */
#define LPWG_RAW_DATA_MAX	((6000) + RAW_DATA_MARGIN)
#define LPWG_RAW_DATA_MIN	((6000) - RAW_DATA_MARGIN)
#define LPWG_CB_MAX 		(50)
#define LPWG_CB_MIN 		(1)
#define LPWG_NOISE_MAX 		(250)
#define LPWG_NOISE_MIN 		(0)
#define LPWG_JITTER_MAX		(250)
#define LPWG_JITTER_MIN 	(0)

#define FTS_WORK_MODE		0x00
#define FTS_FACTORY_MODE	0x40

#define FTS_MODE_CHANGE_LOOP	20

/* Number of channel */
#define MAX_ROW			36
#define MAX_COL			18

#define LOG_BUF_SIZE		(4096 * 4)
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)
#define PRINT_BUF_SIZE		(256)
#define FAIL_LOG_BUF_SIZE 	(500)

#define DELTA_ATTR_SIZE		(8 * 1024) - 500
#define RAWDATA_ATTR_SIZE	(10 * 1024) - 1100
#define LGE_ATTR_DELTA		"delta_ext"
#define LGE_ATTR_RAWDATA	"rawdata_ext"

#define TEST_PACKET_LENGTH	(MAX_ROW * MAX_COL)

enum {
	RAW_DATA_TEST = 0,
	CB_DATA_TEST,
	OPEN_CB_TEST,
	SHORT_ADC_TEST,
	NOISE_TEST,
	JITTER_TEST,
	DELTA_SHOW,
	LPWG_RAW_DATA_TEST,
	LPWG_CB_DATA_TEST,
	LPWG_NOISE_TEST,
	LPWG_JITTER_TEST,
	THE_NUMBER_OF_TEST,
};

enum {
	TEST_FAIL = 0,
	TEST_PASS,
};

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};
extern int ft8756_dummy_read(struct device *dev);
extern void touch_msleep(unsigned int msecs);
int ft8756_prd_register_sysfs(struct device *dev);
#endif
