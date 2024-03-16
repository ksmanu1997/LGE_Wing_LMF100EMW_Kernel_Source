/* production_test.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>

#include <touch_hwif.h>
#include <touch_core.h>

#include "touch_sw49107.h"
#include "touch_sw49107_prd.h"

static char line[100000];
static char w_buf[BUF_SIZE];

static u16 data_buf[MAX_CHANNEL*COL_SIZE];
static u16 LowerImage[ROW_SIZE][COL_SIZE];
static u16 UpperImage[ROW_SIZE][COL_SIZE];
static u16 AverageImage[ROW_SIZE][COL_SIZE];


#define TEST_TOTAL_NUM 15
static const char const *test_name_str[TEST_TOTAL_NUM] = {
	"U3_PT_TEST",
	"OPEN_NODE_TEST",
	"SHORT_NODE_TEST",
	"U3_M1_RAWDATA_TEST",
	"U3_M1_JITTER_TEST",
	"U3_M2_RAWDATA_TEST",
	"U3_M2_JITTER_TEST",
	"U0_M1_RAWDATA_TEST",
	"U0_M1_JITTER_TEST",
	"U0_M2_RAWDATA_TEST",
	"U0_M2_JITTER_TEST",
	"U3_BLU_JITTER_TEST",
	"AVERAGE_JITTER_TEST",
};

static void prd_param_set(struct device *dev)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;

	param->sd_test_set = (1<<U3_M2_RAWDATA_TEST)|(1<<U3_M2_JITTER_TEST);
	param->lpwg_sd_test_set = (1<<U0_M1_RAWDATA_TEST)|(1<<U0_M2_RAWDATA_TEST)|(1<<U0_M1_JITTER_TEST)|(1<<U0_M2_JITTER_TEST);

	param->spec_file_path = "/sdcard/cv7as_limit.txt";
	param->mfts_spec_file_path = "/sdcard/cv7as_limit_mfts.txt";

	param->jitter_test_type = PRD_JITTER_DELTA;
}

static void log_file_size_check(struct device *dev)
{
	char *fname = NULL;
	struct file *file;
	loff_t file_size = 0;
	int i = 0;
	char buf1[128] = {0};
	char buf2[128] = {0};
	mm_segment_t old_fs = get_fs();
	int ret = 0;
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
		case TOUCH_NORMAL_BOOT:
			fname = "/sdcard/touch_self_test.txt";
			break;
		case TOUCH_MINIOS_AAT:
			fname = "/data/touch/touch_self_test.txt";
			break;
		case TOUCH_MINIOS_MFTS_FOLDER:
		case TOUCH_MINIOS_MFTS_FLAT:
		case TOUCH_MINIOS_MFTS_CURVED:
			fname = "/data/touch/touch_self_mfts.txt";
			break;
		default:
			TOUCH_I("%s : not support mode\n", __func__);
			break;
	}

	if (fname) {
		file = filp_open(fname, O_RDONLY, 0666);
		ksys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		goto error;
	}

	if (IS_ERR(file)) {
		TOUCH_I("%s : ERR(%ld) Open file error [%s]\n", __func__, PTR_ERR(file), fname);
		goto error;
	}

	file_size = vfs_llseek(file, 0, SEEK_END);
	TOUCH_I("%s : [%s] file_size = %lld\n", __func__, fname, file_size);

	filp_close(file, 0);

	if (file_size > MAX_LOG_FILE_SIZE) {
		TOUCH_I("%s : [%s] file_size(%lld) > MAX_LOG_FILE_SIZE(%d)\n", __func__, fname, file_size, MAX_LOG_FILE_SIZE);

		for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
			if (i == 0)
				sprintf(buf1, "%s", fname);
			else
				sprintf(buf1, "%s.%d", fname, i);

			ret = ksys_access(buf1, 0);

			if (ret == 0) {
				TOUCH_I("%s : file [%s] exist\n", __func__, buf1);

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					if (ksys_unlink(buf1) < 0) {
						TOUCH_E("%s : failed to remove file [%s]\n", __func__, buf1);
						goto error;
					}

					TOUCH_I("%s : remove file [%s]\n", __func__, buf1);
				} else {
					snprintf(buf2, sizeof(buf2), "%s.%d", fname, (i + 1));
					if (ksys_link(buf1, buf2) < 0) {
						TOUCH_E("%s : failed to link file [%s] -> [%s]\n", __func__, buf1, buf2);
						goto error;
					}
					if (ksys_unlink(buf1) < 0) {
						TOUCH_E("%s : failed to remove file [%s]\n", __func__, buf1);
						goto error;
					}
					TOUCH_I("%s : rename file [%s] -> [%s]\n", __func__, buf1, buf2);
				}
			} else {
				TOUCH_I("%s : file [%s] does not exist (ret = %d)\n", __func__, buf1, ret);
			}
		}
	}

error:
	set_fs(old_fs);
	return;
}

static int sdcard_spec_file_read(struct device *dev)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int ret = 0;
	int fd = 0;
	char *path[2] = {param->spec_file_path, param->mfts_spec_file_path};
	int path_idx = 0;

	mm_segment_t old_fs = get_fs();

	TOUCH_TRACE();

	if(touch_check_boot_mode(dev) >= TOUCH_MINIOS_MFTS_FOLDER)
		path_idx = 1;
	else
		path_idx = 0;
	set_fs(KERNEL_DS);
	fd = ksys_open(path[path_idx], O_RDONLY, 0);
	if (fd >= 0) {
		ksys_read(fd, line, sizeof(line));
		ksys_close(fd);
		TOUCH_I("%s file existing\n", path[path_idx]);
		ret = 1;
	}
	set_fs(old_fs);

	return ret;
}

static int spec_file_read(struct device *dev)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fwlimit = NULL;
#if defined (CONFIG_LGE_TOUCH_MODULE_DETECT)
	const char *path[2] = {0, };
	int i = 0;
#else
	const char *path[2] = {ts->panel_spec, ts->panel_spec_mfts};
#endif
	int path_idx = 0;

	TOUCH_TRACE();

#if defined (CONFIG_LGE_TOUCH_MODULE_DETECT)
	if ((ts->def_fwcnt) > 0x01) { /* Two or more panel spec */
		if (panel_id == LGD_INCELL_SW49107) {
			TOUCH_I("[%s]_LGD_%s\n", __func__, ts->dual_panel_spec[0]);
			path[0] = ts->dual_panel_spec[0];
			path[1] = ts->dual_panel_spec_mfts[0];
		} else if (panel_id == TOVIS_INCELL_SW49107) {
			TOUCH_I("[%s]_TOVIS_%s\n", __func__, ts->dual_panel_spec[1]);
			path[0] = ts->dual_panel_spec[1];
			path[1] = ts->dual_panel_spec_mfts[1];
		}
	} else if ((ts->def_fwcnt) == 0x01) { /* Use same limit file */
		TOUCH_I("Use same limit file_%s\n", ts->dual_panel_spec[0]);
		path[0] = ts->dual_panel_spec[0];
		path[1] = ts->dual_panel_spec_mfts[0];
	} else {
		TOUCH_E("no spec file\n");
		goto error;
	}
#endif

	if(touch_check_boot_mode(dev) >= TOUCH_MINIOS_MFTS_FOLDER)
		path_idx = 1;
	else
		path_idx = 0;

#if defined (CONFIG_LGE_TOUCH_MODULE_DETECT)
	for (i=0;i < ts->def_fwcnt;i++) {
		if (ts->dual_panel_spec[i] == NULL || ts->dual_panel_spec_mfts[i] == NULL) {
			TOUCH_E("dual_panel_spec[%d] file name is null\n", i);
			ret = -1;
			goto error;
		}
	}
#else
	if (ts->panel_spec == NULL || ts->panel_spec_mfts == NULL) {
		TOUCH_E("panel_spec_file name is null\n");
		ret = -1;
		goto error;
	}
#endif

	if (request_firmware(&fwlimit, path[path_idx], dev) < 0) {
		TOUCH_E("request ihex is failed in normal mode\n");
		TOUCH_E("path_idx:%d, path:%s \n", path_idx, path[path_idx]);
		ret = -1;
		goto error;
	}

	if (fwlimit->data == NULL) {
		ret = -1;
		TOUCH_E("fwlimit->data is NULL\n");
		goto error;
	}

	strlcpy(line, fwlimit->data, sizeof(line));

error:
	if (fwlimit)
		release_firmware(fwlimit);

	return ret;
}

static int sic_get_limit(struct device *dev, char *breakpoint, u16 (*buf)[COL_SIZE])
{
	int p = 0;
	int q = 0;
	int r = 0;
	int cipher = 1;
	int ret = 0;
	char *found;
	int boot_mode = 0;
	int file_exist = 0;
	int tx_num = 0;
	int rx_num = 0;

	TOUCH_TRACE();

	if (breakpoint == NULL) {
		ret = -1;
		goto error;
	}

	boot_mode = touch_check_boot_mode(dev);
	if (boot_mode > TOUCH_MINIOS_MFTS_CURVED || boot_mode < TOUCH_NORMAL_BOOT) {
		ret = -1;
		goto error;
	}

	file_exist = sdcard_spec_file_read(dev);
	if (!file_exist) {
		ret = spec_file_read(dev);
		if (ret == -1)
			goto error;
	}

	if (line == NULL) {
		ret =  -1;
		goto error;
	}

	found = strnstr(line, breakpoint, sizeof(line));
	if (found != NULL) {
		q = found - line;
	} else {
		TOUCH_E("failed to find breakpoint. The panel_spec_file is wrong\n");
		ret = -1;
		goto error;
	}

	memset(buf, 0, ROW_SIZE * COL_SIZE * 2);

	while (1) {
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') && (line[q - p] <= '9'); p++) {
				buf[tx_num][rx_num] += ((line[q - p] - '0') * cipher);
				cipher *= 10;
			}
			r++;
			if (r % (int)COL_SIZE == 0) {
				rx_num = 0;
				tx_num++;
			} else {
				rx_num++;
			}
		}
		q++;
		if (r == (int)ROW_SIZE * (int)COL_SIZE) {
			break;
		}
	}

	if (ret == 0) {
		ret = -1;
		goto error;

	} else {
		TOUCH_I("panel_spec_file scanning is success\n");
		return ret;
	}

error:
	return ret;
}
static void write_file(struct device *dev, char *data, int write_time)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[TIME_STR_LEN] = {0};
	struct timespec my_time;
	struct tm my_date;
	int boot_mode = 0;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
		case TOUCH_NORMAL_BOOT:
			fname = "/sdcard/touch_self_test.txt";
			break;
		case TOUCH_MINIOS_AAT:
			fname = "/data/touch/touch_self_test.txt";
			break;
		case TOUCH_MINIOS_MFTS_FOLDER:
		case TOUCH_MINIOS_MFTS_FLAT:
		case TOUCH_MINIOS_MFTS_CURVED:
			fname = "/data/touch/touch_self_mfts.txt";
			break;
		default:
			TOUCH_I("%s : not support mode\n", __func__);
			break;
	}

	if (fname) {
		fd = ksys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
		ksys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		set_fs(old_fs);
		return;
	}

	if (fd >= 0) {
		if (write_time == TIME_INFO_WRITE) {
			my_time = current_kernel_time();
			time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
			snprintf(time_string, TIME_STR_LEN,
				"\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
				my_date.tm_mon + 1,
				my_date.tm_mday, my_date.tm_hour,
				my_date.tm_min, my_date.tm_sec,
				(unsigned long) my_time.tv_nsec / 1000000);
			ksys_write(fd, time_string, strlen(time_string));
		}
		ksys_write(fd, data, strlen(data));
		ksys_close(fd);
	} else {
		TOUCH_I("File open failed\n");
	}
	set_fs(old_fs);
}

static int write_test_mode(struct device *dev, u32 type)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;

	u32 testmode = 0;
	u8 disp_mode = 0x3;
	int retry = 40;
	u32 rdata = 0x01;
	int waiting_time = 200;
	u8 type_temp = 0;
	//u32 backlightness = 0;

	switch (type) {
	case OPEN_NODE_TEST:
		type_temp = 0x1;
		testmode = (disp_mode << 8) + type_temp;
		waiting_time = 10;
		break;
	case SHORT_NODE_TEST:
		type_temp = 0x2;
		testmode = (disp_mode << 8) + type_temp;
		waiting_time = 1000;
		break;
	case U3_M1_RAWDATA_TEST:
		type_temp = 0x3;
		testmode = (disp_mode << 8) + type_temp;
		break;
	case U3_M1_JITTER_TEST:
		type_temp = 0x4;
		testmode = (disp_mode << 8) + type_temp;
		waiting_time = 800;
		break;
	case U3_M2_RAWDATA_TEST:
		type_temp = 0x5;
		testmode = (disp_mode << 8) + type_temp;
		break;
	case U3_M2_JITTER_TEST:
		if (param->jitter_test_type == PRD_JITTER || param->jitter_test_type == PRD_JITTER_DIFFER) {
			type_temp = 0x6;
		} else if (param->jitter_test_type == PRD_JITTER_DELTA) {
			type_temp = 0xD;
		} else {
			TOUCH_I("ProductionTest Jitter test Type error [%d]\n", param->jitter_test_type);
			goto error;
		}
		testmode = ((disp_mode << 8) + type_temp);
		waiting_time = 800;
		break;
	case U0_M1_RAWDATA_TEST:
		type_temp = 0x3;
		testmode = type_temp;
		break;
	case U0_M1_JITTER_TEST:
		type_temp = 0x4;
		testmode = type_temp;
		waiting_time = 800;
		break;
	case U0_M2_RAWDATA_TEST:
		type_temp = 0x5;
		testmode = type_temp;
		break;
	case U0_M2_JITTER_TEST:
		if (param->jitter_test_type == PRD_JITTER || param->jitter_test_type == PRD_JITTER_DIFFER) {
			type_temp = 0x6;
		} else if (param->jitter_test_type == PRD_JITTER_DELTA) {
			type_temp = 0xD;
		} else {
			TOUCH_I("ProductionTest Jitter test Type error [%d]\n", param->jitter_test_type);
			goto error;
		}
		testmode = type_temp;
		waiting_time = 800;
		break;
	case U3_BLU_JITTER_TEST:
		type_temp = 0xD;
		waiting_time = 800;
		retry = 60;
		testmode = ((disp_mode << 8) + type_temp);
		break;
	}

	/* TestType Set */
	sw49107_reg_write(dev, TC_PT_TEST_CTL,
			(u32 *)&testmode,
			sizeof(testmode));
	TOUCH_I("write testmode = %xh\n", testmode);
	touch_msleep(waiting_time);

	if (type == U3_BLU_JITTER_TEST) {
#if 0
#if defined(CONFIG_TOUCHSCREEN_MTK)
		backlightness = mt_get_bl_brightness();
#elif defined(CONFIG_LGE_TOUCH_CORE_QCT)
		backlightness = mdss_fb_get_bl_brightness_extern();
#endif
		touch_msleep(742);
#if defined(CONFIG_TOUCHSCREEN_MTK)
		mt65xx_leds_brightness_set(6, 0);
		TOUCH_I("BackLight Off at BLU JITTER (CONFIG_TOUCHSCREEN_MTK)\n");
#elif defined(CONFIG_LGE_TOUCH_CORE_QCT)
		mdss_fb_set_bl_brightness_extern(0);
		TOUCH_I("BackLight Off at BLU JITTER TEST (CONFIG_LGE_TOUCH_CORE_QCT)\n");
#endif
		touch_msleep(278);
#if defined(CONFIG_TOUCHSCREEN_MTK)
		mt65xx_leds_brightness_set(6, backlightness);
		TOUCH_I("BackLight On at BLU JITTER (CONFIG_TOUCHSCREEN_MTK)\n");
#elif defined(CONFIG_LGE_TOUCH_CORE_QCT)
		mdss_fb_set_bl_brightness_extern(backlightness);
		TOUCH_I("BackLight On at BLU JITTER (CONFIG_LGE_TOUCH_CORE_QCT)\n");
#endif
#endif
	}

	/* Check Test Result - wait until 0 is written */
	do {
		touch_msleep(50);
		sw49107_reg_read(dev, PT_TEST_STS, (u8 *)&rdata, sizeof(rdata));
		TOUCH_I("rdata = 0x%x\n", rdata);
	} while ((rdata != 0xAA) && retry--);

	if (rdata != 0xAA) {
		TOUCH_I("ProductionTest Type [%d] Time out\n", type);
		goto error;
	}
	return 1;
error:
	TOUCH_E("[%s] fail\n", __func__);
	return 0;
}

static int prd_read_print_frame(struct device *dev, int offset, int row, int col)
{
	int i = 0, j = 0;
	int ret = 0;
	int log_ret = 0;
	char log_buf[LOG_BUF_SIZE] = {0, };

	//read data
	TOUCH_I("read_offset = %xh\n", offset);

	memset(&data_buf, 0x0, sizeof(data_buf));

	sw49107_write_value(dev, SERIAL_DATA_OFFSET, offset);

	sw49107_reg_read(dev, SERIAL_DATA_ACCESS, (u16 *)data_buf, row*col*sizeof(u16));

	/* print a frame data */
	ret = snprintf(w_buf, PAGE_SIZE, "\n  : ");
	log_ret = snprintf(log_buf, sizeof(log_buf) - log_ret, "  : ");


	//print data
	for (i = 0; i < col; i++) {
		ret += snprintf(w_buf + ret, PAGE_SIZE - ret, " [%2d]", i);
		log_ret += snprintf(log_buf + log_ret, sizeof(log_buf) - log_ret, " [%2d]", i);
	}
	TOUCH_I("%s\n", log_buf);

	for (i = 0; i < row; i++) {
		log_ret = 0;
		memset(log_buf, 0, sizeof(log_buf));
		ret += snprintf(w_buf + ret, PAGE_SIZE - ret,  "\n[%2d]", i);
		log_ret += snprintf(log_buf + log_ret, sizeof(log_buf) - log_ret,  "[%2d]", i);

		for (j = 0; j < col; j++) {
			ret += snprintf(w_buf + ret, PAGE_SIZE - ret, "%5d", data_buf[i*col + j]);
			log_ret += snprintf(log_buf + log_ret, sizeof(log_buf) - log_ret, "%5d", data_buf[i*col + j]);
		}
		TOUCH_I("%s\n", log_buf);
	}

	ret += snprintf(w_buf + ret, PAGE_SIZE - ret, "\n");
	write_file(dev, w_buf, TIME_INFO_SKIP);
	memset(w_buf, 0, BUF_SIZE);

	return ret;
}

static int prd_open_result_get(struct device *dev, int frame)
{
	int ret = 0;

	switch (frame) {
		case PT_FRAME_1:
			// LFD ON Raw 1 frame data
			ret = prd_read_print_frame(dev, PT_FRAME_1_OFFSET, ROW_SIZE, COL_SIZE);
			break;
		case PT_FRAME_2:
			// LFD OFF Raw 1 frame data
			ret = prd_read_print_frame(dev, PT_FRAME_2_OFFSET, ROW_SIZE, COL_SIZE);
			break;
		case PT_FRAME_3:
			// Test 1, 2 diff 1 frame data (LFD OFF Rawdata - LFD ON Rawdata)
			ret = prd_read_print_frame(dev, PT_FRAME_3_OFFSET, ROW_SIZE, COL_SIZE);
			break;
		case PT_FRAME_4:
			// Open Result 1 frame
			ret = prd_read_print_frame(dev, PT_FRAME_4_OFFSET, ROW_SIZE, COL_SIZE);
			break;
	}
	return ret;
}

static int prd_short_result_get(struct device *dev, int frame)
{
	int ret = 0;

	switch (frame) {
		case PT_FRAME_1:
			// Mux short raw data
			ret = prd_read_print_frame(dev, PT_FRAME_1_OFFSET, ROW_SIZE, 4);
			break;
		case PT_FRAME_2:
			TOUCH_E("Attempt to read invalid frame\n");
			break;
		case PT_FRAME_3:
			TOUCH_E("Attempt to read invalid frame\n");
			break;
		case PT_FRAME_4:
			// Short Result 1 frame
			ret = prd_read_print_frame(dev, PT_FRAME_4_OFFSET, ROW_SIZE, 2);
			break;
	}
	return ret;
}

static void prd_read_print_data(struct device *dev, u8 type)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int jitter_frame_offset = 0;

	switch (type) {
	case OPEN_NODE_TEST:
		prd_open_result_get(dev, PT_FRAME_3);
		break;
	case SHORT_NODE_TEST:
		prd_short_result_get(dev, PT_FRAME_1); // Old solution : PT_FRAME_4
		break;
	case U3_M1_RAWDATA_TEST:
		prd_read_print_frame(dev, PT_FRAME_1_OFFSET, ROW_SIZE, 2);
		break;
	case U3_M1_JITTER_TEST:
		prd_read_print_frame(dev, PT_FRAME_4_OFFSET, ROW_SIZE, 2);
		break;
	case U3_M2_RAWDATA_TEST:
		prd_read_print_frame(dev, PT_FRAME_1_OFFSET, ROW_SIZE, COL_SIZE);
		break;
	case U3_M2_JITTER_TEST:
		if (param->jitter_test_type == PRD_JITTER) {
			jitter_frame_offset = PT_FRAME_2_OFFSET;
		} else if (param->jitter_test_type == PRD_JITTER_DIFFER) {
			jitter_frame_offset = PT_FRAME_4_OFFSET;
		} else if (param->jitter_test_type == PRD_JITTER_DELTA) {
			jitter_frame_offset = PT_FRAME_3_OFFSET;
		} else {
			TOUCH_I("ProductionTest Jitter frame offset error [%d]\n", jitter_frame_offset);
			goto error;
		}
		prd_read_print_frame(dev, jitter_frame_offset, ROW_SIZE, COL_SIZE);
		break;
	case U0_M1_RAWDATA_TEST:
		prd_read_print_frame(dev, PT_FRAME_1_OFFSET, ROW_SIZE, 2);
		break;
	case U0_M1_JITTER_TEST:
		prd_read_print_frame(dev, PT_FRAME_4_OFFSET, ROW_SIZE, 2);
		break;
	case U0_M2_RAWDATA_TEST:
		prd_read_print_frame(dev, PT_FRAME_1_OFFSET, ROW_SIZE, COL_SIZE);
		break;
	case U0_M2_JITTER_TEST:
		if (param->jitter_test_type == PRD_JITTER) {
			jitter_frame_offset = PT_FRAME_2_OFFSET;
		} else if (param->jitter_test_type == PRD_JITTER_DIFFER) {
			jitter_frame_offset = PT_FRAME_4_OFFSET;
		} else if (param->jitter_test_type == PRD_JITTER_DELTA) {
			jitter_frame_offset = PT_FRAME_3_OFFSET;
		} else {
			TOUCH_I("ProductionTest Jitter frame offset error [%d]\n", jitter_frame_offset);
			goto error;
		}
		prd_read_print_frame(dev, jitter_frame_offset, ROW_SIZE, COL_SIZE);
		break;
	case U3_BLU_JITTER_TEST:
		prd_read_print_frame(dev, PT_FRAME_3_OFFSET, ROW_SIZE, COL_SIZE);
		break;
	}

	return;
error:
	TOUCH_E("[%s] fail\n", __func__);
	return;
}

/* Rawdata compare result
	Pass : return 0
	Fail : return 1
*/
static int prd_compare_data(struct device *dev, u8 type, int* result)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int col_size = COL_SIZE;
	int i, j;
	int ret = 0;
	/* spec reading */
	char lower_str[64] = {0, };
	char upper_str[64] = {0, };
	char average_str[64] = {0, };

	//Average Jiiter Buf
	u16 m2_raw_average_buf[2][ROW_SIZE] = {{0}};

	switch (type) {
	case OPEN_NODE_TEST:
		snprintf(lower_str, sizeof(lower_str), "OPEN_Lower");
		snprintf(upper_str, sizeof(upper_str), "OPEN_Upper");
		break;
	case SHORT_NODE_TEST:
		snprintf(lower_str, sizeof(lower_str), "SHORT_Lower");
		snprintf(upper_str, sizeof(upper_str), "SHORT_Upper");
		col_size = SHORT_COL_SIZE;
		break;
	case U3_M1_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str), "U3_M1_Lower");
		snprintf(upper_str, sizeof(upper_str), "U3_M1_Upper");
		col_size = M1_COL_SIZE;
		break;
	case U3_M1_JITTER_TEST:
		snprintf(lower_str, sizeof(lower_str), "U3_M1_JITTER_Lower");
		snprintf(upper_str, sizeof(upper_str), "U3_M1_JITTER_Upper");
		col_size = M1_COL_SIZE;
		break;
	case U3_M2_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str), "U3_M2_Lower");
		snprintf(upper_str, sizeof(upper_str), "U3_M2_Upper");
		break;
	case U3_M2_JITTER_TEST:
		snprintf(lower_str, sizeof(lower_str), "U3_M2_JITTER_Lower");
		snprintf(upper_str, sizeof(upper_str), "U3_M2_JITTER_Upper");
		break;
	case U0_M1_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str), "U0_M1_Lower");
		snprintf(upper_str, sizeof(upper_str), "U0_M1_Upper");
		col_size = M1_COL_SIZE;
		break;
	case U0_M1_JITTER_TEST:
		snprintf(lower_str, sizeof(lower_str), "U0_M1_JITTER_Lower");
		snprintf(upper_str, sizeof(upper_str), "U0_M1_JITTER_Upper");
		col_size = M1_COL_SIZE;
		break;
	case U0_M2_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str), "U0_M2_Lower");
		snprintf(upper_str, sizeof(upper_str), "U0_M2_Upper");
		break;
	case U0_M2_JITTER_TEST:
		snprintf(lower_str, sizeof(lower_str), "U0_M2_JITTER_Lower");
		snprintf(upper_str, sizeof(upper_str), "U0_M2_JITTER_Upper");
		break;
	case U3_BLU_JITTER_TEST:
		snprintf(lower_str, sizeof(lower_str), "U3_BLU_JITTER_Lower");
		snprintf(upper_str, sizeof(upper_str), "U3_BLU_JITTER_Upper");
		break;
	}

	sic_get_limit(dev, lower_str, LowerImage);
	sic_get_limit(dev, upper_str, UpperImage);

	for (i = 0; i < ROW_SIZE; i++) {
		for (j = 0; j < col_size; j++) {
			//TOUCH_I("[%d] [%d] [%d] \n", data_buf[i*col_size + j], LowerImage[i][j], UpperImage[i][j]);
			if ((data_buf[i*col_size + j] < LowerImage[i][j]) ||
				(data_buf[i*col_size + j] > UpperImage[i][j])) {
#if 0  // if non Sub-LCD(AOD), need not this code
				if ((type != U0_M1_RAWDATA_TEST) && (i <= 1 && j <= 4)) {
					if (data_buf[i*col_size+j] != 0) {
						*result = 1;
						ret += snprintf(w_buf + ret, BUF_SIZE - ret,
						"F [%d][%d] = %d\n", i, j, data_buf[i*col_size+j]);
					}
				} else {
					*result = 1;
					ret += snprintf(w_buf + ret, BUF_SIZE - ret,
					"F [%d][%d] = %d\n", i, j, data_buf[i*col_size+j]);
				}
#else
				*result = 1;
				TOUCH_I("F [%d][%d] = %d , %d, %d\n", i, j, data_buf[i*col_size + j], LowerImage[i][j], UpperImage[i][j]);
				ret += snprintf(w_buf + ret, BUF_SIZE - ret, "F [%d][%d] = %d\n", i, j, data_buf[i*col_size + j]);
#endif
			}
		}
	}

	if (param->sd_test_set & (1<<AVERAGE_JITTER_TEST)) {
		if ((type == U3_M2_JITTER_TEST) || (type == U3_BLU_JITTER_TEST) || (type == U3_M2_JITTER_TEST)) {
			//read spec file
			snprintf(average_str, sizeof(average_str),
		          "AVERAGE_JITTER");
			sic_get_limit(dev, average_str, AverageImage);

			//Average jitter
			for (i = 0; i < ROW_SIZE; i++) {
				for (j = 0; j < col_size; j++) {
					//Average Jitter Buf store
					//m2_raw_average_buf[j] += data_buf[i*col_size+j];
					if(j < (col_size / 2)) {
						m2_raw_average_buf[0][i] += data_buf[i*col_size+j];
						//TOUCH_I("[Left]aver_sum:%d, data:%d",
						//m2_raw_average_buf[0][i], data_buf[i*col_size+j]);
					} else {
						m2_raw_average_buf[1][i] += data_buf[i*col_size+j];
						//TOUCH_I("[Right]aver_sum:%d, data:%d",
						//m2_raw_average_buf[1][i], data_buf[i*col_size+j]);
					}
				}
				m2_raw_average_buf[0][i] /= (col_size / 2); //Left
				m2_raw_average_buf[1][i] /= (col_size - (col_size / 2)); //Right

				//TOUCH_I("left:%d, left_spec:%d, right:%d, rihgt_spec:%d \n",
				//m2_raw_average_buf[0][i], AverageImage[i][0], m2_raw_average_buf[1][i], AverageImage[i][1]);

				if (m2_raw_average_buf[0][i] > AverageImage[i][0]) {
					*result = 1;
					ret += snprintf(w_buf + ret, BUF_SIZE - ret,
							"F [%d] Row Left Average = %d\n", i, m2_raw_average_buf[0][i]);
					TOUCH_I("F [%d] Row Left Average = %d\n", i, m2_raw_average_buf[0][i]);
				}
				if (m2_raw_average_buf[1][i] > AverageImage[i][1]) {
					*result = 1;
					ret += snprintf(w_buf + ret, BUF_SIZE - ret,
							"F [%d] Row Right Average = %d\n", i, m2_raw_average_buf[1][i]);
					TOUCH_I("F [%d] Row Right Average = %d\n", i, m2_raw_average_buf[1][i]);
				}
			}
		}
	}

	return ret;
}

static int prd_openshort_test(struct device *dev)
{
	int type = 0;
	int ret = 0;
	int write_test_mode_result = 0;
	u32 result = 0;
	u32 openshort_all_result = 0;

	// New solution : Get openshort rawdata from firmware and compare data from driver code
	/* 1. open_test */
	TOUCH_I("\n[OPEN_TEST]\n");
	write_file(dev, "\n[OPEN_TEST]", TIME_INFO_SKIP);

	type = OPEN_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	prd_read_print_data(dev, type);

	/* openshort compare result(pass : 0 fail : 1) */
	memset(w_buf, 0, BUF_SIZE);

	ret = prd_compare_data(dev, type, &result);

	ret += snprintf(w_buf + ret, BUF_SIZE - ret, "%s : %s\n", test_name_str[type], result==0? "Pass":"Fail");
	TOUCH_I("%s : %s\n", test_name_str[type], result==0? "Pass":"Fail");
	if (result)
		openshort_all_result |= 0x1;

	write_file(dev, w_buf, TIME_INFO_SKIP);

	/* 2. short_test */
	TOUCH_I("\n[SHORT_TEST]\n");
	write_file(dev, "\n[SHORT_TEST]", TIME_INFO_SKIP);

	type = SHORT_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	prd_read_print_data(dev, type);

	/* openshort compare result(pass : 0 fail : 1) */
	memset(w_buf, 0, BUF_SIZE);

	ret = prd_compare_data(dev, type, &result);

	ret += snprintf(w_buf + ret, BUF_SIZE - ret, "%s : %s\n", test_name_str[type], result==0? "Pass":"Fail");
	TOUCH_I("%s : %s\n", test_name_str[type], result==0? "Pass":"Fail");
	if (result)
		openshort_all_result |= 0x2;

	write_file(dev, w_buf, TIME_INFO_SKIP);


#if 0	// Old Solution : Get result from firmware
	/* Test Type Write */
	TOUCH_I("\n[OPEN_TEST]\n");
	write_file(dev, "\n[OPEN_TEST]", TIME_INFO_SKIP);

	/* 1. open_test */
	type = OPEN_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	sw49107_reg_read(dev, PT_TEST_PF_RESULT, (u8 *)&open_result, sizeof(open_result));
	TOUCH_I("open_result = %d\n", open_result);

	ret = prd_read_data(dev, type);
	if (open_result) {
		// open test logging
		openshort_all_result |= 0x1;
	}

	/* Test Type Write */
	TOUCH_I("\n[SHORT_TEST]\n");
	write_file(dev, "\n[SHORT_TEST]", TIME_INFO_SKIP);

	/* 2. short_test */
	type = SHORT_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	sw49107_reg_read(dev, PT_TEST_PF_RESULT, (u8 *)&short_result, sizeof(short_result));
	TOUCH_I("short_result = %d\n", short_result);

	ret = prd_read_data(dev, type);
	if (short_result) {
		// short test logging
		openshort_all_result |= 0x2;
	}

	/* fail case */
	if (openshort_all_result != 0) {
		ret += snprintf(w_buf + ret, BUF_SIZE - ret, "OPEN_SHORT_ALL_TEST : Fail\n");
		TOUCH_I("OPEN_SHORT_ALL_TEST : Fail\n");

		/*
		ret = snprintf(w_buf, BUF_SIZE, "\n   : ");
		for (i = 0; i < COL_SIZE; i++) {
			ret += snprintf(w_buf + ret,
					BUF_SIZE - ret,
					" [%2d] ", i);
			TOUCH
		}

		for (i = 0; i < ROW_SIZE; i++) {
			ret += snprintf(w_buf + ret,
					BUF_SIZE - ret,
					"\n[%2d] ", i);
			for (j = 0; j < COL_SIZE; j++) {
				ret += snprintf(w_buf + ret,
					BUF_SIZE - ret, "%5s ",
				((buf[i][j] & 0x3) == 0x3) ?  "O,S" :
				((buf[i][j] & 0x1) == 0x1) ?  "O" :
				((buf[i][j] & 0x2) == 0x2) ?  "S" : "-");
			}
		}
		ret += snprintf(w_buf + ret, BUF_SIZE - ret, "\n");
		*/
	} else {
		ret += snprintf(w_buf + ret, BUF_SIZE - ret, "OPEN_SHORT_ALL_TEST : Pass\n");
		TOUCH_I("OPEN_SHORT_ALL_TEST : Pass\n");
	}
	write_file(dev, w_buf, TIME_INFO_SKIP);
#endif

	return openshort_all_result;
}

int tune_goft_print(t_goft_tune tune_val, char* log_buf, int offset, int mode_select)
{
	if (mode_select == 1) {
		if (tune_val.b.r_goft_tune_m1_sign == 0) {
			offset += snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"-%d  ", tune_val.b.r_goft_tune_m1);
		} else {
			offset += snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"%d  ", tune_val.b.r_goft_tune_m1);
		}
	} else if (mode_select == 2) {
		if (tune_val.b.r_goft_tune_m2_sign == 0) {
			offset += snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"-%d  ", tune_val.b.r_goft_tune_m2);
		} else {
			offset += snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"%d  ", tune_val.b.r_goft_tune_m2);
		}
	}

	offset += snprintf(log_buf + offset, tc_tune_code_size - offset, "\n");

	return offset;
}

int tune_loft_print(int num_ch, u16* tune_val, char* log_buf, int offset)
{
	int i;
	int temp = 0;

	offset = snprintf(log_buf, tc_tune_code_size, "Odd :  ");
	for (i = 0; i < num_ch; i++) {
		if (((tune_val[i] >> 5) & 1) == 0) {
			temp = tune_val[i] & 0x1F;

			offset += snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"-%d ", temp);
		} else if (((tune_val[i] >> 5) & 1) == 1) {
			temp = tune_val[i] & 0x1F;

			offset += snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"%d  ", temp);
		} else {
			offset += snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"ISB  ");
		}
	}
	offset += snprintf(log_buf + offset, tc_tune_code_size - offset, "\n");

	offset += snprintf(log_buf + offset, tc_tune_code_size - offset, "Even : ");
	for (i = 0; i < num_ch; i++) {
		//TOUCH_I("psy>> %x %x << ", (tune_val[i] >> 13), ((tune_val[i] >> 13) & 1));
		if (((tune_val[i] >> 13) & 1) == 0) {
			temp = (tune_val[i] >> 8) & 0x1F;

			offset += snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"-%d ", temp);
		} else if (((tune_val[i] >> 13) & 1) == 1) {
			temp = (tune_val[i] >> 8) & 0x1F;

			offset += snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"%d  ", temp);
		} else {
			offset += snprintf(log_buf + offset,
					tc_tune_code_size - offset,
					"ISB  ");
		}
	}

	offset += snprintf(log_buf + offset, tc_tune_code_size - offset, "\n");

	return offset;
}

static void tune_display(struct device *dev, struct tune_data_format *t, int type)
{
	char log_buf[tc_tune_code_size] = {0,};
	int offset = 0;

	switch (type) {
	case U0_M1_RAWDATA_TEST:
		// GOFT
		offset += snprintf(log_buf, tc_tune_code_size - offset, "GOFT left  tune_code_read : ");
		offset += tune_goft_print(t->r_goft_tune_u0_m1m2_left, log_buf, offset, 1);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "GOFT right tune_code_read : ");
		offset += tune_goft_print(t->r_goft_tune_u0_m1m2_right,	log_buf, offset, 1);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		//LOFT
		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT left tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u0_m1_left, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT right tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u0_m1_right, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		break;

	case U0_M2_RAWDATA_TEST:
		// GOFT
		offset += snprintf(log_buf, tc_tune_code_size, "GOFT left  tune_code_read : ");
		offset += tune_goft_print(t->r_goft_tune_u0_m1m2_left, log_buf, offset, 2);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "GOFT right tune_code_read : ");
		offset += tune_goft_print(t->r_goft_tune_u0_m1m2_right, log_buf, offset, 2);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		// LOFT
		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT G1 left tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u0_m2_g1_left, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT G1 right tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u0_m2_g1_right, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT G2 left tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u0_m2_g2_left, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT G2 right tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u0_m2_g2_right, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT G3 left tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u0_m2_g3_left, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);


		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT G3 right tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u0_m2_g3_right, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		break;
	case U3_M2_RAWDATA_TEST:

		// GOFT
		offset += snprintf(log_buf, tc_tune_code_size, "GOFT left  tune_code_read : ");
		offset += tune_goft_print(t->r_goft_tune_u3_m1m2_left, log_buf, offset, 2);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "GOFT right tune_code_read : ");
		offset += tune_goft_print(t->r_goft_tune_u3_m1m2_right, log_buf, offset, 2);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		// LOFT
		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT G1 left tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u3_m2_g1_left, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT G1 right tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u3_m2_g1_right, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT G2 left tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u3_m2_g2_left, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT G2 right tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u3_m2_g2_right, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT G3 left tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u3_m2_g3_left, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);


		offset = 0;
		offset += snprintf(log_buf, tc_tune_code_size, "LOFT G3 right tune_code_read : ");
		offset += tune_loft_print(LOFT_CH_NUM, t->r_loft_tune_u3_m2_g3_right, log_buf, offset);
		write_file(dev, log_buf, TIME_INFO_SKIP);
		TOUCH_I("%s", log_buf);

		break;
	}
}

static void read_tune_code(struct device *dev, u8 type)
{
	struct tune_data_format t;

	sw49107_reg_read(dev, tune_code_addr, &t,
				sizeof(struct tune_data_format));

	write_file(dev, "\n[Read Tune Code]\n", TIME_INFO_SKIP);
	TOUCH_I("\n");
	TOUCH_I("[Read Tune Code]");

	tune_display(dev, &t, type);
	write_file(dev, "\n", TIME_INFO_SKIP);

}

static int prd_rawdata_test(struct device *dev, u8 type)
{
	char test_type[32] = {0, };
	int result = 0;
	int write_test_mode_result = 0;
	int ret = 0;

	if (type < TEST_TOTAL_NUM) {
		sprintf(test_type, "\n[%s]", test_name_str[type]);
		TOUCH_I("%s", test_type);
	} else {
		TOUCH_E("Test Type not defined");
		return 1;
	}

	/* Test Type Write */
	write_file(dev, test_type, TIME_INFO_SKIP);

	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("production test couldn't be done\n");
		return 1;
	}

	prd_read_print_data(dev, type);

	/* rawdata compare result(pass : 0 fail : 1) */
	memset(w_buf, 0, BUF_SIZE);
	ret = prd_compare_data(dev, type, &result);
	if (result != 0) {
		ret += snprintf(w_buf + ret, BUF_SIZE - ret, "%s : Fail\n", test_name_str[type]);
		TOUCH_I("%s : Fail\n",test_name_str[type]);
	} else {
		ret += snprintf(w_buf + ret, BUF_SIZE - ret, "%s : Pass\n", test_name_str[type]);
		TOUCH_I("%s : Pass\n",test_name_str[type]);
	}
	write_file(dev, w_buf, TIME_INFO_SKIP);

	/* To Do - tune code result check */
	/*result = */
	if((type == U3_M1_RAWDATA_TEST) ||
		(type == U3_M2_RAWDATA_TEST) ||
		(type == U0_M1_RAWDATA_TEST) ||
		(type == U0_M2_RAWDATA_TEST))
	read_tune_code(dev, type);

	return result;
}

static int firmware_version_log(struct device *dev)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	int offset = 0;
	unsigned char log_buf[LOG_BUF_SIZE] = {0,};
	int boot_mode = 0;
	int ret = 0;

	boot_mode = touch_check_boot_mode(dev);
	if (boot_mode >= TOUCH_MINIOS_MFTS_FOLDER)
		sw49107_ic_info(dev);

	offset += snprintf(log_buf, sizeof(log_buf),
			"==================== Version Info ====================\n");
	offset += snprintf(log_buf + offset, sizeof(log_buf) - offset,
			"version: v%d.%02d, build: %d, chip id: %d, protocol: %d\n",
			d->ic_info.version.major_ver, d->ic_info.version.minor_ver, d->ic_info.version.build_ver,
			d->ic_info.version.chip_id, d->ic_info.version.protocol_ver);
	offset += snprintf(log_buf + offset, sizeof(log_buf) - offset,
			"product id: %s, chip rev: %x\n", d->ic_info.product_id, d->ic_info.pt_info.chip_rev);
	offset += snprintf(log_buf + offset, sizeof(log_buf) - offset,
			"lcm type: %d, lot_num: %d, fpc_type: %d\n",
			d->ic_info.pt_info.lcm_type, d->ic_info.pt_info.lot_num, d->ic_info.pt_info.fpc_type);
	offset += snprintf(log_buf + offset, sizeof(log_buf) - offset,
			"date: %04d.%02d.%02d, time: %02d:%02d:%02d, site: %d\n",
			d->ic_info.pt_info.pt_date_year, d->ic_info.pt_info.pt_date_month, d->ic_info.pt_info.pt_date_day,
			d->ic_info.pt_info.pt_time_hour, d->ic_info.pt_info.pt_time_min, d->ic_info.pt_info.pt_time_sec,
			d->ic_info.pt_info.pt_site);
	offset += snprintf(log_buf + offset, sizeof(log_buf) - offset,
			"======================================================\n");

	write_file(dev, log_buf, TIME_INFO_SKIP);

	// check pt info (cv7a chip rev = 1)
	if(!(0 < d->ic_info.pt_info.chip_rev && d->ic_info.pt_info.chip_rev < 11)) {
		TOUCH_E("do not write pt-info. (touch ic chip rev = %x / pass chip rev = %x\n",d->ic_info.pt_info.chip_rev, 1);
		ret = 1;
	}

	return ret;
}

static ssize_t show_sd(struct device *dev, char *buf)
{

	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int pt_command;
	int ret = 0;
	int pt_info_ret = 0;
	int openshort_ret = 0;
	int u3_m1_raw_ret = 0;
	int u3_m2_raw_ret = 0;
	int u3_m1_jitter_ret = 0;
	int u3_m2_jitter_ret = 0;

	/* file create , time log */
	write_file(dev, "\nShow_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_sd Test Start\n");

	/* LCD mode check */
	if (d->lcd_mode != LCD_MODE_U3) {
		ret = snprintf(buf + ret, PAGE_SIZE - ret, "LCD mode is not U3. Test Result : Fail\n");
		TOUCH_I("LCD mode is not U3. Test Result : Fail\n");
		goto out;
	}

	/*
		PT_INFO_TEST
		pass : 0, fail : 1
	*/
	pt_info_ret = firmware_version_log(dev);

	mutex_lock(&ts->lock);

	/* Test enter */
	pt_command = (0x3 << 8) + U3_PT_TEST;
	sw49107_write_value(dev, TC_PT_TEST_CTL, pt_command);

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	sw49107_tc_driving(dev, LCD_MODE_STOP);

	/*
		OPEN_SHORT_ALL_TEST
		open - pass : 0, fail : 1
		short - pass : 0, fail : 2
	*/
	openshort_ret = prd_openshort_test(dev);

	/*
		U3_M1_RAWDATA_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_M1_RAWDATA_TEST)) {
		u3_m1_raw_ret = prd_rawdata_test(dev, U3_M1_RAWDATA_TEST);
	}

	/*
		U3_M2_RAWDATA_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_M2_RAWDATA_TEST)) {
		u3_m2_raw_ret = prd_rawdata_test(dev, U3_M2_RAWDATA_TEST);
	}

	/*
		U3_M1_JITTER_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_M1_JITTER_TEST)) {
		u3_m1_jitter_ret = prd_rawdata_test(dev, U3_M1_JITTER_TEST);
	}

	/*
		U3_M2_JITTER_TEST
		pass : 0, fail : 1
	*/
	if(param->sd_test_set & (1<<U3_M2_JITTER_TEST)) {
		u3_m2_jitter_ret = prd_rawdata_test(dev, U3_M2_JITTER_TEST);
	}

	ret = snprintf(buf, PAGE_SIZE,
			"\n========RESULT=======\n");

	if ((pt_info_ret + u3_m1_raw_ret + u3_m2_raw_ret + u3_m1_jitter_ret + u3_m2_jitter_ret) == 0) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : Pass\n");
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : Fail (pt_info:%d/m1_raw:%d/m2_raw:%d/m1_jitter:%d/m2_jitter:%d)\n",
				pt_info_ret, u3_m1_raw_ret, u3_m2_raw_ret, u3_m1_jitter_ret, u3_m2_jitter_ret);
	}

	if (openshort_ret == 0) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Channel Status : Pass\n");
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Channel Status : Fail (open:%d/short:%d)\n",
			((openshort_ret & 0x1) == 0x1) ? 1 : 0,
			((openshort_ret & 0x2) == 0x2) ? 1 : 0);
	}

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"=====================\n");

	write_file(dev, buf, TIME_INFO_SKIP);

	sw49107_reset_ctrl(dev, POWER_HW_RESET_SYNC);
	mutex_unlock(&ts->lock);

out:
	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);
	log_file_size_check(dev);
	TOUCH_I("Show_sd Test End\n");

	return ret;
}

//for siw app
static int start_firmware(struct device *dev)
{
	u32 const cmd = IT_NONE;
	u32 check_data = 0;
	int ret = 0;

	/* Release F/W to operate */
	ret = sw49107_reg_write(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, (void *)&cmd,
			sizeof(u32));
	if (ret < 0) {
		goto error;
	}
	ret = sw49107_reg_read(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &check_data,
			sizeof(u32));
	if (ret < 0) {
		goto error;
	}
	ret = sw49107_reg_read(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &check_data,
			sizeof(u32));
	if (ret < 0) {
		goto error;
	}
	//TOUCH_I("check_data : %x\n", check_data);

error:
	return ret;
}

static int stop_firmware(struct device *dev, u32 wdata)
{
	u32 read_val;
	u32 check_data=0;
	int try_cnt=0;
	int ret = 0;

	/* STOP F/W to check */
	sw49107_reg_write(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &wdata, sizeof(u32));
	sw49107_reg_read(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &check_data,
			sizeof(u32));
	sw49107_reg_read(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &check_data,
			sizeof(u32));

	try_cnt = 1000;
	do
	{
		--try_cnt;
		if (try_cnt == 0) {
			TOUCH_E("[ERR]get_data->try_cnt == 0\n");
			ret = 1;
			goto error;
		}
		sw49107_reg_read(dev, ADDR_CMD_REG_SIC_GETTER_READYSTATUS,
				&read_val, sizeof(u32));
		//TOUCH_I("read_val = [%x] , RS_IMAGE = [%x]\n",
		//	read_val, (u32)RS_IMAGE);
		touch_msleep(10);
	} while(read_val != (u32)RS_IMAGE);

error:
	return ret;
}

static ssize_t prd_show_app_op_end(struct device *dev, char *buf, int prev_mode)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	int ret = 0;

	buf[0] = REPORT_END_RS_OK;
	if (prev_mode != APP_REPORT_OFF) {
		prd->prd_app_mode = APP_REPORT_OFF;
		ret = start_firmware(dev);
		if (ret < 0) {
			TOUCH_E("Invalid get_data request!\n");
			buf[0] = REPORT_END_RS_NG;
		}
	}

	return 1;
}

static int prd_show_prd_get_data_raw_core(struct device *dev,
					u8 *buf, int size, u32 cmd, u32 offset, int flag)
{
	int ret = 0;

	if (cmd != DONT_USE_CMD) {
		ret = stop_firmware(dev, cmd);
		if (ret < 0) {
			goto out;
		}
	}

	ret = sw49107_reg_write(dev, SERIAL_DATA_OFFSET, (u8 *)&offset, sizeof(offset));
	if (ret < 0) {
		goto out;
	}

	ret = sw49107_reg_read(dev, SERIAL_DATA_ACCESS, (void *)buf, size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int prd_show_prd_get_data_do_raw_ait(struct device *dev, u8 *buf, int size, int flag)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->m2_buf_rawdata;

	return prd_show_prd_get_data_raw_core(dev, pbuf, size,
				CMD_RAWDATA, AIT_RAW_DATA_OFFSET, flag);
}

static int prd_show_prd_get_data_do_ait_basedata(struct device *dev, u8 *buf, int size, int step, int flag)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->m2_buf_rawdata;

	return prd_show_prd_get_data_raw_core(dev, pbuf, size,
				CMD_BASE_DATA, AIT_BASE_DATA_OFFSET, flag);
}


static int prd_show_prd_get_data_do_deltadata(struct device *dev, u8 *buf, int size, int flag)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	int16_t *pbuf = (buf) ? (int16_t *)buf : prd->m2_buf_rawdata;
	int size_rd = (PRD_DELTA_SIZE<<1);
	int row, col;
	int i;
	int ret = 0;

	ret = prd_show_prd_get_data_raw_core(dev, (u8 *)prd->buf_delta, size_rd,
				CMD_DELTADATA, DELTA_DATA_OFFSET, flag);
	if (ret < 0) {
		goto out;
	}

	memset(pbuf, 0, size);

	for (i = 0; i < PRD_M2_ROW_COL_SIZE; i++){
		row = i / PRD_COL_SIZE;
		col = i % PRD_COL_SIZE;
		pbuf[i] = prd->buf_delta[(row + 1)*(PRD_COL_SIZE + 2) + (col + 1)];
	}

out:
	return ret;
}

static int prd_show_prd_get_data_do_labeldata(struct device *dev, u8 *buf, int size, int flag)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	u8 *pbuf = (buf) ? buf : prd->buf_label;
	int size_rd = PRD_LABEL_TMP_SIZE;
	int row, col;
	int i;
	int ret = 0;

	ret = prd_show_prd_get_data_raw_core(dev, (u8 *)prd->buf_label_tmp, size_rd,
				CMD_LABELDATA, LABLE_DATA_OFFSET, flag);
	if (ret < 0) {
		goto out;
	}

	memset(pbuf, 0, size);

	for (i = 0; i < PRD_M2_ROW_COL_SIZE; i++){
		row = i / PRD_COL_SIZE;
		col = i % PRD_COL_SIZE;
		pbuf[i] = prd->buf_label_tmp[(row + 1)*(PRD_COL_SIZE + 2) + (col + 1)];
	}

out:
	return ret;
}

static int prd_show_prd_get_data_do_debug_buf(struct device *dev, u8 *buf, int size, int flag)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->buf_debug;

	return prd_show_prd_get_data_raw_core(dev, pbuf, size,
				CMD_DEBUGDATA, AIT_DEBUG_BUF_DATA_OFFSET, flag);
}

/*
static const char *prd_app_mode_str[] = {
	[APP_REPORT_OFF]		= "OFF",
	[APP_REPORT_RAW]		= "RAW",
	[APP_REPORT_BASE]		= "BASE",
	[APP_REPORT_LABEL]		= "LABEL",
	[APP_REPORT_DELTA]		= "DELTA",
	[APP_REPORT_DEBUG_BUF]	= "DEBUG_BUF",
};
*/
static ssize_t prd_show_app_operator(struct device *dev, char *buf, int mode)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	u8 *pbuf = (u8 *)prd->m2_buf_rawdata;
	int size = (PRD_M2_ROW_COL_SIZE<<1);
	int flag = PRD_SHOW_FLAG_DISABLE_PRT_RAW;
	int prev_mode = prd->prd_app_mode;

	//TOUCH_I("show app mode : %s(%d), 0x%X\n",
	//			prd_app_mode_str[mode], mode, flag);

	if (mode == APP_REPORT_OFF) {
		size = prd_show_app_op_end(dev, buf, prev_mode);
		goto out;
	}

	if (mode < APP_REPORT_MAX) {
		prd->prd_app_mode = mode;
	}

	switch (mode) {
	case APP_REPORT_RAW:
		prd_show_prd_get_data_do_raw_ait(dev, pbuf, size, flag);
		break;
	case APP_REPORT_BASE:
		prd_show_prd_get_data_do_ait_basedata(dev, pbuf, size, 0, flag);
		break;
	case APP_REPORT_DELTA:
		prd_show_prd_get_data_do_deltadata(dev, pbuf, size, flag);
		break;
	case APP_REPORT_LABEL:
		size = PRD_M2_ROW_COL_SIZE;
		pbuf = (u8 *)prd->buf_label,
		prd_show_prd_get_data_do_labeldata(dev, pbuf, size, flag);
		break;
	case APP_REPORT_DEBUG_BUF:
		size = PRD_DEBUG_BUF_SIZE;
		pbuf = (u8 *)prd->buf_debug,
		prd_show_prd_get_data_do_debug_buf(dev, pbuf, size, flag);
		//prd_show_prd_get_data_raw_core(dev, pbuf, PRD_DEBUG_BUF_SIZE,
		//		APP_IT_DONT_USE_CMD, AIT_DEBUG_BUF_DATA_OFFSET, flag);
		break;
	default:
		if (prev_mode != APP_REPORT_OFF) {
			prd_show_app_op_end(dev, buf, prev_mode);
		}
		size = 0;
		break;
	}

	if (size) {
		memcpy(buf, pbuf, size);
	}

out:
	return (ssize_t)size;
}

static ssize_t prd_show_app_raw(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_RAW);
}

static ssize_t prd_show_app_base(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_BASE);
}

static ssize_t prd_show_app_label(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_LABEL);
}

static ssize_t prd_show_app_delta(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_DELTA);
}

static ssize_t prd_show_app_debug_buf(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_DEBUG_BUF);
}

static ssize_t prd_show_app_end(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_OFF);
}

static ssize_t prd_show_app_info(struct device *dev, char *buf)
{
	u32 temp = PRD_SYS_ATTR_EN_FLAG;

	memset(buf, 0, PRD_APP_INFO_SIZE);

	buf[0] = (temp & 0xff);
	buf[1] = ((temp >> 8) & 0xff);
	buf[2] = ((temp >> 16) & 0xff);
	buf[3] = ((temp >> 24) & 0xff);

	buf[8] = PRD_ROW_SIZE;
	buf[9] = PRD_COL_SIZE;
	buf[10] = PRD_COL_ADD;
	buf[11] = PRD_CH;
	buf[12] = PRD_M1_COL;
	buf[13] = PRD_CMD_TYPE;
	buf[14] = SECOND_SCR_BOUND_I;
	buf[15] = SECOND_SCR_BOUND_J;

	TOUCH_I("<prd info> F:%08Xh \n",temp);
	TOUCH_I("R:%d C:%d C_A:%d CH:%d M1_C:%d \n	\
			CMD_T:%d S_SCR_I:%d S_SCR_J:%d \n",	\
		PRD_ROW_SIZE, PRD_COL_SIZE, PRD_COL_ADD, PRD_CH, PRD_M1_COL,	\
		PRD_CMD_TYPE, SECOND_SCR_BOUND_I, SECOND_SCR_BOUND_J);

	return PRD_APP_INFO_SIZE;
}

static struct siw_hal_prd_data *siw_hal_prd_alloc(struct device *dev)
{
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd;

	prd = devm_kzalloc(dev, sizeof(*prd), GFP_KERNEL);
	if (!prd) {
		TOUCH_E("failed to allocate memory for prd\n");
		goto out;
	}

	snprintf(prd->name, sizeof(prd->name)-1, "%s-prd", dev_name(dev));

	prd->dev = d->dev;

	d->prd = prd;

out:
	return prd;
}
//for siw app end

static ssize_t get_data(struct device *dev, int16_t *buf, u32 wdata)
{
	s16 *delta_buf = NULL;
	s8 *label_buf = NULL;
	u32 m2_data_offset;
	int i, row, col;
	int ret = 0;

	TOUCH_I("======== get_data(%d) ========\n", wdata);

	/***********************************************
	 * Match enum value between CMD_XXX and IT_XXX *
	 * when you call stop_firmware.                *
	 ***********************************************/
	if (stop_firmware(dev, wdata)) {
		TOUCH_E("fail to stop FW\n");
		ret = 1;
		goto getdata_error;
	}

	switch(wdata){
	case CMD_RAWDATA:
		m2_data_offset = AIT_RAW_DATA_OFFSET;
		sw49107_reg_write(dev, SERIAL_DATA_OFFSET,
							(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		sw49107_reg_read(dev, SERIAL_DATA_ACCESS, (int16_t *)buf, sizeof(int16_t)*ROW_SIZE*COL_SIZE);
		break;
	case CMD_BASE_DATA:
		m2_data_offset = AIT_BASE_DATA_OFFSET;
		sw49107_reg_write(dev, SERIAL_DATA_OFFSET,
							(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		sw49107_reg_read(dev, SERIAL_DATA_ACCESS, (int16_t *)buf, sizeof(int16_t)*ROW_SIZE*COL_SIZE);
		break;

	case CMD_DELTADATA:
		delta_buf = devm_kzalloc(dev, sizeof(u16) * ((COL_SIZE+2) * (ROW_SIZE+2)),
		GFP_KERNEL);

		if (delta_buf == NULL) {
			TOUCH_E("delta_buf mem_error\n");
			ret = 1;
			goto getdata_error;
		}
		m2_data_offset = DELTA_DATA_OFFSET;
		sw49107_reg_write(dev, SERIAL_DATA_OFFSET,
							(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		sw49107_reg_read(dev, SERIAL_DATA_ACCESS, (s16 *)delta_buf,
								sizeof(int16_t)*(ROW_SIZE+2)*(COL_SIZE+2));

		for(i = 0; i < ROW_SIZE*COL_SIZE; i++)
		{
			row = i / COL_SIZE;
			col = i % COL_SIZE;
			buf[i] = delta_buf[(row + 1)*(COL_SIZE + 2*1) + (col + 1)];
		}

		if(delta_buf) {
			devm_kfree(dev, delta_buf);
		}
		break;
	case CMD_LABELDATA:
		label_buf = devm_kzalloc(dev, sizeof(s8) * ((COL_SIZE+2) * (ROW_SIZE+2)),
		GFP_KERNEL);

		if (label_buf == NULL) {
			TOUCH_E("label_buf mem_error\n");
			ret = 1;
			goto getdata_error;
		}
		m2_data_offset = LABLE_DATA_OFFSET;
		sw49107_reg_write(dev, SERIAL_DATA_OFFSET,
							(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		sw49107_reg_read(dev, SERIAL_DATA_ACCESS, (s8 *)label_buf,
								sizeof(s8)*(ROW_SIZE+2)*(COL_SIZE+2));

		for(i = 0; i < ROW_SIZE*COL_SIZE; i++)
		{
			row = i / COL_SIZE;
			col = i % COL_SIZE;
			buf[i] = (int16_t)label_buf[(row + 1)*(COL_SIZE + 2*1) + (col + 1)];
		}

		if(label_buf) {
			devm_kfree(dev, label_buf);
		}
		break;
	case CMD_DEBUGDATA:
		m2_data_offset = AIT_DEBUG_BUF_DATA_OFFSET;
		sw49107_reg_write(dev, SERIAL_DATA_OFFSET,
							(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		sw49107_reg_read(dev, SERIAL_DATA_ACCESS, (int16_t *)buf,
							sizeof(int16_t)*DEBUG_ROW_SIZE*DEBUG_COL_SIZE);
		break;
	default:
		TOUCH_E("Invalid get_data request!\n");
	}

getdata_error:
	start_firmware(dev);

	return ret;
}

static ssize_t show_fdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	int ret = 0;
	int ret2 = 0;
	u8 type = U3_M2_RAWDATA_TEST;

	/* LCD off */
	if (d->lcd_mode != LCD_MODE_U3) {
		ret = snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD Off. Test Result : Fail\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	ret2 = write_test_mode(dev, type);
	if (ret2 == 0) {
		TOUCH_E("write_test_mode fail\n");
		/*
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(ts->caps.hw_reset_delay);
		ts->driver->init(dev);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		*/
		sw49107_reset_ctrl(dev, POWER_HW_RESET_SYNC);
		return ret;
	}

	prd_read_print_data(dev, type);

	/*
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(ts->caps.hw_reset_delay);
	ts->driver->init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	*/
	sw49107_reset_ctrl(dev, POWER_HW_RESET_SYNC);
	mutex_unlock(&ts->lock);

	return ret;
}
static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	struct prd_test_param *param = &prd->prd_param;
	int pt_info_ret = 0;
	int m1_rawdata_ret = 0;
	int m2_rawdata_ret = 0;
	int m1_jitter_ret = 0;
	int m2_jitter_ret = 0;
	int ret = 0;

	/* file create , time log */
	write_file(dev, "\nShow_lpwg_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_lpwg_sd Test Start\n");

	/* LCD mode check */
	if (d->lcd_mode != LCD_MODE_U0) {
		ret = snprintf(buf + ret, PAGE_SIZE - ret,
			"LCD mode is not U0. Test Result : Fail\n");
		TOUCH_I("LCD mode is not U0. Test Result : Fail\n");
		return ret;
	}

	/*
		PT_INFO_TEST
		pass : 0, fail : 1
	*/
	pt_info_ret = firmware_version_log(dev);

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	sw49107_tc_driving(dev, LCD_MODE_STOP);

	/*
		U0_M1_RAWDATA_TEST
		pass : 0, fail : 1
	*/
	if (param->lpwg_sd_test_set & (1<<U0_M1_RAWDATA_TEST)) {
		m1_rawdata_ret = prd_rawdata_test(dev, U0_M1_RAWDATA_TEST);
	}

	/*
		U0_M2_RAWDATA_TEST
		pass : 0, fail : 1
	*/
	if (param->lpwg_sd_test_set & (1<<U0_M2_RAWDATA_TEST)) {
		m2_rawdata_ret = prd_rawdata_test(dev, U0_M2_RAWDATA_TEST);
	}

	/*
		U0_M1_JITTER_TEST
		pass : 0, fail : 1
	*/
	if (param->lpwg_sd_test_set & (1<<U0_M1_JITTER_TEST)) {
		m1_jitter_ret = prd_rawdata_test(dev, U0_M1_JITTER_TEST);
	}

	/*
		U0_M2_JITTER_TEST
		BLU Jitter - pass : 0, fail : 1
	*/
	if (param->lpwg_sd_test_set & (1<<U0_M2_JITTER_TEST)) {
		m2_jitter_ret = prd_rawdata_test(dev, U0_M2_JITTER_TEST);
	}

	ret = snprintf(buf + ret, PAGE_SIZE, "========RESULT=======\n");

	if (!pt_info_ret && !m1_rawdata_ret && !m2_rawdata_ret && !m1_jitter_ret && !m2_jitter_ret) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"LPWG RawData : %s\n", "Pass");
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"LPWG RawData : %s (pt_info:%d/m1_raw:%d/m2_raw:%d/m1_jitter:%d/m2_jitter:%d)\n", "Fail",
			pt_info_ret, m1_rawdata_ret, m2_rawdata_ret, m1_jitter_ret, m2_jitter_ret);
	}

	write_file(dev, buf, TIME_INFO_SKIP);

#if (0)
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(ts->caps.hw_reset_delay);
	ts->driver->init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	}
#else
	sw49107_reset_ctrl(dev, POWER_HW_RESET_SYNC);
	mutex_unlock(&ts->lock);
#endif

	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);
	log_file_size_check(dev);
	TOUCH_I("Show_lpwg_sd Test End\n");

	return ret;
}

#define CV7A_MAX_BUF 4200
char kobj_log_buf[CV7A_MAX_BUF] = {0,};

static ssize_t read_delta(struct file *filp,
	struct kobject *kobj, struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	int ret = 0;
	int ret2 = 0;
	int16_t *delta = NULL;
	int i = 0;
	int j = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		delta = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (delta == NULL) {
			TOUCH_E("delta mem_error\n");
			return ret;
		}

		ret = snprintf(kobj_log_buf, sizeof(kobj_log_buf), "======== Deltadata ========\n");

		ret2 = get_data(ts->dev, delta, CMD_DELTADATA);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "[%2d]  ", i);
			ret3 += snprintf(log_buf + ret3, sizeof(log_buf) - ret3, "[%2d]  ", i);

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret,
						"%5d ", delta[i * COL_SIZE + j]);
				ret3 += snprintf(log_buf + ret3, sizeof(log_buf) - ret3,
						"%5d ", delta[i * COL_SIZE + j]);
			}
			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[CV7A_MAX_BUF-1] = '\n';
	}

	if (off + count > CV7A_MAX_BUF) {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
	} else {
		memcpy(buf,&kobj_log_buf[off], count);
		ret = count;
	}

error:
	if (delta != NULL)
		kfree(delta);

	return ret;
}

static ssize_t read_rawdata(struct file *filp,
	struct kobject *kobj, struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	int ret = 0;
	int ret2 = 0;
	int16_t *rawdata = NULL;
	int i = 0;
	int j = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		rawdata = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (rawdata == NULL) {
			TOUCH_E("rawdata mem_error\n");
			return ret;
		}

		ret = snprintf(kobj_log_buf, sizeof(kobj_log_buf), "======== rawdata ========\n");

		ret2 = get_data(ts->dev, rawdata, CMD_RAWDATA);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "[%2d]  ", i);
			ret3 += snprintf(log_buf + ret3, sizeof(log_buf) - ret3, "[%2d]  ", i);

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret,
						"%5d ", rawdata[i * COL_SIZE + j]);
				ret3 += snprintf(log_buf + ret3, sizeof(log_buf) - ret3,
						"%5d ", rawdata[i * COL_SIZE + j]);
			}
			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[CV7A_MAX_BUF-1] = '\n';
	}

	if (off + count > CV7A_MAX_BUF) {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
	} else {
		memcpy(buf,&kobj_log_buf[off], count);
		ret = count;
	}

error:
	if (rawdata != NULL)
		kfree(rawdata);

	return ret;
}

static ssize_t read_base(struct file *filp,
	struct kobject *kobj, struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	int ret = 0;
	int ret2 = 0;
	int16_t *baseline = NULL;
	int i = 0;
	int j = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		baseline = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (baseline == NULL) {
			TOUCH_E("baseline mem_error\n");
			return ret;
		}

		ret = snprintf(kobj_log_buf, sizeof(kobj_log_buf), "======== baseline ========\n");

		ret2 = get_data(ts->dev, baseline, CMD_BASE_DATA);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "[%2d]  ", i);
			ret3 += snprintf(log_buf + ret3, sizeof(log_buf) - ret3, "[%2d]  ", i);

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret,
						"%5d ", baseline[i * COL_SIZE + j]);
				ret3 += snprintf(log_buf + ret3, sizeof(log_buf) - ret3,
						"%5d ", baseline[i * COL_SIZE + j]);
			}
			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[CV7A_MAX_BUF-1] = '\n';
	}

	if (off + count > CV7A_MAX_BUF) {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
	} else {
		memcpy(buf,&kobj_log_buf[off], count);
		ret = count;
	}

error:
	if (baseline != NULL)
		kfree(baseline);

	return ret;
}

static ssize_t read_debug(struct file *filp,
	struct kobject *kobj, struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	int ret = 0;
	int ret2 = 0;
	int16_t *debugdata = NULL;
	int i = 0;
	int j = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		debugdata = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (debugdata == NULL) {
			TOUCH_E("debugdata mem_error\n");
			return ret;
		}

		ret = snprintf(kobj_log_buf, sizeof(kobj_log_buf), "======== debugdata ========\n");

		ret2 = get_data(ts->dev, debugdata, CMD_DEBUGDATA);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "[%2d]  ", i);
			ret3 += snprintf(log_buf + ret3, sizeof(log_buf) - ret3, "[%2d]  ", i);

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret,
						"%5d ", debugdata[i * COL_SIZE + j]);
				ret3 += snprintf(log_buf + ret3, sizeof(log_buf) - ret3,
						"%5d ", debugdata[i * COL_SIZE + j]);
			}
			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[CV7A_MAX_BUF-1] = '\n';
	}

	if (off + count > CV7A_MAX_BUF) {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
	} else {
		memcpy(buf,&kobj_log_buf[off], count);
		ret = count;
	}

error:
	if (debugdata != NULL)
		kfree(debugdata);

	return ret;
}

static ssize_t read_labeldata(struct file *filp,
	struct kobject *kobj, struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	int ret = 0;
	int ret2 = 0;
	int16_t *label = NULL;
	int i = 0;
	int j = 0;

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		label = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

		if (label == NULL) {
			TOUCH_E("label mem_error\n");
			return ret;
		}

		ret = snprintf(kobj_log_buf, sizeof(kobj_log_buf), "======== labeldata ========\n");

		ret2 = get_data(ts->dev, label , CMD_LABELDATA);
		if (ret2 < 0) {
			TOUCH_E("Test fail (Check if LCD is OFF)\n");
			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "Test fail (Check if LCD is OFF)\n");
			goto error;
		}

		for (i = 0 ; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0,};
			int ret3 = 0;

			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "[%2d]  ", i);
			ret3 += snprintf(log_buf + ret3, sizeof(log_buf) - ret3, "[%2d]  ", i);

			for (j = 0 ; j < COL_SIZE ; j++) {
				ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret,
						"%5d ", label[i * COL_SIZE + j]);
				ret3 += snprintf(log_buf + ret3, sizeof(log_buf) - ret3,
						"%5d ", label[i * COL_SIZE + j]);
			}
			ret += snprintf(kobj_log_buf + ret, sizeof(kobj_log_buf) - ret, "\n");
			TOUCH_I("%s\n", log_buf);
		}
		kobj_log_buf[CV7A_MAX_BUF-1] = '\n';
	}

	if (off + count > CV7A_MAX_BUF) {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
	} else {
		memcpy(buf,&kobj_log_buf[off], count);
		ret = count;
	}

error:
	if (label!= NULL)
		kfree(label);

	return ret;
}

static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(fdata, show_fdata, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);

/* [Start] for siw app */
static TOUCH_ATTR(prd_app_raw, prd_show_app_raw, NULL);
static TOUCH_ATTR(prd_app_base, prd_show_app_base, NULL);
static TOUCH_ATTR(prd_app_label, prd_show_app_label, NULL);
static TOUCH_ATTR(prd_app_delta, prd_show_app_delta, NULL);
static TOUCH_ATTR(prd_app_debug_buf, prd_show_app_debug_buf, NULL);
static TOUCH_ATTR(prd_app_end, prd_show_app_end, NULL);
static TOUCH_ATTR(prd_app_info, prd_show_app_info, NULL);
/* [End] for siw app */

#define TOUCH_BIN_ATTR(_name, _read, _write, _size)		\
			struct bin_attribute touch_bin_attr_##_name	\
			= __BIN_ATTR(_name, S_IRUGO | S_IWUSR, _read, _write, _size)

static TOUCH_BIN_ATTR(delta, read_delta, NULL, CV7A_MAX_BUF);
static TOUCH_BIN_ATTR(rawdata, read_rawdata, NULL, CV7A_MAX_BUF);
static TOUCH_BIN_ATTR(base, read_base, NULL, CV7A_MAX_BUF);
static TOUCH_BIN_ATTR(debug, read_debug, NULL, CV7A_MAX_BUF);
static TOUCH_BIN_ATTR(label, read_labeldata, NULL, CV7A_MAX_BUF);

static struct attribute *prd_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_fdata.attr,
	&touch_attr_lpwg_sd.attr,

	/* [Start] for siw app */
	&touch_attr_prd_app_raw.attr,
	&touch_attr_prd_app_base.attr,
	&touch_attr_prd_app_label.attr,
	&touch_attr_prd_app_delta.attr,
	&touch_attr_prd_app_debug_buf.attr,
	&touch_attr_prd_app_end.attr,
	&touch_attr_prd_app_info.attr,
	/* [End] for siw app */
	NULL,
};

static struct bin_attribute *prd_attribute_bin_list[] = {
	&touch_bin_attr_delta,
	&touch_bin_attr_rawdata,
	&touch_bin_attr_base,
	&touch_bin_attr_debug,
	&touch_bin_attr_label,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
	.bin_attrs = prd_attribute_bin_list,
};

int sw49107_prd_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49107_data *d = to_sw49107_data(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)d->prd;
	int ret = 0;

	TOUCH_TRACE();

	/* [Start] for siw app */
	prd = siw_hal_prd_alloc(dev);
	if (!prd) {
		ret = -ENOMEM;
		return ret;
	}
	/* [End] for siw app */

	//prd parameter settings
	prd_param_set(dev);

	ret = sysfs_create_group(&ts->kobj, &prd_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	return ret;
}
