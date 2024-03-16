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
#define TS_MODULE "[prd]"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_ft8756.h"
#include "touch_ft8756_prd.h"

//static u16 LowerImage[MAX_ROW][MAX_COL];
//static u16 UpperImage[MAX_ROW][MAX_COL];

static s16 fts_data[MAX_ROW][MAX_COL];
static u8 i2c_data[MAX_COL*MAX_ROW*2];
static u8 log_buf[LOG_BUF_SIZE];		/* !!!!!!!!!Should not exceed the log size !!!!!!!! */
static char print_buf[PRINT_BUF_SIZE];
static u8 fail_log_buf[FAIL_LOG_BUF_SIZE];

static char *prd_str[THE_NUMBER_OF_TEST] = {
	"RAW_DATA_TEST!",
	"CB_DATA_TEST!",
	"OPEN_CB_TEST!",
	"SHORT_ADC_TEST!",
	"NOISE_TEST!",
	"JITTER_TEST!",
	"DELTA_SHOW",
	"LPWG_RAW_DATA_TEST",
	"LPWG_CB_DATA_TEST",
	"LPWG_NOISE_TEST",
	"LPWG_JITTER_TEST",
};

#if 0
static int spec_get_limit(struct device *dev, char *breakpoint,
		u16 limit_data[MAX_ROW][MAX_COL], const struct firmware *fwlimit);
#endif

void print_sd_log(char *buf)
{
	int i = 0;
	int index = 0;

	TOUCH_I("%s : start", __func__);

	while (index < strlen(buf) && buf[index] != '\0' && index < LOG_BUF_SIZE - 1 && i < PRINT_BUF_SIZE - 1) {
		print_buf[i] = buf[index];
		i++;

		/* Final character is not '\n' */
		if ((index == strlen(buf) - 1 || i == PRINT_BUF_SIZE - 2)
				&& print_buf[i - 1] != '\n') {
			print_buf[i] = '\n';
			i++;
		}

		if (print_buf[i - 1] == '\n') {
			print_buf[i - 1] = '\0';
			if (i - 1 != 0)
				TOUCH_I("%s\n", print_buf);

			i = 0;
		}
		index++;
	}
	TOUCH_I("%s : end", __func__);
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
	int boot_mode = TOUCH_NORMAL_BOOT;

	TOUCH_TRACE();

	set_fs(KERNEL_DS);

	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/data/vendor/touch/touch_self_test.txt";
		break;
	case TOUCH_MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
	case TOUCH_MINIOS_MFTS_DS_FLAT:
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
					touch_snprintf(buf2, sizeof(buf2), "%s.%d", fname, (i + 1));
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
}

static void write_file(struct device *dev, char *data, int write_time)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time = {0, };
	struct tm my_date = {0, };
	mm_segment_t old_fs = get_fs();
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_check_boot_mode(dev);

	TOUCH_TRACE();

	switch (boot_mode) {
		case TOUCH_NORMAL_BOOT:
			fname = "/data/vendor/touch/touch_self_test.txt";
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
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec,
					sys_tz.tz_minuteswest * 60 * (-1),
					&my_date);
			touch_snprintf(time_string, 64,
				"\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
				my_date.tm_mon + 1,
				my_date.tm_mday, my_date.tm_hour,
				my_date.tm_min, my_date.tm_sec,
				(unsigned long) my_time.tv_nsec / 1000000);
			ksys_write(fd, time_string, strlen(time_string));
		}
		ksys_write(fd, data, strlen(data));
		print_sd_log(data);
		ksys_close(fd);
	} else {
		TOUCH_E("File open failed\n");
	}
	set_fs(old_fs);
}

#if 0
static int spec_file_read_get_limit(struct device *dev, char *breakpoint, u16 limit_data[MAX_ROW][MAX_COL])
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fwlimit = NULL;
	const char *path[2] = {ts->panel_spec, ts->panel_spec_mfts};
	int boot_mode = 0;
	int path_idx = 0;

	TOUCH_TRACE();

	boot_mode = touch_check_boot_mode(dev);
	if ((boot_mode == TOUCH_MINIOS_MFTS_FOLDER)
			|| (boot_mode == TOUCH_MINIOS_MFTS_FLAT)
			|| (boot_mode == TOUCH_MINIOS_MFTS_CURVED))
		path_idx = 1;
	else
		path_idx = 0;

	if (ts->panel_spec == NULL || ts->panel_spec_mfts == NULL) {
		TOUCH_E("dual_panel_spec file name is null\n");
		ret = -ENOENT;
		goto error;
	}

	TOUCH_I("touch_panel_spec file path = %s\n", path[path_idx]);

	ret = request_firmware(&fwlimit, path[path_idx], dev);

	if (ret) {
		TOUCH_E("%s : request firmware failed(%d)\n", __func__, ret);
		goto error;
	}

	if (fwlimit->data == NULL) {
		TOUCH_E("fwlimit->data is NULL\n");
		ret = -EINVAL;
		goto error;
	}

	if (fwlimit->size == 0) {
		TOUCH_E("fwlimit->size is 0\n");
		ret = -EINVAL;
		goto error;
	}

	ret = spec_get_limit(dev, breakpoint, limit_data, fwlimit);
	if (ret < 0) {
		TOUCH_E("spec_get_limit fail\n");
		ret = -EINVAL;
		goto error;
	}

	TOUCH_I("spec_file_read_get_limit success\n");

error:
	if (fwlimit)
		release_firmware(fwlimit);

	return ret;
}

static int spec_get_limit(struct device *dev, char *breakpoint,
		u16 limit_data[MAX_ROW][MAX_COL], const struct firmware *fwlimit)
{
	int p = 0;
	int q = 0;
	int r = 0;
	int cipher = 1;
	int ret = 0;
	char *found;
	int tx_num = 0;
	int rx_num = 0;
	const u8 *fw_data = fwlimit->data;

	TOUCH_TRACE();

	if (breakpoint == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	found = strnstr(fw_data, breakpoint, fwlimit->size);
	if (found != NULL) {
		q = (u8 *)found - fw_data;
	} else {
		TOUCH_E("failed to find breakpoint. The panel_spec_file is wrong\n");
		ret = -EAGAIN;
		goto error;
	}

	memset(limit_data, 0, MAX_ROW * MAX_COL * 2);

	while (1) {
		if (fw_data[q] == ',') {
			cipher = 1;
			for (p = 1; (fw_data[q - p] >= '0') &&
					(fw_data[q - p] <= '9'); p++) {
				limit_data[tx_num][rx_num] += ((fw_data[q - p] - '0') * cipher);
				cipher *= 10;
			}
			r++;
			if (r % (int)MAX_COL == 0) {
				rx_num = 0;
				tx_num++;
			} else {
				rx_num++;
			}
		}
		q++;
		if (r == (int)MAX_ROW * (int)MAX_COL) {
			TOUCH_I("[%s] panel_spec_file scanning is success\n", breakpoint);
			break;
		}
	}

error:
	return ret;
}
#endif

int ft8756_change_op_mode(struct device *dev, u8 op_mode)
{
	int i = 0;
	u8 ret;
	u8 data;

	TOUCH_I("%s : op_mode = 0x%02x\n", __func__, op_mode);

	data = 0x00;
	ret = ft8756_reg_read(dev, 0x00, &data, 1);
	if(ret < 0) {
		TOUCH_E("0x00 register read error\n");
		return ret;
	}

	if (data == op_mode) {
		TOUCH_I("Already mode changed\n");
		return 0;
	}

	data = op_mode;
	ret = ft8756_reg_write(dev, 0x00, &data, 1);
	if(ret < 0) {
		TOUCH_E("0x00 register op_mode write error\n");
		return ret;
	}

	touch_msleep(10);

	for ( i = 0; i < FTS_MODE_CHANGE_LOOP; i++) {
		data = 0x00;
		ret = ft8756_reg_read(dev, 0x00, &data, 1);
		if(ret < 0) {
			TOUCH_E("op_mode change check error\n");
			return ret;
		}

		if(data == op_mode)
			break;
		touch_msleep(50);
	}

	if (i >= FTS_MODE_CHANGE_LOOP) {
		TOUCH_E("Timeout to change op mode\n");
		return -EPERM;
	}

	touch_msleep(100);

	TOUCH_I("Operation mode changed\n");

	return 0;
}

int ft8756_switch_cal(struct device *dev, u8 cal_en)
{
	u8 ret;
	u8 data;

	TOUCH_I("%s : cal_en = 0x%02x\n", __func__, cal_en);

	ret = ft8756_reg_read(dev, 0xEE, &data, 1);
	if(ret < 0) {
		TOUCH_E("0xC2 register(calibration) read error\n");
		return ret;
	}

	if (data == cal_en) {
		TOUCH_I("Already switch_cal changed\n");
		return 0;
	}

	data = cal_en;
	ret = ft8756_reg_write(dev, 0xEE, &data, 1);
	if(ret < 0) {
		TOUCH_E("0xC2 register(calibration) write error\n");
		return ret;
	}

	touch_msleep(100);

	return 0;
}


int ft8756_prd_check_ch_num(struct device *dev)
{
	u8 ret;
	u8 data;

	TOUCH_I("%s\n", __func__);

	/* Channel number check */
	ret = ft8756_reg_read(dev, 0x02, &data, 1);
	if(ret < 0) {
		TOUCH_E("tx number register read error\n");
		return ret;
	}
	TOUCH_I("TX Channel : %d\n", data);

	touch_msleep(3);

	if (data != MAX_COL) {
		TOUCH_E("Invalid TX Channel Num.\n");
		return -EPERM;
	}

	ret = ft8756_reg_read(dev, 0x03, &data, 1);
	if(ret < 0) {
		TOUCH_E("rx number register read error\n");
		return ret;
	}

	TOUCH_I("RX Channel : %d\n", data);

	touch_msleep(3);

	if (data != MAX_ROW) {
		TOUCH_E("Invalid RX Channel Num.\n");
		return -EPERM;
	}

	return 0;
}

int ft8756_prd_get_raw_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;
	u8 rawdata = 0x00;
	bool bscan = false;

	TOUCH_TRACE();

	memset(i2c_data, 0, sizeof(i2c_data));

	// Data Select to raw data
	data = 0x00;
	ret = ft8756_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("data select to rawdata error\n");
		return ret;
	}
	touch_msleep(10);

	data = 0x01;
	ret = ft8756_reg_write(dev, 0x9E, &data, 1);  // FACTORY_REG_RAWDATA_TEST_EN ENABLE
	if (ret < 0) {
		TOUCH_E("FACTORY_REG_RAWDATA_TEST_EN error\n");
		return ret;
	}

	/* Start SCAN */
	for (k = 0; k < 3; k++)
	{
		TOUCH_I("Start SCAN (%d/3)\n", k+1);
		memset(i2c_data, 0, sizeof(i2c_data));
		ret = ft8756_reg_read(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("start scan read error\n");
			return ret;
		}
		data |= 0x80; // 0x40|0x80
		ret = ft8756_reg_write(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("0x00 register 0x80 write error\n");
			return ret;
		}

		touch_msleep(20);

		for (i = 0; i < 5; i++) {
			ret = ft8756_reg_read(dev, 0x00, &data, 1);
			if (ret < 0) {
				TOUCH_E("0x00 register data read error\n");
				return ret;
			}
			if (data == 0x40) {
				TOUCH_I("SCAN Success : 0x%X, %d ms \n", data, i*20);
				bscan = true;
				break;
			} else {
				bscan = false;
				touch_msleep(20);
			}
		}

		if(bscan==true){
			/* Read Raw data */
			rawdata = 0xAD;
			ret = ft8756_reg_write(dev, 0x01, &rawdata, 1);
			if(ret < 0) {
				TOUCH_E("read raw data error\n");
				return ret;
			}
			touch_msleep(10);

			TOUCH_I("Read Raw data at once\n");

			ret = ft8756_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);
			if(ret < 0) {
				TOUCH_E("0x6A register read error\n");
				return ret;
			}
		} else {
			TOUCH_E("SCAN Fail (%d/3)\n", k+1);
		}
	}

	data = 0x00;
	ret = ft8756_reg_write(dev, 0x9E, &data, 1);  // FACTORY_REG_RAWDATA_TEST_EN DISABLE
	if (ret < 0) {
		TOUCH_E("FACTORY_REG_RAWDATA_TEST_EN error\n");
		return ret;
	}

	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			k = (j * MAX_ROW + i) << 1;
			fts_data[i][j] = (i2c_data[k] << 8) + i2c_data[k+1];
		}
	}
	return 0;
}


int ft8756_prd_get_noise_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;
//	u8 frame_count;

	TOUCH_TRACE();

	memset(i2c_data, 0, sizeof(i2c_data));

	// Data Select 1: diff 0: rawdata
	data = 0x01;
	ret = ft8756_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("0x06 register 0x01(data select:1(diff)) write error\n");
		return ret;
	}
	touch_msleep(10);

	// Set Noise Test Frame Count
	//low  byte
	data = 0x0D; //0x0D = 13, 13*4 = 52 frame
	ret = ft8756_reg_write(dev, 0x12, &data, 1);
	if (ret < 0) {
		TOUCH_E("set noise test frame count(low byte) error\n");
		return ret;
	}
	touch_msleep(10);

#if 0 //FT8756 Not used. delete
	//high  byte
	data = 0x00;
	ret = ft8756_reg_write(dev, 0x13, &data, 1);
	if (ret < 0) {
		TOUCH_E("set noise test frame count(high byte) error\n");
		return ret;
	}
	touch_msleep(10);
#endif

// Start Noise Test
	data = 0x01;
	ret = ft8756_reg_write(dev, 0x11, &data, 1);  // NOISE : W(0X11,0X01)
	if (ret < 0) {
		TOUCH_E("start noise test error\n");
		return ret;
	}
	touch_msleep(100);

	// Clear Raw Data Addr
	data = 0xAD;
	ret = ft8756_reg_write(dev, 0x01, &data, 1);
	if (ret < 0) {
		TOUCH_E("clear raw data addr error\n");
		return ret;
	}
	touch_msleep(100);

	// Check Scan is finished
	for (i = 0; i < 100; i++) {
		ret = ft8756_reg_read(dev, 0x13, &data, 1); // FT8756 0X13, Read value 0xAA : Finish
		if (ret < 0) {
			TOUCH_E("check scan error\n");
			return ret;
		}
		if (data == 0xAA) {
			TOUCH_I("Scan finished : %d ms, data = %x\n", i*50 ,data);
			break;
		}
		touch_msleep(50);
	}

	if (i >= 100) {
		TOUCH_E("Scan failed\n");
		return -EPERM;
	}

	// Get Noise data
	TOUCH_I("Read Noise data at once\n");

	// (Get RMS data)->(Get MaxNoise Data)
	ret = ft8756_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("read noise data error\n");
		return ret;
	}
	touch_msleep(100);

	data = 0x00;
	ret = ft8756_reg_write(dev, 0x11, &data, 1);
	if (ret < 0) {
		TOUCH_E("set to initial value fail\n");
		return ret;
	}

	data = 0x03;
	ret = ft8756_reg_write(dev, 0x13, &data, 1);
	if (ret < 0) {
		TOUCH_E("set idle to test statue register fail\n");
		return ret;
	}

	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			k = (j * MAX_ROW + i) << 1;
			fts_data[i][j] = abs((i2c_data[k] << 8) + i2c_data[k+1]);
		}
	}

	return 0;
}

int ft8756_prd_get_jitter_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;
//	u8 frame_count;

	TOUCH_TRACE();

	memset(i2c_data, 0, sizeof(i2c_data));

	// Data Select 1: diff 0: rawdata
	data = 0x01;
	ret = ft8756_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("0x06 register 0x01(data select:1(diff)) write error\n");
		return ret;
	}
	touch_msleep(10);

	// Set Jitter Test Frame Count
	//low  byte
	data = 0x32;
	ret = ft8756_reg_write(dev, 0x12, &data, 1);
	if (ret < 0) {
		TOUCH_E("set jitter test frame count(low byte) error\n");
		return ret;
	}
	touch_msleep(10);

#if 0 // FT8756 Not used. delete
	//high  byte
	data = 0x00;
	ret = ft8756_reg_write(dev, 0x13, &data, 1);
	if (ret < 0) {
		TOUCH_E("set jitter test frame count(high byte) error\n");
		return ret;
	}
	touch_msleep(10);
#endif

	// Start Jitter Test
	data = 0x02;
	ret = ft8756_reg_write(dev, 0x11, &data, 1);   // JITTER : W(0X11,0X02)
	if (ret < 0) {
		TOUCH_E("start jitter test error\n");
		return ret;
	}
	touch_msleep(100);

	// Clear Raw Data Addr
	data = 0xAD;
	ret = ft8756_reg_write(dev, 0x01, &data, 1);
	if (ret < 0) {
		TOUCH_E("clear raw data addr error\n");
		return ret;
	}
	touch_msleep(100);

	// Check Scan is finished
	for (i = 0; i < 100; i++) {
		ret = ft8756_reg_read(dev, 0x13, &data, 1); // FT8756 0X13, Read value 0xAA : Finish
		if (ret < 0) {
			TOUCH_E("check scan error\n");
			return ret;
		}
		if (data == 0xAA) {
			TOUCH_I("Scan finished : %d ms, data = %x\n", i*50 ,data);
			break;
		}
		touch_msleep(50);
	}

	if (i >= 100) {
		TOUCH_E("Scan failed\n");
		return -EPERM;
	}

	// Get Jitter data
	TOUCH_I("Read Jitter data at once\n");

	// (Get RMS data)->(Get MaxJitter Data)
	ret = ft8756_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);

	if(ret < 0) {
		TOUCH_E("read jitter data error\n");
		return ret;
	}
	touch_msleep(100);

	data = 0x00;
	ret = ft8756_reg_write(dev, 0x11, &data, 1);
	if (ret < 0) {
		TOUCH_E("set to initial value fail\n");
		return ret;
	}

	data = 0x03;
	ret = ft8756_reg_write(dev, 0x13, &data, 1);
	if (ret < 0) {
		TOUCH_E("set idle to test statue register fail\n");
		return ret;
	}

	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			k = (j * MAX_ROW + i) << 1;
			fts_data[i][j] = abs((i2c_data[k] << 8) + i2c_data[k+1]);
		}
	}

	return 0;
}


int ft8756_prd_get_cb_data(struct device *dev)
{
	int i, j;
	int ret = 0;
	u8 data = 0x00;
	int total, offset = 0, read_len;

	TOUCH_TRACE();

	memset(i2c_data, 0, sizeof(i2c_data));

	data = 0x01;
	ret = ft8756_reg_write(dev, 0x9F, &data, 1);  // FACTORY_REG_CB_TEST_EN ENABLE
	if (ret < 0) {
		TOUCH_E("FACTORY_REG_CB_TEST_EN error\n");
		return ret;
	}

    /* auto clb */
	data = 0x04;
	ret = ft8756_reg_write(dev, FACTORY_REG_CLB, &data, 1);  // Auto calibration
	if (ret < 0) {
		TOUCH_E("FACTORY_REG_CB_TEST_EN error\n");
		return ret;
	}
	for (i = 0; i < 200; i++) {
		ret = ft8756_reg_read(dev, FACTORY_REG_CLB, &data, 1);
		if (ret < 0) {
			TOUCH_E("0x04 register data read error\n");
			return ret;
		}
		if (data == 0x02) {
			TOUCH_I("Calibration ok\n");
			break;
		}
		touch_msleep(50);
	}

	total = MAX_COL*MAX_ROW;

	// Get CB data
	for (i = 0; (total - TEST_PACKET_LENGTH*i) > 0; i++)
	{
		offset = TEST_PACKET_LENGTH * i;
		read_len = ((total - offset) >= TEST_PACKET_LENGTH) ? TEST_PACKET_LENGTH : (total - offset);

		data = (offset & 0xFF00) >> 8;
		ret = ft8756_reg_write(dev, 0x18, &data, 1);
		if (ret < 0) {
			TOUCH_E("0x18 register data write error\n");
			return ret;
		}
		touch_msleep(10);

		data = (offset & 0x00FF);
		ret = ft8756_reg_write(dev, 0x19, &data, 1);
		if (ret < 0) {
			TOUCH_E("0x19 register data write error\n");
			return ret;
		}
		touch_msleep(10);

		ret = ft8756_reg_read(dev, 0x6E, &i2c_data[offset], read_len);
		if (ret < 0) {
			TOUCH_E("0x6E register i2c_data read error\n");
			return ret;
		}

		touch_msleep(10);
	}

	data = 0x00;
	ret = ft8756_reg_write(dev, 0x9F, &data, 1);  // FACTORY_REG_CB_TEST_EN DISABLE
	if (ret < 0) {
		TOUCH_E("FACTORY_REG_CB_TEST_EN error\n");
		return ret;
	}

	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			fts_data[i][j] = i2c_data[j * MAX_ROW + i];
		}
	}

	return 0;
}

int ft8756_prd_get_delta_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;

	TOUCH_TRACE();

	memset(i2c_data, 0, sizeof(i2c_data));

	// Data Select to diff data
	data = 0x01;
	ret = ft8756_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("data select to delta error\n");
		return ret;
	}
	touch_msleep(10);

	/* Start SCAN */
	for (k = 0; k < 3; k++)
	{
		TOUCH_I("Start SCAN (%d/3)\n", k+1);
		ret = ft8756_reg_read(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("start scan read error\n");
			return ret;
		}
		data |= 0x80; // 0x40|0x80
		ret = ft8756_reg_write(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("0x00 register 0x80 write error\n");
			return ret;
		}

		touch_msleep(10);

		for (i = 0; i < 200; i++) {
			ret = ft8756_reg_read(dev, 0x00, &data, 1);
			if (ret < 0) {
				TOUCH_E("0x00 register data read error\n");
				return ret;
			}
			if (data == 0x40) {
				TOUCH_I("SCAN Success : 0x%X, %d ms \n", data, i*20);
				break;
			}
			touch_msleep(20);
		}

		if (i < 200) {
			break;
		}

		TOUCH_E("SCAN Fail (%d/3)\n", k+1);
	}

	if (k >= 3) {
		return -EPERM;
	}

	/* Read Raw data */
	data = 0xAD;
	ret = ft8756_reg_write(dev, 0x01, &data, 1);
	if(ret < 0) {
		TOUCH_E("read raw data error\n");
		return ret;
	}
	touch_msleep(10);

	TOUCH_I("Read Delta at once\n");

	ret = ft8756_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("0x6A register read error\n");
		return ret;
	}

	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			k = (j * MAX_ROW + i) << 1;
			fts_data[i][j] = (i2c_data[k] << 8) + i2c_data[k+1];
		}
	}

	return 0;
}

int ft8756_prd_get_short_data(struct device *dev)
{
	u8 data = 0x00;
	int ret = 0;
	int i = 0, j = 0, k = 0;
	int tmp_adc = 0;

	TOUCH_TRACE();

	memset(i2c_data, 0, sizeof(i2c_data));

	// Short test command
	data = 0x01;
	ret = ft8756_reg_write(dev, 0x0F, &data, 1);
	if (ret < 0) {
		TOUCH_E("0x0F register write error\n");
		return ret;
	}

	for ( i = 0; i < 100; i++) {
		data = 0x00;
		ret = ft8756_reg_read(dev, 0x10, &data, 1);
		if(ret < 0) {
			TOUCH_E("0x10 register read error\n");
			return ret;
		}

		if(data == 0xAA) {
			TOUCH_I("%s: Success! 0x%X, %d ms \n", __func__, data, i*100);
			break;
		}
		touch_msleep(50);
	}

	ret = ft8756_reg_read(dev, 0x89, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("0x89 register read error\n");
		return ret;
	}

	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			k = (j * MAX_ROW + i) << 1;
			fts_data[i][j] = (int)(short)((i2c_data[k] << 8) + i2c_data[k+1]);
		}
	}

	/* calculate resistor */
	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
	        tmp_adc = fts_data[i][j];
	        if (tmp_adc > 3500) {
	            tmp_adc = 3500;
	        }
	        fts_data[i][j] = (5160960 + 555 * 32 * tmp_adc) / (1146880 - 320 * tmp_adc);
		}
	}

	/* short test end */
	data = 0x03;
	ret = ft8756_reg_write(dev, 0x10, &data, 1);
	if (ret < 0) {
		TOUCH_E("write idle to short test state fail\n");
		return ret;
	}

	return 0;
}

int ft8756_prd_get_open_data_cb(struct device *dev)
{
	int i, j;
	int ret = 0;
	u8 data = 0x00;
	int total, offset = 0, read_len;

	TOUCH_TRACE();

	memset(i2c_data, 0, sizeof(i2c_data));

	total = MAX_COL*MAX_ROW;

	// Get CB data
	for (i = 0; (total - TEST_PACKET_LENGTH*i) > 0; i++)
	{
		offset = TEST_PACKET_LENGTH * i;
		read_len = ((total - offset) >= TEST_PACKET_LENGTH) ? TEST_PACKET_LENGTH : (total - offset);

		data = (offset & 0xFF00) >> 8;
		ret = ft8756_reg_write(dev, 0x18, &data, 1);
		if (ret < 0) {
			TOUCH_E("0x18 register data write error\n");
			return ret;
		}
		touch_msleep(10);

		data = (offset & 0x00FF);
		ret = ft8756_reg_write(dev, 0x19, &data, 1);
		if (ret < 0) {
			TOUCH_E("0x19 register data write error\n");
			return ret;
		}
		touch_msleep(10);

		ret = ft8756_reg_read(dev, 0x6E, &i2c_data[offset], read_len);
		if (ret < 0) {
			TOUCH_E("0x6E register i2c_data read error\n");
			return ret;
		}

		touch_msleep(10);
	}

	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			fts_data[i][j] = i2c_data[j * MAX_ROW + i];
		}
	}

	return ret;
}

int ft8756_prd_get_open_data(struct device *dev)
{
	u8 data = 0x00;
	int ret = 0;
	int i = 0/*, j = 0, k = 0*/;
	u8 reg_k1 = 0, reg_k2 = 0;

	TOUCH_TRACE();

	memset(i2c_data, 0, sizeof(i2c_data));

	ret = ft8756_reg_read(dev, FACTORY_REG_K1, &reg_k1, 1);
	if(ret < 0) {
		TOUCH_E("FACTORY_REG_K1 read error\n");
		return ret;
	}

	ret = ft8756_reg_read(dev, FACTORY_REG_K2, &reg_k2, 1);
	if(ret < 0) {
		TOUCH_E("FACTORY_REG_K2 read error\n");
		return ret;
	}

	data = OPENTEST_K1THRESHOLD;
	ret = ft8756_reg_write(dev, FACTORY_REG_K1, &data, 1);
	if (ret < 0) {
		TOUCH_E("FACTORY_REG_K1 write error\n");
		return ret;
	}

	data = OPENTEST_K2THRESHOLD;
	ret = ft8756_reg_write(dev, FACTORY_REG_K2, &data, 1);
	if (ret < 0) {
		TOUCH_E("FACTORY_REG_K2 write error\n");
		return ret;
	}

	// Short test command
	data = 0x01;
	ret = ft8756_reg_write(dev, 0x15, &data, 1);
	if (ret < 0) {
		TOUCH_E("0x15 write error\n");
		return ret;
	}

	for ( i = 0; i < 100; i++) {
		data = 0x00;
		ret = ft8756_reg_read(dev, 0x16, &data, 1);
		if(ret < 0) {
			TOUCH_E("0x16 read error\n");
			return ret;
		}

		if(data == 0xAA) {
			TOUCH_I("%s: Success! 0x%X, %d ms \n", __func__, data, i*100);
			break;
		}
		touch_msleep(50);
	}

	/* get open CB data */
	ret = ft8756_prd_get_open_data_cb(dev);
	if (ret < 0) {
		TOUCH_E("ft8756_prd_get_open_data_cb fail\n");
		return ret;
	}

    /* Restore reg */
	ret = ft8756_reg_write(dev, FACTORY_REG_K1, &reg_k1, 1);
	if (ret < 0) {
		TOUCH_E("FACTORY_REG_K1 restore error\n");
		return ret;
	}

	ret = ft8756_reg_write(dev, FACTORY_REG_K2, &reg_k2, 1);
	if (ret < 0) {
		TOUCH_E("FACTORY_REG_K2 restore error\n");
		return ret;
	}

    /* auto clb */
	data = 0x04;
	ret = ft8756_reg_write(dev, FACTORY_REG_CLB, &data, 1);  // Auto calibration
	if (ret < 0) {
		TOUCH_E("FACTORY_REG_CB_TEST_EN error\n");
		return ret;
	}
	for (i = 0; i < 200; i++) {
		ret = ft8756_reg_read(dev, FACTORY_REG_CLB, &data, 1);
		if (ret < 0) {
			TOUCH_E("0x04 register data read error\n");
			return ret;
		}
		if (data == 0x02) {
			TOUCH_I("Calibration ok\n");
			break;
		}
		touch_msleep(50);
	}

	/* open test end */
	data = 0x03;
	ret = ft8756_reg_write(dev, 0x16, &data, 1);
	if (ret < 0) {
		TOUCH_E("write idle to open test state fail\n");
		return ret;
	}

	return 0;
}

int ft8756_prd_test_data(struct device *dev, int test_type, int* test_result)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);

	int i = 0, j = 0;
	int ret = 0;

	int limit_upper = 0, limit_lower = 0;
	int min, max/*, aver, stdev*/;
	int fail_count = 0;
	int check_limit = 1;
	int fail_buf_ret = 0;
	int min_i = 0, min_j = 0;
	int max_i = 0, max_j = 0;

	TOUCH_TRACE();

	*test_result = TEST_FAIL;

	memset(fail_log_buf, 0, sizeof(fail_log_buf));

	ret += touch_snprintf(log_buf, sizeof(log_buf) - ret, "IC F/W Version : V%d.%02d\n", d->ic_info.is_official, d->ic_info.fw_version);

	switch (test_type) {
		case RAW_DATA_TEST:
			ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "============= Raw Data Test Result =============\n");
			limit_upper = RAW_DATA_MAX;
			limit_lower = RAW_DATA_MIN;
//			spec_file_read_get_limit(dev, "LowerImageLimit", LowerImage);
//			spec_file_read_get_limit(dev, "UpperImageLimit", UpperImage);
			break;
		case CB_DATA_TEST:
			ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "============= CB Data Test Result =============\n");
			limit_upper = CB_MAX;
			limit_lower = CB_MIN;
			break;
		case OPEN_CB_TEST:
			ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "============= OPEN Test Result =============\n");
			limit_lower = OPEN_CB_MIN;
			limit_upper = OPEN_CB_MAX;
			break;
		case SHORT_ADC_TEST:
			ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "============= SHORT Test Result =============\n");
			limit_lower = SHORT_RES_MIN;
			limit_upper = SHORT_RES_MAX;
			break;
		case NOISE_TEST:
			ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "============= Noise Test Result =============\n");
			limit_upper = NOISE_MAX;
			limit_lower = NOISE_MIN;
			break;
		case JITTER_TEST:
			ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "============= Jitter Test Result =============\n");
			limit_upper = JITTER_MAX;
			limit_lower = JITTER_MIN;
			break;
		case DELTA_SHOW:
			ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "============= Delta Result =============\n");
			check_limit = 0;
			break;
		case LPWG_RAW_DATA_TEST:
			ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "============= LPWG Raw Data Test Result =============\n");
			limit_upper = LPWG_RAW_DATA_MAX;
			limit_lower = LPWG_RAW_DATA_MIN;
//			spec_file_read_get_limit(dev, "LowerImageLimit", LowerImage);
//			spec_file_read_get_limit(dev, "UpperImageLimit", UpperImage);
			break;
		case LPWG_CB_DATA_TEST:
			ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "============= LPWG CB Data Test Result =============\n");
			limit_upper = LPWG_CB_MAX;
			limit_lower = LPWG_CB_MIN;
			break;
		case LPWG_NOISE_TEST:
			ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "============= LPWG Noise Test Result =============\n");
			limit_upper = LPWG_NOISE_MAX;
			limit_lower = LPWG_NOISE_MIN;
			break;
		case LPWG_JITTER_TEST:
			ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "============= LPWG Jitter Test Result =============\n");
			limit_upper = LPWG_JITTER_MAX;
			limit_lower = LPWG_JITTER_MIN;
			break;
		default:
			ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "Test Failed (Invalid test type)\n");
			return ret;
	}

//	max = min = fts_data[0][0]; //there is no node(0,0) in ft8756.
	max = min = fts_data[0][1];

	for (i = 0; i < MAX_ROW; i++) {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "[%2d] ", i+1);

		for (j = 0; j < MAX_COL; j++) {

			if (test_type == RAW_DATA_TEST || test_type == LPWG_RAW_DATA_TEST) {
				ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "%5d ", fts_data[i][j]);
			}
			else {
				ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "%4d ", fts_data[i][j]);
			}

			if ((i == 0 && j == 0) || (i == 0 && j == MAX_COL - 1)) {
				TOUCH_I("%s, there is no node[%d][%d]. test skip.\n", prd_str[test_type], i, j); //no channel
				continue;
			}

			if (check_limit && (fts_data[i][j] < limit_lower || fts_data[i][j] > limit_upper)) {
				fail_count++;
				TOUCH_I("RT Test : %s, data[%d][%d] = %d\n", prd_str[test_type], i, j, fts_data[i][j]);
				if (fail_count < 5) {
					fail_buf_ret += touch_snprintf(fail_log_buf + fail_buf_ret, FAIL_LOG_BUF_SIZE - fail_buf_ret,
							"Test : %s, data[%d][%d] = %d\n", prd_str[test_type], i, j, fts_data[i][j]);
				}
			}

			if (fts_data[i][j] < min) {
				min_i = i;
				min_j = j;
				min = fts_data[i][j];
			}
			if (fts_data[i][j] > max) {
				max_i = i;
				max_j = j;
				max = fts_data[i][j];
			}
		}
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "\n");
	}

	ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "==================================================\n");

	if(fail_count && check_limit) {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "Test FAIL : %d Errors\n", fail_count);
	}
	else {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "Test PASS : No Errors\n");
		*test_result = TEST_PASS;
	}

	ret += touch_snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "MIN[%d][%d] = %d, MAX[%d][%d] = %d, Upper = %d, Lower = %d\n\n",
			min_i, min_j, min, max_i, max_j, max, limit_upper, limit_lower);
	ret += touch_snprintf(log_buf + ret, LOG_BUF_SIZE - ret, fail_log_buf);

	return ret;
}

static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0;
	int ret_size = 0, ret_total_size = 0;
	int rawdata_ret = TEST_FAIL;
	int cb_ret = TEST_FAIL;
	int noise_ret = TEST_FAIL;
	int open_ret = TEST_FAIL;
	int short_ret = TEST_FAIL;
//	int jitter_ret = TEST_FAIL;

	TOUCH_I("%s\n", __func__);
	// Check Current State
	if(d->state != TC_STATE_ACTIVE) {
		TOUCH_E("Show_sd is called in NOT Active state\n");
		return 0;
	}

	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(300);

	/* file create , time log */
	TOUCH_I("Show_sd Test Start\n");
	write_file(dev, "\nShow_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

	// Change clb switch
	ret = ft8756_switch_cal(dev, 1);
	if (ret < 0) {
		TOUCH_E("change clb switch error\n");
		goto FAIL;
	}

	// Change to factory mode
	ret = ft8756_change_op_mode(dev, FTS_FACTORY_MODE);
	if (ret < 0) {
		TOUCH_E("change to factory mode error\n");
		goto FAIL;
	}

	// Start to raw data test
	TOUCH_I("Show_sd : Raw data test\n");
	memset(log_buf, 0, sizeof(log_buf));

	ret = ft8756_prd_check_ch_num(dev);
	if (ret < 0) {
		TOUCH_E("prd check_ch_num error\n");
		goto FAIL;
	}

	ret = ft8756_prd_get_raw_data(dev);
	if (ret < 0) {
		TOUCH_E("prd get raw data error\n");
		goto FAIL;
	}

	ret_size = ft8756_prd_test_data(dev, RAW_DATA_TEST, &rawdata_ret);

	TOUCH_I("Raw Data Test Result : %d\n", rawdata_ret);
	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);

	// Start to CB data test
	TOUCH_I("Show_sd : CB data test\n");
	memset(log_buf, 0, sizeof(log_buf));

	ret = ft8756_prd_get_cb_data(dev);
	if (ret < 0) {
		TOUCH_E("prd get cb data error\n");
		goto FAIL;
	}

	ret_size = ft8756_prd_test_data(dev, CB_DATA_TEST, &cb_ret);

	TOUCH_I("CB Data Test Result : %d\n", cb_ret);
	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);

	// Start to short adc test
	TOUCH_I("Show_sd : short adc data test\n");
	memset(log_buf, 0, sizeof(log_buf));

	ret = ft8756_prd_get_short_data(dev);
	if (ret < 0) {
		TOUCH_E("prd get short data error\n");
		goto FAIL;
	}

	ret_size = ft8756_prd_test_data(dev, SHORT_ADC_TEST, &short_ret);

	TOUCH_I("short Data Test Result : %d\n", short_ret);
	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);

	// Start to open CB data test
	TOUCH_I("Show_sd : open CB data test\n");
	memset(log_buf, 0, sizeof(log_buf));

	ret = ft8756_prd_get_open_data(dev);
	if (ret < 0) {
		TOUCH_E("prd get open data error\n");
		goto FAIL;
	}

	ret_size = ft8756_prd_test_data(dev, OPEN_CB_TEST, &open_ret);

	TOUCH_I("open CB Data Test Result : %d\n", open_ret);
	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);

	// Start to noise test
	TOUCH_I("Show_sd : Noise test\n");
	memset(log_buf, 0, sizeof(log_buf));

	ret = ft8756_prd_get_noise_data(dev);
	if (ret < 0) {
		TOUCH_E("prd get noise data error\n");
		goto FAIL;
	}

	ret_size = ft8756_prd_test_data(dev, NOISE_TEST, &noise_ret);

	TOUCH_I("Noise Test Result : %d\n", noise_ret);
	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);

#if 0
	// Start to jitter test
	TOUCH_I("Show_sd : Jitter test\n");
	memset(log_buf, 0, sizeof(log_buf));

	ret = ft8756_prd_get_jitter_data(dev);
	if (ret < 0) {
		TOUCH_E("prd get noise data error\n");
		goto FAIL;
	}

	ret_size = ft8756_prd_test_data(dev, JITTER_TEST, &jitter_ret);

	TOUCH_I("Jitter Test Result : %d\n", jitter_ret);
	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);
#endif

	// Change to working mode
	ret = ft8756_change_op_mode(dev, FTS_WORK_MODE);
	if (ret < 0) {
		TOUCH_I("Failed to return WORK_MODE\n");
		goto FAIL;
	}

	// Test result
	ret = touch_snprintf(log_buf, sizeof(log_buf), "\n========RESULT=======\n");
	TOUCH_I("========RESULT=======\n");

	if ((rawdata_ret == TEST_PASS) && (noise_ret == TEST_PASS)) {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "Raw Data : Pass\n");
		TOUCH_I("Raw Data : Pass\n");
	} else {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret,
				"Raw Data : Fail (raw:%d/noise:%d)\n",
				(rawdata_ret == TEST_FAIL)? 0 : 1, (noise_ret == TEST_FAIL)? 0 : 1);
		TOUCH_I("Raw Data : Fail (raw:%d/noise:%d)\n",
				(rawdata_ret == TEST_FAIL)? 0 : 1, (noise_ret == TEST_FAIL)? 0 : 1);
	}

	if (cb_ret == TEST_PASS) {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "Channel Status : Pass\n");
		TOUCH_I("Channel Status : Pass\n");
	} else {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret,
				"Channel Status: Fail (cb:%d/short:%d/open:%d)\n",
				(cb_ret == TEST_FAIL)? 0 : 1, (short_ret == TEST_FAIL)? 0 : 1,
				(open_ret == TEST_FAIL)? 0 : 1);
		TOUCH_I("Channel Status : Fail (cb:%d/short:%d/open:%d)\n",
				(cb_ret == TEST_FAIL)? 0 : 1, (short_ret == TEST_FAIL)? 0 : 1,
				(open_ret == TEST_FAIL)? 0 : 1);
	}

	ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "=====================\n");
	TOUCH_I("=====================\n");

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret);
	ret_total_size += ret;

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_total_size;

FAIL:
	// Change to working mode
	ret = ft8756_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0) {
		TOUCH_I("Failed to return WORK_MODE\n");
	}

	ret = touch_snprintf(log_buf, sizeof(log_buf), "\n========RESULT=======\n");
	TOUCH_I("========RESULT=======\n");

	if ((rawdata_ret == TEST_PASS) && (noise_ret == TEST_PASS)) {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "Raw Data : Pass\n");
		TOUCH_I("Raw Data : Pass\n");
	} else {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "Raw Data : Fail\n");
		TOUCH_I("Raw Data : Fail\n");
	}

	if (cb_ret == TEST_PASS) {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "Channel Status : Pass\n");
		TOUCH_I("Channel Status : Pass\n");
	} else {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "Channel Status: Fail\n"),
		TOUCH_I("Channel Status : Fail\n");
	}

	ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "=====================\n");
	TOUCH_I("=====================\n");

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret);
	ret_total_size += ret;

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_total_size;
}

static ssize_t show_delta_bin(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	int ret = 0;
	static int ret_size = 0;
	int test_result = TEST_FAIL;

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_E("IC state is deep sleep.");
		return 0;
	}

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		TOUCH_I("Show Delta Data\n");
		mutex_lock(&ts->lock);

		memset(log_buf, 0, sizeof(log_buf));

		ft8756_dummy_read(ts->dev);

		// Change clb switch
		ret = ft8756_switch_cal(ts->dev, 0);
		if(ret < 0)
			goto OUT;

		ret = ft8756_change_op_mode(ts->dev, FTS_FACTORY_MODE);
		if(ret < 0)
			goto OUT;

#if 0
		ret = ft8756_prd_check_ch_num(ts->dev);
		if(ret < 0)
			goto OUT;
#endif

		ret = ft8756_prd_get_delta_data(ts->dev);
		if(ret < 0)
			goto OUT;

		ret = ft8756_change_op_mode(ts->dev, FTS_WORK_MODE);
		if(ret < 0)
			goto OUT;

#if 0
		ret = ft8756_switch_cal(dev, 1);
		if(ret < 0)
			goto FAIL;
#endif

		TOUCH_I("Show Delta Data OK !!!\n");

		ret_size = ft8756_prd_test_data(ts->dev, DELTA_SHOW, &test_result);
		TOUCH_I("Show Delta Data Result : %d\n", test_result);

OUT:
		mutex_unlock(&ts->lock);

		if (ret < 0) {
			TOUCH_I("Show Delta Data FAIL !!!\n");
			return 0;
		}

	}

	if (off + count > LOG_BUF_SIZE) {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
	} else {
		if (ret_size > count) {
			ret_size -= count;
		} else {
			count = ret_size;
			ret_size = 0;
		}
		memcpy(buf, &log_buf[off], count);
	}

	return count;

}

static ssize_t show_rawdata_bin(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	int ret = 0;
	static int ret_size = 0;
	int test_result = TEST_FAIL;
//	u8 data = 0x00;

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_E("IC state is deep sleep.");
		return 0;
	}

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		TOUCH_I("Show Raw Data\n");

		mutex_lock(&ts->lock);

		memset(log_buf, 0, sizeof(log_buf));

		ft8756_dummy_read(ts->dev);
#if 1
		// Change clb switch
		ret = ft8756_switch_cal(ts->dev, 0);
		if(ret < 0)
			goto OUT;
#endif

		ret = ft8756_change_op_mode(ts->dev, FTS_FACTORY_MODE);
		if(ret < 0)
			goto OUT;

#if 0
		//ret = ft8756_prd_check_ch_num(ts->dev);
		//if(ret < 0)
		//	goto OUT;
#endif

		ret = ft8756_prd_get_raw_data(ts->dev);
		if(ret < 0)
			goto OUT;

		ret = ft8756_change_op_mode(ts->dev, FTS_WORK_MODE);
		if(ret < 0)
			goto OUT;

		TOUCH_I("Show Raw Data OK !!!\n");

		ret_size = ft8756_prd_test_data(ts->dev, RAW_DATA_TEST, &test_result);
		TOUCH_I("Raw Data Test Result : %d, data size: %d\n", test_result, ret_size);

OUT:
		mutex_unlock(&ts->lock);

		if (ret < 0) {
			TOUCH_I("Show Raw Data FAIL !!!\n");
			return 0;
		}

	}

	if (off + count > LOG_BUF_SIZE) {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
	} else {
		if (ret_size > count) {
			ret_size -= count;
		} else {
			count = ret_size;
			ret_size = 0;
		}
		memcpy(buf, &log_buf[off], count);
	}

	return count;
}

static ssize_t show_noise_bin(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct ft8756_data *d = to_ft8756_data(ts->dev);
	int ret = 0;
	static int ret_size = 0;
	int test_result = TEST_FAIL;

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_E("IC state is deep sleep.");
		return 0;
	}

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		TOUCH_I("Show Noise\n");

		mutex_lock(&ts->lock);

		memset(log_buf, 0, sizeof(log_buf));

		// Reset ???
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(5);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(300);

		ret = ft8756_change_op_mode(ts->dev, FTS_FACTORY_MODE);
		if(ret < 0)
			goto OUT;

		//ret = ft8756_prd_check_ch_num(ts->dev);
		//if(ret < 0)
		//	goto OUT;

		ret = ft8756_prd_get_noise_data(ts->dev);
		if(ret < 0)
			goto OUT;

		ret = ft8756_change_op_mode(ts->dev, FTS_WORK_MODE);
		if(ret < 0)
			goto OUT;

		TOUCH_I("Show Noise OK !!!\n");

		ret_size = ft8756_prd_test_data(ts->dev, NOISE_TEST, &test_result);
		TOUCH_I("Noise Test Result : %d\n", test_result);

OUT:

		mutex_unlock(&ts->lock);

		touch_gpio_direction_output(ts->reset_pin, 0); //reset_ctrl modify, because need init, bring_up
		touch_msleep(5);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(300);

		if(d->state != TC_STATE_ACTIVE) {
			ft8756_lpwg_set(ts->dev); // no need, use reset_ctrl func, no use fb_lock, bring_up
		}

		if (ret < 0) {
			TOUCH_I("Show Noise FAIL !!!\n");
			return 0;
		}

	}

	if (off + count > LOG_BUF_SIZE) {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
	} else {
		if (ret_size > count) {
			ret_size -= count;
		} else {
			count = ret_size;
			ret_size = 0;
		}
		memcpy(buf, &log_buf[off], count);
	}

	return count;
}

static ssize_t show_jitter_bin(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct touch_core_data *ts = container_of(kobj, struct touch_core_data, kobj);
	struct ft8756_data *d = to_ft8756_data(ts->dev);
	int ret = 0;
	static int ret_size = 0;
	int test_result = TEST_FAIL;

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_E("IC state is deep sleep.");
		return 0;
	}

	TOUCH_I("%s : off[%d] count[%d]\n", __func__, (int)off, (int)count);

	if (off == 0) {

		TOUCH_I("Show Jitter\n");

		mutex_lock(&ts->lock);

		memset(log_buf, 0, sizeof(log_buf));

		// Reset ???
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(5);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(300);

		ret = ft8756_change_op_mode(ts->dev, FTS_FACTORY_MODE);
		if(ret < 0)
			goto OUT;

		//ret = ft8756_prd_check_ch_num(ts->dev);
		//if(ret < 0)
		//	goto OUT;

		ret = ft8756_prd_get_jitter_data(ts->dev);
		if(ret < 0)
			goto OUT;

		ret = ft8756_change_op_mode(ts->dev, FTS_WORK_MODE);
		if(ret < 0)
			goto OUT;

		TOUCH_I("Show jitter OK !!!\n");

		ret_size = ft8756_prd_test_data(ts->dev, JITTER_TEST, &test_result);
		TOUCH_I("Jitter Test Result : %d\n", test_result);

OUT:
		mutex_unlock(&ts->lock);

		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(5);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(300);

		mutex_unlock(&ts->lock);
		if(d->state != TC_STATE_ACTIVE) {
			ft8756_lpwg_set(ts->dev);
		}

		if (ret < 0) {
			TOUCH_I("Show Jitter FAIL !!!\n");
			return 0;
		}

	}

	if (off + count > LOG_BUF_SIZE) {
		TOUCH_I("%s size error offset[%d] size[%d]\n", __func__, (int)off, (int)count);
	} else {
		if (ret_size > count) {
			ret_size -= count;
		} else {
			count = ret_size;
			ret_size = 0;
		}
		memcpy(buf, &log_buf[off], count);
	}

	return count;
}

static ssize_t show_open_test(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int test_result = TEST_FAIL;
	int ret_size = 0;

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	ret = ft8756_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

	ret = ft8756_prd_get_open_data(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8756_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show OPEN Data OK !!!\n");

	ret_size = ft8756_prd_test_data(dev, OPEN_CB_TEST, &test_result);
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("OPEN Data Test Result : %d\n", test_result);
	//printk("%s\n", log_buf);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
	return ret_size;

FAIL:

	TOUCH_I("Show open_test FAIL !!!\n");
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
	return 0;

}

static ssize_t show_short_test(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int test_result = TEST_FAIL;
	int ret_size = 0;

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	ret = ft8756_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

	ret = ft8756_prd_get_short_data(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8756_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show SHORT Data OK !!!\n");

	ret_size = ft8756_prd_test_data(dev, SHORT_ADC_TEST, &test_result);
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("SHORT Data Test Result : %d\n", test_result);
	//printk("%s\n", log_buf);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
	return ret_size;

FAIL:

	TOUCH_I("Show short_test FAIL !!!\n");
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
	return 0;

}

static ssize_t show_cb(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result = TEST_FAIL;
//	u8 data = 0x00;

	TOUCH_I("Show CB Data\n");

//	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	ft8756_dummy_read(dev);

	// Change clb switch
	ret = ft8756_switch_cal(dev, 0);
	if(ret < 0)
		goto FAIL;

	ret = ft8756_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

#if 0
	ret = ft8756_prd_check_ch_num(dev);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8756_prd_get_cb_data(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8756_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show CB Data OK !!!\n");

	ret_size = ft8756_prd_test_data(dev, CB_DATA_TEST, &test_result);
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("CB Data Test Result : %d\n", test_result);
	//printk("%s\n", log_buf);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_size;

FAIL:

	TOUCH_I("Show CB Data FAIL !!!\n");
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return 0;
}


static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8756_data *d = to_ft8756_data(dev);
	int ret = 0;
	int ret_size = 0, ret_total_size = 0;
	int rawdata_ret = TEST_FAIL;
	int cb_ret = TEST_FAIL;
	int noise_ret = TEST_FAIL;
//	int jitter_ret = TEST_FAIL;

	TOUCH_I("%s\n", __func__);
	// Check Current State
	if(d->state == TC_STATE_ACTIVE) {
		TOUCH_E("Show_lpwg_sd called in Active state\n");
		return 0;
	}

	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(300);

	/* file create , time log */
	TOUCH_I("Show_lpwg_sd Test Start\n");
	write_file(dev, "\nShow_lpwg_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

	// Change clb switch
	ret = ft8756_switch_cal(dev, 1);
	if (ret < 0) {
		TOUCH_E("change clb switch error\n");
		goto FAIL;
	}

	// Change to factory mode
	ret = ft8756_change_op_mode(dev, FTS_FACTORY_MODE);
	if (ret < 0) {
		TOUCH_E("change to factory mode error\n");
		goto FAIL;
	}

	// Start to raw data test
	TOUCH_I("Show_lpwg_sd : LPWG Raw data test\n");
	memset(log_buf, 0, sizeof(log_buf));

	ret = ft8756_prd_check_ch_num(dev);
	if (ret < 0) {
		TOUCH_E("prd check_ch_num error\n");
		goto FAIL;
	}

	ret = ft8756_prd_get_raw_data(dev);
	if (ret < 0) {
		TOUCH_E("prd get raw data error\n");
		goto FAIL;
	}

	ret_size = ft8756_prd_test_data(dev, LPWG_RAW_DATA_TEST, &rawdata_ret);

	TOUCH_I("LPWG Raw Data Test Result : %d\n", rawdata_ret);
	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);

	// Start to CB data test
	TOUCH_I("Show_lpwg_sd : LPWG CB data test\n");
	memset(log_buf, 0, sizeof(log_buf));

	ret = ft8756_prd_get_cb_data(dev);
	if (ret < 0) {
		TOUCH_E("prd get cb data error\n");
		goto FAIL;
	}

	ret_size = ft8756_prd_test_data(dev, LPWG_CB_DATA_TEST, &cb_ret);

	TOUCH_I("LPWG CB Data Test Result : %d\n", cb_ret);
	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);

	// Start to noise test
	TOUCH_I("Show_lpwg_sd : LPWG Noise test\n");
	memset(log_buf, 0, sizeof(log_buf));

	ret = ft8756_prd_get_noise_data(dev);
	if (ret < 0) {
		TOUCH_E("prd get noise data error\n");
		goto FAIL;
	}

	ret_size = ft8756_prd_test_data(dev, LPWG_NOISE_TEST, &noise_ret);

	TOUCH_I("LPWG Noise Test Result : %d\n", noise_ret);
	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);

#if 0
	// Start to lpwg jitter test
	TOUCH_I("Show_lpwg_sd : Jitter test\n");
	memset(log_buf, 0, sizeof(log_buf));

	ret = ft8756_prd_get_jitter_data(dev);
	if (ret < 0) {
		TOUCH_E("prd get noise data error\n");
		goto FAIL;
	}

	ret_size = ft8756_prd_test_data(dev, LPWG_JITTER_TEST, &jitter_ret);

	TOUCH_I("Jitter Test Result : %d\n", jitter_ret);
	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);
#endif

	// Change to working mode
	ret = ft8756_change_op_mode(dev, FTS_WORK_MODE);
	if (ret < 0) {
		TOUCH_I("Failed to return WORK_MODE\n");
		goto FAIL;
	}

	// Test result
	ret = touch_snprintf(log_buf, sizeof(log_buf), "\n========RESULT=======\n");
	TOUCH_I("========RESULT=======\n");

	if ((rawdata_ret == TEST_PASS) && (cb_ret == TEST_PASS) && (noise_ret == TEST_PASS)) {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "LPWG RawData : Pass\n");
		TOUCH_I("LPWG Raw Data : Pass\n");
	} else {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "LPWG RawData : Fail (raw:%d/cb:%d/noise:%d)\n",
				(rawdata_ret == TEST_FAIL)? 0 : 1, (cb_ret == TEST_FAIL)? 0 : 1, (noise_ret == TEST_FAIL)? 0 : 1);
		TOUCH_I("LPWG RawData : Fail (raw:%d/cb:%d/noise:%d)\n",
				(rawdata_ret == TEST_FAIL)? 0 : 1, (cb_ret == TEST_FAIL)? 0 : 1, (noise_ret == TEST_FAIL)? 0 : 1);
	}

	ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "=====================\n");
	TOUCH_I("=====================\n");

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret);
	ret_total_size += ret;

	mutex_unlock(&ts->lock);
	if(d->state != TC_STATE_ACTIVE) {
		ft8756_lpwg_set(dev);
	}
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_total_size;

FAIL:
	// Test result
	ret = touch_snprintf(log_buf, sizeof(log_buf), "\n========RESULT=======\n");
	TOUCH_I("========RESULT=======\n");

	if ((rawdata_ret == TEST_PASS) && (cb_ret == TEST_PASS) && (noise_ret == TEST_PASS)) {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "LPWG RawData : Pass\n");
		TOUCH_I("LPWG RawData : Pass\n");
	} else {
		ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "LPWG RawData : Fail\n");
		TOUCH_I("LPWG RawData : Fail\n");
	}

	ret += touch_snprintf(log_buf + ret, sizeof(log_buf) - ret, "=====================\n");
	TOUCH_I("=====================\n");

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret);
	ret_total_size += ret;

	mutex_unlock(&ts->lock);

	if(d->state != TC_STATE_ACTIVE) {
		ft8756_lpwg_set(dev);
	}
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_total_size;
}

static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);
static TOUCH_ATTR(cb_test, show_cb, NULL);
static TOUCH_ATTR(open_test, show_open_test, NULL);
static TOUCH_ATTR(short_test, show_short_test, NULL);

#define TOUCH_BIN_ATTR(_name, _read, _write, _size)		\
		struct bin_attribute touch_attr_##_name	\
		= __BIN_ATTR(_name, S_IRUGO | S_IWUSR, _read, _write, _size)

static TOUCH_BIN_ATTR(delta, show_delta_bin, NULL, LOG_BUF_SIZE);
static TOUCH_BIN_ATTR(rawdata, show_rawdata_bin, NULL, LOG_BUF_SIZE);
static TOUCH_BIN_ATTR(noise, show_noise_bin, NULL, LOG_BUF_SIZE);
static TOUCH_BIN_ATTR(jitter, show_jitter_bin, NULL, LOG_BUF_SIZE);

static struct attribute *prd_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_lpwg_sd.attr,
	&touch_attr_cb_test.attr,
	&touch_attr_open_test.attr,
	&touch_attr_short_test.attr,
	NULL,
};

static struct bin_attribute *prd_attribute_bin_list[] = {
	&touch_attr_delta,
	&touch_attr_noise,
	&touch_attr_jitter,
	&touch_attr_rawdata,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
	.bin_attrs = prd_attribute_bin_list,
};

int ft8756_prd_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &prd_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	return ret;
}
