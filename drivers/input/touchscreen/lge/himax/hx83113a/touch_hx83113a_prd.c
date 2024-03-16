/*	Himax Android Driver Sample Code for debug nodes

	Copyright (C) 2018 Himax Corporation.

	This software is licensed under the terms of the GNU General Public
	License version 2, as published by the Free Software Foundation, and
	may be copied, distributed, and modified under those terms.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

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
#include <touch_common.h>

#include "touch_hx83113a_prd.h"
#include "touch_hx83113a_core.h"
#include "touch_hx83113_limit.txt"

extern struct himax_ic_data *ic_data;
extern struct himax_ts_data *private_ts;
extern struct himax_debug *debug_data;
extern struct device *g_hx_dev;
extern unsigned char	IC_CHECKSUM;
extern int i2c_error_count;
struct proc_dir_entry *himax_touch_proc_dir;

#ifdef HX_TP_PROC_2T2R
	extern bool Is_2T2R;
#endif

#ifdef HX_TP_PROC_2T2R
	bool Is_2T2R = false;
	int HX_RX_NUM_2					= 0;
	int HX_TX_NUM_2					= 0;
#endif

uint8_t g_diag_arr_num = 0;

#define HIMAX_PROC_TOUCH_FOLDER 	"android_touch"
uint8_t HX_PROC_SEND_FLAG;

int g_max_mutual = 0;
int g_min_mutual = 0xFFFF;
int g_max_self = 0;
int g_min_self = 0xFFFF;


struct timespec timeStart, timeEnd, timeDelta;
int g_switch_mode = 0;
/* =============================================================================================================

	Segment : Himax PROC Debug Function

============================================================================================================= */
static ssize_t show_CRC_test(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	struct device *dev = g_hx_dev;
	ssize_t ret = 0;
	uint8_t result = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL){
			TOUCH_I("%s, alloc fail\n", __func__);
		   	return ret;
		}
		hx83113a_sense_off(dev, true);
		touch_msleep(20);
		result = hx83113a_calculateChecksum(dev);
		hx83113a_sense_on(dev, 0x01);

		if (result) {
			ret += touch_snprintf(temp_buf + ret, len - ret, "CRC test is Pass! \n");
		} else {
			ret += touch_snprintf(temp_buf + ret, len - ret, "CRC test is Fail! \n");
		}

		if (copy_to_user(buf, temp_buf, len)) {
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);
		}

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static struct file_operations himax_proc_CRC_test_ops = {
	.owner = THIS_MODULE,
	.read = show_CRC_test,
};

static ssize_t show_vendor(struct file *file, char *buf,
									size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL){
			TOUCH_I("%s, alloc fail\n", __func__);
			return ret;
		}
//		ret += touch_snprintf(temp_buf + ret, len - ret, "FW_VER: %2X.%2X \n", ic_data->lge_fw_ver_majr1, ic_data->lge_fw_ver_majr2);
		HX_PROC_SEND_FLAG = 1;

		if (copy_to_user(buf, temp_buf, len)) {
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);
		}

		kfree(temp_buf);
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static struct file_operations himax_proc_vendor_ops = {
	.owner = THIS_MODULE,
	.read = show_vendor,
};

static ssize_t show_attn(struct file *file, char *buf,
								size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	struct himax_ts_data *d;
	char *temp_buf;
	d = private_ts;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL){
			TOUCH_I("%s, alloc fail\n", __func__);
		   	return ret;
		}
		ret += touch_snprintf(temp_buf + ret, len - ret, "attn = %x\n", hx83113a_int_gpio_read(d->pdata->gpio_irq));

		if (copy_to_user(buf, temp_buf, len))
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}


static struct file_operations himax_proc_attn_ops = {
	.owner = THIS_MODULE,
	.read = show_attn,
};

static ssize_t show_int_en(struct file *file, char *buf,
									size_t len, loff_t *pos)
{
	struct himax_ts_data *d = private_ts;
	size_t ret = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL){
			TOUCH_I("%s, alloc fail\n", __func__);
			return ret;
		}
		ret += touch_snprintf(temp_buf + ret, len - ret, "%d ", d->irq_enabled);
		ret += touch_snprintf(temp_buf + ret, len - ret, "\n");

		if (copy_to_user(buf, temp_buf, len))
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t store_int_en(struct file *file, const char *buff,
									size_t len, loff_t *pos)
{
	struct himax_ts_data *d = private_ts;
	char buf_tmp[12] = {0};
	int value = 0;

	if (len >= 12) {
		TOUCH_I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buff, len)) {
		return -EFAULT;
	}

	if (buf_tmp[0] == '0') {
		value = false;
	} else if (buf_tmp[0] == '1') {
		value = true;
	} else {
		return -EINVAL;
	}
	if (value) {
		touch_interrupt_control(d->dev, INTERRUPT_ENABLE);
		d->irq_enabled = 1;
	} else {
		touch_interrupt_control(d->dev, INTERRUPT_DISABLE);
		d->irq_enabled = 0;
	}

	return len;
}

static struct file_operations himax_proc_int_en_ops = {
	.owner = THIS_MODULE,
	.read = show_int_en,
	.write = store_int_en,
};

static ssize_t show_layout(struct file *file, char *buf,
									size_t len, loff_t *pos)
{
	struct touch_core_data *ts = to_touch_core(g_hx_dev);
	size_t ret = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL){
			TOUCH_I("%s, alloc fail\n", __func__);
		   	return ret;
		}

		ret += touch_snprintf(temp_buf + ret, len - ret, "%d ", 0);
		ret += touch_snprintf(temp_buf + ret, len - ret, "%d ", ts->caps.max_x);
		ret += touch_snprintf(temp_buf + ret, len - ret, "%d ", 0);
		ret += touch_snprintf(temp_buf + ret, len - ret, "%d ", ts->caps.max_y);
		ret += touch_snprintf(temp_buf + ret, len - ret, "\n");

		if (copy_to_user(buf, temp_buf, len))
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static struct file_operations himax_proc_layout_ops = {
	.owner = THIS_MODULE,
	.read = show_layout,
};

static ssize_t show_debug_level(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	struct himax_ts_data *d;
	size_t ret = 0;
	char *temp_buf;
	d = private_ts;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL){
			TOUCH_I("%s, alloc fail\n", __func__);
			return ret;
		}
		ret += touch_snprintf(temp_buf + ret, len - ret, "%d\n", d->debug_log_level);

		if (copy_to_user(buf, temp_buf, len))
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t store_debug_level(struct file *file, const char *buff,
										size_t len, loff_t *pos)
{
	struct himax_ts_data *d;
	struct touch_core_data *ts = to_touch_core(g_hx_dev);
	char buf_tmp[11];
	int i;
	d = private_ts;

	if (len >= 12) {
		TOUCH_I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buff, len)) {
		return -EFAULT;
	}

	d->debug_log_level = 0;

	for (i = 0; i < len - 1; i++) {
		if (buf_tmp[i] >= '0' && buf_tmp[i] <= '9')
			d->debug_log_level |= (buf_tmp[i] - '0');
		else if (buf_tmp[i] >= 'A' && buf_tmp[i] <= 'F')
			d->debug_log_level |= (buf_tmp[i] - 'A' + 10);
		else if (buf_tmp[i] >= 'a' && buf_tmp[i] <= 'f')
			d->debug_log_level |= (buf_tmp[i] - 'a' + 10);

		if (i != len - 2)
			d->debug_log_level <<= 4;
	}

	if (d->debug_log_level & BIT(3)) {
		if (ts->caps.max_width_minor > 0 && ts->caps.max_width_major > 0 &&
			ts->caps.max_x > 0 &&	ts->caps.max_y > 0) {
			d->widthFactor = (ts->caps.max_width_minor << SHIFTBITS) / ts->caps.max_x;
			d->heightFactor = (ts->caps.max_width_major << SHIFTBITS) / ts->caps.max_y;

			if (d->widthFactor > 0 && d->heightFactor > 0) {
				d->useScreenRes = 1;
			} else {
				d->heightFactor = 0;
				d->widthFactor = 0;
				d->useScreenRes = 0;
			}
		} else {
			TOUCH_I("Enable finger debug with raw position mode!\n");
		}
	} else {
		d->useScreenRes = 0;
		d->widthFactor = 0;
		d->heightFactor = 0;
	}

	return len;
}

static struct file_operations himax_proc_debug_level_ops = {
	.owner = THIS_MODULE,
	.read = show_debug_level,
	.write = store_debug_level,
};

static ssize_t show_proc_register(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	struct device *dev = g_hx_dev;
	int ret = 0;
	uint16_t loop_i;
	uint8_t data[128];
	char *temp_buf;
	int addr = 0;

	memset(data, 0x00, sizeof(data));

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL){
			TOUCH_I("%s, alloc fail\n", __func__);
			return ret;
		}
		TOUCH_I("himax_register_show: %02X,%02X,%02X,%02X\n", register_command[3], register_command[2], register_command[1], register_command[0]);
		addr = (register_command[3] << 24) + (register_command[2] << 16) + (register_command[1] << 8) + register_command[0];
		hx83113a_register_read(dev, addr, 128, data, ADDR_LEN_4);
		ret += touch_snprintf(temp_buf + ret, len - ret, "command:  %02X,%02X,%02X,%02X\n", register_command[3], register_command[2], register_command[1], register_command[0]);

		for (loop_i = 0; loop_i < 128; loop_i++) {
			ret += touch_snprintf(temp_buf + ret, len - ret, "0x%2.2X ", data[loop_i]);
			if ((loop_i % 16) == 15)
				ret += touch_snprintf(temp_buf + ret, len - ret, "\n");
		}

		ret += touch_snprintf(temp_buf + ret, len - ret, "\n");

		if (copy_to_user(buf, temp_buf, len))
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t store_proc_register(struct file *file, const char *buff,
		size_t len, loff_t *pos)
{
	struct device *dev = g_hx_dev;
	char buf[80] = {0};
	char buf_tmp[16];
	uint8_t length = 0;
	unsigned long result	= 0;
	uint8_t loop_i			= 0;
	uint16_t base			= 2;
	char *data_str = NULL;
	uint8_t w_data[20];
	uint8_t x_pos[20];
	uint8_t count = 0;
	int addr = 0;

	if (len >= 80) {
		TOUCH_I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(w_data, 0x0, sizeof(w_data));
	memset(x_pos, 0x0, sizeof(x_pos));
	memset(register_command, 0x0, sizeof(register_command));

	TOUCH_I("himax %s \n", buf);

	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':' && buf[2] == 'x') {
		length = strlen(buf);

		/* TOUCH_I("%s: length = %d.\n", __func__,length); */
		for (loop_i = 0; loop_i < length; loop_i++) { /* find postion of 'x' */
			if (buf[loop_i] == 'x') {
				x_pos[count] = loop_i;
				count++;
			}
		}

		data_str = strrchr(buf, 'x');
		TOUCH_I("%s: %s.\n", __func__, data_str);
		length = strlen(data_str + 1) - 1;

		if (buf[0] == 'r') {
			if (buf[3] == 'F' && buf[4] == 'E' && length == 4) {
				length = length - base;
				cfg_flag = 1;
				memcpy(buf_tmp, data_str + base + 1, length);
			} else {
				cfg_flag = 0;
				memcpy(buf_tmp, data_str + 1, length);
			}

			byte_length = length / 2;

			if (!kstrtoul(buf_tmp, 16, &result)) {
				for (loop_i = 0 ; loop_i < byte_length ; loop_i++) {
					register_command[loop_i] = (uint8_t)(result >> loop_i * 8);
				}
			}

			if (strcmp(HX_85XX_H_SERIES_PWON, private_ts->chip_name) == 0 && cfg_flag == 0)
				cfg_flag = 2;
		} else if (buf[0] == 'w') {
			if (buf[3] == 'F' && buf[4] == 'E') {
				cfg_flag = 1;
				memcpy(buf_tmp, buf + base + 3, length);
			} else {
				cfg_flag = 0;
				memcpy(buf_tmp, buf + 3, length);
			}

			if (count < 3) {
				byte_length = length / 2;

				if (!kstrtoul(buf_tmp, 16, &result)) { /* command */
					for (loop_i = 0 ; loop_i < byte_length ; loop_i++) {
						register_command[loop_i] = (uint8_t)(result >> loop_i * 8);
					}
				}

				if (!kstrtoul(data_str + 1, 16, &result)) { /* data */
					for (loop_i = 0 ; loop_i < byte_length ; loop_i++) {
						w_data[loop_i] = (uint8_t)(result >> loop_i * 8);
					}
				}
				addr = (register_command[3] << 24) + (register_command[2] << 16) + (register_command[1] << 8) + register_command[0];
				hx83113a_register_write(dev, addr, byte_length, w_data, ADDR_LEN_4);
			} else {
				for (loop_i = 0; loop_i < count; loop_i++) { /* parsing addr after 'x' */
					memset(buf_tmp, 0x0, sizeof(buf_tmp));
					if (cfg_flag != 0 && loop_i != 0)
						byte_length = 2;
					else
						byte_length = x_pos[1] - x_pos[0] - 2; /* original */

					memcpy(buf_tmp, buf + x_pos[loop_i] + 1, byte_length);

					/* TOUCH_I("%s: buf_tmp = %s\n", __func__,buf_tmp); */
					if (!kstrtoul(buf_tmp, 16, &result)) {
						if (loop_i == 0) {
							register_command[loop_i] = (uint8_t)(result);
							/* TOUCH_I("%s: register_command = %X\n", __func__,register_command[0]); */
						} else {
							w_data[loop_i - 1] = (uint8_t)(result);
							/* TOUCH_I("%s: w_data[%d] = %2X\n", __func__,loop_i - 1,w_data[loop_i - 1]); */
						}
					}
				}

				byte_length = count - 1;
				if (strcmp(HX_85XX_H_SERIES_PWON, private_ts->chip_name) == 0 && cfg_flag == 0)
					cfg_flag = 2;
				addr = (register_command[3] << 24) + (register_command[2] << 16) + (register_command[1] << 8) + register_command[0];
				hx83113a_register_write(dev, addr, byte_length, &w_data[0], ADDR_LEN_4);
			}
		} else {
			return len;
		}
	}

	return len;
}

static struct file_operations himax_proc_register_ops = {
	.owner = THIS_MODULE,
	.read = show_proc_register,
	.write = store_proc_register,
};

int32_t *getMutualBuffer(void)
{
	return diag_mutual;
}
int32_t *getMutualNewBuffer(void)
{
	return diag_mutual_new;
}
int32_t *getMutualOldBuffer(void)
{
	return diag_mutual_old;
}
int32_t *getSelfBuffer(void)
{
	return &diag_self[0];
}
int32_t *getSelfNewBuffer(void)
{
	return &diag_self_new[0];
}
int32_t *getSelfOldBuffer(void)
{
	return &diag_self_old[0];
}
void setMutualBuffer(uint8_t x_num, uint8_t y_num)
{
	diag_mutual = kzalloc(x_num * y_num * sizeof(int32_t), GFP_KERNEL);
}
void setMutualNewBuffer(uint8_t x_num, uint8_t y_num)
{
	diag_mutual_new = kzalloc(x_num * y_num * sizeof(int32_t), GFP_KERNEL);
}
void setMutualOldBuffer(uint8_t x_num, uint8_t y_num)
{
	diag_mutual_old = kzalloc(x_num * y_num * sizeof(int32_t), GFP_KERNEL);
}

#ifdef HX_TP_PROC_2T2R
int32_t *getMutualBuffer_2(void)
{
	return diag_mutual_2;
}
void setMutualBuffer_2(uint8_t x_num_2, uint8_t y_num_2)
{
	diag_mutual_2 = kzalloc(x_num_2 * y_num_2 * sizeof(int32_t), GFP_KERNEL);
}
#endif

static bool hx83113a_diag_check_sum(struct himax_report_data *hx_touch_data)
{
	uint16_t check_sum_cal = 0;
	int i;

	/* Check 128th byte CRC */
	for (i = 0, check_sum_cal = 0; i < hx_touch_data->touch_all_size - hx_touch_data->touch_info_size; i += 2) { //CUST_TP_SIZE
		check_sum_cal += (hx_touch_data->hx_rawdata_buf[i + 1] * FLASH_RW_MAX_LEN + hx_touch_data->hx_rawdata_buf[i]);
	}

	if (check_sum_cal % HX64K != 0) {
		TOUCH_I("%s fail=%2X \n", __func__, check_sum_cal);
		return 0;
	}

	return 1;
}

static void hx83113a_diag_parse_raw_data(struct himax_report_data *hx_touch_data, int mul_num, int self_num, uint8_t diag_cmd, int32_t *mutual_data, int32_t *self_data)
{
	int RawDataLen_word;
	int index = 0;
	int temp1, temp2, i;

	if (hx_touch_data->hx_rawdata_buf[0] == FW_DATA_RAWDATA_READY_LB
	    && hx_touch_data->hx_rawdata_buf[1] == FW_DATA_RAWDATA_READY_HB
	    && hx_touch_data->hx_rawdata_buf[2] > 0
	    && hx_touch_data->hx_rawdata_buf[3] == diag_cmd) {
		RawDataLen_word =  hx_touch_data->rawdata_size  / 2;//((MAX_I2C_TRANS_SZ - CUST_TP_SIZE) / 2) - 3; /* 1 byte Header, 1 byte index, 1 byte checksum*/
		index = (hx_touch_data->hx_rawdata_buf[2] - 1) * RawDataLen_word;

		/* TOUCH_I("Header[%d]: %x, %x, %x, %x, mutual: %d, self: %d\n", index, buf[56], buf[57], buf[58], buf[59], mul_num, self_num);
		 TOUCH_I("RawDataLen=%d , RawDataLen_word=%d , hx_touch_info_size=%d\n", RawDataLen, RawDataLen_word, hx_touch_info_size);*/
		for (i = 0; i < RawDataLen_word; i++) {
			temp1 = index + i;

			if (temp1 < mul_num) { /*mutual*/
				mutual_data[index + i] = ((int8_t)hx_touch_data->hx_rawdata_buf[i * 2 + 4 + 1]) * 256 + hx_touch_data->hx_rawdata_buf[i * 2 + 4];
			} else { /*self*/
				temp1 = i + index;
				temp2 = self_num + mul_num;

				if (temp1 >= temp2) {
					break;
				}

				self_data[i + index - mul_num] = (((int8_t)hx_touch_data->hx_rawdata_buf[i * 2 + 4 + 1]) << 8) +
				hx_touch_data->hx_rawdata_buf[i * 2 + 4];
			}
		}
	}
}

int hx83113a_set_diag_cmd(struct himax_ic_data *ic_data, struct himax_report_data *hx_touch_data)
{
	struct himax_ts_data *d = private_ts;
	int32_t *mutual_data;
	int32_t *self_data;
	int mul_num;
	int self_num;
	/* int RawDataLen = 0; */
	hx_touch_data->diag_cmd = d->diag_cmd;

	if (hx_touch_data->diag_cmd >= 1 && hx_touch_data->diag_cmd <= 7) {
		/* Check event stack CRC */
		if (!hx83113a_diag_check_sum(hx_touch_data)) {
			goto bypass_checksum_failed_packet;
		}

#ifdef HX_TP_PROC_2T2R
		if (Is_2T2R && (hx_touch_data->diag_cmd >= 4 && hx_touch_data->diag_cmd <= 6)) {
			mutual_data = getMutualBuffer_2();
			self_data = getSelfBuffer();
			/*	initiallize the block number of mutual and self */
			mul_num = ic_data->HX_RX_NUM_2 * ic_data->HX_TX_NUM_2;
			self_num = ic_data->HX_RX_NUM_2 + ic_data->HX_TX_NUM_2;
		} else
#endif
		{
			mutual_data = getMutualBuffer();
			self_data = getSelfBuffer();
			/*	initiallize the block number of mutual and self */
			mul_num = ic_data->HX_RX_NUM * ic_data->HX_TX_NUM;
			self_num = ic_data->HX_RX_NUM + ic_data->HX_TX_NUM;
		}
		hx83113a_diag_parse_raw_data(hx_touch_data, mul_num, self_num, hx_touch_data->diag_cmd, mutual_data, self_data);
	} else if (hx_touch_data->diag_cmd == 8) {
		memset(diag_coor, 0x00, sizeof(diag_coor));
		memcpy(&(diag_coor[0]), &hx_touch_data->hx_coord_buf[0], hx_touch_data->touch_info_size);
	}

	/* assign state info data */
	memcpy(&(hx_state_info[0]), &hx_touch_data->hx_state_info[0], 2);
	return NO_ERR;
bypass_checksum_failed_packet:
	return 1;
}

/* #if defined(HX_DEBUG_LEVEL) */
extern struct himax_target_report_data *g_target_report_data;
extern struct himax_report_data *hx_touch_data;
void hx83113a_log_touch_data(struct himax_ts_data *d, int start)
{
	int loop_i = 0;
	int print_size = 0;
	uint8_t *buf;

	if (start == 1)
		return; /* report data when end of ts_work*/

	if (hx_touch_data->diag_cmd == 0) {
		print_size = CUST_TP_SIZE;
		buf = kzalloc(CUST_TP_SIZE * sizeof(uint8_t), GFP_KERNEL);
		if(buf == NULL){
			TOUCH_I("%s, alloc fail\n", __func__);
			return;
		}
		if (atomic_read(&d->suspend_mode) && (d->SMWP_enable))
			memcpy(buf, hx_touch_data->hx_event_buf, hx_touch_data->touch_info_size);
		else {
			memcpy(buf, hx_touch_data->hx_coord_buf, hx_touch_data->touch_info_size);
			memcpy(&buf[hx_touch_data->touch_info_size], hx_touch_data->hx_width_buf, CUST_TP_SHAPE_SIZE);
		}
	} else if (hx_touch_data->diag_cmd > 0) {
		print_size = hx_touch_data->touch_all_size;
		buf = kzalloc(hx_touch_data->touch_all_size * sizeof(uint8_t), GFP_KERNEL);//hx_touch_data->touch_info_size
		if(buf == NULL){
			TOUCH_I("%s, alloc fail\n", __func__);
		   	return;
		}
		memcpy(buf, hx_touch_data->hx_coord_buf, hx_touch_data->touch_info_size);
		memcpy(&buf[hx_touch_data->touch_info_size], hx_touch_data->hx_width_buf, CUST_TP_SHAPE_SIZE);
		memcpy(&buf[CUST_TP_SIZE], hx_touch_data->hx_rawdata_buf, hx_touch_data->touch_all_size - CUST_TP_SIZE);
	} else {
		TOUCH_E("%s:cmd fault\n", __func__);
	}

	for (loop_i = 0; loop_i < print_size; loop_i += 2) {
		TOUCH_I("P %2d = 0x%2.2X P %2d = 0x%2.2X ", loop_i, buf[loop_i], loop_i + 1, buf[loop_i + 1]);
		if (loop_i + 2 == print_size)
			TOUCH_I("\n");
	}
	kfree(buf);
}
void hx83113a_log_touch_event(struct himax_ts_data *d, int start)
{
	int loop_i = 0;
	struct touch_core_data *ts = to_touch_core(g_hx_dev);

	if (g_target_report_data->finger_on > 0 && g_target_report_data->finger_num > 0) {
		for (loop_i = 0; loop_i < d->nFinger_support; loop_i++) {
			if (g_target_report_data->x[loop_i] >= 0 && g_target_report_data->x[loop_i] <= ts->caps.max_x && g_target_report_data->y[loop_i] >= 0 && g_target_report_data->y[loop_i] <= ts->caps.max_y) {
				TOUCH_I("Finger %d=> X:%d, Y:%d W:%d, wM:%d, wm:%d, A:%d, F:%d\n", loop_i + 1,
				g_target_report_data->x[loop_i],
				g_target_report_data->y[loop_i],
				g_target_report_data->w[loop_i],
				g_target_report_data->width_major[loop_i],
				g_target_report_data->width_minor[loop_i],
				g_target_report_data->orientation[loop_i],
				loop_i + 1);
			}
		}
	} else if (g_target_report_data->finger_on == 0 && g_target_report_data->finger_num == 0) {
		TOUCH_I("All Finger leave\n");
	} else {
		TOUCH_I("%s : wrong input!\n", __func__);
	}
}
void hx83113a_log_touch_int_devation(int touched)
{
	if (touched == HX_FINGER_ON) {
		getnstimeofday(&timeStart);
		/*  TOUCH_I(" Irq start time = %ld.%06ld s\n",
		timeStart.tv_sec, timeStart.tv_nsec/1000); */
	} else if (touched == HX_FINGER_LEAVE) {
		getnstimeofday(&timeEnd);
		timeDelta.tv_nsec = (timeEnd.tv_sec * 1000000000 + timeEnd.tv_nsec) - (timeStart.tv_sec * 1000000000 + timeStart.tv_nsec);
		/*  TOUCH_I("Irq finish time = %ld.%06ld s\n",
			timeEnd.tv_sec, timeEnd.tv_nsec/1000);*/
		TOUCH_I("Touch latency = %ld us\n", timeDelta.tv_nsec / 1000);
	} else {
		TOUCH_I("%s : wrong input!\n", __func__);
	}
}
void hx83113a_log_touch_event_detail(struct himax_ts_data *d, int start)
{
	int loop_i = 0;
	struct touch_core_data *ts = to_touch_core(g_hx_dev);

	if (start == HX_FINGER_LEAVE) {
		for (loop_i = 0; loop_i < d->nFinger_support; loop_i++) {
			if (((d->old_finger >> loop_i & 1) == 0) && ((d->pre_finger_mask >> loop_i & 1) == 1)) {
					if (g_target_report_data->x[loop_i] >= 0 && g_target_report_data->x[loop_i] <= ts->caps.max_x && g_target_report_data->y[loop_i] >= 0 && g_target_report_data->y[loop_i] <= ts->caps.max_y) {
					TOUCH_I("status: Raw:F:%02d Down, X:%d, Y:%d, W:%d\n", loop_i + 1, g_target_report_data->x[loop_i], g_target_report_data->y[loop_i], g_target_report_data->w[loop_i]);
				}
			} else if ((((d->old_finger >> loop_i & 1) == 1) && ((d->pre_finger_mask >> loop_i & 1) == 0))) {
				TOUCH_I("status: Raw:F:%02d Up, X:%d, Y:%d\n", loop_i + 1, d->pre_finger_data[loop_i][0], d->pre_finger_data[loop_i][1]);
			} else {
				/* TOUCH_I("dbg hx_point_num=%d,old_finger=0x%02X,pre_finger_mask=0x%02X\n",ts->hx_point_num,ts->old_finger,ts->pre_finger_mask);*/
			}
		}
	}
}

void hx83113a_ts_dbg_func(struct himax_ts_data *d, int start)
{
	switch (d->debug_log_level) {
	case 1:
		hx83113a_log_touch_data(d, start);
		break;
	case 2:
		hx83113a_log_touch_event(d, start);
		break;
	case 4:
		hx83113a_log_touch_int_devation(start);
		break;
	case 8:
		hx83113a_log_touch_event_detail(d, start);
		break;
	}
}

/* #endif */
static ssize_t store_diag_arrange(struct file *file, const char *buff,
										size_t len, loff_t *pos)
{
	char buf[80] = {0};

	if (len >= 80) {
		TOUCH_I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	g_diag_arr_num = buf[0] - '0';
	TOUCH_I("%s: g_diag_arr_num = %d \n", __func__, g_diag_arr_num);
	return len;
}

void get_mutual_edge(void)
{
	int i = 0;

	for (i = 0; i < (ic_data->HX_RX_NUM * ic_data->HX_TX_NUM); i++) {
		if (diag_mutual[i] > g_max_mutual)
			g_max_mutual = diag_mutual[i];

		if (diag_mutual[i] < g_min_mutual)
			g_min_mutual = diag_mutual[i];
	}
}

void get_self_edge(void)
{
	int i = 0;

	for (i = 0; i < (ic_data->HX_RX_NUM + ic_data->HX_TX_NUM); i++) {
		if (diag_self[i] > g_max_self)
			g_max_self = diag_self[i];

		if (diag_self[i] < g_min_self)
			g_min_self = diag_self[i];
	}
}

/* print first step which is row */
static struct file_operations himax_proc_diag_arrange_ops = {
	.owner = THIS_MODULE,
	.write = store_diag_arrange,
};
static void hx83113a_print_state_info(struct seq_file *s)
{
	/* seq_printf(s, "State_info_2bytes:%3d, %3d\n",hx_state_info[0],hx_state_info[1]); */
	seq_printf(s, "ReCal = %d\t", hx_state_info[0] & 0x01);
	seq_printf(s, "Palm = %d\t", hx_state_info[0] >> 1 & 0x01);
	seq_printf(s, "AC mode = %d\t", hx_state_info[0] >> 2 & 0x01);
	seq_printf(s, "Water = %d\n", hx_state_info[0] >> 3 & 0x01);
	seq_printf(s, "Glove = %d\t", hx_state_info[0] >> 4 & 0x01);
	seq_printf(s, "TX Hop = %d\t", hx_state_info[0] >> 5 & 0x01);
	seq_printf(s, "Base Line = %d\t", hx_state_info[0] >> 6 & 0x01);
	seq_printf(s, "OSR Hop = %d\t", hx_state_info[1] >> 3 & 0x01);
	seq_printf(s, "KEY = %d\n", hx_state_info[1] >> 4 & 0x0F);
}

static void hx83113a_diag_arrange_print(struct seq_file *s, int i, int j, int transpose)
{
	if (transpose)
		seq_printf(s, "%6d", diag_mutual[j + i * ic_data->HX_RX_NUM]);
	else
		seq_printf(s, "%6d", diag_mutual[i + j * ic_data->HX_RX_NUM]);
}

/* ready to print second step which is column*/
static void hx83113a_diag_arrange_inloop(struct seq_file *s, int in_init, int out_init, bool transpose, int j)
{
	int x_channel = ic_data->HX_RX_NUM;
	int y_channel = ic_data->HX_TX_NUM;
	int i;
	int in_max = 0;

	if (transpose)
		in_max = y_channel;
	else
		in_max = x_channel;

	if (in_init > 0) { /* bit0 = 1 */
		for (i = in_init - 1; i >= 0; i--) {
			hx83113a_diag_arrange_print(s, i, j, transpose);
		}

		if (transpose) {
			if (out_init > 0)
				seq_printf(s, " %5d\n", diag_self[j]);
			else
				seq_printf(s, " %5d\n", diag_self[x_channel - j - 1]);
		}
	} else {	/* bit0 = 0 */
		for (i = 0; i < in_max; i++) {
			hx83113a_diag_arrange_print(s, i, j, transpose);
		}

		if (transpose) {
			if (out_init > 0)
				seq_printf(s, " %5d\n", diag_self[x_channel - j - 1]);
			else
				seq_printf(s, " %5d\n", diag_self[j]);
		}
	}
}

/* print first step which is row */
static void hx83113a_diag_arrange_outloop(struct seq_file *s, int transpose, int out_init, int in_init)
{
	int j;
	int x_channel = ic_data->HX_RX_NUM;
	int y_channel = ic_data->HX_TX_NUM;
	int out_max = 0;
	int self_cnt = 0;

	if (transpose)
		out_max = x_channel;
	else
		out_max = y_channel;

	if (out_init > 0) { /* bit1 = 1 */
		self_cnt = 1;

		for (j = out_init - 1; j >= 0; j--) {
			seq_printf(s, "%3c%02d%c", '[', j + 1, ']');
			hx83113a_diag_arrange_inloop(s, in_init, out_init, transpose, j);

			if (!transpose) {
				seq_printf(s, " %5d\n", diag_self[y_channel + x_channel - self_cnt]);
				self_cnt++;
			}
		}
	} else {	/* bit1 = 0 */
		/* self_cnt = x_channel; */
		for (j = 0; j < out_max; j++) {
			seq_printf(s, "%3c%02d%c", '[', j + 1, ']');
			hx83113a_diag_arrange_inloop(s, in_init, out_init, transpose, j);

			if (!transpose) {
				seq_printf(s, " %5d\n", diag_self[j + x_channel]);
			}
		}
	}
}

/* determin the output format of diag */
static void hx83113a_diag_arrange(struct seq_file *s)
{
	int x_channel = ic_data->HX_RX_NUM;
	int y_channel = ic_data->HX_TX_NUM;
	int bit2, bit1, bit0;
	int i;
	/* rotate bit */
	bit2 = g_diag_arr_num >> 2;
	/* reverse Y */
	bit1 = g_diag_arr_num >> 1 & 0x1;
	/* reverse X */
	bit0 = g_diag_arr_num & 0x1;

	if (g_diag_arr_num < 4) {
		for (i = 0 ; i <= x_channel; i++)
			seq_printf(s, "%3c%02d%c", '[', i, ']');

		seq_printf(s, "\n");
		hx83113a_diag_arrange_outloop(s, bit2, bit1 * y_channel, bit0 * x_channel);
		seq_printf(s, "%6c", ' ');

		if (bit0 == 1) {
			for (i = x_channel - 1; i >= 0; i--)
				seq_printf(s, "%6d", diag_self[i]);
		} else {
			for (i = 0; i < x_channel; i++)
				seq_printf(s, "%6d", diag_self[i]);
		}
	} else {
		for (i = 0 ; i <= y_channel; i++)
			seq_printf(s, "%3c%02d%c", '[', i, ']');

		seq_printf(s, "\n");
		hx83113a_diag_arrange_outloop(s, bit2, bit1 * x_channel, bit0 * y_channel);
		seq_printf(s, "%6c", ' ');

		if (bit1 == 1) {
			for (i = x_channel + y_channel - 1; i >= x_channel; i--) {
				seq_printf(s, "%6d", diag_self[i]);
			}
		} else {
			for (i = x_channel; i < x_channel + y_channel; i++) {
				seq_printf(s, "%6d", diag_self[i]);
			}
		}
	}
}

static void *start_diag_seq(struct seq_file *s, loff_t *pos)
{
	if (*pos >= 1) {
		return NULL;
	}
	return (void *)((unsigned long) *pos + 1);
}

static void *next_diag_seq(struct seq_file *s, void *v, loff_t *pos)
{
	return NULL;
}

static void stop_diag_seq(struct seq_file *s, void *v)
{
}

static int show_diag_seq(struct seq_file *s, void *v)
{
	struct himax_ts_data *d = private_ts;
	int x_channel = ic_data->HX_RX_NUM;
	int y_channel = ic_data->HX_TX_NUM;
#ifdef HX_TP_PROC_2T2R
	int x_channel_2 = ic_data->HX_RX_NUM_2;
	int y_channel_2 = ic_data->HX_TX_NUM_2;
#endif
	size_t ret = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;
	int dsram_type = 0;
	dsram_type = d->diag_cmd / 10;

#ifdef HX_TP_PROC_2T2R
	if (Is_2T2R && (d->diag_cmd >= 4 && d->diag_cmd <= 6)) {
		mutual_num	= x_channel_2 * y_channel_2;
		self_num	= x_channel_2 + y_channel_2; /* don't add KEY_COUNT */
		width		= x_channel_2;
		seq_printf(s, "ChannelStart: %4d, %4d\n\n", x_channel_2, y_channel_2);
	} else
#endif
	{
		mutual_num	= x_channel * y_channel;
		self_num	= x_channel + y_channel; /* don't add KEY_COUNT */
		width		= x_channel;
		seq_printf(s, "ChannelStart: %4d, %4d\n\n", x_channel, y_channel);
	}

	/*	start to show out the raw data in adb shell */
	if ((d->diag_cmd >= 1 && d->diag_cmd <= 3) || (d->diag_cmd == 7)) {
		hx83113a_diag_arrange(s);
		seq_printf(s, "\n");
		seq_printf(s, "ChannelEnd");
		seq_printf(s, "\n");
#ifdef HX_TP_PROC_2T2R
	} else if (Is_2T2R && d->diag_cmd >= 4 && d->diag_cmd <= 6) {
		for (loop_i = 0; loop_i < mutual_num; loop_i++) {
			seq_printf(s, "%4d", diag_mutual_2[loop_i]);

			if ((loop_i % width) == (width - 1))
				seq_printf(s, " %4d\n", diag_self[width + loop_i / width]);
		}

		seq_printf(s, "\n");

		for (loop_i = 0; loop_i < width; loop_i++) {
			seq_printf(s, "%4d", diag_self[loop_i]);

			if (((loop_i) % width) == (width - 1))
				seq_printf(s, "\n");
		}

		seq_printf(s, "ChannelEnd");
		seq_printf(s, "\n");
#endif
	} else if (d->diag_cmd == 8) {
		for (loop_i = 0; loop_i < 128 ; loop_i++) {
			if ((loop_i % 16) == 0)
				seq_printf(s, "LineStart:");

			seq_printf(s, "%4x", diag_coor[loop_i]);

			if ((loop_i % 16) == 15)
				seq_printf(s, "\n");
		}
	} else if (dsram_type > 0 && dsram_type <= 8) {
		hx83113a_diag_arrange(s);
		seq_printf(s, "\n ChannelEnd");
		seq_printf(s, "\n");
	}

	if ((d->diag_cmd >= 1 && d->diag_cmd <= 7) || dsram_type > 0) {
		/* print Mutual/Slef Maximum and Minimum */
		get_mutual_edge();
		get_self_edge();
		seq_printf(s, "Mutual Max:%3d, Min:%3d\n", g_max_mutual, g_min_mutual);
		seq_printf(s, "Self Max:%3d, Min:%3d\n", g_max_self, g_min_self);
		/* recovery status after print*/
		g_max_mutual = 0;
		g_min_mutual = 0xFFFF;
		g_max_self = 0;
		g_min_self = 0xFFFF;
	}

	/*pring state info*/
	hx83113a_print_state_info(s);
	return ret;
}
static struct seq_operations himax_diag_seq_ops = {
	.start	= start_diag_seq,
	.next	= next_diag_seq,
	.stop	= stop_diag_seq,
	.show	= show_diag_seq,
};
static int open_diag(struct inode *inode, struct file *file)
{
	return seq_open(file, &himax_diag_seq_ops);
};
bool DSRAM_Flag = false;

/* DSRAM thread */
static int hx83113a_write_read_reg(struct device *dev, uint32_t tmp_addr, uint8_t *tmp_data, uint8_t hb, uint8_t lb)
{
	int cnt = 0;
	uint8_t r_data[ADDR_LEN_4] = {0};  // Read Buffer

	do {
		if (r_data[1] != lb || r_data[0] != hb)
			hx83113a_register_write(dev, tmp_addr, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		touch_msleep(5);
		hx83113a_register_read(dev, tmp_addr, 4, r_data, ADDR_LEN_4);
		/* TOUCH_I("%s:Now tmp_data[0]=0x%02X,[1]=0x%02X,[2]=0x%02X,[3]=0x%02X\n",
		 __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);*/
	} while ((r_data[1] != hb || r_data[0] != lb) && cnt++ < 9);

	if (cnt == 10) {
		return HX_RW_REG_FAIL;
	}

	TOUCH_I("Now register 0x%08X : high byte=0x%02X,low byte=0x%02X\n", tmp_addr, tmp_data[1], tmp_data[0]);
	return NO_ERR;
}

static bool hx83113a_get_DSRAM_data(struct device *dev, uint8_t *info_data, bool DSRAM_Flag)
{
	int i = 0;
	unsigned char tmp_addr[ADDR_LEN_4];
	unsigned char tmp_data[DATA_LEN_4];
	uint8_t w_data[DATA_LEN_4];
	uint8_t max_i2c_size = MAX_I2C_TRANS_SZ;
	uint8_t x_num = ic_data->HX_RX_NUM;
	uint8_t y_num = ic_data->HX_TX_NUM;
	/*int m_key_num = 0;*/
	int total_size = (x_num * y_num + x_num + y_num) * 2 + 4;
	int total_size_temp;
	int mutual_data_size = x_num * y_num * 2;
	int total_read_times = 0;
	int address = 0;
	uint8_t  *temp_info_data; /*max mkey size = 8*/
	uint32_t check_sum_cal = 0;
	int fw_run_flag = -1;
	int addr = 0;

	temp_info_data = kzalloc(sizeof(uint8_t) * (total_size + 8), GFP_KERNEL);
	if(temp_info_data == NULL){
		TOUCH_I("%s, alloc fail\n", __func__);
		return false;
	}
	/*1. Read number of MKey R100070E8H to determin data size*/
	/*m_key_num = ic_data->HX_BT_NUM;
	TOUCH_I("%s,m_key_num=%d\n",__func__ ,m_key_num);
	total_size += m_key_num * 2;
	 2. Start DSRAM Rawdata and Wait Data Ready */
	tmp_data[3] = 0x00; tmp_data[2] = 0x00;
	tmp_data[1] = Data_PWD1;//5A
	tmp_data[0] = Data_PWD0;//A5
	fw_run_flag = hx83113a_write_read_reg(dev, SRAM_ADR_RAWDATA_ADDR, tmp_data, Data_PWD0, Data_PWD1); //A5,5A

	if (fw_run_flag < 0) {
		TOUCH_I("%s Data NOT ready => bypass \n", __func__);
		kfree(temp_info_data);
		return false;
	}

	/* 3. Read RawData */
	total_size_temp = total_size;

	if (total_size % max_i2c_size == 0) {
		total_read_times = total_size / max_i2c_size;
	} else {
		total_read_times = total_size / max_i2c_size + 1;
	}

	for (i = 0; i < total_read_times; i++) {
		address = SRAM_ADR_RAWDATA_ADDR + i * max_i2c_size;
		TOUCH_I("%s address = %08X \n", __func__, address);

		tmp_addr[3] = (uint8_t)((address >> 24) & 0x00FF);
		tmp_addr[2] = (uint8_t)((address >> 16) & 0x00FF);
		tmp_addr[1] = (uint8_t)((address >> 8) & 0x00FF);
		tmp_addr[0] = (uint8_t)((address) & 0x00FF);

		if (total_size_temp >= max_i2c_size) {
			addr = (tmp_addr[3] << 24) + (tmp_addr[2] << 16) + (tmp_addr[1] << 8) + tmp_addr[0];
			hx83113a_register_read(dev, addr, max_i2c_size, &temp_info_data[i * max_i2c_size], ADDR_LEN_4);
			total_size_temp = total_size_temp - max_i2c_size;
			TOUCH_I("addr = 0x%08X\n", addr);
		} else {
			/*TOUCH_I("last total_size_temp=%d\n",total_size_temp);*/
			addr = (tmp_addr[3] << 24) + (tmp_addr[2] << 16) + (tmp_addr[1] << 8) + tmp_addr[0];
			hx83113a_register_read(dev, addr, total_size_temp % max_i2c_size, &temp_info_data[i * max_i2c_size], ADDR_LEN_4);
		}
	}

	/* 4. FW stop outputing */
	/*TOUCH_I("DSRAM_Flag=%d\n",DSRAM_Flag);*/
	w_data[3] = temp_info_data[3];
	w_data[2] = temp_info_data[2];
	w_data[1] = 0x00;
	w_data[0] = 0x00;
	hx83113a_register_write(dev, SRAM_ADR_RAWDATA_ADDR, DATA_LEN_4, w_data, ADDR_LEN_4);

	/* 5. Data Checksum Check */
	for (i = 2; i < total_size; i += 2) { /* 2:PASSWORD NOT included */
		check_sum_cal += (temp_info_data[i + 1] * 256 + temp_info_data[i]);
	}

	if (check_sum_cal % 0x10000 != 0) {
		TOUCH_I("%s check_sum_cal fail=%2X \n", __func__, check_sum_cal);
		kfree(temp_info_data);
		return false;
	} else {
		memcpy(info_data, &temp_info_data[4], mutual_data_size * sizeof(uint8_t));
		/*TOUCH_I("%s checksum PASS \n", __func__);*/
	}

	kfree(temp_info_data);
	return true;
}

bool hx83113a_ts_diag_func(void)
{
	struct device *dev = g_hx_dev;
	struct himax_ts_data *d = to_himax_data(dev);
	int i = 0, j = 0;
	unsigned int index = 0;
	int x_channel = ic_data->HX_RX_NUM;
	int y_channel = ic_data->HX_TX_NUM;
	int total_size = (y_channel * x_channel + y_channel + x_channel) * 2;
	uint8_t *info_data;
	int32_t *mutual_data;
	int32_t *mutual_data_new;
	int32_t *mutual_data_old;
	int32_t *self_data;
	int32_t *self_data_new;
	int32_t *self_data_old;
	int32_t new_data;
	/* 1:common dsram,2:100 frame Max,3:N-(N-1)frame */
	int dsram_type = 0;
	char temp_buf[20];
	char *write_buf; //char write_buf[total_size * 3]; to do

	mutual_data = NULL;
	mutual_data_new = NULL;
	mutual_data_old = NULL;
	self_data = NULL;
	self_data_new = NULL;
	self_data_old = NULL;

	write_buf = kzalloc(sizeof(char) * total_size * 3, GFP_KERNEL);
	if(write_buf == NULL){
        TOUCH_I("%s, alloc fail\n",__func__);
	    return false;
	}
	info_data = kzalloc(total_size * sizeof(uint8_t), GFP_KERNEL);
	if (info_data == NULL)
	{
        TOUCH_I("%s, alloc fail\n",__func__);
	    kfree(write_buf);
		return false;
     }
	memset(write_buf, '\0', sizeof(char) * total_size * 3);
	memset(info_data, 0, total_size * sizeof(uint8_t));
	dsram_type = d->diag_cmd / 10;
	TOUCH_I("%s:Entering ts->diag_cmd=%d!\n", __func__, d->diag_cmd);

	if (dsram_type == 8) {
		dsram_type = 1;
		TOUCH_I("%s Sorting Mode run sram type1 ! \n", __func__);
	}

	hx83113a_burst_enable(dev, 1);

	if (dsram_type == 1 || dsram_type == 2 || dsram_type == 4) {
		mutual_data = getMutualBuffer();
		self_data = getSelfBuffer();
	} else if (dsram_type == 3) {
		mutual_data = getMutualBuffer();
		mutual_data_new = getMutualNewBuffer();
		mutual_data_old = getMutualOldBuffer();
		self_data = getSelfBuffer();
		self_data_new = getSelfNewBuffer();
		self_data_old = getSelfOldBuffer();
	}

	if (hx83113a_get_DSRAM_data(dev, info_data, DSRAM_Flag) == false)
	{
	    kfree(write_buf);
     	kfree(info_data);
		return false;
	}

	index = 0;

	for (i = 0; i < y_channel; i++) { /*mutual data*/
		for (j = 0; j < x_channel; j++) {
			new_data = (((int8_t)info_data[index + 1] << 8) | info_data[index]);

			if (dsram_type == 1 || dsram_type == 4) {
				mutual_data[i * x_channel + j] = new_data;
			} else if (dsram_type == 2) { /* Keep max data */
				if (mutual_data[i * x_channel + j] < new_data)
					mutual_data[i * x_channel + j] = new_data;
			} else if (dsram_type == 3) { /* Cal data for [N]-[N-1] frame */
				mutual_data_new[i * x_channel + j] = new_data;
				mutual_data[i * x_channel + j] = mutual_data_new[i * x_channel + j] - mutual_data_old[i * x_channel + j];
			}
			index += 2;
		}
	}

	for (i = 0; i < x_channel + y_channel; i++) { /*self data*/
		new_data = (info_data[index + 1] << 8 | info_data[index]);
		if (dsram_type == 1 || dsram_type == 4) {
			self_data[i] = new_data;
		} else if (dsram_type == 2) { /* Keep max data */
			if (self_data[i] < new_data)
				self_data[i] = new_data;
		} else if (dsram_type == 3) { /* Cal data for [N]-[N-1] frame */
			self_data_new[i] = new_data;
			self_data[i] = self_data_new[i] - self_data_old[i];
		}
		index += 2;
	}

	kfree(info_data);

	if (dsram_type == 3) {
		memcpy(mutual_data_old, mutual_data_new, x_channel * y_channel * sizeof(int32_t)); /* copy N data to N-1 array */
		memcpy(self_data_old, self_data_new, (x_channel + y_channel) * sizeof(int32_t)); /* copy N data to N-1 array */
	}

	diag_max_cnt++;

	if (dsram_type >= 1 && dsram_type <= 3) {
		queue_delayed_work(private_ts->himax_diag_wq, &private_ts->himax_diag_delay_wrok, 1 / 10 * HZ);
	} else if (dsram_type == 4) {
		for (i = 0; i < x_channel * y_channel; i++) {
			memset(temp_buf, '\0', sizeof(temp_buf));

			if (i == (x_channel * y_channel - 1)) {
				touch_snprintf(temp_buf, sizeof(temp_buf), "%4d\t", mutual_data[i]);
				touch_snprintf(temp_buf, sizeof(temp_buf), "%4d\n", self_data[x_channel + y_channel - 1]);
				TOUCH_I("%s :i = %d 3\n", __func__, i);
			} else if (i % x_channel == (x_channel - 1)) {
				touch_snprintf(temp_buf, sizeof(temp_buf), "%4d\t", mutual_data[i]);
				touch_snprintf(temp_buf, sizeof(temp_buf), "%4d\n", self_data[x_channel + (i / x_channel) + 1]);
			} else {
				touch_snprintf(temp_buf, sizeof(temp_buf), "%4d\t", mutual_data[i]);
			}
			strlcat(&write_buf[i*strlen(temp_buf)], temp_buf, strlen(write_buf));
		}

		for (i = 0; i < x_channel; i++) {
			memset(temp_buf, '\0', sizeof(temp_buf));
			if (i == x_channel - 1)
				touch_snprintf(temp_buf, sizeof(temp_buf), "%4d\n", self_data[i]);
			else
				touch_snprintf(temp_buf, sizeof(temp_buf), "%4d\t", self_data[i]);
			strlcat(&write_buf[(i+x_channel * y_channel)*strlen(temp_buf)], temp_buf, strlen(write_buf));
		}

		/* save raw data in file */
		if (!IS_ERR(diag_sram_fn)) {
			TOUCH_I("%s create file and ready to write\n", __func__);
			diag_sram_fn->f_op->write(diag_sram_fn, write_buf, sizeof(write_buf), &diag_sram_fn->f_pos);
			write_counter++;

			if (write_counter < write_max_count) {
				queue_delayed_work(private_ts->himax_diag_wq, &private_ts->himax_diag_delay_wrok, 1 / 10 * HZ);
			} else {
				filp_close(diag_sram_fn, NULL);
				write_counter = 0;
			}
		}
	}
    kfree(write_buf);
	return true;
}

void hx83113a_reload_disable(struct device *dev, int disable)
{
	uint8_t w_data[DATA_LEN_4];
	int i = 0;

	if (disable) { /*reload disable*/
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(DRIVER_DATA_FW_DEFINE_FLASH_RELOAD_DIS >> i * 8);
		}
	} else { /*reload enable*/
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(DRIVER_DATA_FW_DEFINE_FLASH_RELOAD_EN >> i * 8);
		}
	}
	hx83113a_register_write(dev, DRIVER_ADDR_FW_DEFINE_FLASH_RELOAD, DATA_LEN_4, w_data, ADDR_LEN_4);

	TOUCH_I("%s: setting OK!\n", __func__);
}

static int hx83113a_determin_diag_rawdata(int diag_command)
{
	return diag_command % 10;
}

static int hx83113a_determin_diag_storage(int diag_command)
{
	return diag_command / 10;
}

static void hx83113a_return_event_stack(struct device *dev)
{
	int retry = 20;
	int i = 0;
	uint8_t tmp_data[DATA_LEN_4];
	TOUCH_I("%s:entering\n", __func__);

	do {
		TOUCH_I("now %d times!\n", retry);

		for (i = 0; i < DATA_LEN_4; i++) {
			tmp_data[i] = (uint8_t)(SRAM_ADR_RAWDATA_END >> i * 8);
		}

		hx83113a_register_write(dev, SRAM_ADR_RAWDATA_ADDR, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		hx83113a_register_read(dev, SRAM_ADR_RAWDATA_ADDR, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		retry--;
		touch_msleep(10);
	} while ((tmp_data[1] != 0 && tmp_data[0] != 0) && retry > 0);

	TOUCH_I("%s: End of setting!\n", __func__);
}

static void hx83113a_idle_mode(struct device *dev, int disable)
{
	int retry = 20;
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t switch_cmd = 0x00;
	TOUCH_I("%s:entering\n", __func__);

	do {
		TOUCH_I("%s,now %d times!\n", __func__, retry);
		hx83113a_register_read(dev, FW_ADDR_FW_MODE_STATUS, DATA_LEN_4, tmp_data, ADDR_LEN_4);

		if (disable) {
			switch_cmd = FW_DATA_IDLE_DIS_PWD;
		} else {
			switch_cmd = FW_DATA_IDLE_EN_PWD;
		}

		tmp_data[0] = switch_cmd;
		hx83113a_register_write(dev, FW_ADDR_FW_MODE_STATUS, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		hx83113a_register_read(dev, FW_ADDR_FW_MODE_STATUS, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		TOUCH_I("%s:After turn ON/OFF IDLE Mode [0] = 0x%02X,[1] = 0x%02X,[2] = 0x%02X,[3] = 0x%02X\n",
		  __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		retry--;
		touch_msleep(10);
	} while ((tmp_data[0] != switch_cmd) && retry > 0);

	TOUCH_I("%s: setting OK!\n", __func__);
}

int hx83113a_assign_sorting_mode(struct device *dev, uint8_t *tmp_data)
{

	TOUCH_I("%s:Now tmp_data[3]=0x%02X,tmp_data[2]=0x%02X,tmp_data[1]=0x%02X,tmp_data[0]=0x%02X\n", __func__, tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
	hx83113a_register_write(dev, FW_ADDR_SORTING_MODE_EN, DATA_LEN_4, tmp_data, ADDR_LEN_4);

	return NO_ERR;
}

static int hx83113a_check_sorting_mode(struct device *dev, uint8_t *tmp_data)
{

	hx83113a_register_read(dev, FW_ADDR_SORTING_MODE_EN, DATA_LEN_4, tmp_data, ADDR_LEN_4);
	TOUCH_I("%s: tmp_data[0]=%x,tmp_data[1]=%x\n", __func__, tmp_data[0], tmp_data[1]);

	return NO_ERR;
}

static int hx83113a_switch_mode(struct device *dev, int mode)
{
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t w_data[DATA_LEN_4];
	int i = 0;
	uint8_t mode_wirte_cmd;
	uint8_t mode_read_cmd;
	int result = -1;
	int retry = 200;

	TOUCH_I("%s: Entering\n", __func__);

	if (mode == 0) { /* normal mode */
		mode_wirte_cmd = FW_DATA_NORMAL_CMD;
		mode_read_cmd = FW_DATA_NORMAL_STATUS;
	} else { /* sorting mode */
		mode_wirte_cmd = FW_DATA_SORTING_CMD;
		mode_read_cmd = FW_DATA_SORTING_STATUS;
	}

	hx83113a_sense_off(dev, true);
	/*hx83113a_interface_on(dev);*/
	/* clean up FW status */
	for (i = 0; i < ADDR_LEN_4; i++) {
		w_data[i] = (uint8_t)(SRAM_ADR_RAWDATA_END >> i * 8);
	}
	hx83113a_register_write(dev, SRAM_ADR_RAWDATA_ADDR, DATA_LEN_4, w_data, ADDR_LEN_4);
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = mode_wirte_cmd;
	tmp_data[0] = mode_wirte_cmd;
	hx83113a_assign_sorting_mode(dev, tmp_data);
	hx83113a_idle_mode(dev, 1);
	hx83113a_reload_disable(dev, 1);

	/* To stable the sorting*/
	if (mode) {
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(DRIVER_DATA_FW_DEFINE_RXNUM_TXNUM_MAXPT_SORTING >> i * 8);
		}
		hx83113a_register_write(dev, DRIVER_ADDR_FW_DEFINE_RXNUM_TXNUM_MAXPT, DATA_LEN_4, w_data, ADDR_LEN_4);
	} else {
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(FW_DATA_SET_FRAME >> i * 8);
		}
		hx83113a_register_write(dev, FW_ADDR_SET_FRAME_ADDR, DATA_LEN_4, w_data, ADDR_LEN_4);
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(DRIVER_DATA_FW_DEFINE_RXNUM_TXNUM_MAXPT_NORMAL >> i * 8);
		}
		hx83113a_register_write(dev, DRIVER_ADDR_FW_DEFINE_RXNUM_TXNUM_MAXPT, DATA_LEN_4, w_data, ADDR_LEN_4);
	}

	hx83113a_sense_on(dev, 0x01);

	while (retry != 0) {
		TOUCH_I("[%d] %s Read\n", retry, __func__);
		hx83113a_check_sorting_mode(dev, tmp_data);
		touch_msleep(100);
		TOUCH_I("mode_read_cmd(0)=0x%2.2X,mode_read_cmd(1)=0x%2.2X\n", tmp_data[0], tmp_data[1]);

		if (tmp_data[0] == mode_read_cmd && tmp_data[1] == mode_read_cmd) {
			TOUCH_I("Read OK!\n");
			result = 0;
			break;
		}

		hx83113a_register_read(dev, FW_ADDR_CHK_FW_STATUS, DATA_LEN_4, tmp_data, ADDR_LEN_4);

		if (tmp_data[0] == 0x00 && tmp_data[1] == 0x00 && tmp_data[2] == 0x00 && tmp_data[3] == 0x00) {
			TOUCH_E("%s,: FW Stop!\n", __func__);
			break;
		}

		retry--;
	}

	if (result == 0) {
		if (mode == 0) { /*normal mode*/
			return HX_NORMAL_MODE;
		} else { /*sorting mode*/
			return HX_SORTING_MODE;
		}
	} else { /*change mode fail*/
		return HX_CHANGE_MODE_FAIL;
	}
}

void hx83113a_diag_register_set(struct device *dev, uint8_t diag_command, uint8_t storage_type)
{
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t back_data[DATA_LEN_4];
	uint8_t cnt = 50;

	if (diag_command > 0 && storage_type % 8 > 0)
		tmp_data[0] = diag_command + 0x08;
	else
		tmp_data[0] = diag_command;
	TOUCH_I("diag_command = %d, tmp_data[0] = %X\n", diag_command, tmp_data[0]);
	hx83113a_interface_on(dev);
	tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = 0x00;
	do {
		hx83113a_register_write(dev, FW_ADDR_RAW_OUT_SEL, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		hx83113a_register_read(dev, FW_ADDR_RAW_OUT_SEL, DATA_LEN_4, back_data, ADDR_LEN_4);
		TOUCH_I("%s: back_data[3]=0x%02X,back_data[2]=0x%02X,back_data[1]=0x%02X,back_data[0]=0x%02X!\n",
		  __func__, back_data[3], back_data[2], back_data[1], back_data[0]);
		cnt--;
	} while (tmp_data[0] != back_data[0] && cnt > 0);
}

static ssize_t store_diag(struct file *file, const char *buf,
								 size_t len, loff_t *pos)
{
	char messages[80] = {0};
	struct device *dev = g_hx_dev;
	struct himax_ts_data *d = to_himax_data(dev);
	struct filename *vts_name;
	uint8_t command[2] = {0x00, 0x00};
	uint8_t receive[1];
	/* 0: common , other: dsram*/
	int storage_type = 0;
	/* 1:IIR,2:DC,3:Bank,4:IIR2,5:IIR2_N,6:FIR2,7:Baseline,8:dump coord */
	int rawdata_type = 0;
	memset(receive, 0x00, sizeof(receive));

	if (len >= 80) {
		TOUCH_I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(messages, buf, len)) {
		return -EFAULT;
	}

	TOUCH_I("%s:g_switch_mode = %d\n", __func__, g_switch_mode);

	if (messages[1] == 0x0A) {
		d->diag_cmd = messages[0] - '0';
	} else {
		d->diag_cmd = (messages[0] - '0') * 10 + (messages[1] - '0');
	}

	storage_type = hx83113a_determin_diag_storage(d->diag_cmd);
	rawdata_type = hx83113a_determin_diag_rawdata(d->diag_cmd);

	if (d->diag_cmd > 0 && rawdata_type == 0) {
		TOUCH_I("[Himax]ts->diag_cmd=0x%x ,storage_type=%d, rawdata_type=%d! Maybe no support!\n"
		  , d->diag_cmd, storage_type, rawdata_type);
		d->diag_cmd = 0x00;
	} else {
		TOUCH_I("[Himax]ts->diag_cmd=0x%x ,storage_type=%d, rawdata_type=%d\n", d->diag_cmd, storage_type, rawdata_type);
	}

	memset(diag_mutual, 0x00, ic_data->HX_RX_NUM * ic_data->HX_TX_NUM * sizeof(int32_t)); /* Set data 0 */
	memset(diag_self, 0x00, sizeof(diag_self));
	if (storage_type == 0 && rawdata_type > 0 && rawdata_type < 8) {
		TOUCH_I("%s,common\n", __func__);

		if (DSRAM_Flag) {
			/* 1. Clear DSRAM flag */
			DSRAM_Flag = false;
			/* 2. Stop DSRAM thread */
			cancel_delayed_work(&private_ts->himax_diag_delay_wrok);
			/* 3. Enable ISR */
			touch_interrupt_control(d->dev, INTERRUPT_ENABLE);
			/*(4) FW leave sram and return to event stack*/
			hx83113a_return_event_stack(dev);
		}

		if (g_switch_mode == 2) {
			hx83113a_idle_mode(dev, 0);
			g_switch_mode = hx83113a_switch_mode(dev, 0);
		}

		if (d->diag_cmd == 0x04) {
#if defined(HX_TP_PROC_2T2R)
			command[0] = d->diag_cmd;
#else
			d->diag_cmd = 0x00;
			command[0] = 0x00;
#endif
		} else {
			command[0] = d->diag_cmd;
		}

		hx83113a_diag_register_set(dev, command[0], storage_type);
	} else if (storage_type > 0 && storage_type < 8 && rawdata_type > 0 && rawdata_type < 8) {
		TOUCH_I("%s,dsram\n", __func__);
		diag_max_cnt = 0;

		/* 0. set diag flag */
		if (DSRAM_Flag) {
			/* (1) Clear DSRAM flag */
			DSRAM_Flag = false;
			/* (2) Stop DSRAM thread */
			cancel_delayed_work(&private_ts->himax_diag_delay_wrok);
			/* (3) Enable ISR */
			touch_interrupt_control(d->dev, INTERRUPT_ENABLE);
			/*(4) FW leave sram and return to event stack*/
			hx83113a_return_event_stack(dev);
		}

		/* close sorting if turn on*/
		if (g_switch_mode == 2) {
			hx83113a_idle_mode(dev, 0);
			g_switch_mode = hx83113a_switch_mode(dev, 0);
		}

		command[0] = rawdata_type;/* ts->diag_cmd; */
		hx83113a_diag_register_set(dev, command[0], storage_type);
		/* 1. Disable ISR */
		touch_interrupt_control(d->dev, INTERRUPT_DISABLE);

		/* Open file for save raw data log */
		if (storage_type == 4) {
			switch (rawdata_type) {
			case 1:
				vts_name = getname_kernel(IIR_DUMP_FILE);
				diag_sram_fn = file_open_name(vts_name, O_CREAT | O_WRONLY, 0);
				break;

			case 2:
				vts_name = getname_kernel(DC_DUMP_FILE);
				diag_sram_fn = file_open_name(vts_name, O_CREAT | O_WRONLY, 0);
				break;

			case 3:
				vts_name = getname_kernel(BANK_DUMP_FILE);
				diag_sram_fn = file_open_name(vts_name, O_CREAT | O_WRONLY, 0);
				break;

			default:
				TOUCH_I("%s raw data type is not true. raw data type is %d \n", __func__, rawdata_type);
			}
		}

		/* 2. Start DSRAM thread */
		queue_delayed_work(private_ts->himax_diag_wq, &private_ts->himax_diag_delay_wrok, 2 * HZ / 100);
		TOUCH_I("%s: Start get raw data in DSRAM\n", __func__);

		if (storage_type == 4)
			touch_msleep(6000);

		/* 3. Set DSRAM flag */
		DSRAM_Flag = true;
	} else if (storage_type == 8) {
		TOUCH_I("Soritng mode!\n");

		if (DSRAM_Flag) {
			/* 1. Clear DSRAM flag */
			DSRAM_Flag = false;
			/* 2. Stop DSRAM thread */
			cancel_delayed_work(&private_ts->himax_diag_delay_wrok);
			/* 3. Enable ISR */
			touch_interrupt_control(d->dev, INTERRUPT_ENABLE);
			/*(4) FW leave sram and return to event stack*/
			hx83113a_return_event_stack(dev);
		}

		hx83113a_idle_mode(dev, 1);
		g_switch_mode = hx83113a_switch_mode(dev, 1);

		if (g_switch_mode == 2) {
			hx83113a_diag_register_set(dev, command[0], storage_type);
		}

		queue_delayed_work(private_ts->himax_diag_wq, &private_ts->himax_diag_delay_wrok, 2 * HZ / 100);
		DSRAM_Flag = true;
	} else {
		/* set diag flag */
		if (DSRAM_Flag) {
			TOUCH_I("return and cancel sram thread!\n");
			/* (1) Clear DSRAM flag */
			DSRAM_Flag = false;
			/* (2) Stop DSRAM thread */
			cancel_delayed_work(&private_ts->himax_diag_delay_wrok);
			/* (3) Enable ISR */
			touch_interrupt_control(d->dev, INTERRUPT_ENABLE);
			/*(4) FW leave sram and return to event stack*/
			hx83113a_return_event_stack(dev);
		}

		if (g_switch_mode == 2) {
			hx83113a_idle_mode(dev, 0);
			g_switch_mode = hx83113a_switch_mode(dev, 0);
		}

		if (d->diag_cmd != 0x00) {
			TOUCH_E("[Himax]ts->diag_cmd error!diag_command=0x%x so reset\n", d->diag_cmd);
			command[0] = 0x00;

			if (d->diag_cmd != 0x08)
				d->diag_cmd = 0x00;

			hx83113a_diag_register_set(dev, command[0], storage_type);
		} else {
			command[0] = 0x00;
			d->diag_cmd = 0x00;
			hx83113a_diag_register_set(dev, command[0], storage_type);
			TOUCH_I("return to normal ts->diag_cmd=0x%x\n", d->diag_cmd);
		}
	}
	return len;
}

static struct file_operations himax_proc_diag_ops = {
	.owner = THIS_MODULE,
	.open = open_diag,
	.read = seq_read,
	.write = store_diag,
};

static ssize_t show_SMWP(struct file *file, char *buf,
							   size_t len, loff_t *pos)
{
	size_t count = 0;
	struct himax_ts_data *d = private_ts;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL){
			TOUCH_I("%s, alloc fail\n",__func__);
			return count;
		}

		count = touch_snprintf(temp_buf, len, "%d\n", d->SMWP_enable);

		if (copy_to_user(buf, temp_buf, len))
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t store_SMWP(struct file *file, const char *buff,
								size_t len, loff_t *pos)
{
	struct device *dev = g_hx_dev;
	struct himax_ts_data *d = to_himax_data(dev);
	char buf[80] = {0};

	if (len >= 80) {
		TOUCH_I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	if (buf[0] == '0')
		d->SMWP_enable = 0;
	else if (buf[0] == '1')
		d->SMWP_enable = 1;
	else
		return -EINVAL;

	hx83113a_set_SMWP_enable(dev, d->SMWP_enable);
	HX_SMWP_EN = d->SMWP_enable;
	TOUCH_I("%s: SMART_WAKEUP_enable = %d.\n", __func__, HX_SMWP_EN);
	return len;
}

static struct file_operations himax_proc_SMWP_ops = {
	.owner = THIS_MODULE,
	.read = show_SMWP,
	.write = store_SMWP,
};

static ssize_t store_reset(struct file *file, const char *buff,
								 size_t len, loff_t *pos)
{
	struct device *dev = g_hx_dev;
	char buf_tmp[12];

	if (len >= 12) {
		TOUCH_I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buff, len)) {
		return -EFAULT;
	}

	if (buf_tmp[0] == '1') {
		hx83113a_ic_reset(dev, false, false);
	} else if (buf_tmp[0] == '2') {
		hx83113a_ic_reset(dev, false, true);
	} else if (buf_tmp[0] == '3') {
		hx83113a_ic_reset(dev, true, false);
	} else if (buf_tmp[0] == '4') {
		hx83113a_ic_reset(dev, true, true);
	}

	return len;
}

static struct file_operations himax_proc_reset_ops = {
	.owner = THIS_MODULE,
	.write = store_reset,
};

static int hx83113a_read_ic_trigger_type(struct device *dev)
{
	uint8_t tmp_data[DATA_LEN_4];
	int trigger_type = false;
	hx83113a_register_read(dev, FW_ADDR_TRIGGER_ADDR, DATA_LEN_4, tmp_data, ADDR_LEN_4);

	if ((tmp_data[1] & 0x01) == 1) {
		trigger_type = true;
	}

	return trigger_type;
}

static int hx83113a_read_i2c_status(void)
{
	return i2c_error_count;
}

static ssize_t show_debug(struct file *file, char *buf,
								size_t len, loff_t *pos)
{
	struct device *dev = g_hx_dev;
	struct himax_ts_data *d = to_himax_data(dev);
	size_t ret = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL){
			TOUCH_I("%s, alloc fail\n",__func__);
			return ret;
		}
		if (debug_level_cmd == 't') {
			if (fw_update_complete) {
				ret += touch_snprintf(temp_buf + ret, len - ret, "FW Update Complete ");
			} else {
				ret += touch_snprintf(temp_buf + ret, len - ret, "FW Update Fail ");
			}
		} else if (debug_level_cmd == 'h') {
			if (handshaking_result == 0) {
				ret += touch_snprintf(temp_buf + ret, len - ret, "Handshaking Result = %d (MCU Running)\n", handshaking_result);
			} else if (handshaking_result == 1) {
				ret += touch_snprintf(temp_buf + ret, len - ret, "Handshaking Result = %d (MCU Stop)\n", handshaking_result);
			} else if (handshaking_result == 2) {
				ret += touch_snprintf(temp_buf + ret, len - ret, "Handshaking Result = %d (I2C Error)\n", handshaking_result);
			} else {
				ret += touch_snprintf(temp_buf + ret, len - ret, "Handshaking Result = error \n");
			}
		} else if (debug_level_cmd == 'v') {
			ret += touch_snprintf(temp_buf + ret, len - ret, "FW_VER = 0x%2.2X \n", d->ic_info.vendor_fw_ver);

			if (private_ts->chip_cell_type == CHIP_IS_ON_CELL) {
				ret += touch_snprintf(temp_buf + ret, len - ret, "CONFIG_VER = 0x%2.2X \n", d->ic_info.vendor_config_ver);
			} else {
				ret += touch_snprintf(temp_buf + ret, len - ret, "TOUCH_VER = 0x%2.2X \n", d->ic_info.vendor_touch_cfg_ver);
				ret += touch_snprintf(temp_buf + ret, len - ret, "DISPLAY_VER = 0x%2.2X \n", d->ic_info.vendor_display_cfg_ver);
			}
			if (d->ic_info.vendor_cid_maj_ver < 0 && d->ic_info.vendor_cid_min_ver < 0)
				ret += touch_snprintf(temp_buf + ret, len - ret, "CID_VER = NULL\n");
			else
				ret += touch_snprintf(temp_buf + ret, len - ret, "CID_VER = 0x%2.2X \n", (d->ic_info.vendor_cid_maj_ver << 8 | d->ic_info.vendor_cid_min_ver));

			if (d->ic_info.vendor_panel_ver < 0)
				ret += touch_snprintf(temp_buf + ret, len - ret, "PANEL_VER = NULL\n");
			else
				ret += touch_snprintf(temp_buf + ret, len - ret, "PANEL_VER = 0x%2.2X \n", d->ic_info.vendor_panel_ver);

			ret += touch_snprintf(temp_buf + ret, len - ret, "LGE_FW_VER: %2d.%02d \n", d->ic_info.ic_official_ver, d->ic_info.ic_fw_ver);

			ret += touch_snprintf(temp_buf + ret, len - ret, "\n");
			ret += touch_snprintf(temp_buf + ret, len - ret, "Himax Touch Driver Version:\n");
			ret += touch_snprintf(temp_buf + ret, len - ret, "%s \n", HIMAX_DRIVER_VER);
		} else if (debug_level_cmd == 'd') {
			ret += touch_snprintf(temp_buf + ret, len - ret, "Himax Touch IC Information :\n");
			ret += touch_snprintf(temp_buf + ret, len - ret, "%s \n", private_ts->chip_name);

			switch (IC_CHECKSUM) {
			case HX_TP_BIN_CHECKSUM_SW:
				ret += touch_snprintf(temp_buf + ret, len - ret, "IC Checksum : SW\n");
				break;

			case HX_TP_BIN_CHECKSUM_HW:
				ret += touch_snprintf(temp_buf + ret, len - ret, "IC Checksum : HW\n");
				break;

			case HX_TP_BIN_CHECKSUM_CRC:
				ret += touch_snprintf(temp_buf + ret, len - ret, "IC Checksum : CRC\n");
				break;

			default:
				ret += touch_snprintf(temp_buf + ret, len - ret, "IC Checksum error.\n");
			}

			if (ic_data->HX_INT_IS_EDGE) {
				ret += touch_snprintf(temp_buf + ret, len - ret, "Driver register Interrupt : EDGE TIRGGER\n");
			} else {
				ret += touch_snprintf(temp_buf + ret, len - ret, "Driver register Interrupt : LEVEL TRIGGER\n");
			}
			if (private_ts->protocol_type == PROTOCOL_TYPE_A) {
				ret += touch_snprintf(temp_buf + ret, len - ret, "Protocol : TYPE_A\n");
			} else {
				ret += touch_snprintf(temp_buf + ret, len - ret, "Protocol : TYPE_B\n");
			}

			ret += touch_snprintf(temp_buf + ret, len - ret, "RX Num : %d\n", ic_data->HX_RX_NUM);
			ret += touch_snprintf(temp_buf + ret, len - ret, "TX Num : %d\n", ic_data->HX_TX_NUM);
			ret += touch_snprintf(temp_buf + ret, len - ret, "BT Num : %d\n", ic_data->HX_BT_NUM);
			ret += touch_snprintf(temp_buf + ret, len - ret, "X Resolution : %d\n", ic_data->HX_X_RES);
			ret += touch_snprintf(temp_buf + ret, len - ret, "Y Resolution : %d\n", ic_data->HX_Y_RES);
			ret += touch_snprintf(temp_buf + ret, len - ret, "Max Point : %d\n", ic_data->HX_MAX_PT);
			ret += touch_snprintf(temp_buf + ret, len - ret, "XY reverse : %d\n", ic_data->HX_XY_REVERSE);
#ifdef HX_TP_PROC_2T2R
			if (Is_2T2R) {
				ret += touch_snprintf(temp_buf + ret, len - ret, "2T2R panel\n");
				ret += touch_snprintf(temp_buf + ret, len - ret, "RX Num_2 : %d\n", HX_RX_NUM_2);
				ret += touch_snprintf(temp_buf + ret, len - ret, "TX Num_2 : %d\n", HX_TX_NUM_2);
			}
#endif
		} else if (debug_level_cmd == 'i') {
			if (hx83113a_read_i2c_status())
				ret += touch_snprintf(temp_buf + ret, len - ret, "I2C communication is bad.\n");
			else
				ret += touch_snprintf(temp_buf + ret, len - ret, "I2C communication is good.\n");
		} else if (debug_level_cmd == 'n') {
			if (hx83113a_read_ic_trigger_type(dev) == 1) /* Edgd = 1, Level = 0 */
				ret += touch_snprintf(temp_buf + ret, len - ret, "IC Interrupt type is edge trigger.\n");
			else if (hx83113a_read_ic_trigger_type(dev) == 0)
				ret += touch_snprintf(temp_buf + ret, len - ret, "IC Interrupt type is level trigger.\n");
			else
				ret += touch_snprintf(temp_buf + ret, len - ret, "Unkown IC trigger type.\n");

			if (ic_data->HX_INT_IS_EDGE)
				ret += touch_snprintf(temp_buf + ret, len - ret, "Driver register Interrupt : EDGE TIRGGER\n");
			else
				ret += touch_snprintf(temp_buf + ret, len - ret, "Driver register Interrupt : LEVEL TRIGGER\n");
		}

		if (copy_to_user(buf, temp_buf, len))
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

extern int g_ts_dbg;
static ssize_t store_debug(struct file *file, const char *buff,
								 size_t len, loff_t *pos)
{
	struct device *dev = g_hx_dev;
	struct himax_ts_data *d = to_himax_data(dev);
	char fileName[128];
	char buf[80] = {0};
	int result = 0;
	int fw_type = 0;
	const struct firmware *fw = NULL;

	if (len >= 80) {
		TOUCH_I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	if (buf[0] == 'v') { /* firmware version */
		touch_interrupt_control(d->dev, INTERRUPT_DISABLE);
		hx83113a_ic_reset(dev, false, false);
		debug_level_cmd = buf[0];
		hx83113a_read_FW_ver(dev);
		hx83113a_ic_reset(dev, true, false);
		touch_interrupt_control(d->dev, INTERRUPT_ENABLE);
		/* himax_check_chip_version(); */
		return len;
	} else if (buf[0] == 'd') { /* ic information */
		debug_level_cmd = buf[0];
		return len;
	} else if (buf[0] == 't') {
		if (buf[1] == 's' &&
		    buf[2] == 'd' &&
		    buf[3] == 'b' &&
		    buf[4] == 'g'
		) {
			if (buf[5] == '1') {
				TOUCH_I("Open Ts Debug!\n");
				g_ts_dbg = 1;
			} else if (buf[5] == '0') {
				TOUCH_I("Close Ts Debug!\n");
				g_ts_dbg = 0;
			} else {
				TOUCH_E("Parameter fault for ts debug\n");
			}
			goto ENDFUCTION;
		}
		touch_interrupt_control(d->dev, INTERRUPT_DISABLE);
		debug_level_cmd			= buf[0];
		fw_update_complete		= false;
		memset(fileName, 0, sizeof(fileName));
		/* parse the file name */
		touch_snprintf(fileName, sizeof(fileName) - 2, "%s", &buf[2]);

		TOUCH_I("NOW Running common flow update!\n");
		TOUCH_I("%s: upgrade from file(%s) start!\n", __func__, fileName);
		result = request_firmware(&fw, fileName, private_ts->dev);

		if (result < 0) {
			TOUCH_I("fail to request_firmware fwpath: %s (ret:%d)\n", fileName, result);
			return result;
		}

		TOUCH_I("%s: FW image: %02X, %02X, %02X, %02X\n", __func__, fw->data[0], fw->data[1], fw->data[2], fw->data[3]);
		fw_type = (fw->size) / 1024;
		/*	start to upgrade */
		touch_interrupt_control(d->dev, INTERRUPT_DISABLE);
		TOUCH_I("Now FW size is : %dk\n", fw_type);

		switch (fw_type) {
		case 128:
			if (hx83113a_fts_ctpm_fw_upgrade_with_sys_fs_128k(dev, (unsigned char *)fw->data, fw->size) == 0) {
				TOUCH_E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
				fw_update_complete = false;
			} else {
				TOUCH_I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
				fw_update_complete = true;
			}
			break;

		default:
			TOUCH_E("%s: Flash command fail: %d\n", __func__, __LINE__);
			fw_update_complete = false;
			break;
		}
		release_firmware(fw);
		goto firmware_upgrade_done;
	} else if (buf[0] == 'i' && buf[1] == '2' && buf[2] == 'c') { /* i2c commutation */
		debug_level_cmd = 'i';
		return len;
	} else if (buf[0] == 'i' && buf[1] == 'n' && buf[2] == 't') { /* INT trigger */
		debug_level_cmd = 'n';
		return len;
	} else { /* others,do nothing */
		debug_level_cmd = 0;
		return len;
	}


firmware_upgrade_done:
	hx83113a_reload_disable(dev, 0);
	hx83113a_read_FW_ver(dev);
	hx83113a_touch_information(dev);
	hx83113a_ic_reset(dev, true, false);

	touch_interrupt_control(d->dev, INTERRUPT_ENABLE);
/*	todo himax_chip->tp_firmware_upgrade_proceed = 0;
	todo himax_chip->suspend_state = 0;
	todo enable_irq(himax_chip->irq); */
ENDFUCTION:
	return len;
}

static struct file_operations himax_proc_debug_ops = {
	.owner = THIS_MODULE,
	.read = show_debug,
	.write = store_debug,
};

static int hx83113a_read_FW_status(struct device *dev, uint8_t *state_addr, uint8_t *tmp_addr)
{
	uint8_t i;
	uint8_t req_size = 0;
	uint8_t status_addr[DATA_LEN_4];
	uint8_t cmd_addr[DATA_LEN_4];
	uint8_t temp_addr[ADDR_LEN_4];
	int addr = 0;

	temp_addr[0] = (uint8_t)FW_ADDR_FW_DBG_MSG_ADDR;
	temp_addr[1] = (uint8_t)(FW_ADDR_FW_DBG_MSG_ADDR >> 8);
	temp_addr[2] = (uint8_t)(FW_ADDR_FW_DBG_MSG_ADDR >> 16);
	temp_addr[3] = (uint8_t)(FW_ADDR_FW_DBG_MSG_ADDR >> 24);

	if (state_addr[0] == 0x01) {
		state_addr[1] = 0x04;

		for (i = 0; i < DATA_LEN_4; i++) {
			state_addr[i + 2] = temp_addr[i];
			status_addr[i] = temp_addr[i];
		}

		req_size = 0x04;
		addr = (status_addr[3] << 24) + (status_addr[2] << 16) + (status_addr[1] << 8) + status_addr[0];

		hx83113a_register_read(dev, addr, req_size, tmp_addr, ADDR_LEN_4);
	} else if (state_addr[0] == 0x02) {
		state_addr[1] = 0x30;

		for (i = 0; i < DATA_LEN_4; i++) {
			state_addr[i + 2] = temp_addr[i];
			cmd_addr[i] = temp_addr[i];
		}

		req_size = 0x30;
		addr = (cmd_addr[3] << 24) + (cmd_addr[2] << 16) + (cmd_addr[1] << 8) + cmd_addr[0];
		hx83113a_register_read(dev, addr, req_size, tmp_addr, ADDR_LEN_4);
	}

	return NO_ERR;
}

static ssize_t show_FW_debug(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	struct device *dev = g_hx_dev;
	ssize_t ret = 0;
	uint8_t loop_i = 0;
	uint8_t tmp_data[64];
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL){
			TOUCH_I("%s, alloc fail\n",__func__);
			return ret;
		}
		cmd_set[0] = 0x01;

		if (hx83113a_read_FW_status(dev, cmd_set, tmp_data) == NO_ERR) {
			ret += touch_snprintf(temp_buf + ret, len - ret, "0x%02X%02X%02X%02X :\t", cmd_set[5], cmd_set[4], cmd_set[3], cmd_set[2]);

			for (loop_i = 0; loop_i < cmd_set[1]; loop_i++) {
				ret += touch_snprintf(temp_buf + ret, len - ret, "%5d\t", tmp_data[loop_i]);
			}

			ret += touch_snprintf(temp_buf + ret, len - ret, "\n");
		}

		cmd_set[0] = 0x02;

		if (hx83113a_read_FW_status(dev, cmd_set, tmp_data) == NO_ERR) {
			for (loop_i = 0; loop_i < cmd_set[1]; loop_i = loop_i + 2) {
				if ((loop_i % 16) == 0)
					ret += touch_snprintf(temp_buf + ret, len - ret, "0x%02X%02X%02X%02X :\t",
									cmd_set[5], cmd_set[4], cmd_set[3] + (((cmd_set[2] + loop_i) >> 8) & 0xFF), (cmd_set[2] + loop_i) & 0xFF);

				ret += touch_snprintf(temp_buf + ret, len - ret, "%5d\t", tmp_data[loop_i] + (tmp_data[loop_i + 1] << 8));

				if ((loop_i % 16) == 14)
					ret += touch_snprintf(temp_buf + ret, len - ret, "\n");
			}
		}

		ret += touch_snprintf(temp_buf + ret, len - ret, "\n");

		if (copy_to_user(buf, temp_buf, len))
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static struct file_operations himax_proc_fw_debug_ops = {
	.owner = THIS_MODULE,
	.read = show_FW_debug,
};

uint8_t hx83113a_read_DD_status(struct device *dev, uint8_t *cmd_set, uint8_t *tmp_data)
{
	int cnt = 0;
	uint8_t req_size = cmd_set[0];
	cmd_set[3] = FW_DATA_DD_REQUEST;
	hx83113a_register_write(dev, FW_ADDR_DD_HANDSHAK_ADDR, DATA_LEN_4, cmd_set, ADDR_LEN_4);
	TOUCH_I("%s: cmd_set[0] = 0x%02X,cmd_set[1] = 0x%02X,cmd_set[2] = 0x%02X,cmd_set[3] = 0x%02X\n",
	  __func__, cmd_set[0], cmd_set[1], cmd_set[2], cmd_set[3]);

	/* Doing hand shaking 0xAA -> 0xBB */
	for (cnt = 0; cnt < 100; cnt++) {
		hx83113a_register_read(dev, FW_ADDR_DD_HANDSHAK_ADDR, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		touch_msleep(10);

		if (tmp_data[3] == FW_DATA_DD_ACK) {
			TOUCH_I("%s Data ready goto moving data\n", __func__);
			break;
		} else {
			if (cnt >= 99) {
				TOUCH_I("%s Data not ready in FW \n", __func__);
				return FW_NOT_READY;
			}
		}
	}

	hx83113a_register_read(dev, FW_ADDR_DD_DATA_ADDR, req_size, tmp_data, ADDR_LEN_4);
	return NO_ERR;
}

static ssize_t show_DD_debug(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	struct device *dev = g_hx_dev;
	ssize_t ret = 0;
	uint8_t tmp_data[64];
	uint8_t loop_i = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL){
			TOUCH_I("%s, alloc fail\n",__func__);
			return ret;
		}
		if (mutual_set_flag == 1) {
			if (hx83113a_read_DD_status(dev, cmd_set, tmp_data) == NO_ERR) {
				for (loop_i = 0; loop_i < cmd_set[0]; loop_i++) {
					if ((loop_i % 8) == 0)
						ret += touch_snprintf(temp_buf + ret, len - ret, "0x%02X : ", loop_i);

					ret += touch_snprintf(temp_buf + ret, len - ret, "0x%02X ", tmp_data[loop_i]);

					if ((loop_i % 8) == 7)
						ret += touch_snprintf(temp_buf + ret, len - ret, "\n");
				}
			}
		}

		ret += touch_snprintf(temp_buf + ret, len - ret, "\n");

		if (copy_to_user(buf, temp_buf, len))
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t store_DD_debug(struct file *file, const char *buff,
		size_t len, loff_t *pos)
{
	uint8_t i = 0;
	uint8_t cnt = 2;
	unsigned long result = 0;
	char buf_tmp[20];
	char buf_tmp2[4];

	if (len >= 20) {
		TOUCH_I("%s: no command exceeds 20 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buff, len)) {
		return -EFAULT;
	}

	memset(buf_tmp2, 0x0, sizeof(buf_tmp2));

	if (buf_tmp[2] == 'x' && buf_tmp[6] == 'x' && buf_tmp[10] == 'x') {
		mutual_set_flag = 1;

		for (i = 3; i < 12; i = i + 4) {
			memcpy(buf_tmp2, buf_tmp + i, 2);

			if (!kstrtoul(buf_tmp2, 16, &result))
				cmd_set[cnt] = (uint8_t)result;
			else
				TOUCH_I("String to oul is fail in cnt = %d, buf_tmp2 = %s\n", cnt, buf_tmp2);

			cnt--;
		}

		TOUCH_I("cmd_set[2] = %02X, cmd_set[1] = %02X, cmd_set[0] = %02X\n", cmd_set[2], cmd_set[1], cmd_set[0]);
	} else {
		mutual_set_flag = 0;
	}

	return len;
}

static struct file_operations himax_proc_dd_debug_ops = {
	.owner = THIS_MODULE,
	.read = show_DD_debug,
	.write = store_DD_debug,
};

uint8_t getFlashCommand(void)
{
	return flash_command;
}

static uint8_t getFlashDumpProgress(void)
{
	return flash_progress;
}

static uint8_t getFlashDumpComplete(void)
{
	return flash_dump_complete;
}

static uint8_t getFlashDumpFail(void)
{
	return flash_dump_fail;
}

uint8_t getSysOperation(void)
{
	return sys_operation;
}

static uint8_t getFlashReadStep(void)
{
	return flash_read_step;
}

void setFlashBuffer(void)
{
	flash_buffer = kzalloc(Flash_Size * sizeof(uint8_t), GFP_KERNEL);
}

void setSysOperation(uint8_t operation)
{
	sys_operation = operation;
}

void setFlashDumpProgress(uint8_t progress)
{
	flash_progress = progress;
	/* TOUCH_I("setFlashDumpProgress : progress = %d ,flash_progress = %d \n",progress,flash_progress); */
}

void setFlashDumpComplete(uint8_t status)
{
	flash_dump_complete = status;
}

void setFlashDumpFail(uint8_t fail)
{
	flash_dump_fail = fail;
}

static void setFlashCommand(uint8_t command)
{
	flash_command = command;
}

static void setFlashReadStep(uint8_t step)
{
	flash_read_step = step;
}

void setFlashDumpGoing(bool going)
{
	flash_dump_going = going;
	debug_data->flash_dump_going = going;
}

static ssize_t show_flash(struct file *file, char *buf,
									 size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	int loop_i;
	uint8_t local_flash_read_step = 0;
	uint8_t local_flash_complete = 0;
	uint8_t local_flash_progress = 0;
	uint8_t local_flash_command = 0;
	uint8_t local_flash_fail = 0;
	char *temp_buf;
	local_flash_complete = getFlashDumpComplete();
	local_flash_progress = getFlashDumpProgress();
	local_flash_command = getFlashCommand();
	local_flash_fail = getFlashDumpFail();
	TOUCH_I("flash_progress = %d \n", local_flash_progress);

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL ){
			TOUCH_I("%s, alloc fail\n",__func__);
		   	return ret;
		}
		if (local_flash_fail) {
			ret += touch_snprintf(temp_buf + ret, len - ret, "FlashStart:Fail \n");
			ret += touch_snprintf(temp_buf + ret, len - ret, "FlashEnd");
			ret += touch_snprintf(temp_buf + ret, len - ret, "\n");

			if (copy_to_user(buf, temp_buf, len))
				TOUCH_I("%s,here:%d\n", __func__, __LINE__);

			kfree(temp_buf);
			HX_PROC_SEND_FLAG = 1;
			return ret;
		}

		if (!local_flash_complete) {
			ret += touch_snprintf(temp_buf + ret, len - ret, "FlashStart:Ongoing:0x%2.2x \n", flash_progress);
			ret += touch_snprintf(temp_buf + ret, len - ret, "FlashEnd");
			ret += touch_snprintf(temp_buf + ret, len - ret, "\n");

			if (copy_to_user(buf, temp_buf, len))
				TOUCH_I("%s,here:%d\n", __func__, __LINE__);

			kfree(temp_buf);
			HX_PROC_SEND_FLAG = 1;
			return ret;
		}

		if (local_flash_command == 1 && local_flash_complete) {
			ret += touch_snprintf(temp_buf + ret, len - ret, "FlashStart:Complete \n");
			ret += touch_snprintf(temp_buf + ret, len - ret, "FlashEnd");
			ret += touch_snprintf(temp_buf + ret, len - ret, "\n");

			if (copy_to_user(buf, temp_buf, len))
				TOUCH_I("%s,here:%d\n", __func__, __LINE__);

			kfree(temp_buf);
			HX_PROC_SEND_FLAG = 1;
			return ret;
		}

		if (local_flash_command == 3 && local_flash_complete) {
			ret += touch_snprintf(temp_buf + ret, len - ret, "FlashStart: \n");

			for (loop_i = 0; loop_i < 128; loop_i++) {
				ret += touch_snprintf(temp_buf + ret, len - ret, "x%2.2x", flash_buffer[loop_i]);

				if ((loop_i % 16) == 15) {
					ret += touch_snprintf(temp_buf + ret, len - ret, "\n");
				}
			}

			ret += touch_snprintf(temp_buf + ret, len - ret, "FlashEnd");
			ret += touch_snprintf(temp_buf + ret, len - ret, "\n");

			if (copy_to_user(buf, temp_buf, len))
				TOUCH_I("%s,here:%d\n", __func__, __LINE__);

			kfree(temp_buf);
			HX_PROC_SEND_FLAG = 1;
			return ret;
		}

		/* flash command == 0 , report the data */
		local_flash_read_step = getFlashReadStep();
		ret += touch_snprintf(temp_buf + ret, len - ret, "FlashStart:%2.2x \n", local_flash_read_step);

		for (loop_i = 0; loop_i < 1024; loop_i++) {
			ret += touch_snprintf(temp_buf + ret, len - ret, "x%2.2X", flash_buffer[local_flash_read_step * 1024 + loop_i]);

			if ((loop_i % 16) == 15) {
				ret += touch_snprintf(temp_buf + ret, len - ret, "\n");
			}
		}

		ret += touch_snprintf(temp_buf + ret, len - ret, "FlashEnd");
		ret += touch_snprintf(temp_buf + ret, len - ret, "\n");

		if (copy_to_user(buf, temp_buf, len))
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t store_flash(struct file *file, const char *buff,
									  size_t len, loff_t *pos)
{
	char buf_tmp[6];
	unsigned long result = 0;
	char buf[80] = {0};

	if (len >= 80) {
		TOUCH_I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	TOUCH_I("%s: buf = %s\n", __func__, buf);

	if (getSysOperation() == 1) {
		TOUCH_E("%s: PROC is busy , return!\n", __func__);
		return len;
	}

	if (buf[0] == '0') {
		setFlashCommand(0);

		if (buf[1] == ':' && buf[2] == 'x') {
			memcpy(buf_tmp, buf + 3, 2);
			TOUCH_I("%s: read_Step = %s\n", __func__, buf_tmp);

			if (!kstrtoul(buf_tmp, 16, &result)) {
				TOUCH_I("%s: read_Step = %lu \n", __func__, result);
				setFlashReadStep(result);
			}
		}
	} else if (buf[0] == '1') { /*	 1_32,1_60,1_64,1_24,1_28 for flash size 32k,60k,64k,124k,128k */
		setSysOperation(1);
		setFlashCommand(1);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		if ((buf[1] == '_') && (buf[2] == '3') && (buf[3] == '2')) {
			Flash_Size = FW_SIZE_32k;
		} else if ((buf[1] == '_') && (buf[2] == '6')) {
			if (buf[3] == '0') {
				Flash_Size = FW_SIZE_60k;
			} else if (buf[3] == '4') {
				Flash_Size = FW_SIZE_64k;
			}
		} else if ((buf[1] == '_') && (buf[2] == '2')) {
			if (buf[3] == '4') {
				Flash_Size = FW_SIZE_124k;
			} else if (buf[3] == '8') {
				Flash_Size = FW_SIZE_128k;
			}
		}
		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	} else if (buf[0] == '2') { /*	 2_32,2_60,2_64,2_24,2_28 for flash size 32k,60k,64k,124k,128k */
		setSysOperation(1);
		setFlashCommand(2);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		if ((buf[1] == '_') && (buf[2] == '3') && (buf[3] == '2')) {
			Flash_Size = FW_SIZE_32k;
		} else if ((buf[1] == '_') && (buf[2] == '6')) {
			if (buf[3] == '0') {
				Flash_Size = FW_SIZE_60k;
			} else if (buf[3] == '4') {
				Flash_Size = FW_SIZE_64k;
			}
		} else if ((buf[1] == '_') && (buf[2] == '2')) {
			if (buf[3] == '4') {
				Flash_Size = FW_SIZE_124k;
			} else if (buf[3] == '8') {
				Flash_Size = FW_SIZE_128k;
			}
		}
		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}

	return len;
}

static struct file_operations himax_proc_flash_ops = {
	.owner = THIS_MODULE,
	.read = show_flash,
	.write = store_flash,
};

static void hx83113a_flash_dump_func(struct device *dev, uint8_t local_flash_command, int Flash_Size, uint8_t *flash_buffer)
{
	uint8_t tmp_addr[DATA_LEN_4];
	uint8_t buffer[256];
	int page_prog_start = 0;
	int addr = 0;

	hx83113a_sense_off(dev, true);
	hx83113a_burst_enable(dev, 1);

	for (page_prog_start = 0; page_prog_start < Flash_Size; page_prog_start += 128) {
		tmp_addr[0] = page_prog_start % 0x100;
		tmp_addr[1] = (page_prog_start >> 8) % 0x100;
		tmp_addr[2] = (page_prog_start >> 16) % 0x100;
		tmp_addr[3] = page_prog_start / 0x1000000;
		addr = (tmp_addr[3] << 24) + (tmp_addr[2] << 16) + (tmp_addr[1] << 8) + tmp_addr[0];

		hx83113a_register_read(dev, addr, 128, buffer, ADDR_LEN_4);
		memcpy(&flash_buffer[page_prog_start], buffer, 128);
	}

	hx83113a_burst_enable(dev, 0);
	hx83113a_sense_on(dev, 0x01);
}

void hx83113a_ts_flash_func(void)
{
	struct device *dev = g_hx_dev;
	struct himax_ts_data *d = to_himax_data(dev);
	uint8_t local_flash_command = 0;
	touch_interrupt_control(d->dev, INTERRUPT_DISABLE);
	setFlashDumpGoing(true);
	/* sector = getFlashDumpSector(); */
	/* page = getFlashDumpPage(); */
	local_flash_command = getFlashCommand();
	touch_msleep(100);
	TOUCH_I("%s: local_flash_command = %d enter.\n", __func__, local_flash_command);

	if ((local_flash_command == 1 || local_flash_command == 2) || (local_flash_command == 0x0F)) {
		hx83113a_flash_dump_func(dev, local_flash_command, Flash_Size, flash_buffer);
	}

	TOUCH_I("Complete~~~~~~~~~~~~~~~~~~~~~~~\n");

	if (local_flash_command == 2) {
		struct file *fn;
		struct filename *vts_name;
		vts_name = getname_kernel(FLASH_DUMP_FILE);
		fn = file_open_name(vts_name, O_CREAT | O_WRONLY, 0);

		if (!IS_ERR(fn)) {
			TOUCH_I("%s create file and ready to write\n", __func__);
			fn->f_op->write(fn, flash_buffer, Flash_Size * sizeof(uint8_t), &fn->f_pos);
			filp_close(fn, NULL);
		}
	}

	touch_interrupt_control(d->dev, INTERRUPT_ENABLE);
	setFlashDumpGoing(false);
	setFlashDumpComplete(1);
	setSysOperation(0);
	return;
}



static ssize_t store_sense_on_off(struct file *file, const char *buff,
										size_t len, loff_t *pos)
{
	struct device *dev = g_hx_dev;
	char buf[80] = {0};

	if (len >= 80) {
		TOUCH_I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	if (buf[0] == '0') {
		hx83113a_sense_off(dev, true);
		TOUCH_I("Sense off \n");
	} else if (buf[0] == '1') {
		if (buf[1] == 's') {
			hx83113a_sense_on(dev, 0x00);
			TOUCH_I("Sense on re-map on, run sram \n");
		} else {
			hx83113a_sense_on(dev, 0x01);
			TOUCH_I("Sense on re-map off, run flash \n");
		}
	} else {
		TOUCH_I("Do nothing \n");
	}

	return len;
}

static struct file_operations himax_proc_sense_on_off_ops = {
	.owner = THIS_MODULE,
	.write = store_sense_on_off,
};

static ssize_t show_esd_cnt(struct file *file, char *buf,
								  size_t len, loff_t *pos)
{
	size_t ret = 0;
	char *temp_buf;
	TOUCH_I("%s: enter, %d \n", __func__, __LINE__);

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(temp_buf == NULL){
			TOUCH_I("%s, alloc fail\n",__func__);
		   	return ret;
		}
		ret += touch_snprintf(temp_buf + ret, len - ret, "EB_cnt = %d, EC_cnt = %d, ED_cnt = %d\n", hx_EB_event_flag, hx_EC_event_flag, hx_ED_event_flag);

		if (copy_to_user(buf, temp_buf, len))
			TOUCH_I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t store_esd_cnt(struct file *file, const char *buff,
								   size_t len, loff_t *pos)
{
	int i = 0;
	char buf[12] = {0};

	if (len >= 12) {
		TOUCH_I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	TOUCH_I("Clear ESD Flag \n");

	if (buf[i] == '0') {
		hx_EB_event_flag = 0;
		hx_EC_event_flag = 0;
		hx_ED_event_flag = 0;
	}

	return len;
}

static struct file_operations himax_proc_esd_cnt_ops = {
	.owner = THIS_MODULE,
	.read = show_esd_cnt,
	.write = store_esd_cnt,
};

static void hx83113a_dbg_data_init(void)
{
	debug_data->fp_ts_dbg_func = hx83113a_ts_dbg_func;
	debug_data->fp_set_diag_cmd = hx83113a_set_diag_cmd;
	debug_data->flash_dump_going = false;
}

static void hx83113a_ts_flash_work_func(struct work_struct *work)
{
	hx83113a_ts_flash_func();
}

static void hx83113a_ts_diag_work_func(struct work_struct *work)
{
	hx83113a_ts_diag_func();
}

int hx83113a_touch_proc_init(void)
{
	himax_proc_debug_level_file = proc_create(HIMAX_PROC_DEBUG_LEVEL_FILE, (S_IWUSR | S_IRUGO),
								  himax_touch_proc_dir, &himax_proc_debug_level_ops);
	if (himax_proc_debug_level_file == NULL) {
		TOUCH_E(" %s: proc debug_level file create failed!\n", __func__);
		goto fail_1;
	}

	himax_proc_vendor_file = proc_create(HIMAX_PROC_VENDOR_FILE, (S_IRUGO),
										 himax_touch_proc_dir, &himax_proc_vendor_ops);
	if (himax_proc_vendor_file == NULL) {
		TOUCH_E(" %s: proc vendor file create failed!\n", __func__);
		goto fail_2;
	}

	himax_proc_attn_file = proc_create(HIMAX_PROC_ATTN_FILE, (S_IRUGO),
									   himax_touch_proc_dir, &himax_proc_attn_ops);
	if (himax_proc_attn_file == NULL) {
		TOUCH_E(" %s: proc attn file create failed!\n", __func__);
		goto fail_3;
	}

	himax_proc_int_en_file = proc_create(HIMAX_PROC_INT_EN_FILE, (S_IWUSR | S_IRUGO),
										 himax_touch_proc_dir, &himax_proc_int_en_ops);
	if (himax_proc_int_en_file == NULL) {
		TOUCH_E(" %s: proc int en file create failed!\n", __func__);
		goto fail_4;
	}

	himax_proc_layout_file = proc_create(HIMAX_PROC_LAYOUT_FILE, (S_IWUSR | S_IRUGO),
										 himax_touch_proc_dir, &himax_proc_layout_ops);
	if (himax_proc_layout_file == NULL) {
		TOUCH_E(" %s: proc layout file create failed!\n", __func__);
		goto fail_5;
	}

	himax_proc_reset_file = proc_create(HIMAX_PROC_RESET_FILE, (S_IWUSR),
										himax_touch_proc_dir, &himax_proc_reset_ops);
	if (himax_proc_reset_file == NULL) {
		TOUCH_E(" %s: proc reset file create failed!\n", __func__);
		goto fail_6;
	}

	himax_proc_SMWP_file = proc_create(HIMAX_PROC_SMWP_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
									   himax_touch_proc_dir, &himax_proc_SMWP_ops);

	if (himax_proc_SMWP_file == NULL) {
		TOUCH_E(" %s: proc SMWP file create failed!\n", __func__);
		goto fail_6_1;
	}

	himax_proc_diag_file = proc_create(HIMAX_PROC_DIAG_FILE, (S_IWUSR | S_IRUGO),
									   himax_touch_proc_dir, &himax_proc_diag_ops);
	if (himax_proc_diag_file == NULL) {
		TOUCH_E(" %s: proc diag file create failed!\n", __func__);
		goto fail_7;
	}

	himax_proc_diag_arrange_file = proc_create(HIMAX_PROC_DIAG_ARR_FILE, (S_IWUSR | S_IRUGO),
								   himax_touch_proc_dir, &himax_proc_diag_arrange_ops);
	if (himax_proc_diag_arrange_file == NULL) {
		TOUCH_E(" %s: proc diag file create failed!\n", __func__);
		goto fail_7_1;
	}

	himax_proc_register_file = proc_create(HIMAX_PROC_REGISTER_FILE, (S_IWUSR | S_IRUGO),
										   himax_touch_proc_dir, &himax_proc_register_ops);
	if (himax_proc_register_file == NULL) {
		TOUCH_E(" %s: proc register file create failed!\n", __func__);
		goto fail_8;
	}

	himax_proc_debug_file = proc_create(HIMAX_PROC_DEBUG_FILE, (S_IWUSR | S_IRUGO),
										himax_touch_proc_dir, &himax_proc_debug_ops);
	if (himax_proc_debug_file == NULL) {
		TOUCH_E(" %s: proc debug file create failed!\n", __func__);
		goto fail_9;
	}

	himax_proc_fw_debug_file = proc_create(HIMAX_PROC_FW_DEBUG_FILE, (S_IWUSR | S_IRUGO),
										   himax_touch_proc_dir, &himax_proc_fw_debug_ops);
	if (himax_proc_fw_debug_file == NULL) {
		TOUCH_E(" %s: proc fw debug file create failed!\n", __func__);
		goto fail_9_1;
	}

	himax_proc_dd_debug_file = proc_create(HIMAX_PROC_DD_DEBUG_FILE, (S_IWUSR | S_IRUGO),
										   himax_touch_proc_dir, &himax_proc_dd_debug_ops);
	if (himax_proc_dd_debug_file == NULL) {
		TOUCH_E(" %s: proc DD debug file create failed!\n", __func__);
		goto fail_9_2;
	}

	himax_proc_flash_dump_file = proc_create(HIMAX_PROC_FLASH_DUMP_FILE, (S_IWUSR | S_IRUGO),
								 himax_touch_proc_dir, &himax_proc_flash_ops);
	if (himax_proc_flash_dump_file == NULL) {
		TOUCH_E(" %s: proc flash dump file create failed!\n", __func__);
		goto fail_10;
	}

	himax_proc_SENSE_ON_OFF_file = proc_create(HIMAX_PROC_SENSE_ON_OFF_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
								   himax_touch_proc_dir, &himax_proc_sense_on_off_ops);
	if (himax_proc_SENSE_ON_OFF_file == NULL) {
		TOUCH_E(" %s: proc SENSE_ON_OFF file create failed!\n", __func__);
		goto fail_16;
	}

	himax_proc_ESD_cnt_file = proc_create(HIMAX_PROC_ESD_CNT_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
										  himax_touch_proc_dir, &himax_proc_esd_cnt_ops);

	if (himax_proc_ESD_cnt_file == NULL) {
		TOUCH_E(" %s: proc ESD cnt file create failed!\n", __func__);
		goto fail_17;
	}

	himax_proc_CRC_test_file = proc_create(HIMAX_PROC_CRC_TEST_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
										   himax_touch_proc_dir, &himax_proc_CRC_test_ops);

	if (himax_proc_CRC_test_file == NULL) {
		TOUCH_E(" %s: proc CRC test file create failed!\n", __func__);
		goto fail_18;
	}

	return 0 ;
fail_18: remove_proc_entry(HIMAX_PROC_ESD_CNT_FILE, himax_touch_proc_dir);
fail_17: remove_proc_entry(HIMAX_PROC_SENSE_ON_OFF_FILE, himax_touch_proc_dir);
fail_16: remove_proc_entry(HIMAX_PROC_FLASH_DUMP_FILE, himax_touch_proc_dir);
fail_10: remove_proc_entry(HIMAX_PROC_DD_DEBUG_FILE, himax_touch_proc_dir);
fail_9_2: remove_proc_entry(HIMAX_PROC_FW_DEBUG_FILE, himax_touch_proc_dir);
fail_9_1: remove_proc_entry(HIMAX_PROC_DEBUG_FILE, himax_touch_proc_dir);
fail_9: remove_proc_entry(HIMAX_PROC_REGISTER_FILE, himax_touch_proc_dir);
fail_8:	remove_proc_entry(HIMAX_PROC_DIAG_ARR_FILE, himax_touch_proc_dir);
fail_7_1: remove_proc_entry(HIMAX_PROC_DIAG_FILE, himax_touch_proc_dir);
fail_7: remove_proc_entry(HIMAX_PROC_SMWP_FILE, himax_touch_proc_dir);
fail_6_1: remove_proc_entry(HIMAX_PROC_RESET_FILE, himax_touch_proc_dir);
fail_6: remove_proc_entry(HIMAX_PROC_LAYOUT_FILE, himax_touch_proc_dir);
fail_5: remove_proc_entry(HIMAX_PROC_INT_EN_FILE, himax_touch_proc_dir);
fail_4: remove_proc_entry(HIMAX_PROC_ATTN_FILE, himax_touch_proc_dir);
fail_3: remove_proc_entry(HIMAX_PROC_VENDOR_FILE, himax_touch_proc_dir);
fail_2: remove_proc_entry(HIMAX_PROC_DEBUG_LEVEL_FILE, himax_touch_proc_dir);
fail_1:
	return -ENOMEM;
}

void hx83113a_touch_proc_deinit(void)
{
	remove_proc_entry(HIMAX_PROC_CRC_TEST_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_ESD_CNT_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_SENSE_ON_OFF_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_FLASH_DUMP_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_DEBUG_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_FW_DEBUG_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_DD_DEBUG_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_REGISTER_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_DIAG_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_SMWP_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_RESET_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_LAYOUT_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_INT_EN_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_ATTN_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_VENDOR_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_DEBUG_LEVEL_FILE, himax_touch_proc_dir);
}

int hx83113a_debug_init(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);
	int err = 0;

	TOUCH_I("%s:Enter\n", __func__);

	if (d == NULL) {
		TOUCH_E("%s: ts struct is NULL \n", __func__);
		return -EPROBE_DEFER;
	}

	debug_data = kzalloc(sizeof(*debug_data), GFP_KERNEL);
	if (debug_data == NULL) { /*Allocate debug data space*/
		err = -ENOMEM;
		goto err_alloc_debug_data_fail;
	}

	hx83113a_dbg_data_init();

	d->flash_wq = create_singlethread_workqueue("himax_flash_wq");

	if (!d->flash_wq) {
		TOUCH_E("%s: create flash workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_flash_dump_wq_failed;
	}

	INIT_WORK(&d->flash_work, hx83113a_ts_flash_work_func);
	setSysOperation(0);
	setFlashBuffer();

	d->himax_diag_wq = create_singlethread_workqueue("himax_diag");

	if (!d->himax_diag_wq) {
		TOUCH_E("%s: create diag workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_diag_wq_failed;
	}

	INIT_DELAYED_WORK(&d->himax_diag_delay_wrok, hx83113a_ts_diag_work_func);

	setMutualBuffer(ic_data->HX_RX_NUM, ic_data->HX_TX_NUM);
	setMutualNewBuffer(ic_data->HX_RX_NUM, ic_data->HX_TX_NUM);
	setMutualOldBuffer(ic_data->HX_RX_NUM, ic_data->HX_TX_NUM);

	if (getMutualBuffer() == NULL) {
		TOUCH_E("%s: mutual buffer allocate fail failed\n", __func__);
		return MEM_ALLOC_FAIL;
	}
#ifdef HX_TP_PROC_2T2R

	if (Is_2T2R) {
		setMutualBuffer_2(ic_data->HX_RX_NUM_2, ic_data->HX_TX_NUM_2);

		if (getMutualBuffer_2() == NULL) {
			TOUCH_E("%s: mutual buffer 2 allocate fail failed\n", __func__);
			return MEM_ALLOC_FAIL;
		}
	}
#endif

	hx83113a_touch_proc_init();

	return 0;

	cancel_delayed_work_sync(&d->himax_diag_delay_wrok);
	destroy_workqueue(d->himax_diag_wq);
err_create_diag_wq_failed:

	destroy_workqueue(d->flash_wq);
err_create_flash_dump_wq_failed:

err_alloc_debug_data_fail:

	return err;
}

int hx83113a_debug_remove(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);

	hx83113a_touch_proc_deinit();

	cancel_delayed_work_sync(&d->himax_diag_delay_wrok);
	destroy_workqueue(d->himax_diag_wq);
	destroy_workqueue(d->flash_wq);

	kfree(debug_data);

	return 0;
}

int hx83113a_proc_init(void)
{
	himax_touch_proc_dir = proc_mkdir(HIMAX_PROC_TOUCH_FOLDER, NULL);

	if (himax_touch_proc_dir == NULL) {
		TOUCH_E(" %s: himax_touch_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}

	return 0;
}

void hx83113a_proc_deinit(void)
{
	remove_proc_entry(HIMAX_PROC_TOUCH_FOLDER, NULL);
}

static int prd_switch_mode(struct device *dev, int mode)
{
	uint8_t tmp_data[4];

	/*Stop Handshaking*/
	//tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00; tmp_addr[0] = 0x00;
	tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = 0x00; tmp_data[0] = 0x00;
	hx83113a_register_write(dev, SRAM_ADR_RAWDATA_ADDR, 4, tmp_data, ADDR_LEN_4);

	/*Swtich Mode*/
	switch (mode) {
	case HIMAX_INSPECTION_SORTING:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_SORTING_START; tmp_data[0] = PWD_SORTING_START;
		break;
	case HIMAX_INSPECTION_OPEN:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_OPEN_START; tmp_data[0] = PWD_OPEN_START;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_OPEN_START; tmp_data[0] = PWD_OPEN_START;
		break;
	case HIMAX_INSPECTION_SHORT:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_SHORT_START; tmp_data[0] = PWD_SHORT_START;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_RAWDATA_START; tmp_data[0] = PWD_RAWDATA_START;
		break;
	case HIMAX_INSPECTION_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_NOISE_START; tmp_data[0] = PWD_NOISE_START;
		break;
	case HIMAX_INSPECTION_LPWG_RAWDATA:
	case HIMAX_INSPECTION_LPWG_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_LPWG_START; tmp_data[0] = PWD_LPWG_START;
		break;
	case HIMAX_INSPECTION_LPWG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWG_IDLE_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_LPWG_IDLE_START; tmp_data[0] = PWD_LPWG_IDLE_START;
		break;
	default:
		TOUCH_I("%s,Nothing to be done!\n", __func__);
		break;
	}

	hx83113a_assign_sorting_mode(dev, tmp_data);
	TOUCH_I("%s: End of setting!\n", __func__);

	return 0;

}

static int prd_get_rawdata(struct device *dev, uint32_t RAW[], uint32_t datalen)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint8_t *tmp_rawdata;
	uint8_t retry = 0;
	uint16_t checksum_cal;
	uint32_t i = 0;

	uint8_t max_i2c_size = 128;
	int address = 0;
	int addr = 0;
	int total_read_times = 0;
	int total_size = datalen * 2 + 4;
	int total_size_temp;

	tmp_rawdata = kzalloc(sizeof(uint8_t)*(datalen*2), GFP_KERNEL);
	if(tmp_rawdata == NULL){
		TOUCH_I("%s, alloc fail\n",__func__);
		return 1;
	}
	/*1 Set Data Ready PWD*/
	while (retry < 300) {
		//tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00; tmp_addr[0] = 0x00;
		tmp_data[3] = 0x00; tmp_data[2] = 0x00;
		tmp_data[1] = Data_PWD1;
		tmp_data[0] = Data_PWD0;
		hx83113a_register_write(dev, SRAM_ADR_RAWDATA_ADDR, 4, tmp_data, ADDR_LEN_4);

		hx83113a_register_read(dev, SRAM_ADR_RAWDATA_ADDR, 4, tmp_data, ADDR_LEN_4);
		if ((tmp_data[0] == Data_PWD0 && tmp_data[1] == Data_PWD1) ||
			(tmp_data[0] == Data_PWD1 && tmp_data[1] == Data_PWD0)) {
			break;
		}

		retry++;
		touch_msleep(1);
	}

	if (retry >= 200) {
		kfree(tmp_rawdata);
		return 1;
	} else {
		retry = 0;
	}

	while (retry < 200) {
		if (tmp_data[0] == Data_PWD1 && tmp_data[1] == Data_PWD0) {
			break;
		}

		retry++;
		touch_msleep(1);
		//addr = (tmp_addr[3] << 24) + (tmp_addr[2] << 16) + (tmp_addr[1] << 8) + tmp_addr[0];
		hx83113a_register_read(dev, SRAM_ADR_RAWDATA_ADDR, 4, tmp_data, ADDR_LEN_4);
	}

	if (retry >= 200) {
		kfree(tmp_rawdata);
		return 1;
	} else {
		retry = 0;
	}

	/*2 Read Data from SRAM*/
	while (retry < 10) {
		checksum_cal = 0;
		total_size_temp = total_size;
		tmp_addr[3] = 0x10;	tmp_addr[2] = 0x00;	tmp_addr[1] = 0x00;	tmp_addr[0] = 0x00;

		if (total_size % max_i2c_size == 0) {
			total_read_times = total_size / max_i2c_size;
		} else {
			total_read_times = total_size / max_i2c_size + 1;
		}

		for (i = 0; i < (total_read_times); i++) {
			if (total_size_temp >= max_i2c_size) {
				addr = (tmp_addr[3] << 24) + (tmp_addr[2] << 16) + (tmp_addr[1] << 8) + tmp_addr[0];
				hx83113a_register_read(dev, addr, max_i2c_size, &tmp_rawdata[i*max_i2c_size], ADDR_LEN_4);
				total_size_temp = total_size_temp - max_i2c_size;
			} else {
				/*TOUCH_I("last total_size_temp=%d\n", total_size_temp);*/
				addr = (tmp_addr[3] << 24) + (tmp_addr[2] << 16) + (tmp_addr[1] << 8) + tmp_addr[0];
				hx83113a_register_read(dev, addr, total_size_temp % max_i2c_size, &tmp_rawdata[i*max_i2c_size], ADDR_LEN_4);
			}

			address = ((i+1)*max_i2c_size);
			tmp_addr[1] = (uint8_t)((address>>8)&0x00FF);
			tmp_addr[0] = (uint8_t)((address)&0x00FF);
		}

		/*3 Check Checksum*/
		for (i = 2; i < datalen * 2 + 4; i = i + 2) {
			checksum_cal += tmp_rawdata[i + 1] * 256 + tmp_rawdata[i];
		}

		if (checksum_cal == 0) {
			break;
		}

		retry++;
	}

	if (checksum_cal != 0) {
		TOUCH_E("%s: Get rawdata checksum fail!\n", __func__);
		kfree(tmp_rawdata);
		return HX_CHKSUM_FAIL;
	}

	/*4 Copy Data*/
	for (i = 0; i < ic_data->HX_TX_NUM*ic_data->HX_RX_NUM; i++) {
		RAW[i] = tmp_rawdata[(i * 2) + 1 + 4] * 256 + tmp_rawdata[(i * 2) + 4];
	}

	kfree(tmp_rawdata);
	return HX_INSPECT_OK;
}

static void prd_switch_data_type(struct device *dev, uint8_t checktype)
{
	uint8_t datatype = 0x00;

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		datatype = DATA_SORTING;
		break;
	case HIMAX_INSPECTION_OPEN:
		datatype = DATA_OPEN;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		datatype = DATA_MICRO_OPEN;
		break;
	case HIMAX_INSPECTION_SHORT:
		datatype = DATA_SHORT;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		datatype = DATA_RAWDATA;
		break;
	case HIMAX_INSPECTION_NOISE:
		datatype = DATA_NOISE;
		break;
	case HIMAX_INSPECTION_BACK_NORMAL:
		datatype = DATA_BACK_NORMAL;
		break;
	case HIMAX_INSPECTION_LPWG_RAWDATA:
		datatype = DATA_LPWG_RAWDATA;
		break;
	case HIMAX_INSPECTION_LPWG_NOISE:
		datatype = DATA_LPWG_NOISE;
		break;
	case HIMAX_INSPECTION_LPWG_IDLE_RAWDATA:
		datatype = DATA_LPWG_IDLE_RAWDATA;
		break;
	case HIMAX_INSPECTION_LPWG_IDLE_NOISE:
		datatype = DATA_LPWG_IDLE_NOISE;
		break;
	default:
		TOUCH_E("Wrong type=%d\n", checktype);
		break;
	}
	hx83113a_diag_register_set(dev, datatype, 0x00);
}

static void prd_cut_cc_ctrl(struct device *dev, uint8_t cut_cc_en)
{
	uint8_t tmp_data[4];

	hx83113a_register_read(dev, FW_ADDR_FW_NEW_METHOLD, 4, tmp_data, ADDR_LEN_4);
	if((tmp_data[3] & 0x20) == 0x20) {
		hx83113a_register_read(dev, FW_ADDR_CC_CUT_NEW, 4, tmp_data, ADDR_LEN_4);
		if (cut_cc_en)
    	tmp_data[1] = tmp_data[1] & 0xEF;
    else
    	tmp_data[1] = tmp_data[1] | 0x10;
    TOUCH_I("%s New Methold support\n", __func__);
	} else {
		hx83113a_register_read(dev, FW_ADDR_CC_CUT_OLD, 4, tmp_data, ADDR_LEN_4);
		if (!cut_cc_en)
    	tmp_data[0] = tmp_data[0] & 0xCF;
    else
    	tmp_data[0] = tmp_data[0] | 0x30;
    TOUCH_I("%s Old Methold support\n", __func__);
	}
}

static void prd_neg_noise_sup(struct device *dev, uint8_t *data)
{
	uint8_t tmp_data[4];

	/*0x10007FD8 Check support negative value or not */
	hx83113a_register_read(dev, FW_ADDR_NEG_NOISE_SUP, 4, tmp_data, ADDR_LEN_4);

	if ((tmp_data[3] & 0x04) == 0x04) {
		data[2] = (FW_DATA_NEG_NOISE >> 16) & 0xFF;
		data[3] = (FW_DATA_NEG_NOISE >> 24) & 0xFF;
	} else
		TOUCH_I("%s Not support negative noise\n", __func__);
}

static void prd_set_N_frame(struct device *dev, uint16_t Nframe, uint8_t checktype)
{
	uint8_t tmp_data[4];

	/*IIR MAX*/
	tmp_data[3] = 0x00; tmp_data[2] = 0x00;
	tmp_data[1] = (uint8_t)((Nframe & 0xFF00) >> 8);
	tmp_data[0] = (uint8_t)(Nframe & 0x00FF);

	/*Set Negtive noise*/
	if (checktype == HIMAX_INSPECTION_NOISE || checktype == HIMAX_INSPECTION_LPWG_NOISE)
		prd_neg_noise_sup(dev, tmp_data);

	hx83113a_register_write(dev, FW_ADDR_SET_FRAME_ADDR, 4, tmp_data, ADDR_LEN_4);

	/*skip frame*/
	hx83113a_register_read(dev, DRIVER_ADDR_FW_DEFINE_RXNUM_TXNUM_MAXPT, 4, tmp_data, ADDR_LEN_4);

	switch (checktype) {
	case HIMAX_INSPECTION_LPWG_RAWDATA:
	case HIMAX_INSPECTION_LPWG_NOISE:
		tmp_data[0] = BS_LPWG;
		break;
	case HIMAX_INSPECTION_LPWG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWG_IDLE_NOISE:
		tmp_data[0] = BS_LPWG_dile;
		break;
	case HIMAX_INSPECTION_RAWDATA:
	case HIMAX_INSPECTION_NOISE:
		tmp_data[0] = BS_RAWDATANOISE;
		break;
	default:
		tmp_data[0] = BS_OPENSHORT;
		break;
	}
	hx83113a_register_write(dev, DRIVER_ADDR_FW_DEFINE_RXNUM_TXNUM_MAXPT, 4, tmp_data, ADDR_LEN_4);
}

static uint32_t prd_check_mode(struct device *dev, uint8_t checktype)
{
	uint8_t tmp_data[4] = {0};
	uint8_t wait_pwd[2] = {0};

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		wait_pwd[0] = PWD_SORTING_END;
		wait_pwd[1] = PWD_SORTING_END;
		break;
	case HIMAX_INSPECTION_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_SHORT:
		wait_pwd[0] = PWD_SHORT_END;
		wait_pwd[1] = PWD_SHORT_END;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_NOISE:
		wait_pwd[0] = PWD_NOISE_END;
		wait_pwd[1] = PWD_NOISE_END;
		break;
	case HIMAX_INSPECTION_LPWG_RAWDATA:
	case HIMAX_INSPECTION_LPWG_NOISE:
		wait_pwd[0] = PWD_LPWG_END;
		wait_pwd[1] = PWD_LPWG_END;
		break;
	case HIMAX_INSPECTION_LPWG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWG_IDLE_NOISE:
		wait_pwd[0] = PWD_LPWG_IDLE_END;
		wait_pwd[1] = PWD_LPWG_IDLE_END;
		break;
	default:
		TOUCH_E("Wrong type=%d\n", checktype);
		break;
	}

	hx83113a_check_sorting_mode(dev, tmp_data);

	if ((wait_pwd[0] == tmp_data[0]) && (wait_pwd[1] == tmp_data[1])) {
		TOUCH_I("Change to mode = %s\n", g_himax_inspection_mode[checktype]);
		return 0;
	} else {
		return 1;
	}
}

static uint32_t prd_wait_sorting_mode(struct device *dev, uint8_t checktype)
{
	uint8_t tmp_data[4] = {0};
	uint8_t wait_pwd[2] = {0};
	int count = 0;

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		wait_pwd[0] = PWD_SORTING_END;
		wait_pwd[1] = PWD_SORTING_END;
		break;
	case HIMAX_INSPECTION_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_SHORT:
		wait_pwd[0] = PWD_SHORT_END;
		wait_pwd[1] = PWD_SHORT_END;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_NOISE:
		wait_pwd[0] = PWD_NOISE_END;
		wait_pwd[1] = PWD_NOISE_END;
		break;
	case HIMAX_INSPECTION_LPWG_RAWDATA:
	case HIMAX_INSPECTION_LPWG_NOISE:
		wait_pwd[0] = PWD_LPWG_END;
		wait_pwd[1] = PWD_LPWG_END;
		break;
	case HIMAX_INSPECTION_LPWG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWG_IDLE_NOISE:
		wait_pwd[0] = PWD_LPWG_IDLE_END;
		wait_pwd[1] = PWD_LPWG_IDLE_END;
		break;
	default:
		TOUCH_I("No Change Mode and now type=%d\n", checktype);
		break;
	}

	do {
		hx83113a_check_sorting_mode(dev, tmp_data);
		if ((wait_pwd[0] == tmp_data[0]) && (wait_pwd[1] == tmp_data[1])) {
			return 0;
		}

		hx83113a_register_read(dev, FW_ADDR_CHK_FW_STATUS, 4, tmp_data, ADDR_LEN_4);
		TOUCH_I("%s: 0x900000A8, tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		hx83113a_register_read(dev, FW_ADDR_CHK_FW_STATUS_2, 4, tmp_data, ADDR_LEN_4);
		TOUCH_I("%s: 0x900000E4, tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		hx83113a_register_read(dev, FW_ADDR_CHK_FW_STATUS_3, 4, tmp_data, ADDR_LEN_4);
		TOUCH_I("%s: 0x10007F40,tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		TOUCH_I("Now retry %d times!\n", count++);
		touch_msleep(50);
	} while (count < 200);

	return 1;
}

/***********LGE sysfs Start***********/
static char W_Buf[BUF_SIZE];
//static u16 Rawdata_buf[ROW_SIZE * COL_SIZE];
//static u16 open_short_buf[ROW_SIZE * COL_SIZE];
//static u16 MutualLowerImage[ROW_SIZE][COL_SIZE];
//static u16 MutualUpperImage[ROW_SIZE][COL_SIZE];

int mul_min;
int mul_max;

uint32_t prd_mp_test(struct device *dev, uint8_t checktype, uint32_t RAW[], uint32_t datalen)
{
	uint32_t ret = 0;

	if (prd_check_mode(dev, checktype)) {
		TOUCH_I("Need Change Mode ,target = %s", g_himax_inspection_mode[checktype]);

		hx83113a_sense_off(dev, true);

		hx83113a_reload_disable(dev, 1);

		prd_switch_mode(dev, checktype);

		if (checktype == HIMAX_INSPECTION_NOISE || checktype == HIMAX_INSPECTION_LPWG_NOISE) {
			TOUCH_I("N frame = %d\n", NOISEFRAME);
			prd_set_N_frame(dev, NOISEFRAME, checktype);
			prd_cut_cc_ctrl(dev, CUT_CC_EN);
		} else {
			TOUCH_I("N frame = %d\n", 2);
			prd_set_N_frame(dev, 2, checktype);
		}

		hx83113a_sense_on(dev, 1);

		ret = prd_wait_sorting_mode(dev, checktype);
		if (ret) {
			TOUCH_E("%s: prd_wait_sorting_mode FAIL\n", __func__);
			return ret;
		}
	}

	prd_switch_data_type(dev, checktype);

	ret = prd_get_rawdata(dev, RAW, datalen);
	if (ret) {
		TOUCH_E("%s: prd_get_rawdata FAIL\n", __func__);
		return ret;
	}

	/* back to normal */
	prd_switch_data_type(dev, HIMAX_INSPECTION_BACK_NORMAL);

	return HX_INSPECT_OK;
}

static void prd_log_size_check(struct device *dev)
{
	char *fname = NULL;
	struct file *file = NULL;
	loff_t file_size = 0;
	int i = 0;
	char buf1[128] = {0};
	char buf2[128] = {0};
	mm_segment_t old_fs = get_fs();
	int ret = 0;
	int boot_mode = TOUCH_NORMAL_BOOT;

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
		TOUCH_E("%s : fname is NULL, can not open FILE\n",
				__func__);
		goto error;
	}

	if (IS_ERR(file)) {
		TOUCH_I("%s : ERR(%ld) Open file error [%s]\n",	__func__, PTR_ERR(file), fname);
		goto error;
	}

	file_size = vfs_llseek(file, 0, SEEK_END);
	TOUCH_I("%s : [%s] file_size = %lld\n",	__func__, fname, file_size);

	filp_close(file, 0);

	if (file_size > MAX_LOG_FILE_SIZE) {
		TOUCH_I("%s : [%s] file_size(%lld) > MAX_LOG_FILE_SIZE(%d)\n",
				__func__, fname, file_size, MAX_LOG_FILE_SIZE);

		for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
			if (i == 0)
				sprintf(buf1, "%s", fname);
			else
				sprintf(buf1, "%s.%d", fname, i);

			ret = ksys_access(buf1, 0);

			if (ret == 0) {
				TOUCH_I("%s : file [%s] exist\n",__func__, buf1);

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					if (ksys_unlink(buf1) < 0) {
						TOUCH_E("%s : failed to remove file [%s]\n",__func__, buf1);
						goto error;
					}

					TOUCH_I("%s : remove file [%s]\n",__func__, buf1);
				} else {
					sprintf(buf2, "%s.%d", fname, (i + 1));
                    if (ksys_link(buf1, buf2) < 0) {
                        TOUCH_E("%s : failed to link file [%s] -> [%s]\n", __func__, buf1, buf2);
                        goto error;
                    }
                    if (ksys_unlink(buf1) < 0) {
                        TOUCH_E("%s : failed to remove file [%s]\n", __func__, buf1);
                        goto error;
                    }
					TOUCH_I("%s : rename file [%s] -> [%s]\n",	__func__, buf1, buf2);
				}
			} else {
				TOUCH_I("%s : file [%s] does not exist (ret = %d)\n",
						__func__, buf1, ret);
			}
		}
	}

error:
	set_fs(old_fs);
	return;
}

static void write_file(struct device *dev, char *data, int write_time)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time= {0, };
	struct tm my_date= {0, };
	mm_segment_t old_fs = get_fs();
	int boot_mode = 0;

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
		return;
	}

	if (fd >= 0) {
		if (write_time) {
			my_time = current_kernel_time();
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
		ksys_close(fd);
	} else {
		TOUCH_I("File open failed\n");
	}
	set_fs(old_fs);
}

static ssize_t prd_mutual_data_read(size_t count, char *buf, int16_t RAW[])
{
	int i, j;
	char log_buf[LOG_BUF_SIZE] = {0, };
	int log_size = 0;

	log_size = touch_snprintf(log_buf, LOG_BUF_SIZE, "   : ");

	for (i = 0; i < ic_data->HX_RX_NUM; i++)
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, " [%2d] ", i);

	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;

	for (i = 0; i < ic_data->HX_TX_NUM; i++) {
		count += touch_snprintf(buf + count, PAGE_SIZE - count, "%c%2d%c",'[', i + 1,']');
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "[%2d] ", i);

		for (j = 0; j < ic_data->HX_RX_NUM; j++) {
			count += touch_snprintf(buf + count, PAGE_SIZE - count, " %5d", RAW[ j + i*ic_data->HX_RX_NUM]);
			log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "%5d ", RAW[i * ic_data->HX_RX_NUM + j]);

		}
		count += touch_snprintf(buf + count, PAGE_SIZE - count, "\n");

		TOUCH_I("%s\n", log_buf);
		memset(log_buf, 0, sizeof(log_buf));
		log_size = 0;

	}

	return count;
}

void prd_set_jitter_para(void)
{
	int cnt = 0;

	if (cnt >= 100)
		TOUCH_E("[Himax]: Jitter data not ready \n");
}

#if 0
static int prd_set_limit(struct device *dev, u8 type, u16 MutualUpper)
{
	int i,j;

	if (type == RAW_DATA_TEST || type == JITTER_TEST) {
		for (i = 0; i < ROW_SIZE; i++) {
			for (j = 0; j < COL_SIZE; j++) {
				MutualUpperImage[i][j] = MutualUpper;
			}
		}
	}
	if (type == RAW_DATA_TEST) {
		for (i = 0; i < ROW_SIZE; i++) {
			for (j = 0; j < COL_SIZE; j++) {
				MutualLowerImage[i][j] = MUL_RAW_DATA_THX_LOW;
			}
		}
	}

	return 0;
}
#endif

int prd_get_raw_data(struct device *dev, uint8_t diag_cmd, uint16_t mutual_num, int16_t RAW[])
{
	struct himax_ts_data *d = to_himax_data(dev);
	int i = 0;
	uint8_t  *info_data;

	TOUCH_I("Start get raw data\n");

	info_data = kzalloc(mutual_num * 2 * sizeof(uint8_t), GFP_KERNEL);
	if(info_data == NULL){
		TOUCH_I("%s, alloc fail\n",__func__);
		return -EINVAL;
	}
	touch_interrupt_control(d->dev, INTERRUPT_DISABLE);

	hx83113a_diag_register_set(dev, diag_cmd, 0);
	hx83113a_get_DSRAM_data(dev, info_data, true);
	hx83113a_diag_register_set(dev, 0, 0);

	touch_interrupt_control(d->dev, INTERRUPT_ENABLE);

	for(i = 0;i < mutual_num * 2;i = i + 2) {
		 RAW[i / 2]= (info_data[i + 1] << 8) + info_data[i];
//		 TOUCH_I("mutual_data[%d] = %d\n", i / 2, RAW[i / 2]);
	}

	kfree(info_data);

	return 0;
}

static int prd_print_data(struct device *dev, char *buf, uint32_t RAW[], u8 type)
{
	int i = 0, j = 0;
	int ret = 0;
	int col_size = ic_data->HX_RX_NUM;
	char test_type[32] = {0, };
	char log_buf[LOG_BUF_SIZE] = {0, };
	int log_size = 0;

	/* print a frame data */
	ret = touch_snprintf(buf, PAGE_SIZE, "\n   : ");
	log_size = touch_snprintf(log_buf, LOG_BUF_SIZE, "   : ");

	switch (type) {
	case OPEN_TEST:
		touch_snprintf(test_type, sizeof(test_type),
				"\n[OPEN_TEST]\n");
		break;
	case MICRO_OPEN_TEST:
		touch_snprintf(test_type, sizeof(test_type),
				"\n[MICRO_OPEN_TEST]\n");
		break;
	case SHORT_TEST:
		touch_snprintf(test_type, sizeof(test_type),
				"\n[SHORT_TEST]\n");
		break;
	case RAW_DATA_TEST:
		touch_snprintf(test_type, sizeof(test_type),
				"\n[RAWDATA_TEST]\n");
		break;
	case JITTER_TEST:
		touch_snprintf(test_type, sizeof(test_type),
				"\n[JITTER_TEST]\n");
		break;
	case LPWG_RAW_DATA_TEST:
		touch_snprintf(test_type, sizeof(test_type),
				"\n[LPWG_RAWDATA_TEST]\n");
		break;
	case LPWG_JITTER_TEST:
		touch_snprintf(test_type, sizeof(test_type),
				"\n[LPWG_JITTER_TEST]\n");
		break;
	default:
		touch_snprintf(test_type, sizeof(test_type),
				"\n[NOT TEST ITEM]\n");
		return 1;
	}
	/* Test Type Write */
	write_file(dev, test_type, TIME_INFO_SKIP);

	for (i = 0; i < col_size; i++) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, " [%2d] ", i);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, " [%2d] ", i);
	}

	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;

	mul_min = mul_max = RAW[0];
	for (i = 0; i < ic_data->HX_TX_NUM; i++) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,  "\n[%2d] ", i);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "[%2d] ", i);

		for (j = 0; j < col_size; j++) {
			ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
					"%5d ", (int16_t)RAW[i*col_size+j]);
			log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "%5d ", (int16_t)RAW[i * col_size + j]);

			mul_max = max(mul_max, (int)((int16_t)RAW[i*col_size+j]));
			mul_min = min(mul_min, (int)((int16_t)RAW[i*col_size+j]));
		}
		TOUCH_I("%s\n", log_buf);
		memset(log_buf, 0, sizeof(log_buf));
		log_size = 0;
	}

	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
			"\n==================================================\n");

	return ret;
}

static int prd_compare_rawdata(struct device *dev, char *buf, u8 type, uint32_t RAW[])
{
	int i, j = 0;
	int ret = 0;
	int result = 1;
	int row_size = ic_data->HX_TX_NUM;
	int col_size = ic_data->HX_RX_NUM;
	unsigned short *limit_upper = NULL;
	unsigned short *limit_lower = NULL;
	short *limit_upper_s = NULL;
	short *limit_lower_s = NULL;
	int fail_num = 0;

	TOUCH_I("%s, row_size[%d], col_size[%d]\n", __func__, row_size, col_size);

	switch (type) {
	case OPEN_TEST:
		limit_upper = (unsigned short*)open_hi_limits;
		limit_lower = (unsigned short*)open_lo_limits;
		break;
	case MICRO_OPEN_TEST:
		limit_upper = (unsigned short*)micro_open_hi_limits;
		limit_lower = (unsigned short*)micro_open_lo_limits;
		break;
	case SHORT_TEST:
		limit_upper = (unsigned short*)short_hi_limits;
		limit_lower = (unsigned short*)short_lo_limits;
		break;
	case RAW_DATA_TEST:
		limit_upper = (unsigned short*)rawdata_hi_limits;
		limit_lower = (unsigned short*)rawdata_lo_limits;
		break;
	case JITTER_TEST:
		limit_upper_s = (short*)jitter_hi_limits;
		limit_lower_s = (short*)jitter_lo_limits;
		break;
	case LPWG_RAW_DATA_TEST:
		limit_upper = (unsigned short*)lpwg_rawdata_hi_limits;
		limit_lower = (unsigned short*)lpwg_rawdata_lo_limits;
		break;
	case LPWG_JITTER_TEST:
		limit_upper_s = (short*)lpwg_jitter_hi_limits;
		limit_lower_s = (short*)lpwg_jitter_lo_limits;
		break;
	default:
		TOUCH_I("Test Type not defined\n");
		result = 0;
		goto exit;
                break;
	}

	//prd_set_limit(dev, type, mutual_upper);

	if (type == RAW_DATA_TEST || type == OPEN_TEST || type == MICRO_OPEN_TEST
		|| type == SHORT_TEST|| type == LPWG_RAW_DATA_TEST) {
		for (i = 0; i < row_size; i++) {
			for (j = 0; j < col_size; j++) {
				if ((RAW[i * col_size + j] < *(limit_lower+(i*col_size+j))) ||
						(RAW[i * col_size + j] > *(limit_upper+(i*col_size+j)))) {
					result = 0;
					fail_num++;
					ret += touch_snprintf(W_Buf + ret, sizeof(W_Buf) - ret,
							"F [%d][%d] = %d\n", i, j, RAW[i * col_size + j]);
					TOUCH_I("F [%d][%d] = %d\n", i, j, RAW[i * col_size + j]);

					if (ret > (BUF_SIZE / 2)) {
						write_file(dev, W_Buf, TIME_INFO_SKIP);
						memset(W_Buf, 0, sizeof(W_Buf));
						ret = 0;
					}
				}
			}
		}
	}

	if (type == JITTER_TEST || type == LPWG_JITTER_TEST) {
		for (i = 0; i < row_size; i++) {
			for (j = 0; j < col_size; j++) {
				if (((int16_t)RAW[i * col_size + j] > *(limit_upper_s+(i*col_size+j))) ||
						((int16_t)RAW[i * col_size + j] < *(limit_lower_s+(i*col_size+j)))) {
					result = 0;
					fail_num++;
					ret += touch_snprintf(W_Buf + ret, sizeof(W_Buf) - ret,
							"F [%d][%d] = %d\n", i, j, (int16_t)RAW[i * col_size + j]);
					TOUCH_I("F [%d][%d] = %d\n", i, j, (int16_t)RAW[i * col_size + j]);

					if (ret > (BUF_SIZE / 2)) {
						write_file(dev, W_Buf, TIME_INFO_SKIP);
						memset(W_Buf, 0, sizeof(W_Buf));
						ret = 0;
					}
				}
			}
		}
	}
exit:
	if (!result)
	{
		ret += touch_snprintf(W_Buf + ret, sizeof(W_Buf) - ret, "Test Fail : %d\n", fail_num);
		TOUCH_I("Test Fail: %d\n", fail_num);
	}
	else
	{
		ret += touch_snprintf(W_Buf + ret, sizeof(W_Buf) - ret, "Test PASS : No Errors\n");
		TOUCH_I("Test PASS: No Errors\n");
	}

	ret += touch_snprintf(W_Buf + ret, sizeof(W_Buf) - ret,
				"MAX = %d, MIN = %d\n", mul_max, mul_min);
	TOUCH_I("MAX = %d, MIN = %d\n", mul_max, mul_min);

	return result;
}

static int prd_rawdata_test(struct device *dev, char *buf, u8 type)
{
	uint32_t datalen = (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM;
	uint32_t* RAW;
	uint8_t check_type;
	int result = 0;

	switch (type) {
	case OPEN_TEST:
		check_type = HIMAX_INSPECTION_OPEN;
		break;
	case MICRO_OPEN_TEST:
		check_type = HIMAX_INSPECTION_MICRO_OPEN;
		break;
	case SHORT_TEST:
		check_type = HIMAX_INSPECTION_SHORT;
		break;
	case RAW_DATA_TEST:
		check_type = HIMAX_INSPECTION_RAWDATA;
		break;
	case JITTER_TEST:
		check_type = HIMAX_INSPECTION_NOISE;
		break;
	case LPWG_RAW_DATA_TEST:
		check_type = HIMAX_INSPECTION_LPWG_RAWDATA;
		break;
	case LPWG_JITTER_TEST:
		check_type = HIMAX_INSPECTION_LPWG_NOISE;
		break;
	default:
		TOUCH_I("Test Type not defined\n");
		return 1;
	}
	if(type == JITTER_TEST)
		prd_set_jitter_para();

    RAW = kzalloc(datalen * sizeof(uint32_t), GFP_KERNEL);
	if(RAW == NULL){
		TOUCH_I("%s, alloc fail\n",__func__);
		result = 1;
	   	return result;
	}
	memset(RAW, 0, datalen * sizeof(uint32_t));
//	prd_read_rawdata(dev, buf, type, mutual_data);
	result = prd_mp_test(dev, check_type, RAW, datalen);

//	memcpy(Rawdata_buf, mutual_data, MUTUAL_NUM * sizeof(uint16_t));

	result = prd_print_data(dev, W_Buf, RAW, type);
	write_file(dev, W_Buf, TIME_INFO_SKIP);
	memset(W_Buf, 0, sizeof(W_Buf));

	result = prd_compare_rawdata(dev, W_Buf, type, RAW);
	write_file(dev, W_Buf, TIME_INFO_SKIP);
	memset(W_Buf, 0, sizeof(W_Buf));

    kfree(RAW);
	return result;
}

static int firmware_version_log(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);

	int ret = 0;
	int fw_ver_ret = 0;
	unsigned char *buffer = NULL;
	int boot_mode = TOUCH_NORMAL_BOOT;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
		fw_ver_ret = hx83113a_read_FW_ver(dev);
		if (fw_ver_ret < 0) {
			TOUCH_E("failed to read ver info (fw_ver_ret: %d)\n", fw_ver_ret);
			return fw_ver_ret;
		}
		break;
	default:
		break;
	}
    buffer = kzalloc(LOG_BUF_SIZE * sizeof(unsigned char), GFP_KERNEL);
	if(buffer == NULL){
		TOUCH_I("%s, alloc fail\n",__func__);
	   	return ret;
	}
	memset(buffer, 0, LOG_BUF_SIZE * sizeof(unsigned char));
	ret = touch_snprintf(buffer, LOG_BUF_SIZE, "=========== Firmware Info ===========\n");
	ret += touch_snprintf(buffer + ret, LOG_BUF_SIZE - ret,
					"Version: v%d.%02d, fw_id: %d, CID_ver: 0x%x\n",
					d->ic_info.ic_official_ver, d->ic_info.ic_fw_ver,
					d->ic_info.vendor_fw_ver,
					(d->ic_info.vendor_cid_maj_ver << 8 | d->ic_info.vendor_cid_min_ver));
	ret += touch_snprintf(buffer + ret, LOG_BUF_SIZE - ret,
				"Panel_cfg: 0x%x, Touch_cfg: 0x%x, Display_cfg: 0x%x\n\n",
				d->ic_info.vendor_panel_ver,
				d->ic_info.vendor_touch_cfg_ver,
				d->ic_info.vendor_display_cfg_ver);

	ret += touch_snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"\n=========== Production Info ===========\n");
	ret += touch_snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"Product_ID : HX83113-A\n\n");

	write_file(dev, buffer, TIME_INFO_SKIP);

	kfree(buffer);
	return 0;
}

static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = to_himax_data(dev);
	uint8_t w_data[DATA_LEN_4] = {0x01, 0x00, 0x00, 0x00};
	int ret = 0;
	int rawdata_ret = 0;
	int jitter_ret = 0;
	int open_ret = 0;
	int micro_open_ret = 0;
	int short_ret = 0;
	int fw_ver_ret = 0;

	TOUCH_I("%s\n", __func__);

	if (d->lcd_mode != LCD_MODE_U3) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD off!!!. Can not sd.\n");
		TOUCH_I("LCD off!!!. Can not sd.\n");
		return ret;
	}
	/* file create , time log */
	write_file(dev, "\nShow_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_sd Test Start\n");

	mutex_lock(&ts->lock);
	touch_interrupt_control(d->dev, INTERRUPT_DISABLE);

	fw_ver_ret = firmware_version_log(dev);
	if (fw_ver_ret < 0) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"\n========RESULT=======\n");
		TOUCH_I("========RESULT=======\n");
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : Fail (Check connector)\n");
		TOUCH_I("Raw Data : Fail (Check connector)\n");
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"Channel Status : Fail (Check connector)\n");
		TOUCH_I("Channel Status : Fail (Check connector)\n");
		goto exit;
	}

	open_ret = prd_rawdata_test(dev, buf, OPEN_TEST);
	micro_open_ret = prd_rawdata_test(dev, buf, MICRO_OPEN_TEST);
	short_ret = prd_rawdata_test(dev, buf, SHORT_TEST);
	jitter_ret = prd_rawdata_test(dev, buf, JITTER_TEST);
	rawdata_ret = prd_rawdata_test(dev, buf, RAW_DATA_TEST);

	ret = touch_snprintf(buf, PAGE_SIZE, "\n========RESULT=======\n");

	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "Channel Status : %s",
						(open_ret && micro_open_ret && short_ret) ? "Pass\n" : "Fail ");

	TOUCH_I("Channel Status : %s",(open_ret && micro_open_ret && short_ret) ? "Pass\n" : "Fail ");

	if (!open_ret || !short_ret || !micro_open_ret)
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "(Open:%s/M-Open:%s/Short:%s)\n",
							(open_ret != 1 ? "0" : "1"),
							(micro_open_ret != 1 ? "0" : "1"),
							(short_ret != 1 ? "0" : "1"));

	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "Raw Data : %s",
						(rawdata_ret && jitter_ret) ? "Pass\n" : "Fail ");

	TOUCH_I("Raw Data : %s",(rawdata_ret && jitter_ret) ? "Pass\n" : "Fail ");

	if (!rawdata_ret || !jitter_ret)
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "(Raw:%s/Jitter:%s)\n",
							(rawdata_ret != 1 ? "0" : "1"),
							(jitter_ret != 1 ? "0" : "1"));

	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "=====================\n");

exit:
	write_file(dev, buf, TIME_INFO_SKIP);
	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);
	prd_log_size_check(dev);

	hx83113a_sense_off(dev, true);
	touch_msleep(120);
	hx83113a_register_write(dev, FW_ADDR_SET_FRAME_ADDR, DATA_LEN_4, w_data, ADDR_LEN_4);
	hx83113a_reload_disable(dev, 0);
	hx83113a_sense_on(dev, 1);
	touch_msleep(120);

	touch_interrupt_control(d->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	touch_msleep(50);
	TOUCH_I("Show_sd Test End\n");

	return ret;
}

static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = to_himax_data(dev);
	uint8_t w_data[DATA_LEN_4] = {0x01, 0x00, 0x00, 0x00};
	int ret = 0;
	int rawdata_ret = 0;
	int jitter_ret = 0;
	int fw_ver_ret = 0;

	TOUCH_I("%s\n", __func__);

	/* Deep sleep check */
	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		ret = touch_snprintf(buf + ret, PAGE_SIZE - ret,
			"LPWG Not Test. IC state is Deep Sleep.\n");
		TOUCH_I("LPWG Not Test. IC state is Deep Sleep.\n");
		return ret;
	}
	if (d->lcd_mode == LCD_MODE_U3) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD on!!!. Can not lpwg_sd.\n");
		TOUCH_I("LCD on!!!. Can not lpwg_sd.\n");
		return ret;
	}

	/* file create , time log */
	write_file(dev, "\nShow_lpwg_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_lpwg_sd Test Start\n");

	mutex_lock(&ts->lock);

	touch_interrupt_control(d->dev, INTERRUPT_DISABLE);

	fw_ver_ret = firmware_version_log(dev);
	if (fw_ver_ret < 0) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"\n========RESULT=======\n");
		TOUCH_I("========RESULT=======\n");
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
			"LPWG RawData : Fail (Check connector)\n");
		TOUCH_I("LPWG RawData : Fail (Check connector)\n");
		goto exit;
	}

	jitter_ret = prd_rawdata_test(dev, buf, LPWG_JITTER_TEST);
	rawdata_ret = prd_rawdata_test(dev, buf, LPWG_RAW_DATA_TEST);

	ret = touch_snprintf(buf, PAGE_SIZE, "\n========RESULT=======\n");

	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "LPWG RawData : %s",
						(rawdata_ret && jitter_ret) ? "Pass\n" : "Fail ");

	TOUCH_I( "LPWG RawData : %s",(rawdata_ret && jitter_ret) ? "Pass\n" : "Fail ");

	if (!rawdata_ret || !jitter_ret)
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "(Raw:%s/Jitter:%s)\n",
							(rawdata_ret != 1 ? "0" : "1"),
							(jitter_ret != 1 ? "0" : "1"));

	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "=====================\n");

exit:
	write_file(dev, buf, TIME_INFO_SKIP);
	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);
	prd_log_size_check(dev);

	hx83113a_sense_off(dev, true);
	touch_msleep(120);
	hx83113a_register_write(dev, FW_ADDR_SET_FRAME_ADDR, DATA_LEN_4, w_data, ADDR_LEN_4);
	hx83113a_reload_disable(dev, 0);
	hx83113a_sense_on(dev, 1);
	touch_msleep(120);

	touch_interrupt_control(d->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	touch_msleep(50);
	TOUCH_I("Show_lpwg_sd Test End\n");

	return ret;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	uint32_t datalen = (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM;
	uint8_t w_data[DATA_LEN_4] = {0xCD, 0xCD, 0xCD, 0xCD};
	int16_t* RAW;
	int i = 0;
	int min_val, max_val;
	int count = 0;

    RAW = kzalloc(datalen * sizeof(int16_t), GFP_KERNEL);
	if(RAW == NULL){
		TOUCH_I("%s, alloc fail\n",__func__);
	   	return count;
	}

	mutex_lock(&ts->lock);
    memset(RAW, 0, datalen * sizeof(int16_t));

	hx83113a_register_write(dev, FW_ADDR_CONTROL_IDLE_MODE, DATA_LEN_4, w_data, ADDR_LEN_4);
        touch_msleep(200);

	prd_get_raw_data(dev, 0x0A, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM), RAW);

	count = prd_mutual_data_read(count, buf, RAW);

	min_val = max_val = RAW[0];
	for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
		max_val = max(max_val, (int)RAW[i]);
		min_val = min(min_val, (int)RAW[i]);
	}

	count += touch_snprintf(buf + count, PAGE_SIZE - count, "==================================================\n");
	if (max_val>MUL_RAW_DATA_THX_UP)
		count += touch_snprintf(buf + count, PAGE_SIZE - count, "Test %s\n", "FAIL : Exceed upper threshold");
	else if (min_val<MUL_RAW_DATA_THX_LOW)
		count += touch_snprintf(buf + count, PAGE_SIZE - count, "Test %s\n", "FAIL : Exceed lower threshold");
	else
		count += touch_snprintf(buf + count, PAGE_SIZE - count, "Test %s\n", "PASS : No Errors");

	count += touch_snprintf(buf + count, PAGE_SIZE - count, "MAX = %3d, MIN = %3d, Upper = %3d, Lower =%3d\n",max_val,min_val,MUL_RAW_DATA_THX_UP,MUL_RAW_DATA_THX_LOW);

	memset(w_data, 0, sizeof(w_data));
        hx83113a_register_write(dev, FW_ADDR_CONTROL_IDLE_MODE, DATA_LEN_4, w_data, ADDR_LEN_4);
	touch_msleep(50);

    kfree(RAW);
	mutex_unlock(&ts->lock);
	return count;
}

static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	uint32_t datalen = (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM;
	int16_t* RAW;
	int count = 0;

    RAW = kzalloc(datalen * sizeof(int16_t), GFP_KERNEL);
	if(RAW == NULL){
		TOUCH_I("%s, alloc fail\n",__func__);
		return count;
	}
	mutex_lock(&ts->lock);
    memset(RAW, 0, datalen * sizeof(int16_t));

	prd_get_raw_data(dev, 0x09, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM), RAW);

	count = prd_mutual_data_read(count, buf, RAW);

	count += touch_snprintf(buf + count, PAGE_SIZE - count, "==================================================\n");
	count += touch_snprintf(buf + count, PAGE_SIZE - count, "Test %s\n", "PASS : No Errors");
	touch_msleep(50);

    kfree(RAW);
	mutex_unlock(&ts->lock);
	return count;
}

static ssize_t show_jitter(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	uint32_t datalen = (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM;
	int16_t* RAW;
	int i = 0;
	int min_val, max_val;
	int count = 0;
	int ret = 0;

    RAW = kzalloc(datalen * sizeof(int16_t), GFP_KERNEL);
	if(RAW == NULL){
		TOUCH_I("%s, alloc fail\n",__func__);
	   	return count;
	}
	mutex_lock(&ts->lock);
    memset(RAW, 0, datalen * sizeof(int16_t));

	hx83113a_sense_off(dev, true);
	hx83113a_reload_disable(dev, 1);
	prd_switch_mode(dev, HIMAX_INSPECTION_NOISE);
	prd_set_N_frame(dev, NOISEFRAME, HIMAX_INSPECTION_NOISE);
	hx83113a_sense_on(dev, 0);

	ret = prd_wait_sorting_mode(dev, HIMAX_INSPECTION_NOISE);
	if (ret) {
		TOUCH_E("%s: prd_wait_sorting_mode FAIL\n", __func__);
		goto result;
	}

	prd_get_raw_data(dev, 0x0F, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM), RAW);

	count = prd_mutual_data_read(count, buf, RAW);

	min_val = max_val = RAW[0];
	for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
		max_val = max(max_val, (int)RAW[i]);
		min_val = min(min_val, (int)RAW[i]);
	}

	hx83113a_sense_off(dev, true);
	prd_set_N_frame(dev, 1, HIMAX_INSPECTION_NOISE);
	hx83113a_reload_disable(dev, 0);
	hx83113a_sense_on(dev, 0);

result:
	count += touch_snprintf(buf + count, PAGE_SIZE - count, "==================================================\n");
	if (max_val>JITTER_THX)
		count += touch_snprintf(buf + count, PAGE_SIZE - count, "Test %s\n", "FAIL : Exceed jitter threshold");
	else
		count += touch_snprintf(buf + count, PAGE_SIZE - count, "Test %s\n", "PASS : No Errors");

	count += touch_snprintf(buf + count, PAGE_SIZE - count, "MAX = %3d, MIN = %3d, Upper = %3d, Lower =%3d\n",max_val,min_val,JITTER_THX,0);
	touch_msleep(50);

    kfree(RAW);
	mutex_unlock(&ts->lock);
	return count;
}

static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(jitter, show_jitter, NULL);
//static TOUCH_ATTR(diag, NULL, store_diag);

static struct attribute *himax_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_lpwg_sd.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_delta.attr,
	&touch_attr_jitter.attr,
	//&touch_attr_diag.attr,
	NULL,
};

static const struct attribute_group himax_attribute_group = {
	.attrs = himax_attribute_list,
};

int hx83113a_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &himax_attribute_group);
	if (ret < 0)
		TOUCH_E("himax sysfs register failed\n");

	return 0;
}

