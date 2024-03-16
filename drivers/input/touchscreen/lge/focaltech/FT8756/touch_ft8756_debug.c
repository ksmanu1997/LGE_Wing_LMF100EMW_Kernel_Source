/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
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

#include <touch_core.h>
#include "touch_ft8756.h"
#include <linux/proc_fs.h>
#include <linux/cdev.h>
#include <linux/syscalls.h>

/*create apk debug channel*/
#define PROC_UPGRADE							0
#define PROC_READ_REGISTER						1
#define PROC_WRITE_REGISTER						2
#define PROC_AUTOCLB							4
#define PROC_UPGRADE_INFO						5
#define PROC_WRITE_DATA							6
#define PROC_READ_DATA							7
#define PROC_SET_TEST_FLAG						8
#define PROC_NAME								"ftxxxx-debug"
#define WRITE_BUF_SIZE							1016
#define READ_BUF_SIZE							1016

static unsigned char proc_operate_mode = PROC_UPGRADE;
//static struct proc_dir_entry *fts_proc_entry = NULL;

int fts_i2c_read(struct device *dev, char *writebuf,int writelen, char *readbuf, int readlen)
{
	int ret=0;
	struct touch_core_data *ts = to_touch_core(dev);

	mutex_lock(&ts->lock);

	if ((writelen > 0) && (writelen <= 128))
		ret = i2c_master_send(to_i2c_client(dev),(void *)writebuf, writelen);

	if ((readlen > 0) && (readlen <= 128))
		ret += i2c_master_recv(to_i2c_client(dev), (void *)readbuf, readlen);

	if (ret < 0)
		TOUCH_E("write fail, ret = %d\n", ret);

	mutex_unlock(&ts->lock);

	return ret;
}

int fts_i2c_write(struct device *dev, char *writebuf, int writelen)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	mutex_lock(&ts->lock);
	ft8756_reg_write(dev, writebuf[0], writebuf+1, writelen-1);
	mutex_unlock(&ts->lock);

	return ret;
}

int fts_write_reg(struct device *dev, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return fts_i2c_write(dev, buf, sizeof(buf));
}

int fts_read_reg(struct device *dev, u8 addr, u8 *val)
{
	return fts_i2c_read(dev, &addr, 1, val, 1);
}

static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	char upgrade_file_path[128];
	struct touch_core_data *ts = (struct touch_core_data *)filp->private_data;

	TOUCH_TRACE();

	if (copy_from_user(&writebuf, buff, buflen)) {
		TOUCH_E("[FTS] %s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	TOUCH_D(TRACE, "[FTS] %s: start, mode : %d\n", __func__, proc_operate_mode);

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
		touch_snprintf(upgrade_file_path, sizeof(upgrade_file_path), "%s", writebuf + 1);
		upgrade_file_path[buflen-1] = '\0';
		TOUCH_I("[FTS] %s\n", upgrade_file_path);

		if (ret < 0) {
			TOUCH_E("[FTS] %s:upgrade failed.\n", __func__);
			return ret;
		}
		break;
	case PROC_SET_TEST_FLAG:
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(ts->dev, writebuf + 1, writelen);
		if (ret < 0) {
			TOUCH_E("[FTS] %s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(ts->dev, writebuf + 1, writelen);
		if (ret < 0) {
			TOUCH_E("[FTS] %s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		TOUCH_I("[FTS] %s: autoclb\n", __func__);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		if (writelen > 0) {
			ret = fts_i2c_write(ts->dev, writebuf + 1, writelen);
			if (ret < 0) {
				TOUCH_E("[FTS] %s:write iic error\n", __func__);
				return ret;
			}
		}
		break;
	default:
		break;
	}
	return count;
}

static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	unsigned char buf[READ_BUF_SIZE];
	struct touch_core_data *ts = (struct touch_core_data *)filp->private_data;

	TOUCH_D(TRACE, "[FTS] %s: start, mode : %d\n", __func__, proc_operate_mode);

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		// after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(ts->dev, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = touch_snprintf(buf, READ_BUF_SIZE, "%s", "get fw version failed.\n");
		else
			num_read_chars = touch_snprintf(buf, READ_BUF_SIZE, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(ts->dev, NULL, 0, buf, readlen);
		if (ret < 0) {
			TOUCH_E("[FTS] %s:read iic error\n", __func__);
			return ret;
		}
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(ts->dev, NULL, 0, buf, readlen);
		if (ret < 0) {
			TOUCH_E("[FTS] %s:read iic error\n", __func__);
			return ret;
		}
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}

	if (copy_to_user(buff, buf, num_read_chars)) {
		TOUCH_E("[FTS] %s:copy to user error\n", __func__);
		return -EFAULT;
	}
	//memcpy(buff, buf, num_read_chars);
	return num_read_chars;

}

int fts_proc_open(struct inode *inode, struct file *filp)
{
	filp->private_data = PDE_DATA(inode);
	return 0;
}

static const struct file_operations fts_proc_fops = {
	.owner 	= THIS_MODULE,
	.open   = fts_proc_open,
	.read 	= fts_debug_read,
	.write 	= fts_debug_write,
};

int fts_create_apk_debug_channel(struct device * dev)
{

	struct touch_core_data *ts = to_touch_core(dev);
	struct proc_dir_entry *fts_proc_entry;

	fts_proc_entry = proc_create_data(PROC_NAME, 0777, NULL, &fts_proc_fops, ts);

	if (fts_proc_entry == NULL) {
		TOUCH_I("[FTS] Couldn't create proc entry!\n");
		return -ENOMEM;
	}

	TOUCH_I("[FTS] Create proc entry success!\n");

	return 0;
}

void fts_release_apk_debug_channel(void)
{
	remove_proc_entry(PROC_NAME, NULL);
	TOUCH_I("[FTS] Remove proc entry success!\n");
}

