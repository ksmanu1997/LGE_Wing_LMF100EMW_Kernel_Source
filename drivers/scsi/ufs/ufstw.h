/*
 * Universal Flash Storage tw Write
 *
 * Copyright (C) 2017-2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Yongmyung Lee <ymhungry.lee@samsung.com>
 *	Jinyoung Choi <j-young.choi@samsung.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * See the COPYING file in the top-level directory or visit
 * <http://www.gnu.org/licenses/gpl-2.0.html>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This program is provided "AS IS" and "WITH ALL FAULTS" and
 * without warranty of any kind. You are solely responsible for
 * determining the appropriateness of using and distributing
 * the program and assume all risks associated with your exercise
 * of rights with respect to the program, including but not limited
 * to infringement of third party rights, the risks and costs of
 * program errors, damage to or loss of data, programs or equipment,
 * and unavailability or interruption of operations. Under no
 * circumstances will the contributor of this Program be liable for
 * any damages of any kind arising from your use or distribution of
 * this program.
 *
 * The Linux Foundation chooses to take subject only to the GPLv2
 * license terms, and distributes only under these terms.
 */

#ifndef _UFSTW_H_
#define _UFSTW_H_

#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/blktrace_api.h>
#include <linux/blkdev.h>
#include <scsi/scsi_cmnd.h>

#include "../../../block/blk.h"

#define UFSTW_VER					0x0101
#define UFSTW_DD_VER					0x010200

#define UFSTW_LIFETIME_SECT				2097152 /* 1GB */
#define UFSTW_MAX_LIFETIME_VALUE			0x0B
#define MASK_UFSTW_LIFETIME_NOT_GUARANTEE		0x80000000

/*
 * UFSTW DEBUG
 */
#define TW_DEBUG(ufsf, msg, args...)			\
	do { if (ufsf->tw_debug)			\
		printk(KERN_ERR "%s:%d " msg "\n",	\
		       __func__, __LINE__, ##args);	\
	} while (0)

enum UFSTW_STATE {
	TW_NEED_INIT = 0,
	TW_PRESENT = 1,
	TW_DISABLED = -2,
	TW_RESET = -3,
};

struct ufstw_dev_info {
	bool tw_device;

	/* from Device Descriptor */
	u16 tw_ver;
	u8 tw_buf_no_reduct;
	u8 tw_buf_type;

	/* from Geometry Descriptor */
	u8 tw_number_lu;
};

struct ufstw_lu {
	struct ufsf_feature *ufsf;

	int lun;

	bool tw_enable;

	struct mutex tw_lock;

	/* Flags */
	bool tw_flush_enable;
	bool tw_flush_during_hibern_enter;

	/* lifetiem estimated */
	unsigned int tw_lifetime_est;
	spinlock_t lifetime_lock;
	u32 stat_write_sec;
	struct work_struct tw_lifetime_work;

	/* Attributes */
	unsigned int tw_flush_status;
	unsigned int tw_available_buffer_size;

	/* for sysfs */
	struct kobject kobj;
	struct mutex sysfs_lock;
	struct ufstw_sysfs_entry *sysfs_entries;
};

struct ufstw_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct ufstw_lu *tw, char *buf);
	ssize_t (*store)(struct ufstw_lu *tw, const char *buf, size_t count);
};

struct ufshcd_lrb;

void ufstw_get_dev_info(struct ufstw_dev_info *tw_dev_info, u8 *desc_buf);
void ufstw_get_geo_info(struct ufstw_dev_info *tw_dev_info, u8 *geo_buf);
int ufstw_get_lu_info(struct ufsf_feature *ufsf, int lun, u8 *lu_buf);
void ufstw_init(struct ufsf_feature *ufsf);
void ufstw_prep_fn(struct ufsf_feature *ufsf, struct ufshcd_lrb *lrbp);
void ufstw_init_work_fn(struct work_struct *work);
void ufstw_reset_work_fn(struct work_struct *work);
void ufstw_suspend(struct ufsf_feature *ufsf);
void ufstw_release(struct kref *kref);

#endif /* End of Header */
