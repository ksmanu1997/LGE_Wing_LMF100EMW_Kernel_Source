/*
 * Universal Flash Storage Turbo Write
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

#include <uapi/scsi/ufs/ufs.h>

#include "ufshcd.h"
#include "ufstw.h"

static int ufstw_create_sysfs(struct ufsf_feature *ufsf, struct ufstw_lu *tw);

static inline int ufstw_is_not_present(struct ufstw_lu *tw)
{
	enum UFSTW_STATE cur_state = atomic_read(&tw->ufsf->tw_state);

	if (cur_state != TW_PRESENT) {
		INFO_MSG("tw_state != TW_PRESENT (%d)", cur_state);
		return -ENODEV;
	}
	return 0;
}

static inline void ufstw_lu_get(struct ufstw_lu *tw)
{
	kref_get(&tw->ufsf->tw_kref);
}

static inline void ufstw_lu_put(struct ufstw_lu *tw)
{
	kref_put(&tw->ufsf->tw_kref, ufstw_release);
}

static int ufstw_read_lu_attr(struct ufstw_lu *tw, u8 idn, u32 *attr_val)
{
	struct ufs_hba *hba = tw->ufsf->hba;
	int err;
	u32 val;

	pm_runtime_get_sync(hba->dev);
	ufstw_lu_get(tw);

	err = ufsf_query_attr_retry(hba, UPIU_QUERY_OPCODE_READ_ATTR, idn,
				    (u8)tw->lun, &val);
	if (err) {
		ERR_MSG("read attr [0x%.2X] failed...err %d", idn, err);
		ufstw_lu_put(tw);
		pm_runtime_put_sync(hba->dev);
		return err;
	}

	*attr_val = val;

	blk_add_trace_msg(tw->ufsf->sdev_ufs_lu[tw->lun]->request_queue,
			  "%s:%d IDN %s (%d)", __func__, __LINE__,
			  idn == QUERY_ATTR_IDN_WB_FLUSH_STATUS ? "TW_FLUSH_STATUS" :
			  idn == QUERY_ATTR_IDN_AVAIL_WB_BUFF_SIZE  ? "TW_BUF_SIZE" :
			  idn == QUERY_ATTR_IDN_WB_BUFF_LIFE_TIME_EST ? "TW_BUF_LIFETIME_EST" :
			  "UNKNOWN", idn);

	TW_DEBUG(tw->ufsf, "tw_attr LUN(%d) [0x%.2X] %u", tw->lun, idn,
		 *attr_val);

	ufstw_lu_put(tw);
	pm_runtime_put_sync(hba->dev);

	return 0;
}

static int ufstw_set_lu_flag(struct ufstw_lu *tw, u8 idn, bool *flag_res)
{
	struct ufs_hba *hba = tw->ufsf->hba;
	int err;

	pm_runtime_get_sync(hba->dev);
	ufstw_lu_get(tw);

	err = ufsf_query_flag_retry(hba, UPIU_QUERY_OPCODE_SET_FLAG, idn,
				    (u8)tw->lun, NULL);
	if (err) {
		ERR_MSG("set flag [0x%.2X] failed...err %d", idn, err);
		ufstw_lu_put(tw);
		pm_runtime_put_sync(hba->dev);
		return err;
	}

	*flag_res = true;
	blk_add_trace_msg(tw->ufsf->sdev_ufs_lu[tw->lun]->request_queue,
			  "%s:%d IDN %s (%d)", __func__, __LINE__,
			  idn == QUERY_FLAG_IDN_WB_EN  ? "TW_EN" :
			  idn == QUERY_FLAG_IDN_WB_BUFF_FLUSH_EN  ? "FLUSH_EN" :
			  idn == QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8 ?
			  "HIBERN_EN" : "UNKNOWN", idn);

	TW_DEBUG(tw->ufsf, "tw_flag LUN(%d) [0x%.2X] %u", tw->lun, idn,
		 *flag_res);

	ufstw_lu_put(tw);
	pm_runtime_put_sync(hba->dev);

	return 0;
}

static int ufstw_clear_lu_flag(struct ufstw_lu *tw, u8 idn, bool *flag_res)
{
	struct ufs_hba *hba = tw->ufsf->hba;
	int err;

	pm_runtime_get_sync(hba->dev);
	ufstw_lu_get(tw);

	err = ufsf_query_flag_retry(hba, UPIU_QUERY_OPCODE_CLEAR_FLAG, idn,
				    (u8)tw->lun, NULL);
	if (err) {
		ERR_MSG("clear flag [0x%.2X] failed...err%d", idn, err);
		ufstw_lu_put(tw);
		pm_runtime_put_sync(hba->dev);
		return err;
	}

	*flag_res = false;

	blk_add_trace_msg(tw->ufsf->sdev_ufs_lu[tw->lun]->request_queue,
			  "%s:%d IDN %s (%d)", __func__, __LINE__,
			  idn == QUERY_FLAG_IDN_WB_EN  ? "TW_EN" :
			  idn == QUERY_FLAG_IDN_WB_BUFF_FLUSH_EN  ? "FLUSH_EN" :
			  idn == QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8 ? "HIBERN_EN" :
			  "UNKNOWN", idn);

	TW_DEBUG(tw->ufsf, "tw_flag LUN(%d) [0x%.2X] %u", tw->lun, idn,
		 *flag_res);

	ufstw_lu_put(tw);
	pm_runtime_put_sync(hba->dev);
	return 0;
}

static int ufstw_read_lu_flag(struct ufstw_lu *tw, u8 idn, bool *flag_res)
{
	struct ufs_hba *hba = tw->ufsf->hba;
	int err;
	bool val;

	pm_runtime_get_sync(hba->dev);
	ufstw_lu_get(tw);

	err = ufsf_query_flag_retry(hba, UPIU_QUERY_OPCODE_READ_FLAG, idn,
				    (u8)tw->lun, &val);
	if (err) {
		ERR_MSG("read flag [0x%.2X] failed...err%d", idn, err);
		ufstw_lu_put(tw);
		pm_runtime_put_sync(hba->dev);
		return err;
	}

	*flag_res = val;

	TW_DEBUG(tw->ufsf, "tw_flag LUN(%d) [0x%.2X] %u", tw->lun, idn,
		 *flag_res);

	ufstw_lu_put(tw);
	pm_runtime_put_sync(hba->dev);
	return 0;

}

static void ufstw_switch_disable_state(struct ufstw_lu *tw)
{
	int err = 0;

	WARNING_MSG("dTurboWriteBUfferLifeTImeEst 0x%X", tw->tw_lifetime_est);
	WARNING_MSG("the tw_state will be changed to disabled state.");

	mutex_lock(&tw->sysfs_lock);
	atomic_set(&tw->ufsf->tw_state, TW_DISABLED);
	if (tw->tw_enable) {
		err = ufstw_clear_lu_flag(tw, QUERY_FLAG_IDN_WB_EN ,
					  &tw->tw_enable);
		if (err)
			WARNING_MSG("tw_enable flag clear fail!!");
	}
	mutex_unlock(&tw->sysfs_lock);
}

static int ufstw_check_lifetime_not_guarantee(struct ufstw_lu *tw)
{
	bool disable_flag = false;

	if (tw->tw_lifetime_est & MASK_UFSTW_LIFETIME_NOT_GUARANTEE) {
		WARNING_MSG("warn: lun %d - dTurboWriteBufferLifeTimeEst[31] == 1", tw->lun);
		WARNING_MSG("Device not guarantee the lifetime of Turbo Write Buffer");
#if defined(CONFIG_UFSTW_IGNORE_GUARANTEE_BIT)
		WARNING_MSG("but we will ignore them for PoC");
#else
		disable_flag = true;
#endif
	}

	if (disable_flag ||
	    (tw->tw_lifetime_est & ~MASK_UFSTW_LIFETIME_NOT_GUARANTEE) >=
	    UFSTW_MAX_LIFETIME_VALUE) {
		ufstw_switch_disable_state(tw);
		return -ENODEV;
	}

	return 0;
}

static void ufstw_lifetime_work_fn(struct work_struct *work)
{
	struct ufstw_lu *tw;

	tw = container_of(work, struct ufstw_lu, tw_lifetime_work);

	ufstw_lu_get(tw);

	if (ufstw_is_not_present(tw))
		goto out;

	if (ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_WB_BUFF_LIFE_TIME_EST,
			       &tw->tw_lifetime_est))
		goto out;

	ufstw_check_lifetime_not_guarantee(tw);
out:
	ufstw_lu_put(tw);
	return;
}

static inline bool ufstw_is_write_lrbp(struct ufshcd_lrb *lrbp)
{
	if (lrbp->cmd->cmnd[0] == WRITE_10 || lrbp->cmd->cmnd[0] == WRITE_16)
		return true;

	return false;
}

void ufstw_prep_fn(struct ufsf_feature *ufsf, struct ufshcd_lrb *lrbp)
{
	struct ufstw_lu *tw;

	if (!lrbp || !ufsf_is_valid_lun(lrbp->lun))
		return;

	if (!ufstw_is_write_lrbp(lrbp))
		return;

	tw = ufsf->tw_lup[lrbp->lun];
	if (!tw)
		return;

	if (!tw->tw_enable)
		return;

	spin_lock_bh(&tw->lifetime_lock);
	tw->stat_write_sec += blk_rq_sectors(lrbp->cmd->request);

	if (tw->stat_write_sec > UFSTW_LIFETIME_SECT) {
		tw->stat_write_sec = 0;
		spin_unlock_bh(&tw->lifetime_lock);
		schedule_work(&tw->tw_lifetime_work);
		return;
	}
	spin_unlock_bh(&tw->lifetime_lock);
}

static inline void ufstw_init_dev_jobs(struct ufsf_feature *ufsf)
{
	INIT_INFO("INIT_WORK(tw_reset_work)");
	INIT_WORK(&ufsf->tw_reset_work, ufstw_reset_work_fn);
}

static inline void ufstw_init_lu_jobs(struct ufstw_lu *tw)
{
	INIT_INFO("INIT_WORK(tw_lifetime_work)");
	INIT_WORK(&tw->tw_lifetime_work, ufstw_lifetime_work_fn);
}

static inline void ufstw_cancel_lu_jobs(struct ufstw_lu *tw)
{
	int ret;

	ret = cancel_work_sync(&tw->tw_lifetime_work);
	INIT_INFO("cancel_work_sync(tw_lifetime_work) ufstw_lu%d = %d",
		  tw->lun, ret);
}

static inline int ufstw_version_check(struct ufstw_dev_info *tw_dev_info)
{
	INIT_INFO("Support TW Spec : Driver = %.4X, Device = %.4X",
		  UFSTW_VER, tw_dev_info->tw_ver);

	INIT_INFO("TW Driver Version : %.6X", UFSTW_DD_VER);

	if (tw_dev_info->tw_ver != UFSTW_VER) {
		ERR_MSG("ERROR: TW Spec Version mismatch. So TW disabled.");
		return -ENODEV;
	}
	return 0;
}

void ufstw_get_dev_info(struct ufstw_dev_info *tw_dev_info, u8 *desc_buf)
{
	tw_dev_info->tw_device = false;

	if (UFSF_EFS_TURBO_WRITE
	    & LI_EN_32(&desc_buf[DEVICE_DESC_PARAM_EXT_UFS_FEATURE_SUP]))
		INIT_INFO("bUFSExFeaturesSupport: TW support");
	else {
		INIT_INFO("bUFSExFeaturesSupport: TW not support");
		return;
	}
	tw_dev_info->tw_buf_no_reduct =
		desc_buf[DEVICE_DESC_PARAM_WB_US_RED_EN];
	tw_dev_info->tw_buf_type = desc_buf[DEVICE_DESC_PARAM_WB_TYPE];

	tw_dev_info->tw_ver = LI_EN_16(&desc_buf[DEVICE_DESC_PARAM_WB_SHARED_ALLOC_UNITS]);

	if (!ufstw_version_check(tw_dev_info))
		tw_dev_info->tw_device = true;

	INFO_MSG("tw_dev [53] bTurboWriteBufferNoUserSpaceReductionEn %u",
		 tw_dev_info->tw_buf_no_reduct);
	INFO_MSG("tw_dev [54] bTurboWriteBufferType %u",
		 tw_dev_info->tw_buf_type);
}

void ufstw_get_geo_info(struct ufstw_dev_info *tw_dev_info, u8 *geo_buf)
{
	tw_dev_info->tw_number_lu = geo_buf[GEOMETRY_DESC_PARAM_WB_MAX_WB_LUNS];
	if (tw_dev_info->tw_number_lu == 0) {
		ERR_MSG("Turbo Write is not supported");
		tw_dev_info->tw_device = false;
		return;
	}

	INFO_MSG("tw_geo [4F:52] dTurboWriteBufferMaxNAllocUnits %u",
		 LI_EN_32(&geo_buf[GEOMETRY_DESC_PARAM_WB_MAX_ALLOC_UNITS]));
	INFO_MSG("tw_geo [53] bDeviceMaxTurboWriteLUs %u",
		 tw_dev_info->tw_number_lu);
	INFO_MSG("tw_geo [54] bTurboWriteBufferCapAdjFac %u",
		 geo_buf[GEOMETRY_DESC_PARAM_WB_BUFF_CAP_ADJ]);
	INFO_MSG("tw_geo [55] bSupportedTurboWriteBufferUserSpaceReductionTypes %u",
		 geo_buf[GEOMETRY_DESC_PARAM_WB_SUP_RED_TYPE]);
	INFO_MSG("tw_geo [56] bSupportedTurboWriteBufferTypes %u",
		 geo_buf[GEOMETRY_DESC_PARAM_WB_SUP_WB_TYPE]);
}

int ufstw_get_lu_info(struct ufsf_feature *ufsf, int lun, u8 *lu_buf)
{
	struct ufsf_lu_desc lu_desc;
	struct ufstw_lu *tw;

	lu_desc.tw_lu_buf_size =
		LI_EN_32(&lu_buf[UNIT_DESC_PARAM_WB_BUF_ALLOC_UNITS]);

	ufsf->tw_lup[lun] = NULL;

	if (lu_desc.tw_lu_buf_size) {
		ufsf->tw_lup[lun] =
			kzalloc(sizeof(struct ufstw_lu), GFP_KERNEL);
		if (!ufsf->tw_lup[lun])
			return -ENOMEM;

		tw = ufsf->tw_lup[lun];
		tw->ufsf = ufsf;
		tw->lun = lun;
		INIT_INFO("tw_lu LUN(%d) [29:2C] dLUNumTurboWriteBufferAllocUnits %u",
			  lun, lu_desc.tw_lu_buf_size);
	} else {
		INIT_INFO("tw_lu LUN(%d) [29:2C] dLUNumTurboWriteBufferAllocUnits %u",
			  lun, lu_desc.tw_lu_buf_size);
		INIT_INFO("===== LUN(%d) is TurboWrite-disabled.", lun);
		return -ENODEV;
	}

	return 0;
}

static inline void ufstw_print_lu_flag_attr(struct ufstw_lu *tw)
{
	INFO_MSG("tw_flag LUN(%d) [%u] fTurboWriteEn %u", tw->lun,
		 QUERY_FLAG_IDN_WB_EN , tw->tw_enable);
	INFO_MSG("tw_flag LUN(%d) [%u] fTurboWriteBufferFlushEn %u", tw->lun,
		 QUERY_FLAG_IDN_WB_BUFF_FLUSH_EN , tw->tw_flush_enable);
	INFO_MSG("tw_flag LUN(%d) [%u] fTurboWriteBufferFlushDuringHibernateEnter %u",
		 tw->lun, QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8,
		 tw->tw_flush_during_hibern_enter);

	INFO_MSG("tw_attr LUN(%d) [%u] flush_status  %u", tw->lun,
		 QUERY_ATTR_IDN_WB_FLUSH_STATUS, tw->tw_flush_status);
	INFO_MSG("tw_attr LUN(%d) [%u] buffer_size  %u", tw->lun,
		 QUERY_ATTR_IDN_AVAIL_WB_BUFF_SIZE , tw->tw_available_buffer_size);
	INFO_MSG("tw_attr LUN(%d) [%d] bufffer_lifetime  %u(0x%X)",
		 tw->lun, QUERY_ATTR_IDN_WB_BUFF_LIFE_TIME_EST,
		 tw->tw_lifetime_est, tw->tw_lifetime_est);
}

static inline void ufstw_lu_update(struct ufstw_lu *tw)
{
	ufstw_lu_get(tw);

	/* Flag */
	if (ufstw_read_lu_flag(tw, QUERY_FLAG_IDN_WB_EN , &tw->tw_enable))
		goto error_put;

	if (ufstw_read_lu_flag(tw, QUERY_FLAG_IDN_WB_BUFF_FLUSH_EN ,
			       &tw->tw_flush_enable))
		goto error_put;

	if (ufstw_read_lu_flag(tw, QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8,
			       &tw->tw_flush_during_hibern_enter))
		goto error_put;

	/* Attribute */
	if (ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_WB_FLUSH_STATUS,
			       &tw->tw_flush_status))
		goto error_put;

	if (ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_AVAIL_WB_BUFF_SIZE ,
			       &tw->tw_available_buffer_size))
		goto error_put;

	ufstw_read_lu_attr(tw, QUERY_ATTR_IDN_WB_BUFF_LIFE_TIME_EST,
			       &tw->tw_lifetime_est);
error_put:
	ufstw_lu_put(tw);
}

static int ufstw_lu_init(struct ufsf_feature *ufsf, int lun)
{
	struct ufstw_lu *tw = ufsf->tw_lup[lun];
	int ret = 0;

	ufstw_lu_get(tw);
	tw->ufsf = ufsf;

	/* Read Flag, Attribute */
	ufstw_lu_update(tw);

	mutex_init(&tw->sysfs_lock);
	spin_lock_init(&tw->lifetime_lock);

	ret = ufstw_check_lifetime_not_guarantee(tw);
	if (ret)
		goto err_out;
	ufstw_set_lu_flag(tw, QUERY_FLAG_IDN_WB_EN, &tw->tw_enable);
	ufstw_set_lu_flag(tw, QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8,
			  &tw->tw_flush_during_hibern_enter);
	ufstw_print_lu_flag_attr(tw);

	tw->stat_write_sec = 0;

	ufstw_init_lu_jobs(tw);

	if (ufstw_create_sysfs(ufsf, tw))
		INIT_INFO("sysfs init fail. but tw could run normally.");

err_out:
	ufstw_lu_put(tw);
	return ret;
}

void ufstw_init(struct ufsf_feature *ufsf)
{
	int lun, ret = 0;
	unsigned int tw_enabled_lun = 0;

	kref_init(&ufsf->tw_kref);
	seq_scan_lu(lun) {
		if (!ufsf->tw_lup[lun])
			continue;

		if (!ufsf->sdev_ufs_lu[lun]) {
			WARNING_MSG("warn: lun %d don't have scsi_device", lun);
			continue;
		}

		ret = ufstw_lu_init(ufsf, lun);
		if (ret)
			goto out_free_mem;

		INIT_INFO("UFSTW LU %d working", lun);
		tw_enabled_lun++;
	}

	if (tw_enabled_lun == 0) {
		ERR_MSG("ERROR: tw_enabled_lun == 0. So TW disabled.");
		goto out_free_mem;
	}

	if (tw_enabled_lun > ufsf->tw_dev_info.tw_number_lu) {
		ERR_MSG("ERROR: dev_info(bDeviceMaxTurboWriteLUs) mismatch. So TW disabled.");
		goto out_free_mem;
	}

	/*
	 * Initialize Device Level...
	 */
	ufstw_init_dev_jobs(ufsf);
	ufsf->tw_debug = false;
	atomic_set(&ufsf->tw_state, TW_PRESENT);
	return;
out_free_mem:
	seq_scan_lu(lun)
		kfree(ufsf->tw_lup[lun]);

	ufsf->tw_dev_info.tw_device = false;
	atomic_set(&ufsf->tw_state, TW_DISABLED);
}

static inline int ufstw_probe_lun_done(struct ufsf_feature *ufsf)
{
	return (ufsf->num_lu == ufsf->slave_conf_cnt);
}

void ufstw_init_work_fn(struct work_struct *work)
{
	struct ufsf_feature *ufsf;
	int ret;

	ufsf = container_of(work, struct ufsf_feature, tw_init_work);

	init_waitqueue_head(&ufsf->tw_wait);

	ret = wait_event_timeout(ufsf->tw_wait,
				 ufstw_probe_lun_done(ufsf),
				 msecs_to_jiffies(10000));
	if (ret == 0) {
		ERR_MSG("Probing LU is not fully completed.");
		return;
	}

	INIT_INFO("TW_INIT_START");

	ufstw_init(ufsf);
}

void ufstw_suspend(struct ufsf_feature *ufsf)
{
	struct ufstw_lu *tw;
	int lun;
	int ret;

	ret = flush_work(&ufsf->tw_reset_work);
	TW_DEBUG(ufsf, "flush_work(tw_reset_work) = %d", ret);

	seq_scan_lu(lun) {
		tw = ufsf->tw_lup[lun];
		if (!tw)
			continue;

		ufstw_lu_get(tw);
		INFO_MSG("ufstw_lu%d goto suspend", lun);
		ufstw_cancel_lu_jobs(tw);
		ufstw_lu_put(tw);
	}
}

void ufstw_release(struct kref *kref)
{
	struct ufsf_feature *ufsf;
	struct ufstw_lu *tw;
	int lun;
	int ret;

	ufsf = container_of(kref, struct ufsf_feature, tw_kref);
	RELEASE_INFO("start release");

	RELEASE_INFO("tw_state : %d -> %d", atomic_read(&ufsf->tw_state),
		     TW_DISABLED);
	atomic_set(&ufsf->tw_state, TW_DISABLED);

	RELEASE_INFO("kref count %d",
		     atomic_read(&ufsf->tw_kref.refcount.refs));

	ret = cancel_work_sync(&ufsf->tw_reset_work);
	RELEASE_INFO("cancel_work_sync(tw_reset_work) = %d", ret);

	seq_scan_lu(lun) {
		tw = ufsf->tw_lup[lun];

		RELEASE_INFO("ufstw_lu%d %p", lun, tw);

		ufsf->tw_lup[lun] = NULL;

		if (!tw)
			continue;

		ufstw_cancel_lu_jobs(tw);

		ret = kobject_uevent(&tw->kobj, KOBJ_REMOVE);
		RELEASE_INFO("kobject error %d", ret);

		kobject_del(&tw->kobj);

		kfree(tw);
	}

	RELEASE_INFO("end release");
}

static void ufstw_reset(struct ufsf_feature *ufsf)
{
	struct ufstw_lu *tw;
	int lun;
	int ret;

	/*
	 * ufstw state was changed to TW_RESET in ufshcd_host_reset_and_restore().
	 * It is possible to change the state to TW_DISABLED
	 * by lifetime_work_fn in this time.
	 */
	if (atomic_read(&ufsf->tw_state) == TW_DISABLED) {
		ERR_MSG("tw_state == TW_DISABLED(%d)",
			atomic_read(&ufsf->tw_state));
		return;
	}

	seq_scan_lu(lun) {
		tw = ufsf->tw_lup[lun];
		TW_DEBUG(ufsf, "reset tw[%d]=%p", lun, tw);
		if (!tw)
			continue;

		INFO_MSG("ufstw_lu%d reset", lun);

		ufstw_lu_get(tw);
		ufstw_cancel_lu_jobs(tw);

		if (tw->tw_enable) {
			ret = ufstw_set_lu_flag(tw, QUERY_FLAG_IDN_WB_EN ,
						&tw->tw_enable);
			if (ret)
				tw->tw_enable = false;
		}

		if (tw->tw_flush_enable) {
			ret = ufstw_set_lu_flag(tw,
						QUERY_FLAG_IDN_WB_BUFF_FLUSH_EN ,
						&tw->tw_flush_enable);
			if (ret)
				tw->tw_flush_enable = false;
		}

		if (tw->tw_flush_during_hibern_enter) {
			ret = ufstw_set_lu_flag(tw,
						QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8,
						&tw->tw_flush_during_hibern_enter);
			if (ret)
				tw->tw_flush_during_hibern_enter = false;
		}
		ufstw_lu_put(tw);
	}

	atomic_set(&ufsf->tw_state, TW_PRESENT);
	INFO_MSG("reset complete.. tw_state %d", atomic_read(&ufsf->tw_state));
}

static inline int ufstw_wait_kref_init_value(struct ufsf_feature *ufsf)
{
	return (atomic_read(&ufsf->tw_kref.refcount.refs) == 1);
}

void ufstw_reset_work_fn(struct work_struct *work)
{
	struct ufsf_feature *ufsf;
	int ret;

	ufsf = container_of(work, struct ufsf_feature, tw_reset_work);
	TW_DEBUG(ufsf, "reset tw_kref.refcount=%d",
		 atomic_read(&ufsf->tw_kref.refcount.refs));

	init_waitqueue_head(&ufsf->tw_wait);

	ret = wait_event_timeout(ufsf->tw_wait,
				 ufstw_wait_kref_init_value(ufsf),
				 msecs_to_jiffies(15000));
	if (ret == 0) {
		ERR_MSG("UFSTW kref is not init_value(=1). kref count = %d ret = %d. So, TW_RESET_FAIL",
			atomic_read(&ufsf->tw_kref.refcount.refs), ret);
		return;
	}

	INIT_INFO("TW_RESET_START");

	ufstw_reset(ufsf);
}

/* sysfs function */
static ssize_t ufstw_sysfs_show_flush_during_hibern_enter(struct ufstw_lu *tw,
							  char *buf)
{
	int ret;

	if (ufstw_is_not_present(tw))
		return -ENODEV;

	if (ufstw_read_lu_flag(tw, QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8,
			       &tw->tw_flush_during_hibern_enter))
		return -ENODEV;

	SYSFS_INFO("TW_flush_during_hibern_enter %d",
		   tw->tw_flush_during_hibern_enter);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", tw->tw_flush_during_hibern_enter);
	return ret;
}

static ssize_t ufstw_sysfs_store_flush_during_hibern_enter(struct ufstw_lu *tw,
							   const char *buf,
							   size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (ufstw_is_not_present(tw))
		return -ENODEV;

	if (val) {
		if (ufstw_set_lu_flag(tw,
				      QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8,
				      &tw->tw_flush_during_hibern_enter))
			return -EINVAL;
	} else {
		if (ufstw_clear_lu_flag(tw,
					QUERY_FLAG_IDN_WB_BUFF_FLUSH_DURING_HIBERN8,
					&tw->tw_flush_during_hibern_enter))
			return -EINVAL;
	}

	SYSFS_INFO("TW_flush_during_hibern_enter %d",
		   tw->tw_flush_during_hibern_enter);

	return count;
}

static ssize_t ufstw_sysfs_show_flush_enable(struct ufstw_lu *tw, char *buf)
{
	int ret;

	if (ufstw_is_not_present(tw))
		return -ENODEV;

	if (ufstw_read_lu_flag(tw, QUERY_FLAG_IDN_WB_BUFF_FLUSH_EN ,
			       &tw->tw_flush_enable))
		return -EINVAL;

	SYSFS_INFO("TW_flush_enable %d", tw->tw_flush_enable);

	ret = snprintf(buf, PAGE_SIZE, "%d\n", tw->tw_flush_enable);

	return ret;
}

static ssize_t ufstw_sysfs_store_flush_enable(struct ufstw_lu *tw,
					      const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (ufstw_is_not_present(tw))
		return -ENODEV;

	if (val) {
		if (ufstw_set_lu_flag(tw, QUERY_FLAG_IDN_WB_BUFF_FLUSH_EN   ,
				      &tw->tw_flush_enable))
			return -EINVAL;
	} else {
		if (ufstw_clear_lu_flag(tw, QUERY_FLAG_IDN_WB_BUFF_FLUSH_EN ,
					&tw->tw_flush_enable))
			return -EINVAL;
	}

	SYSFS_INFO("TW_flush_enable %d", tw->tw_flush_enable);

	return count;
}

static ssize_t ufstw_sysfs_show_debug(struct ufstw_lu *tw, char *buf)
{
	SYSFS_INFO("debug %d", tw->ufsf->tw_debug);

	return snprintf(buf, PAGE_SIZE, "%d\n", tw->ufsf->tw_debug);
}

static ssize_t ufstw_sysfs_store_debug(struct ufstw_lu *tw, const char *buf,
				       size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	tw->ufsf->tw_debug = val ? true : false;

	SYSFS_INFO("debug %d", tw->ufsf->tw_debug);

	return count;
}

static ssize_t ufstw_sysfs_show_version(struct ufstw_lu *tw, char *buf)
{
	SYSFS_INFO("TW version %.4X D/D version %.6X",
		   tw->ufsf->tw_dev_info.tw_ver, UFSTW_DD_VER);

	return snprintf(buf, PAGE_SIZE, "TW version %.4X DD version %.6X\n",
			tw->ufsf->tw_dev_info.tw_ver, UFSTW_DD_VER);
}

/* SYSFS DEFINE */
#define define_sysfs_ro(_name) __ATTR(_name, 0444,\
				      ufstw_sysfs_show_##_name, NULL),
#define define_sysfs_rw(_name) __ATTR(_name, 0644,\
				      ufstw_sysfs_show_##_name, \
				      ufstw_sysfs_store_##_name),

#define define_sysfs_attr_r_function(_name, _IDN) \
static ssize_t ufstw_sysfs_show_##_name(struct ufstw_lu *tw, char *buf) \
{ \
	if (ufstw_read_lu_attr(tw, _IDN, &tw->tw_##_name))\
		return -EINVAL;\
	SYSFS_INFO("TW_"#_name" : %u (0x%X)", tw->tw_##_name, tw->tw_##_name); \
	return snprintf(buf, PAGE_SIZE, "%u\n", tw->tw_##_name); \
}

/* SYSFS FUNCTION */
define_sysfs_attr_r_function(flush_status, QUERY_ATTR_IDN_WB_FLUSH_STATUS)
define_sysfs_attr_r_function(available_buffer_size, QUERY_ATTR_IDN_AVAIL_WB_BUFF_SIZE )
define_sysfs_attr_r_function(lifetime_est, QUERY_ATTR_IDN_WB_BUFF_LIFE_TIME_EST)

static ssize_t ufstw_sysfs_show_tw_enable(struct ufstw_lu *tw, char *buf)
{
	if (ufstw_is_not_present(tw))
		return -ENODEV;

	if (ufstw_read_lu_flag(tw, QUERY_FLAG_IDN_WB_EN  , &tw->tw_enable))
		return -EINVAL;

	SYSFS_INFO("TW_enable: %u (0x%X)", tw->tw_enable, tw->tw_enable);
	return snprintf(buf, PAGE_SIZE, "%u\n", tw->tw_enable);
}

static ssize_t ufstw_sysfs_store_tw_enable(struct ufstw_lu *tw, const char *buf,
					   size_t count)
{
	unsigned long val;
	ssize_t ret = count;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (ufstw_is_not_present(tw))
		return -ENODEV;

	if (val) {
		if (ufstw_set_lu_flag(tw, QUERY_FLAG_IDN_WB_EN ,
				      &tw->tw_enable))
			return -EINVAL;
	} else {
		if (ufstw_clear_lu_flag(tw, QUERY_FLAG_IDN_WB_EN ,
					&tw->tw_enable))
			return -EINVAL;
	}

	SYSFS_INFO("TW_enable : %u (0x%X)", tw->tw_enable, tw->tw_enable);
	return ret;
}

static struct ufstw_sysfs_entry ufstw_sysfs_entries[] = {
	/* Flag */
	define_sysfs_rw(tw_enable)
	define_sysfs_rw(flush_enable)
	define_sysfs_rw(flush_during_hibern_enter)

	/* Attribute */
	define_sysfs_ro(flush_status)
	define_sysfs_ro(available_buffer_size)
	define_sysfs_ro(lifetime_est)

	/* debug */
	define_sysfs_rw(debug)

	/* device level */
	define_sysfs_ro(version)
	__ATTR_NULL
};

static ssize_t ufstw_attr_show(struct kobject *kobj, struct attribute *attr,
			       char *page)
{
	struct ufstw_sysfs_entry *entry;
	struct ufstw_lu *tw;
	ssize_t error;

	entry = container_of(attr, struct ufstw_sysfs_entry, attr);
	tw = container_of(kobj, struct ufstw_lu, kobj);
	if (!entry->show)
		return -EIO;

	ufstw_lu_get(tw);
	mutex_lock(&tw->sysfs_lock);
	error = entry->show(tw, page);
	mutex_unlock(&tw->sysfs_lock);
	ufstw_lu_put(tw);
	return error;
}

static ssize_t ufstw_attr_store(struct kobject *kobj, struct attribute *attr,
				const char *page, size_t length)
{
	struct ufstw_sysfs_entry *entry;
	struct ufstw_lu *tw;
	ssize_t error;

	entry = container_of(attr, struct ufstw_sysfs_entry, attr);
	tw = container_of(kobj, struct ufstw_lu, kobj);

	if (!entry->store)
		return -EIO;

	ufstw_lu_get(tw);
	mutex_lock(&tw->sysfs_lock);
	error = entry->store(tw, page, length);
	mutex_unlock(&tw->sysfs_lock);
	ufstw_lu_put(tw);
	return error;
}

static const struct sysfs_ops ufstw_sysfs_ops = {
	.show = ufstw_attr_show,
	.store = ufstw_attr_store,
};

static struct kobj_type ufstw_ktype = {
	.sysfs_ops = &ufstw_sysfs_ops,
	.release = NULL,
};

static int ufstw_create_sysfs(struct ufsf_feature *ufsf, struct ufstw_lu *tw)
{
	struct device *dev = ufsf->hba->dev;
	struct ufstw_sysfs_entry *entry;
	int err;

	ufstw_lu_get(tw);
	tw->sysfs_entries = ufstw_sysfs_entries;

	kobject_init(&tw->kobj, &ufstw_ktype);

	INIT_INFO("ufstw creates sysfs ufstw_lu(%d) %p dev->kobj %p",
		  tw->lun, &tw->kobj, &dev->kobj);

	err = kobject_add(&tw->kobj, kobject_get(&dev->kobj),
			  "ufstw_lu%d", tw->lun);
	if (!err) {
		for (entry = tw->sysfs_entries; entry->attr.name != NULL;
		     entry++) {
			INIT_INFO("ufstw_lu%d sysfs attr creates: %s",
				  tw->lun, entry->attr.name);
			if (sysfs_create_file(&tw->kobj, &entry->attr))
				break;
		}
		INIT_INFO("ufstw_lu%d sysfs adds uevent", tw->lun);
		kobject_uevent(&tw->kobj, KOBJ_ADD);
	}
	ufstw_lu_put(tw);
	return err;
}
