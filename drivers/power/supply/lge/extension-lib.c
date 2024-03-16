/*
 * CAUTION! :
 * 	This file will be included at the end of "smb5-lib.c".
 * 	So "smb5-lib.c" should be touched before you start to build.
 * 	If not, your work will not be applied to the built image
 * 	because the build system may not care the update time of this file.
 */

/************************
 * OVERRIDDEN CALLBACKS *
 ************************/

#define OTG_WLC_VOTER	"OTG_WLC_VOTER"
int override_vbus_regulator_enable(struct regulator_dev *rdev) {
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval pval = { .intval = 0, };
	int ret = 0;
	int rc = 0;

	if (!chg->wls_psy)
		chg->wls_psy = power_supply_get_by_name("wireless");

	if (chg->wls_psy)
		rc = power_supply_get_property(chg->wls_psy, POWER_SUPPLY_PROP_PIN_ENABLED, &pval);

	if (!rc) {
		if (chg->wls_psy && ext_chg->enable_concurrency_otg_wlc && pval.intval) {
			pval.intval = DC_RESET_BY_CONCURRENT_ON;
			power_supply_set_property(chg->wls_psy, POWER_SUPPLY_PROP_DC_RESET, &pval);
			return ret;
		}
	}

	smblib_err(chg, "regulator on\n");

	ret = smblib_vbus_regulator_enable(rdev);
	wa_control_vbus2_regulator(chg, true);

	return ret;
}

int override_vbus_regulator_disable(struct regulator_dev *rdev) {
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	int ret = 0;
	int rc = 0;

	if (ext_chg->enable_concurrency_otg_wlc
		&& ext_chg->concurrency_otg_wlc) {
		smblib_err(chg, "boost ic off\n");

		gpiod_set_value(ext_chg->otg_wlc_en_gpio, 0);
		usleep_range(100, 110);
		gpiod_set_value(ext_chg->otg_wlc_on_gpio, 0);

		vote(chg->usb_icl_votable, OTG_WLC_VOTER, false, 0);
		rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG,
				USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V);
		if (rc < 0) {
			smblib_err(chg, "Couldn't write to USBIN_ADAPTER_ALLOW_CFG_REG rc=%d\n", rc);
		}
	} else {
		smblib_err(chg, "regulator off\n");

		wa_control_vbus2_regulator(chg, false);
		ret = smblib_vbus_regulator_disable(rdev);
	}

	return ret;
}

int override_vconn_regulator_enable(struct regulator_dev *rdev) {
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	int ret = 0;

	gpiod_set_value(ext_chg->vconn_boost_en_gpio, 1);
	ret = smblib_vconn_regulator_enable(rdev);

	return ret;
}

int override_vconn_regulator_disable(struct regulator_dev *rdev) {
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	int ret = 0;

	ret = smblib_vconn_regulator_disable(rdev);
	gpiod_set_value(ext_chg->vconn_boost_en_gpio, 0);

	return ret;
}

irqreturn_t override_otg_oc_hiccup_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	wa_disable_otg_hiccup_trigger(chg);

	// Call original here
	return default_irq_handler(irq, data);
}

irqreturn_t override_otg_fault_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	gpiod_set_value(ext_chg->load_sw_on_gpio, 0);
	gpiod_set_value(ext_chg->ds_en_gpio, 0);

	// Call original here
	return default_irq_handler(irq, data);
}

irqreturn_t override_chg_state_change_irq_handler(int irq, void *data)
{
	const char str_charger_status[8][12] = {
		"inhibit", "trickle", "precharge", "fullon",
		"taper", "termination", "chg pause", "invalid"
	};
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct ext_smb_charger *ext_chg = chg->ext_chg;

	u8 stat;
	union power_supply_propval chgstep;

	if (ext_chg->concurrency_otg_wlc) {
		union power_supply_propval pval = { .intval = 0, };
		int wlc_vnow = !power_supply_get_property(chg->dc_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval) ? pval.intval : -1;

		if (wlc_vnow < 1000000)
			wlc_vnow /= 10;
		else
			wlc_vnow /= 1000;

		if (!chg->wls_psy)
			chg->wls_psy = power_supply_get_by_name("wireless");

		if (chg->wls_psy && (wlc_vnow < DCIN_THRESHOLD_MV))
			vote(chg->dc_suspend_votable, WLC_OTG_VOTER, true, 0);
	}

	if (smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat) >=0) {
		stat &= BATTERY_CHARGER_STATUS_MASK;
		smblib_dbg(chg, PR_INTERRUPT, "chg_state_change to 0x%X, %s\n",
			stat, str_charger_status[stat&0x7]);
	}

	// Update filtered charging status to VENEER system
	chgstep.intval = stat;
	power_supply_set_property(chg->batt_psy, POWER_SUPPLY_PROP_CHARGE_TYPE, &chgstep);

	// Call original here
	return chg_state_change_irq_handler(irq, data);
}

irqreturn_t override_usb_plugin_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	irqreturn_t rc = IRQ_NONE;
	bool vbus_rising;
	u8 stat;
	int ret = 0;

	ret = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (ret < 0) {
		smblib_err(chg, "Couldn't read INT_RT_STS_OFFSET rc=%d\n", ret);
	}
	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	if (ext_chg->concurrency_otg_wlc) {
		smblib_dbg(chg, PR_MISC, "Skip plugin irq(%d) handler!\n", vbus_rising);
		return rc;
	}

	// Call original here
	rc = usb_plugin_irq_handler(irq, data);
	schedule_work(&ext_chg->usb_plugin_work);

	return rc;
}

irqreturn_t override_usb_source_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	irqreturn_t rc = IRQ_NONE;

	// Call and return original here
	rc = usb_source_change_irq_handler(irq, data);
	schedule_work(&ext_chg->usb_source_change_work);

	return rc;
}

irqreturn_t override_typec_attach_detach_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	irqreturn_t rc = IRQ_NONE;
	int ret = 0;
	u8 legacy, misc;

	// Call and return original here
	rc = typec_attach_detach_irq_handler(irq, data);

	if (chg->early_usb_attach && chg->typec_legacy && chg->ok_to_pd && unified_bootmode_fabproc()) {
		chg->ok_to_pd = false;
		smblib_dbg(chg, PR_MISC, "Rerun APSD for factory cable\n");
		extension_hvdcp_detect_try_enable(chg, true);
		smblib_rerun_apsd_if_required(chg);
	}

	ret = smblib_read(chg, LEGACY_CABLE_STATUS_REG, &legacy);
	if (ret < 0)
		smblib_err(chg, "Couldn't read LEGACY_CABLE_STATUS_REG rc=%d\n", ret);

	ret = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &misc);
	if (ret < 0)
		smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n", ret);

	if (!delayed_work_pending(&chg->pl_enable_work) &&
			get_client_vote(chg->pl_disable_votable, PL_DELAY_VOTER)) {
		switch(chg->typec_mode) {
			case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
			case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
			case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
				smblib_dbg(chg, PR_PARALLEL, "typec : %s, schedule pl_enable_work after %dms\n",
							smblib_typec_mode_name[chg->typec_mode], PL_DELAY_MS);
				schedule_delayed_work(&chg->pl_enable_work, msecs_to_jiffies(PL_DELAY_MS));
				break;
			default:
				break;
		}
	}

	smblib_dbg(chg, PR_INTERRUPT, "legacy(%d)  ok_to_pd(%d) legacy_status(0x%02x), misc_status(0x%02x)\n",
		chg->typec_legacy, chg->ok_to_pd, legacy, misc);
	return rc;
}

irqreturn_t override_typec_state_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval pval = { .intval = 0, };
	irqreturn_t rc = IRQ_NONE;

	// Call and return original here
	rc = typec_state_change_irq_handler(irq, data);
	schedule_work(&ext_chg->usb_source_change_work);

	if (chg->typec_mode != POWER_SUPPLY_TYPEC_NONE
			&& chg->real_charger_type != POWER_SUPPLY_TYPE_UNKNOWN
			&& get_client_vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER) <= USBIN_100MA) {
		if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP
				|| chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3)
			vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, HVDCP_CURRENT_UA);
		else
			smblib_handle_rp_change(chg, chg->typec_mode);
	}

	if (ext_chg->concurrency_otg_wlc
		&& chg->typec_mode == POWER_SUPPLY_TYPEC_NONE) {
		override_vbus_regulator_disable(chg->vbus_vreg->rdev);
		ext_chg->concurrency_otg_wlc = false;
		if (chg->wls_psy) {
			pval.intval = DC_RESET_BY_CONCURRENT_OFF;
			power_supply_set_property(chg->wls_psy, POWER_SUPPLY_PROP_DC_RESET, &pval);
		}
	}

	return rc;
}

irqreturn_t override_typec_or_rid_detection_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	irqreturn_t rc = IRQ_NONE;
	int ret;
	u8 machine, misc, snk_debug, src_status, legacy;

	// Call and return original here
	rc = typec_or_rid_detection_change_irq_handler(irq, data);

	ret = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &misc);
	if (ret < 0)
		smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n", ret);

	ret = smblib_read(chg, TYPE_C_STATE_MACHINE_STATUS_REG, &machine);
	if (ret < 0)
		smblib_err(chg, "Couldn't read TYPE_C_STATE_MACHINE_STATUS_REG rc=%d\n", ret);

#ifdef CONFIG_LGE_USB
	ret = smblib_read(chg, TYPE_C_SNK_DEBUG_ACCESS_STATUS_REG, &snk_debug);
	if (ret < 0)
		smblib_err(chg, "Couldn't read TYPE_C_SNK_DEBUG_ACCESS_STATUS_REG rc=%d\n", ret);
#else
	snk_debug = 0;
#endif

	ret = smblib_read(chg, TYPE_C_SRC_STATUS_REG, &src_status);
	if (ret < 0)
		smblib_err(chg, "Couldn't read TYPE_C_SRC_STATUS_REG rc=%d\n", ret);

	ret = smblib_read(chg, LEGACY_CABLE_STATUS_REG, &legacy);
	if (ret < 0)
		smblib_err(chg, "Couldn't read LEGACY_CABLE_STATUS_REG rc=%d\n", ret);

	smblib_dbg(chg, PR_INTERRUPT, "misc(0x%02x), machine(0x%02x), snk_debug(0x%02x), src_status(0x%02x), legacy(0x%02x)\n",
		misc, machine, snk_debug, src_status, legacy);

	return rc;
}

irqreturn_t override_usbin_uv_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	irqreturn_t rc = IRQ_NONE;

	// Call and return original here
	rc = usbin_uv_irq_handler(irq, data);

	if (chg->qc2_unsupported_voltage)
		smblib_dbg(chg, PR_INTERRUPT, "Disable QC 2.0 at %d\n", chg->qc2_unsupported_voltage);

	return rc;
}

irqreturn_t override_switcher_power_ok_irq_handler(int irq, void *data)
{
	const char str_power_path[4][9] =
		{ "Not Used", "battery", "usbin", "dcin" };
	struct smb_irq_data *irq_data = data;
	struct smb_charger* chg = irq_data->parent_data;
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval buf = { 0, };
	irqreturn_t rc = IRQ_NONE;
	u8 stat;
	int ret;
	int wlc_vnow = !power_supply_get_property(chg->dc_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &buf) ? buf.intval : -1;

	if (wlc_vnow < 1000000)
		wlc_vnow /= 10;
	else
		wlc_vnow /= 1000;

	if (ext_chg->concurrency_otg_wlc) {
		if (!chg->wls_psy)
			chg->wls_psy = power_supply_get_by_name("wireless");

		if (chg->wls_psy && (wlc_vnow < DCIN_THRESHOLD_MV))
			vote(chg->dc_suspend_votable, WLC_OTG_VOTER, true, 0);
	}

	// Call and return original here
	rc = switcher_power_ok_irq_handler(irq, data);

	ret = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (ret < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n", ret);
		return rc;
	}
	wa_support_weak_supply_trigger(chg, stat);

	smblib_dbg(chg, PR_INTERRUPT, "sts=0x%x, %s, vnow=%d\n",
		stat, str_power_path[((stat>>1)&0x03)], wlc_vnow);

	return rc;
}

irqreturn_t override_aicl_fail_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	wa_resuming_suspended_usbin_trigger(chg);
	return default_irq_handler(irq, data);
}

irqreturn_t override_typec_vconn_oc_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	wa_retry_vconn_enable_on_vconn_oc_trigger(chg);

	return default_irq_handler(irq, data);
}

irqreturn_t override_dcin_uv_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval pval = { 0, };
	int wlc_vnow = !power_supply_get_property(chg->dc_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval) ? pval.intval : -1;
	irqreturn_t rc = IRQ_NONE;

	if (ext_chg->concurrency_otg_wlc) {
		if (!chg->wls_psy) {
			chg->wls_psy = power_supply_get_by_name("wireless");
			if (!chg->wls_psy) {
				smblib_err(chg, "'wls_psy' is no device\n");
				return rc;
			}
		}

		if (wlc_vnow < 1000000)
			wlc_vnow /= 10;
		else
			wlc_vnow /= 1000;

		if (wlc_vnow < DCIN_THRESHOLD_MV)
			vote(chg->dc_suspend_votable, WLC_OTG_VOTER, true, 0);
	}

	wa_dcin_rerun_aicl_clear(chg);
	rc = dcin_uv_irq_handler(irq, data);
	wa_dcin_rerun_aicl_trigger(chg);

	if (ext_chg->concurrency_otg_wlc) {
		if (chg->dcin_uv_count >= 3) {
			rc = power_supply_get_property(chg->wls_psy, POWER_SUPPLY_PROP_DC_RESET, &pval);
			if (!rc) { 
				if (pval.intval & AC_MISSING_DETECTION) {
					if (chg->dc_reset_votable) {
						vote(chg->dc_reset_votable, DC_RESET_BY_UV, true, 0);
						vote(chg->dc_reset_votable, DC_RESET_BY_UV, false, 0);
					}
				}
			}
		}
	}

	return rc;
}

irqreturn_t override_dc_plugin_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	union power_supply_propval pval = { 0, };
	int rc, input_present;
	bool dcin_present, vbus_present;

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return IRQ_HANDLED;

	dcin_present = input_present & INPUT_PRESENT_DC;
	vbus_present = input_present & INPUT_PRESENT_USB;

	if (!chg->cp_psy)
		chg->cp_psy = power_supply_get_by_name("charge_pump_master");

	if (chg->cp_psy) {
		pval.intval = dcin_present ? POWER_SUPPLY_PL_USBMID_USBMID : POWER_SUPPLY_PL_USBIN_USBIN;
		if (power_supply_set_property(chg->cp_psy, POWER_SUPPLY_PROP_PARALLEL_MODE, &pval)) {
			smblib_err(chg, "Error to set POWER_SUPPLY_PROP_PARALLEL_MODE : %d\n", pval.intval);
		}
		if (power_supply_get_property(chg->cp_psy, POWER_SUPPLY_PROP_PARALLEL_MODE, &pval)) {
			smblib_err(chg, "Error to get POWER_SUPPLY_PROP_PARALLEL_MODE : %d\n", pval.intval);
		}
	}

	if (!dcin_present) {
		vote(chg->fcc_votable, WLS_PL_CHARGING_VOTER, false, 0);
		vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER, false, 0);

		chg->dcin_aicl_done = false;
	} else {
		/* Reset DCIN ICL to 500mA */
		mutex_lock(&chg->dcin_aicl_lock);
		rc = smblib_set_charge_param(chg, &chg->param.dc_icl, 500000);
		mutex_unlock(&chg->dcin_aicl_lock);
		if (rc < 0)
			return IRQ_HANDLED;

#ifdef CONFIG_CHARGER_IDTP9222_V2
		vote(chg->fcc_votable, WLS_PL_CHARGING_VOTER, true, 3600000);
#else
		vote(chg->fcc_votable, WLS_PL_CHARGING_VOTER, true, 2400000);
#endif
		if (chg->sec_cp_present) {
			/* Not used - Wireless cp charging */
			rc = smblib_select_sec_charger(chg,
				POWER_SUPPLY_CHARGER_SEC_NONE,
				POWER_SUPPLY_CP_NONE, false);
		}
		schedule_work(&chg->dcin_aicl_work);
	}

	/*
	 * Vote for 1500mA FCC upon WLS detach and remove vote upon attach if
	 * FCC stepper is enabled.
	 */
	if (chg->fcc_stepper_enable && !(vbus_present && !ext_chg->concurrency_otg_wlc))
		vote(chg->fcc_votable, FCC_STEPPER_VOTER, !dcin_present,
				dcin_present ? 0 : 1500000);

	if (chg->dc_psy)
		power_supply_changed(chg->dc_psy);

	smblib_dbg(chg, PR_INTERRUPT, "dcin_present= %d, usbin_present= %d, input_mode=%d\n",
			dcin_present, vbus_present, pval.intval);

	return IRQ_HANDLED;
}

irqreturn_t override_dcin_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	if (!strcmp(irq_data->name, "dcin-vashdn"))
		wa_recovery_vashdn_wireless_trigger(chg);

	if (!strcmp(irq_data->name, "dcin-pon"))
		wa_avoid_fod_status_trigger(chg);

	if (!strcmp(irq_data->name, "dcin-en"))
		wa_clear_dc_reverse_volt_v2_en_trigger(chg);

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	return IRQ_HANDLED;
}

void extension_icl_change(struct smb_charger *chg, int settled_ua)
{
	struct ext_smb_charger *ext_chg = chg->ext_chg;
	struct power_supply* veneer = power_supply_get_by_name("veneer");
	union power_supply_propval aicl = { .intval = settled_ua, };

	ext_chg->settled_ua = settled_ua;
	if (veneer) {
		power_supply_set_property(veneer, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED, &aicl);
		power_supply_put(veneer);
	}
}

void extension_typec_src_removal(struct smb_charger *chg)
{
	typec_src_removal(chg);
}

int extension_get_apsd_result(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);

	return apsd_result->pst;
}

void extension_hvdcp_detect_try_enable(struct smb_charger *chg, bool enable)
{
	smblib_hvdcp_detect_try_enable(chg, enable);
}

static void usb_plugin_work(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						usb_plugin_work);
	struct smb_charger *chg = ext_chg->chg;
	union power_supply_propval val = { 0, };
	bool vbus;

	vbus = !smblib_get_prop_usb_present(chg, &val) ? !!val.intval : false;
	raw_notifier_call_chain(&ext_chg->usb_plugin_notifier,
			vbus, chg);
	if (!vbus)
		ext_chg->settled_ua = 0;
}

static void usb_source_change_work(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						usb_source_change_work);
	struct smb_charger *chg = ext_chg->chg;

	raw_notifier_call_chain(&ext_chg->usb_source_change_notifier,
			0, chg);

}

static void typec_state_change_work(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						typec_state_change_work);
	struct smb_charger *chg = ext_chg->chg;

	raw_notifier_call_chain(&ext_chg->typec_state_change_notifier,
			0, chg);
}

static void psy_change_work(struct work_struct *work)
{
	struct ext_smb_charger *ext_chg = container_of(work, struct ext_smb_charger,
						psy_change_work);
	struct smb_charger *chg = ext_chg->chg;

	raw_notifier_call_chain(&ext_chg->psy_change_notifier,
			0, chg);
}

static int psy_change_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct ext_smb_charger *ext_chg = container_of(nb, struct ext_smb_charger,
			psy_change_nb);

	schedule_work(&ext_chg->psy_change_work);

	return NOTIFY_OK;
}

void extension_smb5_determine_initial_status(struct smb_charger *chg, void *data)
{
	struct ext_smb_charger *ext_chg;
	int rc = 0;

	ext_chg = devm_kzalloc(chg->dev, sizeof(*ext_chg), GFP_KERNEL);
	if (!ext_chg) {
		pr_err("Couldn't alloc ext_smb_charger rc=%d\n", ext_chg);
		return;
	}
	chg->ext_chg = ext_chg;
	ext_chg->chg = chg;

	INIT_WORK(&ext_chg->usb_plugin_work, usb_plugin_work);
	INIT_WORK(&ext_chg->usb_source_change_work, usb_source_change_work);
	INIT_WORK(&ext_chg->typec_state_change_work, typec_state_change_work);
	INIT_WORK(&ext_chg->psy_change_work, psy_change_work);
	ext_chg->psy_change_nb.notifier_call = psy_change_call;
	rc = power_supply_reg_notifier(&ext_chg->psy_change_nb);
	if (rc < 0)
		pr_err("Failed register psy notifier rc = %d\n", rc);

	RAW_INIT_NOTIFIER_HEAD(&ext_chg->usb_plugin_notifier);
	RAW_INIT_NOTIFIER_HEAD(&ext_chg->usb_source_change_notifier);
	RAW_INIT_NOTIFIER_HEAD(&ext_chg->typec_state_change_notifier);
	RAW_INIT_NOTIFIER_HEAD(&ext_chg->psy_change_notifier);

	wa_helper_init(chg);

	override_usb_plugin_irq_handler(0, data);
	override_chg_state_change_irq_handler(0, data);
	override_usb_source_change_irq_handler(0, data);
	override_dc_plugin_irq_handler(0, data);

	typec_attach_detach_irq_handler(0, data);
	typec_state_change_irq_handler(0, data);
	icl_change_irq_handler(0, data);
	batt_temp_changed_irq_handler(0, data);
	wdog_bark_irq_handler(0, data);
	typec_or_rid_detection_change_irq_handler(0, data);
	wdog_snarl_irq_handler(0, data);
}
