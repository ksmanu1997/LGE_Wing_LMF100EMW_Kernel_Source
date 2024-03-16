/*
 * CAUTION! :
 * 	This file will be included at the end of "qpnp-fg-gen3.c".
 * 	So "qpnp-fg-gen3.c" should be touched before you start to build.
 * 	If not, your work will not be applied to the built image
 * 	because the build system doesn't care the update time of this file.
 */

#include <linux/thermal.h>
#include <linux/kernel.h>
#include <soc/qcom/lge/board_lge.h>
#include "veneer-primitives.h"

#define LGE_QG_INITVAL -1

struct _fake {
	int temperature;
	int capacity;
	int uvoltage;
};

struct _qginfo {
/* Capacity */
	int capacity_rescaled;
	int capacity_chargecnt;
	int capacity_learned;
/* v/i ADCs */
	int battery_inow;
	int battery_vnow;
	int battery_ocv;
	int input_vusb;
	int input_iusb;
	int input_aicl;
/* Temperature */
	int temp_compensated;
	int temp_thermister;
	int temp_vts;
/* SoCs */
	int monotonic_soc;
	int sys_soc;
	int cc_soc;
/* Impedance */
	int impedance_esr;
/* Misc */
	int misc_cycle;
};

#define TCOMP_TABLE_MAX 3
#define TCOMP_COUNT 25
struct tcomp_param {
	bool load_done;
	int load_max;
	bool icoeff_load_done;
	struct tcomp_entry {
		int temp_cell;
		int temp_bias;
	} table[TCOMP_TABLE_MAX][TCOMP_COUNT];
	int icoeff;

	bool rise_cut;
	int rise_cut_trig;
	bool fall_cut;
	int fall_cut_trig;

	bool qnovo_charging;
	bool logging;
	unsigned long last_temp_update_time;
	int last_cell_temp;
	bool smooth_temp_jump;
	int comp_swap_temp;
};

struct _rescale {
	bool lge_monotonic;
	int	criteria;	// 0 ~ 255
	int	rawsoc;		// 0 ~ 255
	int	result;		// 0 ~ 100
};

enum tcomp_chg_type {
	TCOMP_CHG_NONE = 0,
	TCOMP_CHG_USB,
	TCOMP_CHG_WLC_LCDOFF,
	TCOMP_CHG_WLC_LCDON
};

/* Gloval variable for extension-qpnp-qg */
static struct _fake fake = {
	.temperature = LGE_QG_INITVAL,
	.capacity = LGE_QG_INITVAL,
	.uvoltage = LGE_QG_INITVAL,
};

static struct _qginfo qginfo = {
/* Capacity  */ LGE_QG_INITVAL, LGE_QG_INITVAL, LGE_QG_INITVAL,
/* v/i ADCs  */ LGE_QG_INITVAL, LGE_QG_INITVAL, LGE_QG_INITVAL,
                LGE_QG_INITVAL, LGE_QG_INITVAL, LGE_QG_INITVAL,
/* Temp      */ LGE_QG_INITVAL, LGE_QG_INITVAL, -1000,
/* SoCs      */ LGE_QG_INITVAL, LGE_QG_INITVAL, LGE_QG_INITVAL,
/* impedance */ LGE_QG_INITVAL,
/* Misc      */ LGE_QG_INITVAL
};

static struct tcomp_param tcomp = {
	.load_done = false,
	.icoeff_load_done = false,
	.icoeff = 0,

	.rise_cut = false,
	.rise_cut_trig = -999,
	.fall_cut = false,
	.fall_cut_trig = -999,

	.qnovo_charging = false,
	.logging = false,
	.last_temp_update_time = 0,
	.last_cell_temp = 0,
	.smooth_temp_jump = false,
	.comp_swap_temp = 0,
};

/* For SoC rescaling, .rawsoc(0~255) is updated ONLY ON
 * 'fg_delta_msoc_irq_handler' and it is rescaled to 0~100
 */
static struct _rescale rescale = {
	.lge_monotonic = false,
	.criteria = 247,
	.rawsoc = LGE_QG_INITVAL,
	.result = LGE_QG_INITVAL,
};

/* Extension A. Battery temp tuning */
/* calculate_battery_temperature
 *     bias  : 1st compensation by predefined diffs
 *     icomp : 2nd compensation by (i^2 * k)
 */

static int get_charging_type(struct qpnp_qg *qg)
{
	union power_supply_propval val = { 0, };
#ifdef CONFIG_LGE_PM_VENEER_PSY
	char slimit[20] = "";
#endif

	if (qg->batt_psy) {
		if (!power_supply_get_property(qg->batt_psy,
			POWER_SUPPLY_PROP_STATUS, &val))
			if (val.intval != POWER_SUPPLY_STATUS_CHARGING)
				return TCOMP_CHG_NONE;
	}

	if (qg->usb_psy) {
		if (!power_supply_get_property(qg->usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &val))
			if (val.intval)
				return TCOMP_CHG_USB;
	}

	if (qg->dc_psy) {
		if (!power_supply_get_property(qg->dc_psy,
			POWER_SUPPLY_PROP_PRESENT, &val)) {
			if (val.intval) {
#ifdef CONFIG_LGE_PM_VENEER_PSY
				if (unified_nodes_show("status_lcd", slimit)) {
					if (slimit[0] == '0')
						return TCOMP_CHG_WLC_LCDOFF;
					return TCOMP_CHG_WLC_LCDON;
				} else
#endif
					return TCOMP_CHG_WLC_LCDOFF;
			}
		}
	}

	return TCOMP_CHG_NONE;
}

static int get_batt_charging_current(struct qpnp_qg *qg)
{
	static int ichg = 0;
	bool is_cc = false;
	bool is_fast_charging = false;
	union power_supply_propval val = { 0, };

	if (!qg || !qg->batt_psy || !qg->usb_psy)
		return 0;

	if (!power_supply_get_property(qg->batt_psy,
			POWER_SUPPLY_PROP_STATUS, &val)
				&& val.intval == POWER_SUPPLY_STATUS_CHARGING) {
		if (!power_supply_get_property(qg->usb_psy,
				POWER_SUPPLY_PROP_REAL_TYPE, &val)) {

			if (val.intval == POWER_SUPPLY_TYPE_USB_HVDCP ||
				val.intval == POWER_SUPPLY_TYPE_USB_HVDCP_3 ||
				val.intval == POWER_SUPPLY_TYPE_USB_PD)
				is_fast_charging = true;

			if (!power_supply_get_property(qg->batt_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &val) &&
				(val.intval == POWER_SUPPLY_CHARGE_TYPE_TRICKLE ||
				 val.intval == POWER_SUPPLY_CHARGE_TYPE_FAST))
				is_cc = true;

			/*  in case of fast charging, fcc is refered instead of
				real current for avoiding qni negative pulse effect */
			if (is_fast_charging && is_cc ) {
				ichg = !power_supply_get_property(qg->batt_psy,
							POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val) ?
								val.intval / -1000 : ichg;
				goto out;
			} else {
				/* if charging current is over -25mA,
					batt_therm compensation current keeps the previous value */
				if (!power_supply_get_property(qg->batt_psy,
						POWER_SUPPLY_PROP_CURRENT_NOW, &val) &&
							val.intval < -25000)
					ichg = val.intval / 1000;

				goto out;
			}
		}
	}

	ichg = 0;

out:
	return ichg;
}

static int filtered_batt_therm(bool changed, int comp, int batt)
{
	bool tbl_changed = changed;
	int battemp_cell = batt;
	int battemp_comp = comp;
	static int pre_battemp_cell = -9999;
	static int pre_battemp_comp = -9999;
	static bool is_filtering_rise = false;
	static bool is_filtering_fall = false;
	int battemp_cell_diff = 0;
	int battemp_comp_diff = 0;

	if (!((tbl_changed && tcomp.rise_cut
			&& (battemp_comp > tcomp.rise_cut_trig)) ||
		(tbl_changed && tcomp.fall_cut
			&& (battemp_comp < tcomp.fall_cut_trig))))
		tbl_changed = false;

	if ((tbl_changed || is_filtering_rise || is_filtering_fall)
		&& (pre_battemp_cell > -9999 && pre_battemp_comp > -9999)) {
		battemp_cell_diff = battemp_cell - pre_battemp_cell;
		battemp_comp_diff = battemp_comp - pre_battemp_comp;
		// rise
		if (tcomp.rise_cut && (battemp_comp >= pre_battemp_comp)) {
			if (is_filtering_fall)
				is_filtering_fall = false;

			if (battemp_comp_diff > battemp_cell_diff) {
				is_filtering_rise = true;
				if ( battemp_cell_diff > 0 )
					battemp_comp = pre_battemp_comp + battemp_cell_diff;
				else
					battemp_comp = pre_battemp_comp;
			}
			else {
				is_filtering_rise = false;
			}
		}
		// fall
		else if (tcomp.fall_cut) {
			if (is_filtering_rise)
				is_filtering_rise = false;

			if (battemp_cell_diff > battemp_comp_diff ) {
				is_filtering_fall = true;
				if (battemp_cell_diff < 0)
					battemp_comp = pre_battemp_comp + battemp_cell_diff;
				else
					battemp_comp = pre_battemp_comp;
			}
			else {
				is_filtering_fall = false;
			}
		}
		else if (tcomp.rise_cut) {
			if (is_filtering_rise)
				is_filtering_rise = false;
		}
	}

	pre_battemp_cell = battemp_cell;
	pre_battemp_comp = battemp_comp;
	return battemp_comp;
}

#define TEMP_UPDATE_PERIOD 30 // Unit: seconds, ex. 30 = 30 seconds update period
static bool check_battery_temp_update_time(void){
	bool update_now = false;
	unsigned long current_time = 0;
	unsigned long next_update_time = 0;

	get_rtc_time(&current_time);

	if (tcomp.logging)
		pr_err("time now= %d, last time = %d\n",
			(int)current_time, (int)tcomp.last_temp_update_time);

	if (tcomp.last_temp_update_time == 0){
		tcomp.last_temp_update_time = current_time;
		update_now = true;
		return update_now;
	}

	next_update_time = tcomp.last_temp_update_time + TEMP_UPDATE_PERIOD;

	if ( next_update_time > current_time)
		update_now = false;
	else {
		update_now = true;
		tcomp.last_temp_update_time = current_time;
	}

	return update_now;
}

#define BATTERY_TEMP_COMPENSATION_THRESHOLD_CURRENT 1000000 //unit: uA, 1000 mA
#define BATTERY_TEMP_COMPENSATION_THRESHOLD_TEMP 200
#define MAX_JUMP_THREHSOLD_DEGREE 10 // If delta is greather than 2 degree, smooth jump work.
static int calculate_battery_temperature(/* @Nonnull */ struct qpnp_qg *qg)
{
	int battemp_bias, battemp_icomp = 0, battemp_cell = 0;
	int i, temp, ichg_comp = 0, tbl_pt = 0;
	int temp_comp = 0;
	union power_supply_propval val = { 0, };
	bool tbl_changed = false;
	bool real_temp_updated = false;
	static int pre_tbl_pt = -1;
	int temp_delta = 0;

	if(check_battery_temp_update_time()){
		if (qg_get_battery_temp(qg, &battemp_cell) < 0){
			pr_info("get real batt therm error\n");
			return LGE_QG_INITVAL;
		}
		pr_info("update real batt temp now. temp = %d\n", battemp_cell);
		tcomp.last_cell_temp = battemp_cell;
		real_temp_updated = true;
	}else{
		//Use last update battery temp.
		if (tcomp.logging)
			pr_info("last batt therm use. last_temp = %d\n",
				tcomp.last_cell_temp);
		battemp_cell = tcomp.last_cell_temp;
		real_temp_updated = false;
	}

//	if (wa_skip_batt_temp_on_bootup_check(battemp_cell, false))
//		return 250;

	if (!tcomp.load_done) {
		pr_info("not ready tcomp table. rerun -> table=%d\n",
			tcomp.load_done);
		return battemp_cell;
	}

	if (!tcomp.icoeff_load_done) {
		pr_info("not ready icoeff. rerun -> icoeff=%d\n",
			tcomp.icoeff_load_done);
		return battemp_cell;
	}

	if (tcomp.load_max > 1) {
		switch (get_charging_type(qg)) {
			case TCOMP_CHG_WLC_LCDOFF: tbl_pt = 1; break;
			case TCOMP_CHG_WLC_LCDON:  tbl_pt = 2; break;
			default: tbl_pt = 0; break;
		}
		if (pre_tbl_pt >= 0 )
			if (pre_tbl_pt != tbl_pt)
				tbl_changed = true;
		pre_tbl_pt = tbl_pt;
	}
	else
		tbl_pt = 0;


	/* Compensating battemp_bias */
	for (i = 0; i < TCOMP_COUNT; i++) {
		if (battemp_cell < tcomp.table[tbl_pt][i].temp_cell)
			break;
	}

	if (i == 0)
		battemp_bias = tcomp.table[tbl_pt][0].temp_bias;
	else if (i == TCOMP_COUNT)
		battemp_bias = tcomp.table[tbl_pt][TCOMP_COUNT-1].temp_bias;
	else
		battemp_bias =
		(	(tcomp.table[tbl_pt][i].temp_bias -
				tcomp.table[tbl_pt][i-1].temp_bias)
			* (battemp_cell - tcomp.table[tbl_pt][i-1].temp_cell)
			/ (tcomp.table[tbl_pt][i].temp_cell -
				tcomp.table[tbl_pt][i-1].temp_cell)
		) + tcomp.table[tbl_pt][i-1].temp_bias;

	/* Compensating battemp_icomp */
	if (qg->batt_psy) {
		if (tcomp.qnovo_charging) {
			ichg_comp = get_batt_charging_current(qg);
		}
		else {
			if (!power_supply_get_property(
				qg->batt_psy, POWER_SUPPLY_PROP_STATUS, &val)
				&& val.intval == POWER_SUPPLY_STATUS_CHARGING
				&& !power_supply_get_property(
					qg->batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val)
				&& val.intval > BATTERY_TEMP_COMPENSATION_THRESHOLD_CURRENT)
				ichg_comp = ((val.intval) / 1000);
		}
	} else {
		pr_info("Battery is not available, %d(=%d+%d) as batt temp\n",
			battemp_cell + battemp_bias, battemp_cell, battemp_bias);
	}

	if(battemp_cell >= BATTERY_TEMP_COMPENSATION_THRESHOLD_TEMP) {
		battemp_icomp = ichg_comp * ichg_comp * tcomp.icoeff;
		battemp_icomp = battemp_icomp / 10000000;
		temp_comp = battemp_cell + battemp_bias - battemp_icomp;
	} else {
		temp_comp = battemp_cell + battemp_bias;
	}

	if (tcomp.smooth_temp_jump) {
		//Trigger Jump temp smooth work. delta over 2 degree.
		if (!tcomp.comp_swap_temp && (qginfo.temp_compensated > 0) &&
			(abs(temp_comp - qginfo.temp_compensated) > MAX_JUMP_THREHSOLD_DEGREE +10)){
				tcomp.comp_swap_temp = qginfo.temp_compensated;
		}

		if (real_temp_updated){
			if (tcomp.comp_swap_temp > 0){
				temp_delta = tcomp.comp_swap_temp - temp_comp;
				if (abs(temp_delta) > MAX_JUMP_THREHSOLD_DEGREE){
					if (tcomp.comp_swap_temp > temp_comp)
						tcomp.comp_swap_temp = tcomp.comp_swap_temp - 10;
					else if (tcomp.comp_swap_temp < temp_comp)
						tcomp.comp_swap_temp = tcomp.comp_swap_temp + 10;
					temp = tcomp.comp_swap_temp;
				} else {
					tcomp.comp_swap_temp = 0;
					temp = temp_comp;
				}
			} else {
				temp = temp_comp;
			}
		} else if (tcomp.comp_swap_temp > 0) {
			temp = tcomp.comp_swap_temp;
		} else
			temp = temp_comp;

		if (tcomp.logging)
			pr_info("Temp Smooth work: tcomp.comp_swap_temp = %d battemp_icomp = %d"
				" temp_comp = %d, final_temp = %d\n",
				tcomp.comp_swap_temp, battemp_icomp, temp_comp, temp);
	} else
		temp = temp_comp;

	if (tcomp.logging)
		pr_info("Battery temperature : "
				"%d = (%d)(cell) + (%d)(bias) - %d(icomp), "
				"icoeff = %d, ichg_comp = %d \n",
			temp, battemp_cell, battemp_bias, battemp_icomp,
			tcomp.icoeff, ichg_comp);

	if (tcomp.rise_cut || tcomp.fall_cut)
		return filtered_batt_therm(tbl_changed, temp, battemp_cell);

	//Sync Temp.
	if (tcomp.smooth_temp_jump)
		qginfo.temp_compensated = temp;

	return temp;
}

/* Extension B. Battery UI SOC tuning */

#define LGE_QG_CHARGING         1
#define LGE_QG_DISCHARGING      0
static int lge_is_qg_charging(struct qpnp_qg *qg)
{
	union power_supply_propval val = { 0, };
	bool power_status = false;

	if (!qg || !qg->batt_psy || !qg->usb_psy)
		return -1;

	if (!power_supply_get_property(qg->batt_psy,
					POWER_SUPPLY_PROP_STATUS_RAW, &val))
		power_status = (val.intval == POWER_SUPPLY_STATUS_CHARGING) ? true : false;
	else
		return -1;

	if (!power_status)
		return LGE_QG_DISCHARGING;

	if (!power_supply_get_property(qg->batt_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW, &val)) {
		if (val.intval < -25000 )
			return LGE_QG_CHARGING;
		else
			return LGE_QG_DISCHARGING;
	}

	return -1;
}

#ifdef CONFIG_LGE_PM_CCD_TTF_LOG
static void ttf_logging(struct qpnp_qg *qg, int cttf)
{
	struct power_supply*    battery_psy = power_supply_get_by_name("battery");
	struct power_supply*    usb_psy = power_supply_get_by_name("usb");
	union power_supply_propval      val = { .intval = -1000 };
	int chgtype = POWER_SUPPLY_CHARGE_TYPE_NONE;
	int iusb = 0;
	int ibat = 0;
	int buf = 0;
	static int prev_ibat = 0;

	if (battery_psy && !power_supply_get_property(battery_psy, POWER_SUPPLY_PROP_CHARGE_TYPE, &val))
		chgtype = val.intval;

	if (chgtype == POWER_SUPPLY_CHARGE_TYPE_NONE || (cttf < 0 || cttf > 1000)) {
		ibat = 0;
		iusb = 0;
		prev_ibat = ibat;
		return;
	}

	if (chgtype == POWER_SUPPLY_CHARGE_TYPE_FAST || chgtype == POWER_SUPPLY_CHARGE_TYPE_TAPER) {
		if (usb_psy && !power_supply_get_property(usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &val))
			iusb = val.intval/1000;
		if (battery_psy && !power_supply_get_property(battery_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val))
			ibat = val.intval/1000;

		if (chgtype == POWER_SUPPLY_CHARGE_TYPE_FAST) {
			if (prev_ibat != ibat && iusb != 0)
				pr_info("TTFINFO: [CC] cTTF:%d, iFCC:%d\n", cttf, ibat);
		}

		if (chgtype == POWER_SUPPLY_CHARGE_TYPE_TAPER) {
			qg_get_battery_current(qg, &buf);
			pr_info("TTFINFO: [CV] cTTF:%d, iBAT:%d\n", cttf, abs(buf)/1000);
		}
	}

	if (battery_psy)
		power_supply_put(battery_psy);
	if (usb_psy)
		power_supply_put(usb_psy);
	prev_ibat = ibat;
}
#endif
#define FULL_CAPACITY	100
int lge_get_ui_soc(struct qpnp_qg *qg, int msoc_raw)
{
	int new_result = min(FULL_CAPACITY,
		DIV_ROUND_CLOSEST(msoc_raw/10 * FULL_CAPACITY, rescale.criteria));
#ifdef CONFIG_LGE_PM_CCD
	char buf[10] = {0, };
	static int prev_msoc_raw;
#endif

	rescale.rawsoc = msoc_raw;

#ifdef CONFIG_LGE_PM_CCD
	if (!qg->battery_missing && qg->profile_loaded && qg->soc_reporting_ready) {
		if (prev_msoc_raw != msoc_raw) {
			snprintf(buf, sizeof(buf), "%d", msoc_raw);
			unified_nodes_store("ttf_capacity", buf, strlen(buf));
#ifdef CONFIG_LGE_PM_CCD_TTF_LOG
			ttf_logging(qg, msoc_raw);
#endif
		}
		prev_msoc_raw = msoc_raw;
	}
#endif
	if (!rescale.lge_monotonic) {
		rescale.result = new_result;
		return 0;
	}

	if (rescale.result <= 0 ||
		max(rescale.result, new_result) - min(rescale.result, new_result) > 5 )
		rescale.result = new_result;

	switch (lge_is_qg_charging(qg)) {
		case LGE_QG_CHARGING:
			pr_info("qg_rescale: charging: %d = max(old=%d, new=%d)\n",
				max(rescale.result, new_result), rescale.result, new_result);
			rescale.result = max(rescale.result, new_result);
			break;
		case LGE_QG_DISCHARGING:
			pr_info("fg_rescale: discharging: %d = min(old=%d, new=%d)\n",
				min(rescale.result, new_result), rescale.result, new_result);
			rescale.result = min(rescale.result, new_result);
			break;
		default:
			pr_info("qg_rescale: error: old=%d, new=%d\n", rescale.result, new_result);
			rescale.result = new_result;
			break;
	}

	return 0;
}

/* Extension C. Battery information update & logging*/
static void qginfo_snapshot_print(void)
{
	printk("PMINFO: [CSV] "
/* Capacity  */ "cSYS:%d, cCHG:%d, cLRN:%d, "
/* v/i ADCs  */ "iBAT:%d, vBAT:%d, vOCV:%d, vUSB:%d, iUSB:%d, AiCL:%d\n"
				"PMINFO: [CSV] "
/* Temp      */ "tSYS:%d, tORI:%d, tVTS:%d, "
/* SoCs      */ "cMSOC:%d, cSSOC:%d, cCCSOC:%d, "
/* Impedance */ "rESR:%d, "
/* Misc      */ "CYCLE:%d \n",

/* Capacity  */ qginfo.capacity_rescaled, qginfo.capacity_chargecnt, qginfo.capacity_learned,
/* Battery   */ qginfo.battery_inow, qginfo.battery_vnow, qginfo.battery_ocv,
/* Input     */ qginfo.input_vusb, qginfo.input_iusb, qginfo.input_aicl,
/* Temp      */ qginfo.temp_compensated, qginfo.temp_thermister, qginfo.temp_vts,
/* SoCs      */ qginfo.monotonic_soc, qginfo.sys_soc, qginfo.cc_soc,
/* Impedance */ qginfo.impedance_esr,
/* Misc      */ qginfo.misc_cycle);
}

static void
qginfo_snapshot_inputnow(int* vusb, int* iusb, int* aicl)
{
	struct power_supply* psy_main = power_supply_get_by_name("main");
	struct power_supply* psy_usb = power_supply_get_by_name("usb");

        union power_supply_propval val = { 0, };

	*aicl = (psy_main && !power_supply_get_property(
			psy_main, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED, &val))
				? val.intval/1000 : LGE_QG_INITVAL;
	*vusb = (psy_usb && !power_supply_get_property(
			psy_usb, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val))
				? val.intval/1000 : LGE_QG_INITVAL;
	*iusb = (psy_usb && !power_supply_get_property(
			psy_usb, POWER_SUPPLY_PROP_INPUT_CURRENT_NOW, &val))
				? val.intval/1000 : LGE_QG_INITVAL;

	if (psy_main)
		power_supply_put(psy_main);
	if (psy_usb)
		power_supply_put(psy_usb);
}


#define ABNORMAL_VBAT_AT_SOC_100 3850
#define ABNORMAL_VBAT_AT_SOC_50 4400

static bool check_abnormal_case(void){

	bool is_abnormal = false;

	if (qginfo.capacity_rescaled >= 100
			&& qginfo.battery_vnow <= ABNORMAL_VBAT_AT_SOC_100)
		is_abnormal = true;
	else if (qginfo.capacity_rescaled < 50
			&& qginfo.battery_vnow >= ABNORMAL_VBAT_AT_SOC_50)
		is_abnormal =true;
	else
		is_abnormal = false;

	if (is_abnormal) {
		if (unified_bootmode_fabproc())
			is_abnormal = false;
	}
	return is_abnormal;
}

#define QGAUGE_DUMP_INDEX 10

static void wa_get_qgauge_pmic_dump_func(struct qpnp_qg *qg) {
	union power_supply_propval debug  = {QGAUGE_DUMP_INDEX, };

	power_supply_set_property(qg->batt_psy,
			POWER_SUPPLY_PROP_DEBUG_BATTERY, &debug);
}


static void qginfo_snapshot_update(struct power_supply *psy)
{
	int buf = 0;
#if 0
	int64_t temp64 = 0;
#endif
	struct qpnp_qg*	qg = power_supply_get_drvdata(psy);
	struct thermal_zone_device*	tzd = thermal_zone_get_zone_by_name("vts-virt-therm");
//	struct qpnp_qg *chip = container_of(qg, struct qpnp_qg, qg_device);
	union power_supply_propval val = { 0, };

	if (!tzd || !qg)
		return;

/* Capacity */
	qginfo.capacity_rescaled = rescale.result < 0
		? LGE_QG_INITVAL : rescale.result;
	qginfo.capacity_chargecnt = !power_supply_get_property(qg->qg_psy,
                                    POWER_SUPPLY_PROP_CHARGE_COUNTER, &val)
		? val.intval/1000 : LGE_QG_INITVAL;
	qginfo.capacity_learned = !power_supply_get_property(qg->qg_psy,
                                    POWER_SUPPLY_PROP_CHARGE_FULL, &val)
		? val.intval/1000 : LGE_QG_INITVAL;
/* Battery */
	qginfo.battery_inow = !qg_get_battery_current(qg, &buf)
		? buf / 1000 : LGE_QG_INITVAL;
	qginfo.battery_vnow = !qg_get_battery_voltage(qg, &buf)
		? buf / 1000 : LGE_QG_INITVAL;
	qginfo.battery_ocv = !power_supply_get_property(qg->qg_psy,
                                    POWER_SUPPLY_PROP_VOLTAGE_OCV, &val)
		? val.intval / 1000 : LGE_QG_INITVAL;
/* Input */
	qginfo_snapshot_inputnow(&qginfo.input_vusb, &qginfo.input_iusb, &qginfo.input_aicl);
/* Temperature */
	qginfo.temp_compensated
		= calculate_battery_temperature(qg);
	qginfo.temp_thermister = !qg_get_battery_temp(qg, &buf)
		? buf : LGE_QG_INITVAL;
	qginfo.temp_vts = !thermal_zone_get_temp(tzd, &buf)
		? buf / 100 : -1000;
/* SoCs */
	qginfo.monotonic_soc =  !power_supply_get_property(qg->qg_psy,
                                    POWER_SUPPLY_PROP_REAL_CAPACITY, &val)
		? val.intval: LGE_QG_INITVAL;
	qginfo.sys_soc =  !power_supply_get_property(qg->qg_psy,
                                    POWER_SUPPLY_PROP_CAPACITY_RAW, &val)
		? val.intval: LGE_QG_INITVAL;
	qginfo.cc_soc =  !power_supply_get_property(qg->qg_psy,
                                    POWER_SUPPLY_PROP_CC_SOC, &val)
		? val.intval: LGE_QG_INITVAL;
/* Impedance */
	qginfo.impedance_esr = !power_supply_get_property(qg->qg_psy,
                                    POWER_SUPPLY_PROP_RESISTANCE_NOW, &val)
		? val.intval: LGE_QG_INITVAL;
/* Misc */
	qginfo.misc_cycle = !power_supply_get_property(qg->qg_psy,
                                    POWER_SUPPLY_PROP_CYCLE_COUNT, &val)
		? val.intval: LGE_QG_INITVAL;
#if 0
	qginfo.misc_batid
		= qg->batt_id_ohm / 1000;
#endif

	/* logging finally */
	qginfo_snapshot_print();

	if (check_abnormal_case())
		wa_get_qgauge_pmic_dump_func(qg);
}

///////////////////////////////////////////////////////////////////////////////
#define PROPERTY_CONSUMED_WITH_SUCCESS	0
#define PROPERTY_CONSUMED_WITH_FAIL	EINVAL
#define PROPERTY_BYPASS_REASON_NOENTRY	ENOENT
#define PROPERTY_BYPASS_REASON_ONEMORE	EAGAIN

static enum power_supply_property extension_bms_appended [] = {
	POWER_SUPPLY_PROP_UPDATE_NOW,
};

static int
extension_bms_get_property_pre(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val)
{
	int rc = PROPERTY_CONSUMED_WITH_SUCCESS;
	struct qpnp_qg*	qg = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_RESISTANCE:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
			rc = -PROPERTY_BYPASS_REASON_ONEMORE;
		break;

	case POWER_SUPPLY_PROP_CAPACITY :
		// Battery fake setting has top priority
		if (fake.capacity == LGE_QG_INITVAL)
			rc = -PROPERTY_BYPASS_REASON_ONEMORE;
		else
			val->intval = fake.capacity;
		break;

	case POWER_SUPPLY_PROP_TEMP :
		if (fake.temperature == LGE_QG_INITVAL) {
			val->intval = calculate_battery_temperature(qg); // Use compensated temperature
		} else
			val->intval = fake.temperature;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW :
		if (fake.uvoltage == LGE_QG_INITVAL)
			rc = -PROPERTY_BYPASS_REASON_ONEMORE;
		else
			val->intval = fake.uvoltage;
		break;

	case POWER_SUPPLY_PROP_UPDATE_NOW :
		/* Do nothing and just consume getting */
		if (qg != NULL) {
			val->intval = qg->in_esr_process;
		} else {
			val->intval = -1;
		}
		break;

	default:
		rc = -PROPERTY_BYPASS_REASON_NOENTRY;
		break;
	}

	return rc;
}

static int
extension_bms_get_property_post(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val, int rc)
{
        switch (prop) {
        default:
                break;
        }
	return rc;
}

static int
extension_bms_set_property_pre(struct power_supply *psy,
	enum power_supply_property prop, const union power_supply_propval *val)
{
	int* fakeset = NULL;
	int  rc = PROPERTY_CONSUMED_WITH_SUCCESS;
	struct qpnp_qg* qg = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_UPDATE_NOW :
		if (val->intval)
			qginfo_snapshot_update(psy);
		break;

	case POWER_SUPPLY_PROP_TEMP :
		fakeset = &fake.temperature;
		break;
	case POWER_SUPPLY_PROP_CAPACITY :
		fakeset = &fake.capacity;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW :
		fakeset = &fake.uvoltage;
		break;

	default:
		rc = -PROPERTY_BYPASS_REASON_NOENTRY;
	}

	if (fakeset && *fakeset != val->intval) {
		*fakeset = val->intval;
		power_supply_changed(qg->batt_psy);
	}

	return rc;
}

static int
extension_bms_set_property_post(struct power_supply *psy,
	enum power_supply_property prop, const union power_supply_propval *val, int rc)
{
	return rc;
}

///////////////////////////////////////////////////////////////////////////////
enum power_supply_property* extension_bms_properties(void)
{
	static enum power_supply_property
		extended_properties[ARRAY_SIZE(qg_psy_props)
		+ ARRAY_SIZE(extension_bms_appended)];
	int size_original = ARRAY_SIZE(qg_psy_props);
	int size_appended = ARRAY_SIZE(extension_bms_appended);

	memcpy(extended_properties, qg_psy_props,
		size_original * sizeof(enum power_supply_property));
	memcpy(&extended_properties[size_original], extension_bms_appended,
		size_appended * sizeof(enum power_supply_property));

	veneer_extension_pspoverlap(qg_psy_props, size_original,
		extension_bms_appended, size_appended);

        return extended_properties;
}

size_t extension_bms_num_properties(void)
{
	return ARRAY_SIZE(qg_psy_props) + ARRAY_SIZE(extension_bms_appended);
}

int
extension_bms_get_property(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val)
{
	int rc = extension_bms_get_property_pre(psy, prop, val);
	if (rc == -PROPERTY_BYPASS_REASON_NOENTRY
		|| rc == -PROPERTY_BYPASS_REASON_ONEMORE)
		rc = qg_psy_get_property(psy, prop, val);
	rc = extension_bms_get_property_post(psy, prop, val, rc);

	return rc;
}

int
extension_bms_set_property(struct power_supply *psy,
	enum power_supply_property prop, const union power_supply_propval *val)
{
	int rc = extension_bms_set_property_pre(psy, prop, val);
	if (rc == -PROPERTY_BYPASS_REASON_NOENTRY
		|| rc == -PROPERTY_BYPASS_REASON_ONEMORE)
		rc = qg_psy_set_property(psy, prop, val);
	rc = extension_bms_set_property_post(psy, prop, val, rc);

	return rc;
}

int
extension_bms_property_is_writeable(
	struct power_supply *psy, enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_UPDATE_NOW:
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifndef CONFIG_LGE_PM_CCD
	case POWER_SUPPLY_PROP_CAPACITY_RAW:
#endif
		rc = 1;
		break;
	default:
		rc = qg_property_is_writeable(psy, prop);
		break;
	}
	return rc;
}

struct device_node *
extension_get_batt_profile(struct device_node* container, int resistance_id)
{
	/* Search with resistance_id and
	 * Hold the result to an unified node(sysfs) for the fab process
	 */
	struct device_node* node;
	const char* name;
	char buffer [8] = { '\0', };
	int kohm = 0;
	struct device_node* profile =
			of_batterydata_get_best_profile(container, resistance_id, NULL);

	/* If no matching, set it as default */
	if (!profile) {
		if(HW_SKU_NA_CDMA_VZW == lge_get_sku_carrier())
			node = of_find_node_by_name(NULL, "lge-vzw-battery-supplement");
		else
			node = of_find_node_by_name(NULL, "lge-battery-supplement");

		name = of_property_read_string(node, "default-battery-name", &name)
				? NULL : name;
		kohm = of_property_read_u32(node, "default-battery-kohm", &kohm)
				? 0 : kohm;
		profile = of_batterydata_get_best_profile(container, kohm, name);
		pr_info("Getting default battery profile(%s): %s\n",
			name, profile ? "success" : "fail");
	}

	// At this time, 'battery_valid' may be true always for the embedded battery model
	snprintf(buffer, sizeof(buffer), "%d", !!profile);
	unified_nodes_store("battery_valid", buffer, sizeof(buffer));

	return profile;
}

int extension_qg_load_icoeff_dt(struct qpnp_qg *qg)
{
	struct device_node* tcomp_dtroot;
	struct device_node* tcomp_override;
	int dt_icomp = 0;

	if (tcomp.icoeff_load_done) {
		pr_info("icoeff had already been loaded.\n");
		return 0;
	}

	if (!qg->soc_reporting_ready) {
		pr_info("QG profile is not ready.\n");
		return LGE_QG_INITVAL;
	}

	if(HW_SKU_NA_CDMA_VZW == lge_get_sku_carrier())
		tcomp_dtroot = of_find_node_by_name(NULL, "lge-vzw-battery-supplement");
	else
		tcomp_dtroot = of_find_node_by_name(NULL, "lge-battery-supplement");

	if (!tcomp_dtroot) {
		pr_info("failed to find lge-battery-supplement\n");
		return LGE_QG_INITVAL;
	}

	if (qg->bp.batt_type_str) {
		tcomp_override = of_find_node_by_name(
				tcomp_dtroot, qg->bp.batt_type_str);
		if (tcomp_override &&
				of_property_read_u32(
					tcomp_override, "tempcomp-icoeff", &dt_icomp) >= 0)
			pr_info("ICOEFF is overridden to %d for %s\n", dt_icomp, qg->bp.batt_type_str);
	}

	if (!dt_icomp) {
		if (of_property_read_u32(tcomp_dtroot, "tempcomp-icoeff", &dt_icomp) >= 0) {
			pr_info("ICOEFF is set to %d by default\n", dt_icomp);
		} else {
			pr_info("ICOEFF isn't set. error.\n");
			return -1;
		}
	}

	tcomp.icoeff = dt_icomp;
	tcomp.icoeff_load_done = true;
	return 0;
}

int extension_qg_load_dt(void)
{
	const char str_tempcomp[TCOMP_TABLE_MAX][30] = {
		"tempcomp-offset",
		"tempcomp-offset-wlc-lcdoff",
		"tempcomp-offset-wlc-lcdon"
	};

	struct device_node* tcomp_dtroot;
	int dtarray_count = TCOMP_COUNT * 2;
	u32 dtarray_data [TCOMP_COUNT * 2];
	int i = 0, j = 0;

	if (tcomp.load_done) {
		pr_info("tcomp table had already been loaded.\n");
		return 0;
	}

	if(HW_SKU_NA_CDMA_VZW == lge_get_sku_carrier())
		tcomp_dtroot = of_find_node_by_name(NULL, "lge-vzw-battery-supplement");
	else
		tcomp_dtroot = of_find_node_by_name(NULL, "lge-battery-supplement");

	if (!tcomp_dtroot) {
		pr_info("failed to find lge-battery-supplement\n");
		return -1;
	}

	if (of_property_read_bool(tcomp_dtroot, "tempcomp-offset-wlc-enable"))
		tcomp.load_max = 3;
	else
		tcomp.load_max = 1;

	for (j = 0; j < tcomp.load_max; j++ ) {
		/* Finding tcomp_table and tcomp_icoeff */
		if (of_property_read_u32_array(tcomp_dtroot, str_tempcomp[j],
				dtarray_data, dtarray_count) >= 0) {
			for (i = 0; i < dtarray_count; i += 2) {
				tcomp.table[j][i/2].temp_cell = dtarray_data[i];
				tcomp.table[j][i/2].temp_bias = dtarray_data[i+1];
				pr_debug("Index = %02d : %4d - %4d\n",
					i/2,
					tcomp.table[j][i/2].temp_cell,
					tcomp.table[j][i/2].temp_bias);
			}
		} else {
			pr_info("%s is not found, error\n", str_tempcomp[j]);
			tcomp.table[j][0].temp_cell = INT_MAX;
			tcomp.table[j][i/2].temp_bias = 0;
			return -1;
		}
	}

	tcomp.smooth_temp_jump = of_property_read_bool(tcomp_dtroot,
		"tempcomp-smooth-temp-jump-enable");
	pr_info("smooth_temp_jump is %s\n", tcomp.smooth_temp_jump ? "enabled" : "disabled");

	tcomp.rise_cut = of_property_read_bool(tcomp_dtroot,
		"tempcomp-offset-wlc-rise-filter-enable");
	if (tcomp.rise_cut)
		of_property_read_u32(tcomp_dtroot,
			"tempcomp-offset-wlc-rise-filter-trigger", &tcomp.rise_cut_trig);
	tcomp.fall_cut = of_property_read_bool(tcomp_dtroot,
		"tempcomp-offset-wlc-fall-filter-enable");
	if (tcomp.fall_cut)
		of_property_read_u32(tcomp_dtroot,
			"tempcomp-offset-wlc-fall-filter-trigger", &tcomp.fall_cut_trig);

	of_property_read_u32(tcomp_dtroot,
		"capacity-raw-full", &rescale.criteria);
	rescale.lge_monotonic = of_property_read_bool(tcomp_dtroot,
		"lg-monotonic-soc-enable");
	tcomp.logging = of_property_read_bool(tcomp_dtroot,
		"tempcomp-logging");
	tcomp.qnovo_charging = of_property_read_bool(tcomp_dtroot,
		"tempcomp-qnovo-charging");

	if (j == tcomp.load_max) {
		tcomp.load_done = true;

		pr_info("[tempcomp config] table count: %s (%d/%d), "
			"rise cut filter: %s (trigger = %d degree), "
			"fall cut filter: %s (trigger = %d degree)\n",
			(j == tcomp.load_max) ? "done" : "error", j, tcomp.load_max,
			tcomp.rise_cut ? "enabled" : "disabled", tcomp.rise_cut_trig,
			tcomp.fall_cut ? "enabled" : "disabled", tcomp.fall_cut_trig);
	}

	return 0;
}

