#define pr_fmt(fmt) "BTP: %s: " fmt, __func__
#define pr_battemp(reason, fmt, ...)			\
do {							\
	if (pr_debugmask & (reason))			\
		pr_info(fmt, ##__VA_ARGS__);		\
	else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
} while (0)

static int pr_debugmask;

//#define DEBUG_BTP  //need to enable fake_battery when testing DEBUG_BTP

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#ifdef DEBUG_BTP
extern bool unified_nodes_show(const char* key, char* value);
extern bool unified_nodes_store(const char* key, const char* value, size_t size);
extern bool unified_bootmode_chargerlogo(void);
#endif

#include "veneer-primitives.h"

#define BATTEMP_NOTREADY	INT_MAX
#define BATTEMP_WAKELOCK	"lge-btp-scenario"

#define VOTER_NAME_ICHARGE	"BTP"
#define VOTER_NAME_VFLOAT	"BTP"
#define VOTER_NAME_CHILLY	"BTP(CHILLY)"

static struct protection_battemp {
	struct delayed_work	battemp_dwork;
	struct wakeup_source	*battemp_wakelock;
#ifdef DEBUG_BTP
	struct delayed_work	debug_btp_dwork;
#endif

	// processed in external
	bool (*get_protection_battemp)(bool* charging, int* temperature, int* mvoltage);
	void (*set_protection_battemp)(int health, int micharge, int mvfloat);

	struct voter_entry voter_ichilly;
	struct voter_entry voter_icharge;
	struct voter_entry voter_vfloat;

	bool health_chilly;
	int  health_jeita;

// below fields are set in device tree
	int threshold_degc_upto_cool;	//  30 by default
	int threshold_degc_upto_good;	// 120 by default
	int threshold_degc_upto_warm;	// 450 by default
	int threshold_degc_upto_hot;	// 550 by default
	int threshold_degc_downto_warm;	// 520 by default
	int threshold_degc_downto_good;	// 430 by default
	int threshold_degc_downto_cool;	// 100 by default
	int threshold_degc_downto_cold;	//   0 by default

	int period_ms_emergency;	// 10000 by default
	int period_ms_warning;		// 30000 by default
	int period_ms_normal;		// 60000 by default
	const int period_ms_unknown;

	int cool_mv_alert;
	int cool_ma_alert;
	int cool_ma_normal;

	int warm_ma_charge;
	int warm_mv_float;

	// below fileds are for battery protection in chilly
	bool chilly_is_supported;
	int  chilly_degc_lowerbound;
	int  chilly_degc_upperbound;
	int  chilly_mv_hyst;
	int  chilly_mv_bound;
	int  chilly_mv_alert;
	int  chilly_ma_alert;
	int  chilly_ma_normal;

} battemp_me = {
	.voter_ichilly = { .type = VOTER_TYPE_INVALID },
	.voter_icharge = { .type = VOTER_TYPE_INVALID },
	.voter_vfloat  = { .type = VOTER_TYPE_INVALID },

	.health_jeita  = POWER_SUPPLY_HEALTH_UNKNOWN,
	.health_chilly = false,

	.threshold_degc_upto_cool  = BATTEMP_NOTREADY,
	.threshold_degc_upto_good  = BATTEMP_NOTREADY,
	.threshold_degc_upto_warm  = BATTEMP_NOTREADY,
	.threshold_degc_upto_hot   = BATTEMP_NOTREADY,
	.threshold_degc_downto_warm = BATTEMP_NOTREADY,
	.threshold_degc_downto_good = BATTEMP_NOTREADY,
	.threshold_degc_downto_cool = BATTEMP_NOTREADY,
	.threshold_degc_downto_cold = BATTEMP_NOTREADY,

	.period_ms_emergency = BATTEMP_NOTREADY,
	.period_ms_warning   = BATTEMP_NOTREADY,
	.period_ms_normal    = BATTEMP_NOTREADY,
	.period_ms_unknown   = 1000,

	.cool_mv_alert	= BATTEMP_NOTREADY,
	.cool_ma_alert	= BATTEMP_NOTREADY,
	.cool_ma_normal	= BATTEMP_NOTREADY,
	.warm_ma_charge	= BATTEMP_NOTREADY,
	.warm_mv_float	= BATTEMP_NOTREADY,

	.chilly_is_supported    = false,
	.chilly_degc_upperbound = BATTEMP_NOTREADY,
	.chilly_degc_lowerbound = BATTEMP_NOTREADY,
	.chilly_mv_hyst		= BATTEMP_NOTREADY,
	.chilly_mv_bound	= BATTEMP_NOTREADY,
	.chilly_mv_alert	= BATTEMP_NOTREADY,
	.chilly_ma_alert	= BATTEMP_NOTREADY,
	.chilly_ma_normal       = BATTEMP_NOTREADY,
};

#ifdef DEBUG_BTP
#define BTP_POWER_OFF_TEMP_IDX 22
#define BTP_POWER_OFF_WARNING_TEMP_IDX (BTP_POWER_OFF_TEMP_IDX-3)
#define BTP_POWER_OFF_CNT   3
#define BTP_RECHECK_PERIOD 60000

struct debug_btp {
	int temp;
	int capacity;
	int voltage;
};

const struct debug_btp debug_btp_table[BTP_POWER_OFF_TEMP_IDX] = {
	{-150, 80, 4000},   // HEALTH_COLD
	{-100, 80, 4000},   // HEALTH_COLD
	{-50, 80, 4000},    // HEALTH_COLD
	{0, 80, 4000},      // HEALTH_COLD HEALTH_CHIILY    (Charging Stop)
	{30, 80, 4000},     // HEALTH_COOL HEALTH_CHIILY    (Decrease Charging 0.3C Under 4V, Decrease Charging 0.2C Over 4V)
	{50, 80, 4000},     // HEALTH_COOL HEALTH_CHILLY    (Decrease Charging 0.3C Under 4V, Decrease Charging 0.2C Over 4V)
	{100, 80, 4000},    // HEALTH_COOL HEALTH_CHILLY    (Decrease Charging 0.3C Under 4V, Decrease Charging 0.2C Over 4V)
	{120, 80, 4000},    // HEALTH_GOOD HEALTH_CHILLY    (Up to Normal)
	{150, 80, 4000},    // HEALTH_GOOD                  (HEALTH_CHILLY : Acordding to Battery Spec)
	{200, 80, 4000},    // HEALTH_GOOD                  (HEALTH_CHIILY : Acordding to Battery Spec)
	{250, 80, 4000},    // HEALTH_GOOD
	{300, 80, 4000},    // HEALTH_GOOD
	{350, 80, 4000},    // HEALTH_GOOD
	{400, 80, 4000},    // HEALTH_GOOD
	{430, 80, 4000},    // HEALTH_GOOD                  (Down to Normal)
	{450, 80, 4000},    // HEALTH_WARM                  (Decrease Charging Under 4V, Charging Stop Over 4V)
	{500, 80, 4000},    // HEALTH_WARM                  (Decrease Charging Under 4V, Charging Stop Over 4V)
	{520, 80, 4000},    // HEALTH_WARM                  (Down to Warm)
	{550, 80, 4000},    // HEALTH_HOT                   (Charging Stop)
	{590, 80, 4000},    // HEALTH_HOT                   (VZW Cool Down Noti & Power Off Message)
	{600, 80, 4000},    // HEALTH_HOT                   (Imediately Power Off)
	{650, 80, 4000},    // HEALTH_HOT
};

static void debug_btp_polling_status_work(struct work_struct* work) {
	struct power_supply* psy;
	union power_supply_propval prp_temp, prp_capacity, prp_voltagenow;
	static int tempstep = 0;
	static bool voltagelevel = false;
	static bool upward = true;
	static int power_off_cnt = 0;
	char buf[2] = {0, };

	unified_nodes_show("fake_battery", buf);

	if (unified_bootmode_chargerlogo() && !strcmp(buf, "0")) //enable debug_btp in chargerlogo
		unified_nodes_store("fake_battery", "1", 1);

	if (!strcmp(buf, "1")) {
		if (power_off_cnt < BTP_POWER_OFF_CNT) {
			if(tempstep >= BTP_POWER_OFF_WARNING_TEMP_IDX) {
				upward = false;
			} else if(tempstep <= 0) {
				upward = true;
				voltagelevel = !voltagelevel;
				power_off_cnt++;
			}
		} else {
			if(tempstep >= BTP_POWER_OFF_TEMP_IDX - 1) {
				upward = false;
			} else if(tempstep <= 0) {
				upward = true;
				voltagelevel = !voltagelevel;
			}
		}

		psy = power_supply_get_by_name("bms");
		if(!psy) {
			schedule_delayed_work(to_delayed_work(work), msecs_to_jiffies(BTP_RECHECK_PERIOD));
			return;
		}

		prp_temp.intval = debug_btp_table[tempstep].temp;
		prp_capacity.intval = debug_btp_table[tempstep].capacity;
		if (!voltagelevel)
			prp_voltagenow.intval = (debug_btp_table[tempstep].voltage - 100) * 1000;
		else
			prp_voltagenow.intval = (debug_btp_table[tempstep].voltage + 100) * 1000;

		pr_battemp(UPDATE, "temp = %d, capacity = %d, voltage = %d\n",
				prp_temp.intval, prp_capacity.intval, prp_voltagenow.intval);
		if (psy) {
			power_supply_set_property(psy, POWER_SUPPLY_PROP_TEMP, &prp_temp);
			power_supply_set_property(psy, POWER_SUPPLY_PROP_CAPACITY, &prp_capacity);
			power_supply_set_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &prp_voltagenow);
			power_supply_put(psy);
		}

		if(upward)
			tempstep++;
		else
			tempstep--;

	} else {
		; //Nothing To Do
	}
	schedule_delayed_work(to_delayed_work(work), msecs_to_jiffies(BTP_RECHECK_PERIOD));
}
#endif
static const char* health_to_string(bool chilly, int jhealth) {
	if (!chilly) {
		switch (jhealth) {
		case POWER_SUPPLY_HEALTH_UNKNOWN :
			return "HEALTH_UNKNOWN";
		case POWER_SUPPLY_HEALTH_COLD :
			return "HEALTH_COLD";
		case POWER_SUPPLY_HEALTH_COOL :
			return "HEALTH_COOL";
		case POWER_SUPPLY_HEALTH_GOOD :;
			return "HEALTH_GOOD";
		case POWER_SUPPLY_HEALTH_WARM :
			return "HEALTH_WARM";
		case POWER_SUPPLY_HEALTH_HOT :
			return "HEALTH_HOT";
		default :
			return "Undefined health";
		}
	}
	else
		return "HEALTH_CHILLY";
}

static int health_to_index(bool chilly, int jhealth) {
	if (!chilly) {
		switch (jhealth) {
		case POWER_SUPPLY_HEALTH_UNKNOWN :
			return 0;
		case POWER_SUPPLY_HEALTH_COLD :
			return 1;
		case POWER_SUPPLY_HEALTH_COOL :
			return 2;
		case POWER_SUPPLY_HEALTH_GOOD :;
			return 3 + battemp_me.chilly_is_supported;
		case POWER_SUPPLY_HEALTH_WARM :
			return 4 + battemp_me.chilly_is_supported;
		case POWER_SUPPLY_HEALTH_HOT :
			return 5 + battemp_me.chilly_is_supported;
		default :
			return -1;
		}
	}
	else
		return 3;
}

static long health_to_period(bool chilly, int jhealth) {
	int msecs = 0;

	if (!chilly) {
		switch (jhealth) {
		case POWER_SUPPLY_HEALTH_HOT :
		case POWER_SUPPLY_HEALTH_COLD :
			msecs = battemp_me.period_ms_emergency;
			break;
		case POWER_SUPPLY_HEALTH_WARM :
		case POWER_SUPPLY_HEALTH_COOL :
			msecs = battemp_me.period_ms_warning;
			break;
		case POWER_SUPPLY_HEALTH_GOOD :
			msecs = battemp_me.period_ms_normal;
			break;
		case POWER_SUPPLY_HEALTH_UNKNOWN :
			msecs = battemp_me.period_ms_unknown;
			break;
		default :
			pr_battemp(ERROR, "Check the 'health'\n");
			break;
		}
	}
	else
		msecs = battemp_me.period_ms_warning;

	return msecs_to_jiffies(msecs);
}

static int icharge_by_chilly(bool chilly, int batvol) {
	if (chilly) {
		if(battemp_me.chilly_mv_hyst <= batvol) {
			battemp_me.chilly_mv_hyst = battemp_me.chilly_mv_alert - battemp_me.chilly_mv_bound;
			pr_battemp(ERROR, "make hysterisis voltage 4000 to 3800 mV\n");
			return battemp_me.chilly_ma_alert;
		} else {
			battemp_me.chilly_mv_hyst = battemp_me.chilly_mv_alert;
			pr_battemp(ERROR, "Not in hysterisis threshold 4000 mV\n");
			return battemp_me.chilly_ma_normal;
		}
	} else {
		battemp_me.chilly_mv_hyst = battemp_me.chilly_mv_alert;
		return VOTE_TOTALLY_RELEASED;
	}
}

static int icharge_by_jhealth(int jhealth, int batvol) {
	switch (jhealth) {
	case POWER_SUPPLY_HEALTH_COOL :
		return (battemp_me.cool_mv_alert <= batvol)
			? battemp_me.cool_ma_alert : battemp_me.cool_ma_normal;
	case POWER_SUPPLY_HEALTH_WARM :
		return battemp_me.warm_ma_charge;

	case POWER_SUPPLY_HEALTH_COLD :
	case POWER_SUPPLY_HEALTH_HOT :
		return VOTE_TOTALLY_BLOCKED;

	case POWER_SUPPLY_HEALTH_GOOD :
	case POWER_SUPPLY_HEALTH_UNKNOWN :
		return VOTE_TOTALLY_RELEASED;

	default :
		return -EINVAL;
	}
}

static int vfloat_by_jhealth(int jhealth) {
	switch (jhealth) {
	case POWER_SUPPLY_HEALTH_GOOD :
	case POWER_SUPPLY_HEALTH_COOL :
	case POWER_SUPPLY_HEALTH_COLD :
	case POWER_SUPPLY_HEALTH_UNKNOWN :
		return VOTE_TOTALLY_RELEASED;

	case POWER_SUPPLY_HEALTH_HOT :
	case POWER_SUPPLY_HEALTH_WARM :
		return battemp_me.warm_mv_float;

	default :
		return -EINVAL;
	}
}

#define STAT_NOW (health_now)
#define TEMP_NOW (battemp_now)

#define TEMP_UPTO_COOL (battemp_me.threshold_degc_upto_cool)		//	 30 by default
#define TEMP_UPTO_GOOD (battemp_me.threshold_degc_upto_good)		//	120 by default
#define TEMP_UPTO_WARM (battemp_me.threshold_degc_upto_warm)		//	450 by default
#define TEMP_UPTO_HOT (battemp_me.threshold_degc_upto_hot)		//	550 by default
#define TEMP_DOWNTO_WARM (battemp_me.threshold_degc_downto_warm)	//	520 by default
#define TEMP_DOWNTO_GOOD (battemp_me.threshold_degc_downto_good)	//	430 by default
#define TEMP_DOWNTO_COOL (battemp_me.threshold_degc_downto_cool)	//	100 by default
#define TEMP_DOWNTO_COLD (battemp_me.threshold_degc_downto_cold)	//	  0 by default
static int polling_status_jeita(int health_now, int battemp_now) {
	int health_new;

	switch (STAT_NOW) {
	case POWER_SUPPLY_HEALTH_UNKNOWN :
		if (TEMP_NOW < TEMP_DOWNTO_COLD)
			health_new = POWER_SUPPLY_HEALTH_COLD;
		else if (TEMP_NOW < TEMP_DOWNTO_COOL)
			health_new = POWER_SUPPLY_HEALTH_COOL;
		else if (TEMP_NOW < TEMP_UPTO_WARM )
			health_new = POWER_SUPPLY_HEALTH_GOOD;
		else if (TEMP_NOW < TEMP_UPTO_HOT)
			health_new = POWER_SUPPLY_HEALTH_WARM;
		else
			health_new = POWER_SUPPLY_HEALTH_HOT;
		break;

	case POWER_SUPPLY_HEALTH_COLD : // on the cold
		if (TEMP_NOW < TEMP_UPTO_COOL)
			health_new = POWER_SUPPLY_HEALTH_COLD;
		else if (TEMP_NOW < TEMP_DOWNTO_COOL)
			health_new = POWER_SUPPLY_HEALTH_COOL;
		else if (TEMP_NOW < TEMP_UPTO_WARM )
			health_new = POWER_SUPPLY_HEALTH_GOOD;
		else if (TEMP_NOW < TEMP_UPTO_HOT)
			health_new = POWER_SUPPLY_HEALTH_WARM;
		else
			health_new = POWER_SUPPLY_HEALTH_HOT;
		break;

	case POWER_SUPPLY_HEALTH_COOL : // on the cool
		if (TEMP_NOW < TEMP_DOWNTO_COLD)
			health_new = POWER_SUPPLY_HEALTH_COLD;
		else if (TEMP_NOW < TEMP_UPTO_GOOD)
			health_new = POWER_SUPPLY_HEALTH_COOL;
		else if (TEMP_NOW < TEMP_UPTO_WARM )
			health_new = POWER_SUPPLY_HEALTH_GOOD;
		else if (TEMP_NOW < TEMP_UPTO_HOT)
			health_new = POWER_SUPPLY_HEALTH_WARM;
		else
			health_new = POWER_SUPPLY_HEALTH_HOT;
		break;

	case POWER_SUPPLY_HEALTH_GOOD : // on the normal
		if (TEMP_NOW < TEMP_DOWNTO_COLD)
			health_new = POWER_SUPPLY_HEALTH_COLD;
		else if (TEMP_NOW < TEMP_DOWNTO_COOL)
			health_new = POWER_SUPPLY_HEALTH_COOL;
		else if (TEMP_NOW < TEMP_UPTO_WARM )
			health_new = POWER_SUPPLY_HEALTH_GOOD;
		else if (TEMP_NOW < TEMP_UPTO_HOT)
			health_new = POWER_SUPPLY_HEALTH_WARM;
		else
			health_new = POWER_SUPPLY_HEALTH_HOT;
		break;

	case POWER_SUPPLY_HEALTH_WARM : // on the warm
		if (TEMP_NOW < TEMP_DOWNTO_COLD)
			health_new = POWER_SUPPLY_HEALTH_COLD;
		else if (TEMP_NOW < TEMP_DOWNTO_COOL)
			health_new = POWER_SUPPLY_HEALTH_COOL;
		else if (TEMP_NOW < TEMP_DOWNTO_GOOD )
			health_new = POWER_SUPPLY_HEALTH_GOOD;
		else if (TEMP_NOW < TEMP_UPTO_HOT)
			health_new = POWER_SUPPLY_HEALTH_WARM;
		else
			health_new = POWER_SUPPLY_HEALTH_HOT;
		break;

	case POWER_SUPPLY_HEALTH_HOT : // on the hot
		if (TEMP_NOW < TEMP_DOWNTO_COLD)
			health_new = POWER_SUPPLY_HEALTH_COLD;
		else if (TEMP_NOW < TEMP_DOWNTO_COOL)
			health_new = POWER_SUPPLY_HEALTH_COOL;
		else if (TEMP_NOW < TEMP_UPTO_WARM )
			health_new = POWER_SUPPLY_HEALTH_GOOD;
		else if (TEMP_NOW < TEMP_DOWNTO_WARM)
			health_new = POWER_SUPPLY_HEALTH_WARM;
		else
			health_new = POWER_SUPPLY_HEALTH_HOT;
		break;
	default :
		health_new = POWER_SUPPLY_HEALTH_UNKNOWN;
		break;
	}

	return health_new;
}

#define CHILLY_TEMP_LOWERBOUND (battemp_me.chilly_degc_lowerbound)
#define CHILLY_TEMP_UPPERBOUND (battemp_me.chilly_degc_upperbound)
static bool polling_status_chilly(int battemp_now){
	return battemp_me.chilly_is_supported
		&& CHILLY_TEMP_LOWERBOUND <= battemp_now && battemp_now <= CHILLY_TEMP_UPPERBOUND;
}

static void polling_status_work(struct work_struct* work) {
	bool charging;
	int  temperature, mvoltage;
	int health_jeita, updated_icharge, updated_vfloat, updated_ichilly;
	bool health_chilly, warning_at_charging, warning_wo_charging;

	if (battemp_me.get_protection_battemp(&charging, &temperature, &mvoltage)) {
		// Calculates icharge and vfloat from the jeita health
		health_jeita = polling_status_jeita(battemp_me.health_jeita, temperature);
		updated_icharge = charging ? icharge_by_jhealth(health_jeita, mvoltage) : VOTE_TOTALLY_RELEASED;
		updated_vfloat = charging ? vfloat_by_jhealth(health_jeita) : VOTE_TOTALLY_RELEASED;

		// And ichilly from the boolean 'chilly' status
		health_chilly = polling_status_chilly(temperature);
		updated_ichilly = charging ? icharge_by_chilly(health_chilly, mvoltage) : VOTE_TOTALLY_RELEASED;

		// configure wakelock
		warning_at_charging = charging && (health_jeita != POWER_SUPPLY_HEALTH_GOOD);
		warning_wo_charging = health_jeita == POWER_SUPPLY_HEALTH_HOT;

		if (warning_at_charging || warning_wo_charging) {
			if (!battemp_me.battemp_wakelock->active) {
				pr_battemp(UPDATE, "Acquiring wake lock\n");
				__pm_stay_awake(battemp_me.battemp_wakelock);
			}
		}
		else {
			if (battemp_me.battemp_wakelock->active) {
				pr_battemp(UPDATE, "Releasing wake lock\n");
				__pm_relax(battemp_me.battemp_wakelock);
			}
		}

		// logging for changes
		if (battemp_me.health_chilly != health_chilly || battemp_me.health_jeita != health_jeita)
			pr_battemp(UPDATE, "%s(%d) -> %s(%d), temperature=%d, mvoltage=%d\n",
				health_to_string(battemp_me.health_chilly, battemp_me.health_jeita),
					health_to_index(battemp_me.health_chilly, battemp_me.health_jeita),
				health_to_string(health_chilly, health_jeita),
					health_to_index(health_chilly, health_jeita),
				temperature, mvoltage);

		// Voting for icharge and vfloat
		veneer_voter_set(&battemp_me.voter_ichilly, updated_ichilly);
		veneer_voter_set(&battemp_me.voter_icharge, updated_icharge);
		veneer_voter_set(&battemp_me.voter_vfloat, updated_vfloat);
		if (updated_vfloat != VOTE_TOTALLY_RELEASED)
			veneer_voter_rerun(&battemp_me.voter_vfloat);

		// update member status in 'battemp_me'
		battemp_me.health_chilly = health_chilly;
		battemp_me.health_jeita = health_jeita;

		// finallly, notify the psy-type health to client
		battemp_me.set_protection_battemp(battemp_me.health_jeita,
			min(updated_ichilly, updated_icharge), updated_vfloat);
	}
	else
		pr_battemp(UPDATE, "temperature is not valid.\n");

	schedule_delayed_work(to_delayed_work(work),
		health_to_period(battemp_me.health_chilly, battemp_me.health_jeita));
	return;
}

#define SCALE_UNIT_MA	50
static bool battemp_create_parsedt(struct device_node* dnode, int mincap) {
	int rc = 0;
	int cool_ma_pct = 0, warm_ma_pct = 0;
	int chilly_ma_pct = 0;

	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_upto_cool,
		"threshold-degc-upto-cool", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_upto_good,
		"threshold-degc-upto-good", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_upto_warm,
		"threshold-degc-upto-warm", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_upto_hot,
		"threshold-degc-upto-hot", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_downto_warm,
		"threshold-degc-downto-warm", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_downto_good,
		"threshold-degc-downto-good", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_downto_cool,
		"threshold-degc-downto-cool", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_downto_cold,
		"threshold-degc-downto-cold", rc);

	OF_PROP_READ_S32(dnode, battemp_me.period_ms_emergency,
		"period-ms-emergency", rc);
	OF_PROP_READ_S32(dnode, battemp_me.period_ms_warning,
		"period-ms-warning", rc);
	OF_PROP_READ_S32(dnode, battemp_me.period_ms_normal,
		"period-ms-normal", rc);

	OF_PROP_READ_S32(dnode, battemp_me.cool_mv_alert,
		"cool-mv-alert", rc);
	OF_PROP_READ_S32(dnode, battemp_me.cool_ma_alert,
		"cool-ma-alert", rc);
	OF_PROP_READ_S32(dnode, cool_ma_pct, "cool-ma-pct", rc);
		battemp_me.cool_ma_normal = (mincap * cool_ma_pct / 100) / SCALE_UNIT_MA * SCALE_UNIT_MA;

	OF_PROP_READ_S32(dnode, battemp_me.warm_mv_float,
		"warm-mv-float", rc);
	OF_PROP_READ_S32(dnode, warm_ma_pct, "warm-ma-pct", rc);
		battemp_me.warm_ma_charge = (mincap * warm_ma_pct / 100) / SCALE_UNIT_MA * SCALE_UNIT_MA;

	battemp_me.chilly_is_supported = of_property_read_bool(dnode, "lge,chilly-status-support");
	if (battemp_me.chilly_is_supported) {
		OF_PROP_READ_S32(dnode, battemp_me.chilly_degc_lowerbound,
			"chilly-degc-lowerbound", rc);
		OF_PROP_READ_S32(dnode, battemp_me.chilly_degc_upperbound,
			"chilly-degc-upperbound", rc);
		OF_PROP_READ_S32(dnode, battemp_me.chilly_mv_hyst,
			"chilly-mv-alert", rc);
		OF_PROP_READ_S32(dnode, battemp_me.chilly_mv_bound,
			"chilly-mv-bound", rc);
		OF_PROP_READ_S32(dnode, battemp_me.chilly_mv_alert,
			"chilly-mv-alert", rc);
		OF_PROP_READ_S32(dnode, battemp_me.chilly_ma_alert,
			"chilly-ma-alert", rc);
		OF_PROP_READ_S32(dnode, chilly_ma_pct, "chilly-ma-pct", rc);
			battemp_me.chilly_ma_normal =
				(mincap * chilly_ma_pct / 100) / SCALE_UNIT_MA * SCALE_UNIT_MA;
	}

	return !rc;
}

static bool battemp_create_voters(void) {
	return veneer_voter_register(&battemp_me.voter_ichilly, VOTER_NAME_CHILLY, VOTER_TYPE_IBAT, false)
		&& veneer_voter_register(&battemp_me.voter_icharge, VOTER_NAME_ICHARGE, VOTER_TYPE_IBAT, false)
		&& veneer_voter_register(&battemp_me.voter_vfloat, VOTER_NAME_VFLOAT, VOTER_TYPE_VFLOAT, false);
}

static bool battemp_create_preset(bool (*feed_protection_battemp)(bool* charging, int* temperature, int* mvoltage),
	void (*back_protection_battemp)(int health, int micharge, int mvfloat)) {

	if( feed_protection_battemp && back_protection_battemp ) {
		battemp_me.get_protection_battemp = feed_protection_battemp;
		battemp_me.set_protection_battemp = back_protection_battemp;
	}
	else {
		pr_battemp(ERROR, "feed/back func should not be null\n");
		return false;
	}

	battemp_me.battemp_wakelock = wakeup_source_register(NULL,
			BATTEMP_WAKELOCK);

	INIT_DELAYED_WORK(&battemp_me.battemp_dwork,
		polling_status_work);
#ifdef DEBUG_BTP
	INIT_DELAYED_WORK(&battemp_me.debug_btp_dwork,
		debug_btp_polling_status_work);
#endif

	return true;
}

void protection_battemp_monitor(void) {
	if (delayed_work_pending(&battemp_me.battemp_dwork))
		cancel_delayed_work(&battemp_me.battemp_dwork);
	schedule_delayed_work(&battemp_me.battemp_dwork, msecs_to_jiffies(0));
}

#ifdef DEBUG_BTP
void debug_btp_monitor(void) {
	if (delayed_work_pending(&battemp_me.debug_btp_dwork))
		cancel_delayed_work(&battemp_me.debug_btp_dwork);
	schedule_delayed_work(&battemp_me.debug_btp_dwork, msecs_to_jiffies(0));
}
#endif

bool protection_battemp_create(struct device_node* dnode, int mincap,
	bool (*feed_protection_battemp)(bool* charging, int* temperature, int* mvoltage),
	void (*back_protection_battemp)(int health, int micharge, int mvfloat)) {
	pr_debugmask = ERROR | UPDATE;

	if (!battemp_create_preset(feed_protection_battemp, back_protection_battemp)) {
		pr_battemp(ERROR, "error on battemp_create_preset");
		goto destroy;
	}

	if (!battemp_create_parsedt(dnode, mincap)) {
		pr_battemp(ERROR, "error on battemp_create_devicetree");
		goto destroy;
	}

	if (!battemp_create_voters()) {
		pr_battemp(ERROR, "error on battemp_create_voters");
		goto destroy;
	}

	protection_battemp_monitor();
#ifdef DEBUG_BTP
	debug_btp_monitor();
#endif
	pr_battemp(UPDATE, "Complete to create\n");
	return true;
destroy:
	protection_battemp_destroy();
	return false;
}

void protection_battemp_destroy(void) {
	wakeup_source_unregister(battemp_me.battemp_wakelock);
	cancel_delayed_work_sync(&battemp_me.battemp_dwork);
#ifdef DEBUG_BTP
	cancel_delayed_work_sync(&battemp_me.debug_btp_dwork);
#endif

	veneer_voter_unregister(&battemp_me.voter_ichilly);
	veneer_voter_unregister(&battemp_me.voter_icharge);
	veneer_voter_unregister(&battemp_me.voter_vfloat);

	battemp_me.get_protection_battemp = NULL;
	battemp_me.set_protection_battemp = NULL;

	battemp_me.health_chilly = false;
	battemp_me.health_jeita = POWER_SUPPLY_HEALTH_UNKNOWN;

	battemp_me.threshold_degc_upto_cool   = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_upto_good   = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_upto_warm   = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_upto_hot    = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_downto_warm = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_downto_good = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_downto_cool = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_downto_cold = BATTEMP_NOTREADY;

	battemp_me.period_ms_emergency = BATTEMP_NOTREADY;
	battemp_me.period_ms_warning   = BATTEMP_NOTREADY;
	battemp_me.period_ms_normal    = BATTEMP_NOTREADY;

	battemp_me.cool_mv_alert  = BATTEMP_NOTREADY,
	battemp_me.cool_ma_alert  = BATTEMP_NOTREADY,
	battemp_me.cool_ma_normal = BATTEMP_NOTREADY,
	battemp_me.warm_ma_charge = BATTEMP_NOTREADY;
	battemp_me.warm_mv_float  = BATTEMP_NOTREADY;

	battemp_me.chilly_is_supported    = false;
	battemp_me.chilly_degc_upperbound = BATTEMP_NOTREADY;
	battemp_me.chilly_degc_lowerbound = BATTEMP_NOTREADY;
	battemp_me.chilly_mv_hyst	  = BATTEMP_NOTREADY;
	battemp_me.chilly_mv_bound	  = BATTEMP_NOTREADY;
	battemp_me.chilly_mv_alert	  = BATTEMP_NOTREADY;
	battemp_me.chilly_ma_alert	  = BATTEMP_NOTREADY;
	battemp_me.chilly_ma_normal       = BATTEMP_NOTREADY;
}


