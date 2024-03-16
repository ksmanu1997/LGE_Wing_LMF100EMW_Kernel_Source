// ACTM: Adaptive Charging Thermal Mitigation since 2019.05.15
#define pr_fmt(fmt) "ACTM: %s: " fmt, __func__
#define pr_actm(reason, fmt, ...)			\
do {							\
	if (pr_debugmask & (reason))			\
		pr_info(fmt, ##__VA_ARGS__);		\
	else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
} while (0)

#include "veneer-primitives.h"

#define VOTER_NAME_ACTM	"ACTM"
#define VOTER_NAME_ACTM_SIDECAR	"SIDECAR"
#define ACTM_SENSOR_BATT    BIT(0)
#define ACTM_SENSOR_VTS     BIT(1)
#define ACTM_SENSOR_SKIN    BIT(2)      /* reserved */

#define ACTM_INITVAL        -9999

//static int pr_debugmask = ERROR | UPDATE;
static int pr_debugmask = ERROR | UPDATE | MONITOR;
//static int pr_debugmask = ERROR | UPDATE | MONITOR | EVALUATE;

enum actm_mode_type {
    ACTM_MODE_NOT_SUPPORT = -2,  /* for non-supported model */
    ACTM_MODE_DISABLE = -1,      /* for run-time change */
    ACTM_MODE_THERMAL = 0,
    ACTM_MODE_BALANCE,
    ACTM_MODE_CHARGING,
    ACTM_MODE_AUTO,
    ACTM_MODE_MAX = ACTM_MODE_AUTO,
};

enum actm_conn_type {
    DISCHG = -1,
    WIRED = 0,
    WIRELESS,
    ACTM_CONN_MAX,
};

enum actm_lcd_type {
    LCD_OFF = 0,
    LCD_ON = 1,
    ACTM_LCD_MAX,
};

#define ACTM_CP_TYPE_SIZE  2
enum actm_cp_type {
    CP_PPS = 0,
    CP_QC30 = 1,
    CP_NONE = 2,
};

enum actm_therm_type {
    BATT = 0,
    VTS,
    SKIN,
    ACTM_THERM_MAX,
};

enum actm_stage {
    ACTM_STAGE_NONE = -2,
    ACTM_STAGE_COLD = -1,
    ACTM_STAGE_NORMAL = 0,
    ACTM_STAGE_WARM,
    ACTM_STAGE_HOT,
    ACTM_STAGE_MAX,
};

enum actm_policy {
    TEMP_POLICY_HOLD = 0,
    TEMP_POLICY_CURRENT_UP,
    TEMP_POLICY_COOL_DOWN,
    TEMP_POLICY_FREEZING,
    ACTM_TEMP_POLICY_MAX,
};

#define AUTO_MODE_SOC_SIZE          2
#define AUTO_MODE_CONFIG_SIZE       3
#define ACTM_TEMP_SIZE_MAX          5
#define ACTM_CURR_STEP_SIZE_MAX     5
#define ACTM_PROBATION_INTERVAL     5000   /*  5 sec */
#define ACTM_MIN_INTERVAL_FCC       3000   /* 3000mA */
#define ACTM_MIN_INTERVAL           15000  /* 15 sec */
#define ACTM_MID_INTERVAL_FCC       2500   /* 2500mA */
#define ACTM_MID_INTERVAL           30000  /* 30 sec */
#define ACTM_15W_WLC_MAX_FCC        3500   /* 3.5A for 12V WLC */

struct _actm_dt {
    int therm_type;
    int warm_to_normal_margin;
    int temp_offset[ACTM_MODE_MAX];
    int max_hold_criteria[AUTO_MODE_CONFIG_SIZE];
    int stage_size;
    int temp_criteria[ACTM_TEMP_SIZE_MAX];
    int wired_max_fcc[ACTM_TEMP_SIZE_MAX];
    int wired_curr[ACTM_TEMP_SIZE_MAX];
    int cp_curr[ACTM_CP_TYPE_SIZE];
    int epp_pwr[ACTM_TEMP_SIZE_MAX];
    int bpp_pwr[ACTM_TEMP_SIZE_MAX];
    int max_power_15w[ACTM_MODE_MAX];
    int curr_step_size;
    int curr_step_temp[ACTM_CURR_STEP_SIZE_MAX];
    int curr_step_ma[ACTM_CURR_STEP_SIZE_MAX];
    int epp_pwr_step_mw[ACTM_CURR_STEP_SIZE_MAX];
    int bpp_pwr_step_mw[ACTM_CURR_STEP_SIZE_MAX];
    int timer;

    /* sidecar */
    struct _sidecar {
        bool enable;
        bool is_on[ACTM_LCD_MAX];
        int enter_temp[ACTM_LCD_MAX];
        int exit_temp[ACTM_LCD_MAX];
        int min_curr[ACTM_LCD_MAX];
    } sidecar[ACTM_MODE_MAX];
};

static struct _actm {
    bool enable;
    bool enable_cp_charging;
    bool enable_on_chargerlogo;
    int mode;
    int cp_type;   // charger pump
    int auto_mode_soc[AUTO_MODE_SOC_SIZE];
    int auto_mode_config[AUTO_MODE_CONFIG_SIZE];
    int max_volt;  // it's from idtp9222 driver, mV
    int epp_volt;  // it's from idtp9222 driver, mV
    int bpp_volt;  // it's from idtp9222 driver, mV
    int max_curr;  // it's from idtp9222 driver, mV
    int epp_curr;  // it's from idtp9222 driver, mV
    int bpp_curr;  // it's from idtp9222 driver, mV
    int wireless_volt;  // it's from idtp9222 driver, mV
    int lcdon_temp_offset;

    struct _actm_dt dt[ACTM_CONN_MAX];  // 0-wired, 1-wirless

    int conn;
    int active;

    int old_temp;
    int ref_temp;

    int status_raw;
    int now_stage;
    int pre_stage;
    int now_timer;
    int hold_count;
    /* warm to normal transition margin */
    int cold_count;
    int cold_count_temp;
    int cold_count_temp_delta;

    struct voter_entry idc;
    struct voter_entry ibat;
    struct voter_entry sidecar_idc;
    struct voter_entry sidecar_ibat;
    struct delayed_work	dwork;
    int (*get_actm_param)(int id, int *val);
    int (*set_actm_param)(int id, int val);
} actm_me;

static void actm_sidecar_is_on_clear(void)
{
    actm_me.dt[WIRED].sidecar[ACTM_MODE_THERMAL].is_on[LCD_OFF] = false;
    actm_me.dt[WIRED].sidecar[ACTM_MODE_THERMAL].is_on[LCD_ON] = false;
    actm_me.dt[WIRELESS].sidecar[ACTM_MODE_THERMAL].is_on[LCD_OFF] = false;
    actm_me.dt[WIRELESS].sidecar[ACTM_MODE_THERMAL].is_on[LCD_ON] = false;

    actm_me.dt[WIRED].sidecar[ACTM_MODE_BALANCE].is_on[LCD_OFF] = false;
    actm_me.dt[WIRED].sidecar[ACTM_MODE_BALANCE].is_on[LCD_ON] = false;
    actm_me.dt[WIRELESS].sidecar[ACTM_MODE_BALANCE].is_on[LCD_OFF] = false;
    actm_me.dt[WIRELESS].sidecar[ACTM_MODE_BALANCE].is_on[LCD_ON] = false;

    actm_me.dt[WIRED].sidecar[ACTM_MODE_CHARGING].is_on[LCD_OFF] = false;
    actm_me.dt[WIRED].sidecar[ACTM_MODE_CHARGING].is_on[LCD_ON] = false;
    actm_me.dt[WIRELESS].sidecar[ACTM_MODE_CHARGING].is_on[LCD_OFF] = false;
    actm_me.dt[WIRELESS].sidecar[ACTM_MODE_CHARGING].is_on[LCD_ON] = false;
}

static void actm_init(void)
{
    actm_me.active = false;
    actm_me.ref_temp = ACTM_INITVAL;
    actm_me.old_temp = ACTM_INITVAL;
    actm_me.now_stage = ACTM_STAGE_NONE;
    actm_me.pre_stage = ACTM_STAGE_NONE;
    actm_me.now_timer = 0;
    actm_me.hold_count = 0;
    actm_me.cold_count = 0;
    actm_me.cold_count_temp = 0;
    actm_me.cold_count_temp_delta = 0;
    actm_me.wireless_volt = 0;

    actm_sidecar_is_on_clear();
}

static bool is_lcdon(void)
{
    int lcdon = 0;

    if (actm_me.get_actm_param(
        VENEER_FEED_LCDON_STATUS, &lcdon)) {
        pr_actm(ERROR, "get_actm_param(lcdon status) failure, error\n");
        lcdon = 0;
    }

    if (lcdon)
        return true;

    return false;
}

static int get_actm_mode(void)
{
    int i = 0, msoc = -1;
    int conn = WIRED, mode = ACTM_MODE_NOT_SUPPORT;
    const char str_mode[5][15] =
        {"not supported", "disabled", "thermal first", "balanced", "charging first"};

    if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_5W
        || actm_me.conn == CHARGING_SUPPLY_WIRELESS_9W
        || actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W)
        conn = WIRELESS;

    if (!actm_me.enable)
        return ACTM_MODE_NOT_SUPPORT;

    if (actm_me.mode == ACTM_MODE_DISABLE)
        return ACTM_MODE_DISABLE;

    if (conn == WIRED)
        mode = actm_me.mode % 10;
    else
        mode = actm_me.mode / 10;

    if (mode == ACTM_MODE_AUTO) {
#ifdef CONFIG_QPNP_QG
        actm_me.get_actm_param(VENEER_FEED_CAPACITY, &msoc);
        msoc = msoc * 10;
#else
        actm_me.get_actm_param(VENEER_FEED_CAPACITY_RAW, &msoc);
        msoc = msoc * 1000 / 255;
#endif
        mode = actm_me.auto_mode_config[AUTO_MODE_CONFIG_SIZE - 1];
        for (i = 0; i < AUTO_MODE_SOC_SIZE; i++) {
            if (actm_me.auto_mode_soc[i] >= (msoc / 10)) {
                mode = actm_me.auto_mode_config[i];
                break;
            }
        }
        pr_actm(EVALUATE,
            "auto mode: index=%d, soc=%d, %s", i, msoc, str_mode[mode + 2]);
    }

    return mode;
}

static void set_actm_mode(int mode)
{
    if (!actm_me.enable)
        actm_me.mode = ACTM_MODE_NOT_SUPPORT;
    else
        actm_me.mode = mode;
}

static int get_dt_temp(struct _actm_dt *dt, int stage)
{
    int lcdon_offset = 0;
    int dt_temp = dt->temp_criteria[stage];
    int dt_temp_offset = 0;
    int ret = dt_temp;

    const char str_stage[5][20] =
        {"none", "cold zone", "normal zone", "warm zone", "hot zone"};

    if (actm_me.get_actm_param(
        VENEER_FEED_ACTM_LCDON_TEMP_OFFSET, &lcdon_offset)) {
        pr_actm(ERROR, "get_actm_param(lcdon temp offset) failure, error\n");
        lcdon_offset = 0;
    }

    actm_me.lcdon_temp_offset = lcdon_offset;

    if (get_actm_mode() < ACTM_MODE_MAX &&
        get_actm_mode() > ACTM_MODE_DISABLE)
    {
        if (get_actm_mode() > ACTM_MODE_DISABLE) {
            dt_temp_offset = dt->temp_offset[get_actm_mode()];
            ret = dt_temp + dt_temp_offset - actm_me.lcdon_temp_offset;
        }

        if (stage == ACTM_STAGE_WARM &&
            actm_me.cold_count > 0 &&
            actm_me.cold_count_temp_delta >= 10 )
            ret += dt->warm_to_normal_margin;

        pr_actm(EVALUATE,
            "%s: return=%d (criteria=%d, offset=%d, lcdon=%d, wton=%d)\n",
            str_stage[stage + 2], ret, dt_temp,
            dt_temp_offset, actm_me.lcdon_temp_offset,
            dt->warm_to_normal_margin);
    }

    return ret;
}

static int get_dt_min_current(int stage)
{
    int local_stage = stage;

    if (local_stage > ACTM_STAGE_NORMAL &&
        get_actm_mode() == ACTM_MODE_CHARGING) {
        local_stage = ACTM_STAGE_NORMAL;
    }

    if (is_lcdon()) {
        local_stage++;
        if (local_stage > ACTM_STAGE_HOT)
            local_stage = ACTM_STAGE_HOT;
    }

    if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_9W ||
        actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W){
        return actm_me.dt[WIRELESS].epp_pwr[local_stage];
    }
    else if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_5W) {
        return actm_me.dt[WIRELESS].bpp_pwr[local_stage];
    }
    else {
        if (actm_me.enable_cp_charging && local_stage == ACTM_STAGE_NORMAL) {
            if (actm_me.cp_type < CP_NONE) {
                return min(
                    actm_me.dt[WIRED].cp_curr[actm_me.cp_type],
                    actm_me.dt[WIRED].wired_max_fcc[actm_me.cp_type]);
            }
        }
        return min(
            actm_me.dt[WIRED].wired_curr[local_stage],
            actm_me.dt[WIRED].wired_max_fcc[actm_me.cp_type]);
    }

    return 0;
}

static void set_actm_current(int set_curr, bool sidecar)
{
    int idc = 0, vdc = actm_me.bpp_volt;

    if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_9W ||
        actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W ){
        if (set_curr > 4000)
            vdc = actm_me.epp_volt;
        if (actm_me.max_volt > 0 &&
            actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W &&
            set_curr > (actm_me.epp_volt * 600 / 1000))
            vdc = actm_me.max_volt;
    }

    if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_5W
        || actm_me.conn == CHARGING_SUPPLY_WIRELESS_9W
        || actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W) {

        idc = set_curr * 1000 / vdc;
        idc = (idc / 50) * 50;

        actm_me.set_actm_param(VENEER_FEED_VDC, vdc);
        if (sidecar)
            veneer_voter_set(&actm_me.sidecar_idc, idc);
        else
            veneer_voter_set(&actm_me.idc, idc);

        actm_me.wireless_volt = vdc;

        pr_actm(MONITOR,
            "power=%d, idc=%d, vdc=%d\n", set_curr, idc, vdc);
    }
    else {
        if (sidecar)
            veneer_voter_set(&actm_me.sidecar_ibat, set_curr);
        else
            veneer_voter_set(&actm_me.ibat, set_curr);
    }
}

static int find_current_step(struct _actm_dt *dt, int pre, int now)
{
    int i = 0, curr_step = 0;
    int curr_step_array[ACTM_CURR_STEP_SIZE_MAX] = {0, };
    int temp_diff = max(pre, now) - min(pre, now);

    if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_9W ||
        actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W){
        for (i = 0; i < dt->curr_step_size; i++)
            curr_step_array[i] = dt->epp_pwr_step_mw[i];

        if (actm_me.wireless_volt == actm_me.max_volt)
            for (i = 0; i < dt->curr_step_size; i++)
                curr_step_array[i] =
                    dt->epp_pwr_step_mw[i] * actm_me.max_volt / actm_me.epp_volt;
    }
    else if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_5W) {
        for (i = 0; i < dt->curr_step_size; i++)
            curr_step_array[i] = dt->bpp_pwr_step_mw[i];
    }
    else {
        for (i = 0; i < dt->curr_step_size; i++)
            curr_step_array[i] = dt->curr_step_ma[i];
    }

    for (i = 0; i < dt->curr_step_size; i++) {
        if (dt->curr_step_temp[i] > temp_diff)
            break;
        curr_step = curr_step_array[i];
    }

    if (temp_diff < dt->curr_step_temp[0] || (pre < 0 && now < 0))
        curr_step = 0;

    pr_actm(EVALUATE,
        "step(%d, %d, %d), wlc(now=%d, max=%d), pre=%d, now=%d, step=%d\n",
            curr_step_array[0], curr_step_array[1], curr_step_array[2],
            actm_me.wireless_volt, actm_me.max_volt,
            pre, now, curr_step);
    return curr_step;
}

static int find_now_stage(struct _actm_dt *dt, int temp)
{
    int i = 0, stage = ACTM_STAGE_COLD;

    for (i = 0; i < dt->stage_size; i++) {
        if (temp < get_dt_temp(dt, i))
            break;
        stage = i;
    }

    if (get_actm_mode() == ACTM_MODE_CHARGING) {
        if (stage > ACTM_STAGE_NONE && stage < ACTM_STAGE_HOT)
            stage = ACTM_STAGE_COLD;
    }

    return stage;
}

static void set_actm_stage(struct _actm_dt *dt, int stage, int temp)
{
    int init_current = 0;
    const char str_stage[5][20] =
        {"none", "cold zone", "normal zone", "warm zone", "hot zone"};
    const char str_mode[5][15] =
        {"not supported", "disabled", "thermal first", "balanced", "charging first"};

    if (actm_me.now_stage == ACTM_STAGE_NONE) {
        // It means that this is the first connection to charger.
        actm_me.hold_count = 0;
        actm_me.now_stage = stage;

        actm_me.old_temp = temp;
        actm_me.ref_temp = temp;
        if (stage == ACTM_STAGE_HOT)
            actm_me.ref_temp = get_dt_temp(dt, stage);

        if (stage > ACTM_STAGE_NORMAL) {
            init_current = get_dt_min_current((stage - 1));
            set_actm_current(init_current, false);
        }

        /* init wireless_charging_voltage_now */
        if (actm_me.wireless_volt == 0) {
            if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_5W)
                actm_me.wireless_volt = actm_me.bpp_volt;
            else if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_9W)
                actm_me.wireless_volt = actm_me.epp_volt;
            else if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W)
                actm_me.wireless_volt = actm_me.max_volt;
        }

        pr_actm(UPDATE, "%s: new stage: %s, init current: %d, wlc_volt=%d\n",
            str_mode[max(get_actm_mode() + 2, 0)],
            str_stage[max(stage + 2, 0)], init_current, actm_me.wireless_volt);
    }
    else if (actm_me.now_stage != stage) {
        if (actm_me.pre_stage != actm_me.now_stage) {
            actm_me.pre_stage = actm_me.now_stage;
            actm_me.now_stage = stage;
            pr_actm(UPDATE, "%s, stage: %s -> %s, [temp] now: %d, ref: %d\n",
                str_mode[max(get_actm_mode() + 2, 0)],
                str_stage[max(actm_me.pre_stage + 2, 0)],
                str_stage[max(actm_me.now_stage + 2, 0)],
                temp, get_dt_temp(dt, actm_me.now_stage));
        }
    }
}

static int find_actm_policy(struct _actm_dt *dt, int ref, int now, bool is_hold)
{
    const char str_policy[4][20] =
        {"hold", "current up", "cool down", "freezing"};

    int policy = TEMP_POLICY_HOLD;
    int temp_diff = max(ref, now) - min(ref, now);

    int rise_ratio_per_min = 0;
    int hold_time = 0, max_hold = 0;
    int now_actm_mode = 0;

    if (now > ref && temp_diff >= dt->curr_step_temp[0]) {
        policy = TEMP_POLICY_COOL_DOWN;
        if (is_hold) {
            now_actm_mode = get_actm_mode();
            if (now_actm_mode < ACTM_MODE_THERMAL)
                now_actm_mode = ACTM_MODE_BALANCE;

            hold_time = (actm_me.now_timer / 1000 * actm_me.hold_count);
            max_hold = dt->max_hold_criteria[now_actm_mode];

            rise_ratio_per_min = max_hold * 2;
            if (hold_time > 0)
                rise_ratio_per_min = temp_diff * 60 * 1000 / hold_time;

            if (rise_ratio_per_min <= max_hold)
                policy = TEMP_POLICY_HOLD;
        }
    }
    else if (now < ref && temp_diff >= dt->curr_step_temp[0]) {
        policy = TEMP_POLICY_CURRENT_UP;
    }

    pr_actm(MONITOR,
        "policy: %s, rise_ratio=%d(max=%d) "
        "(diff: %d, ref: %d, now: %d)\n",
        str_policy[policy], rise_ratio_per_min,
        max_hold, temp_diff, ref, now);

    return policy;
}

static int get_actm_policy(struct _actm_dt *dt, int temp)
{
    int policy = TEMP_POLICY_HOLD;
    int mid_warm_and_hot =
        (get_dt_temp(dt, ACTM_STAGE_HOT) + get_dt_temp(dt, ACTM_STAGE_WARM))/2;

    switch (actm_me.now_stage) {
        case ACTM_STAGE_NORMAL:
            policy = find_actm_policy(dt, actm_me.ref_temp, temp, false);
            if (policy == TEMP_POLICY_COOL_DOWN) {
                policy = TEMP_POLICY_FREEZING;
            }
            break;
        case ACTM_STAGE_WARM:
            if ((actm_me.pre_stage == ACTM_STAGE_HOT)
                && (get_actm_mode() != ACTM_MODE_THERMAL)
                && (actm_me.conn != CHARGING_SUPPLY_WIRELESS_5W)
                && (actm_me.conn != CHARGING_SUPPLY_WIRELESS_9W)
                && (actm_me.conn != CHARGING_SUPPLY_WIRELESS_15W)) {
                if (temp > mid_warm_and_hot)
                    policy = TEMP_POLICY_FREEZING;
                else
                    policy = find_actm_policy(dt, actm_me.ref_temp, temp, true);
            }
            else {
                policy = find_actm_policy(dt, actm_me.ref_temp, temp, true);
            }
            break;
        case ACTM_STAGE_HOT:
            actm_me.ref_temp = get_dt_temp(dt, actm_me.now_stage);
            policy = find_actm_policy(dt, actm_me.ref_temp, temp, false);
            if (policy == TEMP_POLICY_COOL_DOWN) {
                policy = TEMP_POLICY_FREEZING;
            }
            break;
    }

    return policy;
}

static int actm_act(struct _actm_dt *dt, int policy, int temp)
{
    const char str_policy[4][20] =
        {"hold", "current up", "cool down", "freezing"};
    const char str_stage[5][20] =
        {"none", "cold zone", "normal zone", "warm zone", "hot zone"};
    const char str_mode[5][20] =
        {"not supported", "disabled", "thermal first", "balanced", "charging first"};

    int rc = 0;
    static int old_now_curr = 0;
    bool update_ref_temp_flag = false;
    int now_min_curr = 0, idc = 0, vdc = 0, now_curr = 0, set_curr = 0, qnovo = 0;

    int curr_step = find_current_step(dt, actm_me.ref_temp, temp);

    now_min_curr = get_dt_min_current(actm_me.now_stage);

    if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_5W
        || actm_me.conn == CHARGING_SUPPLY_WIRELESS_9W
        || actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W) {
        if (actm_me.get_actm_param(VENEER_FEED_IDC, &idc)) {
            pr_actm(ERROR,
                "get_actm_param(id=%d) failure, error\n", VENEER_FEED_IDC);
            return -1;
        }

        if (actm_me.get_actm_param(VENEER_FEED_VDC, &vdc)) {
            pr_actm(ERROR,
                "get_actm_param(id=%d) failure, error\n", VENEER_FEED_VDC);
            return -1;
        }

        now_curr = idc * vdc / 1000;
        pr_actm(EVALUATE,
                "now dc power=%d: i=%d, v=%d\n", now_curr, idc, vdc);
    }
    else {
        if (actm_me.get_actm_param(VENEER_FEED_FCC, &now_curr)) {
            pr_actm(ERROR,
                "get_actm_param(id=%d) failure, error\n", VENEER_FEED_FCC);
            return -1;
        }

        // detect qnovo diagnostic current
        if (now_curr == 500) {
            actm_me.get_actm_param(VENEER_FEED_QNOVO_DIAG_STAGE, &qnovo);
            if (qnovo == 1
                && actm_me.conn != CHARGING_SUPPLY_USB_2P0
                && actm_me.conn != CHARGING_SUPPLY_TYPE_FLOAT
                && actm_me.conn != CHARGING_SUPPLY_WIRELESS_5W
                && actm_me.conn != CHARGING_SUPPLY_WIRELESS_9W
                && actm_me.conn != CHARGING_SUPPLY_WIRELESS_15W) {
                pr_actm(MONITOR,
                    "remove qnovo diag current: now=%d, old=%d\n",
                    now_curr, old_now_curr);
                now_curr = old_now_curr;
            }
        }
    }

    old_now_curr = now_curr;

    switch (policy) {
        case TEMP_POLICY_HOLD:
            actm_me.hold_count++;
            curr_step = 0;
            set_curr = max(now_curr, now_min_curr);
            if (now_curr == 0 || set_curr > now_curr) {
                update_ref_temp_flag = true;
                set_actm_current(set_curr, false);
            }
            if (actm_me.cold_count > 0)
                actm_me.cold_count_temp_delta = actm_me.cold_count_temp - temp;
            break;
        case TEMP_POLICY_CURRENT_UP:
            actm_me.hold_count = 0;
            if (actm_me.cold_count > 0)
                actm_me.cold_count_temp_delta = actm_me.cold_count_temp - temp;

            set_curr = max(now_curr + curr_step, now_min_curr);
            if (actm_me.now_stage > ACTM_STAGE_NORMAL)
                set_curr = min(set_curr, get_dt_min_current(actm_me.now_stage - 1));

            if (now_curr == 0 || set_curr > now_curr) {
                actm_me.cold_count++;
                if (actm_me.cold_count == 1 && actm_me.cold_count_temp == 0)
                    actm_me.cold_count_temp = temp;
                update_ref_temp_flag = true;
                set_actm_current(set_curr, false);
            }
            break;
        case TEMP_POLICY_COOL_DOWN:
            actm_me.hold_count = 0;
            actm_me.cold_count = 0;
            actm_me.cold_count_temp = 0;
            actm_me.cold_count_temp_delta = 0;

            set_curr = max(now_curr - curr_step, now_min_curr);
            if (actm_me.now_stage > ACTM_STAGE_NORMAL)
                set_curr = min(set_curr, get_dt_min_current(actm_me.now_stage - 1));

            if (now_curr == 0 || set_curr < now_curr) {
                update_ref_temp_flag = true;
                set_actm_current(set_curr, false);
            }
            break;
        case TEMP_POLICY_FREEZING:
            actm_me.hold_count = 0;
            actm_me.cold_count = 0;
            actm_me.cold_count_temp = 0;
            actm_me.cold_count_temp_delta = 0;

            curr_step = find_current_step(dt, get_dt_temp(dt, actm_me.now_stage), temp);
            set_curr = max(now_curr - curr_step, now_min_curr);
            if (actm_me.now_stage > ACTM_STAGE_NORMAL) {
                set_curr = min(set_curr, get_dt_min_current(actm_me.now_stage - 1));
                if (now_curr == 0 || set_curr < now_curr) {
                    update_ref_temp_flag = true;
                    set_actm_current(set_curr, false);
                }
            } else if (actm_me.now_stage == ACTM_STAGE_NORMAL) {
                if (now_curr == 0 || set_curr != now_curr) {
                    update_ref_temp_flag = true;
                    set_actm_current(set_curr, false);
                }
            }
            break;
    }

    if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_5W
        || actm_me.conn == CHARGING_SUPPLY_WIRELESS_9W
        || actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W) {
        if (actm_me.get_actm_param(VENEER_FEED_IDC, &idc)) {
            pr_actm(ERROR,
                "get_actm_param(id=%d) failure, error\n", VENEER_FEED_IDC);
            return -1;
        }

        if (actm_me.get_actm_param(VENEER_FEED_VDC, &vdc)) {
            pr_actm(ERROR,
                "get_actm_param(id=%d) failure, error\n", VENEER_FEED_VDC);
            return -1;
        }

        set_curr = idc * vdc / 1000;
    }

    pr_actm(UPDATE,
        "[%s, %s(%d~%d, %d), %s] "
        "from %d(ref=%d) to %d(now=%d, old=%d), step: %d, hold: %d, cold: %d\n",
        str_mode[get_actm_mode() + 2],
        str_stage[(actm_me.now_stage + 2)],
        get_dt_temp(dt, actm_me.now_stage),
        get_dt_temp(dt, min(actm_me.now_stage + 1, ACTM_STAGE_HOT)),
        now_min_curr, str_policy[policy],
        now_curr, actm_me.ref_temp, set_curr, temp,
        actm_me.old_temp, curr_step, actm_me.hold_count, actm_me.cold_count);

    if (update_ref_temp_flag)
        actm_me.ref_temp = temp;

    return rc;
}

static bool actm_is_sidecar_triggered(void)
{
    bool lcdon = is_lcdon();
    int actm_mode = get_actm_mode(), conn_type = WIRED;

    if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_5W
        || actm_me.conn == CHARGING_SUPPLY_WIRELESS_9W
        || actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W)
        conn_type = WIRELESS;

    if (actm_mode > ACTM_MODE_DISABLE && actm_me.dt[conn_type].sidecar[actm_mode].is_on[lcdon])
        return true;

    return false;
}

static bool actm_sidecar_work(struct _actm_dt *dt, int temp)
{
    bool en = false;
    bool lcdon = is_lcdon();
    int actm_mode = get_actm_mode(), conn_type = WIRED;
    const char str_mode[5][15] =
        {"thermal first", "balanced", "charging first"};

    if (actm_mode > ACTM_MODE_CHARGING || actm_mode < ACTM_MODE_THERMAL)
        return false;

    if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_5W
        || actm_me.conn == CHARGING_SUPPLY_WIRELESS_9W
        || actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W)
        conn_type = WIRELESS;

    en = dt->sidecar[actm_mode].is_on[lcdon];
    if (dt->sidecar[actm_mode].enable) {
        if (en) {
            if (temp < dt->sidecar[actm_mode].exit_temp[lcdon])
                en = false;
        } else {
            if (temp > dt->sidecar[actm_mode].enter_temp[lcdon])
                en = true;
        }
    }

    if (dt->sidecar[actm_mode].is_on[lcdon] != en) {
        pr_actm(UPDATE,
            " >>> %s SIDECAR(lcd=%s). type=%s, "
            "temp: %d(in:%d, out:%d), curr: %dmA, mode: %s <<<\n",
            en ? "enter to" : "exit from", lcdon ? "on":"off",
            conn_type ? "wireless" : "wired",
            temp, dt->sidecar[actm_mode].enter_temp[lcdon], dt->sidecar[actm_mode].exit_temp[lcdon],
            dt->sidecar[actm_mode].min_curr[lcdon], str_mode[actm_mode]);
    }

    if (en) {
        set_actm_current(dt->sidecar[actm_mode].min_curr[lcdon], true);
    }
    else {
        if (conn_type == WIRED)
            veneer_voter_release(&actm_me.sidecar_ibat);
        else
            veneer_voter_release(&actm_me.sidecar_idc);
    }

    dt->sidecar[actm_mode].is_on[lcdon] = en;
    return en;
}

static int actm_job(struct _actm_dt *dt, int temp)
{
    int rc = 0;
    int now_stage = ACTM_STAGE_NONE;
    int param_temp = 0;

    if (actm_sidecar_work(dt, temp)) {
        return 0;
    }

    now_stage = find_now_stage(dt, temp);
    set_actm_stage(dt, now_stage, temp);

    if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W) {
        actm_me.set_actm_param(
                VENEER_FEED_BATT_PROFILE_FCC_VOTER, ACTM_15W_WLC_MAX_FCC);
    } else {
        /* set initial batt profile voter */
        actm_me.get_actm_param(
            VENEER_FEED_BATT_PROFILE_FCC_VOTER, &param_temp);
        if (param_temp > 0 &&
            param_temp != actm_me.dt->wired_max_fcc[actm_me.cp_type])
            actm_me.set_actm_param(
                VENEER_FEED_BATT_PROFILE_FCC_VOTER,
                actm_me.dt->wired_max_fcc[actm_me.cp_type]);
    }

    if (now_stage < ACTM_STAGE_NORMAL) {
        if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W)
            actm_me.set_actm_param(VENEER_FEED_VDC, actm_me.max_volt);
        else if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_9W)
            actm_me.set_actm_param(VENEER_FEED_VDC, actm_me.epp_volt);
        else if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_5W)
            actm_me.set_actm_param(VENEER_FEED_VDC, actm_me.bpp_volt);

        if (get_actm_mode() > ACTM_MODE_DISABLE && dt->max_power_15w[get_actm_mode()] > 0 &&
            actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W) {
            set_actm_current(dt->max_power_15w[get_actm_mode()], false);
        } else {
            veneer_voter_release(&actm_me.idc);
        }

        veneer_voter_release(&actm_me.ibat);

        actm_me.ref_temp = temp;
        return 0;
    }

    rc = actm_act(dt, get_actm_policy(dt, temp), temp);
    actm_me.old_temp = temp;

    return rc;
}

static void update_actm_param(int conn)
{
    int param_temp = 0;

    actm_me.cp_type = CP_NONE;
    if (conn == WIRED) {
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_SENSOR_WIRED, &param_temp);
        if (param_temp ==  1 || param_temp ==  2 || param_temp ==  3 ||
            param_temp == 11 || param_temp == 12 || param_temp == 13 ||
            param_temp == 21 || param_temp == 22 || param_temp == 23 ||
            param_temp == 31 || param_temp == 32 || param_temp == 33 )
            actm_me.dt[conn].therm_type = param_temp;
        else
            actm_me.dt[conn].therm_type = 3;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_MAX_HOLD_CRITERIA_WIRED_0, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].max_hold_criteria[0] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_MAX_HOLD_CRITERIA_WIRED_1, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].max_hold_criteria[1] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_MAX_HOLD_CRITERIA_WIRED_2, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].max_hold_criteria[2] = param_temp;

        actm_me.get_actm_param(
            VENEER_FEED_ACTM_TEMPOFFS_WIRED_0, &param_temp);
        if (param_temp > -999 )
            actm_me.dt[conn].temp_offset[0] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_TEMPOFFS_WIRED_1, &param_temp);
        if (param_temp > -999 )
            actm_me.dt[conn].temp_offset[1] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_TEMPOFFS_WIRED_2, &param_temp);
        if (param_temp > -999 )
            actm_me.dt[conn].temp_offset[2] = param_temp;

        actm_me.get_actm_param(
            VENEER_FEED_ACTM_MAX_FCC_PPS, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].wired_max_fcc[CP_PPS] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_MAX_FCC_QC3, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].wired_max_fcc[CP_QC30] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_MAX_FCC_QC2, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].wired_max_fcc[CP_NONE] = param_temp;

        actm_me.get_actm_param(
            VENEER_FEED_ACTM_CURRENT_WIRED_0, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].wired_curr[0] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_CURRENT_WIRED_1, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].wired_curr[1] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_CURRENT_WIRED_2, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].wired_curr[2] = param_temp;

        actm_me.get_actm_param(
            VENEER_FEED_ACTM_CURRENT_CP_PPS, &param_temp);
        actm_me.dt[conn].cp_curr[CP_PPS] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_CURRENT_CP_QC30, &param_temp);
        actm_me.dt[conn].cp_curr[CP_QC30] = param_temp;

        if (actm_me.dt[conn].cp_curr[CP_PPS] > 0)
            actm_me.cp_type = CP_PPS;
        else if (actm_me.dt[conn].cp_curr[CP_QC30] > 0)
            actm_me.cp_type = CP_QC30;
        if (!actm_me.enable_cp_charging)
            actm_me.cp_type = CP_NONE;
    }
    else if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_15W
             || actm_me.conn == CHARGING_SUPPLY_WIRELESS_9W) {
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_SENSOR_WIRELESS, &param_temp);
        if (param_temp ==  1 || param_temp ==  2 || param_temp ==  3 ||
            param_temp == 11 || param_temp == 12 || param_temp == 13 ||
            param_temp == 21 || param_temp == 22 || param_temp == 23 ||
            param_temp == 31 || param_temp == 32 || param_temp == 33 )
            actm_me.dt[conn].therm_type = param_temp;
        else
            actm_me.dt[conn].therm_type = 3;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_MAX_HOLD_CRITERIA_WIRELESS_0, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].max_hold_criteria[0] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_MAX_HOLD_CRITERIA_WIRELESS_1, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].max_hold_criteria[1] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_MAX_HOLD_CRITERIA_WIRELESS_2, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].max_hold_criteria[2] = param_temp;

        actm_me.get_actm_param(
            VENEER_FEED_ACTM_TEMPOFFS_WIRELESS_0, &param_temp);
        if (param_temp > -999 )
            actm_me.dt[conn].temp_offset[0] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_TEMPOFFS_WIRELESS_1, &param_temp);
        if (param_temp > -999 )
            actm_me.dt[conn].temp_offset[1] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_TEMPOFFS_WIRELESS_2, &param_temp);
        if (param_temp > -999 )
            actm_me.dt[conn].temp_offset[2] = param_temp;

        actm_me.get_actm_param(
            VENEER_FEED_ACTM_CURRENT_EPP_0, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].epp_pwr[0] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_CURRENT_EPP_1, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].epp_pwr[1] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_CURRENT_EPP_2, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].epp_pwr[2] = param_temp;
    }
    else if (actm_me.conn == CHARGING_SUPPLY_WIRELESS_5W) {
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_MAX_HOLD_CRITERIA_WIRELESS_0, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].max_hold_criteria[0] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_MAX_HOLD_CRITERIA_WIRELESS_1, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].max_hold_criteria[1] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_MAX_HOLD_CRITERIA_WIRELESS_2, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].max_hold_criteria[2] = param_temp;

        actm_me.get_actm_param(
            VENEER_FEED_ACTM_TEMPOFFS_WIRELESS_0, &param_temp);
        if (param_temp > -999 )
            actm_me.dt[conn].temp_offset[0] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_TEMPOFFS_WIRELESS_1, &param_temp);
        if (param_temp > -999 )
            actm_me.dt[conn].temp_offset[1] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_TEMPOFFS_WIRELESS_2, &param_temp);
        if (param_temp > -999 )
            actm_me.dt[conn].temp_offset[2] = param_temp;

        actm_me.get_actm_param(
            VENEER_FEED_ACTM_CURRENT_BPP_0, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].bpp_pwr[0] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_CURRENT_BPP_1, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].bpp_pwr[1] = param_temp;
        actm_me.get_actm_param(
            VENEER_FEED_ACTM_CURRENT_BPP_2, &param_temp);
        if (param_temp > 0 )
            actm_me.dt[conn].bpp_pwr[2] = param_temp;
    }
}

static int actm_temp_sensor(int conn, int *temp_in, int *temp_out)
{
    int therm_type = 0;

    if (is_lcdon())
        therm_type = actm_me.dt[conn].therm_type / 10; /* lcd on  */
    else
        therm_type = actm_me.dt[conn].therm_type % 10; /* lcd off */

    if (therm_type <= 0)
        therm_type = actm_me.dt[conn].therm_type % 10;

    if (therm_type & ACTM_SENSOR_BATT)
        temp_out[BATT] = temp_in[BATT];
    if (therm_type & ACTM_SENSOR_VTS)
        temp_out[VTS] = temp_in[VTS];

    return therm_type;
}

static void actm_status_work(struct work_struct* work)
{
    int temp[2] = {ACTM_INITVAL, }, conn = DISCHG;
    int temp_buf[ACTM_THERM_MAX] = {0, };
    int conn_buf = 0, status_buf = 0, mode = 0, now_fcc = 0;
    int therm_sensor = 0;
    int usb_present = 0, wireless_present = 0;

    const char str_cp_type[3][8] =
        {"CP_PPS", "CP_QC3", "CP_NONE"};
    const char str_status[5][8] =
        {"Unknown", "chg", "dischg", "not_chg", "full"};
    const char str_sensor_type[4][8] =
        {"Unknown", "BATT", "VTS", "MAX"};
    const char str_charger[CHARGING_SUPPLY_MAX][13] = {
        "UNKNOWN", "FLOAT", "NONE",
	    "DCP_DEFAULT", "DCP_10K", "DCP_22K", "QC2", "QC3",
	    "USB2", "USB3", "CDP", "USB_PD",
	    "FACTORY_56K", "FACTORY_130K", "FACTORY_910K",
	    "WIRELESS_5W", "WIRELESS_9W", "WIRELESS_15W"
    };

    if (actm_me.get_actm_param(VENEER_FEED_ACTM_MODE, &mode)) {
        pr_actm(ERROR, "get_actm_param(actm mode) failure, error\n");
        actm_init();
        return;
    }
    set_actm_mode(mode);

    // force mode set to charging first during chargerlogo
    if (unified_bootmode_chargerlogo() && actm_me.mode > ACTM_MODE_DISABLE)
        actm_me.mode = ACTM_MODE_CHARGING * 10 + ACTM_MODE_CHARGING;

    actm_me.set_actm_param(VENEER_FEED_ACTM_MODE_NOW, actm_me.mode);

    if (actm_me.mode < ACTM_MODE_THERMAL) {
        pr_actm(MONITOR, "actm is disabled by user.\n");
        if (actm_me.max_volt > 0 )
            actm_me.set_actm_param(VENEER_FEED_VDC, actm_me.max_volt);
        else
            actm_me.set_actm_param(VENEER_FEED_VDC, actm_me.epp_volt);
        veneer_voter_release(&actm_me.idc);
        veneer_voter_release(&actm_me.ibat);
        veneer_voter_release(&actm_me.sidecar_idc);
        veneer_voter_release(&actm_me.sidecar_ibat);
        actm_init();
        return;
    }

    if (actm_me.get_actm_param(VENEER_FEED_STATUS_RAW, &status_buf)) {
        pr_actm(ERROR, "get_actm_param(status) failure, error\n");
        actm_init();
        return;
    }
    actm_me.status_raw = status_buf;

    if (status_buf != POWER_SUPPLY_STATUS_CHARGING) {
        actm_me.get_actm_param(VENEER_FEED_USB_PRESENT, &usb_present);
        actm_me.get_actm_param(VENEER_FEED_WIRELESS_PRESENT, &wireless_present);

        if (usb_present || wireless_present) {
            pr_actm(EVALUATE,
                "not charging state...but usb=%d, wireless=%d...5sec delayed.\n",
                usb_present, wireless_present);

            schedule_delayed_work(&actm_me.dwork,
                msecs_to_jiffies(ACTM_PROBATION_INTERVAL));
            return;
        }

        pr_actm(EVALUATE,
            "exit..not charging state..%s\n", str_status[status_buf]);
        if (actm_me.max_volt > 0)
            actm_me.set_actm_param(VENEER_FEED_VDC, actm_me.max_volt);
        else
            actm_me.set_actm_param(VENEER_FEED_VDC, actm_me.epp_volt);
        veneer_voter_release(&actm_me.idc);
        veneer_voter_release(&actm_me.ibat);
        veneer_voter_release(&actm_me.sidecar_idc);
        veneer_voter_release(&actm_me.sidecar_ibat);
        actm_init();
        return;
    }

    if (actm_me.get_actm_param(VENEER_FEED_CHARGER_TYPE, &conn_buf)) {
        pr_actm(ERROR, "get_actm_param(charger type) failure, error\n");
        actm_init();
        return;
    }

    if (actm_me.get_actm_param(VENEER_FEED_SENSOR_BATT, &temp_buf[BATT])) {
        pr_actm(ERROR, "get_actm_param(batt) failure, error\n");
        actm_init();
        return;
    }

    if (actm_me.get_actm_param(VENEER_FEED_SENSOR_VTS, &temp_buf[VTS])) {
        pr_actm(ERROR, "get_actm_param(vts) failure, error\n");
        actm_init();
        return;
    }

    if ((temp_buf[BATT] < 100) && (temp_buf[VTS] < 100)) {
        pr_actm(MONITOR, "low temperature(batt: %d, vts: %d)...skip\n",
            temp_buf[BATT], temp_buf[VTS]);
        actm_init();
        return;
    }

    conn = WIRED;
    if (conn_buf == CHARGING_SUPPLY_WIRELESS_5W
        || conn_buf == CHARGING_SUPPLY_WIRELESS_9W
        || conn_buf == CHARGING_SUPPLY_WIRELESS_15W) {
        conn = WIRELESS;
    }
    else {
        if (actm_me.max_volt > 0)
            actm_me.set_actm_param(VENEER_FEED_VDC, actm_me.max_volt);
        else
            actm_me.set_actm_param(VENEER_FEED_VDC, actm_me.epp_volt);
        veneer_voter_release(&actm_me.idc);
        veneer_voter_release(&actm_me.sidecar_idc);
    }

    actm_me.conn = conn_buf;

    update_actm_param(conn);
    therm_sensor = actm_temp_sensor(conn, temp_buf, temp);

    pr_actm(UPDATE,
        ">>>>> charger: %s(%s, %s), type:%s, Temp:%d (batt:%d, vts:%d) <<<<<\n",
        str_charger[conn_buf], str_cp_type[actm_me.cp_type],
        (status_buf == POWER_SUPPLY_STATUS_CHARGING) ? "CHG" : "DISCHG",
        str_sensor_type[therm_sensor],
        max(temp[BATT], temp[VTS]), temp_buf[BATT], temp_buf[VTS]);

    if (actm_me.active == false) {
        actm_me.active = true;
        schedule_delayed_work(
            &actm_me.dwork, msecs_to_jiffies(ACTM_PROBATION_INTERVAL));
        return;
    }

    if (!actm_job(&actm_me.dt[conn], max(temp[BATT], temp[VTS]))) {
        actm_me.now_timer = actm_me.dt[conn].timer;
        if (conn == WIRED) {
            actm_me.get_actm_param(VENEER_FEED_FCC, &now_fcc);
            if (now_fcc >= ACTM_MIN_INTERVAL_FCC)
                actm_me.now_timer = ACTM_MIN_INTERVAL;
            else if (now_fcc > ACTM_MID_INTERVAL_FCC)
                actm_me.now_timer = ACTM_MID_INTERVAL;
        }

        if (actm_me.now_stage >= ACTM_STAGE_NORMAL)
            actm_me.now_timer =
                min(actm_me.now_timer,
                    actm_me.dt[conn].timer / (actm_me.now_stage + 1));

        if (actm_is_sidecar_triggered())
            actm_me.now_timer = ACTM_MIN_INTERVAL;

        schedule_delayed_work(
            &actm_me.dwork, msecs_to_jiffies(actm_me.now_timer));
    }
}

void actm_trigger(void)
{
    int status_buf = 0;

    if (actm_me.mode == ACTM_MODE_NOT_SUPPORT)
        return;

    if (unified_bootmode_chargerlogo() && !actm_me.enable_on_chargerlogo)
        return;

    if (!actm_me.active) {
        cancel_delayed_work(&actm_me.dwork);
        schedule_delayed_work(
            &actm_me.dwork, msecs_to_jiffies(0));
    }
    else {
        if (actm_me.get_actm_param(VENEER_FEED_STATUS_RAW, &status_buf)) {
            pr_actm(ERROR, "get_actm_param(status) failure, error\n");
            actm_init();
            return;
        }
        if (actm_me.status_raw != status_buf) {
            cancel_delayed_work(&actm_me.dwork);
            schedule_delayed_work(
                &actm_me.dwork, msecs_to_jiffies(0));
            actm_me.status_raw = status_buf;
        }
    }
}

static bool actm_parse_dt(struct device_node* dnode)
{
    int rc = 0;
    int temp[3] = {0, };
    struct device_node* d_idtp9222 =
        of_find_node_by_name(NULL, "idtp9222-charger");

    actm_me.enable =
        of_property_read_bool(dnode, "lge,actm-enable");
    actm_me.enable_cp_charging =
        of_property_read_bool(dnode, "lge,actm-enable-cp-charging");
    actm_me.enable_on_chargerlogo =
        of_property_read_bool(dnode, "lge,actm-enable-on-chargerlogo");

    rc |= of_property_read_u32_array(dnode, "lge,actm-auto-mode-soc",
        actm_me.auto_mode_soc, AUTO_MODE_SOC_SIZE);
    if (rc)
        pr_actm(ERROR, "lge,actm-auto-mode-soc failure, error. %d\n", rc);

    rc |= of_property_read_u32_array(dnode, "lge,actm-auto-mode-config",
        actm_me.auto_mode_config, AUTO_MODE_CONFIG_SIZE);
    if (rc)
        pr_actm(ERROR, "lge,actm-auto-mode-config failure, error. %d\n", rc);

    actm_me.max_volt = 0;
    if (d_idtp9222) {
        of_property_read_u32(d_idtp9222, "idt,configure-bppvolt", &actm_me.bpp_volt);
        of_property_read_u32(d_idtp9222, "idt,configure-eppvolt", &actm_me.epp_volt);
        of_property_read_u32(d_idtp9222, "idt,configure-maxvolt", &actm_me.max_volt);
        of_property_read_u32(d_idtp9222, "idt,configure-bppcurr", &actm_me.bpp_curr);
        of_property_read_u32(d_idtp9222, "idt,configure-eppcurr", &actm_me.epp_curr);
        of_property_read_u32(d_idtp9222, "idt,configure-maxcurr", &actm_me.max_curr);
        actm_me.bpp_volt /= 1000;
        actm_me.bpp_curr /= 1000;
        actm_me.epp_volt /= 1000;
        actm_me.epp_curr /= 1000;
        if (actm_me.max_volt > 0)
            actm_me.max_volt /= 1000;
        if (actm_me.max_curr > 0)
            actm_me.max_curr /= 1000;
    }
    else {
        actm_me.bpp_volt = 6000;
        actm_me.bpp_curr = 750;
        actm_me.epp_volt = 9000;
        actm_me.epp_curr = 800;
        actm_me.max_volt = 0;
        actm_me.max_curr = 0;
        pr_actm(ERROR, "idt,configure-bppvolt dtsi error -> set default(0V, 9V, 6V)\n");
    }

    rc |= of_property_read_u32(dnode,
        "lge,actm-lcdon-temp-offset", &actm_me.lcdon_temp_offset);
    if (rc)
        pr_actm(ERROR, "lge,actm-lcdon-temp-offset failure, error. %d\n", rc);

    pr_actm(UPDATE,
        "enable: %d, chargerlogo: %d, auto-mode-soc: %d, %d (%d, %d, %d), "
        "idtp9222 default: max=(%dmV,%dmA), epp=(%dmV,%dmA), bpp=(%dmV,%dmA), lcdon-temp-offset: %d\n",
        actm_me.enable,
        actm_me.enable_on_chargerlogo,
        actm_me.auto_mode_soc[0],
        actm_me.auto_mode_soc[1],
        actm_me.auto_mode_config[0],
        actm_me.auto_mode_config[1],
        actm_me.auto_mode_config[2],
        actm_me.max_volt, actm_me.max_curr,
        actm_me.epp_volt, actm_me.epp_curr,
        actm_me.bpp_volt, actm_me.bpp_curr,
        actm_me.lcdon_temp_offset);

    if (actm_me.enable) {
        rc |= of_property_read_u32(dnode,
		    "lge,wired-therm-sensor-type", &actm_me.dt[WIRED].therm_type);
        if (rc)
            pr_actm(ERROR,
                "lge,wired-therm-sensor-type failure, error. %d\n", rc);

        rc |= of_property_read_u32(dnode,
		    "lge,wired-wton-backward-margin", &actm_me.dt[WIRED].warm_to_normal_margin);
        if (rc)
            pr_actm(ERROR,
                "lge,wired-wton-backward-offset failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wired-temp-offset",
            actm_me.dt[WIRED].temp_offset, ACTM_MODE_MAX);
        if (rc)
            pr_actm(ERROR, "lge,wired-temp-offset failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
            "lge,wired-max-hold-criteria",
            actm_me.dt[WIRED].max_hold_criteria, AUTO_MODE_CONFIG_SIZE);
        if (rc)
            pr_actm(ERROR,
                "lge,wired-max-hold-criteria failure, error. %d\n", rc);

        rc |= of_property_read_u32(dnode,
		    "lge,wired-stage-size", &actm_me.dt[WIRED].stage_size);
        if (rc)
            pr_actm(ERROR,
                "lge,wired-stage-size failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wired-target-temp",
		    actm_me.dt[WIRED].temp_criteria, actm_me.dt[WIRED].stage_size);
        if (rc)
            pr_actm(ERROR,
                "lge,wired-target-temp failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wired-current-limit-ma",
		    actm_me.dt[WIRED].wired_curr, actm_me.dt[WIRED].stage_size);
        if (rc)
            pr_actm(ERROR,
                "lge,wired-current-limit-ma failure, error. %d\n", rc);

        if (actm_me.enable_cp_charging) {
            rc |= of_property_read_u32_array(dnode,
		        "lge,wired-curr-cp-limit-ma",
                actm_me.dt[WIRED].cp_curr, 2);
            if (rc)
                pr_actm(ERROR,
                    "lge,lge,wired-curr-cp-limit-ma failure, error. %d\n", rc);
        }

        rc |= of_property_read_u32_array(dnode,
		    "lge,wired-max-fcc-ma",
		    actm_me.dt[WIRED].wired_max_fcc, 3);
        if (rc)
            pr_actm(ERROR,
                "lge,wired-max-fcc-ma failure, error. %d\n", rc);

        rc |= of_property_read_u32(dnode,
		    "lge,wired-current-step-size", &actm_me.dt[WIRED].curr_step_size);
        if (rc)
            pr_actm(ERROR,
                "lge,wired-current-step-size failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wired-current-step-temp",
            actm_me.dt[WIRED].curr_step_temp, actm_me.dt[WIRED].curr_step_size);
        if (rc)
            pr_actm(ERROR, "lge,wired-current-step-temp failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wired-current-step-ma",
            actm_me.dt[WIRED].curr_step_ma, actm_me.dt[WIRED].curr_step_size);
        if (rc)
            pr_actm(ERROR, "lge,wired-current-step-ma failure, error. %d\n", rc);

        rc |= of_property_read_u32(dnode,
		    "lge,wired-timer-ms", &actm_me.dt[WIRED].timer);
        if (rc)
            pr_actm(ERROR,
                "lge,wired-timer-ms failure, error. %d\n", rc);
        if (actm_me.dt[WIRED].timer < 30)
            actm_me.dt[WIRED].timer = 30;

        pr_actm(MONITOR,
            "[wired] type=%d, wton_margin=%d, timer=%d, hold=(%d, %d, %d), max_fcc=(%d, %d, %d), "
            "temp offset: %d, %d, %d, (cp_curr: %d, %d), (curr %d: %d, %d), (curr %d: %d, %d), (curr %d: %d, %d), "
            "(stage %d: %d, %d), (stage %d: %d, %d), (stage %d: %d, %d)",
            actm_me.dt[0].therm_type,
            actm_me.dt[0].warm_to_normal_margin,
            actm_me.dt[0].timer,
            actm_me.dt[0].max_hold_criteria[0],
            actm_me.dt[0].max_hold_criteria[1],
            actm_me.dt[0].max_hold_criteria[2],
            actm_me.dt[0].wired_max_fcc[0],
            actm_me.dt[0].wired_max_fcc[1],
            actm_me.dt[0].wired_max_fcc[2],
            actm_me.dt[0].temp_offset[0],
            actm_me.dt[0].temp_offset[1],
            actm_me.dt[0].temp_offset[2],
            actm_me.dt[0].cp_curr[0],
            actm_me.dt[0].cp_curr[1],
            0, actm_me.dt[0].curr_step_temp[0], actm_me.dt[0].curr_step_ma[0],
            1, actm_me.dt[0].curr_step_temp[1], actm_me.dt[0].curr_step_ma[1],
            2, actm_me.dt[0].curr_step_temp[2], actm_me.dt[0].curr_step_ma[2],
            0, actm_me.dt[0].temp_criteria[0], actm_me.dt[0].wired_curr[0],
            1, actm_me.dt[0].temp_criteria[1], actm_me.dt[0].wired_curr[1],
            2, actm_me.dt[0].temp_criteria[2], actm_me.dt[0].wired_curr[2]);

        /* sidecar */
        rc |= of_property_read_u32_array(dnode, "lge,wired-sidecar-enable",
                temp, ACTM_MODE_MAX);
        if (rc) {
            actm_me.dt[0].sidecar[ACTM_MODE_THERMAL].enable = 0;
            actm_me.dt[0].sidecar[ACTM_MODE_BALANCE].enable = 0;
            actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].enable = 0;
            pr_actm(ERROR, "lge,wired-sidecar-enable failure, error. %d\n", rc);
        } else {
            actm_me.dt[0].sidecar[ACTM_MODE_THERMAL].enable = !!temp[0];
            actm_me.dt[0].sidecar[ACTM_MODE_BALANCE].enable = !!temp[1];
            actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].enable = !!temp[2];
        }

        if (actm_me.dt[0].sidecar[ACTM_MODE_THERMAL].enable) {
            actm_me.dt[0].sidecar[ACTM_MODE_THERMAL].is_on[LCD_OFF] = false;
            actm_me.dt[0].sidecar[ACTM_MODE_THERMAL].is_on[LCD_ON] = false;
            rc |= of_property_read_u32_array(dnode, "lge,wired-sidecar-therm-enter-temp",
                    actm_me.dt[0].sidecar[ACTM_MODE_THERMAL].enter_temp, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wired-sidecar-therm-enter-temp failure, error. %d\n", rc);

            rc |= of_property_read_u32_array(dnode, "lge,wired-sidecar-therm-exit-temp",
                    actm_me.dt[0].sidecar[ACTM_MODE_THERMAL].exit_temp, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wired-sidecar-therm-exit-temp failure, error. %d\n", rc);

            rc |= of_property_read_u32_array(dnode, "lge,wired-sidecar-therm-min-current",
                    actm_me.dt[0].sidecar[ACTM_MODE_THERMAL].min_curr, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wired-sidecar-therm-min-current failure, error. %d\n", rc);
        }

        if (actm_me.dt[0].sidecar[ACTM_MODE_BALANCE].enable) {
            actm_me.dt[0].sidecar[ACTM_MODE_BALANCE].is_on[LCD_OFF] = false;
            actm_me.dt[0].sidecar[ACTM_MODE_BALANCE].is_on[LCD_ON] = false;
            rc |= of_property_read_u32_array(dnode, "lge,wired-sidecar-bal-enter-temp",
                    actm_me.dt[0].sidecar[ACTM_MODE_BALANCE].enter_temp, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wired-sidecar-bal-enter-temp failure, error. %d\n", rc);

            rc |= of_property_read_u32_array(dnode, "lge,wired-sidecar-bal-exit-temp",
                    actm_me.dt[0].sidecar[ACTM_MODE_BALANCE].exit_temp, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wired-sidecar-bal-exit-temp failure, error. %d\n", rc);

            rc |= of_property_read_u32_array(dnode, "lge,wired-sidecar-bal-min-current",
                    actm_me.dt[0].sidecar[ACTM_MODE_BALANCE].min_curr, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wired-sidecar-bal-min-current failure, error. %d\n", rc);
        }

        if (actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].enable) {
            actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].is_on[LCD_OFF] = false;
            actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].is_on[LCD_ON] = false;
            rc |= of_property_read_u32_array(dnode, "lge,wired-sidecar-chg-enter-temp",
                    actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].enter_temp, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wired-sidecar-chg-enter-temp failure, error. %d\n", rc);

            rc |= of_property_read_u32_array(dnode, "lge,wired-sidecar-chg-exit-temp",
                    actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].exit_temp, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wired-sidecar-chg-exit-temp failure, error. %d\n", rc);

            rc |= of_property_read_u32_array(dnode, "lge,wired-sidecar-chg-min-current",
                    actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].min_curr, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wired-sidecar-chg-min-current failure, error. %d\n", rc);
        }

        pr_actm(UPDATE,
            "[wired sidecar] enable: %d/%d/%d, lcd off=(%d/%d, %dmA), lcd on=(%d/%d, %dmA)\n",
            actm_me.dt[0].sidecar[ACTM_MODE_THERMAL].enable,
            actm_me.dt[0].sidecar[ACTM_MODE_BALANCE].enable,
            actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].enable,
            actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].enter_temp[LCD_OFF],
            actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].exit_temp[LCD_OFF],
            actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].min_curr[LCD_OFF],
            actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].enter_temp[LCD_ON],
            actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].exit_temp[LCD_ON],
            actm_me.dt[0].sidecar[ACTM_MODE_CHARGING].min_curr[LCD_ON]);

        rc |= of_property_read_u32(dnode,
		"lge,wireless-therm-sensor-type", &actm_me.dt[WIRELESS].therm_type);
        if (rc)
            pr_actm(ERROR,
                "lge,wireless-therm-sensor-type failure, error. %d\n", rc);

        rc |= of_property_read_u32(dnode,
		    "lge,wireless-wton-backward-margin", &actm_me.dt[WIRELESS].warm_to_normal_margin);
        if (rc)
            pr_actm(ERROR,
                "lge,wireless-wton-backward-offset failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wireless-temp-offset",
            actm_me.dt[WIRELESS].temp_offset, ACTM_MODE_MAX);
        if (rc)
            pr_actm(ERROR, "lge,wireless-temp-offset failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wireless-max-hold-criteria",
            actm_me.dt[WIRELESS].max_hold_criteria, AUTO_MODE_CONFIG_SIZE);
        if (rc)
            pr_actm(ERROR,
                "lge,wireless-max-hold-criteria failure, error. %d\n", rc);

        rc |= of_property_read_u32(dnode,
		    "lge,wireless-stage-size", &actm_me.dt[WIRELESS].stage_size);
        if (rc)
            pr_actm(ERROR,
                "lge,wireless-stage-size failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wireless-target-temp",
		    actm_me.dt[WIRELESS].temp_criteria, actm_me.dt[WIRELESS].stage_size);
        if (rc)
            pr_actm(ERROR,
                "lge,wireless-target-temp failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wireless-epp-power-limit-mw",
		    actm_me.dt[WIRELESS].epp_pwr, actm_me.dt[WIRELESS].stage_size);
        if (rc)
            pr_actm(ERROR,
                "lge,wireless-epp-power-limit-mw failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wireless-bpp-power-limit-mw",
		    actm_me.dt[WIRELESS].bpp_pwr, actm_me.dt[WIRELESS].stage_size);
        if (rc)
            pr_actm(ERROR,
                "lge,wireless-bpp-power-limit-mw failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wireless-15w-max-power-mw",
		    actm_me.dt[WIRELESS].max_power_15w, ACTM_MODE_MAX);
        if (rc)
            pr_actm(ERROR,
                "lge,wireless-15w-max-power-mw failure, error. %d\n", rc);

        rc |= of_property_read_u32(dnode,
		    "lge,wireless-power-step-size", &actm_me.dt[WIRELESS].curr_step_size);
        if (rc)
            pr_actm(ERROR,
                "lge,wireless-power-step-size failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wireless-power-step-temp",
            actm_me.dt[WIRELESS].curr_step_temp, actm_me.dt[WIRELESS].curr_step_size);
        if (rc)
            pr_actm(ERROR, "lge,wireless-power-step-temp failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wireless-epp-power-step-mw",
            actm_me.dt[WIRELESS].epp_pwr_step_mw , actm_me.dt[WIRELESS].curr_step_size);
        if (rc)
            pr_actm(ERROR,
                "lge,wireless-epp-power-step-mw failure, error. %d\n", rc);

        rc |= of_property_read_u32_array(dnode,
		    "lge,wireless-bpp-power-step-mw",
            actm_me.dt[WIRELESS].bpp_pwr_step_mw , actm_me.dt[WIRELESS].curr_step_size);
        if (rc)
            pr_actm(ERROR,
                "lge,wireless-bpp-power-step-mw failure, error. %d\n", rc);

        rc |= of_property_read_u32(dnode,
		    "lge,wireless-timer-ms", &actm_me.dt[WIRELESS].timer);
        if (rc)
            pr_actm(ERROR,
                "lge,wireless-timer-ms failure, error. %d\n", rc);
        if (actm_me.dt[WIRELESS].timer < 30)
            actm_me.dt[WIRELESS].timer = 30;

        pr_actm(MONITOR,
            "[wireless] type=%d, wton_margin=%d, timer=%d, hold=(%d, %d, %d), stage=%d, current step=%d, "
            "temp offset: %d, %d, %d, (curr %d: %d, %d, %d), (curr %d: %d, %d, %d), (curr %d: %d, %d, %d), "
            "(stage %d: %d, %d, %d), (stage %d: %d, %d, %d), (stage %d: %d, %d, %d)",
            actm_me.dt[1].therm_type,
            actm_me.dt[1].warm_to_normal_margin,
            actm_me.dt[1].timer,
            actm_me.dt[1].max_hold_criteria[0],
            actm_me.dt[1].max_hold_criteria[1],
            actm_me.dt[1].max_hold_criteria[2],
            actm_me.dt[1].stage_size, actm_me.dt[1].curr_step_size,
            actm_me.dt[1].temp_offset[0],
            actm_me.dt[1].temp_offset[1],
            actm_me.dt[1].temp_offset[2],
            0, actm_me.dt[1].curr_step_temp[0], actm_me.dt[1].bpp_pwr_step_mw[0], actm_me.dt[1].epp_pwr_step_mw[0],
            1, actm_me.dt[1].curr_step_temp[1], actm_me.dt[1].bpp_pwr_step_mw[1], actm_me.dt[1].epp_pwr_step_mw[1],
            2, actm_me.dt[1].curr_step_temp[2], actm_me.dt[1].bpp_pwr_step_mw[2], actm_me.dt[1].epp_pwr_step_mw[2],
            0, actm_me.dt[1].temp_criteria[0], actm_me.dt[1].epp_pwr[0], actm_me.dt[1].bpp_pwr[0],
            1, actm_me.dt[1].temp_criteria[1], actm_me.dt[1].epp_pwr[1], actm_me.dt[1].bpp_pwr[1],
            2, actm_me.dt[1].temp_criteria[2], actm_me.dt[1].epp_pwr[2], actm_me.dt[1].bpp_pwr[2]);

        /* sidecar */
        rc |= of_property_read_u32_array(dnode, "lge,wireless-sidecar-enable",
                temp, ACTM_MODE_MAX);
        if (rc) {
            actm_me.dt[1].sidecar[ACTM_MODE_THERMAL].enable = 0;
            actm_me.dt[1].sidecar[ACTM_MODE_BALANCE].enable = 0;
            actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].enable = 0;
            pr_actm(ERROR, "lge,wireless-sidecar-enable failure, error. %d\n", rc);
        } else {
            actm_me.dt[1].sidecar[ACTM_MODE_THERMAL].enable = !!temp[0];
            actm_me.dt[1].sidecar[ACTM_MODE_BALANCE].enable = !!temp[1];
            actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].enable = !!temp[2];
        }

        if (actm_me.dt[1].sidecar[ACTM_MODE_THERMAL].enable) {
            actm_me.dt[1].sidecar[ACTM_MODE_THERMAL].is_on[LCD_OFF] = false;
            actm_me.dt[1].sidecar[ACTM_MODE_THERMAL].is_on[LCD_ON] = false;
            rc |= of_property_read_u32_array(dnode, "lge,wireless-sidecar-therm-enter-temp",
                    actm_me.dt[1].sidecar[ACTM_MODE_THERMAL].enter_temp, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wireless-sidecar-therm-enter-temp failure, error. %d\n", rc);

            rc |= of_property_read_u32_array(dnode, "lge,wireless-sidecar-therm-exit-temp",
                    actm_me.dt[1].sidecar[ACTM_MODE_THERMAL].exit_temp, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wireless-sidecar-therm-exit-temp failure, error. %d\n", rc);

            rc |= of_property_read_u32_array(dnode, "lge,wireless-sidecar-therm-min-current",
                    actm_me.dt[1].sidecar[ACTM_MODE_THERMAL].min_curr, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wireless-sidecar-therm-min-current failure, error. %d\n", rc);
        }

        if (actm_me.dt[1].sidecar[ACTM_MODE_BALANCE].enable) {
            actm_me.dt[1].sidecar[ACTM_MODE_BALANCE].is_on[LCD_OFF] = false;
            actm_me.dt[1].sidecar[ACTM_MODE_BALANCE].is_on[LCD_ON] = false;
            rc |= of_property_read_u32_array(dnode, "lge,wireless-sidecar-bal-enter-temp",
                    actm_me.dt[1].sidecar[ACTM_MODE_BALANCE].enter_temp, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wireless-sidecar-bal-enter-temp failure, error. %d\n", rc);

            rc |= of_property_read_u32_array(dnode, "lge,wireless-sidecar-bal-exit-temp",
                    actm_me.dt[1].sidecar[ACTM_MODE_BALANCE].exit_temp, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wireless-sidecar-bal-exit-temp failure, error. %d\n", rc);

            rc |= of_property_read_u32_array(dnode, "lge,wireless-sidecar-bal-min-current",
                    actm_me.dt[1].sidecar[ACTM_MODE_BALANCE].min_curr, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wireless-sidecar-bal-min-current failure, error. %d\n", rc);
        }

        if (actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].enable) {
            actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].is_on[LCD_OFF] = false;
            actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].is_on[LCD_ON] = false;
            rc |= of_property_read_u32_array(dnode, "lge,wireless-sidecar-chg-enter-temp",
                    actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].enter_temp, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wireless-sidecar-chg-enter-temp failure, error. %d\n", rc);

            rc |= of_property_read_u32_array(dnode, "lge,wireless-sidecar-chg-exit-temp",
                    actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].exit_temp, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wireless-sidecar-chg-exit-temp failure, error. %d\n", rc);

            rc |= of_property_read_u32_array(dnode, "lge,wireless-sidecar-chg-min-power",
                    actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].min_curr, ACTM_LCD_MAX);
            if (rc)
                pr_actm(ERROR, "lge,wireless-sidecar-chg-min-power failure, error. %d\n", rc);
        }

        pr_actm(UPDATE,
            "[wireless sidecar] enable: %d/%d/%d, lcd off=(%d/%d, %dmA), lcd on=(%d/%d, %dmA)\n",
            actm_me.dt[1].sidecar[ACTM_MODE_THERMAL].enable,
            actm_me.dt[1].sidecar[ACTM_MODE_BALANCE].enable,
            actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].enable,
            actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].enter_temp[LCD_OFF],
            actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].exit_temp[LCD_OFF],
            actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].min_curr[LCD_OFF],
            actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].enter_temp[LCD_ON],
            actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].exit_temp[LCD_ON],
            actm_me.dt[1].sidecar[ACTM_MODE_CHARGING].min_curr[LCD_ON]);
    }
    else {
        rc = 0;
        actm_me.mode = ACTM_MODE_NOT_SUPPORT;
    }

    return !rc ? true : false;
}

bool actm_create(
    struct device_node* dnode,
    int (*get_veneer_param)(int id, int *val),
    int (*set_veneer_param)(int id, int val)
){
    bool ret = true;

    ret &= actm_parse_dt(dnode);
    if (!ret)
        pr_actm(ERROR, "actm_parse_dt failure, error\n");

    if (get_veneer_param)
        actm_me.get_actm_param = get_veneer_param;
    else
        ret &= false;
    if (!ret)
        pr_actm(ERROR, "set get_veneer_param failure, error\n");

    if (set_veneer_param)
        actm_me.set_actm_param = set_veneer_param;
    else
        ret &= false;
    if (!ret)
        pr_actm(ERROR, "set set_veneer_param failure, error\n");

    INIT_DELAYED_WORK(&actm_me.dwork, actm_status_work);

    ret &= veneer_voter_register(
            &actm_me.idc, VOTER_NAME_ACTM, VOTER_TYPE_IDC, false);
    if (!ret)
        pr_actm(ERROR, "veneer_voter_register(idc) failure, error\n");

    ret &= veneer_voter_register(
            &actm_me.ibat, VOTER_NAME_ACTM, VOTER_TYPE_IBAT, false);
    if (!ret)
        pr_actm(ERROR, "veneer_voter_register(ibat) failure, error\n");

    ret &= veneer_voter_register(
            &actm_me.sidecar_idc, VOTER_NAME_ACTM_SIDECAR, VOTER_TYPE_IDC, false);
    if (!ret)
        pr_actm(ERROR, "veneer_voter_register(sidecar_idc) failure, error\n");

    ret &= veneer_voter_register(
            &actm_me.sidecar_ibat, VOTER_NAME_ACTM_SIDECAR, VOTER_TYPE_IBAT, false);
    if (!ret)
        pr_actm(ERROR, "veneer_voter_register(sidecar_ibat) failure, error\n");

    actm_init();
    return ret;
}

void actm_destroy(void)
{
    veneer_voter_unregister(&actm_me.idc);
    veneer_voter_unregister(&actm_me.ibat);
    cancel_delayed_work_sync(&actm_me.dwork);
}
