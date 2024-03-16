/*
 * IDTP9222-2 Wireless Power Receiver driver
 *
 * Copyright (C) 2020 LG Electronics, Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

/***********************************************************
            [ Making 'Online' for IDTP9222_2 ]

 +=====================================================
 | +-------------------------------------|-------------
 | |                                     |              IDT_GPIO_PWRDET
 |-+                                     |
 +=======================================|=============
 |  +------------------------------------|-------------
 |  |                                    |              IDT_GPIO_VRECT
 |--+                                    |              (Trigger 0 -> 1)
 +=======================================|=============
 |   +---+   +---+   +---+               |              DC_PROP_PRESENT
 |   |   |   |   |   |   |<--- 5secs --->|              (Trigger 1 -> 0)
 |---+   +---+   +---+   +---------------|-------------
 +=======================================|=============
 |    +-+     +-+     +-+                |
 |    | |     | |     | |                |              DC_PROP_ONLINE
 |----+ +-----+ +-----+ +----------------|-------------
 +==|====================================|=============
 |  |<---           online           --->|<--- offline

 +=====================================================
 | +-------------------------+
 | |                         |                          IDT_GPIO_PWRDET
 |-+                         +------------------------- (Trigger 1 -> 0)
 +===========================|=========================
 |  +-----------------------+|
 |  |                       ||                          IDT_GPIO_VRECT
 |--+                       +|------------------------- (Trigger 0 -> 1)
 +===========================|=========================
 |   +---+   +---+   +---+   |
 |   |   |   |   |   |   |   |                          DC_PROP_PRESENT
 |---+   +---+   +---+   +---|-------------------------
 +===========================|=========================
 |    +-+     +-+     +-+    |
 |    | |     | |     | |    |                          DC_PROP_ONLINE
 |----+ +-----+ +-----+ +----|-------------------------
 +==|========================|=========================
 |  |<---    online     ---> |<---       offline

***********************************************************/

#define pr_fmt(fmt) "IDTP9222-2: %s: " fmt, __func__

#define pr_idt(reason, fmt, ...)				\
do {								\
	if (idtp9222_debug & (reason))				\
		pr_err(fmt, ##__VA_ARGS__);			\
	else							\
		pr_debug(fmt, ##__VA_ARGS__);			\
} while (0)

#define pr_assert(exp)						\
do {								\
	if ((idtp9222_debug & IDT_ASSERT) && !(exp)) {		\
		pr_idt(IDT_ASSERT, "Assertion failed\n");	\
	}							\
} while (0)

#include "idtp9222-2-charger.h"

static int idtp9222_debug = IDT_ASSERT | IDT_ERROR | IDT_INTERRUPT | IDT_MONITOR | IDT_REGISTER | IDT_UPDATE;

static inline const char *idtp9222_modename(enum idtp9222_opmode modetype) {
	switch (modetype) {
	case WPC :
		return "WPC";
	case PMA :
		return "PMA";
	case UNKNOWN :
	default :
		return "UNKNOWN";
	}
}

static inline int idtp9222_read(struct idtp9222_struct *chip, u16 reg, u8 *val)
{
	u8 address[] = {
		reg >> 8,
		reg & 0xff
	};
	struct i2c_msg msgs[] = {
		{
			.addr   = chip->wlc_client->addr,
			.flags  = 0,
			.buf    = address,
			.len    = 2,
		},
		{
			.addr   = chip->wlc_client->addr,
			.flags  = I2C_M_RD,
			.buf    = val,
			.len    = 1,
		}
	};
	int retry, ret = 0;

	mutex_lock(&chip->io_lock);

	for (retry = 0; retry <= I2C_RETRY_COUNT; retry++) {
		if (retry)
			mdelay(I2C_RETRY_DELAY);

		ret = i2c_transfer(chip->wlc_client->adapter, msgs, 2);
		if (ret == 2) {
			mutex_unlock(&chip->io_lock);

			return 0;
		}
	}

	mutex_unlock(&chip->io_lock);

	pr_idt(IDT_ERROR, "failed to read 0x%04x\n", reg);

	return ret < 0 ? ret : -EIO;
}

static inline int idtp9222_write(struct idtp9222_struct *chip, u16 reg, u8 val)
{
	u8 buf[] = {
		reg >> 8,
		reg & 0xff,
		val
	};
	struct i2c_msg msgs[] = {
		{
			.addr   = chip->wlc_client->addr,
			.flags  = 0,
			.buf    = buf,
			.len    = 3,
		},
	};
	int retry, ret = 0;

	mutex_lock(&chip->io_lock);

	for (retry = 0; retry <= I2C_RETRY_COUNT; retry++) {
		if (retry)
			mdelay(I2C_RETRY_DELAY);

		ret = i2c_transfer(chip->wlc_client->adapter, msgs, 1);
		if (ret == 1) {
			mutex_unlock(&chip->io_lock);

			return 0;
		}
	}

	mutex_unlock(&chip->io_lock);

	pr_idt(IDT_ERROR, "failed to write 0x%02x to 0x%04x\n", val, reg);

	return ret < 0 ? ret : -EIO;
}

static inline int idtp9222_read_word(struct idtp9222_struct *chip, u16 reg, u16 *val)
{
	u8 address[] = {
		reg >> 8,
		reg & 0xff
	};
	u8 buf[2] = { 0, 0 };
	struct i2c_msg msgs[] = {
		{
			.addr   = chip->wlc_client->addr,
			.flags  = 0,
			.buf    = address,
			.len    = 2
		},
		{
			.addr   = chip->wlc_client->addr,
			.flags  = I2C_M_RD,
			.buf    = buf,
			.len    = 2,
		}
	};
	int retry, ret = 0;

	mutex_lock(&chip->io_lock);

	for (retry = 0; retry <= I2C_RETRY_COUNT; retry++) {
		if (retry)
			mdelay(I2C_RETRY_DELAY);

		ret = i2c_transfer(chip->wlc_client->adapter, msgs, 2);
		if (ret == 2) {
			mutex_unlock(&chip->io_lock);

			*val = buf[0] | (buf[1] << 8);

			return 0;
		}
	}

	mutex_unlock(&chip->io_lock);

	pr_idt(IDT_ERROR, "failed to read 0x%04x\n", reg);

	return ret < 0 ? ret : -EIO;
}

static inline int idtp9222_write_word(struct idtp9222_struct *chip, u16 reg, u16 val)
{
	u8 buf[] = {
		reg >> 8,
		reg & 0xFF,
		val & 0xFF,
		val >> 8
	};
	struct i2c_msg msgs[] = {
		{
			.addr   = chip->wlc_client->addr,
			.flags  = 0,
			.buf    = buf,
			.len    = 4,
		},
	};
	int retry, ret = 0;

	mutex_lock(&chip->io_lock);

	for (retry = 0; retry <= I2C_RETRY_COUNT; retry++) {
		if (retry)
			mdelay(I2C_RETRY_DELAY);

		ret = i2c_transfer(chip->wlc_client->adapter, msgs, 1);
		if (ret == 1) {
			mutex_unlock(&chip->io_lock);

			return 0;
		}
	}

	mutex_unlock(&chip->io_lock);

	pr_idt(IDT_ERROR, "failed to write 0x%04x to 0x%04x\n", val, reg);

	return ret < 0 ? ret : -EIO;
}

static inline int idtp9222_read_block(struct idtp9222_struct *chip, u16 reg, u8 *buf,
				   unsigned int len)
{
	u8 address[] = {
		reg >> 8,
		reg & 0xff
	};
	struct i2c_msg msgs[] = {
		{
			.addr   = chip->wlc_client->addr,
			.flags  = 0,
			.buf    = address,
			.len    = 2,
		},
		{
			.addr   = chip->wlc_client->addr,
			.flags  = I2C_M_RD,
			.buf    = buf,
			.len    = len,
		}
	};
	int retry, ret = 0;

	mutex_lock(&chip->io_lock);

	for (retry = 0; retry <= I2C_RETRY_COUNT; retry++) {
		if (retry)
			mdelay(I2C_RETRY_DELAY);

		ret = i2c_transfer(chip->wlc_client->adapter, msgs, 2);
		if (ret == 2) {
			mutex_unlock(&chip->io_lock);

			return 0;
		}
	}

	mutex_unlock(&chip->io_lock);

	pr_idt(IDT_ERROR, "failed to write 0x%04x~0x%04x\n", reg, reg + len);

	return ret < 0 ? ret : -EIO;
}

static inline int idtp9222_write_block(struct idtp9222_struct *chip, u16 reg,
				    const u8 *val, unsigned int len)
{
	u8 *buf;
	struct i2c_msg msgs[] = {
		{
			.addr   = chip->wlc_client->addr,
			.flags  = 0,
			.buf    = buf,
			.len    = len + 2,
		},
	};
	int retry, ret = 0;

	buf = (u8 *)kmalloc(len + 2, GFP_KERNEL);
	if (buf == NULL) {
		pr_idt(IDT_ERROR, "failed to alloc memory\n");
		return -ENOMEM;
	}

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;

	memcpy(&buf[2], val, len);

	msgs[0].buf = buf;

	mutex_lock(&chip->io_lock);

	for (retry = 0; retry <= I2C_RETRY_COUNT; retry++) {
		if (retry)
			mdelay(I2C_RETRY_DELAY);

		ret = i2c_transfer(chip->wlc_client->adapter, msgs, 1);
		if (ret == 1) {
			mutex_unlock(&chip->io_lock);

			return 0;
		}
	}

	mutex_unlock(&chip->io_lock);

	kfree(buf);
	pr_idt(IDT_ERROR, "failed to write 0x%04x~0x%04x\n", reg, reg + len);

	return ret < 0 ? ret : -EIO;
}

static bool idtp9222_wakelock_acquire(struct wakeup_source *wakelock) {
	if (!wakelock->active) {
		pr_idt(IDT_INTERRUPT, "Success!\n");
		__pm_stay_awake(wakelock);

		return true;
	}
	return false;
}

static bool idtp9222_wakelock_release(struct wakeup_source *wakelock) {
	if (wakelock->active) {
		pr_idt(IDT_INTERRUPT, "Success!\n");
		__pm_relax(wakelock);

		return true;
	}
	return false;
}


/*
 * IDTP9222 sysfs for debugging
 */
static unsigned int sysfs_i2c_address = -1;
static unsigned int sysfs_i2c_size = 1;

static ssize_t sysfs_address_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size) {

	if (sscanf(buf, "0x%02x", &sysfs_i2c_address) == 1) {
		pr_idt(IDT_VERBOSE, "I2C address 0x%02x is stored\n", sysfs_i2c_address);
	}

	return size;
}
static ssize_t sysfs_address_show(struct device *dev,
	struct device_attribute *attr, char *buffer) {

	if (sysfs_i2c_address != -1)
		return snprintf(buffer, PAGE_SIZE, "Address: 0x%02x", sysfs_i2c_address);
	else
		return snprintf(buffer, PAGE_SIZE, "Address should be set before reading\n");
}
static DEVICE_ATTR(address, S_IWUSR|S_IRUGO, sysfs_address_show, sysfs_address_store);

static ssize_t sysfs_data_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size) {
	struct idtp9222_struct *idtp9222 = dev->platform_data;
	char *temp, *ptr;
	u8 *value;
	int i = 0;

	if (sysfs_i2c_size > 0) {
		temp = (char *)kmalloc(strlen(buf) + 1, GFP_KERNEL);
		if (temp == NULL) {
			pr_idt(IDT_ERROR, "failed to alloc memory\n");
			return size;
		}

		strcpy(temp, buf);

		value = (u8 *)kmalloc(sizeof(u8) * (sysfs_i2c_size), GFP_KERNEL);
		if (value == NULL) {
			pr_idt(IDT_ERROR, "failed to alloc memory\n");
			goto error_temp;
		}

		ptr = strsep(&temp, " ");
		while (ptr != NULL) {
			if (sscanf(ptr, "0x%02x", &value[i]) != 1) {
				pr_idt(IDT_ERROR, "I2C writing value is wrong for 0x%02x\n", sysfs_i2c_address);
				goto error_value;
			}
			i++;
			ptr = strsep(&temp, " ");
		}

		if (idtp9222_write_block(idtp9222, sysfs_i2c_address, value, MIN(i, sysfs_i2c_size)) < 0)
			pr_idt(IDT_ERROR, "I2C write fail for 0x%02x\n", sysfs_i2c_address);
	}

error_value:
	kfree(value);
error_temp:
	kfree(temp);
	return size;
}
static ssize_t sysfs_data_show(struct device *dev,
	struct device_attribute *attr, char *buffer) {
	struct idtp9222_struct *idtp9222 = dev->platform_data;
	ssize_t buf_pos = 0;
	u8 *value;
	int i;

	if (sysfs_i2c_size > 0) {
		value = (u8 *)kmalloc(sizeof(u8) * (sysfs_i2c_size), GFP_KERNEL);
		if (value == NULL) {
			pr_idt(IDT_ERROR, "failed to alloc memory\n");
			return snprintf(buffer, PAGE_SIZE, "failed to alloc memory\n");
		}

		if (sysfs_i2c_address != -1) {
			if (idtp9222_read_block(idtp9222, sysfs_i2c_address, value, sysfs_i2c_size) == 0) {
				for (i = 0; i < sysfs_i2c_size; i++) {
					buf_pos += snprintf(buffer + buf_pos, PAGE_SIZE - buf_pos, "0x%02x ", value[i]);
				}
				buffer[buf_pos] = '\n';
				buf_pos++;
			}
			else {
				buf_pos = snprintf(buffer, PAGE_SIZE, "I2C read fail for 0x%02x\n", sysfs_i2c_address);
			}
		}
		else {
			buf_pos = snprintf(buffer, PAGE_SIZE, "Address should be set before reading\n");
		}
	}
	else {
		return snprintf(buffer, PAGE_SIZE, "I2C size should be set before reading\n");
	}

	kfree(value);
	return buf_pos;
}
static DEVICE_ATTR(data, S_IWUSR|S_IRUGO, sysfs_data_show, sysfs_data_store);

static ssize_t sysfs_size_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size) {

	if (sscanf(buf, "%d", &sysfs_i2c_size) == 1) {
		pr_idt(IDT_VERBOSE, "I2C size '%d' is stored\n", sysfs_i2c_size);
	}

	return size;
}
static ssize_t sysfs_size_show(struct device *dev,
	struct device_attribute *attr, char *buffer) {

	if (sysfs_i2c_size > 0)
		return snprintf(buffer, PAGE_SIZE, "I2C Size: 0x%02x", sysfs_i2c_size);
	else
		return snprintf(buffer, PAGE_SIZE, "I2C size should be set before reading\n");
}
static DEVICE_ATTR(size, S_IWUSR|S_IRUGO, sysfs_size_show, sysfs_size_store);

static struct attribute *idtp9222_sysfs_attrs [] = {
	&dev_attr_address.attr,
	&dev_attr_data.attr,
	&dev_attr_size.attr,
	NULL
};

static const struct attribute_group idtp9222_sysfs_files = {
	.attrs  = idtp9222_sysfs_attrs,
};


/*
 * IDTP9222 I2C getter
 */
static bool idtp9222_is_onpad(struct idtp9222_struct *idtp9222) {
	/* Refer to shadow here,
	 * And be sure that real GPIO may indicate diffrent value of onpad. */
	return idtp9222->status_onpad;
}

static bool idtp9222_is_enabled(struct idtp9222_struct *idtp9222) {
	bool status = !get_effective_result_locked(idtp9222->wlc_disable);

	pr_assert(!gpio_get_value(idtp9222->gpio_disabled)==status);
	return status;
}

static bool idtp9222_is_vrect(struct idtp9222_struct *idtp9222) {
	pr_assert(!gpio_get_value(idtp9222->gpio_vrect)==idtp9222->status_vrect);
	return idtp9222->status_vrect;
}

static bool idtp9222_is_attached(struct idtp9222_struct *idtp9222) {
	bool status = !gpio_get_value(idtp9222->gpio_detached);

	return status;
}

static bool idtp9222_is_full(struct idtp9222_struct *idtp9222) {
	if (idtp9222_is_onpad(idtp9222)) {
		return idtp9222->status_full;
	}
	else {
		pr_idt(IDT_VERBOSE, "idtp9222 is off now\n");
		// The status should be false on offline
		pr_assert(idtp9222->status_full==false);
		return false;
	}
}

static int idtp9222_get_opmode(struct idtp9222_struct *idtp9222) {
	u8 value = -1;
	int rc;

	// Update system's operating mode {EPP or BPP}
	rc = idtp9222_read(idtp9222, REG_ADDR_OPMODE, &value);

	return (rc == 0) ? ((value & OPMODE_MASK) >> OPMODE_SHIFT) : OPMODE_AC_MISSING;
}

static char *idtp9222_get_opmode_name(int opmode) {
	if (opmode == OPMODE_AC_MISSING)
		return "AC_MISSING";
	else if (opmode == OPMODE_WPC_BPP)
		return "BPP";
	else if (opmode == OPMODE_WPC_EPP)
		return "EPP";
	else
		pr_idt(IDT_VERBOSE, "opmode error\n");

	return "ERROR";
}


/*
 * IDTP9222 I2C setter
 */
static bool idtp9222_set_fod(struct idtp9222_struct *idtp9222) {
	const u8 *parameters = (idtp9222->firmware <= 0x508)
		? idtp9222->fod_legacy : idtp9222->opmode_midpower
		? idtp9222->fod_epp : idtp9222->fod_bpp;
	const int size = (idtp9222->firmware <= 0x508)
		? idtp9222->size_fodlegacy : idtp9222->opmode_midpower
		? idtp9222->size_fodepp : idtp9222->size_fodbpp;

	if (!size) {
		pr_idt(IDT_VERBOSE, "Skip to set %s fod (do not need)\n",
			idtp9222->opmode_midpower ? "EPP" : "BPP");
		return true;
	}

	idtp9222_write_block(idtp9222, REG_ADDR_FODCOEF, parameters, size);

	return true;
}

static bool idtp9222_set_full(struct idtp9222_struct *idtp9222) {
#ifdef CONFIG_QPNP_QG
	bool full = (idtp9222->capacity >= idtp9222->configure_full);
#else
	bool full = (idtp9222->capacity_raw >= idtp9222->configure_full);
#endif

	if (full && idtp9222->capacity_raw < (idtp9222->configure_recharge - 2))
		full = false;

	if (idtp9222_is_full(idtp9222) == full) {
		pr_idt(IDT_VERBOSE, "status full is already set to %d\n", full);
		return false;
	}

	if (!idtp9222_is_vrect(idtp9222) || !idtp9222_is_onpad(idtp9222))
		return false;

	if (full) {
		switch (idtp9222->opmode_type) {
		case WPC:
			/* CS100 is special signal for some TX pads */
			idtp9222_write(idtp9222, REG_ADDR_CHGSTAT, 100);
			idtp9222_write(idtp9222, REG_ADDR_COMMAND_L, SEND_CHGSTAT);
			pr_idt(IDT_UPDATE, "Sending CS100 to WPC pads for EoC\n");
			break;
		case PMA:
			idtp9222_write(idtp9222, REG_ADDR_EPT, EPT_BY_EOC);
			idtp9222_write(idtp9222, REG_ADDR_COMMAND_L, SEND_EPT);
			pr_idt(IDT_UPDATE, "Sending EPT to PMA pads for EoC\n");
			break;
		default:
			pr_idt(IDT_ERROR, "Is IDTP onpad really?\n");
			break;
		}
	}
	else
		; // Nothing to do for !full

	idtp9222->status_full = full;

	return true;
}

static bool idtp9222_set_capacity(struct idtp9222_struct *idtp9222) {
	if (!idtp9222_is_vrect(idtp9222))
		return false;

	if (idtp9222->capacity < 100) {
		if (idtp9222->opmode_type == WPC) {
			/* CS100 is special signal for some TX pads */
			idtp9222_write(idtp9222, REG_ADDR_CHGSTAT,
				idtp9222->capacity);
			idtp9222_write(idtp9222, REG_ADDR_COMMAND_L,
				SEND_CHGSTAT);
		}
	}

	return true;
}

static bool idtp9222_set_fullcurr(struct idtp9222_struct *idtp9222) {
	if (idtp9222->configure_fullcurr < 0) {
		pr_idt(IDT_VERBOSE, "configure is not defined for full current\n");
		return false;
	}

#ifdef CONFIG_QPNP_QG
	if (idtp9222->capacity >= idtp9222->configure_full) {
#else
	if (idtp9222->capacity_raw >= idtp9222->configure_full) {
#endif
		vote(idtp9222->dc_icl_votable, WLC_CS100_VOTER, true,
			idtp9222->configure_fullcurr);
		vote(idtp9222->wlc_voltage, WLC_CS100_VOTER, true,
			idtp9222->configure_bppvolt);
		idtp9222->reqpwr = REQPWR_9W;
	}
	else {
		vote(idtp9222->dc_icl_votable, WLC_CS100_VOTER, false, 0);
		vote(idtp9222->wlc_voltage, WLC_CS100_VOTER, false, 0);
		idtp9222->reqpwr = REQPWR_15W;
	}

	return true;
}

static bool idtp9222_set_overheat(struct idtp9222_struct *idtp9222) {
	/* On shutdown by overheat during wireless charging, send EPT by OVERHEAT */
	if (idtp9222->temperature >= idtp9222->configure_overheat) {
		if (!idtp9222_is_onpad(idtp9222) || idtp9222->status_overheat)
			return true;

		pr_idt(IDT_MONITOR, "The device is overheat, Send EPT_BY_OVERTEMP\n");

		if (idtp9222_is_vrect(idtp9222) &&
			idtp9222_write(idtp9222, REG_ADDR_EPT, EPT_BY_OVERTEMP) &&
			idtp9222_write(idtp9222, REG_ADDR_COMMAND_L, SEND_EPT)) {
			pr_idt(IDT_MONITOR, "Send EPT_BY_OVERTEMP!\n");
			idtp9222->status_overheat = true;
			schedule_delayed_work(&idtp9222->timer_overheat,
				round_jiffies_relative(msecs_to_jiffies(TIMER_OVERHEAT_MS)));
		}
		else {
			pr_idt(IDT_ERROR, "Failed to turning off by EPT_BY_OVERTEMP\n");
			return false;
		}
	}

	return true;
}

static bool idtp9222_set_charge_done(struct idtp9222_struct *idtp9222, bool done) {
	if (!idtp9222->configure_chargedone) {
		pr_idt(IDT_VERBOSE, "configure is not defined for charge done\n");
		return false;
	}

	if (idtp9222->status_done == done) {
		pr_idt(IDT_VERBOSE, "status_done is already set to %d\n", done);
		return false;
	}

	idtp9222->status_done = done;
	vote(idtp9222->wlc_suspend, DISABLE_BY_EOC, done, 0);

	if (idtp9222_is_vrect(idtp9222)
		&& idtp9222_is_onpad(idtp9222)
		&& idtp9222->status_done && !done
		&& !delayed_work_pending(&idtp9222->timer_setoff)) {
		pr_idt(IDT_MONITOR, "Start timer_setoff(%d ms)\n",
			OFFLINE_TIMER_MS);
		schedule_delayed_work(&idtp9222->timer_setoff,
			round_jiffies_relative(msecs_to_jiffies(OFFLINE_TIMER_MS)));
	}

	return true;
}

static bool idtp9222_set_maxinput(/* @Nullable */ struct idtp9222_struct *idtp9222,
	bool enable) {
#ifdef CONFIG_LGE_PM_VENEER_PSY
	/* At this time, Releasing IBAT/IDC via VENEER system */
	veneer_voter_passover(VOTER_TYPE_IBAT, VOTE_TOTALLY_RELEASED, enable);
	veneer_voter_passover(VOTER_TYPE_IDC, VOTE_TOTALLY_RELEASED, enable);
#endif
	return true;
}


/*
 * IDTP9222 charging logic
 */
static void idtp9222_set_default_voltage(struct idtp9222_struct *idtp9222) {
	if (idtp9222->guarpwr >= REQPWR_15W) {
		vote(idtp9222->dc_icl_votable, DEFAULT_VOTER,
			true, idtp9222->configure_maxcurr);
		vote(idtp9222->wlc_voltage, DEFAULT_VOTER,
			true, idtp9222->configure_maxvolt);
	} else {
		vote(idtp9222->dc_icl_votable, DEFAULT_VOTER,
			true, idtp9222->opmode_midpower ?
			idtp9222->configure_eppcurr : idtp9222->configure_bppcurr);
		idtp9222_write(idtp9222, REG_ADDR_VOUT, VOUT_V9P0);
		vote(idtp9222->wlc_voltage, DEFAULT_VOTER,
			true, idtp9222->opmode_midpower ?
			idtp9222->configure_eppvolt : idtp9222->configure_bppvolt);
	}
	rerun_election(idtp9222->wlc_voltage);
}

static bool idtp9222_set_onpad(struct idtp9222_struct *idtp9222, bool onpad) {
	u8 value = 0;

	if (idtp9222->status_onpad == onpad && !idtp9222->forcely) {
		pr_idt(IDT_VERBOSE, "status onpad is already set to %d\n", onpad);
		return false;
	}

	idtp9222->forcely = false;
	idtp9222->status_onpad = onpad;
	pr_idt(IDT_UPDATE, "%s onpad %d\n", IDTP9222_NAME_PSY,
		idtp9222_is_onpad(idtp9222));

	if (delayed_work_pending(&idtp9222->timer_connepp))
		cancel_delayed_work(&idtp9222->timer_connepp);

	if (onpad) {
		u8 vout_now = 0;
		u16 intr = 0;

		// 1. Read Vout Set Register (0x52) if matched (desired set == readback)?
		idtp9222_read(idtp9222, REG_ADDR_VOUT, &vout_now);
		pr_idt(IDT_REGISTER, "REG_ADDR_VOUT(%s) : 0x%02x\n",
			idtp9222->opmode_midpower ? "Y" : "N",  vout_now);

		// 2. Set Foreign Object Register
		idtp9222_set_fod(idtp9222);

		// 3. Check Tx Certificated information - used EPP only (check 8W EPP)
		idtp9222_read(idtp9222, REG_ADDR_TXID, &value);
		idtp9222->txid = value;
		idtp9222_read(idtp9222, REG_ADDR_GUARPWR, &value);
		idtp9222->guarpwr = value / 2;
		idtp9222_read(idtp9222, REG_ADDR_POTPWR, &value);
		idtp9222->potpwr = value / 2;
		idtp9222_read(idtp9222, REG_ADDR_SPECREV, &value);

		if (idtp9222->guarpwr > 0 && idtp9222->potpwr > 0)
			idtp9222->opmode_certi = true;

		// 4. Read Guarentee power & potential power and double check midpower
		pr_idt(IDT_REGISTER, "%s%s%s : TX SpecRev = %d, id = 0x%02x, "
			"GuarPWR = %dW, PotPWR = %dW\n",
			idtp9222_modename(idtp9222->opmode_type),
			idtp9222_get_opmode_name(idtp9222_get_opmode(idtp9222)),
			idtp9222->opmode_certi ? " Certified" : " Non-certified",
			value, idtp9222->txid, idtp9222->guarpwr, idtp9222->potpwr);

		if (idtp9222_get_opmode(idtp9222) == OPMODE_WPC_EPP)
			idtp9222->opmode_midpower = true;

		if (idtp9222->opmode_midpower && idtp9222->opmode_certi
			&& idtp9222->guarpwr < REQPWR_9W) {
			idtp9222->opmode_midpower = false;
		}

		// 5. if certified and guarpwr >= 15W, set 12V voltage
		idtp9222_read_word(idtp9222, REG_ADDR_INT_L, &intr);
		if (intr & EXTENDED_MODE) {
			idtp9222_set_default_voltage(idtp9222);
		}

		schedule_delayed_work(&idtp9222->worker_onpad,
			round_jiffies_relative(msecs_to_jiffies(ONPAD_TIMER_MS)));
	}
	else {
		/* Off pad conditions
		 * 1: idtp9222->gpio_detached HIGH (means device is far from pad) or
		 * 2: idtp9222->gpio_disabled HIGH (means USB inserted) or
		 * 3: psy_dc->PRESENT '0' over 5 secs
		 */
		pr_assert(!idtp9222->status_dcin
			|| !!gpio_get_value(idtp9222->gpio_detached)
			|| !!gpio_get_value(idtp9222->gpio_disabled));

		vote(idtp9222->wlc_voltage, DEFAULT_VOTER, false, 0);
		idtp9222_set_charge_done(idtp9222, false);
		idtp9222->opmode_type = UNKNOWN;
		idtp9222->opmode_midpower = false;
		idtp9222->opmode_certi = false;
		idtp9222->status_full = false;
		idtp9222->firmware = 0;
		idtp9222->txid = 0;
		idtp9222->guarpwr = 0;
		idtp9222->potpwr = 0;
		idtp9222->reqpwr = REQPWR_15W;

#ifdef CONFIG_LGE_PM_WORKAROUND_FW510
		if (idtp9222->probation_enable && idtp9222->probation_status) {
			idtp9222->probation_status = false;
			cancel_delayed_work(&idtp9222->probation_work);
			pr_idt(IDT_REGISTER, "probation is cancaled : configured\n");
		}
#endif
	}

	if (idtp9222->wlc_psy)
		power_supply_changed(idtp9222->wlc_psy);

	return true;
}

static bool idtp9222_set_vrect(struct idtp9222_struct *idtp9222) {
	int opmode = idtp9222_get_opmode(idtp9222);
	int request_volt = (idtp9222->reqpwr > REQPWR_9W)
		? idtp9222->configure_maxvolt : idtp9222->configure_eppvolt;
	u16 value = 0;
	u8 request_pwr = 0;
	u8 max_pwr = POWER_15W;

	if (12000 > ((idtp9222->configure_maxvolt/1000) *
	             (idtp9222->configure_maxcurr/1000) / 1000))
		max_pwr = POWER_13W;

	request_pwr = (idtp9222->reqpwr > REQPWR_9W) ? max_pwr : POWER_10W;

	idtp9222_read_word(idtp9222, REG_ADDR_FIRMWARE, &value);
	pr_idt(IDT_REGISTER, "REG_ADDR_FIRMWARE(%s) : 0x%02x\n",
		idtp9222_get_opmode_name(opmode), value);
	idtp9222->firmware = value;

	// 1. Update system's operating mode {WPC, or PMA} & {MIDPOWER, or not}
	if (opmode == OPMODE_WPC_EPP) {
		idtp9222->opmode_type = WPC;
		idtp9222->opmode_midpower = true;
	}
	else if (opmode == OPMODE_WPC_BPP)
		idtp9222->opmode_type = WPC;
	else if (opmode == OPMODE_PMA_SR1 || opmode == OPMODE_PMA_SR1E)
		idtp9222->opmode_type = PMA;
	else
		idtp9222->opmode_type = UNKNOWN;

	// 2. Set Register for requesting EPP Power Contract
	if (idtp9222->opmode_midpower) {
		value = (u16)((((request_volt + CONF_MARGIN_VOL) / 21000) << 12) / 1000);
		idtp9222_write(idtp9222, REG_ADDR_MPREQNP, request_pwr);
		idtp9222_write(idtp9222, REG_ADDR_MPREQMP, request_pwr);
		idtp9222_write(idtp9222, REG_ADDR_VOUT,
			(request_volt - CONF_MIN_VOL_RANGE)/100000);
		idtp9222_write_word(idtp9222, REG_ADDR_MPVRCALM1_L, value);
		idtp9222_set_fod(idtp9222);
		if (delayed_work_pending(&idtp9222->timer_connepp))
			cancel_delayed_work(&idtp9222->timer_connepp);
		schedule_delayed_work(&idtp9222->timer_connepp,
			round_jiffies_relative(msecs_to_jiffies(UNVOTING_TIMER_MS)));
	}
	else {
		vote(idtp9222->dc_icl_votable, DEFAULT_VOTER,
			true, idtp9222->configure_bppcurr);
		vote(idtp9222->wlc_voltage, DEFAULT_VOTER,
			true, idtp9222->configure_bppvolt);
	}

	return true;
}

static void idtp9222_restart_power_transfer(struct idtp9222_struct *idtp9222, bool disable) {
	int opmode = idtp9222_get_opmode(idtp9222);

	if (opmode == OPMODE_WPC_EPP) {
		pr_idt(IDT_REGISTER, "Restart!\n");
		idtp9222_write(idtp9222, REG_ADDR_EPT, EPT_BY_RESTART_POWER_TRANSFER);
		idtp9222_write(idtp9222, REG_ADDR_COMMAND_L, SEND_EPT);
		idtp9222_restart_silently(idtp9222);
	}
	else {
		if (!disable) {
			idtp9222->status_vrect = !gpio_get_value(idtp9222->gpio_vrect);
			idtp9222_set_vrect(idtp9222);
		}
		else {
			idtp9222_restart_silently(idtp9222);
		}
	}
	idtp9222->forcely = true;
}

static bool idtp9222_restart_silently(struct idtp9222_struct *idtp9222) {
	if (idtp9222->wlc_disable) {
		vote(idtp9222->wlc_disable, DISABLE_BY_RST, true, 0);
		msleep(1000);
		vote(idtp9222->wlc_disable, DISABLE_BY_RST, false, 0);

		pr_idt(IDT_VERBOSE, "Silent restart IDT chip successful!\n");
	}
	else
		return false;

	return true;
}


/*
 * IDTP9222 power_supply function
 */
static bool psy_set_dcin(struct idtp9222_struct *idtp9222, bool dcin) {
	if (idtp9222->status_dcin != dcin) {
		idtp9222->status_dcin = dcin;

		/* In the case of DCIN, release IBAT/IDC for 5 secs to establish wireless link */
		if (idtp9222->status_dcin) {
#ifdef CONFIG_LGE_PM
			if (wa_get_concurrency_mode_for_otg_wlc()) {
				pr_idt(IDT_REGISTER, "Set onpad for WLC+OTG\n");
				idtp9222->forcely = true;
				idtp9222_set_onpad(idtp9222, true);
			}
#endif
			if (idtp9222->opmode_midpower) {
				pr_idt(IDT_UPDATE, "Start timer_maxinput(%d ms)\n", UNVOTING_TIMER_MS);
				idtp9222_set_maxinput(idtp9222, true);
				schedule_delayed_work(&idtp9222->timer_maxinput,
					round_jiffies_relative(msecs_to_jiffies(UNVOTING_TIMER_MS)));
			}
#ifdef CONFIG_LGE_PM
			vote(idtp9222->dc_reset_votable, DC_RESET_BY_FOD, false, 0);
			vote(idtp9222->dc_reset_votable, DC_RESET_BY_MISSING, false, 0);
			vote(idtp9222->dc_reset_votable, DC_RESET_BY_VASHDN, false, 0);
			vote(idtp9222->dc_reset_votable, DC_RESET_BY_UV, false, 0);
#endif
		}
		else {
			if (delayed_work_pending(&idtp9222->timer_maxinput)) {
				pr_idt(IDT_UPDATE, "Cancel timer_maxinput\n");
				cancel_delayed_work(&idtp9222->timer_maxinput);
				idtp9222_set_maxinput(idtp9222, false);
			}
		}

		/* In the case of !DCIN, start timer to check real offline */
		if (idtp9222_is_onpad(idtp9222)
			&& !idtp9222->status_dcin
			&& !idtp9222->status_overheat
			&& !idtp9222->status_done
			&& !delayed_work_pending(&idtp9222->timer_setoff)) {
			pr_idt(IDT_MONITOR, "Start timer_setoff(%d ms)\n",
				OFFLINE_TIMER_MS);
			schedule_delayed_work(&idtp9222->timer_setoff,
				round_jiffies_relative(msecs_to_jiffies(OFFLINE_TIMER_MS)));
		}
		else if (idtp9222->status_dcin
			&& delayed_work_pending(&idtp9222->timer_setoff)) {
			pr_idt(IDT_MONITOR, "Cancel timer_setoff\n");
			cancel_delayed_work(&idtp9222->timer_setoff);
		}
		else
			;

		return true;
	}
	else
		return false;
}

static bool psy_set_capacity(struct idtp9222_struct *idtp9222, int capacity) {
	if (idtp9222->capacity == capacity) {
		pr_idt(IDT_VERBOSE, "status_capacity is already set to %d\n", capacity);
		return false;
	}

	idtp9222->capacity = capacity;

	idtp9222_set_capacity(idtp9222);
#ifdef CONFIG_QPNP_QG
	idtp9222_set_full(idtp9222);
	idtp9222_set_fullcurr(idtp9222);
#endif

	return true;
}

static bool psy_set_capacity_raw(struct idtp9222_struct *idtp9222, int capacity_raw) {
	if (idtp9222->capacity_raw == capacity_raw) {
		pr_idt(IDT_VERBOSE, "capacity_raw is already set to %d\n", capacity_raw);
		return false;
	}

	idtp9222->capacity_raw = capacity_raw;

#ifndef CONFIG_QPNP_QG
	idtp9222_set_full(idtp9222);
	idtp9222_set_fullcurr(idtp9222);
#endif

	if (idtp9222->status_done
		&& (idtp9222->capacity_raw <= idtp9222->configure_recharge))
		idtp9222_set_charge_done(idtp9222, false);

	return true;
}

static bool psy_set_temperature(struct idtp9222_struct *idtp9222, int temperature) {
	if (idtp9222->temperature == temperature) {
		pr_idt(IDT_VERBOSE, "temperature is already set to %d\n", temperature);
		return false;
	}

	pr_idt(IDT_VERBOSE, "Bettery temp is changed from %d to %d\n",
		idtp9222->temperature, temperature);
	idtp9222->temperature = temperature;

	idtp9222_set_overheat(idtp9222);

	return true;
}

static void psy_external_changed(struct power_supply *psy_me) {
	struct idtp9222_struct *idtp9222 = power_supply_get_drvdata(psy_me);
	union power_supply_propval value = { .intval = 0, };

	if (!idtp9222->psy_dc)
		idtp9222->psy_dc = power_supply_get_by_name("dc");

	if (!idtp9222->psy_battery)
		idtp9222->psy_battery = power_supply_get_by_name("battery");

	if (!idtp9222->dc_suspend_votable)
		idtp9222->dc_suspend_votable = find_votable("DC_SUSPEND");

	if (idtp9222->psy_dc) {
		static bool online_cached = false;

		if (!power_supply_get_property(idtp9222->psy_dc, POWER_SUPPLY_PROP_ONLINE, &value)) {
			bool online_now = !!value.intval;
			/* calling idtp9222_set_onpad(true) only if online false -> true */
			if (online_cached != online_now) {
				online_cached = online_now;
				if (online_now) {
					idtp9222_set_onpad(idtp9222, true);
				}
			}
		}
		if (!power_supply_get_property(idtp9222->psy_dc, POWER_SUPPLY_PROP_PRESENT, &value))
			psy_set_dcin(idtp9222, !!value.intval);
	}

	if (idtp9222->psy_battery) {
		if (!power_supply_get_property(idtp9222->psy_battery, POWER_SUPPLY_PROP_CAPACITY, &value))
			psy_set_capacity(idtp9222, value.intval);

		if (!power_supply_get_property(idtp9222->psy_battery, POWER_SUPPLY_PROP_CAPACITY_RAW, &value))
			psy_set_capacity_raw(idtp9222, value.intval);

		if (!power_supply_get_property(idtp9222->psy_battery, POWER_SUPPLY_PROP_TEMP, &value))
			psy_set_temperature(idtp9222, value.intval);

		if (idtp9222_is_onpad(idtp9222)
#ifdef CONFIG_LGE_PM
			&& !wa_get_concurrency_mode_for_otg_wlc()
#endif
			&& !power_supply_get_property(idtp9222->psy_battery, POWER_SUPPLY_PROP_CHARGE_DONE, &value)
			&& !!value.intval
			&& idtp9222->capacity_raw >= 100)
			idtp9222_set_charge_done(idtp9222, value.intval);
	}
}


/*
 * IDTP9222 power_supply getter & setter
 */
static enum power_supply_property psy_property_list[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_DONE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
};

static bool psy_set_enabled(struct idtp9222_struct *idtp9222, bool enabled) {
	if (idtp9222->wlc_disable)
		vote(idtp9222->wlc_disable, DISABLE_BY_USB, !enabled, 0);

	return true;
}

static bool psy_set_suspend(struct idtp9222_struct *idtp9222, bool suspend) {
	if (idtp9222->wlc_disable)
		vote(idtp9222->wlc_disable, DISABLE_BY_WA, suspend, 0);

	return true;
}

static bool psy_set_voltage_max(struct idtp9222_struct *idtp9222, int uV) {
	if (idtp9222->wlc_voltage) {
		if (idtp9222->configure_maxvolt > uV)
			vote(idtp9222->wlc_voltage, USER_VOTER, true,
				max(uV, idtp9222->configure_bppvolt));
		else
			vote(idtp9222->wlc_voltage, USER_VOTER, false, 0);
	}
	else
		return false;

	return true;
}

static bool psy_set_dc_reset(struct idtp9222_struct *idtp9222, int mode) {
	if (mode == DC_RESET_BY_RESTART) {
		idtp9222_restart_power_transfer(idtp9222, true);
	}
#ifdef CONFIG_LGE_PM
	else if (mode == DC_RESET_BY_CONCURRENT_ON) {
		if (!wa_get_concurrency_mode_for_otg_wlc())
			wa_concurrency_mode_on();
		if (wa_get_concurrency_mode_for_otg_wlc()) {
			idtp9222_set_charge_done(idtp9222, false);
			queue_work(idtp9222->wlc_otg_wq, &idtp9222->polling_vout);
		}
	}
	else if (mode == DC_RESET_BY_CONCURRENT_OFF) {
		vote(idtp9222->dc_reset_votable, DC_RESET_BY_FOD, false, 0);
		vote(idtp9222->dc_reset_votable, DC_RESET_BY_MISSING, false, 0);
		vote(idtp9222->dc_reset_votable, DC_RESET_BY_VASHDN, false, 0);
		vote(idtp9222->dc_reset_votable, DC_RESET_BY_UV, false, 0);
	}
#endif
	else
		return false;

	return true;
}

#define STEP_UNIT_1000mA	1000000
#define STEP_UNIT_800mA	800000
static bool psy_set_power_now(struct idtp9222_struct *idtp9222, int power) {
	if (idtp9222->power_now == power) {
		pr_idt(IDT_VERBOSE, "power_now is already set to %d\n", power);
		return false;
	}

	idtp9222->power_now = power;

	if (power > 3300000) {
		vote(idtp9222->dc_icl_votable, WLC_CRITICAL_VOTER, false, 0); /* 1.2A */
		vote(idtp9222->wlc_voltage, WLC_CRITICAL_VOTER, false, 0);
	} else if (power > 2900000) {
		vote(idtp9222->dc_icl_votable, WLC_CRITICAL_VOTER, true,
			MIN(idtp9222->configure_maxcurr, STEP_UNIT_1000mA)); /* 1A */
		vote(idtp9222->wlc_voltage, WLC_CRITICAL_VOTER, false, 0);
	} else if (power > 2500000) {
		vote(idtp9222->dc_icl_votable, WLC_CRITICAL_VOTER, true,
			MIN(idtp9222->configure_maxcurr, STEP_UNIT_800mA)); /* 0.8A */
		vote(idtp9222->wlc_voltage, WLC_CRITICAL_VOTER, false, 0);
	} else if (power > 1100000) {
		vote(idtp9222->dc_icl_votable, WLC_CRITICAL_VOTER, true, idtp9222->configure_eppcurr);
		vote(idtp9222->wlc_voltage, WLC_CRITICAL_VOTER, true, idtp9222->configure_eppvolt);
	} else {
		vote(idtp9222->dc_icl_votable, WLC_CRITICAL_VOTER, true, idtp9222->configure_bppcurr);
		vote(idtp9222->wlc_voltage, WLC_CRITICAL_VOTER, true, idtp9222->configure_bppvolt);
	}

	return true;
}

static int psy_property_set(struct power_supply *psy,
	enum power_supply_property prop, const union power_supply_propval *val) {
	struct idtp9222_struct *idtp9222 = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		psy_set_enabled(idtp9222, !!val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		psy_set_suspend(idtp9222, !!val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
//	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
		psy_set_voltage_max(idtp9222, val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		vote(idtp9222->dc_icl_votable, USER_VOTER, true, val->intval);
		break;
	case POWER_SUPPLY_PROP_DC_RESET:
		psy_set_dc_reset(idtp9222, val->intval);
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		psy_set_power_now(idtp9222, val->intval);
		break;
	default:
		break;
	}
	return 0;
}

static int psy_get_power(struct idtp9222_struct *idtp9222) {
	int power = 0;

	if (idtp9222_is_onpad(idtp9222)) {
		int voltage_mv = 0, current_ma = 0;

		if (!idtp9222->opmode_midpower) {
			voltage_mv = idtp9222->configure_bppvolt / 1000;
			current_ma = idtp9222->configure_bppcurr / 1000;
		}
		else if (idtp9222->guarpwr < REQPWR_15W) {
			voltage_mv = idtp9222->configure_eppvolt / 1000;
			current_ma = idtp9222->configure_eppcurr / 1000;
		}
		else {
			voltage_mv = idtp9222->configure_maxvolt / 1000;
			current_ma = idtp9222->configure_maxcurr / 1000;
		}
		power = voltage_mv * current_ma;
	}

	return power;
}

static int psy_get_current_now(struct idtp9222_struct *idtp9222) {
	u16 value = 0;

	if (idtp9222_is_vrect(idtp9222))
		idtp9222_read_word(idtp9222, REG_ADDR_IADC_L, &value);

	return (int)value;
}

#ifdef CONFIG_LGE_PM_WORKAROUND_FW510
static int psy_get_charge_control_limit(struct idtp9222_struct *idtp9222) {
	int current_limit = get_client_vote(idtp9222->dc_icl_votable, DEFAULT_VOTER);

	if (idtp9222_is_vrect(idtp9222) && idtp9222->probation_enable) {
		current_limit = idtp9222_set_probation_current(idtp9222, current_limit);
	}

	return current_limit;
}
#endif

static int psy_get_voltage_now(struct idtp9222_struct *idtp9222) {
	u16 value = 0;

	if (idtp9222_is_vrect(idtp9222))
		idtp9222_read_word(idtp9222, REG_ADDR_VADC_L, &value);

	return (int)value;
}

static int psy_get_opfreq(struct idtp9222_struct *idtp9222) {
	u16 value = 0;

	if (idtp9222_is_vrect(idtp9222))
		idtp9222_read_word(idtp9222, REG_ADDR_OPFREQ_L, &value);

	return (int)value;
}

static int psy_get_irq_status(struct idtp9222_struct *idtp9222) {
	u16 value = 0;

	if (idtp9222_is_vrect(idtp9222))
		idtp9222_read_word(idtp9222, REG_ADDR_STATUS_L, &value);

	return (int)value;
}

static int psy_property_get(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val) {
	struct idtp9222_struct *idtp9222 = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
#ifdef CONFIG_LGE_PM_VENEER_PSY
		/* Basically, IDTP9222's ONLINE and PRESENT are same.
		 * But in the some cases of LGE scenario,
		 * 'wireless' psy is required to pretend to 'OFFLINE' as fake.
		 */
		if (veneer_voter_suspended(VOTER_TYPE_IDC) == CHARGING_SUSPENDED_WITH_FAKE_OFFLINE) {
			pr_idt(IDT_RETURN, "Set Wireless UI as discharging");
			val->intval = false;
			break;
		}
#endif
		val->intval = idtp9222_is_onpad(idtp9222);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = idtp9222_is_onpad(idtp9222);
		break;
	case POWER_SUPPLY_PROP_PIN_ENABLED:
		val->intval = idtp9222_is_attached(idtp9222);
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = psy_get_power(idtp9222);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = get_effective_result(idtp9222->dc_icl_votable);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = psy_get_current_now(idtp9222);
		break;
#ifdef CONFIG_LGE_PM_WORKAROUND_FW510
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = psy_get_charge_control_limit(idtp9222);
		break;
#endif
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = get_effective_result(idtp9222->wlc_voltage);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = psy_get_voltage_now(idtp9222);
		break;
	case POWER_SUPPLY_PROP_CHARGE_DONE:
		val->intval = idtp9222_is_full(idtp9222);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = idtp9222_is_enabled(idtp9222);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		val->intval = !idtp9222_is_enabled(idtp9222);
		break;
	case POWER_SUPPLY_PROP_BUCK_FREQ:
		/* AC Signal Frequency on the coil in kHz*/
		val->intval = psy_get_opfreq(idtp9222);
		break;
	case POWER_SUPPLY_PROP_DC_RESET:
		val->intval = psy_get_irq_status(idtp9222);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int psy_property_writeable(struct power_supply *psy,
	enum power_supply_property prop) {
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}

	return rc;
}


/*
 * IDTP9222 Votable callback
 */
static int idtp9222_disable_callback(struct votable *votable, void *data,
	int disabled, const char *client) {
	struct idtp9222_struct *idtp9222 = data;

	/* WORKAROUND_TXID is needed to send EPT_BY_NORESPONSE
	 * to stop wireless charging normally */
	if (idtp9222_is_vrect(idtp9222)
		&& disabled
		&& idtp9222->txid == WORKAROUND_TXID) {
		pr_idt(IDT_UPDATE, "[WA] Send EPT_BY_NORESPONSE\n");
		idtp9222_write(idtp9222, REG_ADDR_EPT, EPT_BY_NORESPONSE);
		idtp9222_write(idtp9222, REG_ADDR_COMMAND_L, SEND_EPT);
	}

	gpiod_set_value(gpio_to_desc(idtp9222->gpio_disabled), !!disabled);

	if (disabled
		&& (strcmp(client, DISABLE_BY_WA)
			&& strcmp(client, DISABLE_BY_EOC)
			&& strcmp(client, DISABLE_BY_RST)))
		idtp9222_set_onpad(idtp9222, false);

	/* Wait for 20ms to allow disable normally */
	usleep_range(20000, 20010);

	pr_assert(gpio_get_value(idtp9222->gpio_disabled)==disabled);

	return 0;
}

#ifdef CONFIG_LGE_PM_WORKAROUND_FW510
static int idtp9222_set_probation_current(struct idtp9222_struct *idtp9222, int uA)
{
	u8 reg = 0;
	int value = uA;
	int configure_curr = idtp9222->configure_eppcurr;
	int probation_curr = idtp9222->probation_eppcurr;

	if (idtp9222_read(idtp9222, REG_ADDR_VOUT, &reg))
		if (idtp9222_read(idtp9222, REG_ADDR_VOUT, &reg))
			return value;

	if (reg >= VOUT_V12P0) {
		configure_curr = idtp9222->configure_maxcurr;
	} else if (reg < VOUT_V9P0) {
		configure_curr = idtp9222->configure_bppcurr;
		probation_curr = idtp9222->probation_bppcurr;
	}

	if (idtp9222->probation_status)
		value = min(uA, probation_curr);
	else
		value = min(uA, configure_curr);

	pr_idt(IDT_REGISTER,
		"status=%d, reg=%dmV, uA=%dmA, configure=%dmA, probation=%dmA, result=%dmA\n",
		idtp9222->probation_status, (reg+35)*100, uA/1000,
		configure_curr/1000, probation_curr/1000, value/1000);

	return value;
}
#endif

static int idtp9222_current_callback(struct votable *votable, void *data,
	int uA, const char *client) {
	struct idtp9222_struct *idtp9222 = data;
	union power_supply_propval value = { .intval = uA, };

	if (uA < 0)
		return 0;

	if (idtp9222->psy_dc) {
#ifdef CONFIG_LGE_PM_WORKAROUND_FW510
		if (idtp9222->probation_enable
			&& !wa_get_concurrency_mode_for_otg_wlc())
			value.intval = idtp9222_set_probation_current(idtp9222, uA);
#endif
		power_supply_set_property(idtp9222->psy_dc,
			POWER_SUPPLY_PROP_CURRENT_MAX, &value);
	}
	else
		pr_idt(IDT_VERBOSE, "Couldn't get psy_dc!\n");

	return 0;
}

static void idtp9222_stepper_work(struct work_struct *work) {
	struct idtp9222_struct *idtp9222 = container_of(work,
		struct idtp9222_struct, stepper_work.work);
	int target_voltage = get_effective_result(idtp9222->wlc_voltage);
	int voltage_now, uV;
	u8 value = 0;

	if (get_client_vote(idtp9222->wlc_voltage, DEFAULT_VOTER) <= 0) {
		pr_idt(IDT_REGISTER, "Skip to set voltage during CAL!\n");
		return;
	}

	if (idtp9222_is_vrect(idtp9222)) {
		idtp9222_read(idtp9222, REG_ADDR_VOUT, &value);
		voltage_now = (value * 100000) + CONF_MIN_VOL_RANGE;
		uV = MAX(target_voltage, voltage_now) - MIN(target_voltage, voltage_now);

		if (target_voltage > idtp9222->configure_eppvolt
			|| voltage_now > idtp9222->configure_eppvolt) {
			if (uV < 1000000)
				uV = target_voltage;
			else if (target_voltage > voltage_now)
				uV = voltage_now + 1000000;
			else
				uV = voltage_now - 1000000;
		}
		else {
			uV = target_voltage;
		}

		idtp9222_write(idtp9222, REG_ADDR_VOUT,
			(uV - CONF_MIN_VOL_RANGE)/100000);

		pr_idt(IDT_REGISTER, "Set %duV(%dmV <= %dmV)\n",
			uV, target_voltage / 1000, voltage_now / 1000);

#ifdef CONFIG_LGE_PM_WORKAROUND_FW510
		if (idtp9222->probation_enable &&
			target_voltage < 9000000 && !idtp9222->probation_status
			&& !wa_get_concurrency_mode_for_otg_wlc()) {
			idtp9222->probation_status = true;
			schedule_delayed_work(
				&idtp9222->probation_work,
				msecs_to_jiffies(idtp9222->probation_delay));
			pr_idt(IDT_REGISTER, "(bpp) turn on probation timer : probated\n");
			rerun_election(idtp9222->dc_icl_votable);
		}
#endif

		if (target_voltage != uV) {
			schedule_delayed_work(&idtp9222->stepper_work,
				round_jiffies_relative(msecs_to_jiffies(500)));
		}
	}
}
static int idtp9222_voltage_callback(struct votable *votable, void *data,
	int uV, const char *client) {
	struct idtp9222_struct *idtp9222 = data;

	if (uV < 0)
		return 0;

	if (idtp9222_is_vrect(idtp9222)) {
		if (uV <= idtp9222->configure_bppvolt)
			vote(idtp9222->dc_icl_votable, WLC_MIN_VOTER,
				true, idtp9222->configure_bppcurr);
		else if (uV <= idtp9222->configure_eppvolt)
			vote(idtp9222->dc_icl_votable, WLC_MIN_VOTER,
				true, idtp9222->configure_eppcurr);
		else
			vote(idtp9222->dc_icl_votable, WLC_MIN_VOTER,
				true, idtp9222->configure_maxcurr);

		/* For setting voltage, use stepper work
		idtp9222_write(idtp9222, REG_ADDR_VOUT,
			(uV - CONF_MIN_VOL_RANGE)/100000); */

		if (delayed_work_pending(&idtp9222->stepper_work))
			cancel_delayed_work(&idtp9222->stepper_work);
		schedule_delayed_work(&idtp9222->stepper_work, 0);
	}

	return 0;
}

static int idtp9222_suspend_callback(struct votable *votable, void *data,
	int suspend, const char *client) {
	struct idtp9222_struct *idtp9222 = data;
	union power_supply_propval value = { .intval = suspend, };

	if (idtp9222->psy_dc)
		power_supply_set_property(idtp9222->psy_dc,
			POWER_SUPPLY_PROP_INPUT_SUSPEND, &value);
	else
		pr_idt(IDT_VERBOSE, "Couldn't get psy_dc!\n");

	return 0;
}

static int idtp9222_dc_reset_callback(struct votable *votable, void *data,
	int reset, const char *client) {
	struct idtp9222_struct *idtp9222 = data;
	union power_supply_propval value = { .intval = 1, };

	if (idtp9222->psy_dc) {
		if (reset)
			power_supply_set_property(idtp9222->psy_dc,
				POWER_SUPPLY_PROP_DC_RESET, &value);
	}
	else
		pr_idt(IDT_VERBOSE, "Couldn't get psy_dc!\n");

	return 0;
}


/*
 * IDTP9222 Interrupts & Work structs
 */
static void idtp9222_worker_onpad(struct work_struct *work) {
	struct idtp9222_struct *idtp9222 = container_of(work,
		struct idtp9222_struct, worker_onpad.work);

	// 6. Check overheat status - EPT by Overheat
	idtp9222->status_overheat = false;
	idtp9222_set_overheat(idtp9222);

	// 7. Check Full status - CS100
	idtp9222->status_full = false;
	idtp9222_set_full(idtp9222);
	idtp9222_set_fullcurr(idtp9222);

	idtp9222_set_default_voltage(idtp9222);
}

static void idtp9222_timer_maxinput(struct work_struct *work) {
	struct idtp9222_struct *idtp9222 = container_of(work,
		struct idtp9222_struct, timer_maxinput.work);

	pr_idt(IDT_UPDATE, "Timer expired!\n");
	idtp9222_set_maxinput(idtp9222, false);
}

static void idtp9222_timer_setoff(struct work_struct *work) {
	struct idtp9222_struct *idtp9222 = container_of(work,
		struct idtp9222_struct, timer_setoff.work);

	if (idtp9222_is_onpad(idtp9222) && !idtp9222->status_dcin) {
		pr_idt(IDT_UPDATE, "FOD Detection!\n");
		if (get_effective_result(idtp9222->dc_reset_votable)) {
			rerun_election(idtp9222->dc_reset_votable);
		}
		else {
			vote(idtp9222->dc_reset_votable, DC_RESET_BY_FOD, true, 0);
			vote(idtp9222->dc_reset_votable, DC_RESET_BY_FOD, false, 0);
		}
		idtp9222_set_onpad(idtp9222, false);
	}
}

static void idtp9222_timer_overheat(struct work_struct *work) {
	struct idtp9222_struct *idtp9222 = container_of(work,
		struct idtp9222_struct, timer_overheat.work);

	if (idtp9222->status_dcin) {
		idtp9222->status_overheat = false;
		idtp9222_set_overheat(idtp9222);
	}
	else
		pr_idt(IDT_MONITOR, "Success - OVERHEAT\n");
}

static void idtp9222_timer_connepp(struct work_struct *work) {
	struct idtp9222_struct *idtp9222 = container_of(work,
		struct idtp9222_struct, timer_connepp.work);

	if (!idtp9222_is_vrect(idtp9222))
		return;

	if (idtp9222->status_dcin)
		return;

	pr_idt(IDT_UPDATE, "Can not connect EPP - Restart!\n");

	idtp9222->reqpwr = REQPWR_9W;
	idtp9222_restart_power_transfer(idtp9222, true);
	idtp9222_restart_silently(idtp9222);
}

static void idtp9222_polling_log(struct work_struct *work) {
	struct idtp9222_struct *idtp9222 = container_of(work,
		struct idtp9222_struct, polling_log.work);

	// Monitor 4 GPIOs
	int idtfault = gpio_get_value(idtp9222->gpio_idtfault);
	int detached = gpio_get_value(idtp9222->gpio_detached);
	int vrect = gpio_get_value(idtp9222->gpio_vrect);
	int disabled = get_effective_result_locked(idtp9222->wlc_disable);
	pr_idt(IDT_MONITOR, "IRQ:0x%02x, GPIO%d(<-idtfault):%d, "
		"GPIO%d(<-detached):%d, GPIO%d(<-vrect):%d, GPIO%d(->disabled):%d\n",
		psy_get_irq_status(idtp9222),
		idtp9222->gpio_idtfault, idtfault,
		idtp9222->gpio_detached, detached,
		idtp9222->gpio_vrect, vrect,
		idtp9222->gpio_disabled, disabled);

	schedule_delayed_work(&idtp9222->polling_log, round_jiffies_relative
		(msecs_to_jiffies(1000*30)));
}

#define ABNORMAL_VRECT_MV	2900
static void idtp9222_polling_vrect(struct work_struct *work) {
	struct idtp9222_struct *idtp9222 = container_of(work,
		struct idtp9222_struct, polling_vrect.work);
	int vrect = 0;
	u16 value = -1;

	if (idtp9222_is_vrect(idtp9222)) {
		idtp9222_read_word(idtp9222, REG_ADDR_VRECT_L, &value);
		vrect = (int)value;
	}

	if (vrect < ABNORMAL_VRECT_MV)
		wa_clear_dc_reverse_volt_v2_trigger();

	if (!gpio_get_value(idtp9222->gpio_detached))
		schedule_delayed_work(&idtp9222->polling_vrect,
			round_jiffies_relative(msecs_to_jiffies(1000)));
}

#ifdef CONFIG_LGE_PM
static void idtp9222_polling_vout(struct work_struct *work) {
	struct idtp9222_struct *idtp9222 = container_of(work,
		struct idtp9222_struct, polling_vout);
	int dc_icl_now = idtp9222->configure_bppcurr;
	int suspend = 0;

	if (!idtp9222->psy_dc)
		idtp9222->psy_dc = power_supply_get_by_name("dc");

	if (!idtp9222->psy_dc) {
		pr_idt(IDT_ERROR, "Could not get dc_psy supply\n");
		return;
	}

	if (!idtp9222->dc_suspend_votable)
		idtp9222->dc_suspend_votable = find_votable("DC_SUSPEND");

	if (!idtp9222->dc_suspend_votable) {
		pr_idt(IDT_ERROR, "Could not get DC_SUSPEND votable\n");
		return;
	}

	while(true) {
		union power_supply_propval buf = { .intval = 0, };
		int mid_vnow = !power_supply_get_property(idtp9222->psy_dc,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &buf) ? buf.intval : -1;
		int wlc_vset = get_effective_result(idtp9222->wlc_voltage);

		if (mid_vnow < 1000000)
			mid_vnow /= 10;
		else
			mid_vnow /= 1000;

		if (!wa_get_concurrency_mode_for_otg_wlc())
			break;

		suspend = get_effective_result(idtp9222->dc_suspend_votable);

		if (mid_vnow < DCIN_THRESHOLD_MV) {
			if (!suspend) {
				vote(idtp9222->dc_suspend_votable, WLC_OTG_VOTER, true, 0);
				suspend = 1;
			}
			dc_icl_now = 0;
		}
		else if (mid_vnow < 5500 || dc_icl_now < 200000) {
			vote(idtp9222->dc_icl_votable, WLC_OTG_VOTER, true, 200000);
			dc_icl_now = 200000;
		}
		else if (mid_vnow < 5700 || dc_icl_now < 400000) {
			vote(idtp9222->dc_icl_votable, WLC_OTG_VOTER, true, 400000);
			dc_icl_now = 400000;
		}
		else if (mid_vnow < 5900 || dc_icl_now < 600000) {
			vote(idtp9222->dc_icl_votable, WLC_OTG_VOTER, true, 600000);
			dc_icl_now = 600000;
		}
		else {
			vote(idtp9222->dc_icl_votable, WLC_OTG_VOTER, false, 0);
			dc_icl_now = idtp9222->configure_bppcurr;
		}

		if (wlc_vset >= idtp9222->configure_eppvolt && mid_vnow > 7000) {
			if (suspend) {
				vote(idtp9222->dc_suspend_votable, WLC_OTG_VOTER, false, 0);
				suspend = 0;
			}
		}
		else if (wlc_vset < idtp9222->configure_eppvolt && mid_vnow > 5500) {
			if (suspend) {
				vote(idtp9222->dc_suspend_votable, WLC_OTG_VOTER, false, 0);
				suspend = 0;
			}
		}
		else
			;	// to do nothing

		if (suspend)
			msleep(500);
		else
			usleep_range(4000, 5000);
	}

	vote(idtp9222->dc_suspend_votable, WLC_OTG_VOTER, false, 0);
	vote(idtp9222->dc_icl_votable, WLC_OTG_VOTER, false, 0);
}
#endif

#ifdef CONFIG_LGE_PM_WORKAROUND_FW510
static void idtp9222_probation_work(struct work_struct *work)
{
	struct idtp9222_struct *idtp9222 = container_of(work,
		struct idtp9222_struct, probation_work.work);

	idtp9222->probation_status = false;
	pr_idt(IDT_REGISTER, "probation timer is done : configured\n");
	rerun_election(idtp9222->dc_icl_votable);
}
#endif


/*
 * IDTP9222 Interrupts
 */
static irqreturn_t idtp9222_isr_idtfault(int irq, void *data) {
	/* This ISR will be triggered on below unrecoverable exceptions :
	 * Over temperature, Over current, or Over voltage detected by IDTP922X chip.
	 * IDTP9222 turns off after notifying it to host, so there's nothing to handle
	 * except logging here.
	 */
	struct idtp9222_struct *idtp9222 = data;
	int idtfault = gpio_get_value(idtp9222->gpio_idtfault);
	int vrect = !gpio_get_value(idtp9222->gpio_vrect);
	int irq_status = psy_get_irq_status(idtp9222);
	u16 intr = 0;

	if (vrect)
		idtp9222_read_word(idtp9222, REG_ADDR_INT_L, &intr);

	pr_idt(IDT_INTERRUPT, "triggered %d(int=0x%04x irq=0x%04x)\n",
		idtfault, intr, irq_status);

	if (idtfault != 0)
		return IRQ_HANDLED;

	if (vrect) {
		idtp9222_write_word(idtp9222, REG_ADDR_INT_CLR_L, intr);
		idtp9222_write(idtp9222, REG_ADDR_COMMAND_L, SEND_INT_CLR);

#ifdef CONFIG_LGE_PM
		if (irq_status & AC_MISSING_DETECTION) {
			pr_idt(IDT_UPDATE, "Missing Detection!\n");
			vote(idtp9222->dc_reset_votable, DC_RESET_BY_MISSING, true, 0);
			vote(idtp9222->dc_reset_votable, DC_RESET_BY_MISSING, false, 0);
		}
#endif

		if ((intr & EXTENDED_MODE)
			&& idtp9222_get_opmode(idtp9222) == OPMODE_WPC_EPP
			&& idtp9222_is_onpad(idtp9222)) {
			idtp9222_set_default_voltage(idtp9222);
#ifdef CONFIG_LGE_PM_WORKAROUND_FW510
			if (idtp9222->probation_enable && !idtp9222->probation_status
				&& !wa_get_concurrency_mode_for_otg_wlc()) {
				idtp9222->probation_status = true;
				schedule_delayed_work(
					&idtp9222->probation_work,
					msecs_to_jiffies(idtp9222->probation_delay));
				pr_idt(IDT_REGISTER, "(epp) turn on probation timer : probated\n");
				rerun_election(idtp9222->dc_icl_votable);
			}
#endif
			if (delayed_work_pending(&idtp9222->timer_maxinput)) {
				pr_idt(IDT_UPDATE, "Cancel timer_maxinput\n");
				cancel_delayed_work(&idtp9222->timer_maxinput);
				idtp9222_set_maxinput(idtp9222, false);
			}
		}

#ifdef CONFIG_LGE_PM
		if ((intr == 0x0080)
			&& (irq_status == 0x00E0)
			&& !idtp9222_is_onpad(idtp9222)
			&& wa_get_concurrency_mode_for_otg_wlc()) {
			pr_idt(IDT_REGISTER, "Set onpad for WLC+OTG\n");
			idtp9222->forcely = true;
			idtp9222_set_onpad(idtp9222, true);
		}
#endif
	}

	return IRQ_HANDLED;
}

static irqreturn_t idtp9222_isr_vrect(int irq, void *data) {
	struct idtp9222_struct *idtp9222 = data;
	bool vrect = !gpio_get_value(idtp9222->gpio_vrect);

	if (idtp9222->status_vrect == vrect) {
		pr_idt(IDT_VERBOSE, "status_vrect is already set to %d\n", vrect);
		return IRQ_HANDLED;
	}

	idtp9222->status_vrect = vrect;

	if (vrect) {
#ifdef CONFIG_LGE_PM
		if (!wa_get_concurrency_mode_for_otg_wlc())
			wa_concurrency_mode_on();
		if (wa_get_concurrency_mode_for_otg_wlc())
			queue_work(idtp9222->wlc_otg_wq, &idtp9222->polling_vout);
#endif
		idtp9222_write_word(idtp9222, REG_ADDR_INT_EN_L, 0xFD97);
		idtp9222_set_vrect(idtp9222);
	}
#ifdef CONFIG_LGE_PM
	else {
		idtp9222_set_charge_done(idtp9222, false);
		vote(idtp9222->dc_reset_votable, DC_RESET_BY_FOD, false, 0);
		vote(idtp9222->dc_reset_votable, DC_RESET_BY_MISSING, false, 0);
		vote(idtp9222->dc_reset_votable, DC_RESET_BY_VASHDN, false, 0);
		vote(idtp9222->dc_reset_votable, DC_RESET_BY_UV, false, 0);
	}
#endif

	pr_idt(IDT_INTERRUPT, "triggered %d\n", vrect);

	return IRQ_HANDLED;
}

static irqreturn_t idtp9222_isr_detached(int irq, void *data) {
	struct idtp9222_struct *idtp9222 = data;
	bool detached = !!gpio_get_value(idtp9222->gpio_detached);

	if (detached) {
		idtp9222_set_onpad(idtp9222, false);
		idtp9222_wakelock_release(idtp9222->wlc_wakelock);
	}
	else {
		idtp9222_wakelock_acquire(idtp9222->wlc_wakelock);
		schedule_delayed_work(&idtp9222->polling_vrect,
				round_jiffies_relative(msecs_to_jiffies(1000)));
	}

	pr_idt(IDT_INTERRUPT, "triggered %d\n", detached);

	return IRQ_HANDLED;
}


/*
 * IDTP9222 Probes
 */
static bool idtp9222_probe_votables(struct idtp9222_struct *idtp9222) {
	idtp9222->dc_icl_votable = create_votable("DC_ICL",
		VOTE_MIN,
		idtp9222_current_callback,
		idtp9222);
	if (IS_ERR(idtp9222->dc_icl_votable)) {
		pr_idt(IDT_ERROR, "unable to create DC_ICL votable\n");
		idtp9222->dc_icl_votable = NULL;
		return false;
	}

	idtp9222->wlc_disable = create_votable("WLC_DISABLE",
		VOTE_SET_ANY,
		idtp9222_disable_callback,
		idtp9222);
	if (IS_ERR(idtp9222->wlc_disable)) {
		pr_idt(IDT_ERROR, "unable to create wlc_disable votable\n");
		idtp9222->wlc_disable = NULL;
		return false;
	}

	idtp9222->wlc_voltage = create_votable("WLC_VOLTAGE",
		VOTE_MIN,
		idtp9222_voltage_callback,
		idtp9222);
	if (IS_ERR(idtp9222->wlc_voltage)) {
		pr_idt(IDT_ERROR, "unable to create wlc_voltage votable\n");
		idtp9222->wlc_voltage = NULL;
		return false;
	}

	idtp9222->wlc_suspend = create_votable("WLC_SUSPEND",
		VOTE_SET_ANY,
		idtp9222_suspend_callback,
		idtp9222);
	if (IS_ERR(idtp9222->wlc_suspend)) {
		pr_idt(IDT_ERROR, "unable to create wlc_suspend votable\n");
		idtp9222->wlc_suspend = NULL;
		return false;
	}

	idtp9222->dc_reset_votable = create_votable("DC_RESET",
		VOTE_SET_ANY,
		idtp9222_dc_reset_callback,
		idtp9222);
	if (IS_ERR(idtp9222->dc_reset_votable)) {
		pr_idt(IDT_ERROR, "unable to create dc_reset_votable\n");
		idtp9222->dc_reset_votable = NULL;
		return false;
	}

	if (idtp9222->configure_eppcurr > 0 && idtp9222->configure_bppcurr > 0) {
		vote(idtp9222->dc_icl_votable, DEFAULT_VOTER, true,
			MIN(idtp9222->configure_eppcurr, idtp9222->configure_bppcurr));
	}

	return true;
}

static bool idtp9222_probe_devicetree(struct device_node *dnode,
	struct idtp9222_struct *idtp9222) {
	struct device_node *battery_supp =
		of_find_node_by_name(NULL, "lge-battery-supplement");
	struct device_node *charger_supp =
		of_find_node_by_name(NULL, "qcom,qpnp-smb5");
	int buf = -1;

	if (!dnode) {
		pr_idt(IDT_ERROR, "dnode is null\n");
		return false;
	}

	idtp9222->configure_sysfs = of_property_read_bool(dnode, "idt,configure-sysfs");
	idtp9222->configure_chargedone = of_property_read_bool(dnode, "idt,configure-charge-done");

/* Parse from the other DT */
#ifdef CONFIG_QPNP_QG
	if (!charger_supp
		|| of_property_read_u32(charger_supp, "qcom,auto-recharge-soc", &buf) < 0) {
		pr_idt(IDT_ERROR, "auto-recharge-soc is failed\n");
		idtp9222->configure_recharge = 98;
	}
	else
		idtp9222->configure_recharge = buf;

	if (!battery_supp
		|| of_property_read_u32(battery_supp, "capacity-raw-full", &buf) < 0) {
		pr_idt(IDT_ERROR, "capacity-raw-full is failed\n");
		idtp9222->configure_full = 100;
	} else
		idtp9222->configure_full = buf;
#else
	if (!charger_supp
		|| of_property_read_u32(charger_supp, "qcom,auto-recharge-soc", &buf) < 0) {
		pr_idt(IDT_ERROR, "auto-recharge-soc is failed\n");
		idtp9222->configure_recharge = 249;
	}
	else
		idtp9222->configure_recharge = buf * 255 / 100;

	if (!battery_supp
		|| of_property_read_u32(battery_supp, "capacity-raw-full", &buf) < 0) {
		pr_idt(IDT_ERROR, "capacity-raw-full is failed\n");
		idtp9222->configure_full = 247;
	} else
		idtp9222->configure_full = buf;
#endif

/* Parse GPIOs */
	idtp9222->gpio_idtfault = of_get_named_gpio(dnode, "idt,gpio-idtfault", 0);
	if (idtp9222->gpio_idtfault < 0) {
		pr_idt(IDT_ERROR, "Fail to get gpio-idtfault\n");
		return false;
	}

	idtp9222->gpio_detached = of_get_named_gpio(dnode, "idt,gpio-detached", 0);
	if (idtp9222->gpio_detached < 0) {
		pr_idt(IDT_ERROR, "Fail to get gpio-detached\n");
		return false;
	}

	idtp9222->gpio_vrect = of_get_named_gpio(dnode, "idt,gpio-vrect", 0);
	if (idtp9222->gpio_vrect < 0) {
		pr_idt(IDT_ERROR, "Fail to get gpio-vrect\n");
		return false;
	}

	idtp9222->gpio_disabled = of_get_named_gpio(dnode, "idt,gpio-disabled", 0);
	if (idtp9222->gpio_disabled < 0) {
		pr_idt(IDT_ERROR, "Fail to get gpio-disabled\n");
		return false;
	}

/* Parse FOD parameters */
	idtp9222->fod_legacy = (u8 *)of_get_property(dnode, "idt,fod-legacy", &idtp9222->size_fodlegacy);
	if (idtp9222->fod_legacy == NULL) {
		pr_idt(IDT_VERBOSE, "Not used 'idt,fod-legacy'\n");
		idtp9222->size_fodlegacy = 0;
	}

	idtp9222->fod_bpp = (u8 *)of_get_property(dnode, "idt,fod-bpp", &idtp9222->size_fodbpp);
	if (idtp9222->fod_bpp == NULL) {
		pr_idt(IDT_VERBOSE, "Not used 'idt,fod-bpp'\n");
		idtp9222->size_fodbpp = 0;
	}

	idtp9222->fod_epp = (u8 *)of_get_property(dnode, "idt,fod-epp", &idtp9222->size_fodepp);
	if (idtp9222->fod_epp == NULL) {
		pr_idt(IDT_VERBOSE, "Not used 'idt,fod-epp'\n");
		idtp9222->size_fodepp = 0;
	}

/* Parse misc */
	if (of_property_read_u32(dnode, "idt,configure-bppcurr", &buf) < 0)
		idtp9222->configure_bppcurr = 900000;
	else
		idtp9222->configure_bppcurr = buf;

	if (of_property_read_u32(dnode, "idt,configure-eppcurr", &buf) < 0)
		idtp9222->configure_eppcurr = 900000;
	else
		idtp9222->configure_eppcurr = buf;

	if (of_property_read_u32(dnode, "idt,configure-maxcurr", &buf) < 0)
		idtp9222->configure_maxcurr = 900000;
	else
		idtp9222->configure_maxcurr = buf;

	if (of_property_read_u32(dnode, "idt,configure-bppvolt", &buf) < 0)
		idtp9222->configure_bppvolt = 5500000;
	else
		idtp9222->configure_bppvolt = buf;

	if (of_property_read_u32(dnode, "idt,configure-eppvolt", &buf) < 0)
		idtp9222->configure_eppvolt = 9000000;
	else
		idtp9222->configure_eppvolt = buf;

	if (of_property_read_u32(dnode, "idt,configure-maxvolt", &buf) < 0)
		idtp9222->configure_maxvolt = 12000000;
	else
		idtp9222->configure_maxvolt = buf;

	if (of_property_read_u32(dnode, "idt,configure-fullcurr", &buf) < 0)
		idtp9222->configure_fullcurr = -EINVAL;
	else
		idtp9222->configure_fullcurr = buf;

	if (of_property_read_u32(dnode, "idt,configure-overheat", &buf) < 0) {
		pr_idt(IDT_ERROR, "Fail to get configure-overheat\n");
		return false;
	}
	else
		idtp9222->configure_overheat = buf;

#ifdef CONFIG_LGE_PM_WORKAROUND_FW510
	idtp9222->probation_enable = of_property_read_bool(dnode, "idt,probation-enable");
	if (idtp9222->probation_enable) {
		if (of_property_read_u32(dnode, "idt,probation-delay", &buf) < 0)
			idtp9222->probation_delay = 120000;
		else
			idtp9222->probation_delay = buf;

		if (of_property_read_u32(dnode, "idt,probation-bppcurr", &buf) < 0)
			idtp9222->probation_bppcurr = 400000;
		else
			idtp9222->probation_bppcurr = buf;

		if (of_property_read_u32(dnode, "idt,probation-eppcurr", &buf) < 0)
			idtp9222->probation_eppcurr = 600000;
		else
			idtp9222->probation_eppcurr = buf;

		idtp9222->probation_status = false;
	}
#endif

	return true;
}

static bool idtp9222_probe_gpios(struct idtp9222_struct *idtp9222) {
	struct pinctrl *gpio_pinctrl;
	struct pinctrl_state *gpio_state;
	int ret;

	// PINCTRL here
	gpio_pinctrl = devm_pinctrl_get(idtp9222->wlc_device);
	if (IS_ERR_OR_NULL(gpio_pinctrl)) {
		pr_idt(IDT_ERROR, "Failed to get pinctrl (%ld)\n", PTR_ERR(gpio_pinctrl));
		return false;
	}

	gpio_state = pinctrl_lookup_state(gpio_pinctrl, "wlc_pinctrl");
	if (IS_ERR_OR_NULL(gpio_state)) {
		pr_idt(IDT_ERROR, "pinstate not found, %ld\n", PTR_ERR(gpio_state));
		return false;
	}

	ret = pinctrl_select_state(gpio_pinctrl, gpio_state);
	if (ret < 0) {
		pr_idt(IDT_ERROR, "cannot set pins %d\n", ret);
		return false;
	}

	// Set direction
	ret = gpio_request_one(idtp9222->gpio_idtfault, GPIOF_DIR_IN, "gpio_idtfault");
	if (ret < 0) {
		pr_idt(IDT_ERROR, "Fail to request gpio_idtfault %d\n", ret);
		return false;
	}

	ret = gpio_request_one(idtp9222->gpio_detached, GPIOF_DIR_IN, "gpio_detached");
	if (ret < 0) {
		pr_idt(IDT_ERROR, "Fail to request gpio_detached, %d\n", ret);
		return false;
	}

	ret = gpio_request_one(idtp9222->gpio_vrect, GPIOF_DIR_IN, "gpio_vrect");
	if (ret < 0) {
		pr_idt(IDT_ERROR, "Fail to request gpio_vrect, %d\n", ret);
		return false;
	}

	ret = gpio_request_one(idtp9222->gpio_disabled, GPIOF_DIR_OUT, "gpio_disabled");
	if (ret < 0) {
		pr_idt(IDT_ERROR, "Fail to request gpio_disabled %d\n", ret);
		return false;
	}

	return true;
}

static bool idtp9222_probe_psy(/* @Nonnulll */ struct idtp9222_struct *idtp9222) {
	const static struct power_supply_desc desc = {
		.name = IDTP9222_NAME_PSY,
		.type = POWER_SUPPLY_TYPE_WIRELESS,
		.properties = psy_property_list,
		.num_properties = ARRAY_SIZE(psy_property_list),
		.get_property = psy_property_get,
		.set_property = psy_property_set,
		.property_is_writeable = psy_property_writeable,
		.external_power_changed = psy_external_changed,
	};
	const struct power_supply_config cfg = {
		.drv_data = idtp9222,
		.of_node = idtp9222->wlc_device->of_node,
	};

	idtp9222->wlc_psy = power_supply_register(idtp9222->wlc_device, &desc, &cfg);
	if (!IS_ERR(idtp9222->wlc_psy)) {
		static char *from [] = { "battery", "dc" };
		idtp9222->wlc_psy->supplied_from = from;
		idtp9222->wlc_psy->num_supplies = ARRAY_SIZE(from);
		return true;
	}
	else {
		pr_info("Couldn't register idtp9222 power supply (%ld)\n",
			PTR_ERR(idtp9222->wlc_psy));
		return false;
	}
}

static bool idtp9222_probe_irqs(struct idtp9222_struct *idtp9222) {
	int ret = 0;

	/* GPIO IDTFault */
	ret = request_threaded_irq(gpio_to_irq(idtp9222->gpio_idtfault),
		NULL, idtp9222_isr_idtfault, IRQF_ONESHOT|IRQF_TRIGGER_FALLING,
		"wlc-idtfault", idtp9222);
	if (ret) {
		pr_idt(IDT_ERROR, "Cannot request irq %d (%d)\n",
			gpio_to_irq(idtp9222->gpio_idtfault), ret);
		return false;
	}
	else
		enable_irq_wake(gpio_to_irq(idtp9222->gpio_idtfault));

	/* GPIO Detached */
	ret = request_threaded_irq(gpio_to_irq(idtp9222->gpio_detached),
		NULL, idtp9222_isr_detached, IRQF_ONESHOT|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
		"wlc-detached", idtp9222);
	if (ret) {
		pr_idt(IDT_ERROR, "Cannot request irq %d (%d)\n",
			gpio_to_irq(idtp9222->gpio_detached), ret);
		return false;
	}
	else
		enable_irq_wake(gpio_to_irq(idtp9222->gpio_detached));

	/* GPIO Vrect */
	ret = request_threaded_irq(gpio_to_irq(idtp9222->gpio_vrect),
		NULL, idtp9222_isr_vrect, IRQF_ONESHOT|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
		"wlc-vrect", idtp9222);
	if (ret) {
		pr_idt(IDT_ERROR, "Cannot request irq %d (%d)\n",
			gpio_to_irq(idtp9222->gpio_vrect), ret);
		return false;
	}
	else
		enable_irq_wake(gpio_to_irq(idtp9222->gpio_vrect));

	return true;
}

static int idtp9222_remove(struct i2c_client *client) {
	struct idtp9222_struct *idtp9222 = i2c_get_clientdata(client);
	pr_idt(IDT_VERBOSE, "idt9222 is about to be removed from system\n");

	if (idtp9222) {
	/* Clear descripters */
		if (delayed_work_pending(&idtp9222->worker_onpad))
			cancel_delayed_work_sync(&idtp9222->worker_onpad);
		if (delayed_work_pending(&idtp9222->timer_maxinput))
			cancel_delayed_work_sync(&idtp9222->timer_maxinput);
		if (delayed_work_pending(&idtp9222->timer_setoff))
			cancel_delayed_work_sync(&idtp9222->timer_setoff);
		if (delayed_work_pending(&idtp9222->timer_overheat))
			cancel_delayed_work_sync(&idtp9222->timer_overheat);
		if (delayed_work_pending(&idtp9222->timer_connepp))
			cancel_delayed_work_sync(&idtp9222->timer_connepp);
		if (delayed_work_pending(&idtp9222->polling_log))
			cancel_delayed_work_sync(&idtp9222->polling_log);
		if (delayed_work_pending(&idtp9222->stepper_work))
			cancel_delayed_work_sync(&idtp9222->stepper_work);
		if (delayed_work_pending(&idtp9222->polling_vrect))
			cancel_delayed_work_sync(&idtp9222->polling_vrect);
#ifdef CONFIG_LGE_PM
		cancel_work_sync(&idtp9222->polling_vout);
		flush_workqueue(idtp9222->wlc_otg_wq);
		destroy_workqueue(idtp9222->wlc_otg_wq);
#endif
		if (idtp9222->wlc_disable)
			destroy_votable(idtp9222->wlc_disable);
	/* Clear power_supply */
		if (idtp9222->wlc_psy)
			power_supply_unregister(idtp9222->wlc_psy);
		if (idtp9222->psy_battery)
			power_supply_put(idtp9222->psy_battery);
		if (idtp9222->psy_dc)
			power_supply_put(idtp9222->psy_dc);
	/* Clear gpios */
		if (idtp9222->gpio_idtfault)
			gpio_free(idtp9222->gpio_idtfault);
		if (idtp9222->gpio_detached)
			gpio_free(idtp9222->gpio_detached);
		if (idtp9222->gpio_vrect)
			gpio_free(idtp9222->gpio_vrect);
		if (idtp9222->gpio_disabled)
			gpio_free(idtp9222->gpio_disabled);
		if (idtp9222->wlc_wakelock)
			wakeup_source_unregister(idtp9222->wlc_wakelock);
	/* Finally, make me free */
		kfree(idtp9222);
		return 0;
	}
	else
		return -EINVAL;
}

static int idtp9222_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	struct idtp9222_struct *idtp9222 = kzalloc(sizeof(struct idtp9222_struct), GFP_KERNEL);

	pr_idt(IDT_VERBOSE, "Start\n");

	if (!idtp9222) {
		pr_idt(IDT_ERROR, "Failed to alloc memory\n");
		goto error;
	}
	else {
		// Store the platform_data to drv_data
		i2c_set_clientdata(client, idtp9222);
	}

	// For client and device
	idtp9222->wlc_client = client;
	idtp9222->wlc_device = &client->dev;
	idtp9222->wlc_device->platform_data = idtp9222;

	mutex_init(&idtp9222->io_lock);
	idtp9222->wlc_wakelock = wakeup_source_register(NULL, "IDTP9222: wakelock");

	// For remained preset
	if (!idtp9222_probe_devicetree(idtp9222->wlc_device->of_node, idtp9222)) {
		pr_idt(IDT_ERROR, "Fail to read parse_dt\n");
		goto error;
	}
	// For GPIOs
	if (!idtp9222_probe_gpios(idtp9222)) {
		pr_idt(IDT_ERROR, "Fail to request gpio at probe\n");
		goto error;
	}
	// For psy
	if (!idtp9222_probe_psy(idtp9222)) {
		pr_idt(IDT_ERROR, "Unable to register wlc_psy\n");
		goto error;
	}
	// Request irqs
	if (!idtp9222_probe_irqs(idtp9222)) {
		pr_idt(IDT_ERROR, "Fail to request irqs at probe\n");
		goto error;
	}
	// Create sysfs if it is configured
	if (idtp9222->configure_sysfs
		&& sysfs_create_group(&idtp9222->wlc_device->kobj, &idtp9222_sysfs_files) < 0) {
		pr_idt(IDT_ERROR, "unable to create sysfs\n");
		goto error;
	}
	// For votables
	if (!idtp9222_probe_votables(idtp9222)) {
		pr_idt(IDT_ERROR, "unable to create/get votables\n");
		goto error;
	}

	// For work structs
	INIT_DELAYED_WORK(&idtp9222->worker_onpad, idtp9222_worker_onpad);
	INIT_DELAYED_WORK(&idtp9222->timer_maxinput, idtp9222_timer_maxinput);
	INIT_DELAYED_WORK(&idtp9222->timer_setoff, idtp9222_timer_setoff);
	INIT_DELAYED_WORK(&idtp9222->timer_overheat, idtp9222_timer_overheat);
	INIT_DELAYED_WORK(&idtp9222->timer_connepp, idtp9222_timer_connepp);
	INIT_DELAYED_WORK(&idtp9222->polling_log, idtp9222_polling_log);
	INIT_DELAYED_WORK(&idtp9222->polling_vrect, idtp9222_polling_vrect);
	INIT_DELAYED_WORK(&idtp9222->stepper_work, idtp9222_stepper_work);
#ifdef CONFIG_LGE_PM_WORKAROUND_FW510
	INIT_DELAYED_WORK(&idtp9222->probation_work, idtp9222_probation_work);
#endif
#ifdef CONFIG_LGE_PM
	INIT_WORK(&idtp9222->polling_vout, idtp9222_polling_vout);
	idtp9222->wlc_otg_wq = alloc_ordered_workqueue("wlc-otg", 0);
	if (!idtp9222->wlc_otg_wq) {
		pr_idt(IDT_ERROR, "unable to alloc workqueue\n");
		goto error;
	}
#endif

	schedule_delayed_work(&idtp9222->polling_log, 0);

	idtp9222->reqpwr = REQPWR_15W;
	idtp9222->forcely = false;

	if (!gpio_get_value(idtp9222->gpio_vrect)) {
		idtp9222_restart_power_transfer(idtp9222, false);
	}

	psy_external_changed(idtp9222->wlc_psy);
	pr_idt(IDT_VERBOSE, "Complete probing IDTP9222\n");
	return 0;

error:
	idtp9222_remove(client);
	return -EPROBE_DEFER;
}

//Compatible node must be matched to dts
static struct of_device_id idtp9222_match [] = {
	{ .compatible = IDTP9222_NAME_COMPATIBLE, },
	{ },
};

//I2C slave id supported by driver
static const struct i2c_device_id idtp9222_id [] = {
	{ IDTP9222_NAME_DRIVER, 0 },
	{ }
};

//I2C Driver Info
static struct i2c_driver idtp9222_driver = {
	.driver = {
		.name = IDTP9222_NAME_DRIVER,
		.owner = THIS_MODULE,
		.of_match_table = idtp9222_match,
	},
	.id_table = idtp9222_id,

	.probe = idtp9222_probe,
	.remove = idtp9222_remove,
};

module_i2c_driver(idtp9222_driver);

MODULE_DESCRIPTION(IDTP9222_NAME_DRIVER);
MODULE_LICENSE("GPL v2");
