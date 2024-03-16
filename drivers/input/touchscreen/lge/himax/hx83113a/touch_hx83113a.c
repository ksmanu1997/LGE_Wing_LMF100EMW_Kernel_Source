/*  Himax Android Driver Sample Code for QCT platform

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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>

#include <touch_core.h>
#include <touch_hwif.h>
#include <touch_common.h>

#include "touch_hx83113a.h"
#include "touch_hx83113a_core.h"

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#include <soc/qcom/lge/board_lge.h>
#include <linux/msm_lcd_power_mode.h>
#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY)
#include <linux/msm_lcd_recovery.h>
#endif
#endif

#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
#include <soc/mediatek/lge/board_lge.h>
#include <linux/lcd_power_mode.h>
#endif
#if defined(CONFIG_LGE_MODULE_DETECT)
int panel_id1 = 0;
#endif
#define GEST_SUP_NUM 5
	uint8_t gest_event[GEST_SUP_NUM] = {
	0x80, 0x01, 0x02, 0x03, 0x04};

#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F
#define TS_WAKE_LOCK_TIMEOUT		(2 * HZ)
#define FRAME_COUNT 5

#if 1//driver code moify not to modify core code
#define SWIPE_R SWIPE_R2
#define SWIPE_L SWIPE_L2
#endif

struct himax_ts_data *private_ts;
struct himax_ic_data *ic_data;
struct himax_report_data *hx_touch_data;
struct himax_debug *debug_data;
struct device *g_hx_dev;

struct himax_target_report_data *g_target_report_data = NULL;
static int		HX_TOUCH_INFO_POINT_CNT;

unsigned long	FW_VER_MAJ_FLASH_ADDR;
unsigned long 	FW_VER_MIN_FLASH_ADDR;
unsigned long 	CFG_VER_MAJ_FLASH_ADDR;
unsigned long 	CFG_VER_MIN_FLASH_ADDR;
unsigned long 	CID_VER_MAJ_FLASH_ADDR;
unsigned long 	CID_VER_MIN_FLASH_ADDR;
/*unsigned long	PANEL_VERSION_ADDR;*/

unsigned long 	FW_VER_MAJ_FLASH_LENG;
unsigned long 	FW_VER_MIN_FLASH_LENG;
unsigned long 	CFG_VER_MAJ_FLASH_LENG;
unsigned long 	CFG_VER_MIN_FLASH_LENG;
unsigned long 	CID_VER_MAJ_FLASH_LENG;
unsigned long 	CID_VER_MIN_FLASH_LENG;
/*unsigned long	PANEL_VERSION_LENG;*/

unsigned long 	FW_CFG_VER_FLASH_ADDR;
unsigned long	FW_LGE_VER_MAJ_FLASH_ADDR;

unsigned char	IC_CHECKSUM = 0;

int hx_EB_event_flag = 0;
int hx_EC_event_flag = 0;
int hx_ED_event_flag = 0;
int g_zero_event_count = 0;

#define TCI_FAIL_NUM 8
static const char const *tci_debug_str[TCI_FAIL_NUM] = {
	"SUCCESS",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP",
	"MULTI_FINGER",
	"DELAY_TIME",/* It means Over Tap */
	"PALM_STATE",
	"OUTOF_AREA",
};
#define TCI_SWIPE_FAIL_NUM 11
static const char const *tci_swipe_debug_str[TCI_SWIPE_FAIL_NUM] = {
	"ERROR",
	"FINGER_FAST_RELEASE",
	"MULTI_FINGER",
	"FAST_SWIPE",
	"SLOW_SWIPE",
	"WRONG_DIRECTION",
	"RATIO_FAIL",
	"OUT_OF_START_AREA",
	"INITIAL_RATIO_FAIL",
	"OUT_OF_ACTIVE_AREA",
	"PALM_STATE",
};

static uint8_t 	AA_press = 0x00;
static int	p_point_num	= 0xFFFF;
static int	probe_fail_flag;
static uint8_t DBG_BUF[HX_MAX_MSG_NUM * HX_MSG_SIZE];

static int wMm_value =10;
int g_ts_dbg = 0;

int i2c_error_count = 0;

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY_ESD) || IS_ENABLED(CONFIG_LGE_TOUCH_PANEL_GLOBAL_RESET)
extern void	lge_mdss_report_panel_dead(void);
#endif
#endif

#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
extern void	mtkfb_esd_recovery(void);
#endif

#ifdef CONFIG_LGE_TOUCH_HIMAX_HX83113A_DEBUG
extern int hx83113a_debug_init(struct device *dev);
extern int hx83113a_debug_remove(struct device *dev);
#endif
extern int hx83113a_proc_init(void);
extern void hx83113a_proc_deinit(void);
extern int hx83113a_register_sysfs(struct device *dev);
void hx83113a_rst_gpio_set(int pinnum, uint8_t value);
static int hx83113a_cable_connect(struct device *dev);

static int hx83113a_bus_read(struct device *dev, uint8_t command, uint8_t *data, uint32_t length)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = to_himax_data(dev);
	struct touch_bus_msg msg = {0,};
	int ret = 0;

	mutex_lock(&d->rw_lock);

	ts->tx_buf[0] = command;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = length;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], length);

	mutex_unlock(&d->rw_lock);
	return 0;
}

static int hx83113a_bus_write(struct device *dev, uint8_t command, uint8_t *data, uint32_t length)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = to_himax_data(dev);
	struct touch_bus_msg msg = {0,};
	int ret = 0;

	mutex_lock(&d->rw_lock);
	ts->tx_buf[0] = command;
	memcpy(&ts->tx_buf[1], data, length);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = length + 1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	mutex_unlock(&d->rw_lock);
	return 0;
}

static void hx83113a_in_parse_assign_cmd(uint32_t addr, uint8_t *cmd, int len)
{
	/*TOUCH_I("%s: Entering!\n", __func__);*/
	switch (len) {
	case 1:
		cmd[0] = addr;
		/*TOUCH_I("%s: cmd[0] = 0x%02X\n", __func__, cmd[0]);*/
		break;

	case 2:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		/*TOUCH_I("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X\n", __func__, cmd[0], cmd[1]);*/
		break;

	case 4:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		cmd[2] = (addr >> 16) % 0x100;
		cmd[3] = addr / 0x1000000;
		/*  TOUCH_I("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X,cmd[2] = 0x%02X,cmd[3] = 0x%02X\n",
			__func__, cmd[0], cmd[1], cmd[2], cmd[3]);*/
		break;

	default:
		TOUCH_E("%s: input length fault,len = %d!\n", __func__, len);
	}
}

void hx83113a_burst_enable(struct device *dev, uint8_t auto_add_4_byte)
{
	uint8_t tmp_data[DATA_LEN_4];
	/*TOUCH_I("%s,Entering \n",__func__);*/
	tmp_data[0] = IC_CMD_CONTI;

	if (hx83113a_bus_write(dev, IC_ADR_CONTI, tmp_data, 1) < 0) {
		TOUCH_E("%s: i2c access fail!\n", __func__);
		return;
	}

	tmp_data[0] = (IC_CMD_INCR4 | auto_add_4_byte);

	if (hx83113a_bus_write(dev, IC_ADR_INCR4, tmp_data, 1) < 0) {
		TOUCH_E("%s: i2c access fail!\n", __func__);
		return;
	}
}

int hx83113a_register_read(struct device *dev, uint32_t addr, uint32_t read_length, uint8_t *read_data, uint8_t addr_len)
{
	uint8_t tmp_data[DATA_LEN_4];

	/*TOUCH_I("%s,Entering \n",__func__);*/

	if (read_length > FLASH_RW_MAX_LEN) {
		TOUCH_E("%s: read len over %d!\n", __func__, FLASH_RW_MAX_LEN);
		return LENGTH_FAIL;
	} else {
		hx83113a_in_parse_assign_cmd(addr, tmp_data, addr_len);
	}

		if (read_length > DATA_LEN_4) {
			hx83113a_burst_enable(dev, 1);
		} else {
			hx83113a_burst_enable(dev, 0);
		}

	/*address = (read_addr[3] << 24) + (read_addr[2] << 16) + (read_addr[1] << 8) + read_addr[0];
	i = address;
	tmp_data[0] = (uint8_t)i;
	tmp_data[1] = (uint8_t)(i >> 8);
	tmp_data[2] = (uint8_t)(i >> 16);
	tmp_data[3] = (uint8_t)(i >> 24);*/

		if (hx83113a_bus_write(dev, IC_ADR_AHB_ADDR_BYTE_0, tmp_data, DATA_LEN_4) < 0) {
			TOUCH_E("%s: i2c access fail!\n", __func__);
			return I2C_FAIL;
		}

		tmp_data[0] = IC_CMD_AHB_ACCESS_DIRECTION_READ;

		if (hx83113a_bus_write(dev, IC_ADR_AHB_ACCESS_DIRECTION, tmp_data, 1) < 0) {
			TOUCH_E("%s: i2c access fail!\n", __func__);
			return I2C_FAIL;
		}

		if (hx83113a_bus_read(dev, IC_ADR_AHB_RDATA_BYTE_0, read_data, read_length) < 0) {
			TOUCH_E("%s: i2c access fail!\n", __func__);
			return I2C_FAIL;
		}

	if (read_length > DATA_LEN_4) {
		hx83113a_burst_enable(dev, 0);
	}

	return NO_ERR;
}

static int hx83113a_flash_write_burst_lenth(struct device *dev, uint8_t *reg_byte, uint8_t *write_data, uint32_t length)
{
	uint8_t *data_byte;
	int i = 0, j = 0;

	/* if (length + ADDR_LEN_4 > FLASH_RW_MAX_LEN) {
		TOUCH_E("%s: write len over %d!\n", __func__, FLASH_RW_MAX_LEN);
		return;
	} */

	data_byte = kzalloc(sizeof(uint8_t)*(length + 4), GFP_KERNEL);

	for (i = 0; i < ADDR_LEN_4; i++) {
		data_byte[i] = reg_byte[i];
	}

	for (j = ADDR_LEN_4; j < length + ADDR_LEN_4; j++) {
		data_byte[j] = write_data[j - ADDR_LEN_4];
	}

	if (hx83113a_bus_write(dev, IC_ADR_AHB_ADDR_BYTE_0, data_byte, length + ADDR_LEN_4) < 0) {
		TOUCH_E("%s: i2c access fail!\n", __func__);
		kfree(data_byte);
		return I2C_FAIL;
	}
	kfree(data_byte);

	return NO_ERR;
}

int hx83113a_register_write(struct device *dev, uint32_t addr, uint32_t write_length, uint8_t *write_data, uint8_t addr_len)
{
	/*int address;*/
	uint8_t tmp_addr[DATA_LEN_4];

	/*TOUCH_I("%s,Entering \n", __func__);*/

	/*address = (write_addr[3] << 24) + (write_addr[2] << 16) + (write_addr[1] << 8) + write_addr[0];*/
	hx83113a_in_parse_assign_cmd(addr, tmp_addr, addr_len);

		if (write_length > DATA_LEN_4) {
			hx83113a_burst_enable(dev, 1);
		} else {
			hx83113a_burst_enable(dev, 0);
		}

	if (hx83113a_flash_write_burst_lenth(dev, tmp_addr, write_data, write_length) < 0) {
		TOUCH_E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}

	return NO_ERR;
}

void hx83113a_interface_on(struct device *dev)
{
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t tmp_data2[DATA_LEN_4];
	int cnt = 0;

	/* Read a dummy register to wake up I2C.*/
	if (hx83113a_bus_read(dev, IC_ADR_AHB_RDATA_BYTE_0, tmp_data, DATA_LEN_4) < 0) {/* to knock I2C*/
		TOUCH_E("%s: i2c access fail!\n", __func__);
		return;
	}

	do {
		tmp_data[0] = IC_CMD_CONTI;

		if (hx83113a_bus_write(dev, IC_ADR_CONTI, tmp_data, 1) < 0) {
			TOUCH_E("%s: i2c access fail!\n", __func__);
			return;
		}

		tmp_data[0] = IC_CMD_INCR4;

		if (hx83113a_bus_write(dev, IC_ADR_INCR4, tmp_data, 1) < 0) {
			TOUCH_E("%s: i2c access fail!\n", __func__);
			return;
		}

		/*Check cmd*/
		hx83113a_bus_read(dev, IC_ADR_CONTI, tmp_data, 1);
		hx83113a_bus_read(dev, IC_ADR_INCR4, tmp_data2, 1);

		if (tmp_data[0] == IC_CMD_CONTI && tmp_data2[0] == IC_CMD_INCR4) {
			break;
		}

		touch_msleep(1);
	} while (++cnt < 10);

	if (cnt > 0) {
		TOUCH_I("%s:Polling burst mode: %d times\n", __func__, cnt);
	}
}

static void hx83113a_hw_reset(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	TOUCH_TRACE();

	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(20);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(50);

}

static void hx83113a_reload_config(struct device *dev)
{
	if (hx83113a_report_data_init()) {
		TOUCH_E("%s: allocate data fail\n", __func__);
	}
	hx83113a_sense_on(dev, 0x00);
}

void hx83113a_ic_reset(struct device *dev, uint8_t loadconfig, uint8_t int_off)
{
	struct himax_ts_data *d = to_himax_data(dev);

	TOUCH_I("%s,status: loadconfig=%d,int_off=%d\n", __func__, loadconfig, int_off);

	if (d->rst_gpio >= 0) {
		if (int_off) {
			touch_interrupt_control(d->dev, INTERRUPT_DISABLE);
		}

		hx83113a_hw_reset(dev);

		if (loadconfig) {
			hx83113a_reload_config(dev);
		}

		if (int_off) {
			touch_interrupt_control(d->dev, INTERRUPT_ENABLE);
		}
	}
}

void hx83113a_sense_on(struct device *dev, uint8_t FlashMode)
{
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t w_data[DATA_LEN_4];
	int retry = 0;
	int i = 0;
	TOUCH_I("Enter %s \n", __func__);

	for (i = 0; i < ADDR_LEN_4; i++) {
		w_data[i] = (uint8_t)(FW_DATA_CLEAR >> i * 8);
	}
	hx83113a_interface_on(dev);
	hx83113a_register_write(dev, FW_ADDR_CTRL_FW,	DATA_LEN_4, w_data, ADDR_LEN_4);
	touch_msleep(10);

	if (!FlashMode) {
		hx83113a_ic_reset(dev, false, false);
	} else {
		do {
			hx83113a_register_read(dev, FW_ADDR_FLAG_RESET_EVENT, DATA_LEN_4, tmp_data, ADDR_LEN_4);
			TOUCH_I("%s:Read status from IC = %X,%X\n", __func__, tmp_data[0], tmp_data[1]);
		} while ((tmp_data[1] != 0x01 || tmp_data[0] != 0x00) && retry++ < 5);

		if (retry >= 5) {
			TOUCH_E("%s: Fail:\n", __func__);
			hx83113a_ic_reset(dev, false, false);
		} else {
			TOUCH_I("%s:OK and Read status from IC = %X,%X\n", __func__, tmp_data[0], tmp_data[1]);
			/* reset code*/
			tmp_data[0] = 0x00;

			if (hx83113a_bus_write(dev, IC_ADR_I2C_PSW_LB, tmp_data, 1) < 0) {
				TOUCH_E("%s: i2c access fail!\n", __func__);
			}

			if (hx83113a_bus_write(dev, IC_ADR_I2C_PSW_UB, tmp_data, 1) < 0) {
				TOUCH_E("%s: i2c access fail!\n", __func__);
			}
		}
	}
}

bool hx83113a_sense_off(struct device *dev, bool check_en)
{
	uint8_t cnt = 0;
	uint8_t tmp_data[DATA_LEN_4];
	int i = 0;

	for (i = 0; i < ADDR_LEN_4; i++) {
		tmp_data[i] = (uint8_t)(FW_DATA_FW_STOP >> i * 8);
	}
	do {
		if (cnt == 0 || (tmp_data[0] != 0xA5 && tmp_data[0] != 0x00 && tmp_data[0] != 0x87)) {
		 hx83113a_register_write(dev, FW_ADDR_CTRL_FW, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		}
		touch_msleep(20);

		/* check fw status */
		hx83113a_register_read(dev, IC_ADR_CS_CENTRAL_STATE, DATA_LEN_4, tmp_data, ADDR_LEN_4);

		if (tmp_data[0] != 0x05) {
			TOUCH_I("%s: Do not need wait FW, Status = 0x%02X!\n", __func__, tmp_data[0]);
			break;
		}

		hx83113a_register_read(dev, FW_ADDR_CTRL_FW, 4, tmp_data, ADDR_LEN_4);
		TOUCH_I("%s: cnt = %d, data[0] = 0x%02X!\n", __func__, cnt, tmp_data[0]);
	} while (tmp_data[0] != 0x87 && (++cnt < 50) && check_en == true);

	cnt = 0;

	do {
		tmp_data[0] = IC_CMD_I2C_PSW_LB;
		if (hx83113a_bus_write(dev, IC_ADR_I2C_PSW_LB, tmp_data, 1) < 0) {
			TOUCH_E("%s: i2c access fail!\n", __func__);
			return false;
		}
		tmp_data[0] = IC_CMD_I2C_PSW_UB;
		if (hx83113a_bus_write(dev, IC_ADR_I2C_PSW_UB, tmp_data, 1) < 0) {
			TOUCH_E("%s: i2c access fail!\n", __func__);
			return false;
		}

		tmp_data[0] = 0x00;
		if (hx83113a_bus_write(dev, IC_ADR_I2C_PSW_LB, tmp_data, 1) < 0) {
			TOUCH_E("%s: i2c access fail!\n", __func__);
			return false;
		}

		tmp_data[0] = IC_CMD_I2C_PSW_LB;
		if (hx83113a_bus_write(dev, IC_ADR_I2C_PSW_LB, tmp_data, 1) < 0) {
			TOUCH_E("%s: i2c access fail!\n", __func__);
			return false;
		}
		tmp_data[0] = IC_CMD_I2C_PSW_UB;
		if (hx83113a_bus_write(dev, IC_ADR_I2C_PSW_UB, tmp_data, 1) < 0) {
			TOUCH_E("%s: i2c access fail!\n", __func__);
			return false;
		}

		//tmp_addr[3] = 0x90; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00; tmp_addr[0] = 0xA8;
		hx83113a_register_read(dev, IC_ADR_CS_CENTRAL_STATE, ADDR_LEN_4, tmp_data, ADDR_LEN_4);
		TOUCH_I("%s: Check enter_save_mode data[0]=%X \n", __func__, tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			//tmp_addr[3] = 0x80; tmp_addr[2] = 0x02; tmp_addr[1] = 0x00; tmp_addr[0] = 0x20;
			tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = 0x00; tmp_data[0] = 0x00;
			hx83113a_register_write(dev, IC_ADR_TCON_ON_RST, DATA_LEN_4, tmp_data, ADDR_LEN_4);
			touch_msleep(1);
			tmp_data[0] = 0x01;
			hx83113a_register_write(dev, IC_ADR_TCON_ON_RST, DATA_LEN_4, tmp_data, ADDR_LEN_4);
			//tmp_addr[3] = 0x80; tmp_addr[2] = 0x02; tmp_addr[1] = 0x00; tmp_addr[0] = 0x94;
			tmp_data[0] = 0x00;
			hx83113a_register_write(dev, IC_ADDR_ADC_ON_RST, DATA_LEN_4, tmp_data, ADDR_LEN_4);
			touch_msleep(1);
			tmp_data[0] = 0x01;
			hx83113a_register_write(dev, IC_ADDR_ADC_ON_RST, DATA_LEN_4, tmp_data, ADDR_LEN_4);

			return true;
		} else {
			touch_msleep(10);
			hx83113a_hw_reset(dev);
		}
	} while (cnt++ < 15);

	return false;
}

void hx83113a_rst_gpio_set(int pinnum, uint8_t value)
{
	gpio_direction_output(pinnum, value);
}

uint8_t hx83113a_int_gpio_read(int pinnum)
{
	return gpio_get_value(pinnum);
}

static int hx83113a_power(struct device *dev, int ctrl)
{
//	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("%s\n", __func__);

	switch (ctrl) {

		case POWER_OFF:
			TOUCH_I("%s, off\n", __func__);
			break;

		case POWER_ON:
			TOUCH_I("%s, on\n", __func__);
			break;

		case POWER_SLEEP:
			TOUCH_I("%s, sleep\n", __func__);
			break;

		case POWER_WAKE:
			TOUCH_I("%s, wake\n", __func__);
			break;

		case POWER_HW_RESET_ASYNC:
			TOUCH_I("%s, reset\n", __func__);
			hx83113a_hw_reset(dev);
			break;
	}

	return 0;
}

static int hx83113a_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = to_himax_data(dev);

	int update = 0;

	d->ic_info.bin_official_ver = fw->data[FW_LGE_VER_MAJ_FLASH_ADDR];
	d->ic_info.bin_fw_ver = fw->data[FW_LGE_VER_MAJ_FLASH_ADDR + 1];

	if (ts->force_fwup) {
		update = 1;
	} else if((d->ic_info.ic_official_ver != d->ic_info.bin_official_ver)
			|| (d->ic_info.ic_fw_ver != d->ic_info.bin_fw_ver)) {
		update = 1;
	}

	TOUCH_I("%s : bin_fw_version = V%d.%d,  ic_fw_version = V%d.%d" \
			" ,update = %d, force: %d\n", __func__,
			d->ic_info.bin_official_ver, d->ic_info.bin_fw_ver,
			d->ic_info.ic_official_ver, d->ic_info.ic_fw_ver,
			update, ts->force_fwup);

	return update;
}

static void hx83113a_init_psl(struct device *dev) /*power saving level*/
{
	uint8_t w_data[DATA_LEN_4];
	int i = 0;

	for (i = 0; i < ADDR_LEN_4; i++) {
		w_data[i] = (uint8_t)(IC_CMD_RST >> i * 8);
	}
	hx83113a_register_write(dev, IC_ADR_PSL, DATA_LEN_4, w_data, ADDR_LEN_4);
	TOUCH_I("%s: power saving level reset OK!\n", __func__);
}

static bool hx83113a_wait_wip(struct device *dev, int Timing)
{
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t w_data[DATA_LEN_4];
	int i = 0;
	int retry_cnt = 0;

	for (i = 0; i < ADDR_LEN_4; i++) {
		w_data[i] = (uint8_t)(FLASH_DATA_SPI200_TRANS_FMT >> i * 8);
	}
	hx83113a_register_write(dev, FLASH_ADDR_SPI200_TRANS_FMT, DATA_LEN_4, w_data, ADDR_LEN_4);
	tmp_data[0] = 0x01;

	do {
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(FLASH_DATA_SPI200_TRANS_CTRL_1 >> i * 8);
		}
		hx83113a_register_write(dev, FLASH_ADDR_SPI200_TRANS_CTRL, DATA_LEN_4, w_data, ADDR_LEN_4);
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(FLASH_DATA_SPI200_CMD_1 >> i * 8);
		}
		hx83113a_register_write(dev, FLASH_ADDR_SPI200_CMD, DATA_LEN_4, w_data, ADDR_LEN_4);
		tmp_data[0] = tmp_data[1] = tmp_data[2] = tmp_data[3] = 0xFF;
		hx83113a_register_read(dev, FLASH_ADDR_SPI200_DATA, 4, tmp_data, ADDR_LEN_4);

		if ((tmp_data[0] & 0x01) == 0x00) {
			return true;
		}

		retry_cnt++;

		if (tmp_data[0] != 0x00 || tmp_data[1] != 0x00 || tmp_data[2] != 0x00 || tmp_data[3] != 0x00)
			TOUCH_D(FW_UPGRADE, "%s:Wait wip retry_cnt:%d, buffer[0]=%d, buffer[1]=%d, buffer[2]=%d, buffer[3]=%d \n",
			  __func__, retry_cnt, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		if (retry_cnt > 100) {
			TOUCH_E("%s: Wait wip error!\n", __func__);
			return false;
		}

		touch_msleep(Timing);
	} while ((tmp_data[0] & 0x01) == 0x01);

	return true;
}

static bool hx83113a_block_erase(struct device *dev, int start_addr, int length) /*complete not yet*/
{
	uint32_t page_prog_start = 0;
	uint32_t block_size = 0x10000;
	uint8_t tmp_data[4] = {0};
	uint8_t w_data[DATA_LEN_4];
	int i = 0;

	hx83113a_interface_on(dev);

	hx83113a_init_psl(dev);

	for (i = 0; i < ADDR_LEN_4; i++) {
		w_data[i] = (uint8_t)(FLASH_DATA_SPI200_TRANS_FMT >> i * 8);
	}
	hx83113a_register_write(dev, FLASH_ADDR_SPI200_TRANS_FMT, DATA_LEN_4, w_data, ADDR_LEN_4);

	for (page_prog_start = start_addr; page_prog_start < start_addr + length; page_prog_start = page_prog_start + block_size) {
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(FLASH_DATA_SPI200_TRANS_CTRL_2 >> i * 8);
		}
		hx83113a_register_write(dev, FLASH_ADDR_SPI200_TRANS_CTRL, DATA_LEN_4, w_data, ADDR_LEN_4);
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(FLASH_DATA_SPI200_CMD_2 >> i * 8);
		}
		hx83113a_register_write(dev, FLASH_ADDR_SPI200_CMD, DATA_LEN_4, w_data, ADDR_LEN_4);

		tmp_data[3] = (page_prog_start >> 24)&0xFF;
		tmp_data[2] = (page_prog_start >> 16)&0xFF;
		tmp_data[1] = (page_prog_start >> 8)&0xFF;
		tmp_data[0] = page_prog_start&0xFF;

		hx83113a_register_write(dev, FLASH_ADDR_SPI200_ADDR, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(FLASH_DATA_SPI200_TRANS_CTRL_3 >> i * 8);
		}
		hx83113a_register_write(dev, FLASH_ADDR_SPI200_TRANS_CTRL, DATA_LEN_4, w_data, ADDR_LEN_4);
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(FLASH_DATA_SPI200_CMD_4 >> i * 8);
		}
		hx83113a_register_write(dev, FLASH_ADDR_SPI200_CMD, DATA_LEN_4, w_data, ADDR_LEN_4);
		touch_msleep(1000);

		if (!hx83113a_wait_wip(dev, 100)) {
			TOUCH_E("%s:Erase Fail\n", __func__);
			return false;
		}
	}

	TOUCH_I("%s:END\n", __func__);
	return true;
}

static void hx83113a_flash_programming(struct device *dev, uint8_t *FW_content, int FW_Size)
{
	int page_prog_start = 0, i = 0, j = 0, k = 0;
	int program_length = PROGRAM_SZ;
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t w_data[DATA_LEN_4];
	uint8_t buring_data[FLASH_RW_MAX_LEN];	/* Read for flash data, 128K*/
	/* 4 bytes for padding*/
	hx83113a_interface_on(dev);

	for (i = 0; i < ADDR_LEN_4; i++) {
		w_data[i] = (uint8_t)(FLASH_DATA_SPI200_TRANS_FMT >> i * 8);
	}
	hx83113a_register_write(dev, FLASH_ADDR_SPI200_TRANS_FMT, DATA_LEN_4, w_data, ADDR_LEN_4);

	for (page_prog_start = 0; page_prog_start < FW_Size; page_prog_start += FLASH_RW_MAX_LEN) {
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(FLASH_DATA_SPI200_TRANS_CTRL_2 >> i * 8);
		}
		hx83113a_register_write(dev, FLASH_ADDR_SPI200_TRANS_CTRL, DATA_LEN_4, w_data, ADDR_LEN_4);
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(FLASH_DATA_SPI200_CMD_2 >> i * 8);
		}
		hx83113a_register_write(dev, FLASH_ADDR_SPI200_CMD, DATA_LEN_4, w_data, ADDR_LEN_4);

		 /*Programmable size = 1 page = 256 bytes, word_number = 256 byte / 4 = 64*/
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(FLASH_DATA_SPI200_TRANS_CTRL_4 >> i * 8);
		}
		hx83113a_register_write(dev, FLASH_ADDR_SPI200_TRANS_CTRL, DATA_LEN_4, w_data, ADDR_LEN_4);

		/* Flash start address 1st : 0x0000_0000*/
		if (page_prog_start < 0x100) {
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = (uint8_t)page_prog_start;
		} else if (page_prog_start >= 0x100 && page_prog_start < 0x10000) {
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = (uint8_t)(page_prog_start >> 8);
			tmp_data[0] = (uint8_t)page_prog_start;
		} else if (page_prog_start >= 0x10000 && page_prog_start < 0x1000000) {
			tmp_data[3] = 0x00;
			tmp_data[2] = (uint8_t)(page_prog_start >> 16);
			tmp_data[1] = (uint8_t)(page_prog_start >> 8);
			tmp_data[0] = (uint8_t)page_prog_start;
		}

		hx83113a_register_write(dev, FLASH_ADDR_SPI200_ADDR, DATA_LEN_4, tmp_data, ADDR_LEN_4);

		for (i = 0; i < ADDR_LEN_4; i++) {
			buring_data[i] = (uint8_t)(FLASH_ADDR_SPI200_DATA >> i * 8);
		}

		for (i = page_prog_start, j = 0; i < 16 + page_prog_start; i++, j++) {
			buring_data[j + ADDR_LEN_4] = FW_content[i];
		}

		if (hx83113a_bus_write(dev, IC_ADR_AHB_ADDR_BYTE_0, buring_data, ADDR_LEN_4 + 16) < 0) {
			TOUCH_E("%s: i2c access fail!\n", __func__);
			return;
		}
		for (i = 0; i < ADDR_LEN_4; i++) {
			w_data[i] = (uint8_t)(FLASH_DATA_SPI200_CMD_6 >> i * 8);
		}
		hx83113a_register_write(dev, FLASH_ADDR_SPI200_CMD, DATA_LEN_4, w_data, ADDR_LEN_4);

		for (j = 0; j < 5; j++) {
			for (i = (page_prog_start + 16 + (j * 48)), k = 0; i < (page_prog_start + 16 + (j * 48)) + program_length; i++, k++) {
				buring_data[k + ADDR_LEN_4] = FW_content[i];
			}

			if (hx83113a_bus_write(dev, IC_ADR_AHB_ADDR_BYTE_0, buring_data, program_length + ADDR_LEN_4) < 0) {
				TOUCH_E("%s: i2c access fail!\n", __func__);
				return;
			}
		}

		if (!hx83113a_wait_wip(dev, 1)) {
			TOUCH_E("%s:Flash_Programming Fail\n", __func__);
		}
	}
}

static uint32_t hx83113a_check_CRC(struct device *dev, uint8_t *start_addr, int reload_length)
{
	uint32_t result = 0;
	uint8_t tmp_data[DATA_LEN_4];
	int cnt = 0, ret = 0;
	int length = reload_length / DATA_LEN_4;

	ret = hx83113a_register_write(dev, FW_ADDR_RELOAD_ADDR_FROM, DATA_LEN_4, start_addr, ADDR_LEN_4);
	if (ret < NO_ERR) {
		TOUCH_E("%s: i2c access fail!\n", __func__);
		return HW_CRC_FAIL;
	}

	tmp_data[3] = 0x00; tmp_data[2] = 0x99; tmp_data[1] = (length >> 8); tmp_data[0] = length;
	ret = hx83113a_register_write(dev, FW_ADDR_RELOAD_ADDR_CMD_BEAT, DATA_LEN_4, tmp_data, ADDR_LEN_4);
	if (ret < NO_ERR) {
		TOUCH_E("%s: i2c access fail!\n", __func__);
		return HW_CRC_FAIL;
	}
	cnt = 0;

	do {
		ret = hx83113a_register_read(dev, FW_ADDR_RELOAD_STATUS, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		if (ret < NO_ERR) {
			TOUCH_E("%s: i2c access fail!\n", __func__);
			return HW_CRC_FAIL;
		}

		if ((tmp_data[0] & 0x01) != 0x01) {
			ret = hx83113a_register_read(dev, FW_ADDR_RELOAD_CRC32_RESULT, DATA_LEN_4, tmp_data, ADDR_LEN_4);
			if (ret < NO_ERR) {
				TOUCH_E("%s: i2c access fail!\n", __func__);
				return HW_CRC_FAIL;
			}
			TOUCH_I("%s: tmp_data[3]=%X, tmp_data[2]=%X, tmp_data[1]=%X, tmp_data[0]=%X  \n", __func__, tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
			result = ((tmp_data[3] << 24) + (tmp_data[2] << 16) + (tmp_data[1] << 8) + tmp_data[0]);
			break;
		} else {
			TOUCH_I("Waiting for HW ready!\n");
			touch_msleep(1);
		}

	} while (cnt++ < 100);

	return result;
}

int hx83113a_fts_ctpm_fw_upgrade_with_sys_fs_128k(struct device *dev, unsigned char *fw, int len)
{
	int burnFW_success = 0;
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t w_data[DATA_LEN_4];
	int i = 0;

	if (len != FW_SIZE_128k) {
		TOUCH_E("%s: The file size is not 128K bytes\n", __func__);
		return false;
	}

	hx83113a_ic_reset(dev, false, false);
	hx83113a_sense_off(dev, true);
	hx83113a_block_erase(dev, 0x00, FW_SIZE_128k);
	hx83113a_flash_programming(dev, fw, FW_SIZE_128k);

	for (i = 0; i < ADDR_LEN_4; i++) {
		tmp_data[i] = (uint8_t)(FW_ADDR_PROGRAM_RELOAD_FROM >> i * 8);
	}

	if (hx83113a_check_CRC(dev, tmp_data, FW_SIZE_128k) == 0) {
		burnFW_success = 1;
	}

	/*RawOut select initial*/
	for (i = 0; i < ADDR_LEN_4; i++) {
		w_data[i] = (uint8_t)(FW_DATA_CLEAR >> i * 8);
	}
	hx83113a_register_write(dev, FW_ADDR_RAW_OUT_SEL, DATA_LEN_4, w_data, ADDR_LEN_4);
	/*DSRAM func initial*/
	hx83113a_assign_sorting_mode(dev, w_data);
	hx83113a_ic_reset(dev, false, false);

	return burnFW_success;
}

static int hx83113a_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	char fwpath[256] = {0};
	const struct firmware *fw = NULL;
	int upgrade_times = 0;
	int ret = 0;

	TOUCH_I("%s\n", __func__);

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		return -EPERM;
	}

	if (ts->test_fwpath[0] != 0) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n", &ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath)); // 0 : pramboot bin, 1 : 8607 all bin, 2: 8606 all bin
		TOUCH_I("get fwpath from def_fwpath : %s, count : %d\n", ts->def_fwpath[0], ts->def_fwcnt);
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	if (fwpath == NULL) {
		TOUCH_E("error get fw path\n");
		return -EPERM;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);

	if (ret < 0) {
		TOUCH_I("fail to request_firmware fwpath: %s (ret:%d)\n", fwpath, ret);
		return ret;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	if (hx83113a_fw_compare(dev, fw)) {
update_retry:
		ret = hx83113a_fts_ctpm_fw_upgrade_with_sys_fs_128k(dev, (unsigned char *)fw->data, fw->size);
		if(ret < 0) {
			upgrade_times++;
			TOUCH_E("%s: TP upgrade error, upgrade_times = %d\n", __func__, upgrade_times);
			if(upgrade_times < 3)
				goto update_retry;
			else
				ret = -EPERM;//upgrade fail
		}
		else
		{
			TOUCH_I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
			ret = 0;
		}
	}

	release_firmware(fw);

	return ret;
}
#if defined(CONFIG_LGE_TOUCH_APP_FW_UPGRADE)
static int hx83113a_app_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fw = NULL;
	struct firmware temp_fw;
	int upgrade_times = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		return -EPERM;
	}

	if ((ts->app_fw_upgrade.offset == 0) || (ts->app_fw_upgrade.data == NULL)) {
		TOUCH_E("app_fw_upgrade.offset = %d, app_fw_upgrade.data = %p\n",
				(int)ts->app_fw_upgrade.offset, ts->app_fw_upgrade.data);
		return -EPERM;
	}

	memset(&temp_fw, 0, sizeof(temp_fw));
	temp_fw.size = ts->app_fw_upgrade.offset;
	temp_fw.data = ts->app_fw_upgrade.data;

	fw = &temp_fw;

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	TOUCH_I("fwpath_boot[%s]\n", ts->def_fwpath[0]);  // PARAMBOOT.BIN
	ret = request_firmware(&fw, ts->def_fwpath[0], dev);
	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", ts->def_fwpath[0], ret);
		return ret;
	}

	if (hx83113a_fw_compare(dev, fw)) {
update_retry:
		ret = hx83113a_fts_ctpm_fw_upgrade_with_sys_fs_128k(dev, (unsigned char *)fw->data, fw->size);
		if(ret < 0) {
			upgrade_times++;
			TOUCH_E("%s: TP upgrade error, upgrade_times = %d\n", __func__, upgrade_times);
			if(upgrade_times < 3)
				goto update_retry;
			else
				ret = -EPERM;//upgrade fail
		}
		else
		{
			TOUCH_I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
			ret = 0;
		}
	}

	release_firmware(fw);
	return 0;
}
#endif

bool hx83113a_calculateChecksum(struct device *dev)
{
	uint8_t CRC_result = 0, i;
	uint8_t tmp_data[DATA_LEN_4];

	for (i = 0; i < ADDR_LEN_4; i++) {
		tmp_data[i] = (uint8_t)(SRAM_ADR_RAWDATA_END >> i * 8);
	}

	CRC_result = hx83113a_check_CRC(dev, tmp_data, FW_SIZE_128k);
	touch_msleep(50);

	if (CRC_result != 0) {
		TOUCH_I("%s: CRC Fail=%d\n", __func__, CRC_result);
	}
	return (CRC_result == 0) ? true : false;
}

static bool hx83113a_flash_lastdata_check(struct device *dev)
{
	uint32_t start_addr = 0x1FF80;//0xFF80 DH40
	uint32_t temp_addr = 0;
	uint32_t flash_page_len = 0x80;
	uint8_t flash_tmp_buffer[128];

	for (temp_addr = start_addr; temp_addr < (start_addr + flash_page_len); temp_addr = temp_addr + flash_page_len) {
		/*TOUCH_I("temp_addr=%d,tmp_addr[0]=0x%2X, tmp_addr[1]=0x%2X,tmp_addr[2]=0x%2X,tmp_addr[3]=0x%2X\n", temp_addr,tmp_addr[0], tmp_addr[1], tmp_addr[2],tmp_addr[3]);*/
		/*tmp_addr[0] = temp_addr % 0x100;
		tmp_addr[1] = (temp_addr >> 8) % 0x100;
		tmp_addr[2] = (temp_addr >> 16) % 0x100;
		tmp_addr[3] = temp_addr / 0x1000000;*/
		hx83113a_register_read(dev, temp_addr, flash_page_len, &flash_tmp_buffer[0], ADDR_LEN_4);
	}

	if ((!flash_tmp_buffer[flash_page_len-4]) && (!flash_tmp_buffer[flash_page_len-3]) && (!flash_tmp_buffer[flash_page_len-2]) && (!flash_tmp_buffer[flash_page_len-1])) {
		TOUCH_I("Fail, Last four Bytes are "
		"flash_buffer[FFFC]=0x%2X,flash_buffer[FFFD]=0x%2X,flash_buffer[FFFE]=0x%2X,flash_buffer[FFFF]=0x%2X\n",
		flash_tmp_buffer[flash_page_len-4], flash_tmp_buffer[flash_page_len-3], flash_tmp_buffer[flash_page_len-2], flash_tmp_buffer[flash_page_len-1]);
		return 1;/*FAIL*/
	} else if ((flash_tmp_buffer[flash_page_len-4] == 0xFF) && (flash_tmp_buffer[flash_page_len-3] == 0xFF) && (flash_tmp_buffer[flash_page_len-2] == 0xFF) && (flash_tmp_buffer[flash_page_len-1] == 0xFF)) {
		TOUCH_I("Fail, Last four Bytes are "
		"flash_buffer[FFFC]=0x%2X,flash_buffer[FFFD]=0x%2X,flash_buffer[FFFE]=0x%2X,flash_buffer[FFFF]=0x%2X\n",
		flash_tmp_buffer[flash_page_len-4], flash_tmp_buffer[flash_page_len-3], flash_tmp_buffer[flash_page_len-2], flash_tmp_buffer[flash_page_len-1]);
		return 1;
	} else {
		TOUCH_I("flash_buffer[FFFC]=0x%2X,flash_buffer[FFFD]=0x%2X,flash_buffer[FFFE]=0x%2X,flash_buffer[FFFF]=0x%2X\n",
		flash_tmp_buffer[flash_page_len-4], flash_tmp_buffer[flash_page_len-3], flash_tmp_buffer[flash_page_len-2], flash_tmp_buffer[flash_page_len-1]);
		return 0;/*PASS*/
	}
}

int hx83113a_read_FW_ver(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);

	uint8_t data[DATA_LEN_4];
	uint8_t data_2[DATA_LEN_4];
	int retry = 200;
	int reload_status = 0;
	int ret = 0;

	hx83113a_sense_on(dev, 0x00);

	while (reload_status == 0) {
		hx83113a_register_read(dev, DRIVER_ADDR_FW_DEFINE_FLASH_RELOAD, DATA_LEN_4, data, ADDR_LEN_4);
		hx83113a_register_read(dev, DRIVER_ADDR_FW_DEFINE_2ND_FLASH_RELOAD, DATA_LEN_4, data_2, ADDR_LEN_4);

		if ((data[1] == 0x3A && data[0] == 0xA3)
			|| (data_2[1] == 0x72 && data_2[0] == 0xC0)) {
			TOUCH_I("reload OK! \n");
			reload_status = 1;
			break;
		} else if (retry == 0) {
			TOUCH_E("reload 20 times! fail \n");
			TOUCH_E("Maybe NOT have FW in chipset \n");
			TOUCH_E("Maybe Wrong FW in chipset \n");
			d->ic_info.vendor_panel_ver = 0;
			d->ic_info.vendor_fw_ver = 0;
			d->ic_info.vendor_config_ver = 0;
			d->ic_info.vendor_touch_cfg_ver = 0;
			d->ic_info.vendor_display_cfg_ver = 0;
			d->ic_info.vendor_cid_maj_ver = 0;
			d->ic_info.vendor_cid_min_ver = 0;
			return -EINVAL;
		} else {
			retry--;
			touch_msleep(10);
			if (retry % 10 == 0)
				TOUCH_I("reload fail ,delay 10ms retry=%d\n", retry);
		}
	}

	TOUCH_I("%s : data[0]=0x%2.2X,data[1]=0x%2.2X,data_2[0]=0x%2.2X,data_2[1]=0x%2.2X\n", __func__, data[0], data[1], data_2[0], data_2[1]);
	TOUCH_I("reload_status=%d\n", reload_status);
	/*=====================================
	 Read FW version
	=====================================*/
	hx83113a_sense_off(dev, false);
	hx83113a_register_read(dev, FW_ADDR_FW_VER_ADDR, DATA_LEN_4, data, ADDR_LEN_4);
	d->ic_info.vendor_panel_ver =  data[0];
	d->ic_info.vendor_fw_ver = data[1] << 8 | data[2];

	hx83113a_register_read(dev, FW_ADDR_FW_CFG_ADDR, DATA_LEN_4, data, ADDR_LEN_4);
	d->ic_info.vendor_config_ver = data[2] << 8 | data[3];
	/*TOUCH_I("CFG_VER : %X \n",d->ic_info.vendor_config_ver);*/
	d->ic_info.vendor_touch_cfg_ver = data[2];
	d->ic_info.vendor_display_cfg_ver = data[3];

	hx83113a_register_read(dev, FW_ADDR_FW_VENDOR_ADDR, DATA_LEN_4, data, ADDR_LEN_4);
	d->ic_info.vendor_cid_maj_ver = data[2] ;
	d->ic_info.vendor_cid_min_ver = data[3];

	hx83113a_register_read(dev, FW_ADDR_LGE_FW_VER_ADDR, DATA_LEN_4, data, ADDR_LEN_4);
	d->ic_info.ic_official_ver =  data[0];
	d->ic_info.ic_fw_ver =  data[1];

	TOUCH_I("============ Version Info ============\n");
	TOUCH_I(" IC Version: v%d.%02d, fw_id: %X\n",
			d->ic_info.ic_official_ver, d->ic_info.ic_fw_ver,
			d->ic_info.vendor_fw_ver);
	TOUCH_I(" Bin Version: v%d.%02d, CID_ver: %X\n",
			d->ic_info.bin_official_ver, d->ic_info.bin_fw_ver,
			(d->ic_info.vendor_cid_maj_ver << 8 | d->ic_info.vendor_cid_min_ver));
	TOUCH_I(" Panel_cfg: %X, Touch_cfg: %X, Display_cfg: %X\n",
			d->ic_info.vendor_panel_ver, d->ic_info.vendor_touch_cfg_ver,
			d->ic_info.vendor_display_cfg_ver);
	TOUCH_I("======================================\n");

	return ret;
}
void hx83113a_print_FW_ver(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);
	TOUCH_I("============ Version Info ============\n");
	TOUCH_I(" IC Version: v%d.%02d, fw_id: %X\n",
			d->ic_info.ic_official_ver, d->ic_info.ic_fw_ver,
			d->ic_info.vendor_fw_ver);
	TOUCH_I(" Bin Version: v%d.%02d, CID_ver: %X\n",
			d->ic_info.bin_official_ver, d->ic_info.bin_fw_ver,
			(d->ic_info.vendor_cid_maj_ver << 8 | d->ic_info.vendor_cid_min_ver));
	TOUCH_I(" Panel_cfg: %X, Touch_cfg: %X, Display_cfg: %X\n",
			d->ic_info.vendor_panel_ver, d->ic_info.vendor_touch_cfg_ver,
			d->ic_info.vendor_display_cfg_ver);
	TOUCH_I("======================================\n");
	return;
}

void hx83113a_touch_information(struct device *dev)
{
#ifndef HX_FIX_TOUCH_INFO
	char data[DATA_LEN_8] = {0};

	hx83113a_register_read(dev, DRIVER_ADDR_FW_DEFINE_RXNUM_TXNUM_MAXPT, DATA_LEN_8, data, ADDR_LEN_4);
	ic_data->HX_RX_NUM				= data[2];
	ic_data->HX_TX_NUM				= data[3];
	ic_data->HX_MAX_PT				= data[4];
	/*TOUCH_I("%s : HX_RX_NUM=%d,ic_data->HX_TX_NUM=%d,ic_data->HX_MAX_PT=%d\n",__func__,ic_data->HX_RX_NUM,ic_data->HX_TX_NUM,ic_data->HX_MAX_PT);*/
	hx83113a_register_read(dev, DRIVER_ADDR_FW_DEFINE_XY_RES_ENABLE, DATA_LEN_4, data, ADDR_LEN_4);

	/*TOUCH_I("%s : c_data->HX_XY_REVERSE=0x%2.2X\n",__func__,data[1]);*/
	if ((data[1] & 0x04) == 0x04) {
		ic_data->HX_XY_REVERSE = true;
	} else {
		ic_data->HX_XY_REVERSE = false;
	}

	hx83113a_register_read(dev, DRIVER_ADDR_FW_DEFINE_X_Y_RES, DATA_LEN_4, data, ADDR_LEN_4);
	ic_data->HX_Y_RES = data[0] * 256 + data[1];
	ic_data->HX_X_RES = data[2] * 256 + data[3];
	/*TOUCH_I("%s : ic_data->HX_Y_RES=%d,ic_data->HX_X_RES=%d \n",__func__,ic_data->HX_Y_RES,ic_data->HX_X_RES);*/

	hx83113a_register_read(dev, DRIVER_ADDR_FW_DEFINE_INT_IS_EDGE, DATA_LEN_4, data, ADDR_LEN_4);
	/*TOUCH_I("%s : data[0]=0x%2.2X,data[1]=0x%2.2X,data[2]=0x%2.2X,data[3]=0x%2.2X\n",__func__,data[0],data[1],data[2],data[3]);
	TOUCH_I("data[0] & 0x01 = %d\n",(data[0] & 0x01));*/
	if ((data[1] & 0x01) == 1) {
		ic_data->HX_INT_IS_EDGE = true;
	} else {
		ic_data->HX_INT_IS_EDGE = false;
	}

	if (ic_data->HX_RX_NUM > 48) {
		ic_data->HX_RX_NUM = 32;
	}

	if (ic_data->HX_TX_NUM > 30) {
		ic_data->HX_TX_NUM = 18;
	}

	if (ic_data->HX_MAX_PT > 10) {
		ic_data->HX_MAX_PT = 10;
	}

	if (ic_data->HX_Y_RES > 2880) {
		ic_data->HX_Y_RES = 1920;
	}

	if (ic_data->HX_X_RES > 2000) {
		ic_data->HX_X_RES = 1080;
	}

	/*1. Read number of MKey R100070E8H to determin data size*/
	hx83113a_register_read(dev, SRAM_ADR_MKEY, DATA_LEN_4, data, ADDR_LEN_4);
	/* TOUCH_I("%s: tmp_data[0] = 0x%02X,tmp_data[1] = 0x%02X,tmp_data[2] = 0x%02X,tmp_data[3] = 0x%02X\n",
	 __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);*/
	ic_data->HX_BT_NUM = data[0] & 0x03;
#else
	ic_data->HX_RX_NUM				= FIX_HX_RX_NUM;
	ic_data->HX_TX_NUM				= FIX_HX_TX_NUM;
	ic_data->HX_BT_NUM				= FIX_HX_BT_NUM;
	ic_data->HX_X_RES				= FIX_HX_X_RES;
	ic_data->HX_Y_RES				= FIX_HX_Y_RES;
	ic_data->HX_MAX_PT				= FIX_HX_MAX_PT;
	ic_data->HX_XY_REVERSE			= FIX_HX_XY_REVERSE;
	ic_data->HX_INT_IS_EDGE			= FIX_HX_INT_IS_EDGE;
#endif
	TOUCH_I("%s:HX_RX_NUM =%d,HX_TX_NUM =%d,HX_MAX_PT=%d \n", __func__, ic_data->HX_RX_NUM, ic_data->HX_TX_NUM, ic_data->HX_MAX_PT);
	TOUCH_I("%s:HX_XY_REVERSE =%d,HX_Y_RES =%d,HX_X_RES=%d \n", __func__, ic_data->HX_XY_REVERSE, ic_data->HX_Y_RES, ic_data->HX_X_RES);
	TOUCH_I("%s:HX_INT_IS_EDGE =%d \n", __func__, ic_data->HX_INT_IS_EDGE);
}

static int hx83113a_swipe_enable(struct device *dev, bool enable)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct swipe_ctrl *ctrl[HX83113A_SWIPE_NUM] = {	/* U, D, L, R */
		&ts->swipe[SWIPE_U],
		&ts->swipe[SWIPE_D],
		&ts->swipe[SWIPE_L],
		&ts->swipe[SWIPE_R],
	};
	int i, ret = 0;
	uint32_t swipe_mode_addr[HX83113A_SWIPE_NUM] =
	{FW_ADDR_SWIPE_UP, FW_ADDR_SWIPE_DOWN, FW_ADDR_SWIPE_LEFT, FW_ADDR_SWIPE_RIGHT};
	uint8_t data[SWIPE_INFO_NUM] = {0x0, };

	TOUCH_I("%s: SWIPE U(%d),D(%d),L(%d),R(%d)\n", __func__,
			ctrl[HX83113A_SWIPE_U]->enable,
			ctrl[HX83113A_SWIPE_D]->enable,
			ctrl[HX83113A_SWIPE_L]->enable,
			ctrl[HX83113A_SWIPE_R]->enable);

	if (enable) {
		for (i = 0; i < HX83113A_SWIPE_NUM; i++) {
			if (i == HX83113A_SWIPE_D)
				continue;
			data[0] = ctrl[i]->enable;
			data[1] = ctrl[i]->distance;
			data[2] = ctrl[i]->ratio_thres;
			data[3] = ctrl[i]->min_time >> 8;
			data[4] = ctrl[i]->min_time & 0xFF;
			data[5] = ctrl[i]->max_time >> 8;
			data[6] = ctrl[i]->max_time & 0xFF;
			data[7] = ctrl[i]->area.x1 >> 8;
			data[8] = ctrl[i]->area.x1 & 0xFF;
			data[9] = ctrl[i]->area.y1 >> 8;
			data[10] = ctrl[i]->area.y1 & 0xFF;
			data[11] = ctrl[i]->area.x2 >> 8;
			data[12] = ctrl[i]->area.x2 & 0xFF;
			data[13] = ctrl[i]->area.y2 >> 8;
			data[14] = ctrl[i]->area.y2 & 0xFF;
			data[15] = ctrl[i]->start_area.x1 >> 8;
			data[16] = ctrl[i]->start_area.x1 & 0xFF;
			data[17] = ctrl[i]->start_area.y1 >> 8;
			data[18] = ctrl[i]->start_area.y1 & 0xFF;
			data[19] = ctrl[i]->start_area.x2 >> 8;
			data[20] = ctrl[i]->start_area.x2 & 0xFF;
			data[21] = ctrl[i]->start_area.y2 >> 8;
			data[22] = ctrl[i]->start_area.y2 & 0xFF;
			data[23] = ctrl[i]->wrong_dir_thres;
			data[24] = ctrl[i]->init_ratio_chk_dist;
			data[25] = ctrl[i]->init_ratio_thres;

			ret = hx83113a_register_write(dev, swipe_mode_addr[i], sizeof(data), data, ADDR_LEN_4);
			if (ret < 0) {
				TOUCH_E("failed to register write %d (ret: %d)\n", i, ret);
				goto error;
			}
		}
	} else {
		for (i = 0; i < HX83113A_SWIPE_NUM; i++) {	/* U, D, L, R*/
			data[0] = 0;
			ret = hx83113a_register_write(dev, swipe_mode_addr[i], ADDR_LEN_4, data, ADDR_LEN_4);
			if (ret < 0) {
				TOUCH_E("failed to register write %d (ret: %d)\n", i, ret);
				goto error;

			}
		}
	}

error:
	return ret;
}
#ifdef HX_P_SENSOR
static int himax_mcu_ulpm_in(struct device *dev)
{
	uint8_t tmp_data[4];
	int rtimes = 0;

	TOUCH_I("%s : START\n", __func__);

	/* 34 -> 11 */
	do {
		if (rtimes > 10) {
			TOUCH_D(TRACE, "%s:1/7 retry over 10 times!\n", __func__);
			return false;
		}
		tmp_data[0] = fw_data_ulpm_11;
		if (hx83113a_bus_write(dev, fw_addr_ulpm_34, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 1/7 write fail!\n", __func__);
			continue;
		}
		if (hx83113a_bus_read(dev, fw_addr_ulpm_34, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 1/7 read fail!\n", __func__);
			continue;
		}

		TOUCH_I("%s:retry times %d, addr = 0x34, correct 0x11 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != fw_data_ulpm_11);

	rtimes = 0;
	/* 34 -> 11 */
	do {
		if (rtimes > 10) {
			TOUCH_D(TRACE, "%s:2/7 retry over 10 times!\n", __func__);
			return false;
		}
		tmp_data[0] = fw_data_ulpm_11;
		if (hx83113a_bus_write(dev, fw_addr_ulpm_34, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 2/7 write fail!\n", __func__);
			continue;
		}
		if (hx83113a_bus_read(dev, fw_addr_ulpm_34, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 2/7 read fail!\n", __func__);
			continue;
		}

		TOUCH_I("%s:retry times %d, addr = 0x34, correct 0x11 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != fw_data_ulpm_11);

	/* 33 -> 33 */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TOUCH_D(TRACE, "%s:3/7 retry over 10 times!\n", __func__);
			return false;
		}
		tmp_data[0] = fw_data_ulpm_33;
		if (hx83113a_bus_write(dev, fw_addr_ulpm_33, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 3/7 write fail!\n", __func__);
			continue;
		}
		if (hx83113a_bus_read(dev, fw_addr_ulpm_33, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 3/7 read fail!\n", __func__);
			continue;
		}

		TOUCH_I("%s:retry times %d, addr = 0x33, correct 0x33 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != fw_data_ulpm_33);

	/* 34 -> 22 */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TOUCH_D(TRACE, "%s:4/7 retry over 10 times!\n", __func__);
			return false;
		}
		tmp_data[0] = fw_data_ulpm_22;
		if (hx83113a_bus_write(dev, fw_addr_ulpm_34, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 4/7 write fail!\n", __func__);
			continue;
		}
		if (hx83113a_bus_read(dev, fw_addr_ulpm_34, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 4/7 read fail!\n", __func__);
			continue;
		}

		TOUCH_I("%s:retry times %d, addr = 0x34, correct 0x22 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != fw_data_ulpm_22);

	/* 33 -> AA */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TOUCH_D(TRACE, "%s:5/7 retry over 10 times!\n", __func__);
			return false;
		}
		tmp_data[0] = fw_data_ulpm_aa;
		if (hx83113a_bus_write(dev, fw_addr_ulpm_33, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 5/7 write fail!\n", __func__);
			continue;
		}
		if (hx83113a_bus_read(dev, fw_addr_ulpm_33, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 5/7 read fail!\n", __func__);
			continue;
		}

		TOUCH_I("%s:retry times %d, addr = 0x33, correct 0xAA = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != fw_data_ulpm_aa);

	/* 33 -> 33 */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TOUCH_D(TRACE, "%s:6/7 retry over 10 times!\n", __func__);
			return false;
		}
		tmp_data[0] = fw_data_ulpm_33;
		if (hx83113a_bus_write(dev, fw_addr_ulpm_33, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 6/7 write fail!\n", __func__);
			continue;
		}
		if (hx83113a_bus_read(dev, fw_addr_ulpm_33, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 6/7 read fail!\n", __func__);
			continue;
		}

		TOUCH_I("%s:retry times %d, addr = 0x33, correct 0x33 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != fw_data_ulpm_33);

	/* 33 -> AA */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TOUCH_D(TRACE, "%s:7/7 retry over 10 times!\n", __func__);
			return false;
		}
		tmp_data[0] = fw_data_ulpm_aa;
		if (hx83113a_bus_write(dev, fw_addr_ulpm_33, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 7/7 write fail!\n", __func__);
			continue;
		}
		if (hx83113a_bus_read(dev, fw_addr_ulpm_33, tmp_data, 1) < 0) {
			TOUCH_D(TRACE, "%s: 7/7 read fail!\n", __func__);
			continue;
		}

		TOUCH_I("%s:retry times %d, addr = 0x33, correct 0xAA = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != fw_data_ulpm_aa);

	TOUCH_I("%s : END\n", __func__);

	return true;
}

static int hx83113a_deep_sleep(struct device *dev, bool onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	/* on = deep sleep in, off = deep sleep out */
	if (onoff) {
		touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
		himax_mcu_ulpm_in(dev);
		atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
		TOUCH_I("IC status = IC_DEEP_SLEEP");
	} else {
		hx83113a_ic_reset(dev, false, false);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		atomic_set(&ts->state.sleep, IC_NORMAL);
		TOUCH_I("IC status = IC_NORMAL");
	}

	return 0;
}
#else
static int hx83113a_deep_sleep(struct device *dev, bool onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (onoff) {
		touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
		//himax_mcu_ulpm_in(dev);
		hx83113a_sense_off(dev, true);
		atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
		TOUCH_I("IC status = IC_DEEP_SLEEP");
	} else {
		//hx83113a_ic_reset(dev, false, false);
		hx83113a_sense_on(dev, 0x00);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		atomic_set(&ts->state.sleep, IC_NORMAL);
		TOUCH_I("IC status = IC_NORMAL");
	}

	return 0;
}
#endif

#define ACT_SENSELESS_AREA_W	5 //5mm // 54 pixel
static int hx83113a_tci_active_area(struct device *dev)
{
	int ret = 0;
	uint8_t tci_data[ADDR_LEN_4] = {0, };

	TOUCH_I("%s\n", __func__);

	ret = hx83113a_register_read(dev, FW_ADDR_TCI_AREA, ADDR_LEN_4, tci_data, ADDR_LEN_4);
	if (ret < 0) {
		TOUCH_E("failed to register read (ret: %d)\n", ret);
		goto error;
	}

	tci_data[0] = ACT_SENSELESS_AREA_W;
	tci_data[1] = ACT_SENSELESS_AREA_W;
	ret = hx83113a_register_write(dev, FW_ADDR_TCI_AREA, ADDR_LEN_4, tci_data, ADDR_LEN_4);
	if (ret < 0)
		TOUCH_E("failed to register write (ret: %d)\n", ret);

error:
	return ret;
}

static int hx83113a_tci_knock(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info = &ts->tci.info[TCI_1];
	uint8_t tmp_data[DATA_LEN_4] = {0,};
	u8 lpwg_data[4] = {0,};
	int ret = 0;

	TOUCH_I("%s\n", __func__);

     ret = hx83113a_register_read(dev, FW_ADDR_TCI_MODE, DATA_LEN_4, tmp_data, ADDR_LEN_4);
	if (ret < 0) {
		TOUCH_E("failed to register read (ret: %d)\n", ret);
		goto error;
	}

	tmp_data[0] = ts->tci.mode;
	ret = hx83113a_register_write(dev, FW_ADDR_TCI_MODE, DATA_LEN_4, tmp_data, ADDR_LEN_4);
	if (ret < 0) {
		TOUCH_E("failed to register write (ret: %d)\n", ret);
		goto error;
	}

	if (!ts->tci.mode)
		return ret;

	lpwg_data[0] = info->tap_distance;
	lpwg_data[1] = info->max_intertap;
	lpwg_data[2] = info->min_intertap;
	lpwg_data[3] = info->touch_slop;

	ret = hx83113a_register_write(dev, FW_ADDR_TCI_SET, ADDR_LEN_4, lpwg_data, ADDR_LEN_4);
	if (ret < 0) {
		TOUCH_E("failed to register write (ret: %d)\n", ret);
		goto error;
	}

	ret = hx83113a_tci_active_area(dev);
	if (ret < 0)
		TOUCH_E("failed to set tci_active_area (ret: %d)\n", ret);

error:
	return ret;
}

static int hx83113a_lpwg_control(struct device *dev, int tci_mode, int swipe_enable)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	int ret = 0;

	TOUCH_I("%s - lpwg.mode=%d, swipe=%d", __func__, tci_mode, swipe_enable);

	switch (tci_mode) {
	case LPWG_NONE:
		ts->tci.mode = 0x00;
		TOUCH_I("%s LPWG_DOUBLE_TAP Disable\n", __func__);
		ret = hx83113a_tci_knock(dev);
		if (ret < 0) {
			TOUCH_E("failed to set LPWG_NONE (ret: %d)\n", ret);
			goto error;
		}
		break;
	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x02;
		info1->intr_delay = 0;
		info1->tap_distance = 10;

		TOUCH_I("%s LPWG_DOUBLE_TAP ENABLE\n", __func__);
		ret = hx83113a_tci_knock(dev);
		if (ret < 0) {
			TOUCH_E("failed to set LPWG_DOUBLE_TAP (ret: %d)\n", ret);
			goto error;
		}
		break;
	default:
		TOUCH_E("Unknown lpwg control case = %d\n", tci_mode);
		break;
	}

	ret = hx83113a_swipe_enable(dev, swipe_enable);
	if (ret < 0)
		TOUCH_E("failed to set swipe enable (ret: %d)\n", ret);

error:
	return ret;
}

static int hx83113a_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	int ret = 0;
	TOUCH_I("%s, fb : %s\n", __func__,atomic_read(&ts->state.fb) ? "FB_SUSPEND" : "FB_RESUME");

	/* FB_SUSPEND */
	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->mfts_lpwg){
			TOUCH_I("FB_SUSPEND == mfts_lpwg\n");
			ret = hx83113a_lpwg_control(dev, LPWG_DOUBLE_TAP, SWIPE_ENABLE);
			if (ret < 0) {
				TOUCH_E("failed to set lpwg control (ret: %d)\n", ret);
				goto error;
			}
			return 0;
		}

		if (ts->lpwg.screen) {
			TOUCH_I("Skip lpwg setting\n");
			hx83113a_SMWP_dbg_msg(dev);
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			TOUCH_I("suspend ts->lpwg.sensor == PROX_NEAR\n");
			if (atomic_read(&ts->state.sleep) == IC_NORMAL) {
				ret = hx83113a_deep_sleep(dev, IC_DEEP_SLEEP);
				if (ret < 0) {
					TOUCH_E("failed to set IC_DEEP_SLEEP (ret: %d)\n", ret);
					goto error;
				}
			}
		} else if (ts->lpwg.qcover == HALL_NEAR) {
			/* set qcover action */
			TOUCH_I("suspend ts->lpwg.qcover == HALL_NEAR\n");
			if (atomic_read(&ts->state.sleep) == IC_NORMAL) {
				ret = hx83113a_deep_sleep(dev, IC_DEEP_SLEEP);
				if (ret < 0) {
					TOUCH_E("failed to set IC_DEEP_SLEEP (ret: %d)\n", ret);
					goto error;
				}
			}
		} else {
			/* Knock on Case */
			if (ts->lpwg.mode == LPWG_NONE &&
					!ts->swipe[SWIPE_U].enable &&
					!ts->swipe[SWIPE_L].enable &&
					!ts->swipe[SWIPE_R].enable) {
				TOUCH_I("suspend ts->lpwg.mode == LPWG_NONE\n");
				if (atomic_read(&ts->state.sleep) == IC_NORMAL) {
					ret = hx83113a_deep_sleep(dev, IC_DEEP_SLEEP);
					if (ret < 0) {
						TOUCH_E("failed to set IC_DEEP_SLEEP (ret: %d)\n", ret);
						goto error;
					}
				}
			} else {
				if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
					ret = hx83113a_deep_sleep(dev, IC_NORMAL);
					if (ret < 0) {
						TOUCH_E("failed to set IC_NORMAL (ret: %d)\n", ret);
						goto error;
					}

				}

				ret = hx83113a_lpwg_control(dev, ts->lpwg.mode,
						(ts->swipe[SWIPE_U].enable ||
						 ts->swipe[SWIPE_L].enable ||
						 ts->swipe[SWIPE_R].enable));
				if (ret < 0) {
					TOUCH_E("failed to set lpwg control (ret: %d)\n", ret);
					goto error;
				}

			}
		}
		return 0;
	}

	touch_report_all_event(ts);

	/* FB_RESUME */
	if (ts->lpwg.screen) {
		TOUCH_I("resume ts->lpwg.screen on\n");
		if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
			ret = hx83113a_deep_sleep(dev, IC_NORMAL);
			if (ret < 0) {
				TOUCH_E("failed to set IC_NORMAL (ret: %d)\n", ret);
				goto error;
			}
		}

		ret = hx83113a_lpwg_control(dev, LPWG_NONE, SWIPE_DISABLE);
		if (ret < 0) {
			TOUCH_E("failed to set lpwg control (ret: %d)\n", ret);
			goto error;
		}
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		TOUCH_I("resume ts->lpwg.sensor == PROX_NEAR\n");
		if (atomic_read(&ts->state.sleep) == IC_NORMAL) {
			ret = hx83113a_deep_sleep(dev, IC_DEEP_SLEEP);
			if (ret < 0) {
				TOUCH_E("failed to set IC_DEEP_SLEEP (ret: %d)\n", ret);
				goto error;
			}
		}
	} else if (ts->lpwg.qcover == HALL_NEAR) {
			/* set qcover action */
		TOUCH_I("suspend ts->lpwg.qcover == HALL_NEAR\n");
		if (atomic_read(&ts->state.sleep) == IC_NORMAL) {
			ret = hx83113a_deep_sleep(dev, IC_DEEP_SLEEP);
			if (ret < 0) {
				TOUCH_E("failed to set IC_DEEP_SLEEP (ret: %d)\n", ret);
				goto error;
			}
		}
	} else {
		TOUCH_I("resume partial\n");
		if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
			ret = hx83113a_deep_sleep(dev, IC_NORMAL);
			if (ret < 0) {
				TOUCH_E("failed to set IC_NORMAL (ret: %d)\n", ret);
				goto error;
			}
		}

		ret = hx83113a_lpwg_control(dev, ts->lpwg.mode,
				(ts->swipe[SWIPE_U].enable ||
				 ts->swipe[SWIPE_L].enable ||
				 ts->swipe[SWIPE_R].enable));
		if (ret < 0) {
			TOUCH_E("failed to set lpwg control (ret: %d)\n", ret);
			goto error;
		}
	}

error:
	return ret;
}

static int hx83113a_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int *value = (int *)param;
	int ret = 0;

	TOUCH_TRACE();

	switch (code) {
	case LPWG_ACTIVE_AREA:
		ts->tci.area.x1 = value[0];
		ts->tci.area.x2 = value[1];
		ts->tci.area.y1 = value[2];
		ts->tci.area.y2 = value[3];
		TOUCH_I("LPWG_ACTIVE_AREA: x0[%d], x1[%d], x2[%d], x3[%d]\n",
			value[0], value[1], value[2], value[3]);
		break;

	case LPWG_TAP_COUNT:
		ts->tci.info[TCI_2].tap_count = value[0];
		TOUCH_I("LPWG_TAP_COUNT: [%d]\n", value[0]);
		break;

	case LPWG_DOUBLE_TAP_CHECK:
		ts->tci.double_tap_check = value[0];
		TOUCH_I("LPWG_DOUBLE_TAP_CHECK: [%d]\n", value[0]);
		break;

	case LPWG_UPDATE_ALL:
		ts->lpwg.mode = value[0];
		ts->lpwg.screen = value[1];
		ts->lpwg.sensor = value[2];
		ts->lpwg.qcover = value[3];

		TOUCH_I(
			"LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
			value[0],
			value[1] ? "ON" : "OFF",
			value[2] ? "FAR" : "NEAR",
			value[3] ? "CLOSE" : "OPEN");

		ret = hx83113a_lpwg_mode(dev);
		if (ret < 0) {
			TOUCH_E("failed to set lpwg mode (ret: %d)\n", ret);
			return ret;
		}
		break;

	case LPWG_REPLY:
		break;

	}

	return 0;
}

static void hx83113a_lcd_mode(struct device *dev, u32 mode)
{
	struct himax_ts_data *d = to_himax_data(dev);

	TOUCH_I("lcd_mode: %d (prev: %d)\n", mode, d->lcd_mode);

	if (mode == LCD_MODE_U2_UNBLANK)
		mode = LCD_MODE_U2;

	d->lcd_mode = mode;
}

static int hx83113a_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *ev = (struct fb_event *)data;

	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == FB_BLANK_UNBLANK) {
			TOUCH_I("FB_UNBLANK\n");
		} else if (*blank == FB_BLANK_POWERDOWN) {
			TOUCH_I("FB_BLANK\n");
		}
	}

	return 0;
}

static int hx83113a_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	bool update_flag = false;
	int ret = 0;
	TOUCH_I("%s\n", __func__);

	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		TOUCH_I("fb_notif change\n");
		fb_unregister_client(&ts->fb_notif);
		ts->fb_notif.notifier_call = hx83113a_fb_notifier_callback;
		fb_register_client(&ts->fb_notif);
		update_flag = (!hx83113a_calculateChecksum(dev));
		update_flag |= hx83113a_flash_lastdata_check(dev);
		if (update_flag) {
			ts->force_fwup = 1;
			TOUCH_I("Calculate CRC fail - force fwup:%d\n", ts->force_fwup);
		}
		hx83113a_read_FW_ver(dev);
		hx83113a_sense_on(dev, 0x00);
		hx83113a_set_SMWP_enable(dev, true);
	}
	if (atomic_read(&ts->state.core) == CORE_UPGRADE) {
		hx83113a_read_FW_ver(dev);
		hx83113a_touch_information(dev);
		hx83113a_sense_on(dev, 0x00);
	}

   	hx83113a_print_FW_ver(dev);

	hx83113a_cable_connect(dev);
	ret = hx83113a_lpwg_mode(dev);
	if (ret < 0)
		TOUCH_E("failed to set lpwg mode (ret: %d)\n", ret);

	return ret;
}

static void hx83113a_calcDataSize(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);

	d->x_channel = ic_data->HX_RX_NUM;
	d->y_channel = ic_data->HX_TX_NUM;
	d->nFinger_support = ic_data->HX_MAX_PT;

	d->coord_data_size = 4 * d->nFinger_support;
	d->area_data_size = ((d->nFinger_support / 4) + (d->nFinger_support % 4 ? 1 : 0)) * 4;
	d->coordInfoSize = d->coord_data_size + d->area_data_size + 4;

	TOUCH_I("%s: coord_data_size: %d, area_data_size:%d\n", __func__, d->coord_data_size, d->area_data_size);
}

static void hx83113a_calculate_point_number(void)
{
	HX_TOUCH_INFO_POINT_CNT = ic_data->HX_MAX_PT * 4;

	if ((ic_data->HX_MAX_PT % 4) == 0) {
		HX_TOUCH_INFO_POINT_CNT += (ic_data->HX_MAX_PT / 4) * 4;
	} else {
		HX_TOUCH_INFO_POINT_CNT += ((ic_data->HX_MAX_PT / 4) + 1) * 4;
	}
}

static void hx83113a_esd_ic_reset(struct device *dev)
{
	hx83113a_hw_reset(dev);
	TOUCH_I("%s:\n", __func__);
}

static void hx83113a_esd_hw_reset(struct device *dev)
{
	if (g_ts_dbg != 0)
		TOUCH_I("%s: Entering \n", __func__);

	TOUCH_I("START_Himax TP: ESD - Reset\n");
	hx83113a_esd_ic_reset(dev);
	TOUCH_I("END_Himax TP: ESD - Reset\n");
}

static int hx83113a_parse_tci_data(struct device *dev, uint8_t *buf, uint8_t tap_count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int i = 0;
	int j = 0;

	TOUCH_I("%s Tap Count %d \n",__func__, tap_count);
	if (tap_count > MAX_LPWG_CODE)
		tap_count = MAX_LPWG_CODE;

	ts->lpwg.code_num = tap_count;
	for (i = 0; i < tap_count; i++) {
		j = i*4+2;
		ts->lpwg.code[i].x = (int)(buf[GEST_STATE_COOR+1+(i*4)]<<8 | buf[GEST_STATE_COOR+(i*4)]);
		ts->lpwg.code[i].y = (int)(buf[GEST_STATE_COOR+3+(i*4)]<<8 | buf[GEST_STATE_COOR+2+(i*4)]);

		TOUCH_I("LPWG data %d, %d\n", ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[tap_count].x = -1;
	ts->lpwg.code[tap_count].y = -1;

	return 0;
}

static int hx83113a_parse_swipe_data(struct device *dev, uint8_t *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int count = 2;
	int time = 0;

	ts->lpwg.code_num = count;
	ts->lpwg.code[0].x = (int)(buf[GEST_STATE_COOR+1]<<8 | buf[GEST_STATE_COOR]);
	ts->lpwg.code[0].y = (int)(buf[GEST_STATE_COOR+3]<<8 | buf[GEST_STATE_COOR+2]);
	ts->lpwg.code[1].x = (int)(buf[GEST_STATE_COOR+5]<<8 | buf[GEST_STATE_COOR+4]);
	ts->lpwg.code[1].y = (int)(buf[GEST_STATE_COOR+7]<<8 | buf[GEST_STATE_COOR+6]);

	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	time = (int)(buf[GEST_FAIL_STATUS + 1]<<8 | buf[GEST_FAIL_STATUS]);

	TOUCH_I("Swipe Gesture: start(%4d,%4d) end(%4d,%4d) swipe_time(%dms)\n",
			ts->lpwg.code[0].x, ts->lpwg.code[0].y,
			ts->lpwg.code[1].x, ts->lpwg.code[1].y,
			time);

	return 0;
}

static int hx83113a_parse_lpwg_debug_data(struct device *dev, uint8_t *buf, uint8_t tap_count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int i = 0;
	int j = 0;
	u8 debug_type = 0;
	u8 fail_msg = 0;

	/* Return if valid ID or empty msg */
	for (i = 0; i < GEST_SUP_NUM ;i++) {
		if (buf[0] == gest_event[i]) {
			TOUCH_I("Skip debug msg, valid ID : [%d]\n", buf[0]);
			return 0;
		} else if (buf[0] == 0 && buf[GEST_PTLG_HDR_LEN] == 0) {
			return 0;
		}
	}

	TOUCH_I("%s Tap Count %d \n",__func__, tap_count);

	if (tap_count > MAX_LPWG_CODE)
		tap_count = MAX_LPWG_CODE;

	ts->lpwg.code_num = tap_count;
	for (i = 0; i < tap_count; i++) {
		j = i*4+2;
		ts->lpwg.code[i].x = (int)(buf[GEST_STATE_COOR+1+(i*4)]<<8 | buf[GEST_STATE_COOR+(i*4)]);
		ts->lpwg.code[i].y = (int)(buf[GEST_STATE_COOR+3+(i*4)]<<8 | buf[GEST_STATE_COOR+2+(i*4)]);

		TOUCH_I("LPWG data %d, %d\n", ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[tap_count].x = -1;
	ts->lpwg.code[tap_count].y = -1;

	debug_type = buf[GEST_FAIL_TYPE_MSG] >> 4;
	if (debug_type == 1) { //Knock on fail
		fail_msg = buf[GEST_FAIL_TYPE_MSG] & 0xF;
		TOUCH_I("Knock-on Failure Reason Reported : %d [%s]\n", fail_msg, tci_debug_str[fail_msg]);
	}	else if (debug_type == 2) { //Swipe on fail
		fail_msg = buf[GEST_FAIL_TYPE_MSG] & 0xF;
		TOUCH_I("SWIPE Failure Reason Reported : %d [%s]\n", fail_msg, tci_swipe_debug_str[fail_msg]);
	} else {
		TOUCH_I("LPWG Tpye : [%d]\n", debug_type);
	}

	return 0;
}

static int hx83113a_wake_event_parse(struct device *dev, int ts_status)
{
	uint8_t *buf;
	int i = 0, check_FC = 0;
	int j = 0, gesture_pos = 0, gesture_flag = -1;
	uint8_t tap_count;

	if (g_ts_dbg != 0)
		TOUCH_I("%s: Entering!, ts_status=%d\n", __func__, ts_status);

	buf = kzalloc(hx_touch_data->event_size * sizeof(uint8_t), GFP_KERNEL);
	if (buf == NULL) {
		return -ENOMEM;
	}

	memcpy(buf, hx_touch_data->hx_event_buf, hx_touch_data->event_size);

	for (i = 0; i < GEST_PTLG_ID_LEN; i++) {
		for (j = 0; j < GEST_SUP_NUM; j++) {
			if (buf[i] == gest_event[j]) {
				gesture_flag = buf[i];
				gesture_pos = j;
				break;
			}
		}
		TOUCH_D(IRQ_HANDLE, "0x%2.2X ", buf[i]);
		if (buf[i] == gesture_flag) {
			check_FC++;
		} else {
			TOUCH_D(IRQ_HANDLE, "ID START at %x , value = 0x%2X skip the event\n", i, buf[i]);
			break;
		}
	}

	TOUCH_D(IRQ_HANDLE, "Himax gesture_flag = %x, gesture_pos = %x\n", gesture_flag, gesture_pos);
	TOUCH_D(IRQ_HANDLE, "Himax check_FC is %d\n", check_FC);

	tap_count = buf[GEST_PT_COUNT];
	if (tap_count == 0) {
		TOUCH_I("%s No point count packet \n",__func__);
		kfree(buf);
		return gesture_flag;
	}

	if (check_FC != GEST_PTLG_ID_LEN) {
		hx83113a_parse_lpwg_debug_data(dev, buf, tap_count);
		kfree(buf);
		return 0;
	}

	if (buf[GEST_PTLG_HDR_LEN] != GEST_PTLG_HDR_ID1 ||
		buf[GEST_PTLG_HDR_LEN + 1] != GEST_PTLG_HDR_ID2) {
		hx83113a_parse_lpwg_debug_data(dev, buf, tap_count);
		kfree(buf);
		return 0;
	}

	if (gesture_pos == 0) { //Knock on
		hx83113a_parse_tci_data(dev, buf, tap_count);
		g_target_report_data->SMWP_event_chk = TOUCH_IRQ_KNOCK;
	} else if (gesture_pos == 1) { //Swipe UP
		hx83113a_parse_swipe_data(dev, buf);
		g_target_report_data->SMWP_event_chk = TOUCH_IRQ_SWIPE_UP;
	} else if (gesture_pos == 2) { //Swipe DOWN
		TOUCH_I("%s Swipe DOWN is Not support \n",__func__);
	}	else if (gesture_pos == 3) { //Swipe LEFT
		hx83113a_parse_swipe_data(dev, buf);
		g_target_report_data->SMWP_event_chk = TOUCH_IRQ_SWIPE_LEFT2;
	} else if (gesture_pos == 4) { //Swipe RIGHT
		hx83113a_parse_swipe_data(dev, buf);
		g_target_report_data->SMWP_event_chk = TOUCH_IRQ_SWIPE_RIGHT2;
	}
//	else {
//		hx83113a_parse_lpwg_debug_data(dev, buf, tap_count);
//	}

	kfree(buf);

	return gesture_pos;
}

static void hx83113a_wake_event_report(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int IRQ_EVENT = g_target_report_data->SMWP_event_chk;

	if (g_ts_dbg != 0)
		TOUCH_I("%s: Entering! \n", __func__);

	if (IRQ_EVENT) {
		ts->intr_status = IRQ_EVENT;
		TOUCH_I(" %s SMART WAKEUP KEY event %d\n", __func__, IRQ_EVENT);
		g_target_report_data->SMWP_event_chk = 0;
	}
}

static int hx83113a_get_touch_data_size(void)
{
	return HIMAX_TOUCH_DATA_SIZE;
}

static int hx83113a_cal_data_len(int raw_cnt_rmd, int HX_MAX_PT, int raw_cnt_max)
{
	int RawDataLen;

	if (raw_cnt_rmd != 0x00) {
		RawDataLen = MAX_I2C_TRANS_SZ - ((HX_MAX_PT + raw_cnt_max + 3) * 4) - 1;
	} else {
		RawDataLen = MAX_I2C_TRANS_SZ - ((HX_MAX_PT + raw_cnt_max + 2) * 4) - 1;
	}

	return RawDataLen;
}

int hx83113a_report_data_init(void)
{
	if (hx_touch_data->hx_coord_buf != NULL) {
		kfree(hx_touch_data->hx_coord_buf);
	}

	if (hx_touch_data->hx_width_buf != NULL) {
		kfree(hx_touch_data->hx_width_buf);
	}

	if (hx_touch_data->hx_rawdata_buf != NULL) {
		kfree(hx_touch_data->hx_rawdata_buf);
	}

	hx_touch_data->event_size = hx83113a_get_touch_data_size();

	if (hx_touch_data->hx_event_buf != NULL) {
		kfree(hx_touch_data->hx_event_buf);
	}

	hx_touch_data->touch_all_size = hx83113a_get_touch_data_size();
	hx_touch_data->raw_cnt_max = ic_data->HX_MAX_PT / 4;
	hx_touch_data->raw_cnt_rmd = ic_data->HX_MAX_PT % 4;
	/* more than 4 fingers */
	if (hx_touch_data->raw_cnt_rmd != 0x00) {
		hx_touch_data->rawdata_size = hx83113a_cal_data_len(hx_touch_data->raw_cnt_rmd, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max);
		hx_touch_data->touch_info_size = (ic_data->HX_MAX_PT + hx_touch_data->raw_cnt_max + 2) * 4;
	} else { /* less than 4 fingers */
		hx_touch_data->rawdata_size = hx83113a_cal_data_len(hx_touch_data->raw_cnt_rmd, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max);
		hx_touch_data->touch_info_size = (ic_data->HX_MAX_PT + hx_touch_data->raw_cnt_max + 1) * 4;
	}

	TOUCH_I("%s: ic_data->HX_MAX_PT:%d, hx_raw_cnt_max:%d, hx_raw_cnt_rmd:%d, g_hx_rawdata_size:%d, hx_touch_data->touch_info_size:%d\n", __func__, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max, hx_touch_data->raw_cnt_rmd, hx_touch_data->rawdata_size, hx_touch_data->touch_info_size);
	hx_touch_data->hx_coord_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->touch_info_size), GFP_KERNEL);

	if (hx_touch_data->hx_coord_buf == NULL) {
		goto mem_alloc_fail;
	}

	hx_touch_data->hx_width_buf = kzalloc(sizeof(uint8_t) * CUST_TP_SHAPE_SIZE, GFP_KERNEL);

	if (hx_touch_data->hx_width_buf == NULL) {
		goto mem_alloc_fail;
	}

	if (g_target_report_data == NULL) {
		g_target_report_data = kzalloc(sizeof(struct himax_target_report_data), GFP_KERNEL);
		if (g_target_report_data == NULL)
			goto mem_alloc_fail;
		g_target_report_data->x = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->x == NULL)
			goto mem_alloc_fail;
		g_target_report_data->y = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->y == NULL)
			goto mem_alloc_fail;
		g_target_report_data->w = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->w == NULL)
			goto mem_alloc_fail;
		g_target_report_data->width_major = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->width_major == NULL)
			goto mem_alloc_fail;
		g_target_report_data->width_minor = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->width_minor == NULL)
			goto mem_alloc_fail;
		g_target_report_data->orientation = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->orientation == NULL)
			goto mem_alloc_fail;
		g_target_report_data->finger_id = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->finger_id == NULL)
			goto mem_alloc_fail;
	}
	g_target_report_data->SMWP_event_chk = 0;

	//hx_touch_data->hx_rawdata_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->touch_all_size - hx_touch_data->touch_info_size), GFP_KERNEL);
	hx_touch_data->hx_rawdata_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->touch_all_size - CUST_TP_SIZE), GFP_KERNEL);

	if (hx_touch_data->hx_rawdata_buf == NULL) {
		goto mem_alloc_fail;
	}

	hx_touch_data->hx_event_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->event_size), GFP_KERNEL);

	if (hx_touch_data->hx_event_buf == NULL) {
		goto mem_alloc_fail;
	}

	return NO_ERR;
mem_alloc_fail:
	kfree(g_target_report_data->x);
	kfree(g_target_report_data->y);
	kfree(g_target_report_data->w);
	kfree(g_target_report_data->width_major);
	kfree(g_target_report_data->width_minor);
	kfree(g_target_report_data->orientation);
	kfree(g_target_report_data->finger_id);
	kfree(g_target_report_data);
	g_target_report_data = NULL;
	kfree(hx_touch_data->hx_width_buf);
	kfree(hx_touch_data->hx_coord_buf);
	kfree(hx_touch_data->hx_rawdata_buf);
	kfree(hx_touch_data->hx_event_buf);

	TOUCH_I("%s: Memory allocate fail!\n", __func__);
	return MEM_ALLOC_FAIL;
}

/*start ts_work*/
static int hx83113a_ts_work_status(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);
	/* 1: normal, 2:SMWP */
	int result = HX_REPORT_COORD;

	hx_touch_data->diag_cmd = d->diag_cmd;
	if (hx_touch_data->diag_cmd)
		result = HX_REPORT_COORD_RAWDATA;

	if (atomic_read(&d->suspend_mode) && (d->SMWP_enable) && (!hx_touch_data->diag_cmd))
		result = HX_REPORT_SMWP_EVENT;
	/* TOUCH_I("Now Status is %d\n", result); */
	return result;
}

static bool hx83113a_read_event_stack(struct device *dev, uint8_t *buf, uint8_t length)
{
	//uint8_t cmd[DATA_LEN_4];
	/*  AHB_I2C Burst Read Off */
	/*cmd[0] = FW_DATA_AHB_DIS;

	if (hx83113a_bus_write(dev, FW_ADDR_AHB_ADDR, cmd, 1) < 0) {
		TOUCH_E("%s: i2c access fail!\n", __func__);
		return 0;
	}*/

	hx83113a_bus_read(dev, FW_ADDR_EVENT_ADDR, buf, length);
	/*  AHB_I2C Burst Read On */
	//cmd[0] = FW_DATA_AHB_EN;

	/*if (hx83113a_bus_write(dev, FW_ADDR_AHB_ADDR, cmd, 1) < 0) {
		TOUCH_E("%s: i2c access fail!\n", __func__);
		return 0;
	}*/

	return 1;
}

#if 0
static void hx83113a_touch_info_get(struct device *dev)
{
	uint8_t cnt = 50;

	//tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x05; tmp_addr[0] = 0x78;
	do {
		hx83113a_register_read(dev, DRIVER_ADDR_FW_WIDTH_ANGLE, 40, hx_touch_data->hx_width_buf, ADDR_LEN_4);
		//TOUCH_I("0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X\n", hx_touch_data->hx_width_buf[0], hx_touch_data->hx_width_buf[1], hx_touch_data->hx_width_buf[2], hx_touch_data->hx_width_buf[3]);
		cnt--;
	} while (hx_touch_data->hx_width_buf[3] == 0 && cnt > 0);
}
#endif

static int hx83113a_touch_get(struct device *dev, uint8_t *buf, int ts_path, int ts_status)
{
	if (g_ts_dbg != 0)
		TOUCH_I("%s: Entering, ts_status=%d! \n", __func__, ts_status);

	switch (ts_path) {
	/*normal*/
	case HX_REPORT_COORD:
		if (!hx83113a_read_event_stack(dev, buf, CUST_TP_SIZE)) { //hx_touch_data->touch_info_size
			TOUCH_E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
			goto END_FUNCTION;
		}
		/* read touch width info from SRAM */
		/*hx83113a_touch_info_get(dev);*/
		break;

	/*SMWP*/
	case HX_REPORT_SMWP_EVENT:
		hx83113a_burst_enable(dev, 0);

		if (!hx83113a_read_event_stack(dev, buf, hx_touch_data->event_size)) {
			TOUCH_E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
			goto END_FUNCTION;
		}
		break;

	case HX_REPORT_COORD_RAWDATA:
		if (!hx83113a_read_event_stack(dev, buf, 128)) {
			TOUCH_E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
			goto END_FUNCTION;
		}
		break;
	default:
		break;
	}

END_FUNCTION:
	return ts_status;
}

/* start error_control*/
static int hx83113a_checksum_cal(struct device *dev, uint8_t *buf, int ts_path, int ts_status)
{
	uint16_t check_sum_cal = 0;
	int32_t	i = 0;
	int length = 0;
	int zero_cnt = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		TOUCH_I("%s: Entering, ts_status=%d! \n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = hx_touch_data->touch_info_size;
		break;
/* SMWP */
	case HX_REPORT_SMWP_EVENT:
		length = (GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN);
		break;
	case HX_REPORT_COORD_RAWDATA:
		length = hx_touch_data->touch_info_size;
		break;
	default:
		TOUCH_I("%s, Neither Normal Nor SMWP error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	for (i = 0; i < length; i++) {
		//TOUCH_I("P %2d = 0x%2.2X ", i, buf[i]);
		check_sum_cal += buf[i];
		if (buf[i] == 0x00)
			zero_cnt++;
	}

	if (check_sum_cal % 0x100 != 0) {
		if (g_ts_dbg != 0)
			TOUCH_I("[HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n", check_sum_cal);
		ret_val = HX_CHKSUM_FAIL;
	} else if (zero_cnt == length) {
		if (g_ts_dbg != 0)
			TOUCH_I("[HIMAX TP MSG] All Zero event\n");
		ret_val = HX_CHKSUM_FAIL;
	}

	/*Check sum for shape info*/
	if (ts_path == HX_REPORT_COORD) {// || ts_path == HX_REPORT_COORD_RAWDATA
		for (i = 0; i < CUST_TP_SHAPE_SIZE; i++) {
		//TOUCH_I("P %2d = 0x%2.2X ", i, buf[i]);
			check_sum_cal += buf[i + length];
		}
		if (check_sum_cal % 0x100 != 0) {
			if (g_ts_dbg != 0)
				TOUCH_I("shape info :checksum fail : 0x%02X\n", check_sum_cal);
			ret_val = HX_CHKSUM_FAIL;
		}
	}

END_FUNCTION:
	if (g_ts_dbg != 0)
		TOUCH_I("%s: END, ret_val=%d! \n", __func__, ret_val);
	return ret_val;
}

static int hx83113a_ic_esd_recovery(int hx_esd_event, int hx_zero_event, int length)
{
	int ret_val = NO_ERR;

	if (g_zero_event_count > 5) {
		g_zero_event_count = 0;
		if (g_ts_dbg != 0)
			TOUCH_I("[HIMAX TP MSG]: ESD event checked - ALL Zero.\n");
		ret_val = HX_ESD_EVENT;
		goto END_FUNCTION;
	}

	if (hx_esd_event == length) {
		g_zero_event_count = 0;
		ret_val = HX_ESD_EVENT;
		goto END_FUNCTION;
	} else if (hx_zero_event == length) {
		g_zero_event_count++;
		if (g_ts_dbg != 0)
			TOUCH_I("[HIMAX TP MSG]: ALL Zero event is %d times.\n", g_zero_event_count);
		ret_val = HX_ZERO_EVENT_COUNT;
		goto END_FUNCTION;
	}

END_FUNCTION:
	return ret_val;
}

static int hx83113a_esd_recovery(struct device *dev)
{
	TOUCH_TRACE();
#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY_ESD) || IS_ENABLED(CONFIG_LGE_TOUCH_PANEL_GLOBAL_RESET)
	lge_mdss_report_panel_dead();
#endif
#endif

#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
	mtkfb_esd_recovery();
#endif

	TOUCH_I("hx83113a_esd_recovery!!\n");
	return 0;
}

static int hx83113a_ts_event_check(struct device *dev, uint8_t *buf, int ts_path, int ts_status)
{
	int hx_EB_event = 0;
	int hx_EC_event = 0;
	int hx_ED_event = 0;
	int hx_esd_event = 0;
	int hx_zero_event = 0;
	int shaking_ret = 0;

	int32_t	loop_i = 0;
	int length = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		TOUCH_I("%s: Entering, ts_status=%d! \n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = hx_touch_data->touch_info_size;
		break;
/* SMWP */
	case HX_REPORT_SMWP_EVENT:
		length = (GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN);
		break;
	case HX_REPORT_COORD_RAWDATA:
		length = hx_touch_data->touch_info_size;
		break;
	default:
		TOUCH_I("%s, Neither Normal Nor SMWP error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	if (g_ts_dbg != 0)
		TOUCH_I("Now Path=%d, Now status=%d, length=%d\n", ts_path, ts_status, length);

	for (loop_i = 0; loop_i < length; loop_i++) {
		if (ts_path == HX_REPORT_COORD || ts_path == HX_REPORT_COORD_RAWDATA) {
			/* case 1 ESD recovery flow */
			if (buf[loop_i] == 0xEB) {
				hx_EB_event++;
			} else if (buf[loop_i] == 0xEC) {
				hx_EC_event++;
			} else if (buf[loop_i] == 0xED) {
				hx_ED_event++;
			} else if (buf[loop_i] == 0x00) { /* case 2 ESD recovery flow-Disable */
				hx_zero_event++;
			} else {
				hx_EB_event = 0;
				hx_EC_event = 0;
				hx_ED_event = 0;
				hx_zero_event = 0;
				g_zero_event_count = 0;
			}
		}
	}

	if (hx_EB_event == length) {
		hx_esd_event = length;
		hx_EB_event_flag++;
		if (g_ts_dbg != 0)
			TOUCH_I("[HIMAX TP MSG]: ESD event checked - ALL 0xEB.\n");
	} else if (hx_EC_event == length) {
		hx_esd_event = length;
		hx_EC_event_flag++;
		if (g_ts_dbg != 0)
			TOUCH_I("[HIMAX TP MSG]: ESD event checked - ALL 0xEC.\n");
	} else if (hx_ED_event == length) {
		hx_esd_event = length;
		hx_ED_event_flag++;
		if (g_ts_dbg != 0)
			TOUCH_I("[HIMAX TP MSG]: ESD event checked - ALL 0xED.\n");
	} else {
		hx_esd_event = 0;
	}

	if ((hx_esd_event == length || hx_zero_event == length)
		&& (hx_touch_data->diag_cmd == 0)) {
		shaking_ret = hx83113a_ic_esd_recovery(hx_esd_event, hx_zero_event, length);

		if (shaking_ret == HX_ESD_EVENT) {
			hx83113a_esd_hw_reset(dev);
			ret_val = HX_ESD_EVENT;
		} else if (shaking_ret == HX_ZERO_EVENT_COUNT) {
			ret_val = HX_ZERO_EVENT_COUNT;
		} else {
			TOUCH_I("I2C running. Nothing to be done!\n");
			ret_val = HX_IC_RUNNING;
		}
	}

END_FUNCTION:
	if (g_ts_dbg != 0)
		TOUCH_I("%s: END, ret_val=%d! \n", __func__, ret_val);

	return ret_val;
}

static int hx83113a_err_ctrl(struct device *dev, uint8_t *buf, int ts_path, int ts_status)
{
	ts_status = hx83113a_checksum_cal(dev, buf, ts_path, ts_status);
	if (ts_status == HX_CHKSUM_FAIL) {
		goto CHK_FAIL;
	} else {
		/* continuous N times record, not total N times. */
		g_zero_event_count = 0;
		goto END_FUNCTION;
	}
CHK_FAIL:
	ts_status = hx83113a_ts_event_check(dev, buf, ts_path, ts_status);

END_FUNCTION:
	if (g_ts_dbg != 0)
		TOUCH_I("%s: END, ts_status=%d! \n", __func__, ts_status);
	return ts_status;
}
/* end error_control*/

/* start distribute_data*/
static int hx83113a_distribute_touch_data(uint8_t *buf, int ts_path, int ts_status)
{
	uint8_t hx_state_info_pos = hx_touch_data->touch_info_size - 3;

	if (g_ts_dbg != 0)
		TOUCH_I("%s: Entering, ts_status=%d! \n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD) {
		memcpy(hx_touch_data->hx_coord_buf, &buf[0], hx_touch_data->touch_info_size);
		memcpy(hx_touch_data->hx_width_buf, &buf[hx_touch_data->touch_info_size], CUST_TP_SHAPE_SIZE);

		if (buf[hx_state_info_pos] != 0xFF && buf[hx_state_info_pos + 1] != 0xFF) {
			memcpy(hx_touch_data->hx_state_info, &buf[hx_state_info_pos], 2);
		} else {
			memset(hx_touch_data->hx_state_info, 0x00, sizeof(hx_touch_data->hx_state_info));
		}
	} else if (ts_path == HX_REPORT_COORD_RAWDATA) {
		memcpy(hx_touch_data->hx_coord_buf, &buf[0], hx_touch_data->touch_info_size);
		memcpy(hx_touch_data->hx_width_buf, &buf[hx_touch_data->touch_info_size], CUST_TP_SHAPE_SIZE);

		if (buf[hx_state_info_pos] != 0xFF && buf[hx_state_info_pos + 1] != 0xFF) {
			memcpy(hx_touch_data->hx_state_info, &buf[hx_state_info_pos], 2);
		} else {
			memset(hx_touch_data->hx_state_info, 0x00, sizeof(hx_touch_data->hx_state_info));
		}
		//memcpy(hx_touch_data->hx_rawdata_buf, &buf[CUST_TP_SIZE], hx_touch_data->touch_all_size - CUST_TP_SIZE);
		memcpy(hx_touch_data->hx_rawdata_buf, &buf[hx_touch_data->touch_info_size], hx_touch_data->touch_all_size - hx_touch_data->touch_info_size);
	} else if (ts_path == HX_REPORT_SMWP_EVENT) {
		memcpy(hx_touch_data->hx_event_buf, buf, hx_touch_data->event_size);
		memset(hx_touch_data->hx_state_info, 0x00, sizeof(hx_touch_data->hx_state_info));
	} else {
		TOUCH_E("%s, Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		TOUCH_I("%s: End, ts_status=%d! \n", __func__, ts_status);
	return ts_status;
}
/* end assign_data*/

/* start parse_report_data*/
int hx83113a_parse_report_points(struct device *dev, int ts_path, int ts_status)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = to_himax_data(dev);
	int x = 0, y = 0, w = 0;
	int wM = 0, wm = 0, angle = 1;
	int base = 0;
	int32_t	loop_i = 0;

	if (g_ts_dbg != 0)
		TOUCH_I("%s: start! \n", __func__);


//grip supression misoperation fix
	wMm_value = (wMm_value==10)? 11: 10;
	wM = wMm_value;
	wm = wMm_value;
	d->old_finger = d->pre_finger_mask;
	d->pre_finger_mask = 0;
	hx_touch_data->finger_num = hx_touch_data->hx_coord_buf[d->coordInfoSize - 4] & 0x0F;
	hx_touch_data->finger_on = 1;
	AA_press = 1;

	g_target_report_data->finger_num = hx_touch_data->finger_num;
	g_target_report_data->finger_on = hx_touch_data->finger_on;

	if (g_ts_dbg != 0)
		TOUCH_I("%s:finger_num = 0x%2X, finger_on = %d \n", __func__, g_target_report_data->finger_num, g_target_report_data->finger_on);

	for (loop_i = 0; loop_i < d->nFinger_support; loop_i++) {
		base = loop_i * 4;
		x = hx_touch_data->hx_coord_buf[base] << 8 | hx_touch_data->hx_coord_buf[base + 1];
		y = (hx_touch_data->hx_coord_buf[base + 2] << 8 | hx_touch_data->hx_coord_buf[base + 3]);
		w = hx_touch_data->hx_coord_buf[(d->nFinger_support * 4) + loop_i];
//grip supression misoperation fix
		//wM = hx_touch_data->hx_width_buf[base] << 4 | (hx_touch_data->hx_width_buf[base + 1] & 0xF0) >> 4;
		//wm = (hx_touch_data->hx_width_buf[base + 1] & 0x0F) << 8 | hx_touch_data->hx_width_buf[base + 2];
		angle = hx_touch_data->hx_width_buf[base + 3];

		if (g_ts_dbg != 0)
			TOUCH_I("%s: now parsing[%d]:x=%d, y=%d, w=%d\n", __func__, loop_i, x, y, w);

		if (x >= 0 && x <= ts->caps.max_x && y >= 0 && y <= ts->caps.max_y) {
			hx_touch_data->finger_num--;

			g_target_report_data->x[loop_i] = x;
			g_target_report_data->y[loop_i] = y;
			g_target_report_data->w[loop_i] = w;
			g_target_report_data->width_major[loop_i] = wM;
			g_target_report_data->width_minor[loop_i] = wm;
			g_target_report_data->orientation[loop_i] = angle;
			g_target_report_data->finger_id[loop_i] = 1;

			/* TOUCH_I("%s: g_target_report_data->x[loop_i]=%d, g_target_report_data->y[loop_i]=%d, g_target_report_data->w[loop_i]=%d", */
			/* 	__func__, g_target_report_data->x[loop_i], g_target_report_data->y[loop_i], g_target_report_data->w[loop_i]); */


			if (!d->first_pressed) {
				d->first_pressed = 1;
				TOUCH_I("S1@%d, %d\n", x, y);
			}

			d->pre_finger_data[loop_i][0] = x;
			d->pre_finger_data[loop_i][1] = y;

			d->pre_finger_mask = d->pre_finger_mask + (1 << loop_i);
		} else {/* report coordinates */
			g_target_report_data->x[loop_i] = x;
			g_target_report_data->y[loop_i] = y;
			g_target_report_data->w[loop_i] = w;
			g_target_report_data->width_major[loop_i] = wM;
			g_target_report_data->width_minor[loop_i] = wm;
			g_target_report_data->orientation[loop_i] = angle;
			g_target_report_data->finger_id[loop_i] = 0;

			if (loop_i == 0 && d->first_pressed == 1) {
				d->first_pressed = 2;
				TOUCH_I("E1@%d, %d\n", d->pre_finger_data[0][0], d->pre_finger_data[0][1]);
			}
		}
	}

	if (g_ts_dbg != 0) {
		for (loop_i = 0; loop_i < 10; loop_i++) {
			TOUCH_I("DBG X=%d  Y=%d ID=%d\n", g_target_report_data->x[loop_i], g_target_report_data->y[loop_i], g_target_report_data->finger_id[loop_i]);
		}
		TOUCH_I("DBG finger number %d\n", g_target_report_data->finger_num);
	}

	if (g_ts_dbg != 0)
		TOUCH_I("%s: end! \n", __func__);
	return ts_status;
}

static int hx83113a_parse_report_data(struct device *dev, int ts_path, int ts_status)
{
	struct himax_ts_data *d = to_himax_data(dev);

	if (g_ts_dbg != 0)
		TOUCH_I("%s: start now_status=%d! \n", __func__, ts_status);

	p_point_num = d->hx_point_num;

	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = hx83113a_parse_report_points(dev, ts_path, ts_status);
		break;
	case HX_REPORT_COORD_RAWDATA:
		/* touch monitor rawdata */
		if (debug_data != NULL) {
			if (debug_data->fp_set_diag_cmd(ic_data, hx_touch_data))
				TOUCH_I("%s: coordinate dump fail and bypass with checksum err\n", __func__);
		} else {
			TOUCH_E("%s,There is no init set_diag_cmd\n", __func__);
		}
		ts_status = hx83113a_parse_report_points(dev, ts_path, ts_status);
		break;
	case HX_REPORT_SMWP_EVENT:
		hx83113a_wake_event_parse(dev, ts_status);
		break;
	default:
		TOUCH_E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
		break;
	}
	if (g_ts_dbg != 0)
		TOUCH_I("%s: end now_status=%d! \n", __func__, ts_status);
	return ts_status;
}

/* end parse_report_data*/

/* start report_point*/
static void hx83113a_finger_report(struct device *dev)
{
	struct touch_data *tdata;
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = to_himax_data(dev);
	int i = 0;
	bool valid = false;
	uint8_t finger_index = 0;

	if (g_ts_dbg != 0) {
		TOUCH_I("%s:start\n", __func__);
		TOUCH_I("hx_touch_data->finger_num=%d\n", hx_touch_data->finger_num);
	}
	ts->new_mask = 0;
	for (i = 0; i < d->nFinger_support; i++) {
		if (g_target_report_data->x[i] >= 0 && g_target_report_data->x[i] <= ts->caps.max_x && g_target_report_data->y[i] >= 0 && g_target_report_data->y[i] <= ts->caps.max_y)
			valid = true;
		else
			valid = false;
		if (g_ts_dbg != 0)
			TOUCH_I("valid=%d\n", valid);
		if (valid) {
			if (g_ts_dbg != 0)
				TOUCH_I("g_target_report_data->x[i]=%d, g_target_report_data->y[i]=%d, g_target_report_data->w[i]=%d\n", g_target_report_data->x[i], g_target_report_data->y[i], g_target_report_data->w[i]);
			ts->new_mask |= (1 << i);
			tdata = ts->tdata;
			tdata[i].id = i;
			tdata[i].type = MT_TOOL_FINGER;
			tdata[i].x = g_target_report_data->x[i];
			tdata[i].y = g_target_report_data->y[i];
			tdata[i].pressure = g_target_report_data->w[i];
			tdata[i].width_major = g_target_report_data->width_major[i];
			tdata[i].width_minor = g_target_report_data->width_minor[i];
			if (g_target_report_data->width_major[i] == g_target_report_data->width_minor[i])
				tdata[i].orientation = 1;
			else
				tdata[i].orientation = (s16)(g_target_report_data->orientation[i] - 90);
			finger_index++;

			TOUCH_D(ABS, "tdata[ID:%2d Type:%2d X:%4d Y:%4d Z:%4d WM:%4d Wm:%4d Angle:%4d]\n",
					tdata[i].id,
					tdata[i].type,
					tdata[i].x,
					tdata[i].y,
					tdata[i].pressure,
					tdata[i].width_major,
					tdata[i].width_minor,
					tdata[i].orientation);
		}
	}

	ts->intr_status = TOUCH_IRQ_FINGER;
	ts->tcount = finger_index;

	if (g_ts_dbg != 0)
		TOUCH_I("%s:end\n", __func__);
}

static void hx83113a_finger_leave(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = to_himax_data(dev);

	if (g_ts_dbg != 0)
		TOUCH_I("%s: start! \n", __func__);

	hx_touch_data->finger_on = 0;
	AA_press = 0;

	if (d->pre_finger_mask > 0) {
		d->pre_finger_mask = 0;
	}

	if (d->first_pressed == 1) {
		d->first_pressed = 2;
		TOUCH_I("E1@%d, %d\n", d->pre_finger_data[0][0], d->pre_finger_data[0][1]);
	}

	ts->new_mask = 0;
	ts->intr_status = TOUCH_IRQ_FINGER;
	ts->tcount = 0;

	if (g_ts_dbg != 0)
		TOUCH_I("%s: end! \n", __func__);
	return;
}

static void hx83113a_report_points(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);

	if (g_ts_dbg != 0)
		TOUCH_I("%s: start! \n", __func__);

	if (d->hx_point_num != 0) {
		hx83113a_finger_report(dev);
	} else {
		hx83113a_finger_leave(dev);
	}

	if (g_ts_dbg != 0)
		TOUCH_I("%s: end! \n", __func__);
}

static int hx83113a_report_data(struct device *dev, int ts_path, int ts_status)
{
	struct himax_ts_data *d = to_himax_data(dev);

	if (g_ts_dbg != 0)
		TOUCH_I("%s: Entering, ts_status=%d! \n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD || ts_path == HX_REPORT_COORD_RAWDATA) {
		if (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] == 0xff) {
			d->hx_point_num = 0;
		} else {
			d->hx_point_num = hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;
		}
		hx83113a_report_points(dev);
		ts_status = HX_REPORT_DATA;
	} else if (ts_path == HX_REPORT_SMWP_EVENT) {
		//wake_lock_timeout(&d->ts_SMWP_wake_lock, TS_WAKE_LOCK_TIMEOUT);
		hx83113a_wake_event_report(dev);
		ts_status = HX_REPORT_DATA;
	} else {
		TOUCH_E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		TOUCH_I("%s: END, ts_status=%d! \n", __func__, ts_status);
	return ts_status;
}
/* end report_data */

static int hx83113a_ts_operation(struct device *dev, int ts_path, int ts_status)
{
	struct touch_core_data *ts = to_touch_core(dev);

	uint8_t hw_reset_check[2];
	uint8_t buf[128];

	memset(buf, 0x00, sizeof(buf));
	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));

	ts_status = hx83113a_touch_get(dev, buf, ts_path, ts_status);
	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto END_FUNCTION;

	ts_status = hx83113a_err_ctrl(dev, buf, ts_path, ts_status);
	if (ts_status == HX_REPORT_DATA || ts_status == HX_TS_NORMAL_END) {
		ts_status = hx83113a_distribute_touch_data(buf, ts_path, ts_status);
		ts_status = hx83113a_parse_report_data(dev, ts_path, ts_status);

		if (hx_touch_data->hx_state_info[0] & 0x08) { /* palm detect */
			if (hx_touch_data->hx_palm == 0) {
				hx_touch_data->hx_palm = 1;
				TOUCH_I("Palm Detected\n");
				ts->is_cancel = 1;
				ts->tcount = 0;
				ts->intr_status = TOUCH_IRQ_FINGER;
				goto END_FUNCTION;
			}
		} else if (hx_touch_data->hx_palm) {
			hx_touch_data->hx_palm = 0;
			TOUCH_I("Palm Released\n");
			ts->is_cancel = 1;
			ts->tcount = 0;
			ts->intr_status = TOUCH_IRQ_FINGER;
			goto END_FUNCTION;
		}
	} else {
		goto END_FUNCTION;
	}
	ts_status = hx83113a_report_data(dev, ts_path, ts_status);


END_FUNCTION:
	return ts_status;
}

static int hx83113a_irq_handler(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);

	int ts_status = HX_TS_NORMAL_END;
	int ts_path = 0;

	if (debug_data != NULL)
		debug_data->fp_ts_dbg_func(d, HX_FINGER_ON);

	ts_path = hx83113a_ts_work_status(dev);
	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = hx83113a_ts_operation(dev, ts_path, ts_status);
		break;
	case HX_REPORT_SMWP_EVENT:
		ts_status = hx83113a_ts_operation(dev, ts_path, ts_status);
		break;
	case HX_REPORT_COORD_RAWDATA:
		ts_status = hx83113a_ts_operation(dev, ts_path, ts_status);
		break;
	default:
		TOUCH_E("%s:Path Fault! value=%d\n", __func__, ts_path);
		goto END_FUNCTION;
		break;
	}

	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto GET_TOUCH_FAIL;
	else
		goto END_FUNCTION;

GET_TOUCH_FAIL:
	TOUCH_I("%s: Now reset the Touch chip.\n", __func__);
	return -EHWRESET_ASYNC;
END_FUNCTION:
	if (debug_data != NULL)
		debug_data->fp_ts_dbg_func(d, HX_FINGER_LEAVE);

	return 0;
}

void hx83113a_set_SMWP_enable(struct device *dev, uint8_t SMWP_enable)
{
	uint8_t tmp_data[DATA_LEN_4] = {0, };
	uint8_t back_data[DATA_LEN_4] = {0, };
	uint8_t w_data[DATA_LEN_4] = {0, };
	int i = 0;
	uint8_t retry_cnt = 0;

	TOUCH_I("%s\n", __func__);

	do {
		if (SMWP_enable) {
			for (i = 0; i < ADDR_LEN_4; i++) {
				w_data[i] = (uint8_t)(FW_FUNC_HANDSHAKING_PWD >> i * 8);
			}
		} else {
			for (i = 0; i < ADDR_LEN_4; i++) {
				w_data[i] = (uint8_t)(FW_DATA_SAFE_MODE_RELEASE_PW_RESET >> i * 8);
			}
		}
		hx83113a_register_write(dev, FW_ADDR_SMWP_ENABLE, DATA_LEN_4, w_data, ADDR_LEN_4);

		hx83113a_register_read(dev, FW_ADDR_SMWP_ENABLE, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		/*TOUCH_I("%s: tmp_data[0]=%d, SMWP_enable=%d, retry_cnt=%d \n", __func__, tmp_data[0],SMWP_enable,retry_cnt);*/
		retry_cnt++;
	} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1]  || tmp_data[0] != back_data[0]) && retry_cnt < HIMAX_REG_RETRY_TIMES);
}

void hx83113a_SMWP_dbg_msg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int read_len = HX_MAX_MSG_NUM * HX_MSG_SIZE;
	u8 tap_count = 2;
	int i = 0;


	if (ts->lpwg.mode) {
		hx83113a_register_read(dev, HX_ADDR_DBG_KNOCK_ON, read_len, DBG_BUF, ADDR_LEN_4);
		for (i = 0; i < read_len; i = i + HX_MSG_SIZE)
			hx83113a_parse_lpwg_debug_data(dev, &DBG_BUF[i], tap_count);
	}
	if (ts->swipe[SWIPE_U].enable ||
			ts->swipe[SWIPE_L].enable ||
			ts->swipe[SWIPE_R].enable) {
		hx83113a_register_read(dev, HX_ADDR_DBG_SWIPE, read_len, DBG_BUF, ADDR_LEN_4);
		for (i = 0; i < read_len; i = i + HX_MSG_SIZE)
			hx83113a_parse_lpwg_debug_data(dev, &DBG_BUF[i], tap_count);
	}
}

static int hx83113a_suspend(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);
	struct touch_core_data *ts = to_touch_core(dev);

	int boot_mode = TOUCH_NORMAL_BOOT;

	TOUCH_TRACE();

	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
	case TOUCH_MINIOS_AAT:
		break;
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
		if (!ts->mfts_lpwg) {
			TOUCH_I("%s : touch_suspend - MFTS\n", __func__);
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			hx83113a_power(dev, POWER_OFF);
			return -EPERM;
		}
		break;
	case TOUCH_CHARGER_MODE:
	case TOUCH_LAF_MODE:
	case TOUCH_RECOVERY_MODE:
		TOUCH_I("%s: Etc boot_mode(%d)!!!\n", __func__, boot_mode);
		return -EPERM;
	default:
		TOUCH_E("%s: invalid boot_mode = %d\n", __func__, boot_mode);
		return -EPERM;
	}

	TOUCH_I("%s : touch_suspend start\n", __func__);

	if (debug_data != NULL && debug_data->flash_dump_going == true) {
		TOUCH_I("[himax] %s: Flash dump is going, reject suspend\n", __func__);
		return 0;
	}

	atomic_set(&d->suspend_mode, 1);
	hx_touch_data->hx_palm = 0;
	d->pre_finger_mask = 0;
	hx83113a_lpwg_mode(dev);

	return 0;
}

static int hx83113a_resume(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = to_himax_data(dev);

	int boot_mode = TOUCH_NORMAL_BOOT;

	TOUCH_TRACE();

	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
	case TOUCH_MINIOS_AAT:
			hx83113a_ic_reset(dev,false,false);
		break;
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
		if (!ts->mfts_lpwg) {
			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			hx83113a_power(dev, POWER_ON);
			touch_msleep(ts->caps.hw_reset_delay);
			hx83113a_read_FW_ver(dev);
			hx83113a_sense_on(dev, 0x00);
            hx83113a_set_SMWP_enable(dev, true);
		}
		break;
	case TOUCH_CHARGER_MODE:
	case TOUCH_LAF_MODE:
	case TOUCH_RECOVERY_MODE:
		TOUCH_I("%s: Etc boot_mode(%d)!!!\n", __func__, boot_mode);
		touch_interrupt_control(dev, INTERRUPT_DISABLE);
		hx83113a_power(dev, POWER_OFF);
		return -EPERM;
	default:
		TOUCH_E("%s: invalid boot_mode = %d\n", __func__, boot_mode);
		return -EPERM;
	}

	atomic_set(&d->suspend_mode, 0);
	return 0;
}

static bool hx83113a_chip_detect(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);
	uint8_t tmp_data[DATA_LEN_4];
	bool ret_data = false;
	int i = 0;

	hx83113a_hw_reset(dev);
	hx83113a_sense_off(dev, false); //hx83102ab_sense_off(false);

	for (i = 0; i < 5; i++) {
		hx83113a_register_read(dev, FW_ADDR_ICID_ADDR, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		TOUCH_I("%s:Read driver IC ID = %X,%X,%X\n", __func__, tmp_data[3], tmp_data[2], tmp_data[1]); /*83,10,2X*/

		if ((tmp_data[3] == 0x83) && (tmp_data[2] == 0x11) && (tmp_data[1] == 0x3a)) {
			strlcpy(d->chip_name, HX_83113A_SERIES_PWON, 30);
			TOUCH_I("%s:detect IC HX83113A successfully\n", __func__);
			ret_data = true;
			break;
		} else {
			ret_data = false;
			TOUCH_E("%s:Read driver ID register Fail:\n", __func__);
			TOUCH_E("Could NOT find Himax Chipset \n");
			TOUCH_E("Please check 1.VCCD,VCCA,VSP,VSN \n");
			TOUCH_E("2. LCM_RST,TP_RST \n");
			TOUCH_E("3. Power On Sequence \n");
		}
	}

	return ret_data;
}

static void hx83113a_chip_init(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);

	d->chip_cell_type = CHIP_IS_IN_CELL;
	TOUCH_I("%s:IC cell type = %d\n", __func__, d->chip_cell_type);
	IC_CHECKSUM 			= HX_TP_BIN_CHECKSUM_CRC;
	/*Himax: Set FW and CFG Flash Address*/
	FW_VER_MAJ_FLASH_ADDR = 74757;/*0x00C005 -> 0x12405 */
	FW_VER_MAJ_FLASH_LENG = 1;
	FW_VER_MIN_FLASH_ADDR = 74758;/*0x00C006 -> 0x12406 */
	FW_VER_MIN_FLASH_LENG = 1;
	CFG_VER_MAJ_FLASH_ADDR = 75008;/*0x00C100 -> 0x12500 */
	CFG_VER_MAJ_FLASH_LENG = 1;
	CFG_VER_MIN_FLASH_ADDR = 75009;/*0x00C101 -> 0x12501 */
	CFG_VER_MIN_FLASH_LENG = 1;
	CID_VER_MAJ_FLASH_ADDR = 74754;/*0x00C002 -> 0x12402 */
	CID_VER_MAJ_FLASH_LENG = 1;
	CID_VER_MIN_FLASH_ADDR = 74755;/*0x00C003 -> 0x12403 */
	CID_VER_MIN_FLASH_LENG = 1;
	FW_LGE_VER_MAJ_FLASH_ADDR = 74760;/*0xC008 -> 0x12408 */
	/*PANEL_VERSION_ADDR		= 49156;*/  /*0x00C004*/
	/*PANEL_VERSION_LENG		= 1;*/
}

static void hx83113a_power_on_init(struct device *dev)
{
	uint8_t tmp_data[DATA_LEN_4];
	int i = 0;

	for (i = 0; i < ADDR_LEN_4; i++) {
		tmp_data[i] = (uint8_t)(FW_DATA_CLEAR >> i * 8);
	}

	TOUCH_I("%s:\n", __func__);

	hx83113a_touch_information(dev);
	/*RawOut select initial*/
	hx83113a_register_write(dev, FW_ADDR_RAW_OUT_SEL, DATA_LEN_4, tmp_data, ADDR_LEN_4);
	/*DSRAM func initial*/
	hx83113a_assign_sorting_mode(dev, tmp_data);
	hx83113a_sense_on(dev, 0x00);
}

static int hx83113a_common_init(struct device *dev)
{
	struct himax_ts_data *d = to_himax_data(dev);
	int err = 0;

	TOUCH_I("ic_data START\n");
	ic_data = kzalloc(sizeof(*ic_data), GFP_KERNEL);
	if (ic_data == NULL) { /*Allocate IC data space*/
		return -ENOMEM;
	}

	/* allocate report data */
	hx_touch_data = kzalloc(sizeof(struct himax_report_data), GFP_KERNEL);
	if (hx_touch_data == NULL) {
		return -ENOMEM;
	}

	if (hx83113a_chip_detect(dev) != false) {
		hx83113a_chip_init(dev);
	} else {
		TOUCH_E("%s: chip detect failed!\n", __func__);
		return -ENOMEM;
	}

//	hx83113a_read_FW_ver(dev);

	/*Himax Power On and Load Config*/
	hx83113a_power_on_init(dev);
	hx83113a_calculate_point_number();

	/*calculate the i2c data size*/
	hx83113a_calcDataSize(dev);
	TOUCH_I("%s: calcDataSize complete\n", __func__);
/*
#ifdef CONFIG_OF
	d->pdata->abs_pressure_min        = 0;
	d->pdata->abs_pressure_max        = 200;
	d->pdata->abs_width_min           = 0;
	d->pdata->abs_width_max           = 200;
	pdata->cable_config[0]             = 0xF0;
	pdata->cable_config[1]             = 0x00;
#endif
*/
	d->protocol_type = PROTOCOL_TYPE_B;
	TOUCH_I("%s: Use Protocol Type %c\n", __func__,
	  d->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');

	d->SMWP_enable = 1;
	//wake_lock_init(&d->ts_SMWP_wake_lock, WAKE_LOCK_SUSPEND, HIMAX_common_NAME);

	/*touch data init*/
	err = hx83113a_report_data_init();

	if (err) {
		return -ENOMEM;
	}

	if (hx83113a_proc_init()) {
		TOUCH_E(" %s: himax_common proc_init failed!\n", __func__);
		return -ENOMEM;
	}

#ifdef CONFIG_LGE_TOUCH_HIMAX_HX83113A_DEBUG
	if (hx83113a_debug_init(dev))
		TOUCH_E(" %s: debug initial failed!\n", __func__);
#endif
	return 0;
}

static int hx83113a_init_pm(struct device *dev)
{
#if defined(CONFIG_DRM_MSM)
#if defined(CONFIG_LGE_TOUCH_USE_PANEL_NOTIFY)
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("%s: drm_notif change\n", __func__);
	ts->drm_notif.notifier_call = hx83113a_drm_notifier_callback;
#endif
#elif defined(CONFIG_FB)
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("%s: fb_notif change\n", __func__);
	ts->fb_notif.notifier_call = hx83113a_fb_notifier_callback;
#endif

	return 0;
}

static void hx83113a_fb_notify_work_func(struct work_struct *fb_notify_work)
{
	struct himax_ts_data *d = container_of(to_delayed_work(fb_notify_work),
			struct himax_ts_data, fb_notify_work);
	int ret = 0;

	if (d->lcd_mode == LCD_MODE_U3)
		ret = FB_RESUME;
	else
		ret = FB_SUSPEND;

	touch_notifier_call_chain(NOTIFY_FB, &ret);
}

static void hx83113a_init_works(struct himax_ts_data *d)
{
	INIT_DELAYED_WORK(&d->fb_notify_work, hx83113a_fb_notify_work_func);
}

static void hx83113a_init_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 0;
	ts->tci.info[TCI_1].max_intertap = 70;
	ts->tci.info[TCI_1].touch_slop = 10;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].min_intertap = 0;
	ts->tci.info[TCI_2].max_intertap = 70;
	ts->tci.info[TCI_2].touch_slop = 10;
	ts->tci.info[TCI_2].tap_distance = 255;
	ts->tci.info[TCI_2].intr_delay = 0;
}

static void hx83113a_init_swipe_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	float mm_to_point = 15.8f; // 1mm -> pixel

	TOUCH_TRACE();

	ts->swipe[SWIPE_U].enable = false;
	ts->swipe[SWIPE_U].debug_enable = false;
	ts->swipe[SWIPE_U].distance = 20;
	ts->swipe[SWIPE_U].ratio_thres = 150;
	ts->swipe[SWIPE_U].min_time = 4;
	ts->swipe[SWIPE_U].max_time = 150;
	ts->swipe[SWIPE_U].wrong_dir_thres = 5;
	ts->swipe[SWIPE_U].init_ratio_chk_dist = 4;
	ts->swipe[SWIPE_U].init_ratio_thres = 100;
	ts->swipe[SWIPE_U].area.x1 = 0 + LPWG_SENSELESS_AREA_W;
	ts->swipe[SWIPE_U].area.y1 = 0;
	ts->swipe[SWIPE_U].area.x2 = ts->caps.max_x - LPWG_SENSELESS_AREA_W;
	ts->swipe[SWIPE_U].area.y2 = ts->caps.max_y;
	ts->swipe[SWIPE_U].start_area.x1 = (ts->caps.max_x / 2) - (int)(mm_to_point * 18);		// start_area_width 36mm
	ts->swipe[SWIPE_U].start_area.y1 = ts->swipe[SWIPE_U].area.y2 - (int)(mm_to_point * 10);	// start_area_height 10mm
	ts->swipe[SWIPE_U].start_area.x2 = (ts->caps.max_x / 2) + (int)(mm_to_point * 18);		// start_area_width 36mm
	ts->swipe[SWIPE_U].start_area.y2 = ts->swipe[SWIPE_U].area.y2;
	ts->swipe[SWIPE_U].border_area.x1 = 0;
	ts->swipe[SWIPE_U].border_area.y1 = 0;
	ts->swipe[SWIPE_U].border_area.x2 = 0;
	ts->swipe[SWIPE_U].border_area.y2 = 0;
	ts->swipe[SWIPE_U].start_border_area.x1 = 0;
	ts->swipe[SWIPE_U].start_border_area.y1 = 0;
	ts->swipe[SWIPE_U].start_border_area.x2 = 0;
	ts->swipe[SWIPE_U].start_border_area.y2 = 0;

	ts->swipe[SWIPE_L].enable = false;
	ts->swipe[SWIPE_L].debug_enable = false;
	ts->swipe[SWIPE_L].distance = 12;
	ts->swipe[SWIPE_L].ratio_thres = 150;
	ts->swipe[SWIPE_L].min_time = 4;
	ts->swipe[SWIPE_L].max_time = 150;
	ts->swipe[SWIPE_L].wrong_dir_thres = 5;
	ts->swipe[SWIPE_L].init_ratio_chk_dist = 4;
	ts->swipe[SWIPE_L].init_ratio_thres = 100;
	ts->swipe[SWIPE_L].area.x1 = 0;
	ts->swipe[SWIPE_L].area.y1 = 0;
	ts->swipe[SWIPE_L].area.x2 = ts->caps.max_x;
	ts->swipe[SWIPE_L].area.y2 = ts->caps.max_y;
	ts->swipe[SWIPE_L].start_area.x1 = ts->caps.max_x - (int)(mm_to_point * 10);
	ts->swipe[SWIPE_L].start_area.y1 = 305;
	ts->swipe[SWIPE_L].start_area.x2 = ts->caps.max_x;
	ts->swipe[SWIPE_L].start_area.y2 = 1655;
	ts->swipe[SWIPE_L].border_area.x1 = 0;
	ts->swipe[SWIPE_L].border_area.y1 = 0;
	ts->swipe[SWIPE_L].border_area.x2 = 0;
	ts->swipe[SWIPE_L].border_area.y2 = 0;
	ts->swipe[SWIPE_L].start_border_area.x1 = 0;
	ts->swipe[SWIPE_L].start_border_area.y1 = 0;
	ts->swipe[SWIPE_L].start_border_area.x2 = 0;
	ts->swipe[SWIPE_L].start_border_area.y2 = 0;

	ts->swipe[SWIPE_R].enable = false;
	ts->swipe[SWIPE_R].debug_enable = false;
	ts->swipe[SWIPE_R].distance = 12;
	ts->swipe[SWIPE_R].ratio_thres = 150;
	ts->swipe[SWIPE_R].min_time = 4;
	ts->swipe[SWIPE_R].max_time = 150;
	ts->swipe[SWIPE_R].wrong_dir_thres = 5;
	ts->swipe[SWIPE_R].init_ratio_chk_dist = 4;
	ts->swipe[SWIPE_R].init_ratio_thres = 100;
	ts->swipe[SWIPE_R].area.x1 = 0;
	ts->swipe[SWIPE_R].area.y1 = 0;
	ts->swipe[SWIPE_R].area.x2 = ts->caps.max_x;
	ts->swipe[SWIPE_R].area.y2 = ts->caps.max_y;
	ts->swipe[SWIPE_R].start_area.x1 = 0;
	ts->swipe[SWIPE_R].start_area.y1 = 305;
	ts->swipe[SWIPE_R].start_area.x2 = (int)(mm_to_point * 10);
	ts->swipe[SWIPE_R].start_area.y2 = 1655;
	ts->swipe[SWIPE_R].border_area.x1 = 0;
	ts->swipe[SWIPE_R].border_area.y1 = 0;
	ts->swipe[SWIPE_R].border_area.x2 = 0;
	ts->swipe[SWIPE_R].border_area.y2 = 0;
	ts->swipe[SWIPE_R].start_border_area.x1 = 0;
	ts->swipe[SWIPE_R].start_border_area.y1 = 0;
	ts->swipe[SWIPE_R].start_border_area.x2 = 0;
	ts->swipe[SWIPE_R].start_border_area.y2 = 0;
}

static int hx83113a_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = NULL;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate hx83113a data\n");
		return -ENOMEM;
	}

	touch_set_device(ts, d);

	private_ts = d;

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_power_init(dev);
	touch_bus_init(dev, MAX_BUF_SIZE);

	d->dev = dev;
	g_hx_dev= d->dev;
	mutex_init(&d->rw_lock);

	hx83113a_init_works(d);
	hx83113a_common_init(dev);
	d->lcd_mode = LCD_MODE_U3;
	hx83113a_init_tci_info(dev);
	hx83113a_init_swipe_info(dev);

	return 0;
}

static int hx83113a_remove(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = to_himax_data(dev);

#ifdef CONFIG_LGE_TOUCH_HIMAX_HX83113A_DEBUG
	hx83113a_debug_remove(dev);
#endif

	hx83113a_proc_deinit();

	//wake_lock_destroy(&d->ts_SMWP_wake_lock);
	if (gpio_is_valid(ts->int_pin))
		gpio_free(ts->int_pin);
	if (gpio_is_valid(ts->reset_pin))
		gpio_free(ts->reset_pin);

	kfree(hx_touch_data);
	kfree(ic_data);
	kfree(d);
	probe_fail_flag = 0;

	return 0;
}

static int hx83113a_shutdown(struct device *dev)
{
	return 0;
}

static void hx83113a_usb_detect_set(struct device *dev, uint8_t *cable_config)
{
	uint8_t tmp_data[DATA_LEN_4] = {0, };
	uint8_t back_data[DATA_LEN_4] = {0, };
	uint8_t w_data[DATA_LEN_4] = {0, };
	uint8_t retry_cnt = 0;
	int i = 0;

	TOUCH_I("%s: USB detect status %s\n", __func__, cable_config[1] ? "IN" : "OUT");

	do {
		if (cable_config[1] == 0x01) {
			for (i = 0; i < ADDR_LEN_4; i++)
				w_data[i] = (uint8_t)(FW_FUNC_HANDSHAKING_PWD >> i * 8);
		} else {
			for (i = 0; i < ADDR_LEN_4; i++)
				w_data[i] = (uint8_t)(FW_DATA_SAFE_MODE_RELEASE_PW_RESET >> i * 8);
		}
		hx83113a_register_write(dev, FW_USB_DETECT_ADDR, DATA_LEN_4, w_data, ADDR_LEN_4);

		hx83113a_register_read(dev, FW_USB_DETECT_ADDR, DATA_LEN_4, tmp_data, ADDR_LEN_4);
		/*TOUCH_I("%s: tmp_data[0]=%d, USB detect=%d, retry_cnt=%d \n", __func__, tmp_data[0],cable_config[1] ,retry_cnt);*/
		retry_cnt++;
	} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1]  || tmp_data[0] != back_data[0]) && retry_cnt < HIMAX_REG_RETRY_TIMES);
}

static int hx83113a_cable_connect(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = to_himax_data(dev);
	int charger_state = atomic_read(&ts->state.connect);

	TOUCH_I("TA Type: %d\n", charger_state);

	d->cable_config[1] = (charger_state == 0) ? 0x00 : 0x01;
	hx83113a_usb_detect_set(dev, d->cable_config);
	TOUCH_I("%s: Cable status change: 0x%2.2X\n", __func__, d->cable_config[1]);

	return 0;
}

static int hx83113a_notify(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct himax_ts_data *d = to_himax_data(dev);
	int ret = 0;

	TOUCH_TRACE();
	TOUCH_I("%s event=0x%x\n", __func__, (unsigned int)event);

	switch (event) {
	case NOTIFY_TOUCH_RESET:
		if(atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_1)
			ret = 1;
		else
			ret = 0;
		TOUCH_I("NOTIFY_TOUCH_RESET! return = %d\n", ret);
		break;
	case LCD_EVENT_LCD_BLANK:
		TOUCH_I("LCD_EVENT_LCD_BLANK!\n");
		atomic_set(&ts->state.fb, FB_SUSPEND);
		break;
	case LCD_EVENT_LCD_UNBLANK:
		TOUCH_I("LCD_EVENT_LCD_UNBLANK!\n");
		atomic_set(&ts->state.fb, FB_RESUME);
		break;
	case LCD_EVENT_LCD_MODE:
		TOUCH_I("LCD_EVENT_LCD_MODE!\n");
		hx83113a_lcd_mode(dev, *(u32 *)data);
		queue_delayed_work(ts->wq, &d->fb_notify_work, 0);
		break;
	case NOTIFY_CONNECTION:
		TOUCH_I("NOTIFY_CONNECTION!\n");
		ret = hx83113a_cable_connect(dev);
		break;
	case NOTIFY_WIRELESS:
		TOUCH_I("NOTIFY_WIRELESS!\n");
		break;
	case NOTIFY_EARJACK:
		TOUCH_I("NOTIFY_EARJACK!\n");
		break;
	case NOTIFY_IME_STATE:
		TOUCH_I("NOTIFY_IME_STATE!\n");
		break;
	case NOTIFY_DEBUG_TOOL:
		TOUCH_I("NOTIFY_DEBUG_TOOL!\n");
		break;
	default:
		TOUCH_E("%lu is not supported\n", event);
		break;
	}

	return ret;
}

static int hx83113a_get_cmd_version(struct device *dev, char *buf)
{
	struct himax_ts_data *d = to_himax_data(dev);
	int offset = 0;

	hx83113a_read_FW_ver(dev);

	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			"============ Version Info ============\n");
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			" IC Version: v%d.%02d, fw_id: %X\n",
			d->ic_info.ic_official_ver, d->ic_info.ic_fw_ver,
			d->ic_info.vendor_fw_ver);
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			" Bin Version: v%d.%02d, CID_ver: %X\n",
			d->ic_info.bin_official_ver, d->ic_info.bin_fw_ver,
			(d->ic_info.vendor_cid_maj_ver << 8 | d->ic_info.vendor_cid_min_ver));
;
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			" Panel_cfg: %X, Touch_cfg: %X, Display_cfg: %X\n",
			d->ic_info.vendor_panel_ver, d->ic_info.vendor_touch_cfg_ver,
			d->ic_info.vendor_display_cfg_ver);
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset, "Product_id : [HX83113-A]\n");
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			"======================================\n");

	hx83113a_sense_on(dev, 0x00);

	return offset;
}

static int hx83113a_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct himax_ts_data *d = to_himax_data(dev);
	int offset = 0;

	hx83113a_read_FW_ver(dev);

	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset, "v%d.%02d\n",
				d->ic_info.ic_official_ver, d->ic_info.ic_fw_ver);

	hx83113a_sense_on(dev, 0x00);

	return offset;
}

static int hx83113a_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();
	TOUCH_I("%s\n", __func__);

	return 0;
}

static int hx83113a_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_I("%s\n", __func__);
	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = hx83113a_get_cmd_version(dev, (char *)output);
		break;

	case CMD_ATCMD_VERSION:
		ret = hx83113a_get_cmd_atcmd_version(dev, (char *)output);
		break;

	default:
		break;
	}

	return ret;
}

static struct touch_driver touch_driver = {
	.probe = hx83113a_probe,
	.remove = hx83113a_remove,
	.shutdown = hx83113a_shutdown,
	.suspend = hx83113a_suspend,
	.resume = hx83113a_resume,
	.init = hx83113a_init,
	.irq_handler = hx83113a_irq_handler,
	.power = hx83113a_power,
	.upgrade = hx83113a_upgrade,
#if defined(CONFIG_LGE_TOUCH_APP_FW_UPGRADE)
	.app_upgrade = hx83113a_app_upgrade,
#endif
	.esd_recovery = hx83113a_esd_recovery,
	.lpwg = hx83113a_lpwg,
	.notify = hx83113a_notify,
	.init_pm = hx83113a_init_pm,
	.swipe_enable = hx83113a_swipe_enable,
	.register_sysfs = hx83113a_register_sysfs,
	.set = hx83113a_set,
	.get = hx83113a_get,
};

#define MATCH_NAME			"himax,hx83113a"
static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{},
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(touch_match_ids),
};

static int __init touch_device_init(void)
{
	TOUCH_TRACE();

/* to do
	if (is_ddic_name("TXD-HX83113")) {
		TOUCH_I("%s, HX83113 found!!! lcm_name = %s\n", __func__, get_ddic_name());
	} else {
		TOUCH_I("%s, HX83113 not found.\n", __func__);
		return 0;
	}
*/
	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);

MODULE_DESCRIPTION("Himax for LGE touch driver v1");
MODULE_LICENSE("GPL");

