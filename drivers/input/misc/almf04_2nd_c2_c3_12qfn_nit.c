#define MODULE_TAG	"<almf04_2nd>"
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/pm_wakeup.h>
#include <linux/unistd.h>
#include <linux/async.h>
#include <linux/in.h>

#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/sort.h>

#include <linux/firmware.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif


#include <soc/qcom/lge/board_lge.h>

//#include "bs_log.h"

#include "almf04_c2_c3_12qfn_nit.h"

#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
#include <linux/notifier.h>
#endif

#define CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE

#define ALMF04_DRV_NAME     "lge_sar_rf2"

#define FAR_STATUS      1
#define NEAR_STATUS     0

#define CNT_MAX_CH		4

#define CH1_FAR         0x2
#define CH1_NEAR        0x1
#define CH2_FAR         (0x2 << 2)
#define CH2_NEAR        (0x1 << 2)

#define CH3_FAR         (0x2 << 4)
#define CH3_NEAR        (0x1 << 4)

#define SNR_FAR			0x2
#define SNR_NEAR		0x1

/* I2C Suspend Check */
#define ALMF04_STATUS_RESUME        0
#define ALMF04_STATUS_SUSPEND       1
#define ALMF04_STATUS_QUEUE_WORK    2

#define BIT_PERCENT_UNIT            8.192
#define MK_INT(X, Y)                (((int)X << 8)+(int)Y)

#define ENABLE_SENSOR_PINS          0
#define DISABLE_SENSOR_PINS         1

#define ON_SENSOR                   1
#define OFF_SENSOR                  2
#define PATH_CAPSENSOR_CAL  "/mnt/vendor/sns/sar_controller_cal.dat"

#if defined (CONFIG_MACH_LITO_ACELM)
#define CNT_INITCODE					42
static const unsigned char InitCodeAddr[CNT_INITCODE] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B };
static const unsigned char InitCodeVal[CNT_INITCODE] =  { 0x01, 0x9A, 0x00, 0x21, 0x01, 0x9A, 0x33, 0x13, 0x13, 0x13, 0x0F, 0x72, 0x82, 0x64, 0x82, 0x64, 0x82, 0x64, 0xD0, 0x89, 0x20, 0x10, 0x0B, 0x07, 0x0B, 0x07, 0x0B, 0x07, 0x33, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x9E, 0x4F, 0x03, 0x33, 0x01, 0x02, 0x04 };

#elif defined (CONFIG_MACH_LAGOON_ACEXLM)
#define CNT_INITCODE					42
static const unsigned char InitCodeAddr[CNT_INITCODE] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B };
static const unsigned char InitCodeVal[CNT_INITCODE] =  { 0x11, 0x9A, 0x00, 0x14, 0x11, 0x9A, 0x33, 0x13, 0x13, 0x13, 0x0F, 0x72, 0x82, 0x64, 0x82, 0x64, 0x82, 0x64, 0xD0, 0x89, 0x20, 0x10, 0x0B, 0x07, 0x0B, 0x07, 0x0B, 0x07, 0x33, 0x00, 0xFF, 0x13, 0x28, 0x00, 0xFF, 0xCC, 0x4F, 0x10, 0x00, 0x03, 0x02, 0x04 };

#else
//[200609] Change Cap value for perfomance by shin.choi@lge.com
#define CNT_INITCODE					42
static const unsigned char InitCodeAddr[CNT_INITCODE]   = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B  };
static const unsigned char InitCodeVal[CNT_INITCODE]    = { 0x00, 0x29, 0x00, 0x3D, 0x00, 0x29, 0x33, 0x13, 0x13, 0x13, 0x0F, 0x70, 0x82, 0x64, 0x82, 0x64, 0x82, 0x64, 0xD0, 0x89, 0x20, 0x10, 0x0B, 0x07, 0x15, 0x0D, 0x0B, 0x07, 0x33, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xCC, 0x4F, 0x03, 0x33, 0x00, 0x07, 0x04  };
#endif

static struct i2c_driver almf04_driver;
static struct workqueue_struct *almf04_workqueue;

#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
static struct workqueue_struct *sar_controller_callback_workqueue;
static ATOMIC_NOTIFIER_HEAD(sar_controller_atomic_notifier);
#endif

#ifdef CONFIG_OF
enum sensor_dt_entry_status {
	DT_REQUIRED,
	DT_SUGGESTED,
	DT_OPTIONAL,
};

enum sensor_dt_entry_type {
	DT_U32,
	DT_GPIO,
	DT_BOOL,
	DT_STRING,
};

struct sensor_dt_to_pdata_map {
	const char			*dt_name;
	void				*ptr_data;
	enum sensor_dt_entry_status status;
	enum sensor_dt_entry_type	type;
	int				default_val;
};
#endif

static struct i2c_client *almf04_i2c_client; /* global i2c_client to support ioctl */

struct almf04_platform_data {
	int (*init)(struct i2c_client *client);
	void (*exit)(struct i2c_client *client);
	unsigned int irq_gpio;
	unsigned long chip_enable;

	int (*power_on)(struct i2c_client *client, bool on);
	u32 irq_gpio_flags;

	bool i2c_pull_up;

	struct regulator *vcc_ana;
	struct regulator *vcc_dig;
	struct regulator *vcc_i2c;

	u32 vdd_ana_supply_min;
	u32 vdd_ana_supply_max;
	u32 vdd_ana_load_ua;

	u32 input_pins_num; // not include ref sensor pin
	const char *fw_name;
};

struct almf04_data {
	int (*get_nirq_low)(void);
	struct i2c_client *client;
	struct mutex update_lock;
	struct mutex enable_lock;
	struct delayed_work	dwork;		/* for PS interrupt */
#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
        struct delayed_work     callback_dwork; // for usb detect
        unsigned long           event; // event for usb detect
        int                     data;  // data for usb detect
#endif
	struct input_dev *input_dev_cap;
#ifdef CONFIG_OF
	struct almf04_platform_data *platform_data;
	int irq;
#endif
	unsigned int enable;
	unsigned int sw_mode;
	atomic_t i2c_status;
#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
        struct notifier_block atomic_notif;
#endif

	unsigned int cap_detection;
	int touch_out;
};

static bool on_sensor = false;
static bool check_allnear = false;
static bool cal_result = false; // debugging calibration paused

static atomic_t pm_suspend_flag;

#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
static bool probe_end_flag = false;
#endif

static int get_bit(unsigned short x, int n);
static void check_firmware_ready(struct i2c_client *client);

static void chg_mode(unsigned char flag, struct i2c_client *client)
{
	if(flag == ON) {
		i2c_smbus_write_byte_data(client, ADDR_EFLA_STS, 0x80);
		PINFO("ALMF04 change_mode : %d\n",i2c_smbus_read_byte_data(client, ADDR_EFLA_STS));

		#if defined(USE_ALMF04)
			i2c_smbus_write_byte_data(client, ADDR_ROM_SAFE, VAL_ROM_MASK1);
			i2c_smbus_write_byte_data(client, ADDR_ROM_SAFE, VAL_ROM_MASK2);
		#endif
	}
	else {
		i2c_smbus_write_byte_data(client, ADDR_EFLA_STS, 0x00);
		PINFO("ALMF04 change_mode : %d\n",i2c_smbus_read_byte_data(client, ADDR_EFLA_STS));
	}
	mdelay(1);
}

static int Update_Register(struct almf04_data *data, struct i2c_client *client)
{
	int ret = 0;
    unsigned char loop, rdata;
	bool update_flag = false;

	#ifdef REG_VER_CHK_USE
		//--------------------------------------------------------------------------//
		//Register Setting Version Check
		rdata = i2c_smbus_read_byte_data(client, InitCodeAddr[IDX_REG_VER]);
		if(rdata != InitCodeVal[IDX_REG_VER]) update_flag = true;
		//--------------------------------------------------------------------------//
	#else
		for(loop = 0 ; loop < CNT_INITCODE ; loop++)
		{
			rdata = i2c_smbus_read_byte_data(client, InitCodeAddr[loop]);

			PINFO(" ALMF04 <Register> [Address: 0x%02x] read value:[0x%02x], Initvalue:[0x%02x]\n", InitCodeAddr[loop], rdata, InitCodeVal[loop]);

			if(rdata != InitCodeVal[loop])
			{
				update_flag = true;
				break;
			}
		}
	#endif

	if(update_flag == true) {

		mdelay(10);

		PINFO("Update_Register Start");
		for(loop = 0 ; loop < CNT_INITCODE ; loop++) {
			ret = i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
			if (ret) {
				PINFO("i2c_write_fail[0x%x]",InitCodeAddr[loop]);
				return ret;
			}
			PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		}
		check_firmware_ready(client);

		//Rom Data Save Command
		ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x04);

		if (ret) {
			PINFO("i2c_write_fail[0x%x]",I2C_ADDR_SYS_CTRL);
			goto i2c_fail;
		}
		
		check_firmware_ready(client);
		
		PINFO("Update_Register Finished");
	}
	return 0;

i2c_fail:
	return ret;
}


static unsigned char chk_done(unsigned int wait_cnt, struct i2c_client *client)
{
	unsigned int trycnt = 0;
	unsigned char rtn;

	do
	{
		if(++trycnt > wait_cnt) {
			PINFO("ALMF04  RTN_TIMEOUT");
			return RTN_TIMEOUT;
		}
		mdelay(1);
		rtn = i2c_smbus_read_byte_data(client, ADDR_EFLA_STS);
	}while((rtn & FLAG_DONE) != FLAG_DONE);

	return RTN_SUCC;
}

static unsigned char chk_done_erase(unsigned int wait_cnt, struct i2c_client *client)
{
	unsigned int trycnt = 0;
	unsigned char rtn;

	do
	{
		if(++trycnt > wait_cnt) return RTN_TIMEOUT;

	mdelay(5);
	
    rtn = i2c_smbus_read_byte_data(client, ADDR_EFLA_STS);
	}while((rtn & FLAG_DONE_ERASE) != FLAG_DONE_ERASE);

	return RTN_SUCC;
}

static unsigned char erase_rom(struct i2c_client *client)
{
	#if defined(USE_ALMF04)
		i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
		i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, EMD_ALL_ERASE);
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);
	#else
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_ERASE_ALL);
	#endif

	if(chk_done_erase(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT)
		return RTN_TIMEOUT; //timeout

	return RTN_SUCC;
}

static void chg_active_mode(struct almf04_data *data, bool flag)
{
	int en_mode;

	if(flag){
		en_mode = gpio_get_value(data->platform_data->chip_enable);
		if(en_mode) {
			gpio_set_value(data->platform_data->chip_enable, 0);    /*chip_en pin - low : on, high : off*/
			mdelay(150);
		}
	}
	else
		gpio_set_value(data->platform_data->chip_enable, 1);    /*chip_en pin - low : on, high : off*/
}

static void onoff_sensor(struct almf04_data *data, int onoff_mode)
{
	int nparse_mode;

	nparse_mode = onoff_mode;
	PINFO("ALMF04 onoff_sensor: nparse_mode [%d]",nparse_mode);

	if (nparse_mode == ENABLE_SENSOR_PINS) {
#if 1 // fixed bug to support cap sensor HAL
		input_report_abs(data->input_dev_cap, ABS_DISTANCE, 3);/* Initializing input event */
		input_sync(data->input_dev_cap);
#endif
		chg_active_mode(data, true);
		if (!on_sensor)
			enable_irq_wake(data->irq);
		on_sensor = true;
#if 0 // fixed bug to support cap sensor HAL
		if (gpio_get_value(data->platform_data->irq_gpio)) {
			input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* force FAR detection */
			input_sync(data->input_dev_cap);
		}
#endif
	}
	if (nparse_mode == DISABLE_SENSOR_PINS) {
		chg_active_mode(data, false);
		if (on_sensor)
			disable_irq_wake(data->irq);
		on_sensor = false;
	}
}

static unsigned char load_firmware(struct almf04_data *data, struct i2c_client *client, const char *name)
{
	const struct firmware *fw = NULL;
	unsigned char rtn;
	int ret, i, count = 0;
	int max_page;
	unsigned short main_version, sub_version;
	unsigned char page_addr[2];
	unsigned char page_num;
	int version_addr;
	unsigned char rdata, vfy_fail;
	int retry_cnt = 0;
  
	PINFO("ALMF04 Load Firmware Entered");
  
	chg_active_mode(data, true);

	ret = request_firmware(&fw, name, &data->client->dev);
	if (ret) 
	{
		PINFO("ALMF04 Unable to open bin [%s]  ret %d", name, ret);
		if (fw) release_firmware(fw);
		return 1;
	}
	else 
		PINFO("ALMF04 Open bin [%s] ret : %d ", name, ret);
	
	max_page = (fw->size)/SZ_PAGE_DATA;
	version_addr = (fw->size)-SZ_PAGE_DATA;
	page_num = fw->data[version_addr+3];
	PINFO("###########fw version : %d.%02d, page_num : %d###########", fw->data[version_addr], fw->data[version_addr+1], page_num);

	main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
	sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);
	PINFO("###########ic version : %d.%02d, ###########", main_version, sub_version);

	if( (fw->data[version_addr] != main_version) || (fw->data[version_addr+1] != sub_version)) 
	{
		PINFO("ALMF04 update firmware");

		/* IC Download Mode Change */
		chg_mode(ON, client);

		//FW Download & Verify 
		for(retry_cnt = 0 ; retry_cnt < 3 ; retry_cnt++)
		{
			//rom erase
			rtn = erase_rom(client);
			if(rtn != RTN_SUCC) {
				PINFO("earse fail\n");
				if (fw) release_firmware(fw);
				chg_mode(OFF, client);
				return rtn;		//earse fail
			}

			//fw download
			while(count < page_num) 
			{
				for(i=0; i < SZ_PAGE_DATA; i++) i2c_smbus_write_byte_data(client, i, fw->data[i + (count*SZ_PAGE_DATA)]);
				
				page_addr[1] = (unsigned char)((count & 0xFF00) >> 8);
				page_addr[0] = (unsigned char)(count & 0x00FF);

				i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
				i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, (page_addr[1] | EMD_PG_WRITE));
				i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);
				
				if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT)
				{
					if (fw) release_firmware(fw);
					chg_mode(OFF, client);
					return RTN_TIMEOUT;
				}
				count++;
			}

			//fw verify
			count = 0;
			vfy_fail = 0;
			while(count < page_num) 
			{
				page_addr[1] = (unsigned char)((count & 0xFF00) >> 8);
				page_addr[0] = (unsigned char)(count & 0x00FF);

				i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
				i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, (page_addr[1] | EMD_PG_READ));
				i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);

				if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) {
					if (fw) release_firmware(fw);
					chg_mode(OFF, client);
					return RTN_TIMEOUT;
				}

				for(i = 0; i < SZ_PAGE_DATA; i++) 
				{
					rdata = i2c_smbus_read_byte_data(client, i);
					//Verify fail
					if(rdata != fw->data[i + (count*SZ_PAGE_DATA)])
					{
						PINFO("verify failed!! [retry:%d][%d:(%d,%d)]......", retry_cnt, i + (count*SZ_PAGE_DATA), rdata, fw->data[i + (count*SZ_PAGE_DATA)]);
						vfy_fail = 1;
					}
				}
				
				if(vfy_fail == 1) break;
				count++;
			}

			//verify success
			if(vfy_fail == 0)
			{
				PINFO("verify success [retry:%d]", retry_cnt);
				break;
			}
		}

		chg_mode(OFF, client);
		
		mdelay(100);

		main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
		sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);

		if( (fw->data[version_addr] != main_version) || (fw->data[version_addr+1] != sub_version)) 
		{
			PINFO("ATMF04 Firmware update failed.(ic version : %d.%02d)", main_version, sub_version);
			if (fw) release_firmware(fw);
			return 3;
		}
		else
			PINFO("ATMF04 Firmware update is done");
	}
	else {
		PINFO("Not update firmware. Firmware version is the same as IC.");
	}
	
	Update_Register(data, client);

	chg_active_mode(data, false);
	mdelay(50);

	PINFO("disable ok");
	release_firmware(fw);

	return 0;
}

static bool valid_multiple_input_pins(struct almf04_data *data)
{
	if (data->platform_data->input_pins_num > 1)
		return true;

	return false;

}

static int write_calibration_data(struct almf04_data *data, char *filename)
{
	int fd = 0;

	char file_result[2]; // debugging calibration paused
	mm_segment_t old_fs = get_fs();

	PINFO("write_calibration_data Entered[%d]",data->platform_data->irq_gpio);
	set_fs(KERNEL_DS);
	fd = ksys_open(filename, O_WRONLY|O_CREAT, 0664);

	if (fd >= 0) {
#if 1 // debugging calibration paused
		if(cal_result) {
			file_result[0] = CAP_CAL_RESULT_PASS;
			file_result[1] = '\0';
			ksys_write(fd, file_result, 1);
		} else {
			strncpy(file_result, CAP_CAL_RESULT_FAIL, strlen(CAP_CAL_RESULT_FAIL));
			file_result[1] = '\0';
			ksys_write(fd, file_result, 1);
		}
		PINFO("%s: write [%s] to %s", __FUNCTION__, file_result, filename);
		ksys_close(fd);
    set_fs(old_fs);
#else
		ksys_write(fd,0,sizeof(int));
		ksys_close(fd);
#endif
	} else {
		PINFO("%s: %s open failed [%d]......", __FUNCTION__, filename, fd);
	}

	//PINFO("sys open to save cal.dat");

	return 0;
}

static void check_firmware_ready(struct i2c_client *client)
{
	unsigned char sys_status = 1;
	int i;

	sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
	for(i = 0 ; i < 100 ; i++) {
		if (get_bit(sys_status, 0) == 1) {
			//============================================================//
			//[200507] ADS Delete
			//[START]=====================================================//
			//PINFO("%s: Firmware is busy now.....[%d] sys_status = [0x%x]", __FUNCTION__, i, sys_status);
			//[END]=======================================================//
			mdelay(10);
			sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
		} else {
			break;
		}
	}

	//============================================================//
	//[200507] ADS Change
	//[START]=====================================================//
	//PINFO("%s: sys_status = [0x%x]", __FUNCTION__, sys_status);
	if (get_bit(sys_status, 0) == 1)
		PINFO("%s: [%d] sys_status = [0x%x] Firmware is busy now.", __FUNCTION__, i, sys_status);
	else
		PINFO("%s: [%d] sys_status = [0x%x])", __FUNCTION__, i, sys_status);
	//[END]=======================================================//
	return;
}

static ssize_t almf04_show_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	int loop;
	char buf_line[256] = "";
	int nlength = 0;
	char buf_regproxdata[512] = "";
	client = data->client;

	for (loop = 0; loop < CNT_INITCODE; loop++) {
		memset(buf_line, 0, sizeof(buf_line));
		sprintf(buf_line, "[0x%x:0x%x]", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		PINFO("###### [0x%x][0x%x]###", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		nlength = strlen(buf_regproxdata);
		strcpy(&buf_regproxdata[nlength], buf_line);
	}

	return sprintf(buf,"%s\n", buf_regproxdata);
}

static ssize_t almf04_store_read_reg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	int loop;
	int ret = 0;
	unsigned long val;
	bool flag_match = false;

	client = data->client;
	PINFO("input :%s", buf);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	for (loop = 0; loop < CNT_INITCODE; loop++) {
		PINFO("###### loop:%d [0x%x][0x%x]###", loop,InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		if( val ==  InitCodeAddr[loop])
		{
			PINFO("match Addr :%s", buf);
			flag_match = true;
			break;
		}
	}

	if(flag_match)
	{
		ret = i2c_smbus_read_byte_data(client, InitCodeAddr[loop]);
		PINFO("read register [0x%x][0x%x]",InitCodeAddr[loop],i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
	}
	else
	{
		PINFO("match Addr fail");
	}

	return count;
}

static ssize_t almf04_store_write_reg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char loop;
	bool flag_match = false;
	unsigned int addr, val;
	client = data->client;

	PINFO("input :%s", buf);

	if (sscanf(buf, "%x %x", &addr, &val) <= 0)
		return count;

	for (loop = 0; loop < CNT_INITCODE; loop++) {
		PINFO("###### [0x%x][0x%x]###", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		if( addr ==  InitCodeAddr[loop]) {
			PINFO("match Addr :%s", buf);
			flag_match = true;
			break;
		}
	}

	if(flag_match) {
		i2c_smbus_write_byte_data(client, InitCodeAddr[loop], val);
		PINFO("write register ##[0x%x][0x%x]##", InitCodeAddr[loop], val);
		i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x04);
		check_firmware_ready(client);
		i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x01);
		// return sprintf(buf,"0x%02x\n",i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		return count;
	}
	else
		return count;
}

static ssize_t almf04_show_regproxctrl0(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	PINFO("almf04_show_regproxctrl0: %d\n",on_sensor);
	if(on_sensor==true)
		return sprintf(buf,"0x%02x\n",0x0C);
	return sprintf(buf,"0x%02x\n",0x00);
}

static ssize_t almf04_store_reg(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char loop;
	client = data->client;

	for (loop = 0; loop < CNT_INITCODE; loop++) {
		i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
		PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], InitCodeVal[loop]);
	}
	i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x01);

	check_firmware_ready(client);
	return count;
}

static ssize_t almf04_show_proxstatus(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret ;
	struct almf04_data *data = dev_get_drvdata(dev);
	ret = gpio_get_value(data->platform_data->irq_gpio);
	return sprintf(buf, "%d\n", ret);
}


static ssize_t almf04_store_onoffsensor(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct almf04_data *data = dev_get_drvdata(dev);
	unsigned long val;
	struct i2c_client *client = to_i2c_client(dev);
	client = data->client;
	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if(val == ON_SENSOR) {
		onoff_sensor(data,ENABLE_SENSOR_PINS);
		PINFO("Store ON_SENSOR[%d]",data->platform_data->irq_gpio);
	}
	else if (val == OFF_SENSOR) {
		PINFO("Store OFF_SENSOR[%d]",data->platform_data->irq_gpio);
		onoff_sensor(data,DISABLE_SENSOR_PINS);
	}
	return count;
}

static ssize_t almf04_store_regreset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char loop, rdata[2];
	int duty_val[CNT_MAX_CH], per_result[(CNT_MAX_CH -1)];

	int ret;
	client = data->client;

	check_firmware_ready(client);

#if 1 // debugging calibration paused
	// Whether cal is pass or fail, make it cal_result true to check raw data/CS/CR/count in bypass mode of AAT
	cal_result = true;
	write_calibration_data(data, PATH_CAPSENSOR_CAL);
#endif

	//ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x02);
	ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x01);
	if(ret)
		PINFO("[%d]i2c_write_fail\n",data-> platform_data->irq_gpio);

	check_firmware_ready(client);

	//Percent Result
	for(loop = 0 ; loop < (CNT_MAX_CH -1) ; loop++)
	{
		rdata[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_CH1_PER_H + (loop*2));
		rdata[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_CH1_PER_L + (loop*2));
		per_result[loop] = MK_INT(rdata[0], rdata[1]);
		per_result[loop] /= 8;
	}
	
	//Duty
	for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
	{
		rdata[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H + (loop*2));
		rdata[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L + (loop*2));
		duty_val[loop] = MK_INT(rdata[0], rdata[1]);
	}

	
	PINFO("[%d] Result: %2d %2d %2d %6d %6d %6d %6d", data-> platform_data->irq_gpio, 
		per_result[0], per_result[1], per_result[2], duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
	
	return count;
}

static int get_bit(unsigned short x, int n) {
	return (x & (1 << n)) >> n;
}

/*
 static short get_abs(short x) {
	return ((x >= 0) ? x : -x);
}
*/
static ssize_t almf04_show_regproxdata(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char loop, rdata[2];
	int duty_val[CNT_MAX_CH];
	unsigned short per_result[(CNT_MAX_CH -1)];
	int nlength = 0;
	char buf_regproxdata[256] = "";
	char buf_line[256] = "";
	unsigned char sys_status;

	client = data->client;
	memset(buf_line, 0, sizeof(buf_line));
	memset(buf_regproxdata, 0, sizeof(buf_regproxdata));

	check_firmware_ready(client);

	sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
	
	//Percent Result
	for(loop = 0 ; loop < (CNT_MAX_CH -1) ; loop++)
	{
		rdata[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_CH1_PER_H + (loop*2));
		rdata[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_CH1_PER_L + (loop*2));
		per_result[loop] = MK_INT(rdata[0], rdata[1]);
		per_result[loop] /= 8;
	}
	
	//Duty
	for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
	{
		rdata[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H + (loop*2));
		rdata[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L + (loop*2));
		duty_val[loop] = MK_INT(rdata[0], rdata[1]);
	}

	//Calibration check bit delete
	//sprintf(buf_line, "[R] %6d %6d %6d %6d %6d %6d %6d\n",
	//	per_result[0], per_result[1], per_result[2], duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
	//Safe Duty Check 
	sprintf(buf_line, "[R]%6d %6d %6d %6d %6d %6d %6d %6d\n",
		(get_bit(sys_status, 2) == 0), per_result[0], per_result[1], per_result[2], duty_val[0], duty_val[1], duty_val[2], duty_val[3]);

	nlength = strlen(buf_regproxdata);
	strcpy(&buf_regproxdata[nlength], buf_line);

	return sprintf(buf, "%s", buf_regproxdata);
}

static ssize_t almf04_store_checkallnear(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val == 0)
		check_allnear = false;
	else if (val == 1)
		check_allnear = true;

	printk("almf04_store_checkallnear %d\n",check_allnear);
	return count;
}

static ssize_t almf04_show_count_inputpins(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count_inputpins = 0;

	struct almf04_data *data = dev_get_drvdata(dev);

	count_inputpins = data->platform_data->input_pins_num;
	if (count_inputpins > 1) {
		if (valid_multiple_input_pins(data) == false)
			count_inputpins = 1;
	}
	return sprintf(buf, "%d\n", count_inputpins);
}

static ssize_t almf04_store_firmware(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	const char *fw_name = NULL;
	struct almf04_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
	client = data->client;

	fw_name = data->platform_data->fw_name;
	load_firmware(data, client, fw_name);
	return count;
}

static ssize_t almf04_show_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned short main_version, sub_version;
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = dev_get_drvdata(dev);
	char buf_line[256] = "";
	int nlength = 0;
	char buf_regproxdata[256] = "";
	client = data->client;

	memset(buf_line, 0, sizeof(buf_line));
	onoff_sensor(data,ENABLE_SENSOR_PINS);

	mdelay(200);

	main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
	sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);
	PINFO("###########ic version : %d.%02d###########", main_version, sub_version);

	sprintf(buf_line, "%d.%02d\n",main_version, sub_version);
	nlength = strlen(buf_regproxdata);
	strcpy(&buf_regproxdata[nlength], buf_line);

	return sprintf(buf,"%s", buf_regproxdata);
}

static ssize_t almf04_show_check_far(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char loop, rdata[2];
	int duty_val[CNT_MAX_CH];
	unsigned char sys_status;
	unsigned int check_result = 0;

	client = data->client;
	mutex_lock(&data->update_lock);
	check_firmware_ready(client);

	//Duty
	for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
	{
		rdata[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H + (loop*2));
		rdata[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L + (loop*2));
		duty_val[loop] = MK_INT(rdata[0], rdata[1]);
	}
	
	sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

	/* Unsafe Duty Range Check */
    if(get_bit(sys_status, 2) == 1)
       check_result += 1;

	if(gpio_get_value(data->platform_data->irq_gpio) == 1)
	{
		PINFO(" ALMF04 Check_Far IRQ Status NEAR[%d]\n",gpio_get_value(data->platform_data->irq_gpio));
		check_result += 1;
	}
	
	//============================================================//
	//[200512] ADS Change
	//[START]=====================================================//
	//if (check_result > 0)
    //{
	//	PINFO(" ALMF04 [fail] 1.safe_duty: %d, cr: %d, cs1: %d, cs2: %d, cs3: %d\n",
	//		(get_bit(sys_status, 2) == 0), duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
    //}
    //else
    //{
	//	PINFO(" ALMF04 [PASS] 2.safe_duty: %d, cr: %d, cs1: %d, cs2: %d, cs3: %d\n",
	//		(get_bit(sys_status, 2) == 0), duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
    //}
	if (check_result > 0)
    {
		PINFO(" ALMF04 [fail] 1.safe_duty: %d, cr: %d, ch1: %d, ch2: %d, ch3: %d\n",
			(get_bit(sys_status, 2) == 0), duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
    }
    else
    {
		PINFO(" ALMF04 [PASS] 2.safe_duty: %d, cr: %d, ch1: %d, ch2: %d, ch3: %d\n",
			(get_bit(sys_status, 2) == 0), duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
    }
	//[END]=======================================================//

	mutex_unlock(&data->update_lock);
	return sprintf(buf,"%d",1);
}

static ssize_t almf04_show_check_mid(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);

	unsigned char loop, rdata[2];
	int duty_val[CNT_MAX_CH];
	unsigned char sys_status;
	unsigned int check_result = 0;

	client = data->client;
	mutex_lock(&data->update_lock);
	check_firmware_ready(client);

	//Duty
	for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
	{
		rdata[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H + (loop*2));
		rdata[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L + (loop*2));
		duty_val[loop] = MK_INT(rdata[0], rdata[1]);
	}
	
	sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

	/* Unsafe Duty Range Check */
    if(get_bit(sys_status, 2) == 1)
       check_result += 1;

	//============================================================//
	//[200512] ADS Change
	//[START]=====================================================//
	//if (check_result > 0)
    //{
	//	PINFO(" ALMF04 [fail] 1.safe_duty: %d, cr: %d, cs1: %d, cs2: %d, cs3: %d\n",
	//		(get_bit(sys_status, 2) == 0), duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
    //}
    //else
    //{
	//	PINFO(" ALMF04 [PASS] 2.safe_duty: %d, cr: %d, cs1: %d, cs2: %d, cs3: %d\n",
	//		(get_bit(sys_status, 2) == 0), duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
    //}
	if (check_result > 0)
    {
		PINFO(" ALMF04 [fail] 1.safe_duty: %d, cr: %d, ch1: %d, ch2: %d, ch3: %d\n",
			(get_bit(sys_status, 2) == 0), duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
    }
    else
    {
		PINFO(" ALMF04 [PASS] 2.safe_duty: %d, cr: %d, ch1: %d, ch2: %d, ch3: %d\n",
			(get_bit(sys_status, 2) == 0), duty_val[0], duty_val[1], duty_val[2], duty_val[3]);
    }
	//[END]=======================================================//

	mutex_unlock(&data->update_lock);
	return sprintf(buf,"%d",0);
}

#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
static void almf04_sw_reset(struct almf04_data *data, struct i2c_client *client)
{ 
    unsigned char sys_status;
    int ret;
 
    if( on_sensor == true )
    {
	//============================================================//
	//[200513] ADS Add
	//[START]=====================================================//
	client = data->client;
	mutex_lock(&data->update_lock);
	//[END]=======================================================//
        check_firmware_ready(client);
        sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

        if(get_bit(sys_status, 0) == 1){
            PINFO("Store SWReset Fail[ Busy(Before) ]");
            goto sw_reset_err;
        }

        ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x01);
        if(ret){
            PINFO("[%d]i2c_write_fail\n",data-> platform_data->irq_gpio);
            goto sw_reset_err;
        }

        check_firmware_ready(client);
        sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

        if(get_bit(sys_status, 0) == 1){
            PINFO("Store SWReset Fail[ Busy(After) ]");
            goto sw_reset_err;
        }
        PINFO("Done SW Reset : IC %s", on_sensor ? "ON" : "OFF");
sw_reset_err:
	mutex_unlock(&data->update_lock);
    }
    else
        PINFO("Ignore SW Reset caused by IC %s", on_sensor ? "ON" : "OFF");

    return;
}
#endif

static ssize_t almf04_store_swrst(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
    struct almf04_data *data = dev_get_drvdata(dev);
    struct i2c_client *client = to_i2c_client(dev);
    client = data->client;

    almf04_sw_reset(data, client);
    return count;
}

//============================================================//
//[200512] ADS Add
//[START]=====================================================//
static ssize_t almf04_show_ch_output(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char output;

	client = data->client;
	mutex_lock(&data->update_lock);
	check_firmware_ready(client);

	output = i2c_smbus_read_byte_data(client, I2C_ADDR_TCH_OUTPUT);

	PINFO(" ALMF04 [CH OUTPUT] ABN_OUT:%d, ch1: %d, ch2: %d, ch3: %d\n",
			get_bit(output, 6), get_bit(output, 2), get_bit(output, 3), get_bit(output, 4));
	
	mutex_unlock(&data->update_lock);

	//ABN_OUT CH1 CH2 CH3
	return sprintf(buf,"%d %d %d %d",get_bit(output, 6), get_bit(output, 2), get_bit(output, 3), get_bit(output, 4));
}

static ssize_t almf04_show_used_ch(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char use_ch;

	client = data->client;
	mutex_lock(&data->update_lock);
	check_firmware_ready(client);

	use_ch = i2c_smbus_read_byte_data(client, I2C_ADDR_USE_CH_INF);

	PINFO(" ALMF04 [CH IN USE] ch1: %d, ch2: %d, ch3: %d\n",
			get_bit(use_ch, 0), get_bit(use_ch, 1), get_bit(use_ch, 2));
	
	mutex_unlock(&data->update_lock);

	//CH1 CH2 CH3
	return sprintf(buf,"%d %d %d",get_bit(use_ch, 0), get_bit(use_ch, 1), get_bit(use_ch, 2));
}

static ssize_t almf04_show_check_busy(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct almf04_data *data = i2c_get_clientdata(client);
	unsigned char sys_status;

	client = data->client;
	mutex_lock(&data->update_lock);
	
	sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

	PINFO(" ALMF04 [System Busy : %d]\n", get_bit(sys_status, 0));
	
	mutex_unlock(&data->update_lock);

	//1:Busy
	return sprintf(buf,"%d",get_bit(sys_status, 0));
}

//[END]=======================================================//

static DEVICE_ATTR(onoff,        0664, NULL, almf04_store_onoffsensor);
static DEVICE_ATTR(proxstatus,   0664, almf04_show_proxstatus, NULL);
static DEVICE_ATTR(reg_ctrl,     0664, almf04_show_reg, almf04_store_reg);
static DEVICE_ATTR(write_reg,    0664, NULL, almf04_store_write_reg);
static DEVICE_ATTR(read_reg,     0664, NULL, almf04_store_read_reg);
static DEVICE_ATTR(regproxdata,  0664, almf04_show_regproxdata, NULL);
static DEVICE_ATTR(regreset,     0664, NULL, almf04_store_regreset);
static DEVICE_ATTR(checkallnear, 0664, NULL, almf04_store_checkallnear);
static DEVICE_ATTR(cntinputpins, 0664, almf04_show_count_inputpins, NULL);
static DEVICE_ATTR(regproxctrl0, 0664, almf04_show_regproxctrl0, NULL);
static DEVICE_ATTR(download,     0664, NULL, almf04_store_firmware);
static DEVICE_ATTR(version,      0664, almf04_show_version, NULL);
static DEVICE_ATTR(check_far,    0664, almf04_show_check_far, NULL);
static DEVICE_ATTR(check_mid,    0664, almf04_show_check_mid, NULL);
static DEVICE_ATTR(swrst,        0664, NULL, almf04_store_swrst);

//============================================================//
//[200512] ADS Add
//[START]=====================================================//
static DEVICE_ATTR(ch_output,   0664, almf04_show_ch_output, NULL);
static DEVICE_ATTR(used_ch,		0664, almf04_show_used_ch, NULL);
static DEVICE_ATTR(check_busy,	0664, almf04_show_check_busy, NULL);
//[END]=======================================================//


static struct attribute *almf04_attributes[] = {
	&dev_attr_onoff.attr,
	&dev_attr_proxstatus.attr,
	&dev_attr_reg_ctrl.attr,
	&dev_attr_write_reg.attr,
	&dev_attr_read_reg.attr,
	&dev_attr_regproxdata.attr,
	&dev_attr_regreset.attr,
	&dev_attr_checkallnear.attr,
	&dev_attr_cntinputpins.attr,
	&dev_attr_regproxctrl0.attr,
	&dev_attr_download.attr,
	&dev_attr_version.attr,
	&dev_attr_check_far.attr,
	&dev_attr_check_mid.attr,
	&dev_attr_swrst.attr,
	//============================================================//
	//[200512] ADS Add
	//[START]=====================================================//
	&dev_attr_ch_output.attr,
	&dev_attr_used_ch.attr,
	&dev_attr_check_busy.attr,
	//[END]=======================================================//
	NULL,
};

static struct attribute_group almf04_attr_group = {
	.attrs = almf04_attributes,
};

static void almf04_reschedule_work(struct almf04_data *data,
		unsigned long delay)
{
	int ret;
	struct i2c_client *client = data->client;
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	if (gpio_get_value(data->platform_data->chip_enable) == 0) { // if power on
		dev_err(&client->dev, "almf04_reschedule_work : set wake lock timeout!\n");
		pm_wakeup_event(&client->dev,1500);
		//cancel_delayed_work(&data->dwork);
                if (atomic_read(&pm_suspend_flag) == DEV_PM_RESUME){
                    ret = queue_delayed_work(almf04_workqueue, &data->dwork, delay);
                    if (ret < 0) {
                        PINFO("queue_work fail, ret = %d", ret);
                    }
                } else {
                    atomic_set(&pm_suspend_flag, DEV_PM_SUSPEND_IRQ);
                    PINFO("I2C is not yet ready... try queue_delayed_work after resume");
		}
	} else {
		PINFO("ALMF04 cap sensor enable pin is already high... power off status");
	}
}

/* assume this is ISR */
static irqreturn_t almf04_interrupt(int vec, void *info)
{
	struct i2c_client *client = (struct i2c_client *)info;
	struct almf04_data *data = i2c_get_clientdata(client);
	int tmp = -1;
	tmp = atomic_read(&data->i2c_status);

	if (gpio_get_value(data->platform_data->chip_enable) == 0) { // if power on
		dev_err(&client->dev,"almf04_interrupt\n");
		almf04_reschedule_work(data, 0);
	}

	return IRQ_HANDLED;
}

static void almf04_work_handler(struct work_struct *work)
{
	struct almf04_data *data = container_of(work, struct almf04_data, dwork.work);
	struct i2c_client *client = data->client;
	int irq_state;

#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
	if(probe_end_flag == true) {
#endif
	data->touch_out = i2c_smbus_read_byte_data(client, I2C_ADDR_TCH_OUTPUT);
	irq_state = gpio_get_value(data->platform_data->irq_gpio);
    PINFO("ALMF04 touch_out[%x] irq_state[%d]", data->touch_out, irq_state);

    /* When I2C fail and abnormal status*/
    if (data->touch_out < 0)
    {
		PINFO("ALMF04 I2C Error[%d]", data->touch_out);
        input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
        input_sync(data->input_dev_cap);

        PINFO("ALMF04 [%d]NEAR ",data->platform_data->irq_gpio);
    }
    else
    {
        if (gpio_get_value(data->platform_data->chip_enable) == 1) { // if power off
			PINFO("ALMF04 cap sensor is already power off : touch_out(0x%x)", data->touch_out);
			return;
	    }
		
		//==============================================================================================//
		/* General Mode Touch */
		//----------------------------------------------------------------------------------------------//
		if(get_bit(data->touch_out, 7) == 0)
		{
			/* FAR */
			if (irq_state == 0 && (data->touch_out == (CH3_FAR | CH2_FAR | CH1_FAR)))
			{
			   data->cap_detection = 0;

			   input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* NEAR-to-FAR detection */
			   input_sync(data->input_dev_cap);

			   PINFO("ALMF04 [%d]FAR ",data->platform_data->irq_gpio);
			}
			/* NEAR */
			else
			{
    		   data->cap_detection = 1;

			   input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
			   input_sync(data->input_dev_cap);

			   PINFO("ALMF04 [%d]NEAR ",data->platform_data->irq_gpio);
			}
		}
		//==============================================================================================//
		/* Scenario Mode Touch */
		//----------------------------------------------------------------------------------------------//
		else
		{
			/* FAR */
			//if (irq_state == 0 && ((data->touch_out & 0x03) == SNR_FAR))
			if (irq_state == 0 && ((data->touch_out & 0x43) == SNR_FAR))		//Default Near Bit6 --> 1
			{
			   data->cap_detection = 0;

			   input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* NEAR-to-FAR detection */
			   input_sync(data->input_dev_cap);

			   PINFO("ALMF04 [%d]FAR(SM) ",data->platform_data->irq_gpio);
			}
			/* NEAR */
			else
			{
    		   data->cap_detection = 1;

			   input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
			   input_sync(data->input_dev_cap);

			   PINFO("ALMF04 [%d]NEAR(SM) ",data->platform_data->irq_gpio);
			}
		}
		//==============================================================================================//
        PINFO("ALMF04 [%d] Work Handler done",data->platform_data->irq_gpio);

	}
#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
	}
	else{
		PINFO("ALMF04 probe_end_flag = False[%d]",probe_end_flag);
	}
#endif
}

static int sensor_regulator_configure(struct almf04_data *data, bool on)
{
	struct i2c_client *client = data->client;
	struct almf04_platform_data *pdata = data->platform_data;
	int rc;

	if (on == false)
		goto hw_shutdown;

	pdata->vcc_ana = regulator_get(&client->dev, "Adsemicon,vdd_ana");
	if (IS_ERR(pdata->vcc_ana)) {
		rc = PTR_ERR(pdata->vcc_ana);
		dev_err(&client->dev,
				"Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->vcc_ana) > 0) {
		rc = regulator_set_voltage(pdata->vcc_ana, pdata->vdd_ana_supply_min,
				pdata->vdd_ana_supply_max);

		if (rc) {
			dev_err(&client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}

	return 0;

error_set_vtg_vcc_ana:
	regulator_put(pdata->vcc_ana);
	return rc;

hw_shutdown:
	if (regulator_count_voltages(pdata->vcc_ana) > 0)
		regulator_set_voltage(pdata->vcc_ana, 0, pdata->vdd_ana_supply_max);

	regulator_put(pdata->vcc_ana);
	regulator_put(pdata->vcc_dig);

	if (pdata->i2c_pull_up) {
		regulator_put(pdata->vcc_i2c);
	}
	return 0;
}

static int sensor_regulator_power_on(struct almf04_data *data, bool on)
{
	struct i2c_client *client = data->client;
	struct almf04_platform_data *pdata = data->platform_data;

	int rc;

	if (on == false)
		goto power_off;
    
	rc = regulator_set_load(pdata->vcc_ana, pdata->vdd_ana_load_ua);
	if (rc < 0) {
		dev_err(&client->dev,
			"Regulator vcc_ana set_opt failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(pdata->vcc_ana);
	if (rc) {
		dev_err(&client->dev,
				"Regulator vcc_ana enable failed rc=%d\n", rc);
	}
	return 0;

power_off:
	regulator_disable(pdata->vcc_ana);
	if (pdata->i2c_pull_up) {
		regulator_disable(pdata->vcc_i2c);
	}
	return 0;
}

static int sensor_platform_hw_power_on(struct i2c_client *client, bool on)
{
	sensor_regulator_power_on(i2c_get_clientdata(client), on);
	return 0;
}

static int sensor_platform_hw_init(struct i2c_client *client)
{
	struct almf04_data *data = i2c_get_clientdata(client);
	int error;

	error = sensor_regulator_configure(data, true);

	error = gpio_request(data->platform_data->chip_enable, "almf04_2nd_chip_enable");
	if(error) {
		PINFO(" ALMF04 chip_enable request fail\n");
	}
        // chip_enable pin must be set as output
        gpio_set_value(data->platform_data->chip_enable, 0);    /*chip_en pin - low : on, high : off*/

	PINFO(" ALMF04 gpio direction output ok\n");

	if (gpio_is_valid(data->platform_data->irq_gpio)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(data->platform_data->irq_gpio, "almf04_2nd_irq_gpio");
		if (error) {
			PINFO(" ALMF04 unable to request gpio [%d]\n",
					data->platform_data->irq_gpio);
		}
		error = gpio_direction_input(data->platform_data->irq_gpio);
		if (error) {
			PINFO(" ALMF04 unable to set direction for gpio [%d]\n",
					data->platform_data->irq_gpio);
		}
		data->irq = client->irq = gpio_to_irq(data->platform_data->irq_gpio);
	} else {
		PINFO(" ALMF04 irq gpio not provided\n");
	}
	PINFO(" ALMF04 sensor_platform_hw_init end\n");
	return 0;
}

static void sensor_platform_hw_exit(struct i2c_client *client)
{
	struct almf04_data *data = i2c_get_clientdata(client);;

	sensor_regulator_configure(data, false);

	if (gpio_is_valid(data->platform_data->irq_gpio))
		gpio_free(data->platform_data->irq_gpio);
	PINFO(" ALMF04 sensor_platform_hw_exit entered\n");
}

static int sensor_parse_dt(struct device *dev,
		struct almf04_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	int ret, err = 0;
	struct sensor_dt_to_pdata_map *itr;
	struct sensor_dt_to_pdata_map map[] = {
#if defined (CONFIG_MACH_SDM845_JUDYPN)
		{"Adsemicon,irq-gpio",		&pdata->irq_gpio,		DT_REQUIRED,	DT_GPIO,	0},
		{"Adsemicon,irq-gpio2",		&pdata->irq_gpio2,		DT_REQUIRED,	DT_GPIO,	0},
#else
		{"Adsemicon,irq-gpio",		&pdata->irq_gpio,		DT_REQUIRED,	DT_GPIO,	0},
#endif
		{"Adsemicon,vdd_ana_supply_min",	&pdata->vdd_ana_supply_min,	DT_SUGGESTED,	DT_U32,		0},
		{"Adsemicon,vdd_ana_supply_max",	&pdata->vdd_ana_supply_max,	DT_SUGGESTED,	DT_U32,		0},
		{"Adsemicon,vdd_ana_load_ua",	&pdata->vdd_ana_load_ua,	DT_SUGGESTED,	DT_U32,		0},
		{"Adsemicon,chip_enable",   &pdata->chip_enable,    DT_SUGGESTED,   DT_GPIO,     0},
		{"Adsemicon,InputPinsNum",         &pdata->input_pins_num,      DT_SUGGESTED,   DT_U32,  0},
		{"Adsemicon,fw_name",              &pdata->fw_name,             DT_SUGGESTED,   DT_STRING,  0},
		/* add */
		{NULL,				NULL,				0,		0,		0},
	};

	for (itr = map; itr->dt_name ; ++itr) {
		switch (itr->type) {
			case DT_GPIO:
				ret = of_get_named_gpio(np, itr->dt_name, 0);
				if (ret >= 0) {
					*((int *) itr->ptr_data) = ret;
					ret = 0;
				}
				break;
			case DT_U32:
				ret = of_property_read_u32(np, itr->dt_name,
						(u32 *) itr->ptr_data);
				break;
			case DT_BOOL:
				*((bool *) itr->ptr_data) =
					of_property_read_bool(np, itr->dt_name);
				ret = 0;
				break;
			case DT_STRING:
				ret = of_property_read_string(np, itr->dt_name, itr->ptr_data);
				break;
			default:
				PINFO(" ALMF04 %d is an unknown DT entry type\n",
						itr->type);
				ret = -EBADE;
		}

		PINFO(" ALMF04 DT entry ret:%d name:%s val:%d\n",
				ret, itr->dt_name, *((int *)itr->ptr_data));

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;

			if (itr->status < DT_OPTIONAL) {
				PINFO(" ALMF04 Missing '%s' DT entry\n",
						itr->dt_name);

				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQUIRED && !err)
					err = ret;
			}
		}
	}

	/* set functions of platform data */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->power_on = sensor_platform_hw_power_on;
	/*pdata->ppcount = 12;	//no need to set, dt_parse */

	return err;

	return 0;
}

#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
void sar_2nd_controller_notify_connect(u32 type, bool is_connected)
{
    if( (&sar_controller_atomic_notifier)->head != NULL ){
        PINFO("[sar_2nd]notifier_callback function is registered is_connected %d", is_connected );
        atomic_notifier_call_chain(&sar_controller_atomic_notifier, USB_CONNECTION, &type);
    }
    else 
        PINFO("[sar_2nd]notifier_callback function isn't registered %d", is_connected );
}
EXPORT_SYMBOL(sar_2nd_controller_notify_connect);


static void sar_controller_callback_func(struct work_struct *work)
{
    struct almf04_data *data = container_of(work, struct almf04_data, callback_dwork.work);
    struct i2c_client *client = data->client;

    switch(data->event){
        case USB_CONNECTION:
            PINFO("SW Reset!! event: %d data: %d", data->event, data->data);
            almf04_sw_reset(data, client);
            break;
    }
}

static int almf04_atomic_notifier_callback(struct notifier_block *this,
                unsigned long event, void *data)
{
    struct almf04_data *almf04_data = container_of(this, struct almf04_data, atomic_notif );

    almf04_data->event = event;
    almf04_data->data = *(int*)data;
    schedule_delayed_work(&almf04_data->callback_dwork, 0);
    return 0;
}
#endif

static int almf04_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct almf04_data *data;
#ifdef CONFIG_OF
	struct almf04_platform_data *platform_data;
#endif
	int err = 0;

        /* Check for support by carrier */
        /* some operator doesn't support cap sensor. */
        if (lge_get_capsensor() == 0)
        {
            PINFO("cap sensor isn't supported in this operator");
            return -ENODEV;
        }

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		return -EIO;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct almf04_data), GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}
	PINFO("ALMF04 probe, devm_kzalloc complete\n");

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
				sizeof(struct almf04_platform_data), GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		data->platform_data = platform_data;
		client->dev.platform_data = platform_data;
		err = sensor_parse_dt(&client->dev, platform_data);
		if (err)
			return err;
#if defined(CONFIG_MACH_SDM845_JUDYPN)
		if(strcmp(lge_get_board_subrevision(),"subrev_0"))
		{
			PINFO("**Capsensor BLSP Change [%d] to [%d] for	subrev**",platform_data->irq_gpio, platform_data->irq_gpio2);
			platform_data->irq_gpio = platform_data->irq_gpio2;
		}
#endif
	} else {
		platform_data = client->dev.platform_data;
	}
#endif
	data->client = client;
	almf04_i2c_client = client;
	i2c_set_clientdata(client, data);
	data->cap_detection = 0;

#ifdef CONFIG_OF
	/* h/w initialization */
	if (platform_data->init)
		err = platform_data->init(client);

	if (platform_data->power_on)
		err = platform_data->power_on(client, true);
#endif
	PINFO("**Capsensor BLSP[%d]",platform_data->irq_gpio);

	client->adapter->retries = 15;

	if (client->adapter->retries == 0)
		goto exit;

	atomic_set(&data->i2c_status, ALMF04_STATUS_RESUME);

	mutex_init(&data->update_lock);
	mutex_init(&data->enable_lock);

	INIT_DELAYED_WORK(&data->dwork, almf04_work_handler);
#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
        INIT_DELAYED_WORK(&data->callback_dwork, sar_controller_callback_func);
#endif

	err = request_irq(client->irq, almf04_interrupt, IRQ_TYPE_EDGE_FALLING|IRQ_TYPE_EDGE_RISING,
				 ALMF04_DRV_NAME, (void *)client);
	if(err) {
		PINFO("ALMF04 chip_enable request fail\n");
		goto exit_irq_init_failed;
	}

	err = enable_irq_wake(data->irq);

	data->input_dev_cap = input_allocate_device();
	if (!data->input_dev_cap) {
		PINFO("ALMF04 Failed to allocate input device cap !\n");
		return -ENOMEM;
	}

	set_bit(EV_ABS, data->input_dev_cap->evbit);

	input_set_abs_params(data->input_dev_cap, ABS_DISTANCE, 0, 1, 0, 0);

	data->input_dev_cap->name = ALMF04_DRV_NAME;
	data->input_dev_cap->dev.init_name = ALMF04_DRV_NAME;
	data->input_dev_cap->id.bustype = BUS_I2C;

	input_set_drvdata(data->input_dev_cap, data);

	err = input_register_device(data->input_dev_cap);
	if (err) {
		PINFO("ALMF04 Unable to register input device cap(%s)\n",
				data->input_dev_cap->name);
                return -ENOMEM;
	}

	if (data->platform_data->fw_name) {
		err = load_firmware(data, client, data->platform_data->fw_name);
		if (err) {
			PINFO("ALMF04 Failed to request firmware\n");
			goto exit_load_firmware_failed;
		}
	}
	PINFO("sysfs create start!\n");
	err = sysfs_create_group(&data->input_dev_cap->dev.kobj, &almf04_attr_group);
	if (err)
		PINFO("input sysfs create fail!\n");

	err = sysfs_create_group(&client->dev.kobj, &almf04_attr_group);
        if (err)
		PINFO("ALMF04 sysfs create fail!\n");

	/* default sensor off */
#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
	probe_end_flag = true;
#endif
	onoff_sensor(data, DISABLE_SENSOR_PINS);
	PINFO("interrupt is hooked\n");

#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
        data->atomic_notif.notifier_call = almf04_atomic_notifier_callback;
        err = atomic_notifier_chain_register(&sar_controller_atomic_notifier, &data->atomic_notif);
        if (err<0){
            PINFO("almf04 failed to register almf04 atomic_notify callback\n");
        }
        PINFO("notify callback is registered\n");
#endif
        atomic_set(&pm_suspend_flag, DEV_PM_RESUME); // initial value
	return 0;

exit_irq_init_failed:
        // chip_enable pin must be set as output and when driver probe failed, chip_enable pin should be set as high to avoid current leakage
        gpio_set_value(data->platform_data->chip_enable, 1);          /*chip_en pin - low : on, high : off*/
	mutex_destroy(&data->update_lock);
	mutex_destroy(&data->enable_lock);
exit_load_firmware_failed:
        if (data->input_dev_cap != NULL) {
            input_unregister_device(data->input_dev_cap);
        }
        PINFO("exit_irq_init_failed ALMF04 Error");
	free_irq(data->irq, client);
        PINFO("exit_free_irq ALMF04 Error");
exit:
	PINFO("exit ALMF04 Error");

	return err;
}

static int almf04_remove(struct i2c_client *client)
{

	struct almf04_data *data = i2c_get_clientdata(client);
	struct almf04_platform_data *pdata = data->platform_data;


	disable_irq_wake(client->irq);

	free_irq(client->irq, client);

	if (pdata->power_on)
		pdata->power_on(client, false);

	if (pdata->exit)
		pdata->exit(client);
#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
        atomic_notifier_chain_unregister(&sar_controller_atomic_notifier, &data->atomic_notif);
#endif

	mutex_destroy(&data->update_lock);
	mutex_destroy(&data->enable_lock);
	PINFO("ALMF04 remove\n");
	return 0;
}

static const struct i2c_device_id almf04_id[] = {
	{ "almf04_2nd", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, almf04_id);

#ifdef CONFIG_OF
static struct of_device_id almf04_match_table[] = {
	{ .compatible = "adsemicon,almf04_2nd",},
	{ },
};
#else
#define almf04_match_table NULL
#endif

static int almf04_pm_suspend(struct device *dev)
{
    atomic_set(&pm_suspend_flag, DEV_PM_SUSPEND);
    return 0;
}

static int almf04_pm_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct almf04_data *data = i2c_get_clientdata(client);
    int ret = 0;

    if (atomic_read(&pm_suspend_flag) == DEV_PM_SUSPEND_IRQ) {
        if (gpio_get_value(data->platform_data->chip_enable) == 0) { // if power on
            PINFO("queue_delayed_work now.");
            ret = queue_delayed_work(almf04_workqueue, &data->dwork, 0);
            if (ret < 0) {
                PINFO("queue_work fail, ret = %d", ret);
            }
        }
        else {
            PINFO("cap sensor enable pin is already high... power off status");
        }
    }

    atomic_set(&pm_suspend_flag, DEV_PM_RESUME);
    return 0;
}

static const struct dev_pm_ops almf04_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(
        almf04_pm_suspend, //Get call when suspend is happening
        almf04_pm_resume   //Get call when resume is happening
    )
};

static struct i2c_driver almf04_driver = {
	.driver = {
		.name   = ALMF04_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = almf04_match_table,
        .pm     = &almf04_pm_ops,
	},
	.probe  = almf04_probe,
	.remove = almf04_remove,
	.id_table = almf04_id,
};
static void async_almf04_init(void *data, async_cookie_t cookie)
{
	PINFO("ALMF04 init Proximity driver: release.\n");
	almf04_workqueue = create_workqueue("capsensor");
#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
        sar_controller_callback_workqueue = create_workqueue("sar_controller_callback");
#endif
	i2c_add_driver(&almf04_driver);
}

static int __init almf04_init(void)
{
    async_schedule(async_almf04_init, NULL);
	return 0;
}

static void __exit almf04_exit(void)
{
	PINFO("ALMF04 Proximity driver: release.\n");
	if (almf04_workqueue){
            destroy_workqueue(almf04_workqueue);
        }
#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
        if(sar_controller_callback_workqueue){
            destroy_workqueue(sar_controller_callback_workqueue);
        }
#endif

	almf04_workqueue = NULL;
#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
        sar_controller_callback_workqueue = NULL;
#endif
	i2c_del_driver(&almf04_driver);
}

MODULE_DESCRIPTION("ALMF04 cap sensor driver");
MODULE_LICENSE("GPL");

module_init(almf04_init);
module_exit(almf04_exit);


