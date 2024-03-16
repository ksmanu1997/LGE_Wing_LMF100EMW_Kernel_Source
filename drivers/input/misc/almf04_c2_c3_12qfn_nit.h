#ifndef __ALMF04_12QFN_H__
#define __ALMF04_12QFN_H__

// To sync between LGE and ADSem, ADSem only increase the driver version.
// if LGE modify the driver, LGE should write history only without increasing driver version
#define ALMF04_DRIVER_VERSION 0x0003

/*
 * History
 * This section contains comments describing changes made to the module.
 * Notice that changes are listed in reverse chronolgical order
 *
 * when           version         who            what
 * -------------------------------------------------------------------------------------------------------------
 * 2020-06-04     0x0003          LGE            when probe failed, current issue occurred,
 *                                               so set gpio high when probe func failed becaused ic is working as active low
 * 2020-06-03     0x0003          ADSem          1. add new register of Init Code (0x26)
 *                                               2. change the almf04_show_regproxdata() function internal per_result variable type
 * 2020-05-29     0x0002          LGE            add callback function for IC SW Reset.
 *                                               ex) when detecting usb connection or change capacitive.
 * 2020-05-28     0x0001          ADSem          create almf04 driver for auto calibration with firmware(3.10)
 * -------------------------------------------------------------------------------------------------------------
 */

/*! ERROR LOG LEVEL */
#define LOG_LEVEL_E 3
/*! NOTICE LOG LEVEL */
#define LOG_LEVEL_N 5
/*! INFORMATION LOG LEVEL */
#define LOG_LEVEL_I 6
/*! DEBUG LOG LEVEL */
#define LOG_LEVEL_D 7

#ifndef LOG_LEVEL
/*! LOG LEVEL DEFINATION */
#define LOG_LEVEL LOG_LEVEL_I
#endif

#ifndef MODULE_TAG
/*! MODULE TAG DEFINATION */
#define MODULE_TAG "[ALMF04] "
#endif

#if (LOG_LEVEL >= LOG_LEVEL_E)
/*! print error message */
#define PERR(fmt, args...) \
	pr_err(MODULE_TAG \
	"%s: " fmt "\n", __func__, ##args)
#else
/*! invalid message */
#define PERR(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_N)
/*! print notice message */
#define PNOTICE(fmt, args...) \
	pr_notice(MODULE_TAG \
	"%s: " fmt "\n", __func__, ##args)
#else
/*! invalid message */
#define PNOTICE(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_I)
/*! print information message */
#define PINFO(fmt, args...) pr_info(MODULE_TAG \
	"%s: " fmt "\n", __func__, ##args)
#else
/*! invalid message */
#define PINFO(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_D)
/*! print debug message */
#define PDEBUG(fmt, args...) pr_devel(MODULE_TAG \
	"%s: " fmt "\n", __func__, ##args)
#else
/*! invalid message */
#define PDEBUG(fmt, args...)
#endif
#define USE_ALMF04				//ALMF04 Use status

#define SZ_PAGE_DATA                64

#define ADDR_EFLA_STS               0xFF	//eflash status register
#define ADDR_EFLA_PAGE_L            0xFD	//eflash page
#define ADDR_EFLA_PAGE_H            0xFE	//eflash page
#define ADDR_EFLA_CTRL              0xFC	//eflash control register

#if defined(USE_ALMF04)
#define ADDR_ROM_SAFE		    0xFB	//eflash write 보호 루틴
#define VAL_ROM_MASK1		    0x02
#define VAL_ROM_MASK2		    0x06

#define EMD_ALL_ERASE		    (0x07 << 1)
#define EMD_PG_ERASE		    (0x04 << 1)
#define EMD_PG_WRITE		    (0x08 << 1)
#define EMD_PG_READ		    0x00

#define CMD_EEP_START		    0x01
#define CMD_EUM_START		    0x03

#define FLAG_DONE                   0x03
#define FLAG_DONE_ERASE             0x03
#else
#define CMD_EFL_L_WR                0x01	//Eflash Write
#define CMD_EFL_RD                  0x03	//Eflash Read
#define CMD_EFL_ERASE_ALL           0x07	//Eflash All Page Erase

#define CMD_EUM_WR                  0x21	//Extra user memory write
#define CMD_EUM_RD                  0x23	//Extra user memory read
#define CMD_EUM_ERASE               0x25	//Extra user memory erase

#define FLAG_DONE                   0x03
#define FLAG_DONE_ERASE             0x02
#endif

#define FLAG_FUSE                   1
#define FLAG_FW                     2

#define FL_EFLA_TIMEOUT_CNT         20
#define IC_TIMEOUT_CNT				5

#define RTN_FAIL                    0
#define RTN_SUCC                    1
#define RTN_TIMEOUT                 2

#define ON                          1
#define OFF                         2

enum {
    DEV_PM_RESUME = 0,
    DEV_PM_SUSPEND,
    DEV_PM_SUSPEND_IRQ,
};

#if 1 // debugging calibration paused
#define CAP_CAL_RESULT_PASS			0 // "1"
#define CAP_CAL_RESULT_FAIL			"0"
#endif

//Register Setting version Check Using
//#define REG_VER_CHK_USE

#define IDX_REG_VER						(CNT_INITCODE - 2)

/* I2C Register */
#define	I2C_ADDR_SSTVT_CH1_H			0x01
#define	I2C_ADDR_SSTVT_CH1_L			0x02
#define	I2C_ADDR_SSTVT_CH2_H			0x03
#define	I2C_ADDR_SSTVT_CH2_L			0x04
#define	I2C_ADDR_SSTVT_CH3_H			0x05
#define	I2C_ADDR_SSTVT_CH3_L			0x06

#define	I2C_ADDR_SFDT1_MIN				0x1F
#define	I2C_ADDR_SFDT1_MAX				0x20
#define	I2C_ADDR_SFDT2_MIN				0x21
#define	I2C_ADDR_SFDT2_MAX				0x22
#define	I2C_ADDR_SFDT3_MIN				0x23
#define	I2C_ADDR_SFDT3_MAX				0x24

//============================================================//
//[200512] ADS Add
//[START]=====================================================//
#define I2C_ADDR_USE_CH_INF				0x2A
//[END]=======================================================//

#define	I2C_ADDR_SYS_CTRL				0x31
#define	I2C_ADDR_SYS_STAT				0x32
#define	I2C_ADDR_SYS_STAT2				0x33
#define	I2C_ADDR_TCH_OUTPUT				0x34
#define	I2C_ADDR_CH1_PER_H				0x35
#define	I2C_ADDR_CH1_PER_L				0x36
#define	I2C_ADDR_CH2_PER_H				0x37
#define	I2C_ADDR_CH2_PER_L				0x38
#define	I2C_ADDR_CH3_PER_H				0x39
#define	I2C_ADDR_CH3_PER_L				0x3A
#define	I2C_ADDR_CR_DUTY_H				0x3B
#define	I2C_ADDR_CR_DUTY_L				0x3C
#define	I2C_ADDR_CS1_DUTY_H				0x3D
#define	I2C_ADDR_CS1_DUTY_L				0x3E
#define	I2C_ADDR_CS2_DUTY_H				0x3F
#define	I2C_ADDR_CS2_DUTY_L				0x40
#define	I2C_ADDR_CS3_DUTY_H				0x41
#define	I2C_ADDR_CS3_DUTY_L				0x42

#define	I2C_ADDR_REG_VER				0x29
#define	I2C_ADDR_USE_CH_INF				0x2A

#define I2C_ADDR_PGM_VER_MAIN			0x71
#define I2C_ADDR_PGM_VER_SUB			0x72

//Calibration Data Backup/Restore
#define I2C_ADDR_CMD_OPT 					0x7E
#define I2C_ADDR_COMMAND 					0x7F
#define I2C_ADDR_REQ_DATA					0x80
#define CMD_R_CD_DUTY						0x04		//Cal Data Duty Read
#define CMD_R_CD_REF						0x05		//Cal Data Ref Read
#define CMD_W_CD_DUTY						0x84		//Cal Data Duty Read
#define CMD_W_CD_REF						0x85		//Cal Data Ref Read

#ifdef CONFIG_LGE_SAR_CONTROLLER_USB_DETECT
#define USB_CONNECTION 0x01
#endif

#endif
