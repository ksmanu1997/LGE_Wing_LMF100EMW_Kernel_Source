#ifndef __ATMF04_EFLASH_H__
#define __ATMF04_EFLASH_H__


//============================================================//
//[190306] ADS Add
//[START]=====================================================//
#define USE_ALMF04				//ALMF04 Use status
//[END]======================================================//

#define SZ_PAGE_DATA                64

//============================================================//
//[190306] ADS Change
//[START]=====================================================//
//#ifdef CONFIG_LGE_USE_SAR_CONTROLLER
//#define FW_DATA_PAGE               	115
//#else
//#define FW_DATA_PAGE               	96
//#endif
//
//#define ADDR_EFLA_STS               0xFF	//eflash status register
//#define ADDR_EFLA_PAGE_L            0xFD	//eflash page
//#define ADDR_EFLA_PAGE_H            0xFE	//eflash page
//#define ADDR_EFLA_CTRL              0xFC	//eflash control register
//
//#define CMD_EFL_L_WR                0x01	//Eflash Write
//#define CMD_EFL_RD                  0x03	//Eflash Read
//#define CMD_EFL_ERASE_ALL           0x07	//Eflash All Page Erase
//
//#define CMD_EUM_WR                  0x21	//Extra user memory write
//#define CMD_EUM_RD                  0x23	//Extra user memory read
//#define CMD_EUM_ERASE               0x25	//Extra user memory erase
//
//#define FLAG_DONE                   0x03
//#define FLAG_DONE_ERASE             0x02

#define ADDR_EFLA_STS               0xFF	//eflash status register
#define ADDR_EFLA_PAGE_L            0xFD	//eflash page
#define ADDR_EFLA_PAGE_H            0xFE	//eflash page
#define ADDR_EFLA_CTRL              0xFC	//eflash control register

#if defined(USE_ALMF04)

	#define ADDR_ROM_SAFE				0xFB	//eflash write 보호 루틴
	#define VAL_ROM_MASK1				0x02
	#define VAL_ROM_MASK2				0x06

	#define EMD_ALL_ERASE				(0x07 << 1)
	#define EMD_PG_ERASE				(0x04 << 1)
	#define EMD_PG_WRITE				(0x08 << 1)
	#define EMD_PG_READ					0x00

	#define CMD_EEP_START				0x01
	#define CMD_EUM_START				0x03

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

#endif		//defined(USE_ALMF04)
//[END]======================================================//

#define FLAG_FUSE                   1
#define FLAG_FW                     2

//============================================================//
//[20180327] ADS Change
//[START]=====================================================//
//#define FL_EFLA_TIMEOUT_CNT         200
#define FL_EFLA_TIMEOUT_CNT         20
//[END]======================================================//
#define IC_TIMEOUT_CNT        5

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
#define CAP_CAL_RESULT_PASS                     "pass"
#define CAP_CAL_RESULT_FAIL                     "fail"
#endif

#endif

#define CONFIG_LGE_ATMF04_2CH

#if defined(CONFIG_LGE_ATMF04_2CH)

#define CNT_INITCODE               31

// Each operator use different initcode value

static const unsigned char InitCodeAddr[CNT_INITCODE]   = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x09, 0x0A, 0x0B, 0X0C, 0X0D, 0x0E, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D};
static const unsigned char InitCodeVal[CNT_INITCODE]    = { 0x00, 0x7B, 0x00, 0x31, 0x33, 0x0B, 0x0B, 0x64, 0x64, 0x81, 0x6A, 0x4F, 0x27, 0x00, 0x52, 0x00, 0x19, 0xD0, 0xA4, 0x14, 0x09, 0x12, 0x07, 0x33, 0x04, 0x2F};
#endif

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
#define MODULE_TAG "<atmf04_12qfn>"
#endif

#if (LOG_LEVEL >= LOG_LEVEL_E)
/*! print error message */
#define PERR(fmt, args...) \
	pr_err(MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
/*! invalid message */
#define PERR(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_N)
/*! print notice message */
#define PNOTICE(fmt, args...) \
	pr_notice(MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
/*! invalid message */
#define PNOTICE(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_I)
/*! print information message */
#define PINFO(fmt, args...) pr_info(MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
/*! invalid message */
#define PINFO(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_D)
/*! print debug message */
#define PDEBUG(fmt, args...) pr_devel(MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
/*! invalid message */
#define PDEBUG(fmt, args...)
#endif
