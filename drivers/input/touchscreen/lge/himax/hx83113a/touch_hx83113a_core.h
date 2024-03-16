#include "touch_hx83113a.h"
#include <linux/slab.h>

#define DATA_LEN_8     8
#define DATA_LEN_4     4
#define ADDR_LEN_4     4
#define FLASH_RW_MAX_LEN      256
#define FLASH_WRITE_BURST_SZ  8
#define PROGRAM_SZ            48
#define MAX_I2C_TRANS_SZ      128
#define HIMAX_REG_RETRY_TIMES 5
#define FW_BIN_16K_SZ         0x4000
#define HIMAX_TOUCH_DATA_SIZE 128
#define MASK_BIT_0 0x01
#define MASK_BIT_1 0x02
#define MASK_BIT_2 0x04

#define FW_SECTOR_PER_BLOCK			8
#define FW_PAGE_PER_SECTOR 			64
#define FW_PAGE_SEZE 						128
#define HX256B	0x100
#define HX4K	0x1000
#define HX_32K_SZ 0x8000
#define HX_40K_SZ 0xA000
#define HX_48K_SZ 0xC000
#define HX64K	0x10000
#define HX124K	0x1f000
#define HX4000K	0x1000000

#define HX_NORMAL_MODE		1
#define HX_SORTING_MODE		2
#define HX_CHANGE_MODE_FAIL	(-1)
#define HX_RW_REG_FAIL			(-1)

#define IC_ADR_AHB_ADDR_BYTE_0	         0x00
#define IC_ADR_AHB_RDATA_BYTE_0	         0x08
#define IC_ADR_AHB_ACCESS_DIRECTION      0x0C
#define IC_ADR_CONTI	                 0x13
#define IC_ADR_INCR4	                 0x0D
#define IC_ADR_I2C_PSW_LB                0x31
#define IC_ADR_I2C_PSW_UB                0x32
#define IC_CMD_AHB_ACCESS_DIRECTION_READ 0x00
#define IC_CMD_CONTI	                 0x31
#define IC_CMD_INCR4	                 0x10
#define IC_CMD_I2C_PSW_LB                0x27
#define IC_CMD_I2C_PSW_UB                0x95
#define IC_ADR_TCON_ON_RST      0x80020020
#define IC_ADDR_ADC_ON_RST      0x80020094
#define IC_ADR_PSL              0x900000A0
#define IC_ADR_CS_CENTRAL_STATE 0x900000A8
#define IC_CMD_RST              0x00000000

//#define FW_ADDR_SYSTEM_RESET                 0x90000018
#define FW_ADDR_SAFE_MODE_RELEASE_PW         0x90000098
#define FW_ADDR_CTRL_FW         			 0x9000005C
#define FW_ADDR_FLAG_RESET_EVENT             0x900000E4
//#define FW_ADDR_HSEN_ENABLE                  0x10007F14
#define FW_ADDR_SMWP_ENABLE                  0x10007F10
#define FW_USB_DETECT_ADDR					 0x10007F38
#define FW_ADDR_PROGRAM_RELOAD_FROM          0x00000000
//#define FW_ADDR_PROGRAM_RELOAD_TO            0x08000000
//#define FW_ADDR_PROGRAM_RELOAD_PAGE_WRITE    0x0000FB00
#define FW_ADDR_RAW_OUT_SEL                  0x800204B4
#define FW_ADDR_RELOAD_STATUS                0x80050000
#define FW_ADDR_RELOAD_CRC32_RESULT          0x80050018
#define FW_ADDR_RELOAD_ADDR_FROM             0x80050020
#define FW_ADDR_RELOAD_ADDR_CMD_BEAT         0x80050028
//#define FW_DATA_SYSTEM_RESET                 0x00000055
#define FW_DATA_SAFE_MODE_RELEASE_PW_ACTIVE  0x00000053
#define FW_DATA_SAFE_MODE_RELEASE_PW_RESET   0x00000000
#define FW_DATA_CLEAR					 	 0x00000000
#define FW_DATA_FW_STOP					 	 0x000000A5
//#define FW_DATA_PROGRAM_RELOAD_START         0x0A3C3000
//#define FW_DATA_PROGRAM_RELOAD_COMPARE       0x04663000
//#define FW_DATA_PROGRAM_RELOAD_BREAK         0x15E75678
//#define FW_ADDR_SELFTEST_ADDR_EN             0x10007F18
//#define FW_ADDR_SELFTEST_RESULT_ADDR	     0x10007F24
//#define FW_DATA_SELFTEST_REQUEST	         0x00006AA6
//#define FW_ADDR_CRITERIA_ADDR	             0x10007F1C
//#define FW_DATA_CRITERIA_AA_TOP	             0x64
//#define FW_DATA_CRITERIA_AA_BOT	             0x00
//#define FW_DATA_CRITERIA_KEY_TOP	         0x64
//#define FW_DATA_CRITERIA_KEY_BOT	         0x00
//#define FW_DATA_CRITERIA_AVG_TOP	         0x64
//#define FW_DATA_CRITERIA_AVG_BOT	         0x00
#define FW_ADDR_SET_FRAME_ADDR	             0x10007294
#define FW_DATA_SET_FRAME	             0x0000000A
#define FW_ADDR_NEG_NOISE_SUP	             0x10007FD8
#define FW_DATA_NEG_NOISE	             0x7F0C0000
#define FW_ADDR_FW_NEW_METHOLD	             0x10007FD8
#define FW_ADDR_CC_CUT_NEW	             0x100071C4
#define FW_ADDR_CC_CUT_OLD	             0x10007088

//#define FW_DATA_SELFTEST_ACK_HB              0xA6
//#define FW_DATA_SELFTEST_ACK_LB              0x6A
//#define FW_DATA_SELFTEST_PASS                0xAA
#define FW_DATA_NORMAL_CMD                   0x00
#define FW_DATA_NORMAL_STATUS                0x99
#define FW_DATA_SORTING_CMD                  0xAA
#define FW_DATA_SORTING_STATUS               0xCC
#define FW_DATA_IDLE_DIS_PWD                 0x17
#define FW_DATA_IDLE_EN_PWD	                 0x1F
#define FW_ADDR_SORTING_MODE_EN	             0x10007F04
#define FW_ADDR_FW_MODE_STATUS	             0x10007088
#define FW_ADDR_ICID_ADDR	                 0x900000D0
#define FW_ADDR_TRIGGER_ADDR		         0x10007088
#define FW_ADDR_FW_VER_ADDR		             0x10007004
#define FW_ADDR_FW_CFG_ADDR		             0x10007084
#define FW_ADDR_FW_VENDOR_ADDR	             0x10007000
//#define FW_ADDR_FW_STATE_ADDR                0x900000F8
#define FW_ADDR_FW_DBG_MSG_ADDR              0x10007F44
#define FW_ADDR_CHK_FW_STATUS	             0x900000A8
#define FW_ADDR_CHK_FW_STATUS_2	             0x900000E4
#define FW_ADDR_CHK_FW_STATUS_3	             0x10007F40
#define FW_ADDR_DD_HANDSHAK_ADDR	         0x900000FC
#define FW_ADDR_DD_DATA_ADDR	             0x10007F80
#define FW_DATA_DD_REQUEST                   0xAA
#define FW_DATA_DD_ACK                       0xBB
#define FW_DATA_RAWDATA_READY_HB             0xA3
#define FW_DATA_RAWDATA_READY_LB             0x3A
#define FW_ADDR_AHB_ADDR                     0x11
#define FW_DATA_AHB_DIS	                     0x00
#define FW_DATA_AHB_EN                       0x01
#define FW_ADDR_EVENT_ADDR                   0x30
#define FW_FUNC_HANDSHAKING_PWD				 0xA55AA55A
//#define FW_FUNC_HANDSHAKING_END				 0x77887788
#define FW_ADDR_LGE_FW_VER_ADDR		     	 0x10007008
#define FW_ADDR_CONTROL_IDLE_MODE            0x10007FD4

#define SWIPE_INFO_NUM 32
#define FW_ADDR_TCI_MODE	0x10007FE8
#define FW_ADDR_SWIPE_UP 	0x10000600
#define FW_ADDR_SWIPE_DOWN 	0x10000620
#define FW_ADDR_SWIPE_LEFT 	0x10000640
#define FW_ADDR_SWIPE_RIGHT	0x10000660

#define FW_ADDR_TCI_SET		0x1000738C
#define FW_ADDR_TCI_AREA	0x10007390
//#define FW_ADDR_TCI_AREA_X1					 0x10007188
//#define FW_ADDR_TCI_AREA_Y1					 0x1000718A
//#define FW_ADDR_TCI_AREA_X2					 0x1000718C
//#define FW_ADDR_TCI_AREA_Y2					 0x1000718E
//#define FW_ADDR_TCI_MIN_INTERTAP			 0x10007190
//#define FW_ADDR_TCI_MAX_INTERTAP			 0x10007194
//#define FW_ADDR_TCI_INTR_DELAY				 0x10007198
//#define FW_ADDR_TCI_TAP_COUNT				 0x1000719C
//#define FW_ADDR_TCI_TOUCH_SLOP				 0x1000719E
//#define FW_ADDR_TCI_TAP_DISTANCE			 0x100071A0


#define FLASH_ADDR_CTRL_BASE              0x80000000
#define FLASH_ADDR_SPI200_TRANS_FMT      (FLASH_ADDR_CTRL_BASE + 0x10)
#define FLASH_ADDR_SPI200_TRANS_CTRL     (FLASH_ADDR_CTRL_BASE + 0x20)
#define FLASH_ADDR_SPI200_CMD            (FLASH_ADDR_CTRL_BASE + 0x24)
#define FLASH_ADDR_SPI200_ADDR           (FLASH_ADDR_CTRL_BASE + 0x28)
#define FLASH_ADDR_SPI200_DATA           (FLASH_ADDR_CTRL_BASE + 0x2C)
//#define FLASH_ADDR_SPI200_BT_NUM         (FLASH_ADDR_CTRL_BASE + 0xE8)
#define FLASH_DATA_SPI200_TRANS_FMT       0x00020780
#define FLASH_DATA_SPI200_TRANS_CTRL_1    0x42000003
#define FLASH_DATA_SPI200_TRANS_CTRL_2    0x47000000
#define FLASH_DATA_SPI200_TRANS_CTRL_3    0x67000000
#define FLASH_DATA_SPI200_TRANS_CTRL_4    0x610FF000
//#define FLASH_DATA_SPI200_TRANS_CTRL_5    0x694002FF
#define FLASH_DATA_SPI200_CMD_1           0x00000005
#define FLASH_DATA_SPI200_CMD_2           0x00000006
//#define FLASH_DATA_SPI200_CMD_3           0x000000C7
#define FLASH_DATA_SPI200_CMD_4           0x000000D8
//#define FLASH_DATA_SPI200_CMD_5           0x00000020
#define FLASH_DATA_SPI200_CMD_6           0x00000002
//#define FLASH_DATA_SPI200_CMD_7           0x0000003B
//#define FLASH_DATA_SPI200_ADDR            0x00000000//

#define SRAM_ADR_MKEY	       0x100070E8
#define SRAM_ADR_RAWDATA_ADDR  0x10000000
#define SRAM_ADR_RAWDATA_END   0x00000000
#define SRAM_CMD_CONTI	       0x44332211
#define SRAM_CMD_FIN	       0x00000000
//#definE	SRAM_PASSWRD_START	0x5AA5
//#definE	SRAM_PASSWRD_END	0xA55A

#define DRIVER_ADDR_FW_WIDTH_ANGLE              0x10000578
#define DRIVER_ADDR_FW_DEFINE_FLASH_RELOAD              0x10007F00
#define DRIVER_ADDR_FW_DEFINE_2ND_FLASH_RELOAD          0x100072C0
#define DRIVER_DATA_FW_DEFINE_FLASH_RELOAD_DIS	        0x0000A55A
#define DRIVER_DATA_FW_DEFINE_FLASH_RELOAD_EN	        0x00000000
#define DRIVER_ADDR_FW_DEFINE_INT_IS_EDGE               0x10007088
#define DRIVER_ADDR_FW_DEFINE_RXNUM_TXNUM_MAXPT         0x100070F4
#define DRIVER_DATA_FW_DEFINE_RXNUM_TXNUM_MAXPT_SORTING 0x00000008
#define DRIVER_DATA_FW_DEFINE_RXNUM_TXNUM_MAXPT_NORMAL  0x00000014
#define DRIVER_ADDR_FW_DEFINE_XY_RES_ENABLE             0x100070F8
#define DRIVER_ADDR_FW_DEFINE_X_Y_RES                   0x100070FC

#define fw_addr_ulpm_33 0x33
#define fw_addr_ulpm_34 0x34
#define fw_data_ulpm_11 0x11
#define fw_data_ulpm_22 0x22
#define fw_data_ulpm_33 0x33
#define fw_data_ulpm_aa 0xAA

