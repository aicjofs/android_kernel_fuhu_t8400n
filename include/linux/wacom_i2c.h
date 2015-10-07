/***********************************************************************************/
/*  Copyright (c) 2010, Silicon Image, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: Silicon Image, Inc.,            */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                           */
/***********************************************************************************/
#ifndef _TOUCHSCREEN_WACOM_I2C__
#define __TOUCHSCREEN_WACOM_I2C__
#include <linux/types.h>
#include <linux/kernel.h>
//--------------------------------------wacom i2c flash----------------------------------------

#define FLASH_START0 	      'f'
#define FLASH_START1 	      'l'
#define FLASH_START2          'a'
#define FLASH_START3 	      's'
#define FLASH_START4 	      'h'
#define FLASH_START5 	     '\r'

#define FLASH_BLOCK_SIZE       64

#define ACK                     0

#define CMD_GET_FEATURE	        2
#define CMD_SET_FEATURE	        3

#define BOOT_CMD             0x04
#define MPU_CMD              0x05
#define WRITE_CMD            0x01
#define QUERY_CMD            0x07
#define ERS_CMD              0x00
#define VERIFY_CMD           0x02
#define SEC_CMD              0x06
#define MARK_CMD             0x02

#define BOOT_CMD_SIZE          78
#define BOOT_RSP_SIZE	        6
#define CMD_SIZE           (72+6)
#define RSP_SIZE                6

#define BOOT_CMD_REPORT_ID	7
#define BOOT_BLVER		4
#define BOOT_MPU		5
#define BOOT_QUERY		7
#define BOOT_ERASE_FLASH	0
#define BOOT_WRITE_FLASH	1
#define BOOT_EXIT		3
#define BOOT_SECURITY_UNLOCK	6
#define BOOT_VERIFY_FLASH	2

#define QUERY_RSP            0x06



#define ERR                     1
#define ERR_WRITE               3
#define ERR_SET_IRQ 0x0e
#define ERR_GET_IRQ 0x0d

#define BLOCK_NUM 62
#define MPU_W9007 0x2A
#define MPU_W9002 0x15
#if (defined(CONFIG_MACH_T8400N_8_3CM) || defined(CONFIG_MACH_KEENHI01N) || defined(CONFIG_MACH_T8400N))
//w9002
#define USER_START_ADDR 0x4000
#define USER_MAX_ADDR 0x12fff
#else
//w9007
#define USER_START_ADDR 0x2000
#define USER_MAX_ADDR 0xfbff
#endif

#define PROGRAM_MAX_ADDR 0x11FC0
#define CHECK_PROC_MAX_ADDR 0x11fbf

#define MARKER 56

#define SET_IRQ_DETECT 0x0a

#define DATA_SIZE (65536 * 2)

#define HEX_READ_ERR -1
#define ASCII_EOF 0x1A

#define WACOM_CMD_QUERY0       0x04
#define WACOM_CMD_QUERY1       0x00
#define WACOM_CMD_QUERY2       0x33
#define WACOM_CMD_QUERY3       0x02
#define WACOM_CMD_THROW0       0x05
#define WACOM_CMD_THROW1       0x00
#define WACOM_QUERY_SIZE       19

//------------------------------------------end----------------------------------------------

struct wacom_platform_data {
	int display_xres;
	int display_yres;
	int discover_pencil_irq;
	int plug_detect_gpio;
	int display_enable_gpio;
	int debounce_interval;
	int active_state;
	int (*powerInit) (void);
	int (*powerDisable) (void);
	int	pdct_pin;
	int	prst_pin;
};


#endif  // __MHL_SII8334_H__

