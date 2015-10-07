///*****************************************
//  Copyright (C) 2009-2014
//  ITE Tech. Inc. All Rights Reserved
//  Proprietary and Confidential
///*****************************************
//   @file   <IT6811.h>
//   @author Hermes.Wu@ite.com.tw
//   @date   2013/05/07
//   @fileversion: ITE_IT6811_6607_SAMPLE_1.06
//******************************************/
#ifndef _IT6681_ARCH_H_
#define _IT6681_ARCH_H_

//===============================================
// Detect Build environment
//===============================================

#if defined(__ANDROID__)
    #define ENV_ANDROID
    
#elif defined(__linux__)
    #define ENV_LINUX

#elif defined(__BORLANDC__)
    #define ENV_WINDOWS_BCB
    #pragma message("BCB")

#elif defined(__C51__)
    #define ENV_8051_KEIL_C

#else
    #define ENV_ANDROID

#endif
//===============================================


#if defined(ENV_ANDROID)

	#include <linux/module.h>
	#include <linux/delay.h>
	#include <linux/slab.h>
	#include <video/omapdss.h>
	#include <linux/i2c.h>
	#include <linux/printk.h>
	#include <linux/workqueue.h>
	#include <linux/kthread.h>
	#include <linux/input.h>
	#include <asm/atomic.h>
    #include <linux/it6681.h>

    #define _ITE_668X_DEMO_BOARD 0

    #define debug_6681	pr_err

    typedef char BOOL;
    typedef unsigned char BYTE, *PBYTE ;
    typedef short SHORT, *PSHORT ;
    typedef unsigned short USHORT, *PUSHORT ;
    typedef unsigned long ULONG, *PULONG ;

#elif defined(ENV_WINDOWS_BCB)

    #include <windows.h>
    #include <stdio.h>
    #include "it6681.h"

    #define _ITE_668X_DEMO_BOARD 0

    #define debug_6681	__myprintf
    #ifdef __cplusplus
    extern "C" {
    #endif
    void __myprintf( char *fmt, ... );
    #ifdef __cplusplus
    }
    #endif

#elif defined(ENV_8051_KEIL_C)

    #pragma message("ENV_8051_KEIL_C")

    #include <stdio.h>
    #include <string.h>
    //#include "mcu.h"
    //#include "io.h"
    //#include "Utility.h"
    //#include "It6681.h"
    #define const code
    #define mdelay(ms)  delay1ms(ms)
    #define mutex_unlock(x)
    #define mutex_lock(x)
    //#define IT6681_MHL_ADDR   0xC8

    #define _ITE_668X_DEMO_BOARD 1
    void it6681_copy_edid_ite_demo_board(void);

    #define debug_6681 printf
    //#define debug_6681 

    typedef char BOOL;
    typedef char CHAR, *PCHAR ;
    typedef unsigned char uchar, *puchar ;
    typedef unsigned char UCHAR, *PUCHAR ;
    typedef unsigned char byte, *pbyte ;
    typedef unsigned char BYTE, *PBYTE ;
    
    typedef short SHORT, *PSHORT ;
    
    typedef unsigned short USHORT, *PUSHORT ;
    typedef unsigned short word, *pword ;
    typedef unsigned short WORD, *PWORD ;
    
    typedef long LONG, *PLONG ;
    
    typedef unsigned long ULONG, *PULONG ;
    typedef unsigned long dword, *pdword ;
    typedef unsigned long DWORD, *PDWORD ;
    #define IT6681_EDID_MAX_BLOCKS 2

    #include "it6681.h"

#else

    #pragma error("No build environment was defined !")

#endif


#endif
