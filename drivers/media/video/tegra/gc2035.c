/*
 * kernel/drivers/media/video/tegra
 *
 * Aptina MT9D115 sensor driver
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/gc2035.h>
#include <linux/hrtimer.h>
#include <linux/module.h>

#define DRIVER_VERSION "0.0.1"
#define DEBUG 1

/** Macro for determining the size of an array */
#define KH_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

struct sensor_reg {
	u8 addr;
	u8 val;
};

struct sensor_info {
	int isRecoded;
	int mode;
	int current_wb;
	int current_exposure;
	u16 current_addr;
	atomic_t power_on_loaded;
	atomic_t sensor_init_loaded;
	struct i2c_client *i2c_client;
	struct yuv_sensor_platform_data *pdata;
};

static struct sensor_info *gc2035_sensor_info;

static struct sensor_reg mode_init[] = {
	{0xfe,0x80},  
	{0xfe,0x80},  
	{0xfe,0x80},  
	{0xfc,0x06},  
	{0xf9,0xfe},  
	{0xfa,0x00},  
	{0xf6,0x00},  
		
	{0xf7,0x05},   // don't change
	{0xf8,0x87},  // don't change
		
	{0xfe,0x00},  
	{0x82,0x00},  
	{0xb3,0x60},  
	{0xb4,0x40},  
	{0xb5,0x60},  
	{0x03,0x02},  
	{0x04,0xda},  
		
	{0xfe,0x00},  
	{0xec,0x06},//04 
	{0xed,0x06},//04 
	{0xee,0x62},//60 
	{0xef,0x92},//90 
		
	{0x0a,0x00},  
	{0x0c,0x00},  
	{0x0d,0x04},  
	{0x0e,0xc0},  
	{0x0f,0x06},  
	{0x10,0x58},  
	{0x17,0x14},
	{0x18,0x0e}, //0a 2012.10.26
	{0x19,0x0c},  
	{0x1a,0x01},  
	{0x1b,0x8b},
	{0x1c,0x05}, // add by lanking 20130403
	{0x1e,0x88},  
	{0x1f,0x08}, //[3] tx-low en//
	{0x20,0x05},  
	{0x21,0x0f},  
	{0x22,0xf0},//// d0 20130628 
	{0x23,0xc3},  
	{0x24,0x17}, //pad drive  16
		                 //AEC
	{0xfe,0x01},  
	{0x11,0x20},//AEC_out_slope , 0x
	{0x1f,0xc0},//max_post_gain
	{0x20,0x60},//max_pre_gain
	{0x47,0x30},//AEC_outdoor_th
	{0x0b,0x10},//
	{0x13,0x75},//y_target
		
	{0xfe,0x00},  
	{0x05,0x01},//
	{0x06,0x35},  
	{0x07,0x00},//
	{0x08,0xe2},  
	{0xfe,0x01},  
	{0x27,0x00},//
	{0x28,0xce},
	
	{0x29,0x05},//
	{0x2a,0xa2},  
	{0x2b,0x06},//
	{0x2c,0x70},  
	{0x2d,0x08},//
	{0x2e,0x0c},  
	{0x2f,0x09},//
	{0x30,0xa8},  
	{0x3e,0x40},//
	{0xfe,0x00}, 
		
	{0xb6,0x03},  
	{0xfe,0x00},  
	{0x3f,0x00},  
	{0x40,0x77},  
	{0x42,0x7f},  
	{0x43,0x30},  
	{0x5c,0x08},  
	{0x5e,0x20},  
	{0x5f,0x20},  
	{0x60,0x20},  
	{0x61,0x20},  
	{0x62,0x20},  
	{0x63,0x20},  
	{0x64,0x20},  
	{0x65,0x20},  

		///block////////////
	{0x80,0xff},  
	{0x81,0x26},  
	{0x87,0x90}, //[7]middle gamma 
	{0x84,0x00}, //output put foramat
	{0x86,0x02}, //02 //sync plority 
	{0x8b,0xbc},
	{0xb0,0x80}, //globle gain
	{0xc0,0x40},//Yuv bypass
	{0xfe,0x01},  
	{0xc2,0x2a},  
	{0xc3,0x1a},  
	{0xc4,0x16},  
	{0xc8,0x21},  
	{0xc9,0x1c},  
	{0xca,0x18},  
	{0xbc,0x46},  
	{0xbd,0x2a}, 
	{0xbe,0x26},  
	{0xb6,0x35},  
	{0xb7,0x24},  
	{0xb8,0x1b},  
	{0xc5,0x00},  
	{0xc6,0x00},  
	{0xc7,0x00},  
	{0xcb,0x00},  
	{0xcc,0x00},  
	{0xcd,0x00},  
	{0xbf,0x0c},  
	{0xc0,0x12},  
	{0xc1,0x17},  
	{0xb9,0x00},  
	{0xba,0x00},  
	{0xbb,0x07},  
	{0xaa,0x1b},  
	{0xab,0x20},  
	{0xac,0x20},  
	{0xad,0x24},  
	{0xae,0x1f},  
	{0xaf,0x23},  
	{0xb0,0x20},  
	{0xb1,0x20},  
	{0xb2,0x20},  
	{0xb3,0x16},  
	{0xb4,0x1c},  
	{0xb5,0x16},  
	{0xd0,0x00},  
	{0xd2,0x00},  
	{0xd3,0x00},  
	{0xd8,0x00},  
	{0xda,0x00},  
	{0xdb,0x00},  
	{0xdc,0x00},  
	{0xde,0x00},  
	{0xdf,0x00},  
	{0xd4,0x00},  
	{0xd6,0x00},  
	{0xd7,0x0c},  
	{0xa4,0x00},  
	{0xa5,0x00},  
	{0xa6,0x00},  
	{0xa7,0x00},  
	{0xa8,0x00},  
	{0xa9,0x00},  
	{0xa1,0x80},  
	{0xa2,0x80},  
	{0xfe,0x02},  
	{0xa4,0x00},  
	{0xfe,0x00},  
	{0xfe,0x02},  
	{0xc0,0x01},  
	{0xc1,0x40},  
	{0xc2,0xfc},  
	{0xc3,0x05},  
	{0xc4,0xec},  
	{0xc5,0x42},  
	{0xc6,0xf8},  
	{0xc7,0x40},  
	{0xc8,0xf8},  
	{0xc9,0x06},  
	{0xca,0xfd},  
	{0xcb,0x3e},  
	{0xcc,0xf3},  
	{0xcd,0x36},  
	{0xce,0xf6},  
	{0xcf,0x04},  
	{0xe3,0x0c},  
	{0xe4,0x44},  
	{0xe5,0xe5}, 
	{0xfe,0x00},  
	{0xfe,0x01},
	{0x4f,0x00},
	{0x4d,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0x10}, // 10
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0x20}, // 20
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0x30},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00}, // 30
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0x40}, // 40
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0x50}, // 50
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0x60}, // 60
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0x70}, // 70
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0x80}, // 80
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0x90}, // 90
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0xa0}, // a0
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0xb0}, // b0
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0xc0}, // c0
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4d,0xd0}, // d0
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4e,0x00},
	{0x4f,0x01},
		///// awb value////////
	{0xfe,0x01},
	{0x4f,0x00},
	{0x4d,0x30},
	{0x4e,0x00},
	{0x4e,0x80},
	{0x4e,0x80},
	{0x4e,0x02},
	{0x4e,0x02},
	{0x4d,0x40},
	{0x4e,0x00},
	{0x4e,0x80},
	{0x4e,0x80},
	{0x4e,0x02},
	{0x4e,0x02},
	{0x4e,0x02},
	{0x4d,0x53},
	{0x4e,0x08},
	{0x4e,0x04},
	{0x4d,0x62},
	{0x4e,0x10},
	{0x4d,0x72},
	{0x4e,0x20},
	{0x4f,0x01},

		////wb////
	{0xfe,0x01},
	{0x50,0x88},//c0//[6]green mode
	{0x52,0x40},
	{0x54,0x60},
	{0x56,0x06},
	{0x57,0x20}, //pre adjust
	{0x58,0x01}, 
	{0x5b,0x02}, //AWB_gain_delta
	{0x61,0xaa},//R/G stand
	{0x62,0xaa},//R/G stand
	{0x71,0x00},
	{0x74,0x10},  //0x//AWB_C_max
	{0x77,0x08}, // 0x//AWB_p2_x
	{0x78,0xfd}, //AWB_p2_y
	{0x86,0x30},
	{0x87,0x00},
	{0x88,0x04},//06 , 0x//[1]dark mode
	{0x8a,0xc0},//awb move mode
	{0x89,0x75},
	{0x84,0x08},  //0x//auto_window
	{0x8b,0x00}, // 0x//awb compare luma
	{0x8d,0x70}, //awb gain limit R 
	{0x8e,0x70},//G
	{0x8f,0xf4},//B
	{0xfe,0x00},
	{0x82,0x02},//awb_en
	{0xfe,0x01},  
	{0x21,0xbf},  
	{0xfe,0x02},  
	{0xa4,0x00},//
	{0xa5,0x40}, //lsc_th
	{0xa2,0xa0}, //lsc_dec_slope
	{0xa6,0x80}, //dd_th
	{0xa7,0x80}, //ot_th
	{0xab,0x31}, //
	{0xa9,0x6f}, //
	{0xb0,0x99}, //0x//edge effect slope low
	{0xb1,0x34},//edge effect slope low
	{0xb3,0x80}, //saturation dec slope
	{0xde,0xb6},  //
	{0x38,0x0f}, // 
	{0x39,0x60}, //
	{0xfe,0x00},  
	{0x81,0x26},  
	{0xfe,0x02},  
	{0x83,0x00},  
	{0x84,0x45},  
	{0xd1,0x38},  
	{0xd2,0x38},  
	{0xd3,0x40},//contrast ?			
	{0xd4,0x80},//contrast center 
	{0xd5,0x00},//luma_offset 
	{0xdc,0x30},
	{0xdd,0xb8},//edge_sa_g,b
	{0xfe,0x00},
		/////dndd///////////
	{0xfe,0x02},
	{0x88,0x15},//dn_b_base
	{0x8c,0xf6}, //[2]b_in_dark_inc
	{0x89,0x03}, //dn_c_weight
		//////EE ///////////
	{0xfe,0x02},
	{0x90,0x6c},// EEINTP mode1
	{0x97,0x45},// edge effect
		///=============RGB Gamma 
	{0xfe,0x02},
	{0x15,0x0a},
	{0x16,0x12},
	{0x17,0x19},
	{0x18,0x1f},
	{0x19,0x2c},
	{0x1a,0x38},
	{0x1b,0x42},
	{0x1c,0x4e},
	{0x1d,0x63},
	{0x1e,0x76},
	{0x1f,0x87},
	{0x20,0x96},
	{0x21,0xa2},
	{0x22,0xb8},
	{0x23,0xca},
	{0x24,0xd8},
	{0x25,0xe3},
	{0x26,0xf0},
	{0x27,0xf8},
	{0x28,0xfd},
	{0x29,0xff},
	{0xfe,0x02},  
	{0x2b,0x00},  
	{0x2c,0x04},  
	{0x2d,0x09},  
	{0x2e,0x18},  
	{0x2f,0x27},  
	{0x30,0x37},  
	{0x31,0x49},  
	{0x32,0x5c},  
	{0x33,0x7e},  
	{0x34,0xa0},  
	{0x35,0xc0},  
	{0x36,0xe0},  
	{0x37,0xff},  
	{0xfe,0x00},  
	{0x82,0xfe},
		
	       /////////mipi setting////////
	{0xf2,0x00},
	{0xf3,0x00},
	{0xf4,0x00},
	{0xf5,0x00},
	{0xfe,0x01},
	{0x0b,0x90},
	{0x87,0x10},
	{0xfe,0x00},

	{0xfe,0x03},
	{0x01,0x03},
	{0x02,0x11},
	{0x03,0x11},
	{0x06,0x80},
	{0x11,0x1E},
	{0x12,0x80},
	{0x13,0x0c},
	{0x15,0x12},
	{0x04,0x20},
	{0x05,0x00},
	{0x17,0x00},

	{0x21,0x02},
	{0x29,0x02},
	{0x2a,0x03},
	{0x2b,0x08},

	{0x10,0x94},
	{0xfe,0x00},


/////subsample for vga////
	{0xC8,0x00},// 1/2 subsample
	{0x99,0x55},// 1/2 subsample
	{0x9a,0x06},
	{0x9b,0x02},
	{0x9c,0x03},
	{0x9d,0x04},
	{0x9e,0x00},
	{0x9f,0x02},
	{0xa0,0x00},  
	{0xa1,0x00},
	{0xa2,0x00},
	{0x90,0x01},
	{0x95,0x01},
	{0x96,0xe0},
	{0x97,0x02},
	{0x98,0x80},
	{0xfe,0x03},
	{0x12,0x00},
	{0x13,0x05},
	{0x04,0x90},
	{0x05,0x00},
	{0xfe,0x00},

//{SENSOR_WAIT_MS,0x80},
	{SENSOR_TABLE_END, 0x00}
};

static struct sensor_reg mode_640x480[] = {

	{0xfe,0x00},	
	{0xb6,0x03},	
	{0xfa,0x00},
	{0xc8,0x00},

	{0x99,0x55},// 1/2 subsample
	{0x9a,0x06},
	{0x9b,0x02},
	{0x9c,0x03},
	{0x9d,0x04},
	{0x9e,0x00},
	{0x9f,0x02},
	{0xa0,0x00},  
	{0xa1,0x00},
	{0xa2,0x00},

	{0x90,0x01},
	{0x95,0x01},
	{0x96,0xe0},
	{0x97,0x02},
	{0x98,0x80},

	{0xfe,0x03},
	{0x12,0x00},
	{0x13,0x05},
	{0x04,0x90},
	{0x05,0x00},
	{0xfe,0x00},

	{SENSOR_TABLE_END, 0x00}
};
static struct sensor_reg mode_800x600[] = {
	{0xfe,0x00},	
	{0xb6,0x03},	
	{0xfa,0x00},
	{0xc8,0x00},

	
	{0x99, 0x22},
	{0x9a, 0x06},
	{0x9b, 0x00},  
	{0x9c, 0x00},
	{0x9d, 0x00},
	{0x9e, 0x00},
	{0x9f, 0x00},  
	{0xa0, 0x00},
	{0xa1, 0x00},
	{0xa2, 0x00},
		
	{0x90, 0x01},
	{0x95, 0x02},
	{0x96, 0x58},
	{0x97, 0x03},
	{0x98, 0x20},
	
	{0xfe, 0x03},
	{0x12, 0x40},
	{0x13, 0x06},
	{0x04, 0x90},
	{0x05, 0x01},
	{0xfe, 0x00},
	{SENSOR_TABLE_END, 0x00}
};
static struct sensor_reg mode_1280x720[] = {
	{0xfe,0x00},
	{0xfa,0x11},
	{0xc8,0x00},
	
	{0x99,0x55},// 4/5 subsample
	{0x9a,0x06},
	{0x9b,0x00},
	{0x9c,0x00},
	{0x9d,0x01},
	{0x9e,0x23},
	{0x9f,0x00},
	{0xa0,0x00},
	{0xa1,0x01},
	{0xa2,0x23},
	
	{0x90,0x01},
	{0x95,0x02},
	{0x96,0xd0},  
	{0x97,0x05},
	{0x98,0x00},
	{0xfe,0x03},
	{0x12,0x00},
	{0x13,0x0a},
	{0x04,0x90},
	{0x05,0x01},
	{SENSOR_TABLE_END, 0x00}
};
static struct sensor_reg mode_1600x1200[] = { 
	{0xfe,0x00},	
	{0xfa,0x11},
	{0xc8,0x00},

	{0x99, 0x11},
	{0x9a, 0x06},
	{0x9b, 0x00},  
	{0x9c, 0x00},
	{0x9d, 0x00},
	{0x9e, 0x00},
	{0x9f, 0x00},  
	{0xa0, 0x00},
	{0xa1, 0x00},
	{0xa2, 0x00},
	
	{0x90, 0x01},
	{0x95, 0x04},
	{0x96, 0xb0},
	{0x97, 0x06},
	{0x98, 0x40},
	
	{0xfe, 0x03},
	{0x12, 0x80},
	{0x13, 0x0c},
	{0x04, 0x20},
	{0x05, 0x00},
	{0xfe, 0x00},
	{SENSOR_TABLE_END, 0x00}
};

static  struct sensor_reg sensor_WhiteB_Auto[]=
{
	{0xfe, 0x00},
	{0xb3, 0x61},
	{0xb4, 0x40},
	{0xb5, 0x61},
	{0x82, 0xfe},
	{SENSOR_TABLE_END, 0x00}
};
/* Cloudy Colour Temperature : 6500K - 8000K  */
static  struct sensor_reg sensor_WhiteB_Cloudy[]=
{
	{0xfe, 0x00},
	{0x82, 0xfc},
	{0xb3, 0x58},
	{0xb4, 0x40},
	{0xb5, 0x50},
    {SENSOR_TABLE_END, 0x00}
};
/* ClearDay Colour Temperature : 5000K - 6500K  */
static  struct sensor_reg sensor_WhiteB_ClearDay[]=
{
    //Sunny
	{0xfe, 0x00},
	{0x82, 0xfc},
	{0xb3, 0x78},
	{0xb4, 0x40},
	{0xb5, 0x50},
	{SENSOR_TABLE_END, 0x00}
};
/* Office Colour Temperature : 3500K - 5000K  */
static  struct sensor_reg sensor_WhiteB_TungstenLamp1[]=
{
    //Office
	{0xfe, 0x00},
	{0x82, 0xfc},
	{0xb3, 0x50},
	{0xb4, 0x40},
	{0xb5, 0xa8},   
	{SENSOR_TABLE_END, 0x00}
};



static  struct sensor_reg sensor_Effect_Normal[] =
{
	{SENSOR_TABLE_END,0x00}
};

static  struct sensor_reg sensor_Effect_WandB[] =
{
	{SENSOR_TABLE_END,0x00}
};

static  struct sensor_reg sensor_Effect_Sepia[] =
{
	{SENSOR_TABLE_END,0x00}
};

static  struct sensor_reg sensor_Effect_Negative[] =
{
	{SENSOR_TABLE_END,0x00}
};
static  struct sensor_reg sensor_Effect_Bluish[] =
{
	{SENSOR_TABLE_END,0x00}
};

static  struct sensor_reg sensor_Effect_Green[] =
{
	{SENSOR_TABLE_END,0x00}
};



static  struct sensor_reg sensor_Exposure0[]=
{
	{0xfe, 0x01},
	{0x13, 0x60},
	{0xfe, 0x02},
	{0xd5, 0xe0},
	{SENSOR_TABLE_END, 0x00}
};

static  struct sensor_reg sensor_Exposure1[]=
{
	{0xfe, 0x01},
	{0x13, 0x70},
	{0xfe, 0x02},
	{0xd5, 0xf0},
	{SENSOR_TABLE_END, 0x00}
};

static  struct sensor_reg sensor_Exposure2[]=
{
	{0xfe, 0x01},
	{0x13, 0x75},
	{0xfe, 0x02},
	{0xd5, 0x00},
	{SENSOR_TABLE_END, 0x00}
};

static  struct sensor_reg sensor_Exposure3[]=
{
	{0xfe, 0x01},
	{0x13, 0x88},
	{0xfe, 0x02},
	{0xd5, 0x10},
	{SENSOR_TABLE_END, 0x00}
};

static  struct sensor_reg sensor_Exposure4[]=
{
	{0xfe, 0x01},
	{0x13, 0x90},
	{0xfe, 0x02},
	{0xd5, 0x20},
	{SENSOR_TABLE_END, 0x00}
};


static  struct sensor_reg sensor_SceneAuto[] =
{
	{SENSOR_TABLE_END, 0x00}
};

static  struct sensor_reg sensor_SceneNight[] =
{
	{SENSOR_TABLE_END, 0x00}
};

enum {
	SENSOR_MODE_INIT,
	SENSOR_MODE_640x480,
	SENSOR_MODE_800x600,
	SENSOR_MODE_1280x720,
	SENSOR_MODE_1600x1200,
};

static struct sensor_reg *mode_table[] = {
	[SENSOR_MODE_INIT]   = mode_init,
	[SENSOR_MODE_640x480]   = mode_640x480,
	[SENSOR_MODE_800x600]   = mode_800x600,
	[SENSOR_MODE_1280x720]   = mode_1280x720,
	[SENSOR_MODE_1600x1200]   = mode_1600x1200,
};

static int sensor_write_reg8(struct i2c_client *client, u8 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	u8 data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	//data[0] = (u8) (addr >> 8);
	//data[1] = (u8) (addr & 0xff);
	//data[2] = (u8) (val & 0xff);

	data[0] = addr & 0xFF;
    	data[1] = val;
	
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("gc2035 : i2c transfer failed, retrying %x %x %d\n",
		       addr, val, err);
		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

static int sensor_write_table(struct i2c_client *client,
			      const struct sensor_reg table[])
{
	const struct sensor_reg *next;
	int err;
    
	pr_info("gc2035 %s \n",__func__);
    next = table ;       

	for (next = table; next->addr!= SENSOR_TABLE_END; next++) {
	       err = sensor_write_reg8(client, next->addr,next->val);
		if (err){
			pr_err("%s: write  0x%x failed\n", __func__,
				next->addr);
			return err;
		}
	}
	return 0;
}
static void sensor_set_op_mode(struct sensor_info *info, KhOperationalMode *mode)
{
	//fix 24fps
#if 0
	u16 addr = 0x038F;
   	u8 val = 0x03;
	int err = 0;
	pr_info(" sensor_set_op_mode : pA = %d,vA = %d,\n",mode->PreviewActive,mode->VideoActive);

	info->isRecoded = (mode->PreviewActive&&mode->VideoActive)?1:0;
	if(info->isRecoded&&(info->mode == SENSOR_MODE_800x600)){
		pr_info(" sensor_set_op_mode : it will set fixed rate!!\n");
		err = sensor_write_reg8_addr16(info->i2c_client,addr, val);
		if (err){
			pr_err("%s:=========>wirte 0x%04x fail!err=%d\n",__func__,addr,err);
		}
		
		addr = 0x0390;
	   	val = 0x11;
		err = sensor_write_reg8_addr16(info->i2c_client,addr, val);
		if (err){
			pr_err("%s:=========>wirte 0x%04x fail!err=%d\n",__func__,addr,err);
		}
	}else if(atomic_read(&info->power_on_loaded)&&atomic_read(&info->sensor_init_loaded)&&(info->mode == SENSOR_MODE_800x600)){
		pr_info(" sensor_set_op_mode : it will set auto rate!!\n");
		
		addr = 0x038F;
		val = get_sensor_8len_reg(info->i2c_client,addr);
		
	   	if(val == 0x03){
			err = sensor_write_reg8_addr16(info->i2c_client,addr, 0x09);
			if (err){
				pr_err("%s:=========>wirte 0x%04x fail!err=%d\n",__func__,addr,err);
			}
			pr_info(" sensor_set_op_mode : it will set 0x09!!\n");
	   	}
		
		addr = 0x0390;
		val = get_sensor_8len_reg(info->i2c_client,addr);
		if(val == 0x11){
			err = sensor_write_reg8_addr16(info->i2c_client,addr, 0xd0);
			if (err){
				pr_err("%s:=========>wirte 0x%04x fail!err=%d\n",__func__,addr,err);
			}
			pr_info(" sensor_set_op_mode : it will set 0xd0!!\n");
		}
	}
#endif
}
static int sensor_set_mode(struct sensor_info *info, struct gc2035_mode *mode)
{
	int sensor_table;
	int err;


	pr_info("%s: xres %u yres %u\n",__func__, mode->xres, mode->yres);
	 if (mode->xres == 640 && mode->yres == 480)
		sensor_table = SENSOR_MODE_640x480;
	else if (mode->xres == 800 && mode->yres == 600)
		sensor_table = SENSOR_MODE_800x600;
	else if (mode->xres == 1280 && mode->yres == 720)
		sensor_table = SENSOR_MODE_1280x720;
	else if (mode->xres == 1600 && mode->yres == 1200)
		sensor_table = SENSOR_MODE_1600x1200;
	else{
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

    //check already program the sensor mode, Aptina support Context B fast switching capture mode back to preview mode
    //we don't need to re-program the sensor mode for 640x480 table
	if(atomic_read(&info->power_on_loaded)&&!atomic_read(&info->sensor_init_loaded)){
		err = sensor_write_table(info->i2c_client, mode_table[SENSOR_MODE_INIT]);
	      if (err){
			atomic_set(&info->sensor_init_loaded, 0);
	     		return err;
	      }
		atomic_set(&info->sensor_init_loaded, 1);
	}else
		pr_err("%s:============>commond reg inited!\n",__func__);
	
        err = sensor_write_table(info->i2c_client, mode_table[sensor_table]);
        if (err)
            return err;

	switch(info->current_wb){
		case YUV_Whitebalance_Auto:

			err = sensor_write_table(info->i2c_client, sensor_WhiteB_Auto);
		 break;
               case YUV_Whitebalance_Incandescent:

                 	err = sensor_write_table(info->i2c_client, sensor_WhiteB_TungstenLamp1);
                break;
                case YUV_Whitebalance_Daylight:

                 	err = sensor_write_table(info->i2c_client, sensor_WhiteB_ClearDay);
                     break;
                case YUV_Whitebalance_Fluorescent:

                 	err = sensor_write_table(info->i2c_client, sensor_WhiteB_TungstenLamp1);
                     break;
		  case YUV_Whitebalance_CloudyDaylight:

                 	err = sensor_write_table(info->i2c_client, sensor_WhiteB_Cloudy);
                     break;
                default:
                     break;
		}
	     switch(info->current_exposure)
            {
                case YUV_YUVExposure_Positive2:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure4);
                     break;
                case YUV_YUVExposure_Positive1:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure3);
                     break;
                case YUV_YUVExposure_Number0:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure2);
                     break;
                case YUV_YUVExposure_Negative1:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure1);
                     break;
		  case YUV_YUVExposure_Negative2:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure0);
                     break;
                default:
                     break;
            }
	info->mode = sensor_table;
	return 0;
}
static int gc2035_get_status(struct sensor_info *info,
		struct gc2035_status *dev_status)
{
	int err = 0;
       dev_status->status = 0;

	return err;
}

static long sensor_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct sensor_info *info = file->private_data;
    int err=0;

	pr_info("gc2035 %s\n cmd %d", __func__, cmd);
    
	switch (cmd) 
    {
    	case GC2035_IOCTL_SET_MODE:
    	{
    		struct gc2035_mode mode;
    		if (copy_from_user(&mode,
    				   (const void __user *)arg,
    				   sizeof(struct gc2035_mode))) {
    			return -EFAULT;
    		}
		
    		return sensor_set_mode(info, &mode);
    	}
	case SENSOR_IOCTL_SET_OP_MODE:
    	{
    		KhOperationalMode mode;
    		if (copy_from_user(&mode,
    				   (const void __user *)arg,
    				   sizeof(KhOperationalMode))) {
    			return -EFAULT;
    		}
		
    		sensor_set_op_mode(info, &mode);
    	}
        case GC2035_IOCTL_SET_COLOR_EFFECT:
        {
            u8 coloreffect;

        	if (copy_from_user(&coloreffect,
        			   (const void __user *)arg,
        			   sizeof(coloreffect))) {
        		return -EFAULT;
        	}

            switch(coloreffect)
            {
                case YUV_ColorEffect_None:
                 err = sensor_write_table(info->i2c_client, sensor_Effect_Normal);
                     break;
                case YUV_ColorEffect_Mono:
                 err = sensor_write_table(info->i2c_client, sensor_Effect_WandB);
                     break;
                case YUV_ColorEffect_Sepia:
                 err = sensor_write_table(info->i2c_client, sensor_Effect_Sepia);
                     break;
                case YUV_ColorEffect_Negative:
                 err = sensor_write_table(info->i2c_client, sensor_Effect_Negative);
                     break;
                case YUV_ColorEffect_Solarize:
                 err = sensor_write_table(info->i2c_client, sensor_Effect_Bluish);
                     break;
                case YUV_ColorEffect_Posterize:
                 err = sensor_write_table(info->i2c_client, sensor_Effect_Green);
                     break;
                default: 	
                     break;
            }

            if (err)
    	        return err;

            return 0;
        }
        case GC2035_IOCTL_SET_WHITE_BALANCE:
        {
            u8 whitebalance;
        	if (copy_from_user(&whitebalance,
        			   (const void __user *)arg,
        			   sizeof(whitebalance))) {
        		return -EFAULT;
        	}

            switch(whitebalance)
            {
                case YUV_Whitebalance_Auto:

                     err = sensor_write_table(info->i2c_client, sensor_WhiteB_Auto);
		       info->current_wb =YUV_Whitebalance_Auto; 
                     break;
                case YUV_Whitebalance_Incandescent:

                 err = sensor_write_table(info->i2c_client, sensor_WhiteB_TungstenLamp1);
		       info->current_wb =YUV_Whitebalance_Incandescent; 
                     break;
                case YUV_Whitebalance_Daylight:
                 err = sensor_write_table(info->i2c_client, sensor_WhiteB_ClearDay);
			info->current_wb =YUV_Whitebalance_Daylight; 
                     break;
                case YUV_Whitebalance_Fluorescent:
                 err = sensor_write_table(info->i2c_client, sensor_WhiteB_TungstenLamp1);
			info->current_wb =YUV_Whitebalance_Fluorescent; 
                     break;
		  case YUV_Whitebalance_CloudyDaylight:
                 err = sensor_write_table(info->i2c_client, sensor_WhiteB_Cloudy);
			info->current_wb =YUV_Whitebalance_CloudyDaylight; 
                     break;
                default:
                     break;
            }
            if (err)
    	        return err;

            return 0;
        }
	 case GC2035_IOCTL_SET_YUV_EXPOSURE:
        {
	      u8 yuvexposure;
        	if (copy_from_user(&yuvexposure,
        			   (const void __user *)arg,
        			   sizeof(yuvexposure))) {
        		return -EFAULT;
        	}

            switch(yuvexposure)
            {
                case YUV_YUVExposure_Positive2:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure4);
			info->current_exposure=YUV_YUVExposure_Positive2;
                     break;
                case YUV_YUVExposure_Positive1:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure3);
		       info->current_exposure=YUV_YUVExposure_Positive1;
                     break;
                case YUV_YUVExposure_Number0:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure2);
			info->current_exposure=YUV_YUVExposure_Number0;
                     break;
                case YUV_YUVExposure_Negative1:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure1);
			info->current_exposure=YUV_YUVExposure_Negative1;
                     break;
		  case YUV_YUVExposure_Negative2:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure0);
			info->current_exposure=YUV_YUVExposure_Negative2;
                     break;
                default:
                     break;
            }
            if (err)
    	        return err;
            return 0;
        }
	 case GC2035_IOCTL_SET_SCENE_MODE:
        {
              u8 scene_mode;
        	if (copy_from_user(&scene_mode,
        			   (const void __user *)arg,
        			   sizeof(scene_mode))) {
        		return -EFAULT;
        	}

            switch(scene_mode)
            {
			 case YUV_YUVSCENE_Auto:
	                 err = sensor_write_table(info->i2c_client, sensor_SceneAuto);
	                 	break;
	                case YUV_YUVSCENE_Night:
	             		err = sensor_write_table(info->i2c_client, sensor_SceneNight);
	                 	break;
			  default:
                    		 break;
		}
              if (err)
    	        return err;

            return 0;
        }
	 case GC2035_IOCTL_GET_STATUS:
	{
		struct gc2035_status dev_status;
		if (copy_from_user(&dev_status,
				   (const void __user *)arg,
				   sizeof(struct gc2035_status))) {
			return -EFAULT;
		}

		err = gc2035_get_status(info, &dev_status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &dev_status,
				 sizeof(struct gc2035_status))) {
			return -EFAULT;
		}
		return 0;
	}
    	default:
    		return -EINVAL;
	}
	return 0;
}

static int sensor_open(struct inode *inode, struct file *file)
{
	pr_err("gc2035 %s\n",__func__);
	file->private_data = gc2035_sensor_info;
	if (gc2035_sensor_info->pdata && gc2035_sensor_info->pdata->power_on){
		gc2035_sensor_info->pdata->power_on();
		atomic_set(&gc2035_sensor_info->power_on_loaded, 1);
		atomic_set(&gc2035_sensor_info->sensor_init_loaded, 0);
	}
	return 0;
}

static int sensor_release(struct inode *inode, struct file *file)
{
	pr_err("gc2035 %s\n",__func__);
	if (gc2035_sensor_info->pdata && gc2035_sensor_info->pdata->power_off){
		gc2035_sensor_info->pdata->power_off();
		atomic_set(&gc2035_sensor_info->power_on_loaded, 0);
		atomic_set(&gc2035_sensor_info->sensor_init_loaded, 0);
	}
	file->private_data = NULL;
	return 0;
}


static const struct file_operations sensor_fileops = {
	.owner = THIS_MODULE,
	.open = sensor_open,
	.unlocked_ioctl = sensor_ioctl,
	.release = sensor_release,
};

static struct miscdevice sensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = SENSOR_NAME,
	.fops = &sensor_fileops,
};

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	pr_info("gc2035 %s\n",__func__);

	gc2035_sensor_info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (!gc2035_sensor_info) {
		pr_err("gc2035 : Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&sensor_device);
	if (err) {
		pr_err("gc2035 : Unable to register misc device!\n");
		goto EXIT;
	}
	gc2035_sensor_info->pdata = client->dev.platform_data;
	gc2035_sensor_info->i2c_client = client;
	atomic_set(&gc2035_sensor_info->power_on_loaded, 0);
	atomic_set(&gc2035_sensor_info->sensor_init_loaded, 0);
	i2c_set_clientdata(client, gc2035_sensor_info);
	pr_info("gc2035 %s register successfully!\n",__func__);
	return 0;
EXIT:
	kfree(gc2035_sensor_info);
	return err;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor_info *info;

	pr_info("gc2035 %s\n",__func__);
	info = i2c_get_clientdata(client);
	misc_deregister(&sensor_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "gc2035", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = "gc2035",
		.owner = THIS_MODULE,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static int __init sensor_init(void)
{
	pr_info("gc2035 %s\n",__func__);
	return i2c_add_driver(&sensor_i2c_driver);
}

static void __exit sensor_exit(void)
{
	pr_info("gc2035 %s\n",__func__);
	i2c_del_driver(&sensor_i2c_driver);
}

module_init(sensor_init);
module_exit(sensor_exit);

