/*
 * arch/arm/mach-tegra/panel-p-wuxga-10-1.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <mach/dc.h>
#include <mach/iomap.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>
#include <linux/max8831_backlight.h>
#include <linux/leds.h>
#include <linux/ioport.h>
#include <generated/mach-types.h>
#include "board.h"
#include "board-panel.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra11_host1x_devices.h"

#define TEGRA_DSI_GANGED_MODE	0

#define DSI_PANEL_RESET		0
#define DSI_PANEL_LCD_EN_GPIO	TEGRA_GPIO_PH5
#define DSI_BL_EN_GPIO TEGRA_GPIO_PH0
#define DSI_PANEL_BL_PWM	TEGRA_GPIO_PH1
#define DSI_PANEL_BL_EN_GPIO TEGRA_GPIO_PH2

//TEGRA_DC_OUT_INITIALIZED_MODE

#define DC_CTRL_MODE	(TEGRA_DC_OUT_CONTINUOUS_MODE | TEGRA_DC_OUT_INITIALIZED_MODE)
#define SN65DSI83_MIPI_EN 0


static bool reg_requested;
static bool gpio_requested;
static int gpio_bl_en;
static int gpio_bl_pwm;
static int gpio_lcd_bl_en;

static int gpio_lcd_rst;
static int gpio_bridge_mip_en;
static atomic_t tegra_release_bootloader_fb_flag = ATOMIC_INIT(1);
static struct platform_device *disp_device;
static struct regulator *avdd_lcd_3v3;
static struct regulator *vdd_lcd_bl;
static struct regulator *vdd_lcd_bl_en;
static struct regulator *dvdd_lcd_1v8;
static struct regulator *vdd_ds_1v8;

#if 1
static tegra_dc_bl_output dsi_p_900x1440_7_0_bl_output_measured = {
	0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4,
	4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8,
	8, 8, 9, 9, 9, 10, 14, 21, 22, 23, 24, 25,
	27, 28, 29, 30, 31, 32, 32, 33, 34, 35, 36, 36,
	37, 38, 39, 39, 40, 41, 42, 43, 44, 44, 45, 46,
	47, 48, 49, 49, 50, 52, 53, 54, 55, 57, 58, 59,
	60, 61, 62, 63, 64, 65, 65, 66, 67, 68, 69, 70,
	71, 72, 73, 74, 76, 77, 78, 79, 80, 81, 82, 83,
	84, 85, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94,
	96, 97, 98, 100, 101, 102, 103, 105, 106, 107, 108, 110,
	111, 112, 113, 115, 116, 117, 118, 120, 121, 122, 123, 125,
	126, 127, 128, 130, 131, 131, 132, 133, 134, 135, 136, 136,
	137, 138, 139, 140, 141, 142, 142, 143, 144, 145, 146, 147,
	147, 148, 149, 150, 151, 152, 154, 155, 156, 157, 159, 160,
	161, 162, 162, 163, 164, 165, 166, 167, 167, 168, 169, 170,
	170, 171, 171, 172, 173, 173, 174, 174, 175, 175, 176, 176,
	177, 177, 178, 178, 179, 179, 180, 182, 191, 192, 194, 195,
	196, 198, 199, 200, 201, 201, 202, 203, 203, 204, 204, 205,
	206, 206, 207, 208, 208, 209, 210, 216, 221, 221, 222, 222,
	223, 224, 224, 225, 226, 226, 227, 228, 228, 229, 230, 238,
	241, 241, 242, 243, 243, 244, 244, 245, 246, 246, 247, 248,
	248, 249, 250, 255
};
#endif

static struct tegra_dsi_cmd dsi_p_900x1440_7_0_init_cmd[] = {
	/* no init command required */

	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_EXIT_SLEEP_MODE, 0x0),
	DSI_DLY_MS(120),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_ON, 0x0),
	DSI_DLY_MS(100),

};

static struct tegra_dsi_out dsi_p_900x1440_7_0_pdata = {
	.controller_vs = DSI_VS_1,
	.n_data_lanes = 4,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.refresh_rate = 60,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,
	.dsi_instance = DSI_INSTANCE_0,
	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
	.dsi_init_cmd = dsi_p_900x1440_7_0_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_p_900x1440_7_0_init_cmd),
};


static int macallan_dsi_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;
	avdd_lcd_3v3 = regulator_get(dev, "avdd_lcd");
	if (IS_ERR_OR_NULL(avdd_lcd_3v3)) {
		pr_err("avdd_lcd regulator get failed\n");
		err = PTR_ERR(avdd_lcd_3v3);
		avdd_lcd_3v3 = NULL;
		goto fail;
	}
	vdd_lcd_bl_en = NULL;

	reg_requested = true;
	return 0;
fail:
	return err;
}

static int macallan_dsi_gpio_get(void)
{
	int err = 0;

	if (gpio_requested)
		return 0;

	gpio_lcd_rst = DSI_PANEL_LCD_EN_GPIO;
	err = gpio_request(gpio_lcd_rst, "panel enable");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		goto fail;
	}
	//gpio_direction_output(gpio_lcd_rst, 0);//for test

	gpio_bridge_mip_en= SN65DSI83_MIPI_EN;
	if(gpio_bridge_mip_en>0){
		err = gpio_request(gpio_bridge_mip_en, "mipi enable");
		if (err < 0) {
			pr_err("panel reset gpio request failed\n");
			goto fail;
		}
		//gpio_direction_output(gpio_bridge_mip_en, 0);
	}

	gpio_bl_en = DSI_BL_EN_GPIO;
	err = gpio_request(gpio_bl_en, "bl enable");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		gpio_bl_en = 0;
		goto fail;
	}
	//gpio_direction_output(gpio_bl_en, 0);
	
	/* free pwm GPIO */
	gpio_bl_pwm = DSI_PANEL_BL_PWM;
	if(gpio_bl_pwm>0){
	err = gpio_request(gpio_bl_pwm, "panel_pwm");
	if (err < 0) {
		pr_err("panel pwm gpio request failed\n");
		goto fail;
	}
	gpio_free( gpio_bl_pwm);
	}
	
	
	gpio_lcd_bl_en= DSI_PANEL_BL_EN_GPIO;
	if(gpio_lcd_bl_en>0){
	err = gpio_request(gpio_lcd_bl_en, "plcd_bl_en");
	if (err < 0) {
		pr_err("panel pwm gpio request failed\n");
		goto fail;
	}
	//gpio_direction_output(gpio_lcd_bl_en, 0);
	}
	
	gpio_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_p_900x1440_7_0_enable(struct device *dev)
{
	int err = 0;
	if(!atomic_read(&tegra_release_bootloader_fb_flag)) {
		tegra_release_bootloader_fb();
		atomic_set(&tegra_release_bootloader_fb_flag, 1);
	}

	err = macallan_dsi_regulator_get(dev);
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}
	err = macallan_dsi_gpio_get();
	if (err < 0) {
		pr_err("dsi gpio request failed\n");
		goto fail;
	}

	if (vdd_ds_1v8) {
		err = regulator_enable(vdd_ds_1v8);
		if (err < 0) {
			pr_err("vdd_ds_1v8 regulator enable failed\n");
			goto fail;
		}
	}

	if (dvdd_lcd_1v8) {
		err = regulator_enable(dvdd_lcd_1v8);
		if (err < 0) {
			pr_err("dvdd_lcd regulator enable failed\n");
			goto fail;
		}
	}

	if (avdd_lcd_3v3) {
		err = regulator_enable(avdd_lcd_3v3);
		if (err < 0) {
			pr_err("avdd_lcd regulator enable failed\n");
			goto fail;
		}
	}
	if(gpio_lcd_rst>0){
		gpio_set_value(gpio_lcd_rst, 1);
	}
	
    msleep(300);

	if (vdd_lcd_bl) {
		err = regulator_enable(vdd_lcd_bl);
		if (err < 0) {
			pr_err("vdd_lcd_bl regulator enable failed\n");
			goto fail;
		}
	}

	if(gpio_bl_en>0){
		gpio_set_value(gpio_bl_en, 1);
	}
	
	if(gpio_bl_pwm>0){
		//gpio_set_value(gpio_bl_pwm, 1);
	}

	if(gpio_lcd_bl_en>0){
		gpio_set_value(gpio_lcd_bl_en, 1);
	}

	if (vdd_lcd_bl_en) {
		err = regulator_enable(vdd_lcd_bl_en);
		if (err < 0) {
			pr_err("vdd_lcd_bl_en regulator enable failed\n");
			goto fail;
		}
	}

	msleep(210);
	if(gpio_bridge_mip_en>0){
	gpio_set_value(gpio_bridge_mip_en, 1);
	msleep(50);
	gpio_set_value(gpio_bridge_mip_en, 0);
	msleep(50);
	gpio_set_value(gpio_bridge_mip_en, 1);
	}
	
#if DSI_PANEL_RESET
	gpio_direction_output(gpio_lcd_rst, 1);
	usleep_range(1000, 5000);
	gpio_set_value(gpio_lcd_rst, 0);
	msleep(150);
	gpio_set_value(gpio_lcd_rst, 1);
	msleep(20);
#endif

	return 0;
fail:
	return err;
}


static int dsi_p_900x1440_7_0_prepoweroff(void)
{
	if(gpio_bl_en>0)
	{
	  gpio_set_value(gpio_bl_en, 0);	
	}
	if(gpio_bl_pwm>0){
		//gpio_set_value(gpio_bl_pwm, 0);
	}
	if(gpio_lcd_bl_en>0){
		gpio_set_value(gpio_lcd_bl_en, 0);
	}
   msleep(120);
	return 0;
}


static int dsi_p_900x1440_7_0_disable(void)
{

	if (vdd_lcd_bl)
		regulator_disable(vdd_lcd_bl);
	
	if (vdd_lcd_bl_en)
		regulator_disable(vdd_lcd_bl_en);

	if (avdd_lcd_3v3)
		regulator_disable(avdd_lcd_3v3);
		
	if(gpio_lcd_rst>0){
		gpio_set_value(gpio_lcd_rst, 0);
	}

	if (dvdd_lcd_1v8)
		regulator_disable(dvdd_lcd_1v8);

	if(gpio_bridge_mip_en>0){
		gpio_set_value(gpio_bridge_mip_en, 0);
	}	
	if (vdd_ds_1v8)
		regulator_disable(vdd_ds_1v8);

	return 0;
}

static int dsi_p_900x1440_7_0_postsuspend(void)
{
	return 0;
}
static struct tegra_dc_mode dsi_p_900x1440_7_0_modes[] = {
	{
		.pclk = 92900000,
		.h_ref_to_sync = 2, 
		.v_ref_to_sync = 1,
		
		.h_sync_width = 2, 
		.v_sync_width = 2,
		.h_back_porch = 98,
		.v_back_porch = 10,
		.h_active = 900,
		.v_active = 1440,
		.h_front_porch = 60,
		.v_front_porch = 8,
	},
};
#ifdef CONFIG_TEGRA_DC_CMU
static struct tegra_dc_cmu dsi_p_900x1440_7_0_cmu = {
	/* lut1 maps sRGB to linear space. */
	{
	0,    1,    2,    4,    5,    6,    7,    9,
	10,   11,   12,   14,   15,   16,   18,   20,
	21,   23,   25,   27,   29,   31,   33,   35,
	37,   40,   42,   45,   48,   50,   53,   56,
	59,   62,   66,   69,   72,   76,   79,   83,
	87,   91,   95,   99,   103,  107,  112,  116,
	121,  126,  131,  136,  141,  146,  151,  156,
	162,  168,  173,  179,  185,  191,  197,  204,
	210,  216,  223,  230,  237,  244,  251,  258,
	265,  273,  280,  288,  296,  304,  312,  320,
	329,  337,  346,  354,  363,  372,  381,  390,
	400,  409,  419,  428,  438,  448,  458,  469,
	479,  490,  500,  511,  522,  533,  544,  555,
	567,  578,  590,  602,  614,  626,  639,  651,
	664,  676,  689,  702,  715,  728,  742,  755,
	769,  783,  797,  811,  825,  840,  854,  869,
	884,  899,  914,  929,  945,  960,  976,  992,
	1008, 1024, 1041, 1057, 1074, 1091, 1108, 1125,
	1142, 1159, 1177, 1195, 1213, 1231, 1249, 1267,
	1286, 1304, 1323, 1342, 1361, 1381, 1400, 1420,
	1440, 1459, 1480, 1500, 1520, 1541, 1562, 1582,
	1603, 1625, 1646, 1668, 1689, 1711, 1733, 1755,
	1778, 1800, 1823, 1846, 1869, 1892, 1916, 1939,
	1963, 1987, 2011, 2035, 2059, 2084, 2109, 2133,
	2159, 2184, 2209, 2235, 2260, 2286, 2312, 2339,
	2365, 2392, 2419, 2446, 2473, 2500, 2527, 2555,
	2583, 2611, 2639, 2668, 2696, 2725, 2754, 2783,
	2812, 2841, 2871, 2901, 2931, 2961, 2991, 3022,
	3052, 3083, 3114, 3146, 3177, 3209, 3240, 3272,
	3304, 3337, 3369, 3402, 3435, 3468, 3501, 3535,
	3568, 3602, 3636, 3670, 3705, 3739, 3774, 3809,
	3844, 3879, 3915, 3950, 3986, 4022, 4059, 4095,
	},
	/* csc */
	{
		0x138, 0x3Ba, 0x00D, /* 1.220 -0.270 0.050 */
		0x3F5, 0x120, 0x3E6, /* 0.038 1.124 0.100 */
		0x3FE, 0x3F8, 0x0E9, /* -0.002 -0.026 0.913 */
	},
	/* lut2 maps linear space to sRGB */
	{
		0, 1, 2, 3, 4, 5, 6, 6,
		7, 8, 9, 10, 11, 11, 12, 13,
		13, 14, 15, 15, 16, 17, 17, 18,
		18, 19, 19, 20, 20, 21, 21, 22,
		22, 23, 23, 23, 24, 24, 24, 25,
		25, 25, 26, 26, 26, 27, 27, 27,
		28, 28, 28, 28, 29, 29, 29, 29,
		30, 30, 30, 30, 31, 31, 31, 31,
		32, 32, 32, 32, 33, 33, 33, 33,
		34, 34, 34, 35, 35, 35, 35, 36,
		36, 36, 37, 37, 37, 37, 38, 38,
		38, 39, 39, 39, 39, 40, 40, 40,
		41, 41, 41, 41, 42, 42, 42, 43,
		43, 43, 43, 44, 44, 44, 45, 45,
		45, 45, 46, 46, 46, 46, 47, 47,
		47, 47, 48, 48, 48, 48, 49, 49,
		49, 49, 50, 50, 50, 50, 50, 51,
		51, 51, 51, 52, 52, 52, 52, 52,
		53, 53, 53, 53, 53, 53, 54, 54,
		54, 54, 54, 55, 55, 55, 55, 55,
		55, 56, 56, 56, 56, 56, 56, 57,
		57, 57, 57, 57, 57, 57, 58, 58,
		58, 58, 58, 58, 59, 59, 59, 59,
		59, 59, 59, 60, 60, 60, 60, 60,
		60, 60, 61, 61, 61, 61, 61, 61,
		61, 62, 62, 62, 62, 62, 62, 62,
		63, 63, 63, 63, 63, 63, 63, 64,
		64, 64, 64, 64, 64, 64, 65, 65,
		65, 65, 65, 65, 66, 66, 66, 66,
		66, 66, 66, 67, 67, 67, 67, 67,
		67, 68, 68, 68, 68, 68, 68, 69,
		69, 69, 69, 69, 69, 70, 70, 70,
		70, 70, 70, 71, 71, 71, 71, 71,
		71, 72, 72, 72, 72, 72, 72, 73,
		73, 73, 73, 73, 73, 74, 74, 74,
		74, 74, 74, 74, 75, 75, 75, 75,
		75, 75, 76, 76, 76, 76, 76, 76,
		77, 77, 77, 77, 77, 77, 77, 78,
		78, 78, 78, 78, 78, 79, 79, 79,
		79, 79, 79, 79, 80, 80, 80, 80,
		80, 80, 80, 80, 81, 81, 81, 81,
		81, 81, 81, 82, 82, 82, 82, 82,
		82, 82, 82, 83, 83, 83, 83, 83,
		83, 83, 83, 83, 84, 84, 84, 84,
		84, 84, 84, 84, 85, 85, 85, 85,
		85, 85, 85, 85, 85, 85, 86, 86,
		86, 86, 86, 86, 86, 86, 86, 86,
		87, 87, 87, 87, 87, 87, 87, 87,
		87, 87, 88, 88, 88, 88, 88, 88,
		88, 88, 88, 88, 88, 88, 89, 89,
		89, 89, 89, 89, 89, 89, 89, 89,
		89, 89, 90, 90, 90, 90, 90, 90,
		90, 90, 90, 90, 90, 90, 91, 91,
		91, 91, 91, 91, 91, 91, 91, 91,
		91, 91, 91, 92, 92, 92, 92, 92,
		92, 92, 92, 92, 92, 92, 92, 92,
		93, 93, 93, 93, 93, 93, 93, 93,
		93, 93, 93, 93, 93, 93, 94, 94,
		94, 94, 94, 94, 94, 94, 94, 94,
		94, 94, 94, 94, 95, 95, 95, 95,
		95, 95, 95, 95, 95, 95, 95, 95,
		95, 96, 96, 96, 96, 96, 96, 96,
		96, 96, 96, 96, 96, 96, 97, 97,
		97, 97, 97, 97, 97, 97, 97, 97,
		98, 99, 99, 100, 101, 101, 102, 103,
		103, 104, 105, 105, 106, 107, 107, 108,
		109, 110, 110, 111, 112, 112, 113, 114,
		114, 115, 115, 116, 117, 117, 118, 119,
		119, 120, 120, 121, 121, 122, 123, 123,
		124, 124, 125, 125, 126, 126, 127, 128,
		128, 129, 129, 130, 130, 131, 131, 132,
		132, 133, 133, 134, 134, 135, 135, 136,
		136, 137, 138, 138, 139, 139, 140, 140,
		141, 141, 142, 142, 143, 143, 144, 144,
		144, 145, 145, 146, 146, 147, 147, 148,
		148, 149, 149, 150, 150, 151, 151, 152,
		152, 153, 153, 153, 154, 154, 155, 155,
		156, 156, 157, 157, 157, 158, 158, 159,
		159, 160, 160, 160, 161, 161, 162, 162,
		162, 163, 163, 164, 164, 164, 165, 165,
		165, 166, 166, 167, 167, 167, 168, 168,
		168, 169, 169, 169, 170, 170, 171, 171,
		171, 172, 172, 172, 173, 173, 173, 174,
		174, 174, 175, 175, 175, 176, 176, 176,
		177, 177, 177, 178, 178, 178, 179, 179,
		179, 180, 180, 180, 181, 181, 181, 182,
		182, 182, 183, 183, 183, 184, 184, 184,
		185, 185, 185, 185, 186, 186, 186, 187,
		187, 187, 188, 188, 188, 189, 189, 189,
		190, 190, 190, 191, 191, 191, 191, 192,
		192, 192, 193, 193, 193, 194, 194, 194,
		195, 195, 195, 195, 196, 196, 196, 197,
		197, 197, 198, 198, 198, 199, 199, 199,
		199, 200, 200, 200, 201, 201, 201, 202,
		202, 202, 203, 203, 203, 203, 204, 204,
		204, 205, 205, 205, 206, 206, 206, 207,
		207, 207, 208, 208, 208, 208, 209, 209,
		209, 210, 210, 210, 211, 211, 211, 212,
		212, 212, 213, 213, 213, 214, 214, 214,
		215, 215, 215, 215, 216, 216, 216, 217,
		217, 217, 218, 218, 218, 219, 219, 219,
		220, 220, 220, 220, 221, 221, 221, 222,
		222, 222, 222, 223, 223, 223, 224, 224,
		224, 224, 225, 225, 225, 226, 226, 226,
		226, 227, 227, 227, 227, 228, 228, 228,
		229, 229, 229, 229, 230, 230, 230, 230,
		230, 231, 231, 231, 231, 232, 232, 232,
		232, 233, 233, 233, 233, 234, 234, 234,
		234, 234, 235, 235, 235, 235, 236, 236,
		236, 236, 236, 237, 237, 237, 237, 238,
		238, 238, 238, 238, 239, 239, 239, 239,
		239, 240, 240, 240, 240, 240, 241, 241,
		241, 241, 241, 242, 242, 242, 242, 243,
		243, 243, 243, 243, 244, 244, 244, 244,
		244, 245, 245, 245, 245, 245, 246, 246,
		246, 246, 246, 247, 247, 247, 247, 248,
		248, 248, 248, 248, 249, 249, 249, 249,
		250, 250, 250, 250, 251, 251, 251, 251,
		251, 252, 252, 252, 253, 253, 253, 253,
		254, 254, 254, 254, 255, 255, 255, 255,
	},
};
#endif

static int dsi_p_900x1440_7_0_bl_notify(struct device *unused, int brightness)
{
#if 1
	int cur_sd_brightness = atomic_read(&sd_brightness);
	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;
	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = dsi_p_900x1440_7_0_bl_output_measured[brightness];	
	/* SD brightness is a percentage */
	return  (255- brightness);
#else
  return  (255- brightness);
#endif
}

static int dsi_p_900x1440_7_0_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &disp_device->dev;
}

static int dsi_p_900x1440_7_0_display_init(struct device *dev)
{
	return atomic_read(&display_ready);
}

static struct platform_pwm_backlight_data dsi_p_900x1440_7_0_bl_data = {
	.pwm_id		= 1,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 1000000,
	.notify		= dsi_p_900x1440_7_0_bl_notify,
	.edp_states = { 3950, 2950, 1850, 940, 750, 0},
	.edp_brightness = {255, 213, 136, 73, 60, 0},
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= dsi_p_900x1440_7_0_check_fb,
	.init = dsi_p_900x1440_7_0_display_init,
};
/*if used __initdata attr will cause NULL pointer issue at the 17r17*/
static struct platform_device __maybe_unused
		dsi_p_900x1440_7_0_bl_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &dsi_p_900x1440_7_0_bl_data,
	},
};

static struct platform_device __maybe_unused
			*dsi_p_900x1440_7_0_bl_devices[] __initdata = {
	&tegra_pwfm1_device,
	&dsi_p_900x1440_7_0_bl_device,
};

static int  __init dsi_p_900x1440_7_0_register_bl_dev(void)
{
	int err = 0;

	err = platform_add_devices(dsi_p_900x1440_7_0_bl_devices,
				ARRAY_SIZE(dsi_p_900x1440_7_0_bl_devices));
	if (err) {
		pr_err("disp1 bl device registration failed");
		return err;
	}
	return err;
}

static void dsi_p_900x1440_7_0_set_disp_device(
	struct platform_device *display_device)
{
	disp_device = display_device;
}

static void dsi_p_900x1440_7_0_resources_init(struct resource *
resources, int n_resources)
{
	int i;
	for (i = 0; i < n_resources; i++) {
		struct resource *r = &resources[i];
		if (resource_type(r) == IORESOURCE_MEM &&
			!strcmp(r->name, "dsi_regs")) {
			r->start = TEGRA_DSI_BASE;
			r->end = TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1;
		}
	}
}

static void dsi_p_900x1440_7_0_dc_out_init(struct tegra_dc_out *dc)
{
	dc->dsi = &dsi_p_900x1440_7_0_pdata;
	dc->parent_clk = "pll_d_out0";
	dc->modes = dsi_p_900x1440_7_0_modes;
	dc->n_modes = ARRAY_SIZE(dsi_p_900x1440_7_0_modes);
	dc->enable = dsi_p_900x1440_7_0_enable;
	dc->prepoweroff= dsi_p_900x1440_7_0_prepoweroff;
	dc->disable = dsi_p_900x1440_7_0_disable;
	dc->postsuspend	= dsi_p_900x1440_7_0_postsuspend,
	dc->width = 94;
	dc->height = 151;
	dc->flags = DC_CTRL_MODE;
}

static void dsi_p_900x1440_7_0_fb_data_init(struct tegra_fb_data *fb)
{
	fb->xres = dsi_p_900x1440_7_0_modes[0].h_active;
	fb->yres = dsi_p_900x1440_7_0_modes[0].v_active;
}

static void
dsi_p_900x1440_7_0_sd_settings_init(struct tegra_dc_sd_settings *settings)
{
	settings->bl_device_name = "pwm-backlight";
}

static void dsi_p_900x1440_7_0_cmu_init(struct tegra_dc_platform_data *pdata)
{
#ifdef CONFIG_TEGRA_DC_CMU
	pdata->cmu = &dsi_p_900x1440_7_0_cmu;
#endif
}

struct tegra_panel __initdata dsi_p_900x1440_7_0 = {
	.init_sd_settings = dsi_p_900x1440_7_0_sd_settings_init,
	.init_dc_out = dsi_p_900x1440_7_0_dc_out_init,
	.init_fb_data = dsi_p_900x1440_7_0_fb_data_init,
	.init_resources = dsi_p_900x1440_7_0_resources_init,
	.register_bl_dev = dsi_p_900x1440_7_0_register_bl_dev,
	.init_cmu_data = dsi_p_900x1440_7_0_cmu_init,
	.set_disp_device = dsi_p_900x1440_7_0_set_disp_device,
};
EXPORT_SYMBOL(dsi_p_900x1440_7_0);

