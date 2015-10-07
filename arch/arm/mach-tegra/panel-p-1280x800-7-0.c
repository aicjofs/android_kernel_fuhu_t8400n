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
#define DSI_PANEL_LCD_EN_GPIO	TEGRA_GPIO_PH3
#define DSI_PANEL_BL_EN_GPIO TEGRA_GPIO_PH2
#define DSI_PANEL_BL_PWM	TEGRA_GPIO_PH1

#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE
#define SN65DSI83_MIPI_EN TEGRA_GPIO_PH4


static bool reg_requested;
static bool gpio_requested;
static int gpio_bl_en;
static int gpio_lcd_rst;
static int gpio_bridge_mip_en;

static struct platform_device *disp_device;
static struct regulator *avdd_lcd_3v3;
static struct regulator *vdd_lcd_bl;
static struct regulator *vdd_lcd_bl_en;
static struct regulator *dvdd_lcd_1v8;
static struct regulator *vdd_ds_1v8;


static tegra_dc_bl_output dsi_p_1280x800_7_0_bl_output_measured = {
	0, 0, 1, 2, 3, 4, 5, 6,
	7, 8, 9, 9, 10, 11, 12, 13,
	13, 14, 15, 16, 17, 17, 18, 19,
	20, 21, 22, 22, 23, 24, 25, 26,
	27, 27, 28, 29, 30, 31, 32, 32,
	33, 34, 35, 36, 37, 37, 38, 39,
	40, 41, 42, 42, 43, 44, 45, 46,
	47, 48, 48, 49, 50, 51, 52, 53,
	54, 55, 56, 57, 57, 58, 59, 60,
	61, 62, 63, 64, 65, 66, 67, 68,
	69, 70, 71, 71, 72, 73, 74, 75,
	76, 77, 77, 78, 79, 80, 81, 82,
	83, 84, 85, 87, 88, 89, 90, 91,
	92, 93, 94, 95, 96, 97, 98, 99,
	100, 101, 102, 103, 104, 105, 106, 107,
	108, 109, 110, 111, 112, 113, 115, 116,
	117, 118, 119, 120, 121, 122, 123, 124,
	125, 126, 127, 128, 129, 130, 131, 132,
	133, 134, 135, 136, 137, 138, 139, 141,
	142, 143, 144, 146, 147, 148, 149, 151,
	152, 153, 154, 155, 156, 157, 158, 158,
	159, 160, 161, 162, 163, 165, 166, 167,
	168, 169, 170, 171, 172, 173, 174, 176,
	177, 178, 179, 180, 182, 183, 184, 185,
	186, 187, 188, 189, 190, 191, 192, 194,
	195, 196, 197, 198, 199, 200, 201, 202,
	203, 204, 205, 206, 207, 208, 209, 210,
	211, 212, 213, 214, 215, 216, 217, 219,
	220, 221, 222, 224, 225, 226, 227, 229,
	230, 231, 232, 233, 234, 235, 236, 238,
	239, 240, 241, 242, 243, 244, 245, 246,
	247, 248, 249, 250, 251, 252, 253, 255
};

static struct tegra_dsi_cmd dsi_p_1280x800_7_0_init_cmd[] = {
	/* no init command required */
};

struct tegra_dsi_out dsi_p_1280x800_7_0_pdata = {
	.controller_vs = DSI_VS_1,

	.dsi2lvds_bridge_enable = true, //by david add
//davidlin check++++++
	.n_data_lanes = 4,
#if 1
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE,
#else	
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END,
#endif
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,//TEGRA_DSI_PIXEL_FORMAT_18BIT_P,
	.refresh_rate = 60,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.dsi_instance = DSI_INSTANCE_0,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
#if 1
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
#else
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
#endif	
	.dsi_init_cmd = dsi_p_1280x800_7_0_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_p_1280x800_7_0_init_cmd),

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
#if 0
	vdd_lcd_bl_en = regulator_get(dev, "vdd_lcd_bl_en");
	if (IS_ERR_OR_NULL(vdd_lcd_bl_en)) {
		pr_err("vdd_lcd_bl_en regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl_en);
		vdd_lcd_bl_en = NULL;
		goto fail;
	}
#endif
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
	gpio_direction_output(gpio_lcd_rst, 0);//for test

	gpio_bridge_mip_en= SN65DSI83_MIPI_EN;
	err = gpio_request(gpio_bridge_mip_en, "mipi enable");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		goto fail;
	}
	gpio_direction_output(gpio_bridge_mip_en, 0);

	gpio_bl_en = DSI_PANEL_BL_EN_GPIO;
	err = gpio_request(gpio_bl_en, "bl enable");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		gpio_bl_en = 0;
		goto fail;
	}
	gpio_direction_output(gpio_bl_en, 0);
	
	/* free pwm GPIO */
	err = gpio_request(DSI_PANEL_BL_PWM, "panel pwm");
	if (err < 0) {
		pr_err("panel pwm gpio request failed\n");
		goto fail;
	}
	gpio_free(DSI_PANEL_BL_PWM);
	gpio_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_p_1280x800_7_0_enable(struct device *dev)
{
	int err = 0;

	pr_err("%s:================>",__func__);
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

	if (vdd_lcd_bl_en) {
		err = regulator_enable(vdd_lcd_bl_en);
		if (err < 0) {
			pr_err("vdd_lcd_bl_en regulator enable failed\n");
			goto fail;
		}
	}

	msleep(100);

	gpio_set_value(gpio_bridge_mip_en, 1);
	msleep(50);
	gpio_set_value(gpio_bridge_mip_en, 0);
	msleep(50);
	gpio_set_value(gpio_bridge_mip_en, 1);
	
	
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

static int dsi_p_1280x800_7_0_disable(void)
{

pr_err("%s:================>",__func__);
	if (vdd_lcd_bl)
		regulator_disable(vdd_lcd_bl);
	
	if(gpio_bl_en>0){
		gpio_set_value(gpio_bl_en, 0);
	}
	
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

static int dsi_p_1280x800_7_0_postsuspend(void)
{
	return 0;
}
static struct tegra_dc_mode dsi_p_1280x800_7_0_modes[] = {
	{
		.pclk = 67166666,
		.h_ref_to_sync = 1, 
		.v_ref_to_sync = 1,
		.h_sync_width = 32, 
		.v_sync_width = 1, 
		.h_back_porch = 16,
		.v_back_porch = 2,
		.h_active = 800,
		.v_active = 1280,
		.h_front_porch = 16,
		.v_front_porch = 16,
	},
};


static int dsi_p_1280x800_7_0_bl_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = dsi_p_1280x800_7_0_bl_output_measured[brightness];

	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	return brightness;
}

static int dsi_p_1280x800_7_0_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &disp_device->dev;
}

static int dsi_p_1280x800_7_0_display_init(struct device *dev)
{
	return atomic_read(&display_ready);
}

static struct platform_pwm_backlight_data dsi_p_1280x800_7_0_bl_data = {
	.pwm_id		= 1,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 1000000,
	.notify		= dsi_p_1280x800_7_0_bl_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= dsi_p_1280x800_7_0_check_fb,
	.init = dsi_p_1280x800_7_0_display_init,
};

static struct platform_device __maybe_unused
		dsi_p_1280x800_7_0_bl_device __initdata = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &dsi_p_1280x800_7_0_bl_data,
	},
};

static struct platform_device __maybe_unused
			*dsi_p_1280x800_7_0_bl_devices[] __initdata = {
	&tegra_pwfm1_device,
	&dsi_p_1280x800_7_0_bl_device,
};

static int  __init dsi_p_1280x800_7_0_register_bl_dev(void)
{
	int err = 0;
	err = platform_add_devices(dsi_p_1280x800_7_0_bl_devices,
				ARRAY_SIZE(dsi_p_1280x800_7_0_bl_devices));
	if (err) {
		pr_err("disp1 bl device registration failed");
		return err;
	}
	return err;
}

static void dsi_p_1280x800_7_0_set_disp_device(
	struct platform_device *display_device)
{
	disp_device = display_device;
}

static void dsi_p_1280x800_7_0_resources_init(struct resource *
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

static void dsi_p_1280x800_7_0_dc_out_init(struct tegra_dc_out *dc)
{
	dc->dsi = &dsi_p_1280x800_7_0_pdata;
	dc->parent_clk = "pll_d_out0";
	dc->modes = dsi_p_1280x800_7_0_modes;
	dc->n_modes = ARRAY_SIZE(dsi_p_1280x800_7_0_modes);
	dc->enable = dsi_p_1280x800_7_0_enable;
	dc->disable = dsi_p_1280x800_7_0_disable;
	dc->postsuspend	= dsi_p_1280x800_7_0_postsuspend,
	dc->width = 151;
	dc->height = 94;
	dc->flags = DC_CTRL_MODE;
}

static void dsi_p_1280x800_7_0_fb_data_init(struct tegra_fb_data *fb)
{
	fb->xres = dsi_p_1280x800_7_0_modes[0].h_active;
	fb->yres = dsi_p_1280x800_7_0_modes[0].v_active;
}

static void
dsi_p_1280x800_7_0_sd_settings_init(struct tegra_dc_sd_settings *settings)
{
	settings->bl_device_name = "pwm-backlight";
}



struct tegra_panel __initdata dsi_p_1280x800_7_0 = {
	.init_sd_settings = dsi_p_1280x800_7_0_sd_settings_init,
	.init_dc_out = dsi_p_1280x800_7_0_dc_out_init,
	.init_fb_data = dsi_p_1280x800_7_0_fb_data_init,
	.init_resources = dsi_p_1280x800_7_0_resources_init,
	.register_bl_dev = dsi_p_1280x800_7_0_register_bl_dev,
	.set_disp_device = dsi_p_1280x800_7_0_set_disp_device,
};
EXPORT_SYMBOL(dsi_p_1280x800_7_0);

