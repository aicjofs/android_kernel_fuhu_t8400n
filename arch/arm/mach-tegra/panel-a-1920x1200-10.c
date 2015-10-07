/*
 * arch/arm/mach-tegra/panel-a-1920x1200-11-6.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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
#include "board.h"
#include "board-panel.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra11_host1x_devices.h"

#define TEGRA_DSI_GANGED_MODE	0
#define DSI_MIPI_RESET		0
#define DSI_PANEL_RESET		0
#define DSI_PANEL_LCD_EN_GPIO	TEGRA_GPIO_PH5
#define DSI_PANEL_BL_EN_GPIO TEGRA_GPIO_PN4
#define DSI_PANEL_BL_PWM	TEGRA_GPIO_PH1
#define DSI_VDD_BL_EN_GPIO TEGRA_GPIO_PH0
#define DSI_PANEL_BL_VDD_GPIO TEGRA_GPIO_PH2

#define DC_CTRL_MODE	(TEGRA_DC_OUT_CONTINUOUS_MODE | TEGRA_DC_OUT_INITIALIZED_MODE)
#define SN65DSI83_MIPI_EN TEGRA_GPIO_PH4

static atomic_t tegra_release_bootloader_fb_flag = ATOMIC_INIT(1);
static atomic_t dsi2lvds_enabled = ATOMIC_INIT(1);
static bool reg_requested;
static bool gpio_requested;
static int gpio_vdd_bl_en;
static int gpio_vdd_bl;
static int gpio_bl_en;
static int gpio_lcd_rst;
static int gpio_bridge_mip_en;

static struct platform_device *disp_device;
static struct regulator *avdd_lcd_3v3;



static struct tegra_dsi_cmd dsi_a_1920x1200_11_6_init_cmd[] = {
	/* no init command required */
};

static struct tegra_dsi_out dsi_a_1920x1200_11_6_pdata = {
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
	.dsi_init_cmd = dsi_a_1920x1200_11_6_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_a_1920x1200_11_6_init_cmd),

};

static int keenhi_dsi_regulator_get(struct device *dev)
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

	err = regulator_enable(avdd_lcd_3v3);
	if (err < 0) {
		pr_err("avdd_lcd regulator enable failed2\n");
		goto fail;
	}


	reg_requested = true;
	return 0;
fail:
	return err;
}

static int keenhi_dsi_gpio_get(void)
{
	int err = 0;

	if (gpio_requested)
		return 0;

	gpio_lcd_rst = DSI_PANEL_LCD_EN_GPIO;
	err = gpio_request(gpio_lcd_rst, "panel_enable");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		goto fail;
	}
	//gpio_direction_output(gpio_lcd_rst, 0);//for test

	gpio_bridge_mip_en= SN65DSI83_MIPI_EN;
	err = gpio_request(gpio_bridge_mip_en, "mipi_enable");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		goto fail;
	}
	//gpio_direction_output(gpio_bridge_mip_en, 0);

	gpio_vdd_bl= DSI_PANEL_BL_VDD_GPIO;
	err = gpio_request(gpio_vdd_bl, "bl_vdd");
	if (err < 0) {
	    pr_err("panel reset gpio request failed\n");
	    gpio_vdd_bl = 0;
	    goto fail;
	}
	//gpio_direction_output(DSI_PANEL_BL_VDD_GPIO, 1);

	gpio_bl_en = DSI_PANEL_BL_EN_GPIO;
	err = gpio_request(gpio_bl_en, "bl_enable");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		gpio_bl_en = 0;
		goto fail;
	}
	//gpio_direction_output(gpio_bl_en, 0);

	gpio_vdd_bl_en = DSI_VDD_BL_EN_GPIO;
	err = gpio_request(gpio_vdd_bl_en, "bl_vdd_enable");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		gpio_vdd_bl_en = 0;
		goto fail;
	}
	//gpio_direction_output(gpio_vdd_bl_en, 0);
	
	/* free pwm GPIO */
	err = gpio_request(DSI_PANEL_BL_PWM, "panel_pwm");
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

static int dsi_a_1920x1200_11_6_enable(struct device *dev)
{
	int err = 0;
	if(!atomic_read(&tegra_release_bootloader_fb_flag)) {
		tegra_release_bootloader_fb();
		atomic_set(&tegra_release_bootloader_fb_flag, 1);
	}
	//pr_err("%s:================>\n",__func__);
	err = keenhi_dsi_regulator_get(dev);
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}
	err = keenhi_dsi_gpio_get();
	if (err < 0) {
		pr_err("dsi gpio request failed\n");
		goto fail;
	}

	if(atomic_read(&dsi2lvds_enabled))return 0;


	if (avdd_lcd_3v3) {
		err = regulator_enable(avdd_lcd_3v3);
		if (err < 0) {
			pr_err("avdd_lcd regulator enable failed\n");
			goto fail;
		}
	}
	
	if(gpio_vdd_bl_en>0){
		gpio_set_value(gpio_vdd_bl_en, 1);
	}

	if(gpio_vdd_bl>0){
	    gpio_set_value(gpio_vdd_bl, 1);
	}

	if(gpio_bl_en>0){
		gpio_set_value(gpio_bl_en, 1);
	}

	if(gpio_lcd_rst>0){
		gpio_set_value(gpio_lcd_rst, 1);
	}

	gpio_set_value(gpio_bridge_mip_en, 1);
	msleep(50);
	gpio_set_value(gpio_bridge_mip_en, 0);
	msleep(50);
	gpio_set_value(gpio_bridge_mip_en, 1);
	msleep(50);
		

#if DSI_PANEL_RESET
	gpio_direction_output(gpio_lcd_rst, 1);
	usleep_range(1000, 5000);
	gpio_set_value(gpio_lcd_rst, 0);
	msleep(150);
	gpio_set_value(gpio_lcd_rst, 1);
	msleep(20);
#endif
	atomic_set(&dsi2lvds_enabled, 1);
	return 0;
fail:
	return err;
}

static int dsi_a_1920x1200_11_6_disable(void)
{
	//pr_err("%s:================>\n",__func__);

	if(!atomic_read(&dsi2lvds_enabled))return 0; //prevent unbalanced disable

	if(gpio_vdd_bl_en>0){
		gpio_set_value(gpio_vdd_bl_en, 0);
	}

	if(gpio_bl_en>0){
		gpio_set_value(gpio_bl_en, 0);
	}

	if(gpio_vdd_bl>0){
	    gpio_set_value(gpio_vdd_bl, 0);
	}
	
	if (avdd_lcd_3v3)
		regulator_disable(avdd_lcd_3v3);
	
	if(gpio_lcd_rst>0){
		gpio_set_value(gpio_lcd_rst, 0);
	}

	if(gpio_bridge_mip_en>0){
		gpio_set_value(gpio_bridge_mip_en, 0);
	}
	
	atomic_set(&dsi2lvds_enabled, 0);
	return 0;
}

static int dsi_a_1920x1200_11_6_postsuspend(void)
{
	//pr_err("%s:================>\n",__func__);
	return 0;
}

static struct tegra_dc_mode dsi_a_1920x1200_11_6_modes[] = {
	{
	       .pclk = 154700000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 16,
		.v_sync_width = 2,
		.h_back_porch = 32,
		.v_back_porch = 16,
		.h_active = 1920,
		.v_active = 1200,
		.h_front_porch = 120,
		.v_front_porch = 17,
	},
};

static int dsi_a_1920x1200_11_6_bl_notify(struct device *unused, int brightness)
{
	return brightness;
}

static int dsi_a_1920x1200_11_6_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &disp_device->dev;
}

static int dsi_p_1920x1200_7_0_display_init(struct device *dev)
{
	return atomic_read(&display_ready);
}
static struct platform_pwm_backlight_data dsi_a_1920x1200_11_6_bl_data = {
	.pwm_id		= 1,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 1000000,
	.notify		= dsi_a_1920x1200_11_6_bl_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= dsi_a_1920x1200_11_6_check_fb,
	.init = dsi_p_1920x1200_7_0_display_init,
};

static struct platform_device __maybe_unused
		dsi_a_1920x1200_11_6_bl_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &dsi_a_1920x1200_11_6_bl_data,
	},
};

static struct platform_device __maybe_unused
			*dsi_a_1920x1200_11_6_bl_devices[] __initdata = {
	&tegra_pwfm1_device,
	&dsi_a_1920x1200_11_6_bl_device,
};

static int  __init dsi_a_1920x1200_11_6_register_bl_dev(void)
{
	int err = 0;
	err = platform_add_devices(dsi_a_1920x1200_11_6_bl_devices,
				ARRAY_SIZE(dsi_a_1920x1200_11_6_bl_devices));
	if (err) {
		pr_err("disp1 bl device registration failed");
		return err;
	}
	return err;
}

static void dsi_a_1920x1200_11_6_set_disp_device(
	struct platform_device *dalmore_display_device)
{
	disp_device = dalmore_display_device;
}

static void dsi_a_1920x1200_11_6_resources_init(struct resource *
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

static void dsi_a_1920x1200_11_6_dc_out_init(struct tegra_dc_out *dc)
{
	dc->dsi = &dsi_a_1920x1200_11_6_pdata;
	dc->parent_clk = "pll_d_out0";
	dc->modes = dsi_a_1920x1200_11_6_modes;
	dc->n_modes = ARRAY_SIZE(dsi_a_1920x1200_11_6_modes);
	dc->enable = dsi_a_1920x1200_11_6_enable;
	dc->disable = dsi_a_1920x1200_11_6_disable;
	dc->postsuspend	= dsi_a_1920x1200_11_6_postsuspend,
	dc->width = 217;
	dc->height = 136;
	dc->flags = DC_CTRL_MODE;
}

static void dsi_a_1920x1200_11_6_fb_data_init(struct tegra_fb_data *fb)
{
	fb->xres = dsi_a_1920x1200_11_6_modes[0].h_active;
	fb->yres = dsi_a_1920x1200_11_6_modes[0].v_active;
}

static void
dsi_a_1920x1200_11_6_sd_settings_init(struct tegra_dc_sd_settings *settings)
{
	settings->bl_device_name = "pwm-backlight";
}



struct tegra_panel __initdata dsi_a_1920x1200_11_6 = {
	.init_sd_settings = dsi_a_1920x1200_11_6_sd_settings_init,
	.init_dc_out = dsi_a_1920x1200_11_6_dc_out_init,
	.init_fb_data = dsi_a_1920x1200_11_6_fb_data_init,
	.init_resources = dsi_a_1920x1200_11_6_resources_init,
	.register_bl_dev = dsi_a_1920x1200_11_6_register_bl_dev,
	.set_disp_device = dsi_a_1920x1200_11_6_set_disp_device,
};
EXPORT_SYMBOL(dsi_a_1920x1200_11_6);

