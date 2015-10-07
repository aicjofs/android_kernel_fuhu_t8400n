/*
 * LP3367.c - LP3367 flash/torch kernel driver
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.

 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.

 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/edp.h>
#include <linux/regmap.h>
#include <media/nvc.h>
#include <media/lp3367.h>

/* #define DEBUG_I2C_TRAFFIC */

#define LP3367_MAX_FLASH_LEVEL		256
#define LP3367_MAX_TORCH_LEVEL		128

#define LP3367_FLASH_TIMER_NUM		256
#define LP3367_TORCH_TIMER_NUM		1

#define LP3367_REG_CHIPID		0x00
#define LP3367_REG_LED1_SET_CURR	0x01
#define LP3367_REG_LED2_SET_CURR	0x02
#define LP3367_REG_TXMASK		0x03
#define LP3367_REG_LOWVOLTAGE		0x04
#define LP3367_REG_FLASHTIMER		0x05
#define LP3367_REG_CONTROL		0x06
#define LP3367_REG_STROBE		0x07
#define LP3367_REG_FAULT		0x08
#define LP3367_REG_PWM_INDICATOR	0x09
#define LP3367_REG_LED_CURR_MIN		0x0E
#define LP3367_REG_LED_CURR_ACT		0x0F
#define LP3367_REG_PASSWORD		0x80
#define LP3367_REG_CURR_BOOST		0x81

#define LP3367_REG_CONTROL_MODE_NONE	0x00
#define LP3367_REG_CONTROL_MODE_IND	0x01
#define LP3367_REG_CONTROL_MODE_TORCH	0x02
#define LP3367_REG_CONTROL_MODE_FLASH	0x03

#define LP3367_LEVEL_OFF		0
#define LP3367_TORCH_TIMER_FOREVER	0xFFFFFFFF

#define SUSTAINTIME_DEF			558
#define DEFAULT_FLASHTIME	((SUSTAINTIME_DEF > 256) ? \
				((SUSTAINTIME_DEF - 249) / 8 + 128) : \
				((SUSTAINTIME_DEF - 1) / 2))

#define LP3367_MAX_ASSIST_CURRENT(x)    \
			DIV_ROUND_UP(((x) * 0xff * 0x7f / 0xff), 1000)
#define LP3367_MAX_INDICATOR_CURRENT(x) \
			DIV_ROUND_UP(((x) * 0xff * 0x3f / 0xff), 1000)

#define GET_CURRENT_BY_INDEX(i, c)	\
			((c) * (i)->dev_cap->curr_step_uA / 1000)
#define GET_INDEX_BY_CURRENT(i, c)	\
			((c) * 1000 / (i)->dev_cap->curr_step_uA)

#define lp3367_flash_cap_size \
			(sizeof(struct nvc_torch_flash_capabilities_v1) \
			+ sizeof(struct nvc_torch_lumi_level_v1) \
			* LP3367_MAX_FLASH_LEVEL)
#define lp3367_flash_timeout_size \
			(sizeof(struct nvc_torch_timer_capabilities_v1) \
			+ sizeof(struct nvc_torch_timeout_v1) \
			* LP3367_FLASH_TIMER_NUM)
#define lp3367_max_flash_cap_size (lp3367_flash_cap_size * 2 \
			+ lp3367_flash_timeout_size * 2)

#define lp3367_torch_cap_size \
			(sizeof(struct nvc_torch_torch_capabilities_v1) \
			+ sizeof(struct nvc_torch_lumi_level_v1) \
			* LP3367_MAX_TORCH_LEVEL)
#define lp3367_torch_timeout_size \
			(sizeof(struct nvc_torch_timer_capabilities_v1) \
			+ sizeof(struct nvc_torch_timeout_v1) \
			* LP3367_TORCH_TIMER_NUM)
#define lp3367_max_torch_cap_size (lp3367_torch_timeout_size * 2\
			+ lp3367_torch_timeout_size * 2)

struct lp3367_caps_struct {
	char *name;
	u32 curr_step_uA;
	u32 curr_step_boost_uA;
	u32 txmask_step_uA;
	u32 txmask_step_boost_uA;
	u32 num_regs;
	u32 max_peak_curr_mA;
	u32 min_ilimit_mA;
	u32 max_assist_curr_mA;
	u32 max_indicator_curr_mA;
	bool led2_support;
};

struct lp3367_reg_cache {
	u8 dev_id;
	u8 led1_curr;
	u8 led2_curr;
	u8 txmask;
	u8 strobe;
	u8 ftime;
	u8 vlow;
	u8 pwm_ind;
};

struct lp3367_info {
	struct platform_device *pdev;
	struct miscdevice miscdev;
	struct device *dev;
	struct dentry *d_lp3367;
	struct regmap *regmap;
	struct list_head list;
	struct mutex mutex;
	struct regulator *v_in;
	struct lp3367_power_rail power;
	struct lp3367_platform_data *pdata;
	const struct lp3367_caps_struct *dev_cap;
	struct nvc_torch_capability_query query;
	struct nvc_torch_flash_capabilities_v1 *flash_cap[2];
	struct nvc_torch_timer_capabilities_v1 *flash_timeouts[2];
	struct nvc_torch_torch_capabilities_v1 *torch_cap[2];
	struct nvc_torch_timer_capabilities_v1 *torch_timeouts[2];
	struct lp3367_config config;
	struct lp3367_reg_cache regs;
#if 0
	struct edp_client *edpc;
	unsigned edp_state;
#endif
	atomic_t in_use;
	int flash_cap_size;
	int torch_cap_size;
	int pwr_state;
	u8 max_flash[2];
	u8 max_torch[2];
	u8 s_mode;
	u8 op_mode;
	u8 led_num;
	u8 led_mask;
	u8 power_on;
};

static const struct lp3367_caps_struct lp3367_caps[] = {
	{"as3643", 5098, 0, 81600, 0, 11, 1300, 1000,
		LP3367_MAX_ASSIST_CURRENT(5098),
		LP3367_MAX_INDICATOR_CURRENT(5098), false},
	{"as3647", 6274, 0, 100400, 0, 11, 1600, 2000,
		LP3367_MAX_ASSIST_CURRENT(6274),
		LP3367_MAX_INDICATOR_CURRENT(6274), false},
	{"as3648", 3529, 3921, 56467, 62747, 14, 1000, 2000,
		LP3367_MAX_ASSIST_CURRENT(3529),
		LP3367_MAX_INDICATOR_CURRENT(3529), true},
};

static const u16 v_in_low[] = {0, 3000, 3070, 3140, 3220, 3300, 3338, 3470};

/* flash timer duration settings in uS */
static u32 lp3367_flash_timer[LP3367_FLASH_TIMER_NUM];

static u32 lp3367_torch_timer[LP3367_TORCH_TIMER_NUM] = {
	LP3367_TORCH_TIMER_FOREVER
};

static struct nvc_torch_lumi_level_v1
	lp3367_def_flash_levels[LP3367_MAX_FLASH_LEVEL - 1];

/* translated from the default register values after power up */
static const struct lp3367_config default_cfg = {
	.led_mask	= 3,
	.use_tx_mask = 0,
	.I_limit_mA = 3000,
	.txmasked_current_mA = 339,
	.vin_low_v_run_mV = 0,
	.vin_low_v_mV = 0,
	.strobe_type = 2,
	.freq_switch_on = 0,
	.led_off_when_vin_low = 0,
	.max_peak_current_mA = 900,
	.max_sustained_current_mA = 0,
	.max_peak_duration_ms = 0,
	.min_current_mA = 0,
	.def_ftimer = 0x23,
	.led_config[0] = {
		.flash_torch_ratio = 10000,
		.granularity = 1000,
		.flash_levels = ARRAY_SIZE(lp3367_def_flash_levels),
		.lumi_levels = lp3367_def_flash_levels,
	},
	.led_config[1] = {
		.flash_torch_ratio = 10000,
		.granularity = 1000,
		.flash_levels = ARRAY_SIZE(lp3367_def_flash_levels),
		.lumi_levels = lp3367_def_flash_levels,
	},
};

static struct lp3367_platform_data lp3367_default_pdata = {
	.cfg		= 0,
	.num		= 0,
	.dev_name	= "torch",
	.pinstate	= {0x0000, 0x0000},
};

#if 0
static const struct regmap_config lp3367_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};
#endif

static const struct platform_device_id lp3367_id[] = {
	{ "lp3367", 0 },
	{ },
};

static LIST_HEAD(lp3367_info_list);
static DEFINE_SPINLOCK(lp3367_spinlock);

#if 0
static int lp3367_debugfs_init(struct lp3367_info *info);
#endif

static inline void lp3367_i2c_dump(
	struct lp3367_info *info, u8 reg, u8 *buf, u8 num)
{
#if 0
#ifdef DEBUG_I2C_TRAFFIC
	static unsigned char i2c_buf[32 + 3 * 16];
	int len = sprintf(i2c_buf, "%s %02x =", __func__, reg);
	int i;

	for (i = 0; i < num; i++)
		len += sprintf(i2c_buf + len, " %02x", buf[i]);
	i2c_buf[len] = 0;
	dev_info(info->dev, "%s\n", i2c_buf);
#else
	if (num == 1) {
		dev_dbg(info->dev, "%s %02x = %02x\n", __func__, reg, *buf);
		return;
	}
	dev_dbg(info->dev, "%s %02x = %02x %02x ...\n",
		__func__, reg, buf[0], buf[1]);
#endif
#endif
}

static inline int lp3367_reg_rd(struct lp3367_info *info, u8 reg, u8 *val)
{
#if 0
	int err = -ENODEV;

	mutex_lock(&info->mutex);
	if (info->power_on)
		err = regmap_raw_read(info->regmap, reg, val, sizeof(*val));
	else
		dev_err(info->dev, "%s: power is off.\n", __func__);
	mutex_unlock(&info->mutex);

	return err;
#endif
	return 0;
}

static int lp3367_reg_raw_wr(struct lp3367_info *info, u8 reg, u8 *buf, u8 num)
{
#if 0
	int err = -ENODEV;

	lp3367_i2c_dump(info, reg, buf, num);
	mutex_lock(&info->mutex);
	if (info->power_on)
		err = regmap_raw_write(info->regmap, reg, buf, num);
	else
		dev_err(info->dev, "%s: power is off.\n", __func__);
	mutex_unlock(&info->mutex);

	return err;
#endif
	return 0;
}

static int lp3367_reg_wr(struct lp3367_info *info, u8 reg, u8 val)
{
#if 0
	int err = -ENODEV;

	lp3367_i2c_dump(info, reg, &val, 1);
	mutex_lock(&info->mutex);
	if (info->power_on)
		err = regmap_write(info->regmap, reg, val);
	else
		dev_err(info->dev, "%s: power is off.\n", __func__);
	mutex_unlock(&info->mutex);

	return err;
#endif
	return 0;
}

#if 0
static void lp3367_edp_lowest(struct lp3367_info *info)
{
	if (!info->edpc)
		return;

	info->edp_state = info->edpc->num_states - 1;
	dev_dbg(&info->dev, "%s %d\n", __func__, info->edp_state);
	if (edp_update_client_request(info->edpc, info->edp_state, NULL)) {
		dev_err(&info->dev, "THIS IS NOT LIKELY HAPPEN!\n");
		dev_err(&info->dev,
			"UNABLE TO SET LOWEST EDP STATE!\n");
	}
}

static void lp3367_throttle(unsigned int new_state, void *priv_data);
static void lp3367_edp_register(struct lp3367_info *info)
{
	struct edp_manager *edp_manager;
	struct edp_client *edpc = &info->pdata->edpc_config;
	int ret;

	info->edpc = NULL;
	if (!edpc->num_states) {
		dev_notice(&info->dev,
			"%s: NO edp states defined.\n", __func__);
		return;
	}

	strncpy(edpc->name, "lp3367", EDP_NAME_LEN - 1);
	edpc->name[EDP_NAME_LEN - 1] = 0;
	edpc->throttle = lp3367_throttle;
	edpc->private_data = info;

	dev_dbg(&info->dev, "%s: %s, e0 = %d, p %d\n",
		__func__, edpc->name, edpc->e0_index, edpc->priority);
	for (ret = 0; ret < edpc->num_states; ret++)
		dev_dbg(&info->dev, "e%d = %d mA\n",
			ret - edpc->e0_index, edpc->states[ret]);

	edp_manager = edp_get_manager("battery");
	if (!edp_manager) {
		dev_err(&info->dev,
			"unable to get edp manager: battery\n");
		return;
	}

	ret = edp_register_client(edp_manager, edpc);
	if (ret) {
		dev_err(&info->dev,
			"unable to register edp client\n");
		return;
	}

	info->edpc = edpc;
	/* set to lowest state at init */
	lp3367_edp_lowest(info);
}

static int lp3367_edp_req(struct lp3367_info *info,
		u8 mask, u8 *curr1, u8 *curr2)
{
	unsigned *estates;
	unsigned total_curr = 0;
	unsigned curr_mA;
	unsigned approved;
	unsigned new_state;
	int ret = 0;

	if (!info->edpc)
		return 0;

	dev_dbg(&info->dev, "%s: %d curr1 = %02x curr2 = %02x\n",
		__func__, mask, *curr1, *curr2);
	estates = info->edpc->states;
	if (mask & 1)
		total_curr += *curr1;
	if (mask & 2)
		total_curr += *curr2;
	curr_mA = GET_CURRENT_BY_INDEX(info, total_curr);

	for (new_state = info->edpc->num_states - 1; new_state > 0; new_state--)
		if (estates[new_state] >= curr_mA)
			break;

	dev_dbg(&info->dev,
		"edp req: %d curr = %d mA\n", new_state, curr_mA);
	ret = edp_update_client_request(info->edpc, new_state, &approved);
	if (ret) {
		dev_err(&info->dev, "E state transition failed\n");
		return ret;
	}

	if (approved > new_state) { /* edp manager returned less current */
		curr_mA = GET_INDEX_BY_CURRENT(info, estates[approved]);
		if (mask & 1)
			*curr1 = curr_mA * (*curr1) / total_curr;
		*curr2 = curr_mA - (*curr1);
		dev_dbg(&info->dev,
			"new state: %d curr = %d mA (%d %d)\n",
			approved, curr_mA, *curr1, *curr2);
	}

	info->edp_state = approved;

	return 0;
}
#endif

static int lp3367_set_leds(struct lp3367_info *info,
			u8 mask, u8 curr1, u8 curr2)
{
	int err = 0;
	u8 regs[6];

	if ((info->op_mode == LP3367_REG_CONTROL_MODE_FLASH) &&
			(curr1 || curr2 )) {
		gpio_set_value(info->pdata->led_en, 1);
		gpio_set_value(info->pdata->flash_en, 1);
	} else if ((info->op_mode == LP3367_REG_CONTROL_MODE_TORCH) &&
			(curr1 || curr2)) {
		gpio_set_value(info->pdata->led_en, 1);
		gpio_set_value(info->pdata->flash_en, 0);
	} else {
		gpio_set_value(info->pdata->led_en, 0);
		gpio_set_value(info->pdata->flash_en, 0);
	}

	if (mask & 1) {
		if (info->op_mode == LP3367_REG_CONTROL_MODE_FLASH) {
			if (curr1 >= info->max_flash[0])
				curr1 = info->max_flash[0];
		} else {
			if (curr1 >= info->max_torch[0])
				curr1 = info->max_torch[0];
		}
	} else
		curr1 = 0;

	if (mask & 2 && info->dev_cap->led2_support) {
		if (info->op_mode == LP3367_REG_CONTROL_MODE_FLASH) {
			if (curr2 >= info->max_flash[1])
				curr2 = info->max_flash[1];
		} else {
			if (curr2 >= info->max_torch[1])
				curr2 = info->max_torch[1];
		}
	} else
		curr2 = 0;

#if 0
	err = lp3367_edp_req(info, mask, &curr1, &curr2);
	if (err)
		return err;
#endif

	regs[0] = curr1;
	regs[1] = curr2;
	regs[2] = info->regs.txmask;
	regs[3] = info->regs.vlow;
	regs[4] = info->regs.ftime;
	if (mask == 0 || (curr1 == 0 && curr2 == 0))
		regs[5] = info->op_mode & (~0x08);
	else
		regs[5] = info->op_mode | 0x08;

	err = lp3367_reg_raw_wr(
		info, LP3367_REG_LED1_SET_CURR, regs, sizeof(regs));
	if (!err) {
		info->regs.led1_curr = curr1;
		info->regs.led2_curr = curr2;
#if 0
		if ((curr1 | curr2) == 0)
			lp3367_edp_lowest(info);
#endif
	}

	dev_dbg(info->dev, "%s %x %x %x %x control = %x\n",
			__func__, mask, curr1, curr2,
			info->regs.ftime, regs[5]);
	return err;
}

static int lp3367_set_txmask(struct lp3367_info *info)
{
	const struct lp3367_caps_struct *p_cap = info->dev_cap;
	struct lp3367_config *p_cfg = &info->config;
	int err;
	u8 tm;
	u32 limit = 0, txmask;

	dev_dbg(info->dev, "%s\n", __func__);

	tm = p_cfg->use_tx_mask ? 1 : 0;

	if (p_cfg->I_limit_mA > p_cap->min_ilimit_mA)
		limit = (p_cfg->I_limit_mA - p_cap->min_ilimit_mA) / 500;

	if (limit > 3)
		limit = 3;
	tm |= limit<<2;

	txmask = p_cfg->txmasked_current_mA * 1000;

	if (p_cfg->boost_mode)
		txmask /= p_cap->txmask_step_boost_uA;
	else
		txmask /= p_cap->txmask_step_uA;

	if (txmask > 0xf)
		txmask = 0xf;

	tm |= txmask<<4;

	err = lp3367_reg_wr(info, LP3367_REG_TXMASK, tm);
	if (!err)
		info->regs.txmask = tm;

	return err;
}

static int lp3367_get_vin_index(u16 mV)
{
	int vin;

	for (vin = ARRAY_SIZE(v_in_low) - 1; vin >= 0; vin--) {
		if (mV >= v_in_low[vin])
			break;
	}

	return vin;
}

static void lp3367_config_init(struct lp3367_info *info)
{
	struct lp3367_config *pcfg = &info->config;
	struct lp3367_config *pcfg_cust = &info->pdata->config;
	unsigned i;

	dev_dbg(info->dev, "%s +++\n", __func__);
	dev_dbg(info->dev, "lp3367_def_flash_levels:\n");
	for (i = 0; i < ARRAY_SIZE(lp3367_def_flash_levels); i++) {
		lp3367_def_flash_levels[i].guidenum = i + 1;
		lp3367_def_flash_levels[i].luminance =
			(i + 1) * info->dev_cap->curr_step_uA;
		dev_dbg(info->dev, "0x%02x - %d\n",
			i, lp3367_def_flash_levels[i].luminance);
	}

	dev_dbg(info->dev, "lp3367_flash_timer:\n");
	/* 0 ~ 0x7f, 2 ms step / 0x80 ~ 0xff, 8 ms step*/
	for (i = 0; i < 0x80; i++) {
		lp3367_flash_timer[i] = (i + 1) * 2000;
		dev_dbg(info->dev,
			"0x%02x - %06d\n", i, lp3367_flash_timer[i]);
	}
	for (; i < ARRAY_SIZE(lp3367_flash_timer); i++) {
		lp3367_flash_timer[i] = 256000 + (i - 0x7f) * 8000;
		dev_dbg(info->dev,
			"0x%02x - %06d\n", i, lp3367_flash_timer[i]);
	}

	memcpy(pcfg, &default_cfg, sizeof(*pcfg));
	if (!info->pdata) {
		info->pdata = &lp3367_default_pdata;
		dev_dbg(info->dev, "%s No platform data.  Using defaults.\n",
			__func__);
		goto config_init_done;
	}
	pcfg_cust = &info->pdata->config;

	pcfg->synchronized_led = pcfg_cust->synchronized_led;
	pcfg->use_tx_mask = pcfg_cust->use_tx_mask;
	pcfg->freq_switch_on = pcfg_cust->freq_switch_on;
	pcfg->inct_pwm = pcfg_cust->inct_pwm;
	pcfg->load_balance_on = pcfg_cust->load_balance_on;
	pcfg->led_off_when_vin_low = pcfg_cust->led_off_when_vin_low;
	pcfg->boost_mode = pcfg_cust->boost_mode;

	if (pcfg_cust->led_mask)
		pcfg->led_mask = pcfg_cust->led_mask;

	if (pcfg_cust->strobe_type)
		pcfg->strobe_type = pcfg_cust->strobe_type;

	if (pcfg_cust->vin_low_v_run_mV) {
		if (pcfg_cust->vin_low_v_run_mV == 0xffff)
			pcfg->vin_low_v_run_mV = 0;
		else
			pcfg->vin_low_v_run_mV = pcfg_cust->vin_low_v_run_mV;
	}

	if (pcfg_cust->vin_low_v_mV) {
		if (pcfg_cust->vin_low_v_mV == 0xffff)
			pcfg->vin_low_v_mV = 0;
		else
			pcfg->vin_low_v_mV = pcfg_cust->vin_low_v_mV;
	}

	if (pcfg_cust->I_limit_mA)
		pcfg->I_limit_mA = pcfg_cust->I_limit_mA;

	if (pcfg_cust->txmasked_current_mA)
		pcfg->txmasked_current_mA = pcfg_cust->txmasked_current_mA;

	if (pcfg_cust->max_total_current_mA)
		pcfg->max_total_current_mA = pcfg_cust->max_total_current_mA;

	if (pcfg_cust->max_peak_current_mA)
		pcfg->max_peak_current_mA = pcfg_cust->max_peak_current_mA;

	if (pcfg_cust->max_torch_current_mA)
		pcfg->max_torch_current_mA = pcfg_cust->max_torch_current_mA;

	if (pcfg_cust->max_peak_duration_ms)
		pcfg->max_peak_duration_ms = pcfg_cust->max_peak_duration_ms;

	if (pcfg_cust->max_sustained_current_mA)
		pcfg->max_sustained_current_mA =
			pcfg_cust->max_sustained_current_mA;

	if (pcfg_cust->min_current_mA)
		pcfg->min_current_mA = pcfg_cust->min_current_mA;

	for (i = 0; i < 2; i++) {
		if (pcfg_cust->led_config[i].flash_levels &&
			pcfg_cust->led_config[i].flash_torch_ratio &&
			pcfg_cust->led_config[i].granularity &&
			pcfg_cust->led_config[i].lumi_levels)
			memcpy(&pcfg->led_config[i], &pcfg_cust->led_config[i],
				sizeof(pcfg_cust->led_config[0]));
		else
			dev_notice(info->dev,
				"%s: invalid led config[%d].\n", __func__, i);
	}

config_init_done:
	dev_dbg(info->dev, "%s ---\n", __func__);
}

static int lp3367_update_settings(struct lp3367_info *info)
{
	int err;

	err = lp3367_set_txmask(info);

	err |= lp3367_reg_wr(info, LP3367_REG_LOWVOLTAGE, info->regs.vlow);

	err |= lp3367_reg_wr(info, LP3367_REG_PWM_INDICATOR,
			info->regs.pwm_ind);

	err |= lp3367_reg_wr(info, LP3367_REG_STROBE, info->regs.strobe);

	if (info->dev_cap->led2_support) {
		err |= lp3367_reg_wr(info, LP3367_REG_PASSWORD, 0xa1);
		if (info->config.boost_mode)
			err |= lp3367_reg_wr(info, LP3367_REG_CURR_BOOST, 1);
		else
			err |= lp3367_reg_wr(info, LP3367_REG_CURR_BOOST, 0);
	}

	err |= lp3367_set_leds(info,
		info->led_mask, info->regs.led1_curr, info->regs.led2_curr);

	dev_dbg(info->dev, "UP: strobe: %x pwm_ind: %x vlow: %x\n",
		info->regs.strobe, info->regs.pwm_ind, info->regs.vlow);
	return err;
}

static int lp3367_configure(struct lp3367_info *info, bool update)
{
	struct lp3367_config *pcfg = &info->config;
	const struct lp3367_caps_struct *pcap = info->dev_cap;
	struct nvc_torch_capability_query *pqry = &info->query;
	struct nvc_torch_flash_capabilities_v1	*pfcap = NULL;
	struct nvc_torch_torch_capabilities_v1	*ptcap = NULL;
	struct nvc_torch_timer_capabilities_v1	*ptmcap = NULL;
	struct nvc_torch_lumi_level_v1		*plvls = NULL;
	int val;
	int i;
	int j;

	if (!pcap->led2_support)
		pcfg->boost_mode = false;

	val = lp3367_get_vin_index(pcfg->vin_low_v_run_mV);
	info->regs.vlow = val<<0;

	val = lp3367_get_vin_index(pcfg->vin_low_v_mV);
	info->regs.vlow |= val<<3;

	if (pcfg->led_off_when_vin_low)
		info->regs.vlow |= 0x40;

	info->regs.pwm_ind = pcfg->inct_pwm & 0x03;
	if (pcfg->freq_switch_on)
		info->regs.pwm_ind |= 0x04;
	if (pcfg->load_balance_on)
		info->regs.pwm_ind |= 0x20;

	switch (pcfg->strobe_type) {
	case 1:
		info->regs.strobe = 0x80;
		break;
	case 2:
		info->regs.strobe = 0xc0;
		break;
	case 3:
	default:
		info->regs.strobe = 0x00;
		break;
	}

	info->led_mask = pcfg->led_mask;

	info->regs.ftime = DEFAULT_FLASHTIME;

	if (pcfg->max_peak_current_mA > pcap->max_peak_curr_mA ||
		!pcfg->max_peak_current_mA) {
		dev_notice(info->dev,
			"max_peak_current_mA of %d invalid changing to %d\n",
			pcfg->max_peak_current_mA, pcap->max_peak_curr_mA);
		pcfg->max_peak_current_mA = pcap->max_peak_curr_mA;
	}

	info->led_num = 1;
	if (!pcfg->synchronized_led && pcap->led2_support &&
		(info->led_mask & 3) == 3)
		info->led_num = 2;

	pqry->version = NVC_TORCH_CAPABILITY_VER_1;
	pqry->flash_num = info->led_num;
	pqry->torch_num = info->led_num;
	pqry->led_attr = 0;

	val = pcfg->max_peak_current_mA * info->led_num;

	if (!pcfg->max_total_current_mA || pcfg->max_total_current_mA > val)
		pcfg->max_total_current_mA = val;
	pcfg->max_peak_current_mA =
		info->config.max_total_current_mA / info->led_num;

	if (pcfg->max_sustained_current_mA > pcap->max_assist_curr_mA ||
		!pcfg->max_sustained_current_mA) {
		dev_notice(info->dev,
			"max_sustained_current_mA of %d invalid"
			"changing to %d\n",
			pcfg->max_sustained_current_mA,
			pcap->max_assist_curr_mA);
		pcfg->max_sustained_current_mA =
			pcap->max_assist_curr_mA;
	}
	if ((1000 * pcfg->min_current_mA) < pcap->curr_step_uA) {
		pcfg->min_current_mA = pcap->curr_step_uA / 1000;
		dev_notice(info->dev,
			"min_current_mA lower than possible, increasing to %d\n",
			pcfg->min_current_mA);
	}
	if (pcfg->min_current_mA > pcap->max_indicator_curr_mA) {
		dev_notice(info->dev,
			"min_current_mA of %d higher than possible,"
			" reducing to %d",
			pcfg->min_current_mA, pcap->max_indicator_curr_mA);
		pcfg->min_current_mA =
			pcap->max_indicator_curr_mA;
	}

	for (i = 0; i < pqry->flash_num; i++) {
		pfcap = info->flash_cap[i];
		pfcap->version = NVC_TORCH_CAPABILITY_VER_1;
		pfcap->led_idx = i;
		pfcap->attribute = 0;
		pfcap->granularity = pcfg->led_config[i].granularity;
		pfcap->timeout_num = ARRAY_SIZE(lp3367_flash_timer);
		ptmcap = info->flash_timeouts[i];
		pfcap->timeout_off = (void *)ptmcap - (void *)pfcap;
		pfcap->flash_torch_ratio =
				pcfg->led_config[i].flash_torch_ratio;

		plvls = pcfg->led_config[i].lumi_levels;
		pfcap->levels[0].guidenum = LP3367_LEVEL_OFF;
		pfcap->levels[0].luminance = 0;
		for (j = 1; j < pcfg->led_config[i].flash_levels + 1; j++) {
			if (GET_CURRENT_BY_INDEX(info, plvls[j - 1].guidenum) >
				pcfg->max_peak_current_mA)
				break;

			pfcap->levels[j].guidenum = plvls[j - 1].guidenum;
			pfcap->levels[j].luminance = plvls[j - 1].luminance;
			info->max_flash[i] = plvls[j - 1].guidenum;
			dev_dbg(info->dev, "%03d - %d\n",
				pfcap->levels[j].guidenum,
				pfcap->levels[j].luminance);
		}
		pfcap->numberoflevels = j;
		dev_dbg(info->dev,
			"%s flash#%d, attr: %x, levels: %d, g: %d, ratio: %d\n",
			__func__, pfcap->led_idx, pfcap->attribute,
			pfcap->numberoflevels, pfcap->granularity,
			pfcap->flash_torch_ratio);

		ptmcap->timeout_num = pfcap->timeout_num;
		for (j = 0; j < ptmcap->timeout_num; j++) {
			ptmcap->timeouts[j].timeout = lp3367_flash_timer[j];
			dev_dbg(info->dev, "t: %03d - %d uS\n", j,
				ptmcap->timeouts[j].timeout);
		}
	}

	for (i = 0; i < pqry->torch_num; i++) {
		ptcap = info->torch_cap[i];
		ptcap->version = NVC_TORCH_CAPABILITY_VER_1;
		ptcap->led_idx = i;
		ptcap->attribute = 0;
		ptcap->granularity = pcfg->led_config[i].granularity;
		ptcap->timeout_num = ARRAY_SIZE(lp3367_torch_timer);
		ptmcap = info->torch_timeouts[i];
		ptcap->timeout_off = (void *)ptmcap - (void *)ptcap;

		plvls = pcfg->led_config[i].lumi_levels;
		ptcap->levels[0].guidenum = LP3367_LEVEL_OFF;
		ptcap->levels[0].luminance = 0;
		for (j = 1; j < pcfg->led_config[i].flash_levels + 1; j++) {
			if (GET_CURRENT_BY_INDEX(info, plvls[j - 1].guidenum) >
				pcfg->max_torch_current_mA)
				break;

			ptcap->levels[j].guidenum = plvls[j - 1].guidenum;
			ptcap->levels[j].luminance = plvls[j - 1].luminance;
			info->max_torch[i] = plvls[j - 1].guidenum;
			dev_dbg(info->dev, "%03d - %d\n",
				ptcap->levels[j].guidenum,
				ptcap->levels[j].luminance);
		}
		ptcap->numberoflevels = j;
		if (ptcap->numberoflevels > LP3367_MAX_TORCH_LEVEL)
			ptcap->numberoflevels = LP3367_MAX_TORCH_LEVEL;
		dev_dbg(info->dev, "torch#%d, attr: %x, levels: %d, g: %d\n",
			ptcap->led_idx, ptcap->attribute,
			ptcap->numberoflevels, ptcap->granularity);

		ptmcap->timeout_num = ptcap->timeout_num;
		for (j = 0; j < ptmcap->timeout_num; j++) {
			ptmcap->timeouts[j].timeout = lp3367_torch_timer[j];
			dev_dbg(info->dev, "t: %03d - %d uS\n", j,
				ptmcap->timeouts[j].timeout);
		}
	}

	if (update && (info->pwr_state == NVC_PWR_COMM ||
			info->pwr_state == NVC_PWR_ON))
		return lp3367_update_settings(info);

	return 0;
}

static int lp3367_strobe(struct lp3367_info *info, int t_on)
{
	u32 gpio = info->pdata->gpio_strobe & 0xffff;
	u32 lact = (info->pdata->gpio_strobe & 0xffff0000) ? 1 : 0;
	return gpio_direction_output(gpio, lact ^ (t_on & 1));
}

#if 0
#ifdef CONFIG_PM
static int lp3367_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct lp3367_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "Suspending %s\n", info->dev_cap->name);

	return 0;
}

static int lp3367_resume(struct i2c_client *client)
{
	struct lp3367_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "Resuming %s\n", info->dev_cap->name);

	return 0;
}

static void lp3367_shutdown(struct i2c_client *client)
{
	struct lp3367_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "Shutting down %s\n", info->dev_cap->name);

	lp3367_set_leds(info, 3, 0, 0);
}
#endif
#endif

static int lp3367_power_on(struct lp3367_info *info)
{
	struct lp3367_power_rail *power = &info->power;
	int err = 0;

	if (info->power_on)
		return 0;

	mutex_lock(&info->mutex);
	if (power->v_in) {
		/*err = regulator_enable(power->v_in);*/
		if (err) {
			dev_err(info->dev, "%s v_in err\n", __func__);
			goto power_on_end;
		}
	}

	if (power->v_i2c) {
		/*err = regulator_enable(power->v_i2c);*/
		if (err) {
			dev_err(info->dev, "%s v_i2c err\n", __func__);
			if (power->v_in)
				regulator_disable(power->v_in);
			goto power_on_end;
		}
	}

	if (info->pdata && info->pdata->power_on_callback)
		err = info->pdata->power_on_callback(&info->power);

	if (!err)
		info->power_on = 1;
#if 0
	lp3367_edp_lowest(info);
#endif
power_on_end:
	mutex_unlock(&info->mutex);

	if (!err) {
		usleep_range(100, 120);
		err = lp3367_update_settings(info);
	}
	return err;
}

static int lp3367_power_off(struct lp3367_info *info)
{
	struct lp3367_power_rail *power = &info->power;
	int err = 0;

	if (!info->power_on)
		return 0;

	mutex_lock(&info->mutex);
	if (info->pdata && info->pdata->power_off_callback)
		err = info->pdata->power_off_callback(&info->power);
	if (IS_ERR_VALUE(err))
		goto power_off_end;

	if (power->v_in) {
		/* err = regulator_disable(power->v_in); */
		if (err) {
			dev_err(info->dev, "%s vi_in err\n", __func__);
			goto power_off_end;
		}
	}

	if (power->v_i2c) {
		/* err = regulator_disable(power->v_i2c); */
		if (err)
			dev_err(info->dev, "%s vi_i2c err\n", __func__);
	}

	if (!err)
		info->power_on = 0;
#if 0
	lp3367_edp_lowest(info);
#endif
power_off_end:
	mutex_unlock(&info->mutex);
	return err;
}

static int lp3367_power(struct lp3367_info *info, int pwr)
{
	int err = 0;

	dev_dbg(info->dev, "%s %d %d\n", __func__, pwr, info->pwr_state);
	if (pwr == info->pwr_state) /* power state no change */
		return 0;

	switch (pwr) {
	case NVC_PWR_OFF:
		err = lp3367_set_leds(info, 3, 0, 0);
		if ((info->pdata->cfg & NVC_CFG_OFF2STDBY) ||
			     (info->pdata->cfg & NVC_CFG_BOOT_INIT))
			pwr = NVC_PWR_STDBY;
		else
			err |= lp3367_power_off(info);
		break;
	case NVC_PWR_STDBY_OFF:
		err = lp3367_set_leds(info, 3, 0, 0);
		if ((info->pdata->cfg & NVC_CFG_OFF2STDBY) ||
			     (info->pdata->cfg & NVC_CFG_BOOT_INIT))
			pwr = NVC_PWR_STDBY;
		else
			err |= lp3367_power_on(info);
		break;
	case NVC_PWR_STDBY:
		err = lp3367_power_on(info);
		err |= lp3367_set_leds(info, 3, 0, 0);
		break;
	case NVC_PWR_COMM:
	case NVC_PWR_ON:
		err = lp3367_power_on(info);
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err < 0) {
		dev_err(info->dev, "%s error\n", __func__);
		pwr = NVC_PWR_ERR;
	}
	info->pwr_state = pwr;
	if (err > 0)
		return 0;

	return err;
}

#if 0
static void lp3367_throttle(unsigned int new_state, void *priv_data)
{
	struct lp3367_info *info = priv_data;

	if (!info)
		return;

	lp3367_power(info, NVC_PWR_OFF);
}
#endif

static int lp3367_get_dev_id(struct lp3367_info *info)
{
	info->regs.dev_id = 0xb0;
	return 0;
#if 0
	int err;

	dev_dbg(info->dev, "%s %02x\n", __func__, info->regs.dev_id);
	/* ChipID[7:3] is a fixed identification B0 */
	if ((info->regs.dev_id & 0xb0) == 0xb0)
		return 0;

	if (NVC_PWR_OFF == info->pwr_state ||
		NVC_PWR_OFF_FORCE == info->pwr_state)
		lp3367_power_on(info);
	err = lp3367_reg_rd(info, LP3367_REG_CHIPID, &info->regs.dev_id);
	if (err)
		goto read_devid_exit;

	if ((info->regs.dev_id & 0xb0) != 0xb0)
		err = -ENODEV;

read_devid_exit:
	if (NVC_PWR_OFF == info->pwr_state)
		lp3367_power_off(info);
	return err;
#endif
}

static int lp3367_user_get_param(struct lp3367_info *info, long arg)
{
	struct nvc_param params;
	struct nvc_torch_pin_state pinstate;
	const void *data_ptr = NULL;
	u32 data_size = 0;
	int err = 0;
	u8 reg;

	if (copy_from_user(&params,
			(const void __user *)arg,
			sizeof(struct nvc_param))) {
		dev_err(info->dev,
			"%s %d copy_from_user err\n", __func__, __LINE__);
		return -EINVAL;
	}

	switch (params.param) {
	case NVC_PARAM_TORCH_QUERY:
		dev_dbg(info->dev, "%s QUERY\n", __func__);
		data_ptr = &info->query;
		data_size = sizeof(info->query);
		break;
	case NVC_PARAM_FLASH_EXT_CAPS:
		dev_dbg(info->dev, "%s EXT_FLASH_CAPS %d\n",
			__func__, params.variant);
		if (params.variant >= info->query.flash_num) {
			dev_err(info->dev, "%s unsupported flash index.\n",
				__func__);
			err = -EINVAL;
			break;
		}
		data_ptr = info->flash_cap[params.variant];
		data_size = info->flash_cap_size;
		break;
	case NVC_PARAM_TORCH_EXT_CAPS:
		dev_dbg(info->dev, "%s EXT_TORCH_CAPS %d\n",
			__func__, params.variant);
		if (params.variant >= info->query.torch_num) {
			dev_err(info->dev, "%s unsupported torch index.\n",
				__func__);
			err = -EINVAL;
			break;
		}
		data_ptr = info->torch_cap[params.variant];
		data_size = info->torch_cap_size;
		break;
	case NVC_PARAM_FLASH_LEVEL:
		if (params.variant >= info->query.flash_num) {
			dev_err(info->dev,
				"%s unsupported flash index.\n", __func__);
			err = -EINVAL;
			break;
		}
		if (info->op_mode != LP3367_REG_CONTROL_MODE_FLASH)
			reg = 0;
		else if (params.variant > 0)
			reg = info->regs.led2_curr;
		else
			reg = info->regs.led1_curr;
		data_ptr = &reg;
		data_size = sizeof(reg);
		dev_dbg(info->dev, "%s FLASH_LEVEL %d\n", __func__, reg);
		break;
	case NVC_PARAM_TORCH_LEVEL:
		if (params.variant >= info->query.torch_num) {
			dev_err(info->dev, "%s unsupported torch index.\n",
				__func__);
			err = -EINVAL;
			break;
		}
		if (info->op_mode != LP3367_REG_CONTROL_MODE_TORCH)
			reg = 0;
		else if (params.variant > 0)
			reg = info->regs.led2_curr;
		else
			reg = info->regs.led1_curr;
		data_ptr = &reg;
		data_size = sizeof(reg);
		dev_dbg(info->dev, "%s TORCH_LEVEL %d\n", __func__, reg);
		break;
	case NVC_PARAM_FLASH_PIN_STATE:
		/* By default use Active Pin State Setting */
		pinstate = info->pdata->pinstate;
		if (info->op_mode != LP3367_REG_CONTROL_MODE_FLASH)
		{
			pinstate.values ^= 0xffff; /* Inactive Pin Setting */
		}

		dev_dbg(info->dev, "%s FLASH_PIN_STATE: %x&%x\n",
			__func__, pinstate.mask, pinstate.values);
		data_ptr = &pinstate;
		data_size = sizeof(pinstate);
		break;
	default:
		dev_err(info->dev, "%s unsupported parameter: %d\n",
			__func__, params.param);
		err = -EINVAL;
	}

	if (!err && params.sizeofvalue < data_size) {
		dev_err(info->dev, "%s data size mismatch %d != %d\n",
			__func__, params.sizeofvalue, data_size);
		err = -EINVAL;
	}

	if (!err && copy_to_user((void __user *)params.p_value,
		data_ptr, data_size)) {
		dev_err(info->dev, "%s copy_to_user err line %d\n",
			__func__, __LINE__);
		err = -EFAULT;
	}

	return err;
}

static int lp3367_get_levels(struct lp3367_info *info,
			       struct nvc_param *params,
			       bool flash_mode,
			       struct nvc_torch_set_level_v1 *plevels)
{
	struct nvc_torch_timer_capabilities_v1 *p_tm;
	u8 op_mode;

	if (copy_from_user(plevels, (const void __user *)params->p_value,
			   sizeof(*plevels))) {
		dev_err(info->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	if (flash_mode) {
		dev_dbg(info->dev, "%s FLASH_LEVEL: %d %d %d\n",
			__func__, plevels->ledmask,
			plevels->levels[0], plevels->levels[1]);
		p_tm = info->flash_timeouts[0];
		op_mode = LP3367_REG_CONTROL_MODE_FLASH;
	} else {
		dev_dbg(info->dev, "%s TORCH_LEVEL: %d %d %d\n",
			__func__, plevels->ledmask,
			plevels->levels[0], plevels->levels[1]);
		p_tm = info->torch_timeouts[0];
		op_mode = LP3367_REG_CONTROL_MODE_TORCH;
	}

	if (plevels->timeout) {
		u16 i;
		for (i = 0; i < p_tm->timeout_num; i++) {
			plevels->timeout = i;
			if (plevels->timeout == p_tm->timeouts[i].timeout)
				break;
		}
	} else
		plevels->timeout = p_tm->timeout_num - 1;

	if (plevels->levels[0] == LP3367_LEVEL_OFF)
		plevels->ledmask &= ~1;
	if (plevels->levels[1] == LP3367_LEVEL_OFF)
		plevels->ledmask &= ~2;
	plevels->ledmask &= info->config.led_mask;

	if (!plevels->ledmask)
		info->op_mode = LP3367_REG_CONTROL_MODE_NONE;
	else {
		info->op_mode = op_mode;
		if (info->config.synchronized_led) {
			plevels->ledmask = 3;
			plevels->levels[1] = plevels->levels[0];
		}
	}

	dev_dbg(info->dev, "Return: %d - %d %d %d\n", info->op_mode,
		plevels->ledmask, plevels->levels[0], plevels->levels[1]);
	return 0;
}

static int lp3367_user_set_param(struct lp3367_info *info, long arg)
{
	struct nvc_param params;
	struct nvc_torch_set_level_v1 led_levels;
	int err = 0;
	u8 val;

	if (copy_from_user(
		&params, (const void __user *)arg, sizeof(struct nvc_param))) {
		dev_err(info->dev,
			"%s %d copy_from_user err\n", __func__, __LINE__);
		return -EINVAL;
	}

	switch (params.param) {
	case NVC_PARAM_FLASH_LEVEL:
		lp3367_get_levels(info, &params, true, &led_levels);
		if (led_levels.timeout == 0)
			info->regs.ftime = info->config.def_ftimer;
		else
			info->regs.ftime = led_levels.timeout;
		err = lp3367_set_leds(info, info->led_mask,
			led_levels.levels[0], led_levels.levels[1]);
		break;
	case NVC_PARAM_TORCH_LEVEL:
		lp3367_get_levels(info, &params, false, &led_levels);
		info->regs.ftime = led_levels.timeout;
		err = lp3367_set_leds(info, info->led_mask,
			led_levels.levels[0], led_levels.levels[1]);
		break;
	case NVC_PARAM_FLASH_PIN_STATE:
		if (copy_from_user(&val, (const void __user *)params.p_value,
			sizeof(val))) {
			dev_err(info->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
			err = -EINVAL;
			break;
		}
		dev_dbg(info->dev, "%s FLASH_PIN_STATE: %d\n", __func__, val);
		err = lp3367_strobe(info, val);
		break;
	default:
		dev_err(info->dev, "%s unsupported parameter: %d\n",
			__func__, params.param);
		err = -EINVAL;
		break;
	}

	return err;
}

static long lp3367_ioctl(struct file *file,
			   unsigned int cmd,
			   unsigned long arg)
{
	struct lp3367_info *info = file->private_data;
	int pwr;
	int err = 0;

	switch (cmd) {
	case NVC_IOCTL_PARAM_WR:
		err = lp3367_user_set_param(info, arg);
		break;
	case NVC_IOCTL_PARAM_RD:
		err = lp3367_user_get_param(info, arg);
		break;
	case NVC_IOCTL_PWR_WR:
		/* This is a Guaranteed Level of Service (GLOS) call */
		pwr = (int)arg * 2;
		dev_dbg(info->dev, "%s PWR_WR: %d\n", __func__, pwr);
		if (!pwr || (pwr > NVC_PWR_ON)) /* Invalid Power State */
			break;

		err = lp3367_power(info, pwr);

		if (info->pdata->cfg & NVC_CFG_NOERR)
			err = 0;
		break;
	case NVC_IOCTL_PWR_RD:
		pwr = info->pwr_state / 2;
		dev_dbg(info->dev, "%s PWR_RD: %d\n", __func__, pwr);
		if (copy_to_user((void __user *)arg, (const void *)&pwr,
				 sizeof(pwr))) {
			dev_err(info->dev, "%s copy_to_user err line %d\n",
				__func__, __LINE__);
			err = -EFAULT;
		}
		break;
	default:
		dev_err(info->dev, "%s unsupported ioctl: %x\n", __func__, cmd);
		err = -EINVAL;
		break;
	}

	return err;
}

static int lp3367_open(struct inode *inode, struct file *file)
{
	struct lp3367_info *info = NULL;
	struct lp3367_info *pos = NULL;

	rcu_read_lock();
	list_for_each_entry_rcu(pos, &lp3367_info_list, list) {
		if (pos->miscdev.minor == iminor(inode)) {
			info = pos;
			break;
		}
	}
	rcu_read_unlock();
	if (!info)
		return -ENODEV;

	if (atomic_xchg(&info->in_use, 1))
		return -EBUSY;

	file->private_data = info;
	dev_dbg(info->dev, "%s\n", __func__);
	return 0;
}

static int lp3367_release(struct inode *inode, struct file *file)
{
	struct lp3367_info *info = file->private_data;

	dev_dbg(info->dev, "%s\n", __func__);
	lp3367_power(info, NVC_PWR_OFF);
	file->private_data = NULL;
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	return 0;
}

static int lp3367_power_put(struct lp3367_power_rail *pw)
{
#if 0
	if (likely(pw->v_in))
		regulator_put(pw->v_in);

	if (likely(pw->v_i2c))
		regulator_put(pw->v_i2c);
#endif

	pw->v_in = NULL;
	pw->v_i2c = NULL;

	return 0;
}

#if 0
static int lp3367_regulator_get(struct lp3367_info *info,
	struct regulator **vreg, char vreg_name[])
{
	struct regulator *reg = NULL;
	int err = 0;

	reg = regulator_get(info->dev, vreg_name);
	if (unlikely(IS_ERR_OR_NULL(reg))) {
		dev_err(info->dev,
			"%s %s ERR: %d\n", __func__, vreg_name, (int)reg);
		err = PTR_ERR(reg);
		reg = NULL;
	} else
		dev_dbg(info->dev, "%s: %s\n", __func__, vreg_name);

	*vreg = reg;
	return err;
}
#endif

static int lp3367_power_get(struct lp3367_info *info)
{
#if 0
	struct lp3367_power_rail *pw = &info->power;

	lp3367_regulator_get(info, &pw->v_in, "vin"); /* 3.7v */
	lp3367_regulator_get(info, &pw->v_i2c, "vi2c"); /* 1.8v */
#endif
	info->pwr_state = NVC_PWR_OFF;

	return 0;
}

static void lp3367_gpio_init(struct lp3367_info *info)
{
	int err;

	err = gpio_request(info->pdata->flash_en, "flash_en");
	if (err)
		dev_err(info->dev,
				"%s: gpio flash_en request fail\n", __func__);
	gpio_direction_output(info->pdata->flash_en, 0);

	err = gpio_request(info->pdata->led_en, "led_en");
	if (err)
		dev_err(info->dev,
				"%s: gpio led_en request fail\n", __func__);
	gpio_direction_output(info->pdata->led_en, 0);

}

static const struct file_operations lp3367_fileops = {
	.owner = THIS_MODULE,
	.open = lp3367_open,
	.unlocked_ioctl = lp3367_ioctl,
	.release = lp3367_release,
};

static void lp3367_del(struct lp3367_info *info)
{
	lp3367_power(info, NVC_PWR_OFF);
	lp3367_power_put(&info->power);

	spin_lock(&lp3367_spinlock);
	list_del_rcu(&info->list);
	spin_unlock(&lp3367_spinlock);
	synchronize_rcu();
}

static int lp3367_remove(struct platform_device *pdev)
{
	struct lp3367_info *info = platform_get_drvdata(pdev);

	dev_dbg(info->dev, "%s\n", __func__);
	misc_deregister(&info->miscdev);
	lp3367_del(info);
	return 0;
}

static void lp3367_caps_layout(struct lp3367_info *info)
{
#define LP3367_FLASH_CAP_TIMEOUT_SIZE \
	(lp3367_flash_cap_size + lp3367_flash_timeout_size)
#define LP3367_TORCH_CAP_TIMEOUT_SIZE \
	(lp3367_torch_cap_size + lp3367_torch_timeout_size)
	void *start_ptr;
	int i;

	start_ptr = (void *)info + sizeof(*info);
	for (i = 0; i < 2; i++) {
		info->flash_cap[i] = start_ptr;
		info->flash_timeouts[i] = start_ptr + lp3367_flash_cap_size;
		start_ptr += LP3367_FLASH_CAP_TIMEOUT_SIZE;
	}
	info->flash_cap_size = LP3367_FLASH_CAP_TIMEOUT_SIZE;

	for (i = 0; i < 2; i++) {
		info->torch_cap[i] = start_ptr;
		info->torch_timeouts[i] = start_ptr + lp3367_torch_cap_size;
		start_ptr += LP3367_TORCH_CAP_TIMEOUT_SIZE;
	}
	info->torch_cap_size = LP3367_TORCH_CAP_TIMEOUT_SIZE;

	dev_dbg(info->dev, "%s: %d(%d + %d), %d(%d + %d)\n", __func__,
		info->flash_cap_size, lp3367_flash_cap_size,
		lp3367_flash_timeout_size, info->torch_cap_size,
		lp3367_torch_cap_size, lp3367_torch_timeout_size);
}

static int lp3367_probe(struct platform_device *pdev)
{
	struct lp3367_info *info;
	char dname[16];
	int err;

	dev_dbg(&pdev->dev, "%s\n", __func__);
	info = devm_kzalloc(&pdev->dev, sizeof(*info) +
			lp3367_max_flash_cap_size +
			lp3367_max_torch_cap_size,
			GFP_KERNEL);
	if (info == NULL) {
		dev_err(&pdev->dev, "%s: kzalloc error\n", __func__);
		return -ENOMEM;
	}

	info->pdev = pdev;
	info->dev = &pdev->dev;
	if (pdev->dev.platform_data)
		info->pdata = pdev->dev.platform_data;
	else {
		info->pdata = &lp3367_default_pdata;
		dev_dbg(&pdev->dev,
				"%s No platform data.  Using defaults.\n",
				__func__);
	}

	lp3367_caps_layout(info);
	info->dev_cap = &lp3367_caps[info->pdata->type];

	lp3367_config_init(info);

	info->op_mode = LP3367_REG_CONTROL_MODE_TORCH; /* torch mode */

	lp3367_configure(info, false);

	platform_set_drvdata(pdev, info);
	mutex_init(&info->mutex);
	INIT_LIST_HEAD(&info->list);
	spin_lock(&lp3367_spinlock);
	list_add_rcu(&info->list, &lp3367_info_list);
	spin_unlock(&lp3367_spinlock);

	lp3367_power_get(info);
	lp3367_gpio_init(info);
#if 0
	lp3367_edp_register(info);
#endif

	err = lp3367_get_dev_id(info);
	if (err < 0) {
		dev_err(&pdev->dev, "%s device not found\n", __func__);
		if (info->pdata->cfg & NVC_CFG_NODEV) {
			lp3367_del(info);
			return -ENODEV;
		}
	} else
		dev_info(&pdev->dev, "%s device %02x found\n",
			__func__, info->regs.dev_id);

	if (info->pdata->dev_name != 0)
		strcpy(dname, info->pdata->dev_name);
	else
		strcpy(dname, "lp3367");
	if (info->pdata->num)
		snprintf(dname, sizeof(dname), "%s.%u",
			 dname, info->pdata->num);

	info->miscdev.name = dname;
	info->miscdev.fops = &lp3367_fileops;
	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	if (misc_register(&info->miscdev)) {
		dev_err(&pdev->dev, "%s unable to register misc device %s\n",
				__func__, dname);
		lp3367_del(info);
		return -ENODEV;
	}

#if 0
	lp3367_debugfs_init(info);
#endif
	return 0;
}

#if 0
static int lp3367_status_show(struct seq_file *s, void *data)
{
	struct lp3367_info *k_info = s->private;
	struct lp3367_config *pcfg = &k_info->config;

	pr_info("%s\n", __func__);

	seq_printf(s, "lp3367 status:\n"
		"    Flash type: %s, bus %d, addr: 0x%02x\n\n"
		"    Led Mask         = %01x\n"
		"    Led1 Current     = 0x%02x\n"
		"    Led2 Current     = 0x%02x\n"
		"    Flash Mode       = 0x%02x\n"
		"    Flash TimeOut    = 0x%02x\n"
		"    Flash Strobe     = 0x%02x\n"
		"    Max_Peak_Current = 0x%04dmA\n"
		"    Use_TxMask       = 0x%02x\n"
		"    TxMask_Current   = 0x%04dmA\n"
		"    Freq_Switch_on   = %s\n"
		"    VIN_low_run      = 0x%04dmV\n"
		"    VIN_low          = 0x%04dmV\n"
		"    LedOff_On_VIN_low = %s\n"
		"    PinState Mask    = 0x%04x\n"
		"    PinState Values  = 0x%04x\n"
		,
		(char *)lp3367_id[k_info->pdata->type + 1].name,
		k_info->i2c_client->adapter->nr,
		k_info->i2c_client->addr,
		k_info->led_mask,
		k_info->regs.led1_curr,
		k_info->regs.led2_curr,
		k_info->op_mode, k_info->regs.ftime,
		pcfg->strobe_type,
		pcfg->max_peak_current_mA,
		pcfg->use_tx_mask,
		pcfg->txmasked_current_mA,
		pcfg->freq_switch_on ? "TRUE" : "FALSE",
		pcfg->vin_low_v_run_mV,
		pcfg->vin_low_v_mV,
		pcfg->led_off_when_vin_low ? "TRUE" : "FALSE",
		k_info->pdata->pinstate.mask,
		k_info->pdata->pinstate.values
	);

	return 0;
}

static ssize_t lp3367_attr_set(struct file *s,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct lp3367_info *k_info =
		((struct seq_file *)s->private_data)->private;
	char buf[24];
	int buf_size;
	u32 val = 0;

	pr_info("%s\n", __func__);

	if (!user_buf || count <= 1)
		return -EFAULT;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	if (sscanf(buf + 1, "0x%x", &val) == 1)
		goto set_attr;
	if (sscanf(buf + 1, "0X%x", &val) == 1)
		goto set_attr;
	if (sscanf(buf + 1, "%d", &val) == 1)
		goto set_attr;

	pr_info("SYNTAX ERROR: %s\n", buf);
	return -EFAULT;

set_attr:
	pr_info("new data = %x\n", val);
	switch (buf[0]) {
	/* enable/disable power */
	case 'p':
		if (val & 0xffff)
			lp3367_power(k_info, NVC_PWR_ON);
		else
			lp3367_power(k_info, NVC_PWR_OFF);
		break;
	/* enable/disable led 1/2 */
	case 'l':
		k_info->config.led_mask = val;
		lp3367_configure(k_info, false);
		break;
	/* change led 1/2 current settings */
	case 'c':
		lp3367_set_leds(k_info, k_info->led_mask,
			val & 0xff, (val >> 8) & 0xff);
		break;
	/* modify flash timeout reg */
	case 'f':
		k_info->regs.ftime = val;
		lp3367_set_leds(k_info, k_info->led_mask,
			k_info->regs.led1_curr,
			k_info->regs.led2_curr);
		break;
	/* set led work mode/trigger mode */
	case 'x':
		if (val & 0xf)
			k_info->config.strobe_type = (val & 0xf);
		k_info->op_mode = (val & 0xf0) >> 4;
		if (val & 0xf00)
			k_info->config.freq_switch_on =
				((val & 0xf00) == 0x200);
		if (val & 0xf000)
			k_info->config.led_off_when_vin_low =
				((val & 0xf000) == 0x2000);
		if (val & 0xf0000) {
			val = ((val & 0xf0000) >> 16) - 1;
			if (val >= LP3367_NUM) {
				pr_err("Invalid dev type %x\n", val);
				return -ENODEV;
			}
			k_info->pdata->type = val;
			k_info->dev_cap = &lp3367_caps[k_info->pdata->type];
		}
		lp3367_configure(k_info, true);
		break;
	/* change txmask/torch settings */
	case 't':
		k_info->config.use_tx_mask = (val >> 4) & 1;
		k_info->config.txmasked_current_mA = val & 0x0f;
		val = (val >> 8) & 0xffff;
		if (val)
			k_info->config.I_limit_mA = val;
		lp3367_set_txmask(k_info);
		break;
	/* change voltage low settings */
	case 'v':
		if (val & 0xffff)
			k_info->config.vin_low_v_run_mV = val & 0xffff;
		val >>= 16;
		if (val & 0xffff)
			k_info->config.vin_low_v_mV = val & 0xffff;
		lp3367_configure(k_info, true);
		break;
	/* set max_peak_current_mA */
	case 'k':
		if (val & 0xffff)
			k_info->config.max_peak_current_mA = val & 0xffff;
		lp3367_configure(k_info, true);
		break;
	/* change pinstate setting */
	case 'm':
		k_info->pdata->pinstate.mask = (val >> 16) & 0xffff;
		k_info->pdata->pinstate.values = val & 0xffff;
		break;
	/* trigger an external flash/torch event */
	case 'g':
		k_info->pdata->gpio_strobe = val;
		lp3367_strobe(k_info, 1);
		break;
	}

	return count;
}

static int lp3367_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, lp3367_status_show, inode->i_private);
}

static const struct file_operations lp3367_debugfs_fops = {
	.open = lp3367_debugfs_open,
	.read = seq_read,
	.write = lp3367_attr_set,
	.llseek = seq_lseek,
	.release = single_release,
};

static int lp3367_debugfs_init(struct lp3367_info *info)
{
	struct dentry *d;

	info->d_lp3367 = debugfs_create_dir(
		info->miscdev.this_device->kobj.name, NULL);
	if (info->d_lp3367 == NULL) {
		pr_info("%s: debugfs create dir failed\n", __func__);
		return -ENOMEM;
	}

	d = debugfs_create_file("d", S_IRUGO|S_IWUSR, info->d_lp3367,
		(void *)info, &lp3367_debugfs_fops);
	if (!d) {
		pr_info("%s: debugfs create file failed\n", __func__);
		debugfs_remove_recursive(info->d_lp3367);
		info->d_lp3367 = NULL;
	}

	return -EFAULT;
}
#endif

static struct platform_driver lp3367_driver = {
	.driver = {
		.name = "lp3367",
		.owner = THIS_MODULE,
	},
	.id_table = lp3367_id,
	.probe = lp3367_probe,
	.remove = lp3367_remove,
#if 0
#ifdef CONFIG_PM
	.shutdown = lp3367_shutdown,
	.suspend  = lp3367_suspend,
	.resume   = lp3367_resume,
#endif
#endif
};

static int __init lp3367_init_driver(void)
{
	return platform_driver_register(&lp3367_driver);
}

static void __exit lp3367_exit_driver(void)
{
	platform_driver_unregister(&lp3367_driver);
}

module_init(lp3367_init_driver);
module_exit(lp3367_exit_driver);

MODULE_DESCRIPTION("LP3367 flash/torch driver");
MODULE_AUTHOR("Charlie Huang <chahuang@nvidia.com>");
MODULE_LICENSE("GPL");
