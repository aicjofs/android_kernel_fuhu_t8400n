/*
 * drivers/video/tegra/dc/sn65dsi86_dsi2edp.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/swab.h>
#include <linux/module.h>
#include <mach/dc.h>
#include "dc_priv.h"
#include "sn65dsi86_dsi2edp.h"
#include "dsi.h"

static struct tegra_dc_dsi2edp_data *sn65dsi86_dsi2edp;
static struct i2c_client *sn65dsi86_i2c_client;
#define DEBUG 0
#define BRIDGE_TABLE_END 0xFF

enum i2c_transfer_type {
	I2C_WRITE,
	I2C_READ,
};


static struct bridge_reg mode_common[] = {
{0x09,0x00},
{0x0A,0x05},
{0x0D,0x00},
{0x10,0x26},
{0x11,0x00},
{0x12,0x0D},//old 0x10 test 0x0d
{0x13,0x00},
{0x20,0x20},
{0x21,0x03},
{0x22,0x00},
{0x23,0x00},
{0x24,0x00},
{0x25,0x00},
{0x2C,0x40},
{0x2D,0x00},
{0x30,0x01},
{0x31,0x00},
{0x34,0x80},
{0x36,0x00},
{0x38,0x00},
{0x3A,0x00},
{0x3C,0x00},
{0x3D,0x00},
{0x3E,0x00},
{BRIDGE_TABLE_END,0x00},
};


/*
 * TC358770 requires register address in big endian
 * and register value in little endian.
 * Regmap currently sends it all in big endian.
*/
#define TO_LITTLE_ENDIAN	(false)
#define DSI2EDP_REG_VAL(addr, val)	{(addr), (val)}
#define BRIDGE_TABLE_END 0xFF
static u8 dsi2edp_disable_config_clk[][2] = {
	DSI2EDP_REG_VAL(0x0d, 0x00), /* pLL disable */
};

static u8 dsi2edp_config_init1[][2] = {
	DSI2EDP_REG_VAL(0x0a,0x06), 
	DSI2EDP_REG_VAL(0x10,0x26), // Single 4 DSI lanes
	DSI2EDP_REG_VAL(0x12,0x54), //DSIA CLK FREQ 422.5MHz
	DSI2EDP_REG_VAL(0x5a, 0x05),//enhanced framing and ASSR
	DSI2EDP_REG_VAL(0x93,0x20),// 2 DP lanes no SSC
	DSI2EDP_REG_VAL(0x94,0x80),//HBR bit rate 2.7Gbps
	DSI2EDP_REG_VAL(0x0d, 0x01), /* pLL enable */
};
static u8 dsi2edp_config_init2[][2] = {
	//read 0x0a Verify PLL is locked
	DSI2EDP_REG_VAL(0x95, 0x00), //POST-Cursor2 0dB
	//======Write DPCD Register 0x0010A in Sink to Enable ASSR======
	DSI2EDP_REG_VAL(0x64, 0x01), 
	DSI2EDP_REG_VAL(0x74, 0x00),
	DSI2EDP_REG_VAL(0x75, 0x01), 
	DSI2EDP_REG_VAL(0x76, 0x0a), 
	DSI2EDP_REG_VAL(0x77, 0x01), 
	DSI2EDP_REG_VAL(0x78, 0x81), 
	
	DSI2EDP_REG_VAL(0x96, 0x0a), //Semi-Auto TRAIN 
};


static u8 dsi2edp_config_init3[][2] = {
	DSI2EDP_REG_VAL(0x20, 0x80),//H_ACTIVE_L = 
	DSI2EDP_REG_VAL(0x21, 0x07),//H_ACTIVE_H = 
	DSI2EDP_REG_VAL(0x24, 0x38),//V_ACTIVE_L =
	DSI2EDP_REG_VAL(0x25, 0x04),//V_ACTIVE_H =
	
	DSI2EDP_REG_VAL(0x2c, 0x1c),// HPW = 16
	DSI2EDP_REG_VAL(0x2d, 0x80),
	
	DSI2EDP_REG_VAL(0x30, 0x05),//VPW = 14
	DSI2EDP_REG_VAL(0x31, 0x80),
	
	DSI2EDP_REG_VAL(0x34, 0x94),//HBP = 148
	DSI2EDP_REG_VAL(0x36, 0x17),//VBP = 19
	DSI2EDP_REG_VAL(0x38, 0x0c),// HFP = 16
	DSI2EDP_REG_VAL(0x3a, 0x0a),//VFP = 3
	DSI2EDP_REG_VAL(0x5b,0x00),//DP- 24bpp
	DSI2EDP_REG_VAL(0x3c, 0x00),//COLOR BAR disabled
	DSI2EDP_REG_VAL(0x5a, 0x0d),//enhanced framing, ASSR, and Vstream enable
};

static inline void sn65dsi86_reg_write(struct tegra_dc_dsi2edp_data *dsi2edp,
					unsigned int addr, unsigned int val)
{
	regmap_write(dsi2edp->regmap, addr,
		TO_LITTLE_ENDIAN ? __swab32(val) : val);
}

static inline void sn65dsi86_reg_read(struct tegra_dc_dsi2edp_data *dsi2edp,
					unsigned int addr, unsigned int *val)
{
	regmap_read(dsi2edp->regmap, addr, val);
	*val = TO_LITTLE_ENDIAN ? __swab32(*val) : *val;
}

static const struct regmap_config sn65dsi86_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int dsi2ledp_i2c_transfer(struct tegra_dc_dsi2edp_data *dsi2edp,
					u8 transfers[][2], u32 no_of_tranfers,
					enum i2c_transfer_type type)
{
	struct i2c_msg *i2c_msg_transfer;
	struct i2c_client *client = dsi2edp->client_i2c;
	int err = 0,count = 1;
	int delay_time = 10;
	u32 cnt = 0;

	i2c_msg_transfer = kzalloc
		(no_of_tranfers * sizeof(*i2c_msg_transfer), GFP_KERNEL);
	if (!i2c_msg_transfer)
		return -ENOMEM;

	for (cnt = 0; cnt < no_of_tranfers; cnt++) {
		i2c_msg_transfer[cnt].addr = client->addr;
		i2c_msg_transfer[cnt].flags = type;
		i2c_msg_transfer[cnt].len = 2;
		i2c_msg_transfer[cnt].buf = transfers[cnt];
	}

	for (cnt = 0; cnt < no_of_tranfers; cnt++) {
		
		err = i2c_transfer(client->adapter, &i2c_msg_transfer[cnt], count);
		if (err < 0) {
			dev_err(&dsi2edp->dsi->dc->ndev->dev,
				"dsi2edp: i2c write failed\n");
			break;
		}
		if(DEBUG)
		printk("%s:==========>addr = %02x,val = %02x\n",__func__,transfers[cnt][0],transfers[cnt][1]);
		msleep(delay_time);
	}

	kfree(i2c_msg_transfer);
	return err;
}

static void sn65dsi86_dump_regs(struct tegra_dc_dsi2edp_data *dsi2edp,u8 addr){
	unsigned int val=0;
	sn65dsi86_reg_read(dsi2edp,addr,&val);
	printk(
		"%s:  sn65dsi83 0x%02x = 0x%x\n",__func__,addr,val);
}
static int bridge_read_table(struct tegra_dc_dsi2edp_data *dsi2edp,
			      const struct bridge_reg table[])
{
	const struct bridge_reg *next;
       next = table ;       

	for (next = table; next->addr!= BRIDGE_TABLE_END; next++) {
	       sn65dsi86_dump_regs(dsi2edp, next->addr);
	}
	return 0;
}
static int sn65dsi86_dsi2edp_init(struct tegra_dc_dsi_data *dsi)
{
	int err = 0;
	
	if (sn65dsi86_dsi2edp) {
		tegra_dsi_set_outdata(dsi, sn65dsi86_dsi2edp);
		return err;
	}

	sn65dsi86_dsi2edp = devm_kzalloc(&dsi->dc->ndev->dev,
					sizeof(*sn65dsi86_dsi2edp),
					GFP_KERNEL);
	if (!sn65dsi86_dsi2edp)
		return -ENOMEM;

	sn65dsi86_dsi2edp->dsi = dsi;

	sn65dsi86_dsi2edp->client_i2c = sn65dsi86_i2c_client;

	sn65dsi86_dsi2edp->regmap = devm_regmap_init_i2c(sn65dsi86_i2c_client,
						&sn65dsi86_regmap_config);
	if (IS_ERR(sn65dsi86_dsi2edp->regmap)) {
		err = PTR_ERR(sn65dsi86_dsi2edp->regmap);
		dev_err(&dsi->dc->ndev->dev,
				"sn65dsi86_dsi2edp: regmap init failed\n");
		goto fail;
	}

	sn65dsi86_dsi2edp->mode = &dsi->dc->mode;

	tegra_dsi_set_outdata(dsi, sn65dsi86_dsi2edp);

	mutex_init(&sn65dsi86_dsi2edp->lock);

fail:
	return err;
}

static void sn65dsi86_dsi2edp_destroy(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp =
				tegra_dsi_get_outdata(dsi);

	if (!dsi2edp)
		return;

	sn65dsi86_dsi2edp = NULL;
	mutex_destroy(&dsi2edp->lock);
}

static void sn65dsi86_dsi2edp_enable(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);
	unsigned int val=0;
	int err = 0;

	if (dsi2edp && dsi2edp->dsi2edp_enabled)
		return;
	mutex_lock(&dsi2edp->lock);
	
	err = dsi2ledp_i2c_transfer(dsi2edp, dsi2edp_config_init1,
			ARRAY_SIZE(dsi2edp_config_init1), I2C_WRITE);
	if (err < 0) {
		dev_err(&dsi->dc->ndev->dev, "dsi2edp: Init 1 failed\n");
		goto fail;
	}
	
	sn65dsi86_reg_read(dsi2edp,0x0a,&val);//Verify PLL is locked
	dev_info(&dsi->dc->ndev->dev,"%s:=========>Verify PLL is locked state=%x\n",__func__,val);
	 
	err = dsi2ledp_i2c_transfer(dsi2edp, dsi2edp_config_init2,
			ARRAY_SIZE(dsi2edp_config_init2), I2C_WRITE);
	if (err < 0) {
		dev_err(&dsi->dc->ndev->dev, "dsi2edp: Init 2 failed\n");
		goto fail;
	}
	sn65dsi86_reg_read(dsi2edp,0x96,&val);//Verify PLL is locked
	dev_info(&dsi->dc->ndev->dev,"%s:=========>Verify Training state=%x\n",__func__,val);
	
	err = dsi2ledp_i2c_transfer(dsi2edp, dsi2edp_config_init3,
			ARRAY_SIZE(dsi2edp_config_init3), I2C_WRITE);
	if (err < 0) {
		dev_err(&dsi->dc->ndev->dev, "dsi2edp: Init 3 failed\n");
		goto fail;
	}

	dsi2edp->dsi2edp_enabled = true;
	if(DEBUG)
	bridge_read_table(dsi2edp,mode_common);
fail:
	pr_warn("%s-------------enable end\n",__func__);
	mutex_unlock(&dsi2edp->lock);
}

static void sn65dsi86_dsi2edp_disable(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);
	int err = 0;

	if (dsi2edp && !dsi2edp->dsi2edp_enabled){
		dev_info(&dsi->dc->ndev->dev,"dsi2edp: dsi have disable!\n"); 
		return;
	}

	mutex_lock(&dsi2edp->lock);
	err = dsi2ledp_i2c_transfer(dsi2edp, dsi2edp_disable_config_clk,
			ARRAY_SIZE(dsi2edp_disable_config_clk), I2C_WRITE);
	if (err < 0) {
		dev_err(&dsi->dc->ndev->dev, "dsi2edp: disable clk failed\n");
	}
	dsi2edp->dsi2edp_enabled = false;
	mutex_unlock(&dsi2edp->lock);
}

#ifdef CONFIG_PM
static void sn65dsi86_dsi2edp_suspend(struct tegra_dc_dsi_data *dsi)
{
	/* To be done */
	struct tegra_dc_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);

	if (dsi2edp)
	dsi2edp->dsi2edp_enabled = false;
}

static void sn65dsi86_dsi2edp_resume(struct tegra_dc_dsi_data *dsi)
{
	/* To be done */
}
#endif

struct tegra_dsi_out_ops tegra_dsi2edp_ops = {
	.init = sn65dsi86_dsi2edp_init,
	.destroy = sn65dsi86_dsi2edp_destroy,
	.enable = sn65dsi86_dsi2edp_enable,
	.disable = sn65dsi86_dsi2edp_disable,
#ifdef CONFIG_PM
	.suspend = sn65dsi86_dsi2edp_suspend,
	.resume = sn65dsi86_dsi2edp_resume,
#endif
};

static int sn65dsi86_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	sn65dsi86_i2c_client = client;
	return 0;
}

static int sn65dsi86_i2c_remove(struct i2c_client *client)
{
	sn65dsi86_i2c_client = NULL;

	return 0;
}

static const struct i2c_device_id sn65dsi86_id_table[] = {
	{"sn65dsi86_dsi2edp", 0},
	{},
};

static struct i2c_driver sn65dsi86_i2c_drv = {
	.driver = {
		.name = "sn65dsi86_dsi2edp_bridge",
		.owner = THIS_MODULE,
	},
	.probe = sn65dsi86_i2c_probe,
	.remove = sn65dsi86_i2c_remove,
	.id_table = sn65dsi86_id_table,
};

static int __init sn65dsi86_i2c_client_init(void)
{
	int err = 0;

	err = i2c_add_driver(&sn65dsi86_i2c_drv);
	if (err)
		pr_err("sn65dsi86_dsi2edp: Failed to add i2c client driver\n");

	return err;
}

static void __exit sn65dsi86_i2c_client_exit(void)
{
	i2c_del_driver(&sn65dsi86_i2c_drv);
}

subsys_initcall(sn65dsi86_i2c_client_init);
module_exit(sn65dsi86_i2c_client_exit);

MODULE_LICENSE("GPL");
