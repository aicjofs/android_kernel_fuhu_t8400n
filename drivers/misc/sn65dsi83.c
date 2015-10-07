/*

 * keenhi sn65dsi83(T007) mipi bridge (SPI bus) - Android version
 *
 * Copyright (C) 2011-2012 keenhi Inc.
 *
 * Licensed under the GPL-2 or later.
 *
 * Version : 0.04
 */

//=============================================================================
//INCLUDED FILES
//=============================================================================
#include <linux/input.h>	// BUS_SPI
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/sched.h>	// wake_up_process()
#include <linux/kthread.h>	// kthread_create()、kthread_run()
#include <asm/uaccess.h>	// copy_to_user(),
#include <linux/miscdevice.h>
#include <asm/siginfo.h>	// siginfo
#include <linux/rcupdate.h>	// rcu_read_lock
#include <linux/sched.h>	// find_task_by_pid_type
#include <linux/syscalls.h>	// sys_clock_gettime()
#include <linux/module.h>	// sys_clock_gettime()
#include <linux/kernel.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include <linux/sn65dsi83.h>	
//=============================================================================
//DEFINITIONS
//=============================================================================

#define MAX_SPI_FREQ_HZ      50000000
#define TS_PEN_UP_TIMEOUT    msecs_to_jiffies(50)

#define UDELAY(n)   (udelay(n))

#define MDELAY(n)   (mdelay(n))

#define BIT(x) (1 << (x))
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define max_khz_rate(x) (x)*1000
#define max_mhz_rate(x) (x)*1000*1000

#define SCREEN_AUO_W  0x0300
#define SCREEN_AUO_H 0x0400

#define SCREEN_CPT_W  0x0556
#define SCREEN_CPT_H 0x0300
 //  
 // 750*1000
#define MAX_SPEED_RATE  2*1000*1000//100*1000 //1MHZ
#define DEBUG 1

#define BRIDGE_TABLE_END     0xFF /* special number to indicate this is end of table */
#define BRIDGE_WAIT_MS       0x00/* special number to indicate this is wait time require */
#define SENSOR_MAX_RETRIES   3 /* max counter for retry I2C access */

#define SN65DSI83_PLL_EN		BIT(0) 


#define KEENHI_DC_OUT_TYPE_DSI 0
#define KEENHI_DC_OUT_TYPE_RGB 1
#define KEENHI_DC_OUT_TYPE_SSD2828_AUO 2
#define DC_CTRL_TYPE      KEENHI_DC_OUT_TYPE_SSD2828_AUO
//=============================================================================
//STRUCTURE DECLARATION
//=============================================================================

//=============================================================================
//GLOBAL VARIABLES DECLARATION
//=============================================================================
struct sn65dsi83_info {
	struct i2c_client		*client;
	struct sn65dsi83_platform_data	plat_data;
	struct sn65dsi83_data *data;
	struct delayed_work		work;
	bool				gpio_detect;
	int				irq;
};


static struct bridge_reg mode_common[] = {
{0x09,0x00},
{0x0A,0x05},
{0x0B,0x00},
{0x0D,0x00},
{0x10,0x26},
{0x11,0x00},
{0x12,0x0D},//old 0x10 test 0x0d
{0x13,0x00},
{0x18,0xF0},//org 0x70
{0x19,0x00},
{0x1A,0x03},
{0x1B,0x00},
{0x20,0x20},
{0x21,0x03},
{0x22,0x00},
{0x23,0x00},
{0x24,0x00},
{0x25,0x00},
{0x26,0x00},
{0x27,0x00},
{0x28,0xe5},
{0x29,0x07},
{0x2A,0x00},
{0x2B,0x00},
{0x2C,0x40},
{0x2D,0x00},
{0x2E,0x00},
{0x2F,0x00},
{0x30,0x01},
{0x31,0x00},
{0x32,0x00},
{0x33,0x00},
{0x34,0x80},
{0x35,0x00},
{0x36,0x00},
{0x37,0x00},
{0x38,0x00},
{0x39,0x00},
{0x3A,0x00},
{0x3B,0x00},
{0x3C,0x00},
{0x3D,0x00},
{0x3E,0x00},
{0x0D,0x01},
{BRIDGE_TABLE_END,0x00},
};

static struct sn65dsi83_info *gsn65dsi83_bridge_info;


static irqreturn_t sn65dsi83_irq(int irq, void *devid)
{
	pr_err("%s:==============>EXE\n",__func__);
	return IRQ_HANDLED;
}

/*
static void sn65dsi83_delayed_work(struct work_struct *work)
{
	struct sn65dsi83_info *sn65dsi83_device;
	s32 ret;

	sn65dsi83_device = container_of(work, struct sn65dsi83_info, work.work);
}
*/
static int sensor_read_reg8_addr8(struct i2c_client *client, u8 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[2];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 1;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err<0)
		return -EINVAL;

        //swap(*(data+2),*(data+3)); //swap high and low byte to match table format
	memcpy(val, data+1, 1);

	return 0;
}

static int sensor_write_reg8_addr8(struct i2c_client *client, u8 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	u8 data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

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
		pr_err("gc0308 : i2c transfer failed, retrying %x %x %d\n",
		       addr, val, err);
		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

static int bridge_write_table(struct i2c_client *client,
			      const struct bridge_reg table[])
{
	const struct bridge_reg *next;
	int err=0;
    
	dev_info(&client->dev," %s: =========>exe\n",__func__);
       next = table ;       

	for (next = table; next->addr!= BRIDGE_TABLE_END; next++) {
	       err = sensor_write_reg8_addr8(client, next->addr,next->val);
		if (err){
			dev_err(&client->dev,"%s: write  0x%x failed\n", __func__,
				next->addr);
			return err;
		}
		switch (next->addr) {
			case BRIDGE_WAIT_MS:
			{
				dev_info(&client->dev,"%s:wait 0x%xms ...",__func__,next->val);
				mdelay(next->val);
				break;
			}
			default:break;
		}
	}
	return 0;
}


static int sn65dsi83_read_all_ID(struct i2c_client *client){
	u8 val=0x00,addr;
	for(addr=0x00;addr<0x09;addr++){
		sensor_read_reg8_addr8(client,addr,&val);
		dev_info(&client->dev,
			"%s:  sn65dsi83 0x%02x = 0x%02x\n", client->name,addr,val);
		val = 0x00;
	}
	
}
static void sn65dsi83_dump_regs(struct i2c_client *client,u8 addr){
	u8 val=0x00;
	sensor_read_reg8_addr8(client,addr,&val);
	dev_info(&client->dev,
		"%s:  sn65dsi83 0x%02x = 0x%02x\n", client->name,addr,val);
}
static int bridge_read_table(struct i2c_client *client,
			      const struct bridge_reg table[])
{
	const struct bridge_reg *next;
	int err=0;
    
	dev_info(&client->dev," %s: =========>exe start\n",__func__);
       next = table ;       

	for (next = table; next->addr!= BRIDGE_TABLE_END; next++) {
	       sn65dsi83_dump_regs(client, next->addr);
	}
	dev_info(&client->dev," %s: =========>exe end\n",__func__);
	return 0;
}

static int sn65dsi83_init_bridge(struct i2c_client *client){
	u8 val=0x00,addr=0x00;
	if(!client)
		dev_err(&client->dev," %s: =========>client is NULL\n",__func__);

	bridge_write_table(client,mode_common);
	bridge_read_table(client,mode_common);
#if 0
	addr = 0x0D;
	sensor_read_reg8_addr8(client,addr,&val);
	val &=0xFE;
	sensor_write_reg8_addr8(client,addr,val);
	val =0x00;

	sn65dsi83_dump_regs(client,addr);
	
	addr = 0x0A;
	sensor_read_reg8_addr8(client,addr,&val);
	val |=BIT(0);
	sensor_write_reg8_addr8(client,addr,val);
	val =0x00;

	sn65dsi83_dump_regs(client,addr);
	
	//addr = 0x0B;
	//sensor_read_reg8_addr8(client,addr,&val);
	//val &=0x1E;
	//sensor_write_reg8_addr8(client,addr,val);


	addr = 0x10;
	sensor_read_reg8_addr8(client,addr,&val);
	val &=0xE7;
	sensor_write_reg8_addr8(client,addr,val);
	val =0X00;
	sn65dsi83_dump_regs(client,addr);
	
	addr = 0x12;
	sensor_read_reg8_addr8(client,addr,&val);
	val =0x0B;
	sensor_write_reg8_addr8(client,addr,val);
	sn65dsi83_dump_regs(client,addr);
	//addr = 0x18;
	//sensor_read_reg8_addr8(client,addr,&val);
	//val |=BIT(3);
	//sensor_write_reg8_addr8(client,addr,val);

	addr = 0x0D;
	sensor_read_reg8_addr8(client,addr,&val);
	val |=SN65DSI83_PLL_EN;
	sensor_write_reg8_addr8(client,addr,val);
	val = 0x00;
	
	sn65dsi83_dump_regs(client,addr);
#endif
	
}
static int sn65dsi83_screen_on()
{
	pr_err("%s:==============>EXE\n",__func__);
	struct sn65dsi83_info *sn65dsi83_device = gsn65dsi83_bridge_info;
	if(sn65dsi83_device)
	sn65dsi83_init_bridge(sn65dsi83_device->client);
	return 0;
}
static int sn65dsi83_screen_off()
{
	pr_err("%s:==============>EXE\n",__func__);
	
	return 0;
}
static int __devinit sn65dsi83_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct sn65dsi83_data *data;
	struct sn65dsi83_info *sn65dsi83_device;
	struct sn65dsi83_platform_data *pdata = client->dev.platform_data;
	int irq = client->irq;
	int rc;
	pr_err("%s:====================>exe\n",__func__);
	if (!pdata) {
		dev_err(&client->dev, "no platform data and no register func for sn65dsi83 driver\n");
		return -EINVAL;
	}
	sn65dsi83_device = kzalloc(sizeof(struct sn65dsi83_info), GFP_KERNEL);
	if (!sn65dsi83_device){
		dev_err(&client->dev, "can't alloc mem for sn65dsi83_info!\n");
		return -ENOMEM;
	}
	gsn65dsi83_bridge_info = sn65dsi83_device;
	data = kzalloc(sizeof(struct sn65dsi83_data), GFP_KERNEL);
	if (!data) {
		rc = -ENOMEM;
		kfree(sn65dsi83_device);
		dev_err(&client->dev, "can't alloc mem for sn65dsi83_data!\n");
		return rc;
	}
	sn65dsi83_device->client = client;


	sn65dsi83_device->gpio_detect =
		gpio_is_valid(pdata->mipi_enable_gpio);
	memcpy(&sn65dsi83_device->plat_data, pdata, sizeof(struct sn65dsi83_platform_data));
	
	sn65dsi83_device->irq = irq;

	data->ssd_init_callback = sn65dsi83_screen_on;
	data->ssd_deinit_callback =sn65dsi83_screen_off;
	if(pdata->register_resume){
		pdata->register_resume(data);
		sn65dsi83_device->data = data;
	}else{
		kfree(data);
		sn65dsi83_device->data = NULL;
	}

	
	i2c_set_clientdata(client, sn65dsi83_device);




	if (!sn65dsi83_device->gpio_detect){
		dev_warn(&client->dev, "Failed to get gpio from pdata, so skip gpio set\n");
		goto skip_gpio;
	}

	rc = gpio_request(pdata->mipi_enable_gpio, "MIPI_EN");
	if (rc) {
		dev_warn(&client->dev, "Failed to request gpio: %d\n", rc);
		sn65dsi83_device->gpio_detect = false;
		goto skip_gpio;
	}

	gpio_direction_output(pdata->mipi_enable_gpio,1);

	sn65dsi83_read_all_ID(client);
	if(sn65dsi83_device->irq<=0){
		dev_warn(&client->dev, "Failed to get irq from pdata, so skip irq set\n");
		goto skip_gpio;
	}
	
	rc = request_irq(sn65dsi83_device->irq, sn65dsi83_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		dev_name(&client->dev), sn65dsi83_device);
	if (rc) {
		dev_warn(&client->dev, "Failed to request irq: %d\n", rc);
		goto skip_gpio;
	}

	
skip_gpio:
	//INIT_DELAYED_WORK(&sn65dsi83_device->work, sn65dsi83_delayed_work);
	dev_info(&client->dev,
		"%s:  sn65dsi83 device registered\n", client->name);
	return 0;


exit_mem_free:
	if (sn65dsi83_device->gpio_detect)
		gpio_free(pdata->mipi_enable_gpio);
	if (sn65dsi83_device->irq)
		free_irq(sn65dsi83_device->irq, sn65dsi83_device);
	kfree(data);
	kfree(sn65dsi83_device);

	return rc;
}

static int __devexit sn65dsi83_remove(struct i2c_client *client)
{
	struct sn65dsi83_info *sn65dsi83_device = i2c_get_clientdata(client);
	
	if (sn65dsi83_device->irq)
		free_irq(sn65dsi83_device->irq, sn65dsi83_device);
	if (sn65dsi83_device->gpio_detect)
		gpio_free(sn65dsi83_device->plat_data.mipi_enable_gpio);
	//cancel_delayed_work_sync(&sn65dsi83_device->work);

	if(sn65dsi83_device->data)
		kfree(sn65dsi83_device->data);
	kfree(sn65dsi83_device);
	sn65dsi83_device = NULL;

	return 0;
}

#if defined CONFIG_PM
static int sn65dsi83_suspend(struct i2c_client *client,
	pm_message_t state)
{
	//struct sn65dsi83_info *sn65dsi83_device = i2c_get_clientdata(client);
	//s32 ret;



	return 0;
}

static int sn65dsi83_resume(struct i2c_client *client)
{
	//struct sn65dsi83_info *sn65dsi83_device = i2c_get_clientdata(client);


	return 0;
}
#else
#define sn65dsi83_suspend		NULL
#define sn65dsi83_resume		NULL
#endif
#if 0
static struct sn65dsi83_platform_data sn65dsi83_bridge_pdata = {
	.register_resume = NULL,						\
	.mipi_enable_gpio = 120,						\
};

static struct i2c_board_info __initdata sn65dsi83_i2c0_board_info[] = {

	{ I2C_BOARD_INFO("sn65dsi83", KIONIX_ACCEL_I2C_ADDR),
		.platform_data = &sn65dsi83_bridge_pdata,
		.irq = 0, // Replace with appropriate GPIO setup
	},
};
#endif
static const struct i2c_device_id sn65dsi83_id[] = {
	{ SN65DSI83_DEV_NAME, 0 },
	{}
};

static struct i2c_driver sn65dsi83_bridge_driver = {
	.probe		= sn65dsi83_probe,
	.remove		= __devexit_p(sn65dsi83_remove),
	.suspend	= sn65dsi83_suspend,
	.resume		= sn65dsi83_resume,
	.id_table	= sn65dsi83_id,
	.driver = {
		.name	= "sn65dsi83-bridge",
	},
};

static int __init sn65dsi83_bridge_init(void)
{
	return i2c_add_driver(&sn65dsi83_bridge_driver);
}
//module_init(sn65dsi83_bridge_init);
subsys_initcall(sn65dsi83_bridge_init);
static void __exit sn65dsi83_bridge_exit(void)
{
	i2c_del_driver(&sn65dsi83_bridge_driver);
}
module_exit(sn65dsi83_bridge_exit);

MODULE_AUTHOR("Tang Yunlong<jimmy.tang@keenhi.com>");
MODULE_DESCRIPTION("sn65dsi83 bridge i2c bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:jimmy-t007");
