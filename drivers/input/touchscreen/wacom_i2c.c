/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011 - 2013 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/unaligned.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/wacom_i2c.h>
#include <linux/workqueue.h>
#include <linux/wacom_i2c_firmware.h>
#include <linux/wacom_i2c_firmware_12.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>


#define WACOM_CMD_QUERY0	0x04
#define WACOM_CMD_QUERY1	0x00
#define WACOM_CMD_QUERY2	0x33
#define WACOM_CMD_QUERY3	0x02
#define WACOM_CMD_THROW0	0x05
#define WACOM_CMD_THROW1	0x00
#define WACOM_QUERY_SIZE	19
#define WACOM_CMD_SLEEP0	0x04
#define WACOM_CMD_SLEEP1	0x00
#define WACOM_CMD_SLEEP2	0x01
#define WACOM_CMD_SLEEP3	0x08

#define DISPALY1_RESOLUTION_XRES 1200
#define DISPALY1_RESOLUTION_YRES 1920

#define DISPALY2_RESOLUTION_XRES 1280
#define DISPALY2_RESOLUTION_YRES 800

#define DISPALY3_RESOLUTION_XRES 1920
#define DISPALY3_RESOLUTION_YRES 1080


#define DEBUG_PLUG 1
#if DEBUG_PLUG
#define debug_plug_print(...) printk(__VA_ARGS__)
#else
#define debug_plug_print(...) 
#endif

//static DEFINE_SPINLOCK(wacom_spinlock);

struct wacom_features {
	int x_max;
	int y_max;
	int pressure_max;
	char fw_version;
};
struct wacom_i2c *g_wac_i2c;

struct wacom_i2c {
	struct wacom_platform_data	*pdata;
	struct i2c_client *client;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	struct wacom_features features;
	u8 input_data[10];
	u8 data[WACOM_QUERY_SIZE];
	bool prox;
	int tool;
	unsigned int timer_debounce;
};
static struct i2c_client *wacom_client=NULL;

static int wacom_query_device(struct i2c_client *client,
			      struct wacom_features *features)
{
	int ret;
	u8 cmd1[] = { WACOM_CMD_QUERY0, WACOM_CMD_QUERY1,
			WACOM_CMD_QUERY2, WACOM_CMD_QUERY3 };
	u8 cmd2[] = { WACOM_CMD_THROW0, WACOM_CMD_THROW1 };
	u8 data[WACOM_QUERY_SIZE];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd1),
			.buf = cmd1,
		},
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd2),
			.buf = cmd2,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(data),
			.buf = data,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	features->x_max = get_unaligned_le16(&data[3]);
	features->y_max = get_unaligned_le16(&data[5]);
	features->pressure_max = get_unaligned_le16(&data[11]);
	features->fw_version = get_unaligned_le16(&data[13]);

	dev_dbg(&client->dev,
		"--> x_max:%d, y_max:%d, pressure:%d, fw:%d\n",
		features->x_max, features->y_max,
		features->pressure_max, features->fw_version);

	return 0;
}

static int wacom_i2c_set_sleep(struct i2c_client *client)
{
	int ret;
	u8 cmd1[] = { WACOM_CMD_SLEEP0, WACOM_CMD_SLEEP1,
			WACOM_CMD_SLEEP2, WACOM_CMD_SLEEP3 };
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd1),
			.buf = cmd1,
		},
	};
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		printk("wacom i2c write error = %d\n",ret);
		return ret;
	}
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;
	
	return 0;
}

static int wacom_i2c_set_on(struct i2c_client *client)
{
	int ret;
	u8 cmd1[] = { WACOM_CMD_SLEEP0, WACOM_CMD_SLEEP1,
			WACOM_CMD_SLEEP1, WACOM_CMD_SLEEP3 };
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd1),
			.buf = cmd1,
		},
	};
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		printk("wacom i2c write error = %d\n",ret);
		return ret;
	}
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;
	
	return 0;
}

void cw2015_irq_displayed(void)
{
	struct input_dev *input = g_wac_i2c->input;
	int displayed = 0;
	
	if(g_wac_i2c&&g_wac_i2c->pdata&&g_wac_i2c->pdata->display_enable_gpio){
		displayed = (gpio_get_value_cansleep(g_wac_i2c->pdata->display_enable_gpio) ? 0 : 1)^g_wac_i2c->pdata->active_state;			
		if(!displayed){
			input_event(input, EV_KEY, KEY_POWER, 1);
			input_event(input, EV_KEY, KEY_POWER, 0);
			input_sync(input);
		}
	}
}
EXPORT_SYMBOL(cw2015_irq_displayed);

static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct wacom_i2c *wac_i2c =
		container_of(work, struct wacom_i2c, work);
	struct input_dev *input = wac_i2c->input;
	int displayed = 0,pluged = 0;
	
	if(wac_i2c&&wac_i2c->pdata&&wac_i2c->pdata->display_enable_gpio){
		pluged = gpio_get_value_cansleep(wac_i2c->pdata->plug_detect_gpio) ? 0 : 1;
		displayed = (gpio_get_value_cansleep(wac_i2c->pdata->display_enable_gpio) ? 0 : 1)^wac_i2c->pdata->active_state;
		debug_plug_print("%s:==========>pluged = %d,displayed = %d\n",__func__,pluged,displayed);
		if(pluged)
			wacom_i2c_set_sleep(wac_i2c->client);
		else
			wacom_i2c_set_on(wac_i2c->client);
			
		if(!pluged&&!displayed){
			input_event(input, EV_KEY, KEY_POWER, 1);
			input_event(input, EV_KEY, KEY_POWER, 0);
			input_sync(input);
		}
	}
}
static void gpio_keys_gpio_timer(unsigned long _data)
{
	struct wacom_i2c *wac_i2c = (struct wacom_i2c *)_data;

	schedule_work(&wac_i2c->work);
}
static irqreturn_t wacom_pluged_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	debug_plug_print("%s:======>timer_debounce=%dms\n",__func__,wac_i2c->timer_debounce);
	if (wac_i2c->timer_debounce)
		mod_timer(&wac_i2c->timer,
			jiffies + msecs_to_jiffies(wac_i2c->timer_debounce));
	else
		schedule_work(&wac_i2c->work);
	return IRQ_HANDLED;
}

static irqreturn_t wacom_i2c_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	struct input_dev *input = wac_i2c->input;
	u8 *data = wac_i2c->input_data;
	unsigned int x, y, pressure;
	unsigned char tsw, f1, f2, ers;
	int dis_xres = DISPALY2_RESOLUTION_XRES,dis_yres = DISPALY2_RESOLUTION_YRES;
	int error;
	int tmp;

	disable_irq_nosync(irq);
	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->input_data, sizeof(wac_i2c->input_data));
	if (error < 0)
		goto out;

	if(wac_i2c->pdata){
		dis_xres = wac_i2c->pdata->display_xres;
		dis_yres = wac_i2c->pdata->display_yres;
	}

	tsw = data[3] & 0x01;
	ers = data[3] & 0x04;
	f1 = data[3] & 0x02;
	f2 = data[3] & 0x10;
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);
	pressure = le16_to_cpup((__le16 *)&data[8]);

	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & 0x0c) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = data[3] & 0x20;
	//printk("========dis_xres=%d,dis_yres=%d=====x:%d,y=%d\n",dis_xres,dis_yres,x,y);
	if(dis_xres == DISPALY1_RESOLUTION_XRES&&dis_yres == DISPALY1_RESOLUTION_YRES){
	tmp = x;
	x = (x*dis_xres)/10764;
	y = (y*dis_yres)/17222;

	}else if(dis_xres == DISPALY3_RESOLUTION_XRES&&dis_yres == DISPALY3_RESOLUTION_YRES){
	tmp = x;
	x = (x*dis_xres)/25632;
	y = (y*dis_yres)/14418;
	x=dis_xres-x;
	y=dis_yres-y;
	}else{
	tmp = x;
	x = y;
	y = tmp;
	x = (x*dis_xres)/17222;
	y = (y*dis_yres)/10764;
	y = dis_yres - y;
	}
	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_key(input, wac_i2c->tool, wac_i2c->prox);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
	input_report_abs(input, ABS_PRESSURE, pressure);
	input_sync(input);
	
out:
	enable_irq(irq);
	return IRQ_HANDLED;
}

static int wacom_i2c_open(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	enable_irq(client->irq);

	return 0;
}

static void wacom_i2c_close(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	disable_irq(client->irq);
}

static void wacom_i2c_enable(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	int pluged = 0;

	if(wac_i2c) {
		pluged = gpio_get_value(wac_i2c->pdata->plug_detect_gpio) ? 0 : 1;
		if(pluged)
			wacom_i2c_set_sleep(wac_i2c->client);
		else
			wacom_i2c_set_on(wac_i2c->client);
	}
	printk("-->%s,	pluged = %d\n",__func__,pluged);
}

static void wacom_i2c_disable(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	int pluged = 0;

	if(wac_i2c) {
		wacom_i2c_set_sleep(wac_i2c->client);
	}
	printk("-->%s\n",__func__);
}

//------------------------wacom fw update----------------------------------
int wac_flash_cmd(struct i2c_client *client)
{
	int ret, len;
	unsigned char buf[10] = {0};

	len = 0;
	buf[len++] = 4;
	buf[len++] = 0;
	buf[len++] = 0x32;
	buf[len++] = CMD_SET_FEATURE;
	struct i2c_msg msgs=
			{
				.addr = client->addr,
				.flags = 0,
				.len = len,
				.buf = buf,
			};
	ret = i2c_transfer(client->adapter, &msgs,1);

	//ret = write(fd, buf, len);
	if (ret < 0)
		return -1;

	len = 0;
	buf[len++] = 5;
	buf[len++] = 0;
	buf[len++] = 4;
	buf[len++] = 0;
	buf[len++] = 2;
	buf[len++] = 2;

	msgs.len=len;
	ret = i2c_transfer(client->adapter, &msgs,1);

	//ret = write(fd, buf, len);
	if (ret < 0)
		return -1;

	printk("flash cmd sent:%d\n", ret);
	msleep(500);

	return 0;
}

int wac_flash_query(struct i2c_client *client)
{
	int ret, ECH;
	unsigned int  len;
	unsigned char command[CMD_SIZE] = {0};
	unsigned char response[RSP_SIZE] = {0};
	unsigned char buf[4] = {0};	
	struct i2c_msg msgs=
				{
					.addr = client->addr,
					.flags = 0,
					.len = len,
					.buf = buf,
				};


	len = 0;
	buf[len++] = 4;					/* Command Register-LSB */
	buf[len++] = 0;					/* Command Register-MSB */
	buf[len++] = 0x37;				/* Command-LSB, ReportType:Feature(11) ReportID:7 */
	buf[len++] = CMD_SET_FEATURE;	                /* Command-MSB, SET_REPORT */

	msgs.len=len;
	msgs.buf=buf;
	//printf("%s started \n", __func__);
	//ret = write(fd, buf, len);
	ret = i2c_transfer(client->adapter, &msgs,1);
	if (ret < 0) {
		printk("%s 1 ret:%d \n", __func__, ret);
		return -ERR;
	}		

	len = 0;
	command[len++] = 5;						/* Data Register-LSB */
	command[len++] = 0;						/* Data-Register-MSB */
	command[len++] = 5;							/* Length Field-LSB */
	command[len++] = 0;							/* Length Field-MSB */
	command[len++] = BOOT_CMD_REPORT_ID;	/* Report:ReportID */
	command[len++] = BOOT_QUERY;					/* Report:Boot Query command */
	command[len++] = ECH = 7;							/* Report:echo */
	msgs.len=len;
	msgs.buf=command;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, command, len);
	if (ret < 0) {
		printk("%s 2 ret:%d \n", __func__, ret);
		return -ERR;
	}	

	len = 0;
	buf[len++] = 4;					/* Command Register-LSB */
	buf[len++] = 0;					/* Command Register-MSB */
	buf[len++] = 0x38;				/* Command-LSB, ReportType:Feature(11) ReportID:8 */
	buf[len++] = CMD_GET_FEATURE;	/* Command-MSB, GET_REPORT */

	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);

	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 3 ret:%d \n", __func__, ret);
		return -ERR;
	}

	len = 0;
	buf[len++] = 5;					/* Data Register-LSB */
	buf[len++] = 0;					/* Data Register-MSB */

	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 4 ret:%d \n", __func__, ret);
		return -ERR;
	}

	msgs.flags=I2C_M_RD;
	msgs.buf=response;
	msgs.len=RSP_SIZE;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = read(fd, response, BOOT_RSP_SIZE);
	if (ret < 0) {
		printk("%s 5 ret:%d \n", __func__, ret);
		return -ERR;
	}	
	
	//printk("response[%s]=%d,%d,%d,%d,%d,%d\n",__func__,response[0],response[1],response[2],response[3],response[4],response[5]);
	if ( (response[3] != QUERY_CMD) ||
	     (response[4] != ECH) ) {
		printk("%s res3:%d res4:%d \n", __func__, response[3], response[4]);
		return -ERR;
	}
	if (response[5] != QUERY_RSP) {
		printk("%s res5:%d \n", __func__, response[5]);
		return -ERR;
	}
	
	return 0;
}

int wac_flash_blver(struct i2c_client *client, int *bootloader_ver)
{
	int ret, ECH;
	unsigned int len;
	unsigned char command[CMD_SIZE] = {0};
	unsigned char response[RSP_SIZE] = {0};
	unsigned char buf[4] = {0};
	struct i2c_msg msgs=
				{
					.addr = client->addr,
					.flags = 0,
					.len = len,
					.buf = buf,
				};

	len = 0;
	buf[len++] = 4;					/* Command Register-LSB */
	buf[len++] = 0;					/* Command Register-MSB */
	buf[len++] = 0x37;				/* Command-LSB, ReportType:Feature(11) ReportID:7 */
	buf[len++] = CMD_SET_FEATURE;	/* Command-MSB, SET_REPORT */
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 1 ret:%d \n", __func__, ret);
		return -ERR;
	}	

	len = 0;
	command[len++] = 5;						/* Data Register-LSB */
	command[len++] = 0;						/* Data-Register-MSB */
	command[len++] = 5;							/* Length Field-LSB */
	command[len++] = 0;							/* Length Field-MSB */
	command[len++] = BOOT_CMD_REPORT_ID;	/* Report:ReportID */
	command[len++] = BOOT_BLVER;					/* Report:Boot Version command */
	command[len++] = ECH = 7;							/* Report:echo */
	msgs.len=len;
	msgs.buf=command;
	ret = i2c_transfer(client->adapter, &msgs,1);

	//ret = write(fd, command, len);
	if (ret < 0) {
		printk("%s 2 ret:%d \n", __func__, ret);
		return -ERR;
	}	

	msleep(100);
	
	len = 0;
	buf[len++] = 4;					/* Command Register-LSB */
	buf[len++] = 0;					/* Command Register-MSB */
	buf[len++] = 0x38;				/* Command-LSB, ReportType:Feature(11) ReportID:8 */
	buf[len++] = CMD_GET_FEATURE;	/* Command-MSB, GET_REPORT */
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 3 ret:%d \n", __func__, ret);
		return -ERR;
	}

	len = 0;
	buf[len++] = 5;					/* Data Register-LSB */
	buf[len++] = 0;					/* Data Register-MSB */
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 4 ret:%d \n", __func__, ret);
		return -ERR;
	}

	msleep(100);
	
	msgs.flags=I2C_M_RD;
	msgs.buf=response;
	msgs.len=RSP_SIZE;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = read(fd, response, BOOT_RSP_SIZE);
	if (ret < 0) {
		printk("%s 5 ret:%d \n", __func__, ret);
		return -ERR;
	}	
	
	//printk("response[%s]=%d,%d,%d,%d,%d,%d\n",__func__,response[0],response[1],response[2],response[3],response[4],response[5]);
	if ((response[3] != BOOT_CMD) ||
		(response[4] != ECH))
		return -ERR;
	
	*bootloader_ver = (int)response[5];
	
	return 0;
}

int wac_flash_mputype(struct i2c_client *client, int *mpu_type)
{
	int ret, ECH;
	unsigned int len;
	unsigned char command[CMD_SIZE] = {0};
	unsigned char response[RSP_SIZE] = {0};
	unsigned char buf[4] = {0};		
        struct i2c_msg msgs=
			{
				.addr = client->addr,
				.flags = 0,
				.len = len,
				.buf = buf,
			};
	len = 0;
	buf[len++] = 4;					/* Command Register-LSB */
	buf[len++] = 0;					/* Command Register-MSB */
	buf[len++] = 0x37;				/* Command-LSB, ReportType:Feature(11) ReportID:7 */
	buf[len++] = CMD_SET_FEATURE;	/* Command-MSB, SET_REPORT */
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
   	if (ret < 0) {
		printk("%s 1 ret:%d \n", __func__, ret);
		return -ERR;
	}
	
	len = 0;
	command[len++] = 5;						/* Data Register-LSB */
	command[len++] = 0;						/* Data-Register-MSB */
	command[len++] = 5;							/* Length Field-LSB */
	command[len++] = 0;							/* Length Field-MSB */
	command[len++] = BOOT_CMD_REPORT_ID;	/* Report:ReportID */
	command[len++] = BOOT_MPU;					/* Report:Boot Query command */
	command[len++] = ECH = 7;							/* Report:echo */
	msgs.len=len;
	msgs.buf=command;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, command, len);
	if (ret < 0) {
		printk("%s 2 ret:%d \n", __func__, ret);
		return -ERR;
	}	

	msleep(100);

	len = 0;
	buf[len++] = 4;					/* Command Register-LSB */
	buf[len++] = 0;					/* Command Register-MSB */
	buf[len++] = 0x38;				/* Command-LSB, ReportType:Feature(11) ReportID:8 */
	buf[len++] = CMD_GET_FEATURE;	/* Command-MSB, GET_REPORT */
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 3 ret:%d \n", __func__, ret);
		return -ERR;
	}

	len = 0;
	buf[len++] = 5;					/* Data Register-LSB */
	buf[len++] = 0;					/* Data Register-MSB */
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 4 ret:%d \n", __func__, ret);
		return -ERR;
	}

	msleep(100);
	
	msgs.flags=I2C_M_RD;
	msgs.buf=response;
	msgs.len=RSP_SIZE;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = read(fd, response, BOOT_RSP_SIZE);
	if (ret < 0) {
		printk("%s 5 ret:%d \n", __func__, ret);
		return -ERR;
	}
	
	//printk("response[%s]=%d,%d,%d,%d,%d,%d\n",__func__,response[0],response[1],response[2],response[3],response[4],response[5]);
	if ((response[3] != MPU_CMD) ||
		(response[4] != ECH))
		return -ERR;
	
	*mpu_type = (int)response[5];
	
	return 0;
}

int get_bootloader_ver(struct i2c_client *client, int *bootloader_ver)
{
	int ret;
	
	ret = wac_flash_blver(client, bootloader_ver);
	if (ret < 0)
		return -1;

	return 0;
}

int get_mpu_type(struct i2c_client *client, int *mpu_type)
{
	int ret;
	
	ret = wac_flash_mputype(client, mpu_type);
	if (ret < 0)
		return -1;

	return 0;
}

bool wac_flash_security_unlock(struct i2c_client *client)
{
	int len, ret, ECH;	
	unsigned char command[CMD_SIZE] = {0};
	unsigned char response[RSP_SIZE] = {0};
	unsigned char buf[4] = {0};	
	struct i2c_msg msgs=
				{
					.addr = client->addr,
					.flags = 0,
					.len = len,
					.buf = buf,
				};

	len = 0;
	buf[len++] = 4;
	buf[len++] = 0;
	buf[len++] = 0x37;
	buf[len++] = CMD_SET_FEATURE;
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 1 ret: %d \n", __func__, ret);
		return false;
	}
		
	len = 0;
	command[len++] = 5;
	command[len++] = 0;
	command[len++] = 5;
	command[len++] = 0;
	command[len++] = BOOT_CMD_REPORT_ID;
	command[len++] = BOOT_SECURITY_UNLOCK;
	command[len++] = ECH = 7;
	msgs.len=len;
	msgs.buf=command;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, command, len);
	if (ret < 0) {
		printk("%s 2 rv:%d \n", __func__, ret);
		return false;
	}	

	msleep(10);

	len = 0;
	buf[len++] = 4;
	buf[len++] = 0;
	buf[len++] = 0x38;
	buf[len++] = CMD_GET_FEATURE;
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 3 ret:%d \n", __func__, ret);
		return false;
	}

	len = 0;
	buf[len++] = 5;
	buf[len++] = 0;
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 4 ret:%d \n", __func__, ret);
		return false;
	}

	msleep(1);

	msgs.flags=I2C_M_RD;
	msgs.buf=response;
	msgs.len=RSP_SIZE;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = read(fd, response, BOOT_RSP_SIZE);
	if (ret < 0) {
		printk("%s 5 ret:%d \n", __func__, ret);
		return false;
	}	
	
	//printk("response[%s]=%d,%d,%d,%d,%d,%d\n",__func__,response[0],response[1],response[2],response[3],response[4],response[5]);
	if ((response[3] != SEC_CMD) ||
		(response[4] != ECH))
		return false;

	printk("%s status: %x, OK \n", __func__, response[5]);	
	
	return true;
}

bool wac_flash_erase(struct i2c_client *client, int *erase_block, int num)
{
	int len, ret, ECH;
	int i,j;
	unsigned char sum;
	unsigned char buf[72] = {0};
	unsigned char cmd_chksum;
	unsigned char command[CMD_SIZE] = {0};
	unsigned char response[RSP_SIZE] = {0};
	struct i2c_msg msgs=
				{
					.addr = client->addr,
					.flags = 0,
					.len = 0,
					.buf = NULL,
				};
	for (i = 0; i < num; i++) {
retry:
		len = 0;
		buf[len++] = 4;					/* Command Register-LSB */
		buf[len++] = 0;					/* Command Register-MSB */
		buf[len++] = 0x37;				/* Command-LSB, ReportType:Feature(11) ReportID:7 */
		buf[len++] = CMD_SET_FEATURE;	/* Command-MSB, SET_REPORT */
		msgs.flags=0;
		msgs.len=len;
		msgs.buf=buf;
		ret = i2c_transfer(client->adapter, &msgs,1);
		//ret = write(fd, buf, len);
		if (ret < 0) {
			printk("%s failing 1:%d \n", __func__, i);
			return false;
		}
		
		len = 0;
		command[len++] = 5;						/* Data Register-LSB */
		command[len++] = 0;						/* Data-Register-MSB */
		command[len++] = 7;						/* Length Field-LSB */
		command[len++] = 0;						/* Length Field-MSB */
		command[len++] = BOOT_CMD_REPORT_ID;	/* Report:ReportID */
		command[len++] = BOOT_ERASE_FLASH;			/* Report:erase command */
		command[len++] = ECH = i;						/* Report:echo */
		command[len++] = *erase_block;				/* Report:erased block No. */
		erase_block++;

		sum = 0;
		for (j = 0; j < 8; j++)
			sum += command[j];
		cmd_chksum = ~sum+1;					/* Report:check sum */
		command[8] = cmd_chksum;
		len++;
		msgs.len=len;
		msgs.buf=command;
		ret = i2c_transfer(client->adapter, &msgs,1);
		//ret = write(fd, command, len);
		if (ret < 0) {
			printk("%s failing 2:%d \n", __func__, i);
			return false;
		}

		msleep(300);
	
		len = 0;
		buf[len++] = 4;					/* Command Register-LSB */
		buf[len++] = 0;					/* Command Register-MSB */
		buf[len++] = 0x38;				/* Command-LSB, ReportType:Feature(11) ReportID:8 */
		buf[len++] = CMD_GET_FEATURE;	/* Command-MSB, GET_REPORT */
		msgs.len=len;
		msgs.buf=buf;
		ret = i2c_transfer(client->adapter, &msgs,1);
		//ret = write(fd, buf, len);
		if (ret < 0) {
			printk("%s failing 3:%d \n", __func__, i);
			return false;
		}

		len = 0;
		buf[len++] = 5;					/* Data Register-LSB */
		buf[len++] = 0;					/* Data Register-MSB */
		msgs.len=len;
		msgs.buf=buf;
		ret = i2c_transfer(client->adapter, &msgs,1);
		//ret = write(fd, buf, len);
		if (ret < 0) {
			printk("%s failing 4:%d \n", __func__, i);
			return false;
		}
		msgs.flags=I2C_M_RD;
		msgs.buf=response;
		msgs.len=BOOT_RSP_SIZE;
		ret = i2c_transfer(client->adapter, &msgs,1);
		//ret = read(fd, response, BOOT_RSP_SIZE);
		if (ret < 0) {
			printk("%s failing 5:%d \n", __func__, i);
			return false;
		}
		//printk("response[%d]=%d,%d,%d,%d,%d,%d\n",i,response[0],response[1],response[2],response[3],response[4],response[5]);
		if ((response[3] != ERS_CMD) ||
		    (response[4] != ECH)) {
			printk("%s failing 6:%d \n", __func__, i);
			return false;
		}

		if (response[5] == 0x80) {
			printk("%s retry \n", __func__);
			goto retry;
		}
		if (response[5] != ACK) {
			printk("%s failing 7:%d res5:%d \n", __func__, i, response[5]);
			return false;
		}
	}

	return true;
}


bool flash_write_block(struct i2c_client *client, char *flash_data, unsigned long ulAddress, unsigned char *pcommand_id)
{
	const int MAX_COM_SIZE = (16 + FLASH_BLOCK_SIZE + 2);
	int len, ECH;
	int ret;
	unsigned char buf[300] = {0};
	unsigned char sum;
	unsigned char command[MAX_COM_SIZE];
	unsigned char response[RSP_SIZE] = {0};
	unsigned int i;
	struct i2c_msg msgs=
					{
						.addr = client->addr,
						.flags = 0,
						.len = 0,
						.buf = NULL,
					};

	len = 0;
	buf[len++] = 4;					/* Command Register-LSB */
	buf[len++] = 0;					/* Command Register-MSB */
	buf[len++] = 0x37;				/* Command-LSB, ReportType:Feature(11) ReportID:7 */
	buf[len++] = CMD_SET_FEATURE;	                /* Command-MSB, SET_REPORT */
	buf[len++] = 5;						/* Data Register-LSB */
	buf[len++] = 0;						/* Data-Register-MSB */
	buf[len++] = 76;					/* Length Field-LSB */
	buf[len++] = 0;						/* Length Field-MSB */
	buf[len++] = BOOT_CMD_REPORT_ID;	                /* Report:ReportID */
	buf[len++] = BOOT_WRITE_FLASH;			        /* Report:program  command */
	buf[len++] = ECH = ++(*pcommand_id);		        /* Report:echo */
	buf[len++] = ulAddress & 0x000000ff;
	buf[len++] = (ulAddress & 0x0000ff00) >> 8;
	buf[len++] = (ulAddress & 0x00ff0000) >> 16;
	buf[len++] = (ulAddress & 0xff000000) >> 24;			/* Report:address(4bytes) */
	buf[len++] = 8;						/* Report:size(8*8=64) */

	sum = 0;
	for (i = 4; i < 16; i++)
		sum += buf[i];
	buf[MAX_COM_SIZE - 2] = ~sum+1;					/* Report:command checksum */
	len++;

	sum = 0;
	for (i = 16; i < (FLASH_BLOCK_SIZE + 16); i++){
		buf[i] = flash_data[ulAddress+(i-16)];
		sum += flash_data[ulAddress+(i-16)];
		len++;
	}
	buf[MAX_COM_SIZE - 1] = ~sum+1;				/* Report:data checksum */
	len++;
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	//rv = wacom_i2c_master_send(wac_i2c->client, buf, (BOOT_CMD_SIZE + 4), WACOM_I2C_FLASH);
	if (ret < 0) {
		printk("%s 1 ret:%d \n", __func__, ret);
		return false;
	}

	msleep(10);

	len = 0;
	buf[len++] = 4;					/* Command Register-LSB */
	buf[len++] = 0;					/* Command Register-MSB */
	buf[len++] = 0x38;				/* Command-LSB, ReportType:Feature(11) ReportID:8 */
	buf[len++] = CMD_GET_FEATURE;	                /* Command-MSB, GET_REPORT */
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 2 ret:%d \n", __func__, ret);
		return false;
	}
	
	len = 0;
	buf[len++] = 5;					/* Data Register-LSB */
	buf[len++] = 0;					/* Data Register-MSB */
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 3 ret:%d \n", __func__, ret);
		return false;
	}
	msgs.flags=I2C_M_RD;
	msgs.buf=response;
	msgs.len=RSP_SIZE;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = read(fd, response, BOOT_RSP_SIZE);
	if (ret < 0) {
		printk("%s 4 ret:%d \n", __func__, ret);
		return false;
	}
	
	//printk("response[%s]=%d,%d,%d,%d,%d,%d\n",__func__,response[0],response[1],response[2],response[3],response[4],response[5]);
	if ((response[3] != WRITE_CMD) ||
		(response[4] != ECH) ||
		response[5] != ACK)
		return false;
	
	return true;
}

bool wac_flash_write(struct i2c_client *client, unsigned char *flash_data, size_t data_size, unsigned long start_address,
		     unsigned long *max_address)
{
	unsigned long ulAddress;
	unsigned long pageNo = 0;
	unsigned char  command_id = 0;
	int i;
	bool bRet;

	for (ulAddress = start_address; ulAddress < *max_address; ulAddress += FLASH_BLOCK_SIZE) {
		unsigned int j;
		bool bWrite = false;
	   
		for (i = 0; i < FLASH_BLOCK_SIZE; i++) {
			if (flash_data[ulAddress + i] != 0xFF)
				break;
		}
		if (i == (FLASH_BLOCK_SIZE))
			continue;

		for (j = 0; j < FLASH_BLOCK_SIZE; j++) {
			if (flash_data[ulAddress + j] == 0xFF)
				continue;
			else {
				bWrite = true;
				break;
			}
		}
			
		if (!bWrite) {
			pageNo++;
			continue;
		}
			
		bRet = flash_write_block(client, flash_data, ulAddress, &command_id);
		if(!bRet)
			return false;
			
		pageNo++;
	}
	
	return true;
}

bool wac_flash_marking(struct i2c_client *client, size_t data_size, bool bMarking)
{
	const int MAX_CMD_SIZE = 12 + FLASH_BLOCK_SIZE + 2;
	int len ,ret, ECH;
	int i, j;
	unsigned char flash_data[FLASH_BLOCK_SIZE] = {0};
	unsigned char buf[300] = {0};
	unsigned char response[RSP_SIZE] = {0};
	unsigned char sum;
	unsigned char command[MAX_CMD_SIZE];
	struct i2c_msg msgs=
					{
						.addr = client->addr,
						.flags = 0,
						.len = 0,
						.buf = NULL,
					};

	for (i = 0; i < FLASH_BLOCK_SIZE; i++) {
		flash_data[i]=0xff;
	}
		
	if (bMarking) {
			flash_data[MARKER] = 0x00;
	}
		
	len = 0;
	buf[len++] = 4;					/* Command Register-LSB */
	buf[len++] = 0;					/* Command Register-MSB */
	buf[len++] = 0x37;				/* Command-LSB, ReportType:Feature(11) ReportID:7 */
	buf[len++] = CMD_SET_FEATURE;	/* Command-MSB, SET_REPORT */
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);	
	if (ret < 0) {
		printk("%s 1 rv:%d \n", __func__, ret);
		return false;
	}
				
	len = 0;
	command[len++] = 5;						/* Data Register-LSB */
	command[len++] = 0;						/* Data-Register-MSB */
	command[len++] = 76;						/* Length Field-LSB */
	command[len++] = 0;						/* Length Field-MSB */
	command[len++] = BOOT_CMD_REPORT_ID;	/* Report:ReportID */
	command[len++] = BOOT_WRITE_FLASH;			/* Report:program  command */
	command[len++] = ECH = 1;						/* Report:echo */
	command[len++] = 0xC0;
	command[len++] = 0x1F;
	command[len++] = 0x01;
	command[len++] = 0x00;				/* Report:address(4bytes) */
	command[len++] = 8;						/* Report:size(8*8=64) */

	sum = 0;
	for (j = 0; j < 12; j++)
		sum += command[j];
	command[MAX_CMD_SIZE - 2] = ~sum+1;

	sum = 0;
	for (i = 12; i < (FLASH_BLOCK_SIZE + 12); i++){
		command[i] = flash_data[i-12];
		sum += flash_data[i-12];
	}
	command[MAX_CMD_SIZE - 1] = ~sum+1;				/* Report:data checksum */
	len++;
	msgs.len=BOOT_CMD_SIZE;
	msgs.buf=command;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, command, BOOT_CMD_SIZE);
	if (ret < 0) {
		printk("%s 2 ret:%d \n", __func__, ret);
		return false;
	}
		
	msleep(10);
		
	len = 0;
	buf[len++] = 4;
	buf[len++] = 0;
	buf[len++] = 0x38;
	buf[len++] = CMD_GET_FEATURE;
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 3 ret:%d \n", __func__, ret);
		return false;
	}		

	len = 0;
	buf[len++] = 5;
	buf[len++] = 0;
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 4 ret:%d \n", __func__, ret);
		return false;
	}
	msgs.flags=I2C_M_RD;
	msgs.buf=response;
	msgs.len=BOOT_RSP_SIZE;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = read(fd, response, BOOT_RSP_SIZE);
	if (ret < 0)
		return false;
	
	//printk("response[%s]=%d,%d,%d,%d,%d,%d\n",__func__,response[0],response[1],response[2],response[3],response[4],response[5]);
	if ((response[3] != 1) ||
	    (response[4] != ECH) ||
	    (response[5] != ACK) ) {
		printk("%s failing res3:%d res4:%d res5:%d \n", __func__, response[3], response[4], response[5]);
		return false;
	}
	
	return true;
}

bool wac_flash_verify(struct i2c_client *client, unsigned char *flash_data, size_t data_size, unsigned long start_address,
		      unsigned long *max_address)
{
	int ret, ECH;
	unsigned long ulAddress;
	unsigned long pageNo = 0;
	unsigned char command_id = 0;
	struct i2c_msg msgs=
					{
						.addr = client->addr,
						.flags = 0,
						.len = 0,
						.buf = NULL,
					};
	const int MAX_CMD_SIZE = 12 + FLASH_BLOCK_SIZE + 2;
	unsigned char buf[300] = {0};
	unsigned char sum;
	unsigned char command[MAX_CMD_SIZE];
	unsigned char response[RSP_SIZE] = {0};
	unsigned int i, j;
	int len;

	printk("%s verify starts \n", __func__);
	for (ulAddress = start_address; ulAddress < *max_address; ulAddress += FLASH_BLOCK_SIZE) {
		

		len = 0;
		buf[len++] = 4;
		buf[len++] = 0;
		buf[len++] = 0x37;
		buf[len++] = CMD_SET_FEATURE;
		msgs.flags=0;
		msgs.len=len;
		msgs.buf=buf;
		ret = i2c_transfer(client->adapter, &msgs,1);
		//ret = write(fd, buf, len);
		if (ret < 0) {
			printk("%s 1 ret:%d \n", __func__, ret);
			return false;
		}
		
		command[0] = 5;
		command[1] = 0;
		command[2] = 76;
		command[3] = 0;
		command[4] = BOOT_CMD_REPORT_ID;
		command[5] = BOOT_VERIFY_FLASH;
		command[6] = ECH = ++command_id;;
		command[7] = ulAddress&0x000000ff;
		command[8] = (ulAddress&0x0000ff00) >> 8;
		command[9] = (ulAddress&0x00ff0000) >> 16;
		command[10] = (ulAddress&0xff000000) >> 24;
		command[11] = 8;

		sum = 0;
		for (j = 0; j < 12; j++)
			sum += command[j];
		command[MAX_CMD_SIZE - 2] = ~sum+1;
					       
		sum = 0;
		for (i = 12; i < (FLASH_BLOCK_SIZE + 12); i++){
			command[i] = flash_data[ulAddress+(i-12)];
			sum += flash_data[ulAddress+(i-12)];
		}
		command[MAX_CMD_SIZE - 1] = ~sum+1;
		msgs.len=BOOT_CMD_SIZE;
		msgs.buf=command;
		ret = i2c_transfer(client->adapter, &msgs,1);
		//ret = write(fd, command, BOOT_CMD_SIZE);
		if (ret < 0) {
			printk("%s 2 ret:%d \n", __func__, ret);
			return false;
		}
		
		if (ulAddress <= 0x0ffff) {
			msleep(1);//usleep(250);
		}
		else if (ulAddress >= 0x10000 && ulAddress <= 0x20000) {
			msleep(1);//usleep(350);
		}
		else {
			msleep(10);
		}

		
		len = 0;
		buf[len++] = 4;
		buf[len++] = 0;
		buf[len++] = 0x38;
		buf[len++] = CMD_GET_FEATURE;
		msgs.len=len;
		msgs.buf=buf;
		ret = i2c_transfer(client->adapter, &msgs,1);
		//ret = write(fd, buf, len);
		if (ret < 0) {
			printk("%s 3 ret:%d \n", __func__, ret);
			return false;
		}
		
		len = 0;
		buf[len++] = 5;
		buf[len++] = 0;
		msgs.len=len;
		msgs.buf=buf;
		ret = i2c_transfer(client->adapter, &msgs,1);
		//ret = write(fd, buf, len);
		if (ret < 0) {
			printk("%s 4 ret:%d \n", __func__, ret);
			return false;
		}
		msgs.flags=I2C_M_RD;
		msgs.buf=response;
		msgs.len=BOOT_RSP_SIZE;
		ret = i2c_transfer(client->adapter, &msgs,1);
		//ret = read(fd, response, BOOT_RSP_SIZE);
		if (ret < 0) {
			printk("%s 5 ret:%d \n", __func__, ret);
			return false;
		}
		
		//printk("response[%s]=%d,%d,%d,%d,%d,%d\n",__func__,response[0],response[1],response[2],response[3],response[4],response[5]);
		if ((response[3] != VERIFY_CMD) ||
		    (response[4] != ECH) ||
		    (response[5] != ACK)) {
			printk("%s res3:%d res4:%d res5:%d \n", __func__, response[3], response[4], response[5]);
			return false;
		}
		pageNo++;
	}	

	return true;
}

bool is_flash_marking(struct i2c_client *client, size_t data_size)
{
	const int MAX_CMD_SIZE = (12 + FLASH_BLOCK_SIZE + 2);
	int len, ret, ECH;
	int i, j;
	unsigned char flash_data[FLASH_BLOCK_SIZE] = {0};
	unsigned char buf[300] = {0};
	unsigned char sum;
	unsigned char response[RSP_SIZE] = {0};
	unsigned char command[MAX_CMD_SIZE];
	struct i2c_msg msgs=
					{
						.addr = client->addr,
						.flags = 0,
						.len = 0,
						.buf = NULL,
					};
	for (i = 0; i<FLASH_BLOCK_SIZE; i++) {
		flash_data[i] = 0xFF;
	}
		
	flash_data[MARKER] = 0x00;
		
	len = 0;
	buf[len++] = 4;
	buf[len++] = 0;
	buf[len++] = 0x37;
	buf[len++] = CMD_SET_FEATURE;
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 1 ret:%d \n", __func__, ret);
		return false;
	}		

	command[0] = 5;
	command[1] = 0;
	command[2] = 76;
	command[3] = 0;
	command[4] = BOOT_CMD_REPORT_ID;
	command[5] = BOOT_VERIFY_FLASH;
	command[6] = ECH = 1;
	command[7] = 0xC0;
	command[8] = 0x1F;
	command[9] = 0x01;
	command[10] = 0x00;
	command[11] = 8;
	
	sum = 0;
	for (j = 0; j < 12; j++)
		sum += command[j];

	command[MAX_CMD_SIZE - 2] = ~sum+1;
	
	sum = 0;
	for (i = 12; i < (FLASH_BLOCK_SIZE + 12); i++){
		command[i] = flash_data[i - 12];
		sum += flash_data[i - 12];
	}
	command[MAX_CMD_SIZE - 1] = ~sum+1;
	msgs.len=MAX_CMD_SIZE;
	msgs.buf=command;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, command, MAX_CMD_SIZE);
	if (ret < 0) {
		printk("%s 2 ret: %d \n", __func__, ret);
		return false;
	}	

	msleep(10);
	
	len = 0;
	buf[len++] = 4;
	buf[len++] = 0;
	buf[len++] = 0x38;
	buf[len++] = CMD_GET_FEATURE;
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 3 ret: %d \n", __func__, ret);
		return false;
	}

	len = 0;
	buf[len++] = 5;
	buf[len++] = 0;
	msgs.len=len;
	msgs.buf=buf;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, buf, len);
	if (ret < 0) {
		printk("%s 4 ret:%d \n", __func__, ret);
		return false;
	}
	msgs.flags=I2C_M_RD;
	msgs.buf=response;
	msgs.len=RSP_SIZE;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = read(fd, response, RSP_SIZE);
	if (ret < 0) {
		printk("%s 5 ret: %d \n", __func__, ret);
		return false;
	}		
	
	//printk("response[%s]=%d,%d,%d,%d,%d,%d\n",__func__,response[0],response[1],response[2],response[3],response[4],response[5]);
	printk("%s checking response\n", __func__);
	if ((response[3] != MARK_CMD) ||
	    (response[4] != ECH) ||
	    (response[5] != ACK) ) {
		printk("%s fails res3:%d res4:%d res5:%d \n", __func__, response[3], response[4], response[5]);
		return false;
	}
	
	return true;
}


bool wac_flash_end(struct i2c_client *client)
{
	int ret, ECH;
	int len;
	unsigned char command[CMD_SIZE] = {0};
	struct i2c_msg msgs=
					{
						.addr = client->addr,
						.flags = 0,
						.len = 0,
						.buf = NULL,
					};
	len = 0;
	command[len++] = 4;
	command[len++] = 0;
	command[len++] = 0x37;
	command[len++] = CMD_SET_FEATURE;
	command[len++] = 5;
	command[len++] = 0;
	command[len++] = 5;
	command[len++] = 0;
	command[len++] = BOOT_CMD_REPORT_ID;
	command[len++] = BOOT_EXIT;
	command[len++] = ECH = 7;
	msgs.len=len;
	msgs.buf=command;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, command, len);
	if (ret < 0) {
		printk("%s 2 ret:%d \n", __func__, ret);
		return false;
	}	

	return true;
}

unsigned int wac_flash_info(struct i2c_client *client)
{
	int ret;
	unsigned char cmd1[] = { WACOM_CMD_QUERY0, WACOM_CMD_QUERY1,
			WACOM_CMD_QUERY2, WACOM_CMD_QUERY3 };
	unsigned char cmd2[] = { WACOM_CMD_THROW0, WACOM_CMD_THROW1 };
	unsigned char data[WACOM_QUERY_SIZE];
	unsigned int x_max, y_max;
	unsigned int fw_ver;
	unsigned int pressure_max;
	struct i2c_msg msgs=
					{
						.addr = client->addr,
						.flags = 0,
						.len = 0,
						.buf = NULL,
					};
	msgs.len=sizeof(cmd1);
	msgs.buf=cmd1;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, cmd1, sizeof(cmd1));
	if (ret < 0) {
		printk("%s sending information query failed \n", __func__);
		fw_ver=0xffff;
		goto end;
	}
	msgs.len=sizeof(cmd2);
	msgs.buf=cmd2;
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = write(fd, cmd2, sizeof(cmd2));
	if (ret < 0) {
		printk("%s sending information query failed \n", __func__);
		fw_ver=0xffff;
		goto end;
	}
	msgs.flags=I2C_M_RD;
	msgs.buf=data;
	msgs.len=sizeof(data);
	ret = i2c_transfer(client->adapter, &msgs,1);
	//ret = read(fd, data, sizeof(data));

	x_max = (data[4] << 8) | data[3];
	y_max = (data[6] << 8) | data[5];
	pressure_max = (data[12] << 8) | data[11];
	fw_ver = (data[14] << 8) | data[13];


end:
	printk("x_max:%d, y_max:%d, pressure:%d, fw:%x\n",x_max, y_max, pressure_max, fw_ver);
	printk("%s finished \n", __func__);
	return fw_ver;
}

int wacom_i2c_update(struct i2c_client* client)
{
	/*For Flash*/
	int i, ret, fd, cnt;
	int bl_ver = 0, mpu_type = 0;
	int erase_block_num = 0;
	int erase_block[BLOCK_NUM + 1] = {0};
	unsigned long start_address = USER_START_ADDR;
	unsigned long max_address = 0;
	unsigned long max_range = max_address - start_address;
	//unsigned char flash_data[DATA_SIZE] = {0};
	bool bRet;
	unsigned int fw_ver;
	//struct i2c_client* client=wacom_client;
	struct wacom_platform_data *pdata=NULL;
	if(!client)
		return -1;
	pdata=client->dev.platform_data;
	//for (i = 0; i < DATA_SIZE; i++)
		//flash_data[i] = 0xff;
		
	fw_ver=wac_flash_info(client);
	if(fw_ver==0xffff)
		return -1;//i2c transfor error
	   /*Obtain the current firmware information*/
#if defined(CONFIG_MACH_T8400N_8_3CM)
	   if(wacom_BinaryVersion_t8400n==fw_ver)
#else
	   if(wacom_BinaryVersion_t8400n_12==fw_ver)	
#endif
	   	{
	   	printk("wacom_i2c has no update \n");
	   	return 0;
	   	}
	
	   printk("Sending FLASH commands to the device \n");  
	   ret = wac_flash_cmd(client);
	   if (ret < 0) {
		   printk("Sending flash failed \n");
		   ret=-1;
		   goto err;
	   }
	   
	   ret = wac_flash_query(client);
	   if (ret < 0) {
		   printk("Falied to send query \n");
		   ret=-1;
		   goto err;
	   }
	   printk("Device in the boot mode \n");
	
	   printk("Trying to get the bootloader version \n");
	   /*Obtain boot loader version*/
	   if ((ret = get_bootloader_ver(client, &bl_ver)) <0) {
		   printk("Failed to get Boot Loader version \n", __func__);
		   ret=-1;
		   goto err;
	   }   
	   printk("Boot loader version: %x \n", bl_ver);
	
	   printk("Trying to obtain MPU type \n");
	   /*Obtain MPU type: this can be manually done in user space*/
	   if ((ret = get_mpu_type(client, &mpu_type)) < 0) {
		   printk("Failed to get MPU type \n", __func__);
		   ret=-1;
		   goto err;
	   }
	   printk("MPU type: %x \n", mpu_type);
#if defined(CONFIG_MACH_T8400N_8_3CM)
	    if (mpu_type != MPU_W9002)
#else
	    if (mpu_type != MPU_W9007) 
#endif
	{
		   printk("The target device is not equipped with W9007 \n");
		   printk("Flash program is now aborted \n");
		   ret = -1;
		   goto err;
	   }
	
	   /**************************************/
	   /**************************************/
	   /*From here starts flashing operations*/
	   /**************************************/
	   /**************************************/
	   #if defined(CONFIG_MACH_T8400N_8_3CM)
	   bRet = wac_flash_security_unlock(client);
	   if (!bRet) {
		   printk("failed to set security unlock \n");
		   ret = -1;
		   goto err;
	   }
	   #endif
	
	   printk("Erasing the current firmware \n");
	   max_address = max_range = USER_MAX_ADDR;
	   max_range -= start_address;
	   max_range >>= 6;
	   if (max_address > (max_range << 6))
		   max_range++;
	#if defined(CONFIG_MACH_T8400N_8_3CM)
	   erase_block[erase_block_num++] = 2;
	   erase_block[erase_block_num++] = 3;
	   erase_block[erase_block_num++] = 1;
	   erase_block[erase_block_num++] = 0;
	#else if defined(CONFIG_MACH_T8400N_11_6CM)
	   for (i = BLOCK_NUM; i >= 8; i--) 
	   {		
	   	erase_block[erase_block_num] = i;		
		erase_block_num++;	
	   }
	   #endif
	   bRet = wac_flash_erase(client, erase_block, erase_block_num);
	   if (!bRet) {
		   printk("Failed to erase the user program \n");
		   ret = -1;
		   goto err;
	   }
	
	   printk("Writing new firmware \n");
	   #if defined(CONFIG_MACH_T8400N_8_3CM)
	
	   max_address = PROGRAM_MAX_ADDR;
	   bRet = wac_flash_write(client, wacom_BinaryFile_t8400n, DATA_SIZE, start_address, &max_address);
	   #else if defined(CONFIG_MACH_T8400N_11_6CM)
	   bRet = wac_flash_write(client, wacom_BinaryFile_t8400n_12, DATA_SIZE, start_address, &max_address);
	   #endif
	   if (!bRet) {
		   printk("Failed to write the new user program \n");
		   ret = -1;
		   goto err;
	   }
	#if defined(CONFIG_MACH_T8400N_8_3CM)
	   printk("Marking the firmware \n");
	   bRet = wac_flash_marking(client, DATA_SIZE, true);
	   if (!bRet) {
	
		   printk("Failed to mark the firmware \n");
		   ret = -1;
		   goto err;
	   }
	
	   printk("Verifying the firmware \n");
	   max_address = CHECK_PROC_MAX_ADDR;
	   bRet = wac_flash_verify(client, wacom_BinaryFile_t8400n, DATA_SIZE, start_address, &max_address);
	   if (!bRet) {
		   printk("Failed to verify the firmware \n");
		   ret = -1;
		   goto err;
	   }
	
	   printk("Checking if the firmware is marked \n");
	   bRet = is_flash_marking(client, DATA_SIZE);
	   if (!bRet) {
		   printk("Firmware is not marked \n");
		   ret = -1;
		   goto err;
	   }
	#endif
	   /*Return to the user mode*/
	   printk("Closing the boot mode \n");
	   bRet = wac_flash_end(client);
	   if (!bRet) {
		   printk("Cannot close the boot mode  \n");
		   ret = -1;
	   }
	   
	   /*Need sleep for the firmware to return from boot mode*/
	   msleep(50);
	
	   /*Obtain the updated firmware information*/
	   wac_flash_info(client);
	
	   printk("Write and verify completed \n");
	   ret = 1;
	   return ret;
	err:
	   if(pdata->pdct_pin) {
		gpio_direction_output(pdata->pdct_pin, 0);
	  }
	   return ret;
}
#if 0

static ssize_t wacom_tp_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	//printk("novatek_tp_show\n");
	return strlen(buf);
}

static ssize_t wacom_tp_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	if(buf==NULL)
		return -1;
	if(buf[0]=='0')
	{
		wac_flash_info(wacom_client);
		//printk("\n [TSP]:device will suspend! \n");
		
	}
	else if(buf[0]=='1')
	{
		//wacom_i2c_update(wacom_client);
#if 0
		struct task_struct *thread = NULL;

		    printk("Ready to run update thread.");
		    thread = kthread_run(wacom_i2c_update, (void*)NULL, "wacom_update");
		    if (IS_ERR(thread))
		    {
		        printk("Failed to create update thread.\n");
		    }
#endif
	}
	return count;


}
static struct device_attribute wacom_i2c_attributes[] = {
	__ATTR(wacom_debug, 0666, wacom_tp_show, wacom_tp_store),
	__ATTR_NULL,
};

#endif

static ssize_t wacom_test_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int ret,x_max,y_max,pressure_max;
	char fw;
	u8 cmd1[] = { WACOM_CMD_QUERY0, WACOM_CMD_QUERY1,
			WACOM_CMD_QUERY2, WACOM_CMD_QUERY3 };
	u8 cmd2[] = { WACOM_CMD_THROW0, WACOM_CMD_THROW1 };
	u8 data[WACOM_QUERY_SIZE];
	struct i2c_msg msgs[] = {
		{
			.addr = g_wac_i2c->client->addr,
			.flags = 0,
			.len = sizeof(cmd1),
			.buf = cmd1,
		},
		{
			.addr = g_wac_i2c->client->addr,
			.flags = 0,
			.len = sizeof(cmd2),
			.buf = cmd2,
		},
		{
			.addr = g_wac_i2c->client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(data),
			.buf = data,
		},
	};

	ret = i2c_transfer(g_wac_i2c->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return sprintf(buf,"%d\n",0);
	if (ret != ARRAY_SIZE(msgs))
		return sprintf(buf,"%d\n",0);

	fw = get_unaligned_le16(&data[13]);

	if(fw == g_wac_i2c->features.fw_version)
		return sprintf(buf,"%d\n",1);
	else
		return sprintf(buf,"%d\n",0);
}


static struct device_attribute wacom_attributes[] = {
	__ATTR(wacom_test, S_IROTH, wacom_test_show, NULL),
	__ATTR_NULL,
};

//------------------------------end---------------------------------------

static int wacom_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct wacom_i2c *wac_i2c;
	struct input_dev *input;
	struct wacom_features features = { 0 };
	int i;
	int dis_xres = DISPALY2_RESOLUTION_XRES,dis_yres = DISPALY2_RESOLUTION_YRES;
	int error;
	printk("--%s--\n",__func__);
	struct wacom_platform_data *pdata = client->dev.platform_data;
	if(!pdata){
		error = -EINVAL;
		goto fail;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	if(pdata->pdct_pin) {
		gpio_request(pdata->pdct_pin, "pdct_pin");
		gpio_direction_input(pdata->pdct_pin);
	}

	if(pdata->powerInit){
		error = pdata->powerInit();
		if(error)
			goto fail;
	}
	error=wacom_i2c_update(client);
	if(error<0)
		goto fail;
	if(pdata->prst_pin)
	{
		mdelay(50);
		gpio_set_value(pdata->prst_pin,1);
		mdelay(50);
		gpio_set_value(pdata->prst_pin, 0);
		mdelay(50);
		gpio_set_value(pdata->prst_pin, 1);
		mdelay(100);
	}
	error = wacom_query_device(client, &features);
	if (error)
	{
		printk("wacom_query_device failed\n");
		goto fail;
	}
	wac_i2c = kzalloc(sizeof(*wac_i2c), GFP_KERNEL);
	input = input_allocate_device();

	if (!wac_i2c || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}
	wac_i2c->pdata =pdata;
	wac_i2c->client = client;
	wac_i2c->input = input;
	wac_i2c->features = features;
	memcpy(&wac_i2c->features,&features,sizeof(wac_i2c->features));
	g_wac_i2c = wac_i2c;
	wacom_client=client;
	input->name = "wacom-i2c";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x56a;
	input->id.version = features.fw_version;
	input->dev.parent = &client->dev;
	input->open = wacom_i2c_open;
	input->close = wacom_i2c_close;
	input->enabled = true;
	input->enable =wacom_i2c_enable;
	input->disable=wacom_i2c_disable;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOOL_RUBBER, input->keybit);
	__set_bit(BTN_STYLUS, input->keybit);
	__set_bit(BTN_STYLUS2, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);
	if(wac_i2c->pdata){
		dis_xres = wac_i2c->pdata->display_xres;
		dis_yres = wac_i2c->pdata->display_yres;
	}
	input_set_abs_params(input, ABS_X, 0, dis_xres/*features.x_max*/, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, dis_yres/*features.y_max*/, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE,
			     0, features.pressure_max, 0, 0);
	input_set_capability(input, EV_KEY, KEY_POWER);
	input_set_drvdata(input, wac_i2c);

	error = request_threaded_irq(client->irq, NULL, wacom_i2c_irq,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     "wacom_i2c", wac_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_mem;
	}

	if(pdata->plug_detect_gpio){

		if (pdata->debounce_interval) {
			error = gpio_set_debounce(pdata->plug_detect_gpio,
					pdata->debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
			if (error < 0){
				debug_plug_print("set gpio =%d debounce fail!",pdata->plug_detect_gpio);
				wac_i2c->timer_debounce = pdata->debounce_interval;
			}
		}

		INIT_WORK(&wac_i2c->work, gpio_keys_gpio_work_func);
		if(wac_i2c->timer_debounce)
		setup_timer(&wac_i2c->timer,
			    	gpio_keys_gpio_timer, (unsigned long)wac_i2c);
		
		
		
		error = request_threaded_irq(gpio_to_irq(pdata->plug_detect_gpio), NULL, wacom_pluged_irq,
					     IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					     "wacom_plug", wac_i2c);
		if (error) {
			dev_err(&client->dev,
				"Failed to enable plug IRQ, error: %d\n", error);
			goto err_free_works;
		}
		enable_irq_wake(gpio_to_irq(pdata->plug_detect_gpio));
	}

	/* Disable the IRQ, we'll enable it in wac_i2c_open() */
	disable_irq(client->irq);

	error = input_register_device(wac_i2c->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device, error: %d\n", error);
		goto err_free_irq;
	}
	//device_create_file(&wac_i2c->input->dev, &wacom_i2c_attributes);
	i2c_set_clientdata(client, wac_i2c);
	device_create_file(&client->dev,wacom_attributes);
	return 0;

err_free_irq:
	if(pdata->plug_detect_gpio)
		free_irq(gpio_to_irq(pdata->plug_detect_gpio), wac_i2c);
	free_irq(client->irq, wac_i2c);
err_free_works:
	if(pdata->plug_detect_gpio){
		if (wac_i2c->timer_debounce)
			del_timer_sync(&wac_i2c->timer);
		cancel_work_sync(&wac_i2c->work);
	}
err_free_mem:
	input_free_device(input);
	kfree(wac_i2c);
fail:
	pr_err("%s:==============> wacom probe fail!!\n",__func__);
	if(pdata->powerDisable)
		pdata->powerDisable();

	if(pdata->pdct_pin) {
		gpio_direction_output(pdata->pdct_pin, 0);
	}
	return error;
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	
	free_irq(client->irq, wac_i2c);
	if (wac_i2c->timer_debounce)
		del_timer_sync(&wac_i2c->timer);
	cancel_work_sync(&wac_i2c->work);
	input_unregister_device(wac_i2c->input);
	kfree(wac_i2c);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int wacom_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	disable_irq(client->irq);

	return 0;
}

static int wacom_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	enable_irq(client->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(wacom_i2c_pm, wacom_i2c_suspend, wacom_i2c_resume);

static const struct i2c_device_id wacom_i2c_id[] = {
	{ "WAC_I2C_EMR", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, wacom_i2c_id);

static struct i2c_driver wacom_i2c_driver = {
	.driver	= {
		.name	= "wacom_i2c",
		.owner	= THIS_MODULE,
		.pm	= &wacom_i2c_pm,
	},

	.probe		= wacom_i2c_probe,
	.remove		= wacom_i2c_remove,
	.id_table	= wacom_i2c_id,
};
module_i2c_driver(wacom_i2c_driver);

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@wacom.co.jp>");
MODULE_DESCRIPTION("WACOM EMR I2C Driver");
MODULE_LICENSE("GPL");
