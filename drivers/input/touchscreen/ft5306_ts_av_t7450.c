/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 */
#if 0
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/input-polldev.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>
#include <mach/iomux.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
//#include <mach/rk29_iomap.h>
#include <mach/board.h>
//#include <mach/rk29_nand.h>
//#include <mach/rk29_camera.h>                          /* ddl@rock-chips.com : camera support */
#include <media/soc_camera.h>                               /* ddl@rock-chips.com : camera support */
//#include <mach/vpu_mem.h>
#include <mach/sram.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#endif
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/i2c.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include<linux/i2c/ft5x0x_ts_data.h>
#include <linux/gpio.h>
#include <mach/gpio-tegra.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>


#define FTS_CTL_IIC

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

#include <linux/input/mt.h>

struct FT5X0X_i2c_ts_platform_data *ft_pdata=NULL;

#if 0
#define FTprintk(x...) printk(x)
#else
#define FTprintk(x...) do{} while(0)
#endif

#define CONFIG_FT5X0X_MULTITOUCH  1
#define MAX_POINT                 10
#define FT5306_IIC_SPEED          100*1000    //300*1000
//#define TOUCH_RESET_PIN           RK30_PIN1_PC0
#define FT5X0X_REG_THRES          0x80         /* Thresshold, the threshold be low, the sensitivy will be high */
#define FT5X0X_REG_REPORT_RATE    0x88         /* **************report rate, in unit of 10Hz **************/
#define FT5X0X_REG_PMODE          0xA5         /* Power Consume Mode 0 -- active, 1 -- monitor, 3 -- sleep */    
#define FT5X0X_REG_FIRMID         0xA6         /* ***************firmware version **********************/
#define FT5X0X_REG_NOISE_MODE     0xb2         /* to enable or disable power noise, 1 -- enable, 0 -- disable */
#define SCREEN_MAX_X              900//
#define SCREEN_MAX_Y              1440//
#define PRESS_MAX                 255
#define FT5X0X_NAME	              "ft5x0x_ts"
#define TOUCH_MAJOR_MAX           200
#define WIDTH_MAJOR_MAX           200
//FT5X0X_REG_PMODE
#define PMODE_ACTIVE              0x00
#define PMODE_MONITOR             0x01
#define PMODE_STANDBY             0x02
#define PMODE_HIBERNATE           0x03

/* zhengxing@rock-chips: avoid to firmwarm upgrade directly cause barrage when startup. */
#define FT5X0X_FW_UPGRADE_ASYNC         (1)
#define FT5X0X_FW_UPGRADE_DIRECTLY      (2)

#if 0
#define FT5X0X_FW_UPGRADE_MODE          FT5X0X_FW_UPGRADE_ASYNC
#else
#define FT5X0X_FW_UPGRADE_MODE          FT5X0X_FW_UPGRADE_DIRECTLY
#endif

#define IC_FT5X06	0
#define IC_FT5606	1

#define FT_UPGRADE_AA	0xAA
#define FT_UPGRADE_55 	0x55
#define FT_UPGRADE_EARSE_DELAY		2000

/*upgrade config of FT5606*/
#define FT5606_UPGRADE_AA_DELAY 		50
#define FT5606_UPGRADE_55_DELAY 		10
#define FT5606_UPGRADE_ID_1			0x79
#define FT5606_UPGRADE_ID_2			0x06
#define FT5606_UPGRADE_READID_DELAY 	100

/*upgrade config of FT5x06(x=2,3,4)*/
#define FT5X06_UPGRADE_AA_DELAY 		50
#define FT5X06_UPGRADE_55_DELAY 		30
#define FT5X06_UPGRADE_ID_1			0x79
#define FT5X06_UPGRADE_ID_2			0x03
#define FT5X06_UPGRADE_READID_DELAY 	1

#define DEVICE_IC_TYPE	IC_FT5606
struct Upgrade_Info {
	u16 delay_aa;		/*delay of write FT_UPGRADE_AA */
	u16 delay_55;		/*delay of write FT_UPGRADE_55 */
	u8 upgrade_id_1;	/*upgrade id 1 */
	u8 upgrade_id_2;	/*upgrade id 2 */
	u16 delay_readid;	/*delay of read id */
};
struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
    s16  touch_ID1;
	s16  touch_ID2;
    s16  touch_ID3;
    s16  touch_ID4;
	s16  touch_ID5;
	u8   touch_point;
	u8   status;
};

struct tp_event {
	u16	x;
	u16	y;
    s16 id;
	u16	pressure;
	u8  touch_point;
	u8  flag;
};

struct ft5x0x_ts_data {
	struct i2c_client *client;
	struct input_dev	*input_dev;
	int    irq;
	//int     (*platform_sleep)(void);
   // int     (*platform_wakeup)(void);
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend ft5306_early_suspend;
#endif
#if (FT5X0X_FW_UPGRADE_MODE == FT5X0X_FW_UPGRADE_ASYNC)
    struct work_struct  fw_upgrade_work;
    struct workqueue_struct *fw_workqueue;
#endif
};
static struct i2c_client *this_client;
//extern __sramdata int TouchPanelFound;
/***********************************************************************/

enum  __touch__{
	TOUCH_ERROR = -1,
	EDT_TOUCH = 0x0c,
	GFT_TOUCH =0x08,
};


#define    FTS_PACKET_LENGTH        128

static unsigned char * CTPM_FW=NULL;
static unsigned int CTPM_FW_SIZE=0;
static unsigned char CTPM_FW_EDT[]=
{
#include "ft5606_EDT_firmware.h"
};
static unsigned char CTPM_FW_GFT[]=
{
//#include "ft_app_gft.i"
};
static int touch_type=0;

static bool CTPM_FW_init()
{
	bool ret=false;//update tp fw or not
	switch(touch_type)
	{
		case EDT_TOUCH:
		CTPM_FW=CTPM_FW_EDT;
		CTPM_FW_SIZE=sizeof(CTPM_FW_EDT);
		ret=true;
		break;
		case GFT_TOUCH:
		CTPM_FW=CTPM_FW_GFT;
		CTPM_FW_SIZE=sizeof(CTPM_FW_GFT);
		ret=true;
		break;
		default:
		CTPM_FW=NULL;
		CTPM_FW_SIZE=0;
		break;
	}
	return ret;
}



typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

/***********************************************************************/

/***********************************************************************
    [function]: 
		           callback:                send data to ctpm by i2c interface;
    [parameters]:
			    txdata[in]:              data buffer which is used to send data;
			    length[in]:              the length of the data buffer;
    [return]:
			    FTS_TRUE:              success;
			    FTS_FALSE:             fail;
************************************************************************/
static int fts_i2c_txdata(u8 *txdata, int length)
{
	int ret;

	struct i2c_msg msg;

      msg.addr = this_client->addr;
      msg.flags = 0;
      msg.len = length;
      msg.buf = txdata;
      //msg.scl_rate = FT5306_IIC_SPEED;
	ret = i2c_transfer(this_client->adapter, &msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}




/***********************************************************************
    [function]: 
		           callback:               write data to ctpm by i2c interface;
    [parameters]:
			    buffer[in]:             data buffer;
			    length[in]:            the length of the data buffer;
    [return]:
			    FTS_TRUE:            success;
			    FTS_FALSE:           fail;
************************************************************************/
static bool  i2c_write_interface(u8* pbt_buf, int dw_lenth)
{
    int ret;
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        FTprintk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return false;
    }

    return true;
}

/***********************************************************************
    [function]: 
		           callback:                read register value ftom ctpm by i2c interface;
    [parameters]:
                        reg_name[in]:         the register which you want to write;
			    tx_buf[in]:              buffer which is contained of the writing value;
    [return]:
			    FTS_TRUE:              success;
			    FTS_FALSE:             fail;
************************************************************************/
static bool fts_register_write(u8 reg_name, u8* tx_buf)
{
	u8 write_cmd[2] = {0};

	write_cmd[0] = reg_name;
	write_cmd[1] = *tx_buf;

	/*call the write callback function*/
	return i2c_write_interface(write_cmd, 2);
}

/***********************************************************************
[function]: 
                      callback:         send a command to ctpm.
[parameters]:
			  btcmd[in]:       command code;
			  btPara1[in]:     parameter 1;    
			  btPara2[in]:     parameter 2;    
			  btPara3[in]:     parameter 3;    
                      num[in]:         the valid input parameter numbers, 
                                           if only command code needed and no 
                                           parameters followed,then the num is 1;    
[return]:
			  FTS_TRUE:      success;
			  FTS_FALSE:     io fail;
************************************************************************/
static bool cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
    u8 write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(write_cmd, num);
}

/***********************************************************************
    [function]: 
		           callback:              read data from ctpm by i2c interface;
    [parameters]:
			    buffer[in]:            data buffer;
			    length[in]:           the length of the data buffer;
    [return]:
			    FTS_TRUE:            success;
			    FTS_FALSE:           fail;
************************************************************************/
static bool i2c_read_interface(u8* pbt_buf, int dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        FTprintk("[TSP]i2c_read_interface error\n");
        return false;
    }
  
    return true;
}


/***********************************************************************
[function]: 
                      callback:         read a byte data  from ctpm;
[parameters]:
			  buffer[in]:       read buffer;
			  length[in]:      the size of read data;    
[return]:
			  FTS_TRUE:      success;
			  FTS_FALSE:     io fail;
************************************************************************/
static bool byte_read(u8* buffer, int length)
{
    return i2c_read_interface(buffer, length);
}

/***********************************************************************
[function]: 
                      callback:         write a byte data  to ctpm;
[parameters]:
			  buffer[in]:       write buffer;
			  length[in]:      the size of write data;    
[return]:
			  FTS_TRUE:      success;
			  FTS_FALSE:     io fail;
************************************************************************/
static bool byte_write(u8* buffer, int length)
{
    
    return i2c_write_interface(buffer, length);
}

/***********************************************************************
    [function]: 
		           callback:                 read register value ftom ctpm by i2c interface;
    [parameters]:
                        reg_name[in]:         the register which you want to read;
			    rx_buf[in]:              data buffer which is used to store register value;
			    rx_length[in]:          the length of the data buffer;
    [return]:
			    FTS_TRUE:              success;
			    FTS_FALSE:             fail;
************************************************************************/
static bool fts_register_read(u8 reg_name, u8* rx_buf, int rx_length)
{
	u8 read_cmd[2]= {0};
	u8 cmd_len 	= 0;

	read_cmd[0] = reg_name;
	cmd_len = 1;	

	/*send register addr*/
	if(!i2c_write_interface(&read_cmd[0], cmd_len))
	{
		return false;
	}

	/*call the read callback function to get the register value*/		
	if(!i2c_read_interface(rx_buf, rx_length))
	{
		return false;
	}
	return true;
}


/*
*get upgrade information depend on the ic type
*/
static void fts_get_upgrade_info(struct Upgrade_Info *upgrade_info)
{
	switch (DEVICE_IC_TYPE) {
	case IC_FT5X06:
		upgrade_info->delay_55 = FT5X06_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5X06_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5X06_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5X06_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5X06_UPGRADE_READID_DELAY;
		break;
	case IC_FT5606:
		upgrade_info->delay_55 = FT5606_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5606_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5606_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5606_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5606_UPGRADE_READID_DELAY;
		break;
	default:
		break;
	}
}
 
/***********************************************************************
[function]: 
                        callback:          burn the FW to ctpm.
[parameters]:
			    pbt_buf[in]:     point to Head+FW ;
			    dw_lenth[in]:   the length of the FW + 6(the Head length);    
[return]:
			    ERR_OK:          no error;
			    ERR_MODE:      fail to switch to UPDATE mode;
			    ERR_READID:   read id fail;
			    ERR_ERASE:     erase chip fail;
			    ERR_STATUS:   status error;
			    ERR_ECC:        ecc error.
************************************************************************/
E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(u8* pbt_buf, int dw_lenth)
{
    u8  cmd,reg_val[2] = {0};
	u8  buffer[2] = {0};
    u8  packet_buf[FTS_PACKET_LENGTH + 6];
    u8  auc_i2c_write_buf[10];
    u8  bt_ecc;
	
    int  j,temp,lenght,i_ret,packet_number, i = 0;
    int  i_is_new_protocol = 0;
	struct Upgrade_Info upgradeinfo;

	fts_get_upgrade_info(&upgradeinfo);
    /*******Step 1:Reset  CTPM ****/
    /******write 0xaa to register 0xfc******/
    cmd=FT_UPGRADE_AA;
    fts_register_write(0xfc,&cmd);
    mdelay(50);
	
     /******write 0x55 to register 0xfc******/
    cmd=FT_UPGRADE_55;
    fts_register_write(0xfc,&cmd);
    FTprintk("[TSP] Step 1: Reset CTPM test\n");
   
    mdelay(10);   
    
    /*******Step 2:Enter upgrade mode ****/
    FTprintk("\n[TSP] Step 2:enter new update mode\n");
    auc_i2c_write_buf[0] = FT_UPGRADE_55;
    auc_i2c_write_buf[1] = FT_UPGRADE_AA;
    do
    {
        i ++;
        i_ret = fts_i2c_txdata(auc_i2c_write_buf, 2);
        mdelay(5);
    }while(i_ret <= 0 && i < 10 );

    if (i > 1)
    {
        i_is_new_protocol = 1;
    }    
    /********Step 3:check READ-ID********/  
    mdelay(upgradeinfo.delay_readid);	
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == upgradeinfo.upgrade_id_1 && reg_val[1] ==   upgradeinfo.upgrade_id_2)
    {
        FTprintk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
	    FTprintk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
        return ERR_READID;
        //i_is_new_protocol = 1;
    }    
    
     /*********Step 4:erase app**********/
    if (i_is_new_protocol)
    {
        cmd_write(0x61,0x00,0x00,0x00,1);
    }
    else
    {
        cmd_write(0x60,0x00,0x00,0x00,1);
    }
    mdelay(1500);
    FTprintk("[TSP] Step 4: erase. \n");



    /*Step 5:write firmware(FW) to ctpm flash*/
    bt_ecc = 0;
    FTprintk("[TSP] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
	FTprintk("[TSP]  packet_number = %d\n",packet_number);
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8)(lenght>>8);
        packet_buf[5] = (u8)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        mdelay(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              FTprintk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8)(temp>>8);
        packet_buf[5] = (u8)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);    
        mdelay(20);
    }

    /***********send the last six byte**********/
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;
        temp =1;
        packet_buf[4] = (u8)(temp>>8);
        packet_buf[5] = (u8)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);  
        mdelay(20);
    }

    /********send the opration head************/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    FTprintk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*******Step 7: reset the new FW**********/
    cmd_write(0x07,0x00,0x00,0x00,1);
	mdelay(100);//100ms	
	fts_register_read(0xfc, buffer, 1);	
	if (buffer[0] == 1)
	{
	cmd=4;
	fts_register_write(0xfc, &cmd);
	mdelay(2500);//2500ms	
	 do	
	 {	
	 fts_register_read(0xfc, buffer, 1);	
	 mdelay(100);//100ms	
	 }while (buffer[0] != 1); 		   	
	}
    return ERR_OK;
}
int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp;
    unsigned char i ;
    u8  cmd;
    printk("[FTS] start auto CLB.\n");
    msleep(200);
	cmd=0x40;
    fts_register_write(0, &cmd);  
    mdelay(100);   //make sure already enter factory mode
	cmd=0x4;
    fts_register_write(2, &cmd);  //write command to start calibration
    mdelay(300);
    for(i=0;i<100;i++)
    {
        fts_register_read(0,&uc_temp,2);
        if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
        {
            break;
        }
        mdelay(200);
        printk("[FTS] waiting calibration %d\n",i);
        
    }
    printk("[FTS] calibration OK.\n");
    
    msleep(300);
	cmd =0x40;
    fts_register_write(0, &cmd);  //goto factory mode
    mdelay(100);   //make sure already enter factory mode
	cmd=0x5;
    fts_register_write(2, &cmd);  //store CLB result
    mdelay(300);
	cmd=0x0;
    fts_register_write(0, &cmd); //return to normal mode 
    msleep(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}

/***********************************************************************/

int fts_ctpm_fw_upgrade_with_i_file(void)
{
   u8*     pbt_buf = 0;
   int i_ret;

	pbt_buf = CTPM_FW;
	i_ret =  fts_ctpm_fw_upgrade(pbt_buf,CTPM_FW_SIZE);

   if (i_ret != 0)
   {
       printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
       //error handling ...
       //TBD
   }
   else
   {
       printk("[FTS] upgrade successfully.\n");
       fts_ctpm_auto_clb();  //start auto CLB
   }
   return i_ret;
}

/***********************************************************************/

unsigned char fts_ctpm_get_upg_ver(void)
{
    if (CTPM_FW_SIZE > 2)
    {
        return CTPM_FW[CTPM_FW_SIZE - 2];
    }
    else
        return 0xff; 
 
}

static void ft5606_power_supply(bool state)//true:vdd power on,false:vdd power off
{  
  //if(machine_is_terra10()||machine_is_terra7()||machine_is_t8400n())
  {  	
  		static int enabled=0;
  		static struct regulator *keenhi_t40_touch_vddio=NULL;
		if(!keenhi_t40_touch_vddio)
			keenhi_t40_touch_vddio = regulator_get(NULL, "vdd_touch_5v0");
		if (WARN_ON(IS_ERR(keenhi_t40_touch_vddio))) 
		{
			pr_err("%s: couldn't get regulator vdd_touch_5v0: %ld\n",
				__func__, PTR_ERR(keenhi_t40_touch_vddio));
		} 
		else 
		{
			//printk("ft5606_power_supply %d\n",state);
			if(state)
			{
				int ret=0;
				if(enabled)
					return;
				ret = regulator_enable(keenhi_t40_touch_vddio);
				enabled=1;
				if (ret < 0) 
					pr_err("%s: couldn't enable regulator vdd_touch_5v0: %ld\n",
						__func__, PTR_ERR(keenhi_t40_touch_vddio));	
			}
			else
			{
				if(!enabled)
					return;
				regulator_disable(keenhi_t40_touch_vddio);
				regulator_put(keenhi_t40_touch_vddio);
				keenhi_t40_touch_vddio=NULL;
				enabled=0;
			}
		}		
  }
}


/*read the it7260 register ,used i2c bus*/
static int ft5306_read_regs(struct i2c_client *client, u8 reg, u8 *buf, unsigned len)
{
	struct i2c_msg msgs[2];
	int ret=-1;

	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=1;
	msgs[0].buf=&reg;

	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len;
	msgs[1].buf=buf;

	ret=i2c_transfer(client->adapter,msgs, 2);

	return ret;


}

/* set the it7260 registe,used i2c bus*/
static int ft5306_set_regs(struct i2c_client *client, u8 reg, u8 *buf, unsigned short len)
{
	struct i2c_msg msg;
	int ret=-1;
	
	msg.flags=!I2C_M_RD;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=&reg;		
	

	ret=i2c_transfer(client->adapter,&msg, 1);

	return ret;

}

#if (FT5X0X_FW_UPGRADE_MODE == FT5X0X_FW_UPGRADE_ASYNC)
static void ft5306_firmware_upgrade_work(struct work_struct *work)
{
    struct ft5x0x_ts_data *data = container_of(work, struct ft5x0x_ts_data, fw_upgrade_work);
    int err = 0;
    unsigned char reg_value;
    unsigned char reg_version;

    printk("<-- %s --> enter\n", __FUNCTION__);

    fts_register_read(FT5X0X_REG_FIRMID, &reg_version,1);
    printk("cdy == [TSP] firmware version = 0x%2x\n", reg_version);

    fts_register_read(FT5X0X_REG_FIRMID, &reg_version,1);
    FTprintk("[TSP] firmware version = 0x%2x\n", reg_version);
    if (fts_ctpm_get_upg_ver() != reg_version)  
    {
      FTprintk("[TSP] start upgrade new verison 0x%2x\n", fts_ctpm_get_upg_ver());
      msleep(200);
      err =  fts_ctpm_fw_upgrade_with_i_file();
      if (err == 0)
      {
          FTprintk("[TSP] ugrade successfuly.\n");
          msleep(300);
          fts_register_read(FT5X0X_REG_FIRMID, &reg_value,1);
          FTprintk("FTS_DBG from old version 0x%2x to new version = 0x%2x\n", reg_version, reg_value);
      }
      else
      {
          FTprintk("[TSP]  ugrade fail err=%d, line = %d.\n",err, __LINE__);
      }
      msleep(4000);
    }
    cancel_work_sync(&data->fw_upgrade_work);
    destroy_workqueue(data->fw_workqueue);
}
#endif

static void ft5306_queue_work(struct work_struct *work)
{
	struct ft5x0x_ts_data *data = container_of(work, struct ft5x0x_ts_data, pen_event_work);
	struct tp_event event;
	u8 start_reg=0x0;
	//u8 buf[32] = {0};
	u8 buf[6*MAX_POINT+1] = {0};
	int ret,i,offset,points;
	static u8 points_last_flag[MAX_POINT]={0};
	struct tp_event  current_events[MAX_POINT];
		
#if CONFIG_FT5X0X_MULTITOUCH
	ret = ft5306_read_regs(data->client,start_reg, buf, 6*MAX_POINT+1);
#else
	ret = ft5306_read_regs(data->client,start_reg, buf, 7);
#endif
	if (ret < 0) {
		dev_err(&data->client->dev, "ft5306_read_regs fail:%d!\n",ret);
		enable_irq(data->irq);
		return;
	}
#if 0
	for (i=0; i<32; i++) {
		FTprintk("buf[%d] = 0x%x \n", i, buf[i]);
	}
#endif
	
	//points = buf[2] & 0x07;
	points = buf[2] & 0x0F;
	
	//dev_info(&data->client->dev, "ft5306_read_and_report_data points = %d\n",points);
	if (points == 0) {
#if   CONFIG_FT5X0X_MULTITOUCH
		//input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		//input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		
		for(i=0;i<MAX_POINT;i++)
		{
			if(points_last_flag[i]!=0)
			{
				FTprintk("Point UP event.id=%d\n",i);
				input_mt_slot(data->input_dev, i);
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);					
			}
		}

		memset(points_last_flag, 0, sizeof(points_last_flag));
		//input_mt_sync(data->input_dev);
#else
		input_report_abs(data->input_dev, ABS_PRESSURE, 0);
		input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif
		input_sync(data->input_dev);
		enable_irq(data->irq);
		return; 
	}
	memset(&event, 0, sizeof(struct tp_event));
#if CONFIG_FT5X0X_MULTITOUCH
  memset(current_events, 0, sizeof(current_events));
  
	for(i=0;i<points;i++){
		offset = i*6+3;
		event.x = (((s16)(buf[offset+0] & 0x0F))<<8) | ((s16)buf[offset+1]);
		event.y = (((s16)(buf[offset+2] & 0x0F))<<8) | ((s16)buf[offset+3]);
		event.id = (s16)(buf[offset+2] & 0xF0)>>4;
		event.flag = ((buf[offset+0] & 0xc0) >> 6);
		event.pressure = 200;
		FTprintk("x=%d, y=%d event.id=%d event.flag=%d\n",event.x,event.y,event.id,event.flag);
		if(event.x<=SCREEN_MAX_X && event.y<=SCREEN_MAX_Y+60){
			//dev_info(&data->client->dev, 
			//	"ft5306 multiple report event[%d]:x = %d,y = %d,id = %d,flag = %d,pressure = %d\n",
			//	i,event.x,event.y,event.id,event.flag,event.pressure);
		if(event.flag)
			memcpy(&current_events[event.id], &event, sizeof(event));
			//points_current[event.id] = event.flag;			
		}
	}
	
	for(i=0;i<MAX_POINT;i++)
	{
		  if((current_events[i].flag == 0) && (points_last_flag[i] != 0))
			{
      		FTprintk("Point UP event.id=%d\n",i);
					input_mt_slot(data->input_dev, i);
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);		 				
			}
			else  if(current_events[i].flag)	
			{	
		  		FTprintk("Point DN event.id=%d\n",i);
					input_mt_slot(data->input_dev, i);
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
					input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 1);
					//input_report_abs(data->input_dev, ABS_MT_PRESSURE, event.pressure);
					input_report_abs(data->input_dev, ABS_MT_POSITION_X,  current_events[i].x);
					input_report_abs(data->input_dev, ABS_MT_POSITION_Y,  current_events[i].y);							  		
			}		
			points_last_flag[i] = 	current_events[i].flag;
	}
#else
	event.x = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
	event.y = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
	event.pressure =200;
	input_report_abs(data->input_dev, ABS_X, event.x);
	input_report_abs(data->input_dev, ABS_Y, event.y);
	//input_report_abs(data->input_dev, ABS_PRESSURE, event.pressure);
	input_report_key(data->input_dev, BTN_TOUCH, 1);
	
	
	//dev_info(&data->client->dev, "ft5306 single report event:x = %d,y = %d\n",event.x,event.y);
#endif
	//printk("ft5306 sync  x = %d ,y = %d\n",event.x,event.y);
	input_sync(data->input_dev);
	enable_irq(data->irq);
	return;
}

static irqreturn_t ft5306_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;
	FTprintk("[TSP]  ft5306_interrupt\n");
	disable_irq_nosync(ft5x0x_ts->irq);
	if (!work_pending(&ft5x0x_ts->pen_event_work)) 
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	return IRQ_HANDLED;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
#define TOUCH_RESET_PIN  ft_pdata->gpio_shutdown

static void ft5306_suspend(struct early_suspend *h)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	char buf_w[1] = {3};
	int err;
    ft5x0x_ts = container_of(h, struct ft5x0x_ts_data, ft5306_early_suspend);
	FTprintk("TSP ft5306_suspend\n");
    err = ft5306_set_regs(this_client, 0xA5, buf_w, 1);
    if (err > 0)
        printk("ft5306_set_regs OK!!\n");
    #if 0
    msleep(20);

	if (ft5x0x_ts->platform_sleep){ 
		ft5x0x_ts->platform_sleep();
	}
    #endif
	gpio_set_value(TOUCH_RESET_PIN,GPIO_LOW);
	disable_irq(ft5x0x_ts->irq);
}

static void ft5306_resume(struct early_suspend *h)
{
	struct ft5x0x_ts_data *ft5x0x_ts;	
	int trytimes;
	u8 val = 0;
	ft5x0x_ts = container_of(h, struct ft5x0x_ts_data, ft5306_early_suspend);
	FTprintk("TSP ft5306_resume\n");
	gpio_set_value(TOUCH_RESET_PIN,GPIO_LOW);
	msleep(10);
	gpio_set_value(TOUCH_RESET_PIN,GPIO_HIGH);
	mdelay(200);
	enable_irq(ft5x0x_ts->irq);
	#if 0
	if (ft5x0x_ts->platform_wakeup)                              
		ft5x0x_ts->platform_wakeup();
		
     
	for(trytimes = 0 ;trytimes < 5; trytimes++){
		if(ft5306_read_regs(this_client, 0x00,  &val,1)<0){
			if (ft5x0x_ts->platform_sleep) 
				ft5x0x_ts->platform_sleep();
			if (ft5x0x_ts->platform_wakeup)                              
				ft5x0x_ts->platform_wakeup();
		}else{
			break;
		}
	}
    #endif  
	//gpio_set_value(TOUCH_RESET_PIN,GPIO_LOW);
	//mdelay(100);
	//gpio_set_value(TOUCH_RESET_PIN,GPIO_HIGH);
}
#endif
#define TOUCH_RESET_PIN  ft_pdata->gpio_shutdown

static ssize_t ft5606_tp_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	printk("ft5606_tp_show\n");
	return strlen(buf);
}

static ssize_t ft5606_tp_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	char buf_w[1] = {3};
	int err;
	u8 cmd;
    	ft5x0x_ts = container_of(this_client, struct ft5x0x_ts_data, client);
	printk("ft5606_tp_store!\n");
	FTprintk("TSP ft5306_suspend\n");
	if(buf==NULL)
		return -1;
	if(buf[0]=='0')
	{
		cmd=PMODE_HIBERNATE;
		printk("\n [TSP]:device will suspend! \n");
		fts_register_write(FT5X0X_REG_PMODE,&cmd);
		msleep(20);
		gpio_set_value(84,0);
		disable_irq(ft5x0x_ts->irq);
		
	}
	else if(buf[0]=='1')
	{
		gpio_set_value(84,1);
		gpio_set_value(84,0);
		msleep(10);
		gpio_set_value(84,1);
		msleep(400);
		enable_irq(ft5x0x_ts->irq);
	}
	return count;


}

static int ft5606_tp_enable(struct input_dev *input_dev)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
  ft5x0x_ts = container_of(input_dev, struct ft5x0x_ts_data, input_dev);
	printk("ft5606_tp_resume!\n");
	{
		ft5606_power_supply(true);	
		gpio_set_value(TOUCH_RESET_PIN,0);
		msleep(10);
		gpio_set_value(TOUCH_RESET_PIN,1);
		msleep(200);
		enable_irq(ft5x0x_ts->irq);
		
			
	}
	return 0;
}
static int ft5606_tp_disable(struct input_dev *input_dev)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	char buf_w[1] = {3};
	int err;
	u8 cmd;
  ft5x0x_ts = container_of(input_dev, struct ft5x0x_ts_data, input_dev);
	printk("ft5606_tp_will_suspend!\n");
	{
		cmd=PMODE_HIBERNATE;
		err=fts_register_write(FT5X0X_REG_PMODE,&cmd);
		if(err<=0)
			printk("ft5606_tp_suspend error:i2c write failed\n");
		msleep(20);
		gpio_set_value(84,0);
		disable_irq(ft5x0x_ts->irq);
		ft5606_power_supply(false);		
	}
	return err;
}


#if 0
static struct device_attribute ft_ts_attributes[] = {
	__ATTR(tp_control, 0666, ft5606_tp_show, ft5606_tp_store),
	__ATTR_NULL,
};
#endif

static int __devexit ft5306_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);

	free_irq(ft5x0x_ts->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	kfree(ft5x0x_ts);

	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif

	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ft5x0x_ts->ft5306_early_suspend);
#endif 
    this_client = NULL;
	return 0;
}

static int  ft5306_probe(struct i2c_client *client ,const struct i2c_device_id *id)
{
	 //gpio_direction_input(RK30_PIN0_PD4);
	//gpio_direction_input(RK30_PIN0_PD7); // add by yu 9-5
	
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	
	int err = 0;
	int ret = 0;
	int retry = 0;
	u8 buf_w[1];
	u8 buf_r[1];
	u8 buf_test[1] = {0};
    unsigned char reg_value;
    unsigned char reg_version;
	unsigned char reg_firmware;
	
	dev_info(&client->dev, "ft5306_ts_probe!\n");
	ft_pdata= client->dev.platform_data;
	if (!ft_pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		dev_err(&client->dev, "Must have I2C_FUNC_I2C.\n");
		return -ENODEV;
	}
	
	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		return -ENOMEM;
	}

    while(retry < 2) 
    {    
        ret = ft5306_set_regs(client,FT5X0X_REG_PMODE, buf_test, 1);
        if(ret > 0) 
            break;
        retry++;

        printk("FT5306 I2C TEST FAILED, retry = %d, ret = %d, will again...\n", retry, ret);

	msleep(20);
        //if (pdata->exit_platform_hw)     
        //    pdata->exit_platform_hw();
        //if (pdata->init_platform_hw)     
        //    pdata->init_platform_hw();
    }    
    printk("FT5306 I2C TEST OK, retry = %d, ret = %d\n", retry, ret);

    if(ret <= 0)
    {    
        printk("FT5306 I2C TEST ERROR! retry = %d, ret = %d\n", retry, ret);
        err = -ENODEV;
        goto exit_i2c_test_fail;
    }
	
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		printk("failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	ft5x0x_ts->client = this_client = client;
	ft5x0x_ts->irq = client->irq;//client->irq;
	ft5x0x_ts->input_dev = input_dev;
  #if   CONFIG_FT5X0X_MULTITOUCH
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	__set_bit(EV_ABS, input_dev->evbit);	

	input_mt_init_slots(input_dev, MAX_POINT);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);	
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	//input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, MAX_POINT, 0, 0);
	//input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	//input_set_abs_params(input_dev, ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

	//input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH); 
	//input_dev->keybit[BIT_WORD(BTN_START)] = BIT_MASK(BTN_START);
	//set_bit(EV_ABS, input_dev->evbit);
	//set_bit(EV_KEY, input_dev->evbit);
	
	input_dev->name		= "ft5x0x_ts";		//dev_name(&client->dev)

	//device_create_file(&ft5x0x_ts->client->dev,&ft_ts_attributes);

	input_dev->enable=ft5606_tp_enable;
	input_dev->disable=ft5606_tp_disable;
	err = input_register_device(input_dev);
	if (err) {
		printk("ft5306_ts_probe: failed to register input device: \n");
		goto exit_input_register_device_failed;
	}
	if (!ft5x0x_ts->irq) {
		err = -ENODEV;
		dev_err(&ft5x0x_ts->client->dev, "no IRQ?\n");
		goto exit_no_irq_fail;
	}else{
		;//ft5x0x_ts->irq = gpio_to_irq(ft5x0x_ts->irq);
	}
#ifdef FTS_CTL_IIC
		if (ft_rw_iic_drv_init(client) < 0)
			dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
					__func__);
#endif
	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5306_queue_work);
	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue("ft5x0x_ts");
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	/***wait CTP to bootup normally***/
	msleep(200); 

if(ft_pdata->detect)
	touch_type=ft_pdata->detect();
printk("touch_type= %d \n",touch_type);
//write firmware 
#if 1 //QC880 are not to updata fiware 
#if (FT5X0X_FW_UPGRADE_MODE == FT5X0X_FW_UPGRADE_ASYNC)
/* zhengxing: will upgrade firmware async */
    printk("will create ft tp firmware upgrade workqueue...\n");
    ft5x0x_ts->fw_workqueue = create_singlethread_workqueue("ft5x0x_fw");
    if (!ft5x0x_ts->fw_workqueue) {
        err = -ESRCH;
        goto exit_create_singlethread;
    }
    INIT_WORK(&ft5x0x_ts->fw_upgrade_work, ft5306_firmware_upgrade_work);    
#else
/* zhengxing: will upgrade firmware directly */
   if(CTPM_FW_init())
   	{
	fts_register_read(FT5X0X_REG_FIRMID, &reg_version,1);
	FTprintk("[TSP] firmware version = 0x%2x\n", reg_version);
	printk("[TSP] firmware version = 0x%2x\n", reg_version);
	reg_firmware=fts_ctpm_get_upg_ver();
	if ((reg_firmware!= reg_version) &&(reg_firmware!= 0xff))
	{
	  FTprintk("[TSP] start upgrade new verison 0x%2x\n", reg_firmware);
	  msleep(200);
	  err =  fts_ctpm_fw_upgrade_with_i_file();
	  if (err == 0)
	  {
		  FTprintk("[TSP] ugrade successfuly.\n");
		  msleep(300);
		  fts_register_read(FT5X0X_REG_FIRMID, &reg_value,1);
		  FTprintk("FTS_DBG from old version 0x%2x to new version = 0x%2x\n", reg_version, reg_value);
	  }
	  else
	  {
		  FTprintk("[TSP]  ugrade fail err=%d, line = %d.\n",err, __LINE__);
	  }
	  msleep(4000);
	}
   	}
#endif
#endif
	ret = request_irq(ft5x0x_ts->irq, ft5306_interrupt, IRQF_TRIGGER_FALLING, client->dev.driver->name, ft5x0x_ts);
	if (ret < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ft5x0x_ts->irq);
		goto exit_irq_request_fail;
	}
	i2c_set_clientdata(client, ft5x0x_ts);
#ifdef CONFIG_HAS_EARLYSUSPEND
	ft5x0x_ts->ft5306_early_suspend.suspend =ft5306_suspend;
	ft5x0x_ts->ft5306_early_suspend.resume =ft5306_resume;
	ft5x0x_ts->ft5306_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;;
	register_early_suspend(&ft5x0x_ts->ft5306_early_suspend);
#endif
	buf_w[0] = 6;
	err = ft5306_set_regs(client,0x88,buf_w,1);
	buf_r[0] = 0;
	err = ft5306_read_regs(client,0x88,buf_r,1);
	FTprintk("read buf[0x88] = %d\n", buf_r[0]);

#if (FT5X0X_FW_UPGRADE_MODE == FT5X0X_FW_UPGRADE_ASYNC) 
    queue_work(ft5x0x_ts->fw_workqueue, &ft5x0x_ts->fw_upgrade_work);
#endif
	//enable_irq(ft5x0x_ts->irq);
    return 0;

	i2c_set_clientdata(client, NULL);
	free_irq(ft5x0x_ts->irq,ft5x0x_ts);
exit_irq_request_fail:
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
exit_no_irq_fail:
	input_unregister_device(input_dev);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
exit_i2c_test_fail:
	//if (pdata->exit_platform_hw)                              
		//pdata->exit_platform_hw();
	kfree(ft5x0x_ts);
	return err;
}



static struct i2c_device_id ft5306_idtable[] = {
	{ FT5X0X_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ft5306_idtable);

static struct i2c_driver ft5306_driver  = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= FT5X0X_NAME
	},
	.id_table	= ft5306_idtable,
	.probe      = ft5306_probe,
    //.suspend	= ft5306_suspend,
	//.resume	    = ft5306_resume,
	.remove 	= __devexit_p(ft5306_remove),
};

static int __init ft5306_ts_init(void)
{
  ft5606_power_supply(true);
	return i2c_add_driver(&ft5306_driver);
}

static void __exit ft5306_ts_exit(void)
{
	FTprintk("Touchscreen driver of ft5306 exited.\n");
	i2c_del_driver(&ft5306_driver);
}


/***********************************************************************/

module_init(ft5306_ts_init);
module_exit(ft5306_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");

