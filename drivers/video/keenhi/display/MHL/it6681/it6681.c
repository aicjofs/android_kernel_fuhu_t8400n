///*****************************************
//  Copyright (C) 2009-2014
//  ITE Tech. Inc. All Rights Reserved
//  Proprietary and Confidential
///*****************************************
//   @file   <IT6681.c>
//   @author Hermes.Wu@ite.com.tw
//   @date   2013/05/07
//   @fileversion: ITE_IT6681_6607_SAMPLE_1.06
//******************************************/
/*
 * MHL support
 *
 * Copyright (C) 2013 ITE Tech. Inc.
 * Author: Hermes Wu <hermes.wu@ite.com.tw>
 *
 * MHL TX driver for IT6681
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/time.h>
#include "it6681_cfg.h"
#include "it6681_arch.h"
#include "it6681_debug.h"
#include "it6681_def.h"
#include "it6681_io.h"
#include <linux/gpio.h>
#include <linux/it6681.h>
#include <linux/reboot.h>
#include "it6681_drv.h"
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <media/it6681_ext.h>
#define GPIO_USB_MHL_SWITCH 134
#define GPIO_ENVBUS 135




static struct device *it6681dev = NULL;
//static bool boost_sclk;
static int init_it6681_loop_kthread(void);

static void it6681_power_suspend(struct work_struct *work);
static void it6681_power_resume(struct work_struct *work);
//static DECLARE_DELAYED_WORK(suspend_work, it6681_power_suspend);
//static DECLARE_DELAYED_WORK(resume_work, it6681_power_resume);
//static DECLARE_WORK(suspend_work, it6681_power_suspend);
//static DECLARE_WORK(resume_work, it6681_power_resume);

struct it6681_data 
{
    struct it6681_platform_data	*pdata;
    struct it6681_dev_data *ddata;
    struct mutex lock;
    struct work_struct work;
    struct task_struct *it6681_timer_task;
    //struct workqueue_struct *power_cmd_wqs;
    struct work_struct suspend_work;
    struct work_struct resume_work;
#if _SUPPORT_RCP_
    struct input_dev *rcp_input;
#endif

#if _SUPPORT_UCP_
    struct input_dev *ucp_input;
#endif

#if _SUPPORT_UCP_MOUSE_
    struct input_dev *ucp_mouse_input;
#endif
    atomic_t it6681_timer_event;
    atomic_t power_down;
    wait_queue_head_t it6681_wq;
    wait_queue_head_t power_resume_wq;
	
    spinlock_t		spinlock;
    int irq;
    int dev_inited;

};

void delay1ms(unsigned short ms)
{
    msleep(ms);
}

static int i2c_write_reg(struct i2c_client *client, unsigned int offset, u8 value)
{
    int ret;

    ret = i2c_smbus_write_byte_data(client, offset, value);
	if (ret < 0)
	{
        pr_err("IT6681 -- %s error %d, offset=0x%02x, val=0x%02x\n", __FUNCTION__, ret, offset, value);
    }

    return ret;
}

static int i2c_read_reg(struct i2c_client *client, unsigned int offset, u8 *value)
{
	int ret;

	if (!value)
		return -EINVAL;

	ret = i2c_smbus_write_byte(client, offset);
	if (ret < 0)
	{
        pr_err("IT6681 -- %s write error %d, offset=0x%02x\n", __FUNCTION__, ret, offset);
		return ret;
    }
    
	ret = i2c_smbus_read_byte(client);
	if (ret < 0)
	{
        pr_err("IT6681 -- %s read error %d, offset=0x%02x\n", __FUNCTION__, ret, offset);	    
		return ret;
    }		

	*value = ret & 0x000000FF;

	return 0;
}

void hdmirxwr( unsigned char offset, unsigned char value )
{
    struct it6681_data *it6681data = dev_get_drvdata(it6681dev);

    i2c_write_reg(it6681data->pdata->hdmi_rx_client ,(unsigned int)offset,value);

    return ;
}

unsigned char hdmirxrd( unsigned char offset )
{
    unsigned char value=0x00;
    struct it6681_data *it6681data = dev_get_drvdata(it6681dev);
    
    i2c_read_reg(it6681data->pdata->hdmi_rx_client,(unsigned int)offset,&value);

    return value;
}

void hdmitxwr( unsigned char offset, unsigned char value )
{
    struct it6681_data *it6681data = dev_get_drvdata(it6681dev);

    i2c_write_reg(it6681data->pdata->hdmi_tx_client ,(unsigned int)offset,value);

    return ;
}

unsigned char hdmitxrd( unsigned char offset )
{
    unsigned char value=0x00;
    struct it6681_data *it6681data = dev_get_drvdata(it6681dev);

    i2c_read_reg(it6681data->pdata->hdmi_tx_client,(unsigned int)offset,&value);

    return value;
}

void hdmitxbrd( unsigned char offset, void *buffer, unsigned char length )
{
    struct it6681_data *it6681data = dev_get_drvdata(it6681dev);
	int ret;

	ret = i2c_smbus_read_i2c_block_data(it6681data->pdata->hdmi_tx_client, offset, length, (u8*)buffer);
	if (ret < 0)
	{
        pr_err("IT6681 -- %s read error %d, offset=0x%02x\n", __FUNCTION__, ret, offset);	    
    }		
}

void mhltxwr( unsigned char offset, unsigned char value )
{
	struct it6681_data *it6681data = dev_get_drvdata(it6681dev);

	i2c_write_reg(it6681data->pdata->mhl_client ,(unsigned int)offset,value);
}

unsigned char mhltxrd( unsigned char offset )
{
    unsigned char value;
    struct it6681_data *it6681data = dev_get_drvdata(it6681dev);
    
    i2c_read_reg(it6681data->pdata->mhl_client,(unsigned int)offset,&value);

    return value;
}

void hdmirxset( unsigned char offset, unsigned char mask, unsigned char wdata )
{
    unsigned char temp;
        
    temp = hdmirxrd(offset);
    temp = (temp&((~mask)&0xFF))+(mask&wdata);
    hdmirxwr(offset, temp);

    return ;
}

void hdmitxset( unsigned char offset, unsigned char mask, unsigned char wdata )
{
    unsigned char temp;
   
    temp = hdmitxrd(offset);
    temp = (temp&((~mask)&0xFF))+(mask&wdata);
    hdmitxwr(offset, temp);

    return ;
}

void mhltxset( unsigned char offset, unsigned char mask, unsigned char wdata )
{
    unsigned char temp;
    
    temp = mhltxrd(offset);
    temp = (temp&((~mask)&0xFF))+(mask&wdata);
    mhltxwr(offset, temp);

    return;
}

#ifdef _USE_PQ6_TRUE
void set_operation_mode( unsigned char mode )
{
    if ( mode == MODE_USB ) 
    {
        // switch to USB mode

        //gpio_set_value( GPIO_USB_MHL_SWITCH, 1 ); 
    }
    else 
    {
        // switch to MHL mode
       // gpio_set_value( GPIO_USB_MHL_SWITCH, 0 ); 
    }
}
#endif

#ifdef _USE_PQ7_TRUE
void set_vbus_output( unsigned char enable )
{
    if ( enable == 0 ) 
    {
        // disable vbus output
        //gpio_set_value( GPIO_ENVBUS, 1 );
    }
    else 
    {
        // enable vbus output
        //gpio_set_value( GPIO_ENVBUS, 0 );
    }
}
#endif

unsigned long it6681_get_tick_count(void)
{
    struct timespec tt;

    tt = CURRENT_TIME;
    
    return ((tt.tv_sec*1000) + (tt.tv_nsec/1000000L));
}

void SetLED_MHL_Out( char Val )
{
    //LED_MHL_Out = !Val;
}
void SetLED_PathEn( char Val )
{
    //LED_MHL_PathEn = !Val;
}

void SetLED_MHL_CBusEn( char Val )
{
    //LED_MHL_CbusEn = !Val;
}

void SetLED_HDMI_InStable( char Val )
{
    //LED_HDMI_In_Stable = !Val;
}

int it6681_read_edid( void *it6681_dev_data, void *pedid, unsigned short max_length)
{
    pr_err("%s ++, pedid=%p, len=%u\n", __FUNCTION__, pedid, max_length);	
    
    if ( pedid )
    {
        if ( max_length > 512 )
        {
            max_length = 512;    
        }        
        
        memcpy( pedid, it6681_edid_buf, max_length );
        
        return 0; 
    }        
    
    return -1;
}

#if _SUPPORT_RCP_

void rcp_report_event( unsigned char key)
{
    struct it6681_data *it6681data = dev_get_drvdata(it6681dev);
	
	MHL_MSC_DEBUG_PRINTF(("rcp_report_event key: %d\n", key));
	input_report_key(it6681data->rcp_input, (unsigned int)key+1, 1);
	input_report_key(it6681data->rcp_input, (unsigned int)key+1, 0);
	input_sync(it6681data->rcp_input);

}
void mhl_RCP_handler(struct it6681_dev_data *it6681)
{
    rcp_report_event(it6681->rxmsgdata[1]);
}
#endif

#if _SUPPORT_UCP_

void ucp_report_event( unsigned char key)
{
    struct it6681_data *it6681data = dev_get_drvdata(it6681dev);
	
	MHL_MSC_DEBUG_PRINTF(("ucp_report_event key: %d\n", key));
	//rep[REP_DELAY] 
	input_report_key(it6681data->ucp_input, (unsigned int)key+1, 1);
	input_report_key(it6681data->ucp_input, (unsigned int)key+1, 0);
	input_sync(it6681data->ucp_input);

}

void mhl_UCP_handler(struct it6681_dev_data *it6681)
{
    ucp_report_event(it6681->rxmsgdata[1]);
}
#endif

#if _SUPPORT_UCP_MOUSE_
void ucp_mouse_report_event( unsigned char key,int x,int y)
{
	struct it6681_data *it6681data = dev_get_drvdata(it6681dev);
     
    /* Report relative coordinates */
    input_report_rel(it6681data->ucp_mouse_input, REL_X, x);
    input_report_rel(it6681data->ucp_mouse_input, REL_Y, y);

    /* Report key event */
    input_report_key(it6681data->ucp_mouse_input, BTN_LEFT,   ( key&0x01));
    input_report_key(it6681data->ucp_mouse_input, BTN_MIDDLE, ((key>>1)&0x01));
    input_report_key(it6681data->ucp_mouse_input, BTN_RIGHT,  ((key>>2)&0x01));
    
    input_sync(it6681data->ucp_mouse_input);
}

void mhl_UCP_mouse_handler( unsigned char key, int x, int y)
{
    ucp_mouse_report_event(key, x, y);
}
#endif

/*
static int it6681_timer_kthread(void *data)
{
	struct it6681_data *it6681data = dev_get_drvdata(it6681dev);

	atomic_set(&it6681data->it6681_timer_event ,1);

	for(;;){

		wait_event_interruptible_timeout(it6681data->it6681_wq,0,100*HZ/1000);//100ms
        if(atomic_read(&it6681data->it6681_timer_event) == 1){

            hdmitx_irq(it6681data->ddata);
			#if _SUPPORT_HDCP_
            Hdmi_HDCP_handler(it6681data->ddata);
			#endif
			//pr_err("it6681_timer_kthread() -->hdmitx_irq();  \n");
		}
		if(kthread_should_stop())
			break;
	}

	return 0;
} 
*/ 


static void it6681_power_suspend(struct work_struct *work)
{
    struct it6681_data *ddata = container_of(work, struct it6681_data, suspend_work);
    
    debug_6681("%s++ :::::::: \n",__func__);
    
    atomic_set(&ddata->it6681_timer_event ,0);
    atomic_set(&ddata->power_down ,1);
    wake_up(&ddata->it6681_wq);
    disable_irq(ddata->irq);
    hdmitx_pwrdn();
    
    debug_6681("%s-- :::::::: \n",__func__);
}

static void it6681_power_resume(struct work_struct *work)
{
    struct it6681_data *ddata = container_of(work, struct it6681_data, resume_work);

    debug_6681("%s++ :::::::: \n",__func__);

	if(ddata->pdata)
		ddata->pdata->reset();
	
 	it6681_fwinit();
	enable_irq(ddata->irq);
	atomic_set(&ddata->power_down ,0);
       wake_up(&ddata->power_resume_wq);    
    
    debug_6681("%s-- :::::::: \n",__func__);
}

extern bool _it6681_drv_switch_to_mhl(void);
static int _it6681_export_symbol_ready = 0;
bool it6681_drv_switch_to_mhl(void)
{
    struct it6681_data *it6681data;
    
    if ( _it6681_export_symbol_ready == 0 ) {
        printk("%s: _it6681_export_symbol_ready = 0\n",__func__);    
        return false;
    }
    
    if ( it6681dev ) {
        it6681data = dev_get_drvdata(it6681dev);
        
        if ( it6681data ) {
            if ( it6681data->dev_inited == 1 ) {
                
                return _it6681_drv_switch_to_mhl();
            }
            else {
                printk("%s: it6681data->dev_inited =%d \n",__func__, it6681data->dev_inited);    
            }
        }
        else {
            printk("%s: it6681data = NULL\n",__func__);    
        }        
    }
    else {
        printk("%s: it6681dev = NULL\n",__func__);    
    }

    return false;
}

EXPORT_SYMBOL_GPL(it6681_drv_switch_to_mhl);

static int it6681_loop_kthread(void *data)
{
    struct it6681_data *it6681data = dev_get_drvdata(it6681dev);
    int doRegDump=0;
    int loopcount=0 ;
    int pp_mode;
    //int hdcp_mode;

    // HDCP mode : 0 = disable, 1 = enable
//    hdcp_mode = 0;
    // packed pixel mode : 0 = auto, 1 = force
	pp_mode = 0;
        
	//atomic_set(&it6681data->it6681_timer_event ,1);
	debug_6681("%s:=================>\n",__func__);
    it6681_fwinit();
	
    //it6681_set_hdcp( hdcp_mode );
	if ( pp_mode == 1 )
	{
		pr_err("Force packed pixel mode\n");
		it6681_set_packed_pixel_mode( 2 );
	}
	else
	{
	    pr_err("auto packed pixel mode\n");
	    it6681_set_packed_pixel_mode( 1 );
    }        

    it6681data->dev_inited = 1;
    if(it6681data->irq>0)
    enable_irq(it6681data->irq);    
    _it6681_export_symbol_ready = 1;
    
    while(1)
    {
        	
        wait_event_interruptible_timeout(it6681data->it6681_wq,0,100*HZ/1000); //100ms
        
        if(kthread_should_stop()) break;        

        //it6681_irq();
        it6681_poll();

        if ( doRegDump )
        {
            loopcount ++ ;
            if( (loopcount % 1000) == 1 )
            {
                DumpHDMITXReg() ;
            }
            if( loopcount >= 1000 )
            {
                loopcount = 0 ;
            }            
        }
		
	if (atomic_read(&it6681data->power_down) )  {
		 pr_err("%s:======start wait standby=====>\n",__func__);
       		 wait_event_interruptible(it6681data->power_resume_wq, (0 == atomic_read(&it6681data->power_down)));
		 pr_err("%s:======start wake up standby=====>\n",__func__);
	}   		
		
    }

	return 0;
}

static irqreturn_t it6811_irq_thread(int irq, void *data)
{
	struct it6681_data *it6681data = data;
	if ( data && it6681data->dev_inited )
	{
    	mutex_lock(&it6681data->lock);
	MHL_MSC_DEBUG_PRINTF(("IT6681 -- %s start exe irq handler\n", __FUNCTION__));	
    	it6681_irq();
    	mutex_unlock(&it6681data->lock);
	}

	return IRQ_HANDLED;
}

static int init_it6681_loop_kthread(void)
{
	struct it6681_data *it6681data = dev_get_drvdata(it6681dev);

	pr_err("IT6681 -- %s ++\n", __FUNCTION__);	

	init_waitqueue_head(&it6681data->power_resume_wq);
	init_waitqueue_head(&it6681data->it6681_wq);
	it6681data->it6681_timer_task = kthread_create(it6681_loop_kthread,NULL,"it6681_loop_kthread");
    wake_up_process(it6681data->it6681_timer_task);

	return 0;
}

//
// ioctl interfaces
//

enum {
    IT668X_I2C_HDMI_RX = 1,
    IT668X_I2C_HDMI_TX,
    IT668X_I2C_MHL_TX,
};

struct it668x_ioctl_reg_op {
    unsigned char i2c_dev;
    unsigned char offset;
    unsigned char bank;
    unsigned char length;
    void *buffer;
};

struct it668x_ioctl_reg_op_set {
    unsigned char i2c_dev;
    unsigned char offset;
    unsigned char bank;
    unsigned char data;
    unsigned char mask;
};

enum {
    IT668X_CTL_HOLD,
    IT668X_CTL_RELEASE,
    IT668X_CTL_MAX,
};

struct it668x_ioctl_data {
    unsigned char op;
    unsigned char rop;
};

enum {
    IT668X_DVAR_GET_LEN,
    IT668X_DVAR_GET,
    IT668X_DVAR_SET,
};

struct it668x_ioctl_driver_var {
    unsigned char op;

    union {
        struct {
            unsigned long driver_var_size;
        } getlen;

        struct {
            unsigned long length;
            void *buffer;
        }get;

        struct {
            unsigned long offset;
            unsigned long length;
            void *buffer;
        }set;
    }; 
     
};

struct it668x_ioctl_read_edid {
    unsigned char op;
    unsigned char status;
    unsigned short length;
    void *buffer;
};

#define IT668X_DEV_MAJOR 123
#define IT668X_DEV_IOCTLID 0xD0

#define	IT668X_IOCTL_READ_REG	_IOR(IT668X_DEV_IOCTLID, 1, struct it668x_ioctl_reg_op)
#define	IT668X_IOCTL_WRITE_REG	_IOW(IT668X_DEV_IOCTLID, 2, struct it668x_ioctl_reg_op)
#define	IT668X_IOCTL_SET_REG	_IOW(IT668X_DEV_IOCTLID, 3, struct it668x_ioctl_reg_op)
#define	IT668X_IOCTL_CONTROL	_IOW(IT668X_DEV_IOCTLID, 4, struct it668x_ioctl_data)
#define	IT668X_IOCTL_DRIVER_VAR	_IOW(IT668X_DEV_IOCTLID, 5, struct it668x_ioctl_driver_var)
#define	IT668X_IOCTL_READ_EDID	_IOR(IT668X_DEV_IOCTLID, 7, struct it668x_ioctl_read_edid)

ssize_t it668x_fop_read(struct file * file,char * buf,size_t count,loff_t * f_ops)
{
    return -1;
}

ssize_t it668x_fop_write (struct file * file,const char * buf, size_t count,loff_t * f_ops)
{
    return -1;
}

int it668x_ioctl_driver_var_handler(unsigned long arg)
{
    struct it668x_ioctl_driver_var *devop = (struct it668x_ioctl_driver_var*)arg;

    switch ( devop->op )
    {
        case IT668X_DVAR_GET_LEN:
            break;

        case IT668X_DVAR_GET:
            break;

        case IT668X_DVAR_SET:
            break;
    }

    return -1;
}

int it668x_ioctl_control_handler(unsigned long arg)
{
    struct it668x_ioctl_data *devop = (struct it668x_ioctl_data*)arg;
    struct it6681_data *ddata = dev_get_drvdata(it6681dev);
    
    switch( devop->op )
    {
        case IT668X_CTL_HOLD:
            if ( ddata ) {
            	atomic_set(&ddata->power_down ,1);
            	wake_up(&ddata->it6681_wq);
            	disable_irq(ddata->irq);            
                devop->rop = 0xff;                
            }
            break;        
            
        case IT668X_CTL_RELEASE:
            if ( ddata ) {
        	    enable_irq(ddata->irq);
        	    atomic_set(&ddata->power_down ,0);
                wake_up(&ddata->power_resume_wq);               
                devop->rop = 0xfe;
            }
            break;  
    }
    
    return 0;
}

int it668x_ioctl_read_edid_handler(unsigned long arg)
{
    return -1;
}

//ssize_t GPIO_VIB_ioctl(struct inode *  inode,struct file * file,unsigned int cmd, long data)
long it668x_fop_ioctl( struct file *file, unsigned int cmd, unsigned long arg )
{
    struct it668x_ioctl_reg_op *regop = (struct it668x_ioctl_reg_op*)arg;
    struct it668x_ioctl_reg_op_set *regopset = (struct it668x_ioctl_reg_op_set*)arg;
    int i;
    unsigned char *buf;

    switch (cmd) {
        case IT668X_IOCTL_READ_REG:
            switch ( regop->i2c_dev )
            {
                case IT668X_I2C_HDMI_RX: 
                    buf = (unsigned char*)regop->buffer;
                    for ( i=0 ; i<regop->length ; i++ )
                    {
                        buf[i] = hdmirxrd(regop->offset+i);
                    }
                    break;
                case IT668X_I2C_HDMI_TX:
                    buf = (unsigned char*)regop->buffer;
                    for ( i=0 ; i<regop->length ; i++ )
                    {
                        buf[i] = hdmitxrd(regop->offset+i);
                    }
                    break;
                case IT668X_I2C_MHL_TX:
                    buf = (unsigned char*)regop->buffer;
                    for ( i=0 ; i<regop->length ; i++ )
                    {
                        buf[i] = mhltxrd(regop->offset+i);
                    }
                    break;
                default:
                    break;
            }
            break; 

        case IT668X_IOCTL_WRITE_REG:
            switch ( regop->i2c_dev )
            {
                case IT668X_I2C_HDMI_RX: 
                    buf = (unsigned char*)regop->buffer;
                    for ( i=0 ; i<regop->length ; i++ )
                    {
                        hdmirxwr(regop->offset+i, buf[i]);
                    }
                    break;
                case IT668X_I2C_HDMI_TX:
                    buf = (unsigned char*)regop->buffer;
                    for ( i=0 ; i<regop->length ; i++ )
                    {
                        hdmitxwr(regop->offset+i, buf[i]);
                    }
                    break;
                case IT668X_I2C_MHL_TX:
                    buf = (unsigned char*)regop->buffer;
                    for ( i=0 ; i<regop->length ; i++ )
                    {
                        mhltxwr(regop->offset+i, buf[i]);
                    }
                    break;
                default:
                    break;
            }
            break; 

        case IT668X_IOCTL_SET_REG:
            switch ( regopset->i2c_dev )
            {
                case IT668X_I2C_HDMI_RX: 
                    hdmirxset(regopset->offset, regopset->mask, regopset->data);
                    break;
                case IT668X_I2C_HDMI_TX:
                    hdmitxset(regopset->offset, regopset->mask, regopset->data);
                    break;
                case IT668X_I2C_MHL_TX:
                    mhltxset(regopset->offset, regopset->mask, regopset->data);
                    break;
                default:
                    break;
            }
            break;

        case IT668X_IOCTL_CONTROL:
            it668x_ioctl_control_handler(arg);
            break;

        case IT668X_IOCTL_DRIVER_VAR:
            it668x_ioctl_driver_var_handler(arg);
            break;

        case IT668X_IOCTL_READ_EDID:
            it668x_ioctl_read_edid_handler(arg);
            break;

        default:
        return -1;
    }
    return 0;
}

ssize_t it668x_fop_open(struct inode * inode,struct file * file)
{
    printk("%s++\n", __FUNCTION__);
    return 0;
}
ssize_t it668x_fop_release(struct inode * inode, struct file * file)
{
    printk("%s++\n", __FUNCTION__);
    return 0;
}

static const struct file_operations it668x_file_ops={
    .owner        = THIS_MODULE,
    .open    = it668x_fop_open,
    .read    =it668x_fop_read,
    .write    =it668x_fop_write,
    //.ioctl     = it668x_fop_ioctl,
    .unlocked_ioctl = it668x_fop_ioctl,
    .release     =it668x_fop_release,
};

static struct class *it668x_dev_class;
static int it668x_ioctl_init(struct class *dev_class)
{
    struct device *dev;
    int ret  = -ENODEV;

    ret = register_chrdev(IT668X_DEV_MAJOR, "it668x", &it668x_file_ops);
    if (ret < 0) {
        printk(KERN_ERR "VIB: unable to get major %d/n", ret);
        return ret;
    }

    if ( dev_class == NULL )
    {
        it668x_dev_class = class_create( THIS_MODULE, "mhl" ); 
        if (IS_ERR(it668x_dev_class)) {
            unregister_chrdev(IT668X_DEV_MAJOR, "it668x");
            return PTR_ERR(it668x_dev_class);
        }
    }

    // create a point under /dev/it668x0
    dev = device_create(dev_class, NULL, MKDEV(IT668X_DEV_MAJOR, 0), NULL, "it668x%d", 0);
    if ( dev == NULL ) {
        printk(KERN_ERR "VIB: unable to get major %d/n", ret);
        return (-ENODEV);
    }

    return 0;
}
static int __devinit it6681_hdmi_rx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
	struct it6681_data *ddata;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct class *sec_mhl;	

#if _SUPPORT_RCP_
	struct input_dev *rcpinput;
#endif

#if _SUPPORT_UCP_
	struct input_dev *ucpinput;
#endif

#if _SUPPORT_UCP_MOUSE_
	struct input_dev *mouse_ucpinput;
#endif

    pr_err("IT6681 -- %s ++\n", __FUNCTION__);	

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    {
		pr_err(" i2c_check_functionality()    FAIL!!!) \n");
		return -EIO;
	}

	ddata = kzalloc(sizeof(struct it6681_data), GFP_KERNEL);

	if (!ddata) 
    {
		pr_err("IT6681 -- failed to allocate driver data\n");
		return -ENOMEM;
	}

    ddata->ddata = get_it6681_dev_data();
    ddata->pdata = client->dev.platform_data;
    if(ddata->pdata){
		ddata->pdata->reset();
		ddata->pdata->read_edid = it6681_fw_read_edid_cb;
		 if(ddata->pdata->register_resume)
			ddata->pdata->register_resume(ddata->pdata);
	}
    
    ddata->pdata->hdmi_rx_client = client;
    atomic_set(&ddata->it6681_timer_event ,1);
    atomic_set(&ddata->power_down,0);
	ddata->it6681_timer_task = NULL;
    ddata->dev_inited = 0;
    mutex_init(&ddata->lock);
    #if 0
    ddata->power_cmd_wqs = create_singlethread_workqueue("it6681-power_wq");
    if (ddata->power_cmd_wqs == NULL){
        pr_err(" it6681 - create_workqueue(  it6681-power_wq   ) FAIL !!! \n");
        ret = -ENXIO;
    }	
	#endif
        
    INIT_WORK(&ddata->suspend_work, it6681_power_suspend);
    INIT_WORK(&ddata->resume_work, it6681_power_resume);
    spin_lock_init(&ddata->spinlock);
    i2c_set_clientdata(client, ddata);

	it6681dev = &client->dev;
	ddata->irq = client->irq;
	//init_waitqueue_head(&ddata->it6681_wq);

    // init GPIO begin
    // This is only necessary for IT6682 with external USB switch
#ifdef _USE_PQ6_TRUE
    ret = gpio_request( GPIO_USB_MHL_SWITCH, "USB_MHL_SWITCH");
    
    if (ret<0)
        pr_err("IT6681 -- %s: gpio_request failed for gpio %d\n", __func__, GPIO_USB_MHL_SWITCH);
    else
        gpio_direction_output( GPIO_USB_MHL_SWITCH, 1 );
#endif

#ifdef _USE_PQ7_TRUE
    ret = gpio_request( GPIO_ENVBUS, "EnVBUS");
    
    if (ret<0)
        pr_err("IT6681 -- %s: gpio_request failed for gpio %d\n", __func__, GPIO_ENVBUS);
    else
        gpio_direction_output(GPIO_ENVBUS, 0);
#endif
    // init GPIO end    
        
    // init interrupt pin
    if(client->irq>0){
	ret = request_threaded_irq(client->irq, NULL, it6811_irq_thread,
                               IRQF_TRIGGER_LOW | IRQF_ONESHOT|IRQF_DISABLED,
                               "it6681", ddata);
	if (ret < 0)
		goto err_exit;
    

	disable_irq(client->irq);
	}


#if _SUPPORT_RCP_
    rcpinput = input_allocate_device();
	if (!rcpinput) 
    {
	    pr_err("IT6681 --failed to allocate RCP input device.\n");
		ret =  -ENOMEM;
		goto err_exit;
	}

	set_bit(EV_KEY, rcpinput->evbit);
	bitmap_fill(rcpinput->keybit, KEY_MAX);

	ddata->rcp_input = rcpinput;

	input_set_drvdata(rcpinput, ddata);
	rcpinput->name = "it6681_rcp";
	rcpinput->id.bustype = BUS_I2C;

	ret = input_register_device(rcpinput);
	if (ret < 0) 
    {
        pr_err("IT6681 --fail to register RCP input device. ret = %d (0x%08x)\n", ret, ret);
		goto err_exit_rcp;
	}
#endif

#if _SUPPORT_UCP_
	ucpinput = input_allocate_device();
	if (!ucpinput) 
    {
		pr_err("IT6681 --failed to allocate UCP input device.\n");
		ret =  -ENOMEM;

		#if _SUPPORT_RCP_
		goto err_exit_rcp;
		#endif

		goto err_exit;
	}

	set_bit(EV_KEY, ucpinput->evbit);
	bitmap_fill(ucpinput->keybit, KEY_MAX);

	ddata->ucp_input = ucpinput;

	input_set_drvdata(ucpinput, ddata);
	ucpinput->name = "it6681_ucp";
	ucpinput->id.bustype = BUS_I2C;

	ret = input_register_device(ucpinput);
	if (ret < 0) 
    {
		pr_err("IT6681 --fail to register UCP input device. ret = %d (0x%08x)\n", ret, ret);
		goto err_exit_ucp;
	}
#endif

#if _SUPPORT_UCP_MOUSE_
	mouse_ucpinput = input_allocate_device();
	if (!mouse_ucpinput) 
    {
		pr_err("IT6811 --failed to allocate UCP  MOUSE input device.\n");
		ret =  -ENOMEM;

		#ifdef _SUPPORT_UCP_
		goto err_exit_ucp;
		#endif
		
		#ifdef _SUPPORT_RCP_
		goto err_exit_rcp;
		#endif

		goto err_exit;
	}
	
    /* Announce that the virtual mouse will generate relative coordinates */
    set_bit(EV_REL, mouse_ucpinput->evbit);
    set_bit(REL_X, mouse_ucpinput->relbit);
    set_bit(REL_Y, mouse_ucpinput->relbit);
    set_bit(REL_WHEEL, mouse_ucpinput->relbit);

    /* Announce key event */
    set_bit(EV_KEY, mouse_ucpinput->evbit);
    set_bit(BTN_LEFT, mouse_ucpinput->keybit);
    set_bit(BTN_MIDDLE, mouse_ucpinput->keybit);
    set_bit(BTN_RIGHT, mouse_ucpinput->keybit);	
	
	ddata->ucp_mouse_input = mouse_ucpinput;
	
	input_set_drvdata(mouse_ucpinput, ddata);
	mouse_ucpinput->name = "it6811_ucp_mouse";
	mouse_ucpinput->id.bustype = BUS_I2C;

	ret = input_register_device(mouse_ucpinput);
	if (ret < 0) 
    {
		pr_err("IT6811 --fail to register UCP MOUSE input device. ret = %d (0x%08x)\n", ret, ret);
		goto err_exit_ucp_mouse;
	}
#endif

    sec_mhl = class_create(THIS_MODULE, "mhl");
    if (IS_ERR(&sec_mhl)) {
        pr_info("failed to create class sec_mhl");
        goto err_exit;
    }

    it668x_ioctl_init(sec_mhl);
	
	
#if 0
	ret = device_create_file(&client->dev, &dev_attr_boost_sclk);
	if (ret) {
		dev_err(&client->dev,
			"%s: device_create_file failed\n", __func__);
		//goto err_req_irq_pend;
	}
#endif
    pr_err("IT6681 -- %s --\n", __FUNCTION__);	
       
	return 0;

#if _SUPPORT_UCP_MOUSE_	
err_exit_ucp_mouse:	
	input_free_device(mouse_ucpinput);
#endif

#if _SUPPORT_UCP_
err_exit_ucp:
	input_free_device(ucpinput);
#endif

#if _SUPPORT_RCP_
err_exit_rcp:
	input_free_device(rcpinput);
#endif

err_exit:
	kfree(ddata);

    pr_err( "IT6811 %s--, ret=%d ( 0x%08x )\n", __FUNCTION__, ret, ret );

    return ret;
}

static int __devinit it6681_hdmi_tx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct it6681_platform_data *pdata = client->dev.platform_data;

    pr_err("%s ++\n", __FUNCTION__);	
	pdata->hdmi_tx_client = client;

	return 0;
}


static int __devinit it6681_mhl_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct it6681_platform_data *pdata = client->dev.platform_data;

    pr_err("%s ++\n", __FUNCTION__);	
	pdata->mhl_client = client;

	return 0;
}


static int __devexit it6681_hdmi_rx_remove(struct i2c_client *client)
{
    struct it6681_data *it6681data;
    
    pr_err("%s ++\n", __FUNCTION__);	
    
    it6681data = i2c_get_clientdata(client);
    //device_remove_file(&client->dev, &dev_attr_boost_sclk);
    if(it6681data->ddata->Aviinfo != NULL)
    {
        kfree(it6681data->ddata->Aviinfo);
    }
   
   //destroy_workqueue(it6681data->power_cmd_wqs);
    kfree(it6681data);

	return 0;
}

static int __devexit it6681_hdmi_tx_remove(struct i2c_client *client)
{
    pr_err("%s ++\n", __FUNCTION__);	
	return 0;
}

static int __devexit  it6681_mhl_remove(struct i2c_client *client)
{
    pr_err("%s ++\n", __FUNCTION__);	
	return 0;
}

static const struct i2c_device_id it6681_hdmi_rx_id[] = {
	{"it6681_hdmi_rx", 0},
	{}
};

static const struct i2c_device_id it6681_hdmi_tx_id[] = {
	{"it6681_hdmi_tx", 0},
	{}
};

static const struct i2c_device_id it6681_mhl_id[] = {
	{"it6681_mhl", 0},
	{}
};

static ssize_t it6681_i2c_early_suspend(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct it6681_data *ddata = i2c_get_clientdata(client);
	
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val)
		;
	else if (!val)
		;

	//boost_sclk = val;
	printk("%s:==========>\n",__func__);
	return count;
}

DEVICE_ATTR(it6681_atter, S_IWUSR, NULL, it6681_i2c_early_suspend);
#ifdef CONFIG_PM_SLEEP
static int it6681_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct it6681_data *ddata = i2c_get_clientdata(client);
	printk("%s:=================> 1\n",__func__);

	cancel_work_sync(&ddata->resume_work);
	printk("%s:=================>2 \n",__func__);
	//--v1---------------------------------------
	atomic_set(&ddata->it6681_timer_event ,0);
	atomic_set(&ddata->power_down ,1);
	wake_up(&ddata->it6681_wq);
	disable_irq(ddata->irq);
    	hdmitx_pwrdn();
    //-----------------------------------------
    
    //--v2---------------------------------------
    //queue_work(ddata->power_cmd_wqs, &ddata->suspend_work);
    //-----------------------------------------
        
	return 0;
}

static int it6681_i2c_resume(struct device *dev)
{
	unsigned long flags;
	struct i2c_client *client = to_i2c_client(dev);
	struct it6681_data *ddata = i2c_get_clientdata(client);
	debug_6681("%s:=================> \n",__func__);
	
	//--v1---------------------------------------
	//atomic_set(&ddata->it6681_timer_event ,1);
	//schedule_work(&ddata->work);
	//-----------------------------------------
	
	//--v2---------------------------------------
    //if(ddata->pdata)
    //    ddata->pdata->reset();
    
    //it6681_fwinit();
    //enable_irq(ddata->irq);
    //atomic_set(&ddata->power_down ,0);
    //wake_up(&ddata->power_resume_wq); 	
    //-----------------------------------------
    spin_lock_irqsave(&ddata->spinlock, flags);
	printk("%s:=================> \n",__func__);
    //--v3---------------------------------------
    //queue_work(ddata->power_cmd_wqs, &ddata->resume_work);    
	//-----------------------------------------
       //__cancel_delayed_work(&touch->dwork);
	schedule_work(&ddata->resume_work);
    spin_unlock_irqrestore(&ddata->spinlock, flags);
	return 0;
}
#endif

static int it6681_i2c_poweroff(void)
{
    unsigned char tmp;
    debug_6681("%s:=================> \n",__func__);
    
    tmp = mhltxrd(0x0f);
    debug_6681("%s:=================> mhltx_0x0f = 0x%02x \n",__func__, (int)tmp);
    
    if ( 0 == (tmp & 0x01) ) { // if mhl mode, pull down CBUS
        mhltxset(0x0f, 0x10, 0x10);
        hdmitxset(0x64, 0x80, 0x80);
        
        debug_6681("%s:=================> mhltx_0x0f = 0x%02x \n",__func__, (int)mhltxrd(0x0f));
        debug_6681("%s:================> hdmitx_0x64 = 0x%02x \n",__func__, (int)hdmitxrd(0x64));
    }
	return 0;
}




MODULE_DEVICE_TABLE(i2c, it6681_hdmi_rx_id);
MODULE_DEVICE_TABLE(i2c, it6681_hdmi_tx_id);
MODULE_DEVICE_TABLE(i2c, it6681_mhl_id);
#ifdef CONFIG_PM_SLEEP
static SIMPLE_DEV_PM_OPS(it6681_i2c_pm, it6681_i2c_suspend, it6681_i2c_resume);
#endif

static struct i2c_driver it6681_hdmi_rx_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "it6681_hdmi_rx",
#ifdef CONFIG_PM_SLEEP
		.pm	= &it6681_i2c_pm,
#endif
	},
	.id_table = it6681_hdmi_rx_id,
	.probe = it6681_hdmi_rx_i2c_probe,
	.remove = __devexit_p(it6681_hdmi_rx_remove),

};

static struct i2c_driver it6681_hdmi_tx_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "it6681_hdmi_tx",
	},
	.id_table = it6681_hdmi_tx_id,
	.probe = it6681_hdmi_tx_i2c_probe,
	.remove = __devexit_p(it6681_hdmi_tx_remove),

};

static struct i2c_driver it6681_mhl_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "it6681_mhl",
	},
	.id_table = it6681_mhl_id,
	.probe = it6681_mhl_i2c_probe,
	.remove = __devexit_p(it6681_mhl_remove),
};
static int it6681_power_off_cb(struct notifier_block *nb,
		unsigned long event, void *unused)
{
	printk(KERN_INFO "it6681: rebooting cleanly.\n");
	switch (event) {
	case SYS_RESTART:
	case SYS_HALT:
	case SYS_POWER_OFF:
		it6681_i2c_poweroff();
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}
struct notifier_block it6681_reboot_notifier = {
	.notifier_call = it6681_power_off_cb,
};

static int __init it6681_init(void)
{
	int ret;

    pr_err("it6681_init ++\n");

	ret = i2c_add_driver(&it6681_hdmi_rx_i2c_driver);
	if (ret < 0)
	{
	    pr_err("i2c_add_driver it6681_hdmi_rx_i2c_driver fail\n");	
		return ret;
	}
	ret = register_reboot_notifier(&it6681_reboot_notifier);
	if (ret){
		pr_err("it6681_init  register reboot notifier fail!!!!!!!!!\n");
	}
	msleep(20);
	ret = i2c_add_driver(&it6681_hdmi_tx_i2c_driver);
	if (ret < 0)
		goto err_exit0;

	ret = i2c_add_driver(&it6681_mhl_i2c_driver);
	if (ret < 0)
		goto err_exit1;

	if(ret <0)
		goto err_exit2;

	init_it6681_loop_kthread();

    pr_err("it6681_init done\n");

	return 0;

err_exit2:
	i2c_del_driver(&it6681_mhl_i2c_driver);
	pr_err("i2c_add_driver hdmitx_ini fail\n");
err_exit1:
	i2c_del_driver(&it6681_hdmi_tx_i2c_driver);
	pr_err("i2c_add_driver it6681_mhl_i2c_driver fail\n");
err_exit0:
	i2c_del_driver(&it6681_hdmi_tx_i2c_driver);
	pr_err("i2c_add_driver it6681_hdmi_tx_i2c_driver fail\n");	
	
	return ret;
}

static void __exit it6681_exit(void)
{
	struct it6681_data *it6681data = dev_get_drvdata(it6681dev);
	if(it6681data&&it6681data->it6681_timer_task){  
  
                kthread_stop(it6681data->it6681_timer_task);  
  
                it6681data->it6681_timer_task = NULL;  
  
            }  
    	i2c_del_driver(&it6681_mhl_i2c_driver);
	i2c_del_driver(&it6681_hdmi_tx_i2c_driver);
	i2c_del_driver(&it6681_hdmi_rx_i2c_driver);	
}

module_init(it6681_init);
module_exit(it6681_exit);
