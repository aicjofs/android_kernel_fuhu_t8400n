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
#include <linux/interrupt.h>
#include <linux/it6681.h>
#include "it6681_drv.h"

#define GPIO_USB_MHL_SWITCH 134
#define GPIO_ENVBUS 135
static struct device *it6681dev;
static struct i2c_device_id mhl_it6681_idtable[] = {
	{DEV_HMDI_RX, 0},
	{DEV_HMDI_TX, 0},
	{DEV_MHL, 0},
	{},
};
static int init_it6681_loop_kthread(void);

struct it6681_data 
{
    struct it6681_platform_data	*pdata;
    struct it6681_dev_data *ddata;
    int irq;
    int dev_inited;
    struct mutex lock;
    wait_queue_head_t it6681_wq;
    struct task_struct *it6681_timer_task;
    atomic_t it6681_timer_event;

#if _SUPPORT_RCP_
    struct input_dev *rcp_input;
#endif

#if _SUPPORT_UCP_
    struct input_dev *ucp_input;
#endif

#if _SUPPORT_UCP_MOUSE_
    struct input_dev *ucp_mouse_input;
#endif
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

void set_operation_mode( unsigned char mode )
{
    if ( mode == MODE_USB ) 
    {
        // switch to USB mode
        gpio_set_value( GPIO_USB_MHL_SWITCH, 1 ); 
    }
    else 
    {
        // switch to MHL mode
        gpio_set_value( GPIO_USB_MHL_SWITCH, 0 ); 
    }
}

void set_vbus_output( unsigned char enable )
{
    if ( enable == 0 ) 
    {
        // disable vbus output
        gpio_set_value( GPIO_ENVBUS, 1 );
    }
    else 
    {
        // enable vbus output
        gpio_set_value( GPIO_ENVBUS, 0 );
    }
}

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

static int it6681_loop_kthread(void *data)
{
    struct it6681_data *it6681data = dev_get_drvdata(it6681dev);
    int doRegDump=1;
    int loopcount=0 ;
    int pp_mode;
    int hdcp_mode;

    // HDCP mode : 0 = disable, 1 = enable
    hdcp_mode = 0;
    // packed pixel mode : 0 = auto, 1 = force
	pp_mode = 0;
        
	atomic_set(&it6681data->it6681_timer_event ,1);

    it6681_fwinit();
	
    it6681_set_hdcp( hdcp_mode );
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
    enable_irq(it6681data->irq);    

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
    }

	return 0;
}

static irqreturn_t it6811_irq_thread(int irq, void *data)
{
	struct it6681_data *it6681data = data;

	if ( data && it6681data->dev_inited )
	{
    	mutex_lock(&it6681data->lock);
    	it6681_irq();
    	mutex_unlock(&it6681data->lock);
	}

	return IRQ_HANDLED;
}

static int init_it6681_loop_kthread(void)
{
	struct it6681_data *it6681data = dev_get_drvdata(it6681dev);

	pr_err("IT6681 -- %s ++\n", __FUNCTION__);	

	init_waitqueue_head(&it6681data->it6681_wq);
	it6681data->it6681_timer_task = kthread_create(it6681_loop_kthread,NULL,"it6681_loop_kthread");
    wake_up_process(it6681data->it6681_timer_task);

	return 0;
}
static int it6681_mhl_reset(struct it6681_platform_data *pdata)
{

	if (pdata&&pdata->reset){
		pdata->reset();
		return 0;
	}
	return -EIO;
}

static int __devinit it6681_hdmi_rx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
	struct it6681_data *ddata;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

#if _SUPPORT_RCP_
	struct input_dev *rcpinput;
#endif

#if _SUPPORT_UCP_
	struct input_dev *ucpinput;
#endif

#ifdef _SUPPORT_UCP_MOUSE_
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
    if(it6681_mhl_reset(ddata->pdata)){
		printk("/nCan't find the reset function in your platform file============================================\n");
		return -EIO;
    }
    msleep(50);
    ddata->pdata->hdmi_rx_client = client;
    atomic_set(&ddata->it6681_timer_event ,0);
	ddata->it6681_timer_task = NULL;
    ddata->dev_inited = 0;
	mutex_init(&ddata->lock);

    i2c_set_clientdata(client, ddata);

	it6681dev = &client->dev;
	ddata->irq = client->irq;
	init_waitqueue_head(&ddata->it6681_wq);

    // init GPIO begin
    // This is only necessary for IT6682 with external USB switch
    ret = gpio_request( GPIO_USB_MHL_SWITCH, "USB_MHL_SWITCH");
    
    if (ret<0)
        pr_err("IT6681 -- %s: gpio_request failed for gpio %d\n", __func__, GPIO_USB_MHL_SWITCH);
    else
        gpio_direction_output( GPIO_USB_MHL_SWITCH, 1 );

    ret = gpio_request( GPIO_ENVBUS, "EnVBUS");
    
    if (ret<0)
        pr_err("IT6681 -- %s: gpio_request failed for gpio %d\n", __func__, GPIO_ENVBUS);
    else
        gpio_direction_output(GPIO_ENVBUS, 0);    
    // init GPIO end    
        
    // init interrupt pin
    #if 1
	ret = request_threaded_irq(client->irq, NULL, it6811_irq_thread,
                               IRQF_TRIGGER_LOW | IRQF_ONESHOT|IRQF_DISABLED,
                               "it6681", ddata);
	if (ret < 0)
		goto err_exit;

	disable_irq(client->irq);
    #endif

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

#ifdef _SUPPORT_UCP_MOUSE_
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

static bool_t match_id(const struct i2c_device_id *id, const struct i2c_client *client)
{
	if (strcmp(client->name, id->name) == 0)
		return true;

	return false;
}
static int __devinit mhl_it6681_probe(struct i2c_client *client,
			const struct i2c_device_id *dev_id)
{
	int ret = 0;
	struct it6681_platform_data *pdata;

	struct it6681_data *ddata;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

#if _SUPPORT_RCP_
	struct input_dev *rcpinput;
#endif

#if _SUPPORT_UCP_
	struct input_dev *ucpinput;
#endif

#ifdef _SUPPORT_UCP_MOUSE_
	struct input_dev *mouse_ucpinput;
#endif

	pr_err("IT6681 -- %s ++\n", __FUNCTION__);	

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    	{
		pr_err(" i2c_check_functionality()    FAIL!!!) \n");
		return -EIO;
	}

	
	
	
	
	if(match_id(&mhl_it6681_idtable[0], client))
	{
	       pdata = client->dev.platform_data;
		pdata->hdmi_rx_client = client;
		ddata = kzalloc(sizeof(struct it6681_data), GFP_KERNEL);

		if (!ddata) 
	    	{
			pr_err("IT6681 -- failed to allocate driver data\n");
			return -ENOMEM;
		}

		ddata->ddata = get_it6681_dev_data();
		ddata->pdata = pdata;
		ddata->irq = client->irq;
		it6681dev = &client->dev;
		i2c_set_clientdata(client, ddata);

		atomic_set(&ddata->it6681_timer_event ,0);
		ddata->it6681_timer_task = NULL;
		ddata->dev_inited = 0;
		mutex_init(&ddata->lock);

		init_waitqueue_head(&ddata->it6681_wq);

		// init GPIO begin
		// This is only necessary for IT6682 with external USB switch
		ret = gpio_request( GPIO_USB_MHL_SWITCH, "USB_MHL_SWITCH");
    
		if (ret<0)
			pr_err("IT6681 -- %s: gpio_request failed for gpio %d\n", __func__, GPIO_USB_MHL_SWITCH);
		else
			gpio_direction_output( GPIO_USB_MHL_SWITCH, 1 );

		ret = gpio_request( GPIO_ENVBUS, "EnVBUS");
    
		if (ret<0)
			pr_err("IT6681 -- %s: gpio_request failed for gpio %d\n", __func__, GPIO_ENVBUS);
		else
		gpio_direction_output(GPIO_ENVBUS, 0);    
		// init GPIO end    
        
		// init interrupt pin

		ret = request_threaded_irq(ddata->irq, NULL, it6811_irq_thread,
	                               IRQF_TRIGGER_LOW | IRQF_ONESHOT|IRQF_DISABLED,
	                               "it6681", ddata);
		if (ret < 0)
			goto err_exit;

		disable_irq(ddata->irq);


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

#ifdef _SUPPORT_UCP_MOUSE_
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
		
		dev_info(&client->adapter->dev, "attached %s "
			"into i2c adapter successfully\n", dev_id->name);
	}
	else if(match_id(&mhl_it6681_idtable[1], client))
	{
		pdata = client->dev.platform_data;
		pdata->hdmi_tx_client = client;
		dev_info(&client->adapter->dev, "attached %s "
			"into i2c adapter successfully \n", dev_id->name);
	}
	else if(match_id(&mhl_it6681_idtable[2], client))
	{
		pdata = client->dev.platform_data;
		pdata->mhl_client= client;
		dev_info(&client->adapter->dev, "attached %s "
			"into i2c adapter successfully\n", dev_id->name);
	}
	else
	{
		dev_info(&client->adapter->dev, "invalid i2c adapter: can not found dev_id matched\n");
		return -EIO;
	}

	if(pdata&&pdata->hdmi_rx_client != NULL 
		&&pdata->hdmi_tx_client != NULL 
		&& pdata->mhl_client != NULL)
	{
		printk("\n============================================\n");
		printk("it6681 Driver Version based on 8051 driver Version 1.0066 \n");
		printk("============================================\n");
    
    		pr_err("%s ++\n", __FUNCTION__);	
		if(it6681_mhl_reset(pdata)){
			printk("/nCan't find the reset function in your platform file============================================\n");
			return -EIO;
		}
    		
	init_it6681_loop_kthread();
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
	}

}

static int mhl_it6681_remove(struct i2c_client *client)
{	

	return 0;
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
    
    if(it6681data->ddata->Aviinfo != NULL)
    {
        kfree(it6681data->ddata->Aviinfo);
    }
    
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

MODULE_DEVICE_TABLE(i2c, it6681_hdmi_rx_id);
MODULE_DEVICE_TABLE(i2c, it6681_hdmi_tx_id);
MODULE_DEVICE_TABLE(i2c, it6681_mhl_id);

static struct i2c_driver it6681_hdmi_rx_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "it6681_hdmi_rx",
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
static struct i2c_driver mhl_it6681_driver = {
	.driver = {
		.name = "It6681_Driver",
	},
	.id_table = mhl_it6681_idtable,
	.probe = mhl_it6681_probe,
	.remove = __devexit_p(it6681_hdmi_rx_remove),

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

static int __init mhl_it6681_init(void)
{
	return i2c_add_driver(&mhl_it6681_driver);
}
static int __init it6681_init(void)
{
	int ret;
    
    pr_err("it6681_init ++\n");
#if 0
	ret = i2c_add_driver(&it6681_hdmi_rx_i2c_driver);
	if (ret < 0)
	{
	    pr_err("i2c_add_driver it6681_hdmi_rx_i2c_driver fail\n");	
		return ret;
	}

	ret = i2c_add_driver(&it6681_hdmi_tx_i2c_driver);
	if (ret < 0)
		goto err_exit0;

	ret = i2c_add_driver(&it6681_mhl_i2c_driver);
	if (ret < 0)
		goto err_exit1;

	if(ret <0)
		goto err_exit2;
#else
	mhl_it6681_init();
#endif
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
    //i2c_del_driver(&it6681_mhl_i2c_driver);
	//i2c_del_driver(&it6681_hdmi_tx_i2c_driver);
	//i2c_del_driver(&it6681_hdmi_rx_i2c_driver);	
	i2c_del_driver(&mhl_it6681_driver);
}

module_init(mhl_it6681_init);
module_exit(it6681_exit);
