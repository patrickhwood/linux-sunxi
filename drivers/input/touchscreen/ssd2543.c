/*
 * drivers/input/touchscreen/tsc2007.c
 *
 * Copyright (c) 2008 MtekVision Co., Ltd.
 *	Kwangwoo Lee <kwlee@mtekvision.com>
 *
 * Using code from:
 *  - ads7846.c
 *	Copyright (c) 2005 David Brownell
 *	Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <asm/gpio.h>
#include <mach/hardware.h>

#define TS_POLL_DELAY	(50 * 1000 * 1000)	/* ns delay before the first sample */
#define TS_POLL_PERIOD	(50 * 1000 * 1000)	/* ns delay between samples */
#define MAX_X        1023
#define MAX_Y        599
#define MAX_PRESSURE 200
#define MT_SUPPORT

#define SSD_I2C_RETRY_COUNT   3
#define SSD_I2C_DRIVER_NAME   "ssd2543"
#define SSD_DEBUG_LEVEL   KERN_INFO
#define SSD_ERROR_LEVEL   KERN_ALERT
#define FINGERNO 5

#define DEVICE_ID_REG      0x02
#define VERSION_ID_REG     0x03
#define EVENT_STATUS       0x79
#define FINGER00_REG       0x7C
#define DEVICE_CHANEL_REG  0x06

#define GPIO_IRQ	IMX_GPIO_NR(3, 23)   //irq
#define GPIO_RESET	IMX_GPIO_NR(3, 24)    //reset

struct ChipSetting
{
	char No;
	char Reg;
	char Data1;
	char Data2;
};

const struct ChipSetting ssdcfgTable[] = {
	{2,0x06,0x19,0x0E},
	{2,0x28,0x00,0x12},
	{2,0x07,0x00,0xE1},
	{2,0x08,0x00,0xE2},
	{2,0x09,0x00,0xE3},
	{2,0x0A,0x00,0xE4},
	{2,0x0B,0x00,0xE5},
	{2,0x0C,0x00,0xE6},
	{2,0x0D,0x00,0xE7},
	{2,0x0E,0x00,0xE8},
	{2,0x0F,0x00,0xE9},
	{2,0x10,0x00,0xEA},
	{2,0x11,0x00,0xEB},
	{2,0x12,0x00,0xEC},
	{2,0x13,0x00,0xED},
	{2,0x14,0x00,0xEE},
	{2,0x15,0x00,0xEF},
	{2,0x16,0x00,0xF0},
	{2,0x17,0x00,0xF1},
	{2,0x18,0x00,0xF2},
	{2,0x19,0x00,0xF3},
	{2,0x1A,0x00,0xF4},
	{2,0x1B,0x00,0xF5},
	{2,0x1C,0x00,0xF6},
	{2,0x1D,0x00,0xF7},
	{2,0x1E,0x00,0xF8},
	{2,0x1F,0x00,0xF9},
	{2,0x20,0x00,0xFA},
	{2,0xD7,0x00,0x02},
	{2,0xD8,0x00,0x07},
	{2,0xDB,0x00,0x02},
	{2,0x30,0x08,0x0D},
	{2,0x34,0xC6,0x40},
	{2,0x36,0x00,0x1A},
	{2,0x3A,0x00,0x00},
	{2,0x65,0x00,0x04},
	{2,0x66,0x27,0xF0},
	{2,0x67,0x27,0x60},
	{2,0x7A,0xFF,0xFF},
	{2,0x7B,0x00,0x03},
	{2,0x25,0x00,0x0C},
	{2,0x04,0x00,0x01},
	{2,0xFF,0x00,0xC8},
};

const struct ChipSetting Resume[]={
	{ 2,0x04,0x00,0x01},
};

const struct ChipSetting Suspend[] ={
	{ 2,0x05,0x00,0x01},
};

struct ssl_ts_priv {
	struct input_dev	*input;
	struct hrtimer		timer;
	struct work_struct  ssl_work;
	struct i2c_client	*client;
	spinlock_t			lock;
	int					irq;
};

static struct workqueue_struct *ssd253x_wq;
static int preEventStatus;

static int ssd_i2c_transfer(struct i2c_client *client, struct i2c_msg *msgs, int cnt)
{
	int ret, count=3;
	while(count >= 0)
	{
		count-= 1;
		ret = i2c_transfer(client->adapter, msgs, cnt);
		if(ret < 0)
		{
			msleep(50);
			continue;
		}
		break;
	}
	return ret;
}

static int ssd_i2c_read(struct i2c_client *client, uint8_t cmd, uint8_t *data, int length)
{
	int ret;
	struct i2c_msg msgs[] =
	{
		{	.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &cmd,
		},
		{	.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	ret = ssd_i2c_transfer(client, msgs, 2);
	if(ret < 0)
	{
		printk(SSD_ERROR_LEVEL "%s, i2c read error, ret %d\n", __func__, ret);
	}
	return ret;
}

static int ssd_i2c_write(struct i2c_client *client, uint8_t cmd, uint8_t *data, int length)
{
	int ret;
	int i;
	unsigned char buf[9]={0};
	struct i2c_msg msgs[] =
	{
		{	.addr = client->addr,
			.flags = 0,
			.len = length+1,
			.buf = buf,
		},
	};

	if (cmd == 0xFF)
	{
		mdelay(data[0]*256 + data[1]);
		return 0;
	}

	buf[0] = cmd;
	for(i = 0; i < length; i++)
	{
		buf[1+i] = data[i];
	}

	ret = ssd_i2c_transfer(client, msgs, 1);
	if(ret < 0)
	{
		printk(SSD_ERROR_LEVEL "%s, i2c read error, ret %d\n", __func__, ret);
	}
	return ret;
}

static int ssd_i2c_read_tp_info(struct ssl_ts_priv *ts)
{
	unsigned char buf[32]={0};
	int i;

	// read firmware version
	if(ssd_i2c_read(ts->client, DEVICE_ID_REG, buf, 2) < 0)
	{
		return -1;
	}

	printk(SSD_DEBUG_LEVEL "%s, chip ID %X%X\n", __func__, buf[0], buf[1]);

	// read firmware version
	if(ssd_i2c_read(ts->client, VERSION_ID_REG, buf, 2) < 0)
	{
		return -1;
	}

	printk(SSD_DEBUG_LEVEL "%s, version ID %X:%X\n", __func__, buf[0], buf[1]);

#ifdef CONFIG_SSD_TOUCH_DEBUG
/* dump all register values from ssdcfgTable to verify settings */
	for (i = 0; i < sizeof(ssdcfgTable)/sizeof(ssdcfgTable[0]); i++)
	{
		if (ssdcfgTable[i].Reg <= 4)
			/* write-only registers */
			continue;
		if (ssdcfgTable[i].Reg == 0xFF) {
			/* internal sleep command */
			mdelay(ssdcfgTable[i].Data1*256 + ssdcfgTable[i].Data2);
			continue;
		}

		if (ssd_i2c_read(ts->client,ssdcfgTable[i].Reg, buf, 2) < 0)
			printk(SSD_DEBUG_LEVEL "%s, reg %X: read error\n", __func__, ssdcfgTable[i].Reg);
		else
			printk(SSD_DEBUG_LEVEL "%s, reg %X %X:%X\n", __func__, ssdcfgTable[i].Reg, buf[0], buf[1]);
	}

	if(ssd_i2c_read(ts->client, DEVICE_CHANEL_REG, buf, 2) < 0)
	{
		return -1;
	}

	printk(SSD_DEBUG_LEVEL "%s, Drive:%d Sense:%d\n", __func__, buf[0], buf[1]);
#endif

	return 0;
}

static int ssd_tp_init(struct ssl_ts_priv *ts)
{
	unsigned char buf[4]={0};
	int i;

	printk(SSD_DEBUG_LEVEL "%s    \n", __func__);

	//init chip config
	for (i = 0; i < sizeof(ssdcfgTable)/sizeof(ssdcfgTable[0]); i++)
	{
		buf[0] = ssdcfgTable[i].Data1;
		buf[1] = ssdcfgTable[i].Data2;
		ssd_i2c_write(ts->client,ssdcfgTable[i].Reg, buf, ssdcfgTable[i].No);
	}

	msleep(50);

	return 0;
}

static irqreturn_t ssd_ts_irq(int irq, void *handle)
{
	struct ssl_ts_priv *ts = handle;
	unsigned long flags;

	spin_lock_irqsave(&ts->lock, flags);
	printk(SSD_ERROR_LEVEL "%s\n", __func__);

//	if (likely(ts->get_pendown_state()))
	{
		// disable_irq(ts->irq);
		// hrtimer_start(&ts->timer, ktime_set(0, TS_POLL_DELAY),
		// 			HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&ts->lock, flags);

	return IRQ_HANDLED;
}

static void ssd_ts_work(struct work_struct *work)
{
	struct ssl_ts_priv *ts = container_of(work,struct ssl_ts_priv,ssl_work);

	unsigned char buf[9]={0};
	int send_report = 0;
	int i;
	unsigned short xpos = 0, ypos = 0, width = 0;
	int EventStatus,EventChange;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];
	int ret;

	// read i2c data from device
	ret = ssd_i2c_read(ts->client, EVENT_STATUS, buf, 2);
	if(ret < 0)
	{
		return;
	}

	EventStatus = ((int)buf[0] << 8 | buf[1]) >> 4;

	// printk(SSD_DEBUG_LEVEL "%s, STATUS%X buf[0]%X buf[1]%X\n", __func__, EventStatus, buf[0], buf[1]);

	for (i = 0; i < FINGERNO; i++)
	{
		if ((EventStatus >> i) & 0x1)
		{
			ret = ssd_i2c_read(ts->client, FINGER00_REG+i, buf, 4);

			if(ret < 0)
			{
				return;
			}
			xpos = ((buf[2] & 0xf0) << 4) | buf[0];
			ypos = ((buf[2] & 0x0f) << 8) | buf[1];
			width = buf[3];
		}
		else
		{
			xpos = ypos = 0xFFF;
			width = 0;
		}
		FingerX[i] = xpos;
		FingerY[i] = ypos;
		FingerP[i] = width;
	}

	for (i = 0; i < FINGERNO; i++)
	{
		xpos = FingerX[i];
		ypos = FingerY[i];
		width = FingerP[i];

		EventChange = ((preEventStatus ^ EventStatus)>> i)&0x0001;

		if(EventChange)							//touch event changed
		{
			preEventStatus ^= (0x0001 << i) ;	//update pre event status
		}
		#ifdef MT_SUPPORT
		if (xpos != 0xFFF)					// touch down , report
		{
			send_report = 1;
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, i);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, width);
			input_report_abs(ts->input, ABS_MT_POSITION_X, xpos);
			input_report_abs(ts->input, ABS_MT_POSITION_Y, ypos);
			input_report_abs(ts->input, ABS_MT_PRESSURE, width);
			input_report_key(ts->input, BTN_TOUCH, 1);
			input_mt_sync(ts->input);
			printk(SSD_DEBUG_LEVEL "%s, ID:%d X:%d Y:%d Z:%d\n", __func__, i, xpos, ypos,width);
		}
		else if (EventChange)				// touch up
		{
			send_report = 1;
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, i);
			input_report_key(ts->input, BTN_TOUCH, 0);
			input_mt_sync(ts->input);
			printk(SSD_DEBUG_LEVEL "%s, ID:%d X:%d Y:%d Z:%d\n", __func__, i, xpos, ypos,width);
		}

		#else	// MT_SUPPORT
		if(i==0)				//only report finger 0
		{
			if (xpos != 0xFFF)					// touch down , report
			{
				input_report_abs(ts->input, ABS_X, xpos);
				input_report_abs(ts->input, ABS_Y, ypos);
				input_report_abs(ts->input, ABS_PRESSURE, width);
				input_report_key(ts->input, BTN_TOUCH, 1);
				send_report = 1;
				printk(SSD_DEBUG_LEVEL "%s, ID:%d X:%d Y:%d Z:%d\n", __func__, i, xpos, ypos,width);

			}
			else if (EventChange)				// touch up/down change
			{
				xpos = 0;
				ypos = 0;
				input_report_key(ts->input, BTN_TOUCH, 0);
				send_report = 1;
				printk(SSD_DEBUG_LEVEL "%s, ID:%d X:%d Y:%d Z:%d\n", __func__, i, xpos, ypos,width);
			}
		}
		#endif	// MT_SUPPORT
	}

	if(send_report==1)
		input_sync(ts->input);

	hrtimer_start(&ts->timer, ktime_set(0, TS_POLL_PERIOD), HRTIMER_MODE_REL);

	return;
}

static enum hrtimer_restart ssd_ts_timer(struct hrtimer *timer)
{
	struct ssl_ts_priv *ts = container_of(timer, struct ssl_ts_priv, timer);
	// printk(SSD_DEBUG_LEVEL "%s\n",__func__);

	queue_work(ssd253x_wq, &ts->ssl_work);
	return HRTIMER_NORESTART;
}

static int ssd2543_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ssl_ts_priv *ts;
	struct input_dev *input_dev;
	int err = 0;

	printk(SSD_DEBUG_LEVEL "%s\n",__func__);
	if (!i2c_check_functionality(client->adapter,
					 I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		printk(SSD_ERROR_LEVEL "%s: i2c_check_functionality failed\n", __func__);
		return -EIO;
	}

	ts = kzalloc(sizeof(struct ssl_ts_priv), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		printk(SSD_ERROR_LEVEL "%s: kzalloc failed\n", __func__);
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->input = input_dev;

	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = ssd_ts_timer ;			//ssd2543_timer;

	spin_lock_init(&ts->lock);

	input_dev->name = "SSD2543 Touch Screen";
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

#ifdef MT_SUPPORT
	input_set_abs_params(input_dev,
				ABS_MT_POSITION_X,  0, MAX_X, 0, 0);
	input_set_abs_params(input_dev,
				ABS_MT_POSITION_Y,  0, MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, MAX_PRESSURE, 0, 0);
	input_set_abs_params(input_dev,
				ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	input_set_abs_params(input_dev,
				ABS_MT_TRACKING_ID, 0,
				FINGERNO-1, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_X, 0, MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_PRESSURE, 0, 0);
#endif

	gpio_request(GPIO_RESET, "SSD-RESET");
	gpio_direction_output(GPIO_RESET, 1);
	mdelay(5);
	gpio_set_value(GPIO_RESET, 0);
	mdelay(5);
	gpio_set_value(GPIO_RESET, 1);
	mdelay(20);

	ssd_i2c_read_tp_info(ts);
	ssd_tp_init(ts);
	ssd_i2c_read_tp_info(ts);

	hrtimer_start(&ts->timer, ktime_set(0, TS_POLL_PERIOD), HRTIMER_MODE_REL);

	INIT_WORK(&ts->ssl_work, ssd_ts_work);	// Intialize the work queue

	ts->irq = client->irq;
	if (ts->irq < 0)
	{
		printk(SSD_ERROR_LEVEL "%s: request irq pin failed\n", __func__);
		err = -ENODEV;
		goto err_free_mem;
	}

	err = gpio_request(GPIO_IRQ, "SSD-IRQ");
	if (err < 0)
	{
		printk(SSD_ERROR_LEVEL "%s: request IO failed\n", __func__);
		goto err_free_mem;
	}

	err = request_irq(ts->irq, ssd_ts_irq, IRQF_TRIGGER_FALLING, client->name, ts);
	if (err < 0){
		printk(SSD_ERROR_LEVEL "%s: request IRQ failed\n", __func__);
		goto err_free_mem;
	}

	// disable_irq_nosync(ts->irq);

	err = input_register_device(input_dev);
	if (err) {
		printk(SSD_ERROR_LEVEL "%s: input_register_device failed\n", __func__);
		goto err_free_irq;
	}

	dev_info(&client->dev, "registered with irq (%d)\n", ts->irq);

	return 0;

 err_free_irq:
	free_irq(ts->irq, ts);
	hrtimer_cancel(&ts->timer);
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	printk(SSD_ERROR_LEVEL "%s: failed, err = %d\n", __func__, err);
	return err;
}

static int ssd2543_remove(struct i2c_client *client)
{
	struct ssl_ts_priv	*ts = i2c_get_clientdata(client);

	free_irq(gpio_to_irq(GPIO_IRQ), NULL);
	free_irq(ts->irq, ts);
	hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}

static struct i2c_device_id ssd2543_idtable[] = {
	{ "ssd2543", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ssd2543_idtable);

static struct i2c_driver ssd2543_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ssd2543"
	},
	.id_table	= ssd2543_idtable,
	.probe		= ssd2543_probe,
	.remove		= ssd2543_remove,
};

static int __init ssd2543_init(void)
{
	ssd253x_wq = create_singlethread_workqueue("ssd253x_wq");
	if (!ssd253x_wq){
		return -ENOMEM;
	}
	else{
	}
	return i2c_add_driver(&ssd2543_driver);
}

static void __exit ssd2543_exit(void)
{
	i2c_del_driver(&ssd2543_driver);
	if (ssd253x_wq) destroy_workqueue(ssd253x_wq);
}

module_init(ssd2543_init);
module_exit(ssd2543_exit);

MODULE_AUTHOR("Kwangwoo Lee <kwlee@mtekvision.com>");
MODULE_DESCRIPTION("TouchScreen Driver");
MODULE_LICENSE("GPL");
