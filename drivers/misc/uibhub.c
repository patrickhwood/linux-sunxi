/*
 * drivers/misc/uibhub.c
 *
 * Copyright (c) Patrick Wood <pat.wood@efi.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
# include <linux/earlysuspend.h>
#endif
#include <mach/hardware.h>

#define VENDOR_ID_LSB      0x01
#define VENDOR_ID_MSB      0x02
#define PRODUCT_ID_LSB     0x03
#define PRODUCT_ID_MSB     0x04
#define STATUS_REG         0xF8

struct uib_hub_priv {
	struct i2c_client	*client;
	spinlock_t			lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif 
};

static int hub_i2c_transfer(struct i2c_client *client, struct i2c_msg *msgs, int cnt)
{
	int ret, count;
	for (count = 0; count < 3; count++) {
		ret = i2c_transfer(client->adapter, msgs, cnt);
		if (ret < 0) {
			msleep(50);
			continue;
		}
		break;
	}
	return ret;
}

static int hub_i2c_read(struct i2c_client *client, uint8_t reg, uint8_t *data)
{
	int ret;
	struct i2c_msg msgs[] =
	{
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = data,
		}
	};

	ret = hub_i2c_transfer(client, msgs, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s, i2c read error, ret %d\n", __func__, ret);
	}
	return ret;
}

static int hub_i2c_write(struct i2c_client *client, uint8_t reg, uint8_t data)
{
	int ret;
	unsigned char buf[2]={0};
	struct i2c_msg msgs[] =
	{
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
	};

	buf[0] = reg;
	buf[1] = data;

	ret = hub_i2c_transfer(client, msgs, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s, i2c write error, ret %d\n", __func__, ret);
	}
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void uibhub_late_resume(struct early_suspend *early_s)
{
	struct uib_hub_priv *hub = container_of(early_s, struct uib_hub_priv, early_suspend);
	unsigned char status = 0xff;
	extern void request_host_on(void);

	// clear cfgActive bit in status register to resume operation
	// after reset
	if (hub_i2c_write(hub->client, STATUS_REG, 0x01) < 0) {
	        dev_err(&hub->client->dev, "%s: error clearing status reg\n", __func__);
	}

	hub_i2c_read(hub->client, STATUS_REG, &status);
	dev_info(&hub->client->dev, "%s: status = %x\n", __func__, status);

	msleep(5);
	request_host_on();
}
static void uibhub_early_suspend(struct early_suspend *early_s)
{
	struct uib_hub_priv *hub = container_of(early_s, struct uib_hub_priv, early_suspend);
	unsigned char status = 0xff;

/* moved reset code to S3 interrupt handler so it can be deferred until the fiery goes to sleep
   instead of when android goes to sleep

	unsigned int *HUB_gpios = (unsigned int *) hub->client->dev.platform_data;

	// reset hub to put into lower power mode
	gpio_set_value(HUB_gpios[0], 0);
	mdelay(5);
	gpio_set_value(HUB_gpios[0], 1);
	mdelay(5);
*/

	hub_i2c_read(hub->client, STATUS_REG, &status);
	dev_info(&hub->client->dev, "%s: status = %x\n", __func__, status);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int uibhub_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct uib_hub_priv *hub;
	unsigned char buf[4];
	unsigned char status;
	int err = 0;
	/* reset GPIO NR passed in dev.platform_data */
	unsigned int *HUB_gpios = (unsigned int *) client->dev.platform_data;

	dev_err(&client->dev, "%s:\n",__func__);
 
	if (!i2c_check_functionality(client->adapter,
					 I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		dev_err(&client->dev, "%s: i2c_check_functionality failed\n", __func__);
		return -EIO;
	}

	hub = kzalloc(sizeof(struct uib_hub_priv), GFP_KERNEL);

	hub->client = client;
	i2c_set_clientdata(client, hub);
	spin_lock_init(&hub->lock);

	// check the vendor and product IDs

	// read VID
	if (hub_i2c_read(hub->client, VENDOR_ID_LSB, &buf[0]) < 0) {
		err = -EIO;
		goto err_free_mem;
	}
	if (buf[0] != 0x51) {
		dev_err(&client->dev, "%s: vendor ID lsb %x mismatch\n", __func__, buf[0]);
		err = -ENODEV;
		goto err_free_mem;
	}

	if (hub_i2c_read(hub->client, VENDOR_ID_MSB, &buf[1]) < 0) {
		err = -EIO;
		goto err_free_mem;
	}
	if (buf[1] != 0x04) {
		dev_err(&client->dev, "%s: vendor ID msb %x mismatch\n", __func__, buf[1]);
		err = -ENODEV;
		goto err_free_mem;
	}

	// read PID
	if (hub_i2c_read(hub->client, PRODUCT_ID_LSB, &buf[2]) < 0) {
		err = -EIO;
		goto err_free_mem;
	}
	if (buf[2] != 0x46 && buf[2] != 0x40) {
		dev_err(&client->dev, "%s: product ID lsb %x mismatch\n", __func__, buf[2]);
		err = -ENODEV;
		goto err_free_mem;
	}

	if (hub_i2c_read(hub->client, PRODUCT_ID_MSB, &buf[3]) < 0) {
		err = -EIO;
		goto err_free_mem;
	}
	if (buf[3] != 0x80 && buf[3] != 0x81) {
		dev_err(&client->dev, "%s: product ID msb %x, mismatch\n", __func__, buf[3]);
		err = -ENODEV;
		goto err_free_mem;
	}

	if (hub_i2c_write(hub->client, STATUS_REG, 0x01) < 0) {
		dev_err(&client->dev, "%s: error clearing status reg\n", __func__);
		err = -EIO;
		goto err_free_mem;
	}

	if (hub_i2c_read(hub->client, STATUS_REG, &status) < 0) {
		err = -EIO;
		goto err_free_mem;
	}

	dev_err(&client->dev,
		"%s: registered hub: VID = %02x%02x, PID = %02x%02x, status = %x\n",
		__func__, buf[1], buf[0], buf[3], buf[2], status);

#ifdef CONFIG_HAS_EARLYSUSPEND
	gpio_direction_output(HUB_gpios[0], 1);
	hub->early_suspend.suspend = uibhub_early_suspend;
	hub->early_suspend.resume  = uibhub_late_resume;
	hub->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN-2;
	register_early_suspend(&hub->early_suspend);
#endif

	return 0;

err_free_mem:
	kfree(hub);
	dev_err(&client->dev, "%s: failed, err = %d\n", __func__, err);
	return err;
}

static int uibhub_remove(struct i2c_client *client)
{
	struct uib_hub_priv	*hub = i2c_get_clientdata(client);

	kfree(hub);

	return 0;
}

static struct i2c_device_id uibhub_idtable[] = {
	{ "uibhub", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, uibhub_idtable);

static struct i2c_driver uibhub_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "uibhub"
	},
	.id_table	= uibhub_idtable,
	.probe		= uibhub_probe,
	.remove		= uibhub_remove,
};

static int __init uibhub_init(void)
{
	return i2c_add_driver(&uibhub_driver);
}

static void __exit uibhub_exit(void)
{
	i2c_del_driver(&uibhub_driver);
}

module_init(uibhub_init);
module_exit(uibhub_exit);

MODULE_AUTHOR("Patrick Wood <pat.wood@efi.com>");
MODULE_DESCRIPTION("UIB HUB Driver");
MODULE_LICENSE("GPL");
