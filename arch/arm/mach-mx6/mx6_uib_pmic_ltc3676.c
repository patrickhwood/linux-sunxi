/*
 * Copyright 2013 Linear Technology Corp. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * mx6x_uib_pmic_ltc3676.c  --  UIB MX6x board file for Linear LTC3676
 * PMIC
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/ltc3676.h>
#include <linux/mfd/ltc3676/core.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <mach/iomux-mx6q.h>
#include <mach/irqs.h>
#include <mach/gpio.h>

#define LTC_I2C_ADDR 	0x3C

static int get_voltage(struct ltc3676_regulator *sreg)
{
	return 0;
}

static int set_voltage(struct ltc3676_regulator *sreg, int uv)
{
	return 0;
}

static int enable(struct ltc3676_regulator *sreg)
{
	printk(KERN_INFO "%s: LTC3676 enable \n", __func__);
	return 1;
}

static int disable(struct ltc3676_regulator *sreg)
{
	return 0;
}

static int is_enabled(struct ltc3676_regulator *sreg)
{
	return 1;
}

// sw4 (buck4) is VDDARM and VDDSOC on UIB.  Since these two are on
// the same rail, we can't control the voltage from software, as these
// need different voltages at different times.  Instead, just set the
// minimum to 0.9V for standby mode and let the internal regulators
// deal with the voltage step downs for different stages and CPU
// clock speeds.
static struct ltc3676_regulator_data sw4_data = {
	.name		= "vddarmsoc_ext",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.min_voltage	= 900000,
	.max_voltage	= 1490000,
};

static struct regulator_consumer_supply sw4_consumers[] = {
	{
		.supply = "vddarmsoc_ext",
	},
};

static struct regulator_init_data sw4_init = {
	.constraints = {
		.name			= "vddarmsoc_ext",
		.min_uV			= 900000,
		.max_uV			= 1350000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.always_on		= 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(sw4_consumers),
	.consumer_supplies = &sw4_consumers[0],
};

static struct ltc3676_regulator sw4_reg = {
		.rdata = &sw4_data,
};

static int __init regulators_init(struct ltc3676_regulator *ltc3676)
{
	sw4_reg.i2c_client = ltc3676->i2c_client;
	sw4_reg.read_dev = ltc3676->read_dev;
	sw4_reg.write_dev = ltc3676->write_dev;

	//ONLY the ARM/SOC rail is init registered, since others are not changeable by Linux

	ltc3676_register_regulator(&sw4_reg, LTC3676_VSW4,  &sw4_init);

	return 0;
}

static struct ltc3676_platform_data __initdata ltc3676_plat = {
	.init = regulators_init,
};

static struct i2c_board_info __initdata ltc3676_i2c_device = {
	I2C_BOARD_INFO("ltc3676", LTC_I2C_ADDR),
	//.irq = gpio_to_irq(UIB_PMIC_INT),   //Please review IRQ declaration at the beginning of file
	.platform_data = &ltc3676_plat,
};

int __init uib_init_i2c(void)
{
	return i2c_register_board_info(1, &ltc3676_i2c_device, 1);
}
subsys_initcall(uib_init_i2c);

static __init int ltc3676_pmic_init(void)
{
	int i = 0;
	int ret = 0;
	struct regulator *reg;
	struct ltc3676_regulator *ltc;

	char *ltc3676_global_regulator[] = {
		"vddarmsoc_ext",
	};

	printk(KERN_INFO "%s: LTC3676 pmic init\n", __func__);

	while ((i < ARRAY_SIZE(ltc3676_global_regulator)))
	{
		reg = regulator_get(NULL, ltc3676_global_regulator[i]);
		if (IS_ERR(reg)) {
			printk(KERN_ERR "%s: LTC can't get regulator %s.\n", __func__, ltc3676_global_regulator[i]);
			goto err0;
		}
		else {
			printk(KERN_INFO "%s: Got %s regulator\n", __func__,ltc3676_global_regulator[i] );
			ret = regulator_enable(reg);
			if (ret) {
				printk(KERN_INFO "%s: Regulator Enable error %d.\n", __func__, ret);
				goto err0;
			}
			else
				printk(KERN_INFO "%s: Regulator %s Enable.\n",__func__, ltc3676_global_regulator[i]);

			ret = regulator_get_mode(reg);
			if (ret < 0) {
				printk(KERN_INFO "%s: Regulator Get Mode error %d.\n", __func__, ret);
				goto err0;
			}
			else
				printk(KERN_INFO "%s: Regulator %s Mode = %d.\n",__func__, ltc3676_global_regulator[i], ret);

			ret = regulator_get_voltage(reg);
			if (!ret) {
				printk(KERN_INFO "%s: Regulator Get Voltage error %d.\n", __func__, ret);
				goto err0;
			}
			else
				printk(KERN_INFO "%s: Regulator %s Voltage = %d.\n",__func__, ltc3676_global_regulator[i], ret);
		}
		i++;
	}

	// Set standby voltage on PMIC's SW4 to 0.9V per the
	// i.MX 6Solo/6DualLite datasheet.

	// This setting can't be enabled via driver calls; the only way
	// to switch to the suspend voltage is for the iMX6 to assert 
	// PMIC_STBY_REQ (pad F11) after performing all internal suspend
	// operations and putting the CPU(s) to sleep.  PMIC_STBY_REQ is
	// deasserted upon receipt of an unmasked interrupt from the GPC.
	reg = regulator_get(NULL, "vddarmsoc_ext");
	ltc = regulator_get_drvdata(reg);
	ret = ltc3676_set_suspend_voltage_ex(ltc, LTC3676_DCDC_4, 900000); // set to .9V for standby mode per Freescale's datasheet

err0:
	return ret;
}
late_initcall(ltc3676_pmic_init);

