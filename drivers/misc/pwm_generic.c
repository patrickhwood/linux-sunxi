/*
 *  Simple generic PWM driver
 *
 *	(c) Copyright 2015  Patrick Wood <pat.wood@efi.com>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/pwm.h>
#include <linux/pwm_generic.h>
#include <linux/err.h>
#include <linux/io.h>

#define PWM_MAX_PERIOD	10000000	// @@@ guess
#define PWM_MIN_PERIOD	1
#define PWM_MAX_DUTY	10000

struct pwm_generic {
	struct pwm_device *pwm;
	struct clk *clk;
    u32		period;
	u32		duty;
	u32		max_period;
	u32		min_period;
	u32		max_duty;
};

/*
 * /sys/devices/platform/generic-pwm.N
 *   /min_period    read-only   minimum pwm output frequency
 *   /max_period    read-only   maximum pwm output frequency
 *   /period        read-write  pwm output frequency (0 = disable output)
 *   /duty          read-write  pwm duty cycle (0 -- max_duty)
 *   /max_duty      read-write  max pwm duty cycle
 */

static ssize_t pwm_generic_get_min_period(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwm_generic *pwm = platform_get_drvdata(pdev);

	return sprintf(buf, "%u\n", pwm->min_period);
}

static ssize_t pwm_generic_get_max_period(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwm_generic *pwm = platform_get_drvdata(pdev);

	return sprintf(buf, "%u\n", pwm->max_period);
}

static ssize_t pwm_generic_get_period(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwm_generic *pwm = platform_get_drvdata(pdev);

	if (pwm->period) {
		return sprintf(buf, "%u\n", 1000000000 / pwm->period);
	} else {
		return sprintf(buf, "disabled\n");
	}
}

static ssize_t pwm_generic_set_period(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwm_generic *pwm = platform_get_drvdata(pdev);
	long period;
	int err;

	err = strict_strtol(buf, 10, &period);
	if (err || period < 0)
		return -EINVAL;
	else if (period == 0) {
		pwm->period = 0;
		pwm_disable(pwm->pwm);
	}
	else {
		period = 1000000000 / period;
		if (period > pwm->max_period)
			period = pwm->max_period;
		if (period < pwm->min_period)
			period = pwm->min_period;

		pwm->period = period;

		/* scale duty based on period */
		pwm_config(pwm->pwm, (pwm->duty * pwm->period) / pwm->max_duty, pwm->period);
		if (pwm->duty) {
			pwm_enable(pwm->pwm);
		}
	}

	return count;
}

static ssize_t pwm_generic_get_duty(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwm_generic *pwm = platform_get_drvdata(pdev);

	if (pwm->period)
		return sprintf(buf, "%u\n", pwm->duty);
	else
		return sprintf(buf, "disabled\n");
}

static ssize_t pwm_generic_set_duty(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwm_generic *pwm = platform_get_drvdata(pdev);
	long duty;
	int err;

	err = strict_strtol(buf, 10, &duty);
	if (err || duty < 0)
		return -EINVAL;
	else if (duty == 0) {
		pwm->duty = 0;
		pwm_disable(pwm->pwm);
	}
	else {
		if (duty > pwm->max_duty)
			duty = pwm->max_duty;

		pwm->duty = duty;

		/* scale duty based on period */
		pwm_config(pwm->pwm, (pwm->duty * pwm->period) / pwm->max_duty, pwm->period);
		if (pwm->period) {
			pwm_enable(pwm->pwm);
		}
	}

	return count;
}

static ssize_t pwm_generic_get_max_duty(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwm_generic *pwm = platform_get_drvdata(pdev);

	return sprintf(buf, "%u\n", pwm->max_duty);
}

static DEVICE_ATTR(min_period, S_IRUGO, pwm_generic_get_min_period, NULL);
static DEVICE_ATTR(max_period, S_IRUGO, pwm_generic_get_max_period, NULL);
static DEVICE_ATTR(period, S_IWUSR | S_IRUGO,
		   pwm_generic_get_period, pwm_generic_set_period);
static DEVICE_ATTR(duty, S_IWUSR | S_IRUGO,
		   pwm_generic_get_duty, pwm_generic_set_duty);
static DEVICE_ATTR(max_duty, S_IRUGO, pwm_generic_get_max_duty, NULL);

static struct attribute *pwm_generic_attrs[] = {
	&dev_attr_min_period.attr,
	&dev_attr_max_period.attr,
	&dev_attr_period.attr,
	&dev_attr_duty.attr,
	&dev_attr_max_duty.attr,
	NULL
};

static const struct attribute_group pwm_generic_sysfs_files = {
	.attrs	= pwm_generic_attrs,
};

static const struct attribute_group *pwm_generic_sysfs_attr_groups[] = {
    &pwm_generic_sysfs_files,
    NULL
};


static int __init pwm_generic_probe(struct platform_device *pdev)
{
	struct platform_pwm_generic_data *data = pdev->dev.platform_data;
	struct pwm_generic *pwm;
	int err;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	pwm = kzalloc(sizeof(struct pwm_generic), GFP_KERNEL);
	if (!pwm) {
		err = -ENOMEM;
		return err;
	}

/*
	pwm->clk = clk_get(&pdev->dev, "pwm");
	if (IS_ERR(pwm->clk)) {
		dev_err(&pdev->dev, "unable to request pwm clock for generic, id = %d\n", data->pwm_id);
		err = PTR_ERR(pwm->clk);
		goto fail_no_clk;
	}

	pwm->max_period = clk_get_rate(pwm->clk) / 2;
*/
	pwm->max_period = PWM_MAX_PERIOD;
	pwm->min_period = PWM_MIN_PERIOD;
	pwm->period = 0;
	pwm->duty = 0;
	pwm->max_duty = PWM_MAX_DUTY;
	pwm->pwm = pwm_request(data->pwm_id, "generic");;

	if (IS_ERR(pwm->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for generic, id = %d\n", data->pwm_id);
		err = PTR_ERR(pwm->pwm);
		goto fail_err_pwm;
	} else
		dev_dbg(&pdev->dev, "got pwm for backlight\n");

	platform_set_drvdata(pdev, pwm);

	/* disable pwm at startup. Avoids zero value. */
	pwm_disable(pwm->pwm);

	return 0;

fail_err_pwm:
	// clk_put(pwm->clk);
// fail_no_clk:
	sysfs_remove_group(&pdev->dev.kobj, &pwm_generic_sysfs_files);
	kfree(pwm);
	return err;
}

static int pwm_generic_remove(struct platform_device *pdev)
{
	struct pwm_generic *pwm = platform_get_drvdata(pdev);

	pwm_config(pwm->pwm, 0, pwm->period);
	pwm_disable(pwm->pwm);
	pwm_free(pwm->pwm);
	// clk_put(pwm->clk);
	platform_set_drvdata(pdev, NULL);
	sysfs_remove_group(&pdev->dev.kobj, &pwm_generic_sysfs_files);
	kfree(pwm);

	return 0;
}

#ifdef CONFIG_PM
static int pwm_generic_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct pwm_generic *pwm = platform_get_drvdata(pdev);

	pwm_config(pwm->pwm, 0, pwm->period);
	pwm_disable(pwm->pwm);
	return 0;
}

static int pwm_generic_resume(struct platform_device *pdev)
{
	struct pwm_generic *pwm = platform_get_drvdata(pdev);

	pwm_config(pwm->pwm, (pwm->duty * pwm->period) / pwm->max_duty, pwm->period);
	pwm_enable(pwm->pwm);
	return 0;
}
#else
#define pwm_backlight_suspend   NULL
#define pwm_backlight_resume    NULL
#endif

static void pwm_generic_shutdown(struct platform_device *pdev)
{
	struct pwm_generic *pwm = platform_get_drvdata(pdev);

	pwm_config(pwm->pwm, 0, pwm->period);
	pwm_disable(pwm->pwm);
}

static struct platform_driver pwm_generic_driver = {
	.driver		= {
		.name	= "pwm-generic",
		.owner	= THIS_MODULE,
		.groups	= pwm_generic_sysfs_attr_groups,
	},
	.probe		= pwm_generic_probe,
	.remove		= pwm_generic_remove,
	.suspend    = pwm_generic_suspend,
	.resume     = pwm_generic_resume,
	.shutdown   = pwm_generic_shutdown,
};

static int __init pwm_generic_init(void)
{
	return platform_driver_probe(&pwm_generic_driver, pwm_generic_probe);
}

static void __exit pwm_generic_exit(void)
{
	platform_driver_unregister(&pwm_generic_driver);
}

module_init(pwm_generic_init);
module_exit(pwm_generic_exit);

MODULE_AUTHOR("Pat Wood <pat...@efi.com>");
MODULE_DESCRIPTION("Generic PWM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-generic");
