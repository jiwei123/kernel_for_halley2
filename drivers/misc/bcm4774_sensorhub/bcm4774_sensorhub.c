/*
 *  gps_bcm.c - Linux kernel modules for controlling bcm4774 enable pin
 *  			and enable the clk32
 *  Version     : 01.00
 *  Time        : Jan.06, 2016
 *  Author      : WenBin <bin.wen@ingenic.com>
 *
 *  Copyright (C) 2016 Ingenic Semiconductor.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/bcm4774_pm.h>
#include <mach/jzmmc.h>


struct bcm4774_device {

	unsigned int nSTANDBY_enable;
	unsigned int nSTANDBY_state;

	struct kobject *kobj;
	struct mutex gps_lock;
	struct regulator *power_rglt;
	struct bcm4774_platform_data *pdata;
};

static struct bcm4774_device *bcm4774_sensorhub_gps = NULL;


static ssize_t bcm4774_clk_switch_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	// add somthing to show clk32 state later

	return 0;
}

static ssize_t bcm4774_clk_switch_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	// add somthing to control clk32 later

	return count;
}

static ssize_t bcm4774_nSTANDBY_enable_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int state = 0;

    mutex_lock(&bcm4774_sensorhub_gps->gps_lock);
    state = bcm4774_sensorhub_gps->nSTANDBY_state;
    mutex_unlock(&bcm4774_sensorhub_gps->gps_lock);

    return sprintf(buf, "%d\n", state);
}

static ssize_t bcm4774_nSTANDBY_enable_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long enable;
	int ret = -1;

    mutex_lock(&bcm4774_sensorhub_gps->gps_lock);

    enable = simple_strtoul(buf, NULL, 10);
    enable = (enable > 0) ? 1 : 0;
    if(0 == enable)
    {
        ret = bcm4774_sensorhub_gps->pdata->enable(bcm4774_sensorhub_gps->pdata,BCM4774_POWER_OFF);
        if( ret == 0)
            bcm4774_sensorhub_gps->nSTANDBY_state = 0;

    } else {
        ret = bcm4774_sensorhub_gps->pdata->enable(bcm4774_sensorhub_gps->pdata, BCM4774_POWER_ON);
        if( ret == 0)
            bcm4774_sensorhub_gps->nSTANDBY_state = 1;
    }
    mutex_unlock(&bcm4774_sensorhub_gps->gps_lock);

	return count;
}

static DEVICE_ATTR(clk_switch, S_IRUGO | S_IWUGO,
		bcm4774_clk_switch_show, bcm4774_clk_switch_store);
static DEVICE_ATTR(nSTANDBY_enable, S_IRUGO | S_IWUGO,
		bcm4774_nSTANDBY_enable_show, bcm4774_nSTANDBY_enable_store);

static struct device_attribute *bcm4774_attributes[] = {
        &dev_attr_clk_switch.attr,
        &dev_attr_nSTANDBY_enable.attr,
        NULL,
};

static const struct attribute_group bcm4774_attr_group = {
		.attrs = bcm4774_attributes,
};

static int bcm4774_pm_probe(struct platform_device *pdev)
{
	int err = -1;

	bcm4774_sensorhub_gps = kzalloc(sizeof(struct bcm4774_device), GFP_KERNEL);
	if(!bcm4774_sensorhub_gps)
	{
		printk("bcm4774 gps kzalloc failed!\n");
		goto err_kzlloc;
	}
	bcm4774_sensorhub_gps->pdata = pdev->dev.platform_data;
	if (IS_ERR(bcm4774_sensorhub_gps->pdata)) {
		printk("bcm4774 dev_get_platdata error!");
		goto err_dev_get_platdata;
	}

	bcm4774_sensorhub_gps->nSTANDBY_enable = bcm4774_sensorhub_gps->pdata->nSTANDBY_enable;

	err = gpio_request(bcm4774_sensorhub_gps->pdata->nSTANDBY_enable, "nSTANDBY");
	if(err < 0){
		printk("bcm4774_gps gpio_request failed!\n");
		goto err_gpio_request;
	}

	bcm4774_sensorhub_gps->kobj = kobject_create_and_add("bcm4774_sensorhub", NULL);
	if(!bcm4774_sensorhub_gps->kobj)
	{
		printk("kobject_create_and_add error!\n");
		goto err_kobject_create_and_add;
	}

	err = sysfs_create_group(bcm4774_sensorhub_gps->kobj, &bcm4774_attr_group);
	if(err < 0)
	{
		printk("bcm4774_gps ysfs_create_group failed !\n");
		goto err_sysfs_create_group;
	}

	bcm4774_sensorhub_gps->power_rglt = regulator_get(NULL, bcm4774_sensorhub_gps->pdata->regulator_name);
	if(!bcm4774_sensorhub_gps->power_rglt)
	{
		printk("bcm4774 gps regulator_get failed !\n");
		goto err_regulator_get;
	}

	mutex_init(&bcm4774_sensorhub_gps->gps_lock);

	bcm4774_sensorhub_gps->pdata->clk_enable();

	return 0;

err_regulator_get:
err_sysfs_create_group:
	kobject_put(bcm4774_sensorhub_gps->kobj);
err_kobject_create_and_add:
	gpio_free(bcm4774_sensorhub_gps->nSTANDBY_enable);
err_gpio_request:
err_dev_get_platdata:
	kfree(bcm4774_sensorhub_gps);
err_kzlloc:
	err = PTR_ERR(bcm4774_sensorhub_gps->pdata);
	return err;
}

static int bcm4774_pm_remove(struct platform_device *pdev)
{
	kobject_put(bcm4774_sensorhub_gps->kobj);
	gpio_free(bcm4774_sensorhub_gps->nSTANDBY_enable);
}

static struct platform_driver bcm4774_pm_driver = {
	.driver		= {
		.name	= "bcm4774_pm",
		.owner	= THIS_MODULE,
	},
	.probe		= bcm4774_pm_probe,
	.remove		= bcm4774_pm_remove,
};

static int __init bcm4774_init(void)
{
	printk("bcm4774 init\n");
	return platform_driver_register(&bcm4774_pm_driver);

}

static void __exit bcm4774_exit(void)
{
	platform_driver_unregister(&bcm4774_pm_driver);
}

late_initcall(bcm4774_init);
module_exit(bcm4774_init);
