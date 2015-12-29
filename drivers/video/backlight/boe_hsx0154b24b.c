/*
 *  LCD control code for truly
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/lcd.h>
#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

struct boe_hsx0154b24b_data {
	int lcd_power;
	struct lcd_device *lcd;
	struct lcd_platform_data *ctrl;
	struct regulator *vccio_regulator;
	struct regulator *vcc_regulator;
};

static int boe_hsx0154b24b_set_power(struct lcd_device *lcd, int power)
{
	struct boe_hsx0154b24b_data *dev= lcd_get_data(lcd);

	if (!power && dev->lcd_power) {
		dev->ctrl->power_on(lcd, 1);
	} else if (power && !dev->lcd_power) {
		if (dev->ctrl->reset) {
			dev->ctrl->reset(lcd);
		}
		dev->ctrl->power_on(lcd, 0);
	}
	dev->lcd_power = power;
	return 0;
}

static int boe_hsx0154b24b_get_power(struct lcd_device *lcd)
{
	struct boe_hsx0154b24b_data *dev= lcd_get_data(lcd);

	return dev->lcd_power;
}

static int boe_hsx0154b24b_set_mode(struct lcd_device *lcd, struct fb_videomode *mode)
{
	return 0;
}

static struct lcd_ops boe_hsx0154b24b_ops = {
	.early_set_power = boe_hsx0154b24b_set_power,
	.get_power = boe_hsx0154b24b_get_power,
	.set_mode = boe_hsx0154b24b_set_mode,
};

static int boe_hsx0154b24b_regulator_get(struct boe_hsx0154b24b_data *dev)
{
	int err = 0;
	dev->vccio_regulator = regulator_get(NULL, "lcd_1v8");
	if(IS_ERR(dev->vccio_regulator)) {
		printk("get vccio_regulator failed!\n");
		err = PTR_ERR(dev->vccio_regulator);
		goto err_get_regulator_vccio;
	}

	dev->vcc_regulator = regulator_get(NULL, "lcd_2v8");
	if(IS_ERR(dev->vcc_regulator)) {
		printk("get vcc_regulator failed!\n");
		err = PTR_ERR(dev->vcc_regulator);
		goto err_get_regulator_vcc;
	}

	regulator_enable(dev->vccio_regulator);
	regulator_enable(dev->vcc_regulator);

	return 0;


err_get_regulator_vcc:
	regulator_put(dev->vccio_regulator);
err_get_regulator_vccio:
	return err;
}

static int boe_hsx0154b24b_probe(struct platform_device *pdev)
{
	int ret;
	struct boe_hsx0154b24b_data *dev;

	dev = kzalloc(sizeof(struct boe_hsx0154b24b_data), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->ctrl = pdev->dev.platform_data;
	if (dev->ctrl == NULL) {
		dev_info(&pdev->dev, "no platform data!");
		return -EINVAL;
	}

	dev->lcd = lcd_device_register("boe_hsx0154b24b_slcd", &pdev->dev,
				       dev, &boe_hsx0154b24b_ops);
	if (IS_ERR(dev->lcd)) {
		ret = PTR_ERR(dev->lcd);
		dev->lcd = NULL;
		dev_info(&pdev->dev, "lcd device register error: %d\n", ret);
	} else {
		dev_info(&pdev->dev, "lcd device(BOE HSX0154B24B) register success\n");
	}
	ret = boe_hsx0154b24b_regulator_get(dev);
	if( ret != 0) {
		printk("boe hsx0154b24b regulator get failed!\n");
		goto err_regulator_get;
	}

	dev_set_drvdata(&pdev->dev, dev);

	return 0;

err_regulator_get:
	lcd_device_unregister(dev->lcd);
	return ret;
}

static int boe_hsx0154b24b_remove(struct platform_device *pdev)
{
	struct boe_hsx0154b24b_data *dev = dev_get_drvdata(&pdev->dev);

	if (dev->lcd_power)
		dev->ctrl->power_on(dev->lcd, 0);

	lcd_device_unregister(dev->lcd);
	dev_set_drvdata(&pdev->dev, NULL);

	regulator_put(dev->vccio_regulator);
	regulator_put(dev->vcc_regulator);

	kfree(dev);

	return 0;
}

#ifdef CONFIG_PM
static int boe_hsx0154b24b_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	struct boe_hsx0154b24b_data *dev;

	dev = (struct boe_hsx0154b24b_data *)dev_get_drvdata(&pdev->dev);
	if(IS_ERR(dev)) {
		printk("boe hsx0154b24b suspend dev get drvdata failed !\n");
		return 0;
	}
	boe_hsx0154b24b_set_power(dev->lcd, 0);

	regulator_force_disable(dev->vccio_regulator);
	regulator_force_disable(dev->vcc_regulator);

	dev->lcd_power = 0;

	return 0;
}

static int boe_hsx0154b24b_resume(struct platform_device *pdev)
{
	struct boe_hsx0154b24b_data *dev;

	dev = (struct boe_hsx0154b24b_data *)dev_get_drvdata(&pdev->dev);
	if(IS_ERR(dev)) {
		printk("boe hsx0154b24b suspend dev get drvdata failed !\n");
		return 0;
	}
	boe_hsx0154b24b_set_power(dev->lcd, 1);

	regulator_enable(dev->vccio_regulator);
	regulator_enable(dev->vcc_regulator);

	dev->lcd_power = 1;

	return 0;
}
#else
#define boe_hsx0154b24b_suspend	NULL
#define boe_hsx0154b24b_resume	NULL
#endif

static struct platform_driver boe_hsx0154b24b_driver = {
	.driver		= {
		.name	= "boe_hsx0154b24b_slcd",
		.owner	= THIS_MODULE,
	},
	.probe		= boe_hsx0154b24b_probe,
	.remove		= boe_hsx0154b24b_remove,
	.suspend	= boe_hsx0154b24b_suspend,
	.resume		= boe_hsx0154b24b_resume,
};

static int __init boe_hsx0154b24b_init(void)
{
	return platform_driver_register(&boe_hsx0154b24b_driver);
}
module_init(boe_hsx0154b24b_init);

static void __exit boe_hsx0154b24b_exit(void)
{
	platform_driver_unregister(&boe_hsx0154b24b_driver);
}
module_exit(boe_hsx0154b24b_exit);

MODULE_DESCRIPTION("BOE HSX0154B24B lcd panel driver");
MODULE_LICENSE("GPL");
