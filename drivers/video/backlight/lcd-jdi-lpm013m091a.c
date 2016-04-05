/*
 * Copyright (C) 2016 Ingenic Electronics
 *
 * JDI 1.34" 320*300 MIPI LCD Driver
 *
 * Model : LPM013M091A
 *
 * Author: MaoLei.Wang <maolei.wang@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/lcd.h>
#include <linux/fb.h>
#include <linux/backlight.h>

#include <video/mipi_display.h>
#include <mach/jz_dsim.h>

#define POWER_IS_ON(pwr)	((pwr) == FB_BLANK_UNBLANK)
#define POWER_IS_OFF(pwr)	((pwr) == FB_BLANK_POWERDOWN)
#define POWER_IS_NRM(pwr)	((pwr) == FB_BLANK_NORMAL)

#define lcd_to_master(a)	(a->dsim_dev->master)
#define lcd_to_master_ops(a)	((lcd_to_master(a))->master_ops)
#define write_command(dsi, data) dsi->master_ops->cmd_write(dsi, data, ARRAY_SIZE(data))

struct jdi_lpm013m091a_dev {
	struct device *dev;
	unsigned int id;
	unsigned int power;

	struct lcd_device *ld;
	struct mipi_dsim_lcd_device *dsim_dev;
	struct lcd_platform_data    *ddi_pd;
	struct mutex lock;
};

static void jdi_lpm013m091a_sleep_in(struct dsi_device *dsi) /* exit sleep */
{
	unsigned char data_to_send[] = {0x15, 0x10, 0x00};

	write_command(dsi, data_to_send);
}

static void jdi_lpm013m091a_sleep_out(struct dsi_device *dsi) /* exit sleep */
{
	unsigned char data_to_send[] = {0x15, 0x11, 0x00};

	write_command(dsi, data_to_send);
}

static void jdi_lpm013m091a_display_off(struct dsi_device *dsi) /* display on */
{
	unsigned char data_to_send[] = {0x15, 0x28, 0x00};

	write_command(dsi, data_to_send);
}

static void jdi_lpm013m091a_display_on(struct dsi_device *dsi) /* display on */
{
	unsigned char data_to_send[] = {0x15, 0x29, 0x00};

	write_command(dsi, data_to_send);
}

static void jdi_lpm013m091a_soft_reset(struct dsi_device *dsi) /* soft reset */
{
	unsigned char data_to_send[] = {0x15, 0x01, 0x00};

	write_command(dsi, data_to_send);
}

static void panel_init_set_sequence(struct dsi_device *dsi)
{
	unsigned char jdi_lpm013m091a_cmd_list[] = {
		0x15, 0xB3, 0x02,
		0x15, 0xBB, 0x10,
		/* Optinal setting (Register list) */
		0x15, 0x3A, 0x06,
		0x05, 0x11, 0x00,
		/* O-1 AM write new image by MIPI */
		0x15, 0xFF, 0x10,
		0x15, 0x3A, 0x06,
		0x39, 0x05, 0x00, 0x2A, 0x00, 0x00, 0x01, 0x3F,
		0x39, 0x05, 0x00, 0x2B, 0x00, 0x00, 0x01, 0x2B,
	};

	jdi_lpm013m091a_soft_reset(dsi);
	mdelay(10); /* Wait more than 10 ms */

	write_command(dsi, jdi_lpm013m091a_cmd_list);

	return;
}


static void jdi_lpm013m091a_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct jdi_lpm013m091a_dev *lcd = dev_get_drvdata(&dsim_dev->dev);
	struct dsi_device *dsi = lcd_to_master(lcd);

	if (!lcd->ddi_pd->lcd_enabled) {
		mutex_lock(&lcd->lock);
		panel_init_set_sequence(dsi);
		jdi_lpm013m091a_sleep_out(dsi);
		lcd->power = FB_BLANK_UNBLANK;
		mutex_unlock(&lcd->lock);
	}

	return;
}

static int jdi_lpm013m091a_ioctl(struct mipi_dsim_lcd_device *dsim_dev, int cmd)
{
	struct jdi_lpm013m091a_dev *lcd = dev_get_drvdata(&dsim_dev->dev);
	struct dsi_device *dsi = lcd_to_master(lcd);

	if (!lcd) {
		pr_err(" jdi_lpm013m091a_ioctl get drv failed\n");
		return -EFAULT;
	}

	mutex_lock(&lcd->lock);
	switch (cmd) {
	case CMD_MIPI_DISPLAY_ON:
		jdi_lpm013m091a_display_on(dsi);
		break;
	default:
		break;
	}
	mutex_unlock(&lcd->lock);

	return 0;
}

static int jdi_lpm013m091a_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct jdi_lpm013m091a_dev *lcd = NULL;

	lcd = devm_kzalloc(&dsim_dev->dev, sizeof(struct jdi_lpm013m091a_dev), GFP_KERNEL);
	if (!lcd) {
		dev_err(&dsim_dev->dev, "failed to allocate jdi_lpm013m091a_dev structure.\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->ddi_pd = (struct lcd_platform_data *)dsim_dev->platform_data;
	lcd->dev = &dsim_dev->dev;

	mutex_init(&lcd->lock);

	lcd->ld = lcd_device_register("jdi_lpm013m091a_dev", lcd->dev, lcd, NULL);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register lcd ops.\n");
		return PTR_ERR(lcd->ld);
	}

	dev_set_drvdata(&dsim_dev->dev, lcd);
	dev_dbg(lcd->dev, "probed jdi_lpm013m091a_dev panel driver.\n");

	return 0;
}

#ifdef CONFIG_PM
static void jdi_lpm013m091a_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power)
{
	struct jdi_lpm013m091a_dev *lcd = dev_get_drvdata(&dsim_dev->dev);

	if (!lcd->ddi_pd->lcd_enabled) {
		/* lcd power on */
		if (lcd->ddi_pd->power_on) {
			lcd->ddi_pd->power_on(lcd->ld, power);
		}

		if (power != POWER_ON_BL) {
			/* lcd reset */
			if (lcd->ddi_pd->reset) {
				lcd->ddi_pd->reset(lcd->ld);
			}
		}
	}

	return;
}

static int jdi_lpm013m091a_suspend(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct jdi_lpm013m091a_dev *lcd = dev_get_drvdata(&dsim_dev->dev);
	struct dsi_device *dsi = lcd_to_master(lcd);

	lcd->ddi_pd->lcd_enabled = 0;

	mutex_lock(&lcd->lock);

	jdi_lpm013m091a_sleep_in(dsi);
	msleep(120);
	jdi_lpm013m091a_display_off(dsi);
	jdi_lpm013m091a_power_on(dsim_dev, !POWER_ON_LCD);

	mutex_unlock(&lcd->lock);

	return 0;
}
#else
#define jdi_lpm013m091a_suspend		NULL
#define jdi_lpm013m091a_resume		NULL
#define jdi_lpm013m091a_power_on		NULL
#endif

static struct mipi_dsim_lcd_driver jdi_lpm013m091a_dsim_ddi_driver = {
	.name = "jdi_lpm013m081a-lcd",
	.id   = 0,
	.set_sequence = jdi_lpm013m091a_set_sequence,
	.ioctl    = jdi_lpm013m091a_ioctl,
	.probe    = jdi_lpm013m091a_probe,
	.suspend  = jdi_lpm013m091a_suspend,
	.power_on = jdi_lpm013m091a_power_on,
};

static int jdi_lpm013m091a_init(void)
{
	mipi_dsi_register_lcd_driver(&jdi_lpm013m091a_dsim_ddi_driver);
	return 0;
}

static void jdi_lpm013m091a_exit(void)
{
	return;
}

arch_initcall_sync(jdi_lpm013m091a_init);
module_exit(jdi_lpm013m091a_exit);

MODULE_DESCRIPTION("AUO 1.39 400*400 MIPI LCD Driver");
MODULE_LICENSE("GPL");
