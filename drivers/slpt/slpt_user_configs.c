/*
 *  Copyright (C) 2015 Wu Jiao <jwu@ingenic.cn wujiaososo@qq.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/slpt.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/syscalls.h> /* sys_sync */
#include <linux/slpt_battery.h>
#include <linux/dma-mapping.h>
#include <linux/bootmem.h>
#include <linux/slpt_cache.h>
#include <linux/default_fb.h>
#include <linux/default_backlight.h>
#include <linux/suspend_state.h>

static DEFINE_MUTEX(slpt_configs_lock);

#ifndef CONFIG_LCD_BRIGHTNESS_ALWAYS_ON_LEVEL
#define CONFIG_LCD_BRIGHTNESS_ALWAYS_ON_LEVEL 102
#endif

int fb_always_on = 0;
int brightness_always_on = 0;
unsigned int brightness_always_on_level = CONFIG_LCD_BRIGHTNESS_ALWAYS_ON_LEVEL;
static int display_on = 0;
static int brightness_locked = 0;
static int saved_brightness = -1;

int brightness_is_always_on(void) {
	return brightness_always_on;
}

int fb_is_always_on(void) {
	return fb_always_on;
}

unsigned int get_brightness_always_on_level(void) {
	return brightness_always_on_level;
}

void set_fb_always_on(int on) {
	fb_always_on = !!on;
}

void set_brightness_always_on(int on) {
	brightness_always_on = !!on;
}

void set_brightness_always_on_level(unsigned int level) {
	unsigned int suspend_state;

	hold_suspend_lock();
	suspend_state = get_suspend_state_no_lock();
	brightness_always_on_level = level;
	if (display_on && suspend_state != STATE_NO_SUSPEND)
		set_brightness_of_default_backlight(level);
	release_suspend_lock();
}

void set_display_on(int on) {
	unsigned int suspend_state;

	hold_suspend_lock();
	suspend_state = get_suspend_state_no_lock();

	set_fb_always_on(on);
	set_brightness_always_on(on);

	if (display_on && !on && suspend_state != STATE_NO_SUSPEND) {
		set_brightness_of_default_backlight(0);
		power_off_default_fb();
	} else if (!display_on && on && suspend_state != STATE_NO_SUSPEND) {
		power_on_default_fb();
		set_brightness_of_default_backlight(brightness_always_on_level);
	}

	display_on = !!on;

	release_suspend_lock();
}

int brightness_is_locked(void) {
	return brightness_locked;
}

void set_brightness_locked(int on) {
	int brightness;

	if (!!brightness_locked == !!on)
		return;

	brightness_locked = !!on;
	brightness = get_brightness_of_default_backlight();

	if (on) {
		pr_err("save brightness is %d\n", brightness);
		saved_brightness = brightness;
	} else {
		if (saved_brightness > brightness)
			brightness = saved_brightness;
		if (fb_always_on)
			set_brightness_of_default_backlight(brightness);
		pr_err("restore brightness is %d\n", brightness);
	}
}

/*
 * config: fb_always_on
 */
static ssize_t fb_always_on_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_configs_lock);
	count = sprintf(buf, "%d\n", !!fb_always_on);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

static ssize_t fb_always_on_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	int enable = simple_strtol(buf, NULL, 10);

	mutex_lock(&slpt_configs_lock);
	set_fb_always_on(enable);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

slpt_attr(fb_always_on);

/*
 * config: brightness_always_on
 */
static ssize_t brightness_always_on_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_configs_lock);
	count = sprintf(buf, "%d\n", !!brightness_always_on);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

static ssize_t brightness_always_on_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	int enable = simple_strtol(buf, NULL, 10);

	mutex_lock(&slpt_configs_lock);
	set_brightness_always_on(enable);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

slpt_attr(brightness_always_on);

/*
 * config: brightness_always_on_level
 */
static ssize_t brightness_always_on_level_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_configs_lock);
	count = sprintf(buf, "%d\n", brightness_always_on_level);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

static ssize_t brightness_always_on_level_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	int level = simple_strtol(buf, NULL, 10);

	mutex_lock(&slpt_configs_lock);
	set_brightness_always_on_level(level);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

slpt_attr(brightness_always_on_level);

/*
 * config: display on/off
 */
static ssize_t display_ctrl_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_configs_lock);
	count = sprintf(buf, "%d\n", display_on);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

static ssize_t display_ctrl_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	int enable = simple_strtol(buf, NULL, 10);

	mutex_lock(&slpt_configs_lock);
	set_display_on(enable);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

slpt_attr(display_ctrl);

/*
 * config: display on/off
 */
static ssize_t lock_brightness_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_configs_lock);
	count = sprintf(buf, "%d\n", brightness_locked);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

static ssize_t lock_brightness_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	int enable = simple_strtol(buf, NULL, 10);

	mutex_lock(&slpt_configs_lock);
	set_brightness_locked(enable);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

slpt_attr(lock_brightness);

/*
 * config: display on/off
 */
static ssize_t default_brightness_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_configs_lock);
	count = sprintf(buf, "%d\n", get_brightness_of_default_backlight());
	mutex_unlock(&slpt_configs_lock);

	return count;
}

static ssize_t default_brightness_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	int brightness = simple_strtol(buf, NULL, 10);

	mutex_lock(&slpt_configs_lock);
	set_brightness_of_default_backlight(brightness);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

slpt_attr(default_brightness);

static struct attribute *slpt_configs_attributes[] = {
	&fb_always_on_attr.attr,
	&brightness_always_on_attr.attr,
	&brightness_always_on_level_attr.attr,
	&display_ctrl_attr.attr,
	&lock_brightness_attr.attr,
	&default_brightness_attr.attr,
	NULL,
};

static struct attribute_group slpt_configs_attrs_g = {
	.attrs = slpt_configs_attributes,
	.name = NULL,
};

int __init slpt_configs_init(void) {
	int ret;

	ret = sysfs_create_group(slpt_configs_kobj, &slpt_configs_attrs_g);
	if (ret) {
		pr_err("SLPT: error: slpt configs sysfs group create failed\n");
	}

	return ret;
}

int __exit slpt_configs_exit(void) {
	sysfs_remove_group(slpt_configs_kobj, &slpt_configs_attrs_g);
	
	return 0;
}
