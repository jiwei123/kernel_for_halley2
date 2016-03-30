#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/linux_logo.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include "../jz_fb_v12/jz_fb.h"
#include "jz_logo.h"

const struct linux_logo * charge_logo[] = {
        &charge_logo_1_clut224,
        &charge_logo_2_clut224,
        &charge_logo_3_clut224,
        &charge_logo_4_clut224,
        &charge_logo_5_clut224,
        &charge_logo_6_clut224,
};

int show_charge_logo(struct fb_info * info) {
    int time = 12;
    int count = 0;
    int size = ARRAY_SIZE(charge_logo);
    struct jzfb *jzfb = info->par;

    fb_blank(info, FB_BLANK_POWERDOWN);
    fb_blank(info, FB_BLANK_UNBLANK);

    while (count < time) {
        if (jz_fb_prepare_logo(info, charge_logo[count%size], FB_ROTATE_CW)) {
            fb_show_logo(info, FB_ROTATE_UR);
        }
        count++;
        jzfb_slcd_restart(jzfb);
        msleep(800);
    }
    fb_blank(info, FB_BLANK_POWERDOWN);
}
EXPORT_SYMBOL_GPL(show_charge_logo);

int show_boot_logo(struct fb_info *info)
{
    fb_blank(info, FB_BLANK_POWERDOWN);

    if (fb_prepare_logo(info, FB_ROTATE_UR)) {
        /* Start display and show logo on boot */
        fb_show_logo(info, FB_ROTATE_UR);
    }

    fb_blank(info, FB_BLANK_UNBLANK);
    return 0;
}
EXPORT_SYMBOL_GPL(show_boot_logo);

int show_low_battery_logo(struct fb_info *info)
{
    fb_blank(info, FB_BLANK_POWERDOWN);

    if (jz_fb_prepare_logo(info, &logo_low_battery_clut224, FB_ROTATE_UR)) {
        fb_show_logo(info, FB_ROTATE_UR);
    }

    fb_blank(info, FB_BLANK_UNBLANK);
    msleep(2000);
    fb_blank(info, FB_BLANK_POWERDOWN);
    return 0;
}
EXPORT_SYMBOL_GPL(show_low_battery_logo);

int battery_is_low() {
    struct class_dev_iter iter;
    struct device *dev;
    struct power_supply *psy;
    union power_supply_propval ret = { 0, };

    class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
    while ((dev = class_dev_iter_next(&iter))) {
        psy = dev_get_drvdata(dev);
        if (psy->type == POWER_SUPPLY_TYPE_BATTERY) {
            psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &ret);
            break;
        }
    }
    return (ret.intval < 10) ? 1 : 0;
}
EXPORT_SYMBOL_GPL(battery_is_low);

int battery_is_charging() {
    struct class_dev_iter iter;
    struct device *dev;
    struct power_supply *psy;
    union power_supply_propval ret = { 0, };

    class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
    while ((dev = class_dev_iter_next(&iter))) {
        psy = dev_get_drvdata(dev);
        if (psy->type == POWER_SUPPLY_TYPE_BATTERY) {
            psy->get_property(psy, POWER_SUPPLY_PROP_CHARGE_NOW, &ret);
            break;
        }
    }
    return ret.intval;
}
EXPORT_SYMBOL_GPL(battery_is_charging);

int show_logo(struct fb_info *info) {
    if (battery_is_low()) {
        if (battery_is_charging()) {
            show_charge_logo(info);
        }
        else {
            show_low_battery_logo(info);
        }
        pm_power_off();
    }
    else {
        show_boot_logo(info);
    }
}
EXPORT_SYMBOL_GPL(show_logo);





