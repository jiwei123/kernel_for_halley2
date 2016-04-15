/* Ingenic Fastboot Command Explain Gadget Driver
 *
 *  Copyright (C) 2013 Ingenic Semiconductor Co., LTD.
 *  Sun Jiwei <jwsun@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/usb/composite.h>
#include <linux/module.h>
#include "./g_fastboot.h"
#include "f_fastboot.c"

#define STRING_MANUFACTURER 25
#define STRING_PRODUCT 2
#define STRING_USBDOWN 0
#define CONFIG_USBDOWNLOADER 1

#define DRIVER_VERSION		"usb_fastboot 1.0"

static const char shortname[] = "usb_";
static const char product[] = "Android 1.0";
static const char manufacturer[] = "Google, Ingenic";
static const char serialnum[] = "0123456789";

static struct usb_device_descriptor device_desc = {
	.bLength = sizeof device_desc,
	.bDescriptorType = USB_DT_DEVICE,

	.bcdUSB = __constant_cpu_to_le16(0x0200),
	.bDeviceClass = USB_CLASS_VENDOR_SPEC,
	.bDeviceSubClass = 0x42, /*0x02:CDC-modem , 0x00:CDC-serial*/
	.bDeviceProtocol = 0x03,
	.bMaxPacketSize0 = 0x40,

	.idVendor = __constant_cpu_to_le16(CONFIG_G_FASTBOOT_VENDOR_NUM),
	.idProduct = __constant_cpu_to_le16(CONFIG_G_FASTBOOT_PRODUCT_NUM),
	.bcdDevice = __constant_cpu_to_le16(0xffff),
	.iManufacturer = 1,
	.iProduct = STRING_PRODUCT,
	.bNumConfigurations = 1,
};

/* static strings, in UTF-8 */
static struct usb_string g_fastboot_string_defs[] = {
	{ 0, manufacturer, },
	{ 1, product, },
	{ 2, serialnum, },
	{  }		/* end of list */
};

static struct usb_gadget_strings g_fastboot_string_tab = {
	.language = 0x0409, /* en-us */
	.strings = g_fastboot_string_defs,
};

static struct usb_gadget_strings *g_fastboot_composite_strings[] = {
	&g_fastboot_string_tab,
	NULL,
};

static int g_fastboot_unbind(struct usb_composite_dev *cdev)
{
	struct usb_gadget *gadget = cdev->gadget;

	printk("%s: calling usb_gadget_disconnect for "
			"controller '%s'\n", shortname, gadget->name);
	usb_gadget_disconnect(gadget);

	return 0;
}

static int g_fastboot_do_config(struct usb_configuration *c)
{
	const char *s = c->cdev->driver->name;
	int ret = -1;

	printk("%s: configuration: 0x%p composite dev: 0x%p\n",
	      __func__, c, c->cdev);

	printk("GADGET DRIVER: %s\n", s);
	if (!strcmp(s, "usb_fastboot"))
		ret = fastboot_add(c);

	return ret;
}

static int g_fastboot_config_register(struct usb_composite_dev *cdev)
{
	static struct usb_configuration config = {
		.label = "usb_fastbootoad",

		.bmAttributes =	USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_BATTERY,
		.MaxPower = 0x80,
		.bConfigurationValue =	CONFIG_USBDOWNLOADER,
		.iConfiguration =	STRING_USBDOWN,
	};

	return usb_add_config(cdev, &config, g_fastboot_do_config);
}

__weak
int g_fastboot_bind_fixup(struct usb_device_descriptor *dev)
{
	return 0;
}

static int g_fastboot_bind(struct usb_composite_dev *cdev)
{
	struct usb_gadget *gadget = cdev->gadget;
	int id, ret;

	printk("%s: gadget: 0x%p cdev: 0x%p\n", __func__, gadget, cdev);

	id = usb_string_id(cdev);

	if (id < 0)
		return id;
	g_fastboot_string_defs[0].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;

	g_fastboot_string_defs[1].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	g_fastboot_string_defs[2].id = id;
	device_desc.iSerialNumber = id;

	g_fastboot_bind_fixup(&device_desc);
	ret = g_fastboot_config_register(cdev);
	if (ret)
		goto error;

	printk("%s: calling usb_gadget_connect for "
			"controller '%s'\n", shortname, gadget->name);
	usb_gadget_connect(gadget);

	return 0;

 error:
	g_fastboot_unbind(cdev);
	return -ENOMEM;
}

static struct usb_composite_driver g_fastboot_driver = {
	.name = "usb_fastboot",
	.dev = &device_desc,
	.strings = g_fastboot_composite_strings,

	.bind = g_fastboot_bind,
	.unbind = g_fastboot_unbind,
};

static int __init g_fastboot_init(void){
    usb_composite_probe(&g_fastboot_driver);
    handle_fastboot_cmd();
    usb_composite_unregister(&g_fastboot_driver);
    return 0;
}

static void __exit g_fastboot_exit(void){
}

late_initcall(g_fastboot_init);
module_exit(g_fastboot_exit);
MODULE_LICENSE("GPL");
