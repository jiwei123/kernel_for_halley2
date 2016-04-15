/* Ingenic JZ Fastboot Command Explain Function Driver
 *
 *  Copyright (C) 2013 Ingenic Semiconductor Co., LTD.
 *  Sun Jiwei <jwsun@ingenic.cn>
 *
 * This program is kfree software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the kfree Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the kfree Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fcntl.h>
#include <linux/reboot.h>
#include <asm/unaligned.h>
#include <linux/err.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>
#include <linux/delay.h>
#include "./g_fastboot.h"
#include <asm/io.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include "../../mmc/host/jzmmc_utils.h"
#define DATA_BUFFER	0x30000000
#define BOOT_START_ADDRESS 0x80f00000
#define RET_LENGTH	64
#define CONFIG_SYS_MALLOC_LEN       (16 * 1024 * 1024)
#define WAIT_CMD_BEFORE_LEAVE 600

#define CMD_COUNT	22
#define MMC_BYTE_PER_BLOCK 512

#define PARTITION_NUM 11
#define CONFIG_FASTBOOT_BOOTLOADER_VER  "2013-03-22"
#define CONFIG_FASTBOOT_BASEBAND_VER    "N/A"
#define CONFIG_FASTBOOT_HARDWARE_VER    "V1.1 20130322"
#define CONFIG_FASTBOOT_CDMA_VER    "N/A"
#define CONFIG_FASTBOOT_VARIANT     "INGENIC"
#define CONFIG_FASTBOOT_PRODUCT     "INGENIC"

#define CONFIG_FASTBOOT_SERIALNO    "0123456789abcdef"
#define CONFIG_FASTBOOT_SECURE      "no"
#define CONFIG_FASTBOOT_UNLOCKED    "yes"
#define CONFIG_FASTBOOT_UART_ON     "NO"
#define CONFIG_FASTBOOT_PART_SIZE_BOOTLOADER    "0x300000"
#define CONFIG_FASTBOOT_PART_TYPE_BOOTLOADER    "emmc"
#define CONFIG_FASTBOOT_PART_SIZE_RECOVERY  "0x800000"
#define CONFIG_FASTBOOT_PART_TYPE_RECOVERY  "emmc"
#define CONFIG_FASTBOOT_PART_SIZE_BOOT      "0x800000"
#define CONFIG_FASTBOOT_PART_TYPE_BOOT      "emmc"
#define CONFIG_FASTBOOT_PART_SIZE_SYSTEM    "0x20000000"
#define CONFIG_FASTBOOT_PART_TYPE_SYSTEM    "ext4"
#define CONFIG_FASTBOOT_PART_SIZE_CACHE     "0x1E00000"
#define CONFIG_FASTBOOT_PART_TYPE_CACHE     "ext4"
#define CONFIG_FASTBOOT_PART_SIZE_USERDATA  "0x40000000"
#define CONFIG_FASTBOOT_PART_TYPE_USERDATA  "ext4"

struct partition_info {
    char pname[16];
    unsigned long offset;
    unsigned long size;
};

struct partition_info partition_info[PARTITION_NUM] = {
   [0] = {.pname = "ndxboot",     .offset = 0*1024*1024,    .size = 3*1024*1024},
   [1] = {.pname = "ndboot",      .offset = 3*1024*1024,    .size = 9*1024*1024},
   [2] = {.pname = "ndrecovery",  .offset = 12*1024*1024,   .size = 16*1024*1024},
   [3] = {.pname = "ndmisc",      .offset = 28*1024*1024,   .size = 16*1024*1024},
   [4] = {.pname = "ndreserved",  .offset = 44*1024*1024,   .size = 52*1024*1024},
   [5] = {.pname = "ndoeminfo",   .offset = 96*1024*1024,   .size = 4*1024*1024},
   [6] = {.pname = "ndcache",     .offset = 100*1024*1024,  .size = 100*1024*1024},
   [7] = {.pname = "ndsystem",    .offset = 200*1024*1024,  .size = 824*1024*1024},
   [8] = {.pname = "nddata",      .offset = 1024*1024*1024, .size = (unsigned long)2560*1024*1024},
};
struct fastboot_dev {
	struct usb_request	*cmd_req;
	struct usb_request	*data_req;
	struct usb_request	*ret_req;
	struct work_struct  work;
	char			*data_buf;
	char			*ret_buf;
	bool			explain_cmd_status;
	u32			data_length;

	int             last_flash_part;
	unsigned long   flashed_size;
	int         connecting;
	int			locked;
	int			leave;
	int			cmd_count;
};

struct fastboot_common {
    struct fastboot_dev * fastboot;
    struct usb_gadget   *gadget;    /*Copy of cdev->gadget*/
    struct usb_function     usb_function;
    struct usb_ep       *bulk_out;
    struct usb_ep       *bulk_in;
    struct usb_ep       *ep0;       /*Copy of gadget->ep0*/
    struct usb_request  *ep0req;    /*Copy of cdev->req*/
    bool            enum_done;
};

struct fastboot_common *fastboot_common;

static const char fastboot_name[] = "Google, Inc";

static struct usb_string fastboot_string_desc[] = {
	[0].s = fastboot_name,
	{}
};

static struct usb_gadget_strings  fastboot_gadget_string = {
	.language = 0x0409, /* en-us */
	.strings = fastboot_string_desc,
};

static struct usb_gadget_strings  *fastboot_gadget_string_tab[] = {
	&fastboot_gadget_string,
	NULL,
};

static struct usb_interface_descriptor fastboot_intf_desc = {
	.bLength =              sizeof(fastboot_intf_desc),
	.bDescriptorType =      USB_DT_INTERFACE,
	.bNumEndpoints =        2,

	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	0x42,
	.bInterfaceProtocol =	0x03,
};

static struct usb_endpoint_descriptor fastboot_fs_bulk_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fastboot_fs_bulk_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fastboot_hs_bulk_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor fastboot_hs_bulk_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_descriptor_header *fastboot_fs_desc_header[] = {
	(struct usb_descriptor_header *) &fastboot_intf_desc,
	(struct usb_descriptor_header *) &fastboot_fs_bulk_out_desc,
	(struct usb_descriptor_header *) &fastboot_fs_bulk_in_desc,
	NULL,
};

static struct usb_descriptor_header *fastboot_hs_desc_header[] = {
	(struct usb_descriptor_header *) &fastboot_intf_desc,
	(struct usb_descriptor_header *) &fastboot_hs_bulk_out_desc,
	(struct usb_descriptor_header *) &fastboot_hs_bulk_in_desc,
	NULL,
};

static inline struct fastboot_common *fastboot_from_func(struct usb_function *f)
{
	return container_of(f, struct fastboot_common, usb_function);
}

static int fastboot_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct fastboot_common	*fastboot_common = fastboot_from_func(f);
	struct usb_gadget	*gadget = c->cdev->gadget;
	struct usb_ep		*ep;
	int			i;

	printk("bind !!!\n");
	fastboot_common->gadget = gadget;
	fastboot_common->ep0 = c->cdev->gadget->ep0;
	fastboot_common->ep0req = c->cdev->req;

	/* New interface */
	i = usb_interface_id(c, f);
	if (i < 0)
		return i;
	fastboot_intf_desc.bInterfaceNumber = i;

	ep = usb_ep_autoconfig(gadget, &fastboot_fs_bulk_in_desc);
	if (!ep) {
	    printk("usb_ep_autoconfig for ep_in failed\n");
		goto autoconf_fail;
	}
	ep->driver_data = fastboot_common;
	fastboot_common->bulk_in = ep;

	ep = usb_ep_autoconfig(gadget, &fastboot_fs_bulk_out_desc);
	if (!ep) {
	    printk("usb_ep_autoconfig for ep_out failed\n");
	    goto autoconf_fail;
	}
	ep->driver_data = fastboot_common;
	fastboot_common->bulk_out = ep;

	if (gadget_is_dualspeed(c->cdev->gadget)) {
		fastboot_hs_bulk_in_desc.bEndpointAddress =
			fastboot_fs_bulk_in_desc.bEndpointAddress;
		fastboot_hs_bulk_out_desc.bEndpointAddress =
			fastboot_fs_bulk_out_desc.bEndpointAddress;
	}

	return 0;
autoconf_fail:
	printk("unable to autoconfigure all endpoints\n");
	return -ENOTSUPP;
}

static void fastboot_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct fastboot_common	*fastboot_common = fastboot_from_func(f);

	printk("unbind\n");

	kfree(fastboot_common);
	fastboot_common = NULL;
	return;
}

static int fastboot_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct fastboot_common *fastboot_common = fastboot_from_func(f);
	struct usb_ep *in_ep = fastboot_common->bulk_in;
	struct usb_ep *out_ep = fastboot_common->bulk_out;
	int status = 0;

	printk("set interface %d alt %d\n",intf,alt);
	fastboot_common->fastboot->leave = 1;
	fastboot_common->fastboot->connecting = 1;

	/*Bulk in endpoint set alt*/
	if (in_ep->driver_data) {
		printk("set alt when ep in enable\n");
		status = usb_ep_disable(in_ep);
		if (status) {
			printk("usb disable ep in failed\n");
			goto failed;
		}
		in_ep->driver_data = NULL;
	}

	status = config_ep_by_speed(fastboot_common->gadget, f, in_ep);
    if (status) {
        printk("config ep by speed failed\n");
        return status;
    }

	status = usb_ep_enable(in_ep);
	if (status < 0) {
		printk("usb enable ep in failed\n");
		goto failed;
	}

	in_ep->driver_data = fastboot_common;

	/*Bulk out endpoint set alt*/
	if (out_ep->driver_data) {
		printk("set alt when ep out enable\n");
		status = usb_ep_disable(out_ep);
		if (status) {
			printk("usb disable ep out failed\n");
			goto failed;
		}

		out_ep->driver_data = NULL;
	}

    status = config_ep_by_speed(fastboot_common->gadget, f, out_ep);
    if (status)
        return status;

	status = usb_ep_enable(out_ep);
	if (status < 0) {
		printk("usb enable ep out failed\n");
		goto failed;
	}

	out_ep->driver_data = fastboot_common;
	fastboot_common->enum_done = 1;
	printk("fastboot set alt success\n");
	return 0;
failed:
printk("fastboot set alt failed\n");
	return status;
}

/* TODO: is this really what we need here? */
static void fastboot_disable(struct usb_function *f)
{
	return;
}

static int fastboot_handle(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct usb_gadget *gadget = f->config->cdev->gadget;
	struct fastboot_common *fastboot_common = f->config->cdev->req->context;
	struct usb_request *req = fastboot_common->ep0req;
#if 0
	u16 wLength = le16_to_cpu(ctrl->wLength);
	u16 wValue = le16_to_cpu(ctrl->wValue);
	u16 wIndex = le16_to_cpu(ctrl->wIndex);
#endif
	int length = 0;

	req->length = 0;
	if (length >= 0) {
		length = usb_ep_queue(gadget->ep0, req, GFP_KERNEL);
		if (length) {
			printk("ep_queue --> %d\n", length);
			req->status = 0;
		}
	}
	return length;
}

static int fastboot_bind_config(struct usb_configuration *c)
{
	int status;

	fastboot_common = kcalloc(sizeof(*fastboot_common), 1, GFP_KERNEL);
	if (!fastboot_common)
		return -ENOMEM;

	fastboot_common->usb_function.name = "fastboot";
	fastboot_common->usb_function.hs_descriptors = fastboot_hs_desc_header;
	fastboot_common->usb_function.fs_descriptors = fastboot_fs_desc_header;
	fastboot_common->usb_function.bind = fastboot_bind;
	fastboot_common->usb_function.unbind = fastboot_unbind;
	fastboot_common->usb_function.set_alt = fastboot_set_alt;
	fastboot_common->usb_function.disable = fastboot_disable;
	fastboot_common->usb_function.strings = fastboot_gadget_string_tab,
	fastboot_common->usb_function.setup = fastboot_handle,

	status = usb_add_function(c, &fastboot_common->usb_function);
	if (status) {
		kfree(fastboot_common);
		fastboot_common = NULL;
	}

	return status;
}

int fastboot_add(struct usb_configuration *c)
{
	int id;

	id = usb_string_id(c->cdev);
	if (id < 0)
		return id;
	fastboot_string_desc[0].id = id;
	fastboot_intf_desc.iInterface = id;

	printk("%s: cdev: 0x%p gadget:0x%p gadget->ep0: 0x%p\n", __func__,
			c->cdev, c->cdev->gadget, c->cdev->gadget->ep0);

	return fastboot_bind_config(c);
}

static int fastboot_cmd_return(struct fastboot_dev *fastboot,
		void (*complete)(struct usb_ep *ep, struct usb_request *req),
		int status, unsigned actual)
{
	int	ret;

	fastboot->ret_req->buf = fastboot->ret_buf;
	fastboot->ret_req->length = strlen(fastboot->ret_buf);
	fastboot->ret_req->complete = complete;
	fastboot->ret_req->status = status;
	fastboot->ret_req->actual = actual;
	fastboot->ret_req->context = fastboot;

	ret = usb_ep_queue(fastboot_common->bulk_in, fastboot->ret_req, GFP_KERNEL);
	if (ret) {
		printk("%s: Error in usb_ep_queue,%d\n", __func__, ret);
		return ret;
	}
	return 0;
}

static void return_buf(struct fastboot_dev *fastboot,
		void (*complete)(struct usb_ep *ep, struct usb_request *req))
{
	u32	actual = 0;
	int	status = 0;

	fastboot_cmd_return(fastboot, complete, status, actual);
}

static void return_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct fastboot_dev *fastboot = req->context;
	printk("return status is over\n");
	fastboot->explain_cmd_status = 1;
}

static void download_data_fail(struct fastboot_dev *fastboot)
{
	strcpy(fastboot->ret_buf, "FAILED");
	return_buf(fastboot, return_complete);
}

static void download_data_over(struct usb_ep *ep,
		struct usb_request *req)
{
	struct fastboot_dev *fastboot = req->context;

	usb_ep_free_request(fastboot_common->bulk_out, fastboot->data_req);
	fastboot->data_req = NULL;
	strcpy(fastboot->ret_buf, "OKAY");
	return_buf(fastboot, return_complete);
}

static void handle_download_complete(struct usb_ep *ep,
		struct usb_request *req)
{
	struct fastboot_dev *fastboot = req->context;

	fastboot->data_req = usb_ep_alloc_request(fastboot_common->bulk_out, GFP_KERNEL);
	if (!fastboot->data_req) {
		printk("%s: Error, usb alloc request\n", __func__);
		return;
	}

	fastboot->data_req->buf = fastboot->data_buf;
	fastboot->data_req->length = fastboot->data_length;
	fastboot->data_req->complete = download_data_over;
	fastboot->data_req->status = 0;
	fastboot->data_req->actual = 0;
	fastboot->data_req->context = fastboot;
	usb_ep_queue(fastboot_common->bulk_out, fastboot->data_req, GFP_KERNEL);
}

static int get_all_var(struct fastboot_dev *fastboot)
{
	switch (fastboot->cmd_count) {
		case 0:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "version-bootloader: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_BOOTLOADER_VER);
			break;
		case 1:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "version-baseband: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_BASEBAND_VER);
			break;
		case 2:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "version-hardware: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_HARDWARE_VER);
			break;
		case 3:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "version-cdma: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_CDMA_VER);
			break;
		case 4:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "variant: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_VARIANT);
			break;
		case 5:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "serialno: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_SERIALNO);
			break;
		case 6:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "product: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PRODUCT);
			break;
		case 7:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "secure: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_SECURE);
			break;
		case 8:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "unlocked: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_UNLOCKED);
			break;
		case 9:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "uart-on: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_UART_ON);
			break;
		case 10:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "partition-size:bootloader: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_SIZE_BOOTLOADER);
			break;
		case 11:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "partition-type:bootloader: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_TYPE_BOOTLOADER);
			break;
		case 12:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "partition-size:recovery: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_SIZE_RECOVERY);
			break;
		case 13:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "partition-type:recovery: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_TYPE_RECOVERY);
			break;
		case 14:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "partition-size:boot: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_SIZE_BOOT);
			break;
		case 15:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "partition-type:boot: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_TYPE_BOOT);
			break;
		case 16:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "partition-size:system: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_SIZE_SYSTEM);
			break;
		case 17:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "partition-type:system: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_TYPE_SYSTEM);
			break;
		case 18:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "partition-size:cache: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_SIZE_CACHE);
			break;
		case 19:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "partition-type:cache: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_TYPE_CACHE);
			break;
		case 20:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "partition-size:userdata: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_SIZE_USERDATA);
			break;
		case 21:
			strcpy(fastboot->ret_buf, "INFO");
			strcat(fastboot->ret_buf, "partition-type:userdata: ");
			strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_TYPE_USERDATA);
			break;
		case 22:
			strcpy(fastboot->ret_buf, "OKAY");
			break;
		case 23:
			strcpy(fastboot->ret_buf, "INFO");
			break;
		default:
			printk("%s:Error the command display over\n", __func__);
			return -1;
	}
	return 0;
}

static void return_getvar_all_complete(struct usb_ep *ep,
		struct usb_request *req)
{
	struct fastboot_dev *fastboot = req->context;

	memset(fastboot->ret_buf, 0, RET_LENGTH);
	if(fastboot->cmd_count < CMD_COUNT) {
		fastboot->cmd_count++;
		if (get_all_var(fastboot))
			strcpy(fastboot->ret_buf, "FAIL");

		return_buf(fastboot, return_getvar_all_complete);
	} else {
		fastboot->cmd_count = 0;
		strcpy(fastboot->ret_buf, "OKAY");
		printk("return status is over\n");
		fastboot->explain_cmd_status = 1;
	}
}

static void explain_cmd_getvar_all(struct fastboot_dev *fastboot)
{
	fastboot->cmd_count = 0;

	if (get_all_var(fastboot))
		strcpy(fastboot->ret_buf, "FAIL");

	return_buf(fastboot, return_getvar_all_complete);
}

static int handle_cmd_getvar(struct fastboot_dev *fastboot)
{
    char buf[20] = {0};
	if (strstr(fastboot->cmd_req->buf, "version-bootloader")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_BOOTLOADER_VER);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "version-baseband")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_BASEBAND_VER);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "version-hardware")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_HARDWARE_VER);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "version-cdma")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_CDMA_VER);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "variant")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_VARIANT);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "serialno")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_SERIALNO);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "product")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PRODUCT);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "secure")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_SECURE);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "unlocked")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_UNLOCKED);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "uart-on")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_UART_ON);
		return 0;
	}

    if (strstr(fastboot->cmd_req->buf, "max-download-size")) {
        strcpy(fastboot->ret_buf, "OKAY");
        num_to_str(buf, sizeof(buf), CONFIG_SYS_MALLOC_LEN);
        strcat(fastboot->ret_buf, buf);
        return 0;
    }

	if (strstr(fastboot->cmd_req->buf, "partition-size:bootloader")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_SIZE_BOOTLOADER);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "partition-type:bootloader")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_TYPE_BOOTLOADER);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "partition-size:recovery")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_SIZE_RECOVERY);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "partition-type:recovery")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_TYPE_RECOVERY);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "partition-size:boot")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_SIZE_BOOT);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "partition-type:boot")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_TYPE_BOOT);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "partition-size:system")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_SIZE_SYSTEM);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "partition-type:system")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_TYPE_SYSTEM);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "partition-size:cache")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_SIZE_CACHE);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "partition-type:cache")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_TYPE_CACHE);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "partition-size:userdata")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_SIZE_USERDATA);
		return 0;
	}

	if (strstr(fastboot->cmd_req->buf, "partition-type:userdata")) {
		strcpy(fastboot->ret_buf, "OKAY");
		strcat(fastboot->ret_buf, CONFIG_FASTBOOT_PART_TYPE_USERDATA);
		return 0;
	}

	printk("%s:Error, The variant is not recongized\n", __func__);
	return -1;
}

static void explain_cmd_getvar(struct fastboot_dev *fastboot)
{
	if (handle_cmd_getvar(fastboot))
		strcpy(fastboot->ret_buf, "FAIL");

	return_buf(fastboot, return_complete);
}

static int handle_cmd_download(struct fastboot_dev *fastboot)
{
    fastboot->data_buf = kmalloc(fastboot->data_length, GFP_KERNEL);
    if (!fastboot->data_buf) {
        printk("%s: malloc for data buffer failed\n", __func__);
        return -ENOMEM;
    }

	memset(fastboot->data_buf, 0, fastboot->data_length);
	return 0;
}

/* Called after flash command explain complete */
static void flash_complete(struct fastboot_dev *fastboot)
{
	memset(fastboot->data_buf, 0, fastboot->data_length);
	fastboot->data_length = 0;
	kfree(fastboot->data_buf);
	fastboot->data_buf = NULL;
}

static int explain_cmd_download(struct fastboot_dev *fastboot)
{
	char	buf_length[9];
	int	status = 0;
	u32	actual = 0;
	int	ret;
	strcpy(fastboot->ret_buf, "DATA");
	memcpy(buf_length, (char *)(fastboot->cmd_req->buf + 9), 8);
	buf_length[8] = '\0';
	strcat(fastboot->ret_buf, buf_length);
	fastboot->data_length = (u32)simple_strtol(buf_length, NULL, 16);

	ret = handle_cmd_download(fastboot);
	if (ret != 0) {
		printk("%s: Error in the handle download cmd:%d\n", __func__, ret);
		return ret;
	}

	ret = fastboot_cmd_return(fastboot, handle_download_complete, status, actual);
	if (ret != 0) {
		printk("%s: Error in the fastboot_cmd_return,%d\n",__func__, ret);
		return ret;
	}
	return 0;
}

static int mmc_flash(unsigned long blk,unsigned long cnt,struct fastboot_dev *fastboot)
{
    struct mmc_card * card = find_emmc_card();
    mmc_write(card, (unsigned char *)fastboot->data_buf, blk, cnt);
	printk("mmc flash OK!\n");
	flash_complete(fastboot);
	return 0;
}

static int handle_cmd_flash(struct fastboot_dev *fastboot, char * boot_buf)
{
	int i;
	unsigned long blk,cnt;

	for(i = 0; i < PARTITION_NUM ; i ++){
		if(!strncmp(partition_info[i].pname + 2 , boot_buf + 6,(strlen(partition_info[i].pname)-2))){
		    if (fastboot->last_flash_part == i) {
		        blk = partition_info[i].offset + fastboot->flashed_size;
		        fastboot->flashed_size += fastboot->data_length;
		    }
		    else {
		        blk = partition_info[i].offset;
		        fastboot->last_flash_part = i;
		        fastboot->flashed_size = fastboot->data_length;
		    }
		    cnt = fastboot->data_length;
			goto do_flash;
			break;
		}
	}
	if(i == PARTITION_NUM){
		printk("There is not a partition named : %s\n",boot_buf + 6);
		return -1;
	}
do_flash:
	if(!mmc_flash(blk, cnt, fastboot)) {
	    return 0;
	}
	return -1;
}

static void explain_cmd_flash(struct fastboot_dev *fastboot, char * boot_buf)
{
	if (handle_cmd_flash(fastboot, boot_buf))
		strcpy(fastboot->ret_buf, "FAILED");
	else
		strcpy(fastboot->ret_buf, "OKAY");

	return_buf(fastboot, return_complete);
}

static int fastboot_mmc_erase(unsigned long addr,int size, struct fastboot_dev *fastboot)
{
	struct mmc_card * card = find_emmc_card();
	mmc_erase_blocks(card, addr, size);
	printk("mmc erase ok\n");
	return 0;
}


static int handle_cmd_erase(struct fastboot_dev *fastboot, char * boot_buf)
{
	u32 blk,cnt;
	int i;
	for(i = 0; i < PARTITION_NUM ; i ++){
		if(!strncmp(partition_info[i].pname + 2 , boot_buf + 6,(strlen(partition_info[i].pname)-2))){
			blk = partition_info[i].offset / MMC_BYTE_PER_BLOCK;
			cnt = (partition_info[i].size + MMC_BYTE_PER_BLOCK - 1) / MMC_BYTE_PER_BLOCK;
			goto do_erase;
			break;
		}
	}
	if(i == PARTITION_NUM){
		printk("There is not a partition named : %s\n",boot_buf + 6);
		return -1;
	}

do_erase:
	if(!fastboot_mmc_erase(blk , cnt ,fastboot))
		return 0;

	return -1;
}

static void explain_cmd_erase(struct fastboot_dev *fastboot, char * boot_buf)
{
	if (handle_cmd_erase(fastboot, boot_buf))
		strcpy(fastboot->ret_buf, "FAILED");
	else
		strcpy(fastboot->ret_buf, "OKAY");

	return_buf(fastboot, return_complete);
}

static int handle_cmd_reboot_bootloader(struct fastboot_dev *fastboot)
{
    machine_restart(NULL);
    return 0;
}

static void explain_cmd_reboot_bootloader(struct fastboot_dev *fastboot)
{
	if (handle_cmd_reboot_bootloader(fastboot))
		strcpy(fastboot->ret_buf, "FAILED");
	else
		strcpy(fastboot->ret_buf, "OKAY");

	return_buf(fastboot, return_complete);
}

static int handle_cmd_reboot(struct fastboot_dev *fastboot)
{
	machine_restart(NULL);
	return 0;
}

static void explain_cmd_reboot(struct fastboot_dev *fastboot)
{
	if (handle_cmd_reboot(fastboot))
		strcpy(fastboot->ret_buf, "FAILED");
	else
		strcpy(fastboot->ret_buf, "OKAY");

	return_buf(fastboot, return_complete);
}

static void return_continue_complete(struct usb_ep *ep,
		struct usb_request *req)
{
	struct fastboot_dev *fastboot = req->context;
	fastboot->explain_cmd_status = 1;
	fastboot->leave = 1;
}

static int handle_cmd_continue(struct fastboot_dev *fastboot)
{
	return 0;
}

static void explain_cmd_continue(struct fastboot_dev *fastboot)
{
	if (handle_cmd_continue(fastboot))
		strcpy(fastboot->ret_buf, "FAILED");
	else
		strcpy(fastboot->ret_buf, "OKAY");

	return_buf(fastboot, return_continue_complete);
}

static int handle_cmd_boot(struct fastboot_dev *fastboot)
{
	return -1;
}

static void explain_cmd_boot(struct fastboot_dev *fastboot)
{
	if (handle_cmd_boot(fastboot))
		strcpy(fastboot->ret_buf, "FAILED");
	else
		strcpy(fastboot->ret_buf, "OKAY");

	return_buf(fastboot, return_complete);
}

static int handle_cmd_oem(struct fastboot_dev *fastboot)
{
	printk("please add the oem cmd explain roution\n");
	return -1;
}

static void explain_cmd_oem(struct fastboot_dev *fastboot)
{
	if (handle_cmd_oem(fastboot))
		strcpy(fastboot->ret_buf, "FAILED");
	else
		strcpy(fastboot->ret_buf, "OKAY");

	return_buf(fastboot, return_complete);
}

static void explain_cmd_else(struct fastboot_dev *fastboot)
{
	strcpy(fastboot->ret_buf, "FAILED: The command is not recongized\n");
	return_buf(fastboot, return_complete);
}

static void free_request(struct usb_ep * ep, struct usb_request *req) {
    if (req) {
        memset(req->buf, 0, req->length);
        kfree(req->buf);
        req->buf = NULL;
        usb_ep_free_request(ep, req);
        req = NULL;
    }
}

static void handle_fastboot_cmd_complete_work(struct work_struct *work)
{
    struct fastboot_dev *fastboot = container_of(work, struct fastboot_dev, work);
	struct usb_request * req = fastboot->cmd_req;

	printk("The received command is:<%s>\n",(char *)req->buf);

	if (!strlen(req->buf)) {
	    strcpy(fastboot->ret_buf, "FAILED: Command lost, try again\n");
	    return_buf(fastboot, return_complete);
	    return;
	}

	if (!strncmp(req->buf, "download:", 9)) {
		explain_cmd_download(fastboot);
		return;
	}

	if (!strncmp(req->buf, "flash:", 6)) {
		explain_cmd_flash(fastboot, req->buf);
		return;
	}

    if (!strncmp(req->buf, "flash_division:", 15)) {
        explain_cmd_flash(fastboot, req->buf + 9);
        return;
    }

    fastboot->last_flash_part = -1;
    fastboot->flashed_size = 0;

	if (!strncmp(req->buf, "getvar:all", 10)) {
		explain_cmd_getvar_all(fastboot);
		return;
	}

	if (!strncmp(req->buf, "getvar:", 7)) {
		explain_cmd_getvar(fastboot);
		return;
	}

	if (!strncmp(req->buf, "erase:", 6)) {
		explain_cmd_erase(fastboot, req->buf);
		return;
	}

	if (!strcmp(req->buf, "reboot-bootloader")) {
		explain_cmd_reboot_bootloader(fastboot);
		return;
	}

	if ((strcmp(req->buf, "reboot-bootloader")) && !strncmp(req->buf, "reboot",6)) {
		explain_cmd_reboot(fastboot);
		return;
	}

	if (!strcmp(req->buf, "continue")) {
		explain_cmd_continue(fastboot);
		return;
	}

	if (!strcmp(req->buf, "boot")) {
		explain_cmd_boot(fastboot);
		return;
	}

	if (!strncmp(req->buf, "oem:", 3)) {
		explain_cmd_oem(fastboot);
		return;
	}

	if (!strncmp(req->buf, "devices", 6)) {
	    strcpy(fastboot->ret_buf, "hello");
	    return;
	}
	explain_cmd_else(fastboot);

	return;
}

static void handle_fastboot_cmd_complete(struct usb_ep *ep,
        struct usb_request *req)
{
    struct fastboot_dev *fastboot = req->context;

    if (req->status == 0) {
        fastboot->leave = 0;
        schedule_work(&fastboot->work);
    }
}

static int handle_fastboot_init(struct fastboot_dev *fastboot)
{
	fastboot->ret_req = usb_ep_alloc_request(fastboot_common->bulk_in, GFP_KERNEL);
	if (!fastboot->ret_req) {
		printk("%s: Error, usb alloc request for return\n", __func__);
		return -ENOMEM;
	}

	fastboot->ret_buf = kzalloc(RET_LENGTH * sizeof(char), GFP_KERNEL);
	if (!fastboot->ret_buf) {
		printk("%s: Error in malloc RAM for ret_buf\n", __func__);
		return -ENOMEM;
	}

    fastboot_common->fastboot = fastboot;
    fastboot->leave = 1;
    fastboot->last_flash_part = -1;
	fastboot_common->enum_done = 0;
	fastboot->explain_cmd_status = 0;
	return 0;
}

static void handle_fastboot_complete(struct fastboot_dev *fastboot)
{
	kfree(fastboot->ret_buf);
	fastboot->ret_buf = NULL;
	usb_ep_free_request(fastboot_common->bulk_in, fastboot->ret_req);
}

/* add somes functions(Start, Power off, Restart bootloader,
 * Recovery mode, they are selected by power key and changed by
 * volume key, and display some images */
static void function_add(struct fastboot_dev *fastboot)
{
	return;
}

void handle_fastboot_cmd(void)
{
	struct fastboot_dev *fastboot;
	int	ret, init = 0;

	fastboot = kzalloc(sizeof(struct fastboot_dev), GFP_KERNEL);
	if (!fastboot) {
		printk("%s:Error malloc for fastboot_dev\n", __func__);
		return;
	}

	ret = handle_fastboot_init(fastboot);
	if (ret != 0){
		printk("%s:Error init,%d\n", __func__, ret);
		goto over1;
	};

	INIT_WORK(&fastboot->work, handle_fastboot_cmd_complete_work);

	while (1) {
		function_add(fastboot);

        if ((fastboot->leave && init) || !fastboot->connecting)
            msleep(WAIT_CMD_BEFORE_LEAVE);

        if ((fastboot->leave && init) || !fastboot->connecting) {
            cancel_work_sync(&fastboot->work);
            usb_ep_dequeue(fastboot_common->bulk_out, fastboot->cmd_req);
            break;
        }

		if (fastboot_common->enum_done || fastboot->explain_cmd_status) {
			int ret_value;
			fastboot_common->enum_done = 0;
			fastboot->explain_cmd_status = 0;
			free_request(fastboot_common->bulk_out, fastboot->cmd_req);
			fastboot->cmd_req = usb_ep_alloc_request(fastboot_common->bulk_out, GFP_KERNEL);
			if (!fastboot->cmd_req) {
				printk("%s: Error, usb alloc request for command\n", __func__);
				goto over1;
			}

			fastboot->cmd_req->length = 64;
			fastboot->cmd_req->buf = kzalloc(fastboot->cmd_req->length, GFP_KERNEL);
			if (!fastboot->cmd_req->buf) {
				printk("%s:Error malloc for command buffer\n", __func__);
				goto over2;
			}

			fastboot->cmd_req->complete = handle_fastboot_cmd_complete;
			fastboot->cmd_req->status = 0;
			fastboot->cmd_req->context = fastboot;

			ret_value = usb_ep_queue(fastboot_common->bulk_out,
					fastboot->cmd_req, GFP_KERNEL);
			if (ret_value != 0) {
				printk("%s: Warning, usb_ep_queue return %d\n",
						__func__, ret_value);
				goto over2;
			}
			init = 1;
		}
	}

over2:
    free_request(fastboot_common->bulk_out, fastboot->cmd_req);
	handle_fastboot_complete(fastboot);
over1:
	kfree(fastboot);
	fastboot = NULL;
}

