/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>

#include <linux/usb/android_composite.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#include "gadget_chips.h"

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
/* FUJITSU:2011-04-26 USB_GB_LISMO start */
extern int serial_notification_init(void);
extern void serial_notification_deinit(void);
/* FUJITSU:2011-04-26 USB_GB_LISMO end */

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by platform data */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001

struct android_dev {
	struct usb_composite_dev *cdev;
	struct usb_configuration *config;
	int num_products;
	struct android_usb_product *products;
	int num_functions;
	char **functions;

	int product_id;
	int version;
/* FUJITSU:2011-07-01 USB_GB +start	*/
	int prev_product_id;
/* FUJITSU:2011-07-01 USB_GB +end	*/
};

static struct android_dev *_android_dev;

/* FUJITSU:2011-04-19 USB_GB +start	*/
#if 0
	#define MAX_STR_LEN		16
#else
	#define MAX_STR_LEN		32
#endif
/* FUJITSU:2011-04-19 USB_GB +end	*/

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

char serial_number[MAX_STR_LEN];
/* String Table */
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = "Android",
	[STRING_PRODUCT_IDX].s = "Android",
	[STRING_SERIAL_IDX].s = "0123456789ABCDEF",
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
	.bcdOTG               = __constant_cpu_to_le16(0x0200),
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

static struct list_head _functions = LIST_HEAD_INIT(_functions);
static int _registered_function_count = 0;

static void android_set_default_product(int product_id);

#if 0   /* FUJITSU:2011-07-01 USB_GB +start	*/
/* FUJITSU:2011-04-21 USB start */
extern int store_state;
/* FUJITSU:2011-04-21 USB end */
#endif  /* FUJITSU:2011-07-01 USB_GB +end	*/

/* FUJITSU:2011-04-26 USB start */
#define FJ_PRODUCT_ID	0x12A8

static char global_serial_number[MAX_STR_LEN] = {0};

int android_get_function_enable(const char *function);
static int android_set_serialnumber(const char *kmessage);
static int android_serialnumber_updata(void);
static void android_set_function_mask(struct android_usb_product *up);
extern unsigned long debugusb_get_pid_from_nv(void);
void android_debugusb_set_port(int pid)
{
	struct android_dev *dev = _android_dev;

pr_info("[gadget/android.c] %s\n", __func__);
	android_set_default_product(pid);

	device_desc.idProduct = __constant_cpu_to_le16(pid);
	if (dev->cdev) {
		dev->cdev->desc.idProduct = device_desc.idProduct;
	}

	android_set_serialnumber(global_serial_number);

pr_info("[gadget/android.c] %s device_desc.idProduct=0x%x\n", __func__,device_desc.idProduct);
	usb_composite_force_reset(dev->cdev);
}

int android_get_current_pid(void)
{
	return (int)device_desc.idProduct;
}

static int android_set_serialnumber(const char *kmessage)
{
	int len = strlen(kmessage);

	if (len >= MAX_STR_LEN) {
		printk(KERN_ERR "serial number string too long\n");
		return -ENOSPC;
	}
	memset(serial_number,0,MAX_STR_LEN);
	strlcpy(serial_number, kmessage, MAX_STR_LEN);
	/* Chop out \n char as a result of echo */
	if (serial_number[len - 1] == '\n')
		serial_number[len - 1] = '\0';

	return 0;
}

static int android_serialnumber_updata(void)
{
	char temp_serial_number[MAX_STR_LEN] = {0};
	int adb_enable = 1;
	int len = 0;

	if (device_desc.idProduct == FJ_PRODUCT_ID) {
		strlcpy(temp_serial_number, global_serial_number, MAX_STR_LEN);
		adb_enable = android_get_function_enable("adb");
		if(adb_enable == 0) {
			/* adb is disable	*/
			len = strlen(temp_serial_number);
			if (len <= (MAX_STR_LEN - 2)) {
			    /* Appending '0' at the end */
				temp_serial_number[len] = '0';
				temp_serial_number[len + 1] = '\0';
			}
		}
		android_set_serialnumber(temp_serial_number);
	}
	else {
		android_set_serialnumber(global_serial_number);
	}

pr_info("[gadget/android.c] %s serialnumber= %s\n", __func__,serial_number);
	return 0;
}

/* FUJITSU:2011-04-26 USB end */

void android_usb_set_connected(int connected)
{
	if (_android_dev && _android_dev->cdev && _android_dev->cdev->gadget) {
		if (connected)
			usb_gadget_connect(_android_dev->cdev->gadget);
		else
			usb_gadget_disconnect(_android_dev->cdev->gadget);
	}
}

static struct android_usb_function *get_function(const char *name)
{
	struct android_usb_function	*f;
	list_for_each_entry(f, &_functions, list) {
		if (!strcmp(name, f->name))
			return f;
	}
	return 0;
}

static void bind_functions(struct android_dev *dev)
{
	struct android_usb_function	*f;
	char **functions = dev->functions;
	int i;

	for (i = 0; i < dev->num_functions; i++) {
		char *name = *functions++;
		f = get_function(name);
		if (f)
			f->bind_config(dev->config);
		else
			pr_err("%s: function %s not found\n", __func__, name);
	}

	/*
	 * set_alt(), or next config->bind(), sets up
	 * ep->driver_data as needed.
	 */
	usb_ep_autoconfig_reset(dev->cdev->gadget);
}

static int __ref android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	pr_debug("android_bind_config\n");
	dev->config = c;

	/* bind our functions if they have all registered */
	if (_registered_function_count == dev->num_functions)
		bind_functions(dev);

	return 0;
}

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl);

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.bind		= android_bind_config,
	.setup		= android_setup_config,
	.bConfigurationValue = 1,
	.bMaxPower	= 0xFA, /* 500ma */
};

/* FUJITSU:2011-04-19 USB_GB +start	*/
int android_get_function_enable(const char *function)
{
	struct usb_function *func;
	int	enable_flag = 0;

/* pr_info("[gadget/android.c] %s %s is enabled?\n", __func__,function);*/

	list_for_each_entry(func, &android_config_driver.functions, list) {

/* pr_info("[gadget/android.c] %s func->name=%s\n", __func__,func->name);	*/
		if (!strcmp(func->name, function)) {
			if (func->disabled == 0) {
				enable_flag = 1;
				pr_info("[gadget/android.c] %s %s is enabled\n", __func__,function);
			}
			else {
				pr_info("[gadget/android.c] %s %s is disabled\n", __func__,function);
			}
			break;
		}
	}

	return enable_flag;
}
/* FUJITSU:2011-04-19 USB_GB +end	*/

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl)
{
	int i;
	int ret = -EOPNOTSUPP;

	for (i = 0; i < android_config_driver.next_interface_id; i++) {
		if (android_config_driver.interface[i]->setup) {
			ret = android_config_driver.interface[i]->setup(
				android_config_driver.interface[i], ctrl);
			if (ret >= 0)
				return ret;
		}
	}
	return ret;
}

static int product_has_function(struct android_usb_product *p,
		struct usb_function *f)
{
	char **functions = p->functions;
	int count = p->num_functions;
	const char *name = f->name;
	int i;

	for (i = 0; i < count; i++) {
		if (!strcmp(name, *functions++))
			return 1;
	}
	return 0;
}

static int product_matches_functions(struct android_usb_product *p)
{
	struct usb_function		*f;
	list_for_each_entry(f, &android_config_driver.functions, list) {
		if (product_has_function(p, f) == !!f->disabled)
			return 0;
	}
	return 1;
}

static int get_product_id(struct android_dev *dev)
{
	struct android_usb_product *p = dev->products;
	int count = dev->num_products;
	int i;

	if (p) {
		for (i = 0; i < count; i++, p++) {
			if (product_matches_functions(p))
				return p->product_id;
		}
	}
	/* use default product ID */
	return dev->product_id;
}

static int __devinit android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, product_id, ret;

	pr_debug("android_bind\n");

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	if (gadget_is_otg(cdev->gadget))
		android_config_driver.descriptors = otg_desc;

	if (!usb_gadget_set_selfpowered(gadget))
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_SELFPOWER;

	if (gadget->ops->wakeup)
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config_driver);
	if (ret) {
		pr_err("%s: usb_add_config failed\n", __func__);
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;
	product_id = get_product_id(dev);
	device_desc.idProduct = __constant_cpu_to_le16(product_id);
	cdev->desc.idProduct = device_desc.idProduct;

	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
	.enable_function = android_enable_function,
};

void android_register_function(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;

	pr_debug("%s: %s\n", __func__, f->name);
	list_add_tail(&f->list, &_functions);
	_registered_function_count++;

	/* bind our functions if they have all registered
	 * and the main driver has bound.
	 */
	if (dev->config && _registered_function_count == dev->num_functions) {
		bind_functions(dev);
		android_set_default_product(dev->product_id);
/* FUJITSU:2011-04-19 USB_GB +start	*/
pr_info("[gadget/android.c] %s idProduct=0x%x\n", __func__,dev->product_id);
		android_serialnumber_updata();
/* FUJITSU:2011-04-19 USB_GB +end	*/
	}
}

/**
 * android_set_function_mask() - enables functions based on selected pid.
 * @up: selected product id pointer
 *
 * This function enables functions related with selected product id.
 */
static void android_set_function_mask(struct android_usb_product *up)
{
	int index, found = 0;
	struct usb_function *func;

	list_for_each_entry(func, &android_config_driver.functions, list) {
		/* adb function enable/disable handled separetely */
		if (!strcmp(func->name, "adb"))
			continue;

		for (index = 0; index < up->num_functions; index++) {
			if (!strcmp(up->functions[index], func->name)) {
				found = 1;
				break;
			}
		}

		if (found) { /* func is part of product. */
			/* if func is disabled, enable the same. */
			if (func->disabled)
				usb_function_set_enabled(func, 1);
			found = 0;
		} else { /* func is not part if product. */
			/* if func is enabled, disable the same. */
			if (!func->disabled)
				usb_function_set_enabled(func, 0);
		}
	}
}

/**
 * android_set_defaut_product() - selects default product id and enables
 * required functions
 * @product_id: default product id
 *
 * This function selects default product id using pdata information and
 * enables functions for same.
*/
static void android_set_default_product(int pid)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_product *up = dev->products;
	int index;

#if 0   /* FUJITSU:2011-07-01 USB_GB +start	*/
/* FUJITSU:2011-04-21 USB start */
	if (pid == 0xffff) {
		if (!store_state)
			return;
	}
/* FUJITSU:2011-04-21 USB end */
#endif  /* FUJITSU:2011-07-01 USB_GB +end	*/

	for (index = 0; index < dev->num_products; index++, up++) {
		if (pid == up->product_id)
			break;
	}
	android_set_function_mask(up);
}

/**
 * android_config_functions() - selects product id based on function need
 * to be enabled / disabled.
 * @f: usb function
 * @enable : function needs to be enable or disable
 *
 * This function selects product id having required function at first index.
 * TODO : Search of function in product id can be extended for all index.
 * RNDIS function enable/disable uses this.
*/
/* #ifdef CONFIG_USB_ANDROID_RNDIS 	*/      /* FUJITSU:2011-07-01 USB_GB -	*/
static void android_config_functions(struct usb_function *f, int enable)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_product *up = dev->products;
	int index;
	char **functions;

	/* Searches for product id having function at first index */
	if (enable) {
		for (index = 0; index < dev->num_products; index++, up++) {
			functions = up->functions;
			if (!strcmp(*functions, f->name))
				break;
		}
		android_set_function_mask(up);
	} else
		android_set_default_product(dev->product_id);
}
/* #endif							*/    /* FUJITSU:2011-07-01 USB_GB -	*/

void android_enable_function(struct usb_function *f, int enable)
{
	struct android_dev *dev = _android_dev;
	int disable = !enable;
	int product_id;

pr_info("[gadget/android.c] %s() func: %s enable: %d \n", __func__, f->name, enable);
	if (!!f->disabled != disable) {
		usb_function_set_enabled(f, !disable);

#ifdef CONFIG_USB_ANDROID_RNDIS
		if (!strcmp(f->name, "rndis")) {

			/* We need to specify the COMM class in the device descriptor
			 * if we are using RNDIS.
			 */
			if (enable) {
#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
				dev->cdev->desc.bDeviceClass = USB_CLASS_MISC;
				dev->cdev->desc.bDeviceSubClass      = 0x02;
				dev->cdev->desc.bDeviceProtocol      = 0x01;
#else
				dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
#endif
/* FUJITSU:2011-07-01 USB_GB +start	*/
				dev->prev_product_id = android_get_current_pid();
/* FUJITSU:2011-07-01 USB_GB +end	*/
			} else {
				dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
				dev->cdev->desc.bDeviceSubClass      = 0;
				dev->cdev->desc.bDeviceProtocol      = 0;
/* FUJITSU:2011-07-01 USB_GB +start	*/
				dev->product_id = dev->prev_product_id;
/* FUJITSU:2011-07-01 USB_GB +end	*/
			}

			android_config_functions(f, enable);
		}
#endif
/* FUJITSU:2011-04-26 USB_GB_LISMO start */
		if (!strcmp(f->name, "lismo")) {
/* FUJITSU:2011-07-01 USB_GB +start	*/
			if (enable)
				dev->prev_product_id = android_get_current_pid();
			else
				dev->product_id = dev->prev_product_id;
/* FUJITSU:2011-07-01 USB_GB +end	*/
			android_config_functions(f, enable);
		}
/* FUJITSU:2011-04-26 USB_GB_LISMO end */

		product_id = get_product_id(dev);
		/* FUJITSU:2011-06-22 USB_GB +start	*/
pr_info("[gadget/android.c] %s OLD idProduct=0x%x -> NEW idProduct=0x%x\n", __func__,device_desc.idProduct,product_id);
		/* FUJITSU:2011-06-22 USB_GB +end	*/
		device_desc.idProduct = __constant_cpu_to_le16(product_id);
		if (dev->cdev)
			dev->cdev->desc.idProduct = device_desc.idProduct;

		/* FUJITSU:2011-04-19 USB_GB +start	*/
pr_info("[gadget/android.c] %s idProduct=0x%x\n", __func__,device_desc.idProduct);
		android_serialnumber_updata();
		/* FUJITSU:2011-04-19 USB_GB +end	*/

		usb_composite_force_reset(dev->cdev);
	}
}

#ifdef CONFIG_DEBUG_FS
static int android_debugfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t android_debugfs_serialno_write(struct file *file, const char
				__user *buf,	size_t count, loff_t *ppos)
{
	char str_buf[MAX_STR_LEN];

	if (count > MAX_STR_LEN)
		return -EFAULT;

	if (copy_from_user(str_buf, buf, count))
		return -EFAULT;

	memcpy(serial_number, str_buf, count);

	if (serial_number[count - 1] == '\n')
		serial_number[count - 1] = '\0';

	strings_dev[STRING_SERIAL_IDX].s = serial_number;

	return count;
}
const struct file_operations android_fops = {
	.open	= android_debugfs_open,
	.write	= android_debugfs_serialno_write,
};

struct dentry *android_debug_root;
struct dentry *android_debug_serialno;

static int android_debugfs_init(struct android_dev *dev)
{
	android_debug_root = debugfs_create_dir("android", NULL);
	if (!android_debug_root)
		return -ENOENT;

	android_debug_serialno = debugfs_create_file("serial_number", 0222,
						android_debug_root, dev,
						&android_fops);
	if (!android_debug_serialno) {
		debugfs_remove(android_debug_root);
		android_debug_root = NULL;
		return -ENOENT;
	}
	return 0;
}

static void android_debugfs_cleanup(void)
{
       debugfs_remove(android_debug_serialno);
       debugfs_remove(android_debug_root);
}
#endif
static int __init android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;
	int result;
	unsigned long pid = 0;  /* FUJITSU:2011-04-19 USB_GB + */
	int ret;                /* FUJITSU:2011-04-26 USB_GB + */

	dev_dbg(&pdev->dev, "%s: pdata: %p\n", __func__, pdata);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	result = pm_runtime_get(&pdev->dev);
	if (result < 0) {
		dev_err(&pdev->dev,
			"Runtime PM: Unable to wake up the device, rc = %d\n",
			result);
		return result;
	}

	if (pdata) {
		dev->products = pdata->products;
		dev->num_products = pdata->num_products;
		dev->functions = pdata->functions;
		dev->num_functions = pdata->num_functions;
		if (pdata->vendor_id)
			device_desc.idVendor =
				__constant_cpu_to_le16(pdata->vendor_id);
		if (pdata->product_id) {
/* FUJITSU:2011-07-01 USB_GB +start	*/
			dev->prev_product_id = pdata->product_id;
/* FUJITSU:2011-07-01 USB_GB +end	*/

/* FUJITSU:2011-05-11 USB start */
			pid = debugusb_get_pid_from_nv();
			if (pid) {
				dev->product_id = pid;
				device_desc.idProduct =
					__constant_cpu_to_le16(pid);
			}else{
				dev->product_id = pdata->product_id;
				device_desc.idProduct =
					__constant_cpu_to_le16(pdata->product_id);
			}
			pr_info("[gadget/android.c] %s product_id = 0x%lx\n", __func__,(long unsigned int)dev->product_id);
/* FUJITSU:2011-05-11 USB end */
		}
		if (pdata->version)
			dev->version = pdata->version;

		if (pdata->product_name)
			strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
		if (pdata->manufacturer_name)
			strings_dev[STRING_MANUFACTURER_IDX].s =
					pdata->manufacturer_name;
		if (pdata->serial_number) {
			/* FUJITSU:2011-04-19 USB_GB +start	*/
			#if 0
				strings_dev[STRING_SERIAL_IDX].s = pdata->serial_number;
			#else
				android_set_serialnumber(pdata->serial_number);
				strings_dev[STRING_SERIAL_IDX].s = serial_number;
				strlcpy(global_serial_number, serial_number, MAX_STR_LEN);
				pr_info("[gadget/android.c] %s serialnumber= %s\n", __func__,serial_number);
			#endif
			/* FUJITSU:2011-04-19 USB_GB +end	*/

		}
	}
/* FUJITSU:2011-04-26 USB_GB_LISMO start */
	ret = serial_notification_init();
/* FUJITSU:2011-04-26 USB_GB_LISMO end */
#ifdef CONFIG_DEBUG_FS
	result = android_debugfs_init(dev);
	if (result)
		pr_debug("%s: android_debugfs_init failed\n", __func__);
#endif
	return usb_composite_register(&android_usb_driver);
}

static int andr_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int andr_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static struct dev_pm_ops andr_dev_pm_ops = {
	.runtime_suspend = andr_runtime_suspend,
	.runtime_resume = andr_runtime_resume,
};

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb", .pm = &andr_dev_pm_ops},
};

static int __init init(void)
{
	struct android_dev *dev;

	pr_debug("android init\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* set default values, which should be overridden by platform data */
	dev->product_id = PRODUCT_ID;
	_android_dev = dev;

	return platform_driver_probe(&android_platform_driver, android_probe);
}
module_init(init);

static void __exit cleanup(void)
{
#ifdef CONFIG_DEBUG_FS
	android_debugfs_cleanup();
#endif
/* FUJITSU:2011-04-26 USB_GB_LISMO start */
	serial_notification_deinit();
/* FUJITSU:2011-04-26 USB_GB_LISMO end */
	usb_composite_unregister(&android_usb_driver);
	platform_driver_unregister(&android_platform_driver);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
