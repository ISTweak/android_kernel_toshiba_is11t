/*
 * Copyright(C) 2011 FUJITSU LIMITED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/usb/android_composite.h>

#include "u_serial.h"
#include "gadget_chips.h"

/* FUJITSU:2011-04-26 USB_GB_LISMO start */
extern void serial_notification_set_state(int serial_mode);
int serial_online;
/* FUJITSU:2011-04-26 USB_GB_LISMO end */
/*
 * This function driver for lismo application packages a simple 
 * "generic serial" port with no real control mechanisms, just raw data 
 * transfer over two bulk endpoints.
 */

struct lismo_descs {
	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;
};

struct f_gser {
	struct gserial			port;
	u8				data_id;
	u8				port_num;
	struct lismo_descs		fs;
	struct lismo_descs		hs;
	u8				online;
};

static inline struct f_gser *func_to_gser(struct usb_function *f)
{
	return container_of(f, struct f_gser, port.func);
}

/*-------------------------------------------------------------------------*/

/* interface descriptor: Refer KDDI-USB spec */

static struct usb_cdc_header_desc header_desc = {
	.bLength            =	sizeof(struct usb_cdc_header_desc),
	.bDescriptorType    =	0x24,
	.bDescriptorSubType =	0x00,
	.bcdCDC             =	__constant_cpu_to_le16(0x0110),
};

/* Communications Class Interface Descriptor for KMMO port */
static struct usb_interface_descriptor kmmo_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */ 
	/* NOTE: InterfaceNumber must be 2 as per KDDI-USB */
	.bNumEndpoints =	2,
	.bInterfaceClass =	0x02,  /* CDC */
	.bInterfaceSubClass =	0x0A,  /* WMC1.1, MDLM subclass */
	.bInterfaceProtocol =	0x01,  /* KMMO port protocol */
};

/* MDLM functional Descriptor */
static struct usb_cdc_mdlm_desc mdlm_desc = {
	.bLength = sizeof(struct usb_cdc_mdlm_desc),
	.bDescriptorType = 0x24,
	.bDescriptorSubType = 0x12,
	.bcdVersion = __constant_cpu_to_le16(0x0100),
	.bGUID[0] = 0xC2,
	.bGUID[1] = 0x29,
	.bGUID[2] = 0x9F,
	.bGUID[3] = 0xCC,
	.bGUID[4] = 0xD4,
	.bGUID[5] = 0x89,
	.bGUID[6] = 0x40,
	.bGUID[7] = 0x66,
	.bGUID[8] = 0x89,
	.bGUID[9] = 0x2B,
	.bGUID[10] = 0x10,
	.bGUID[11] = 0xC3,
	.bGUID[12] = 0x41,
	.bGUID[13] = 0xDD,
	.bGUID[14] = 0x98,
	.bGUID[15] = 0xA9,
};

static struct usb_endpoint_descriptor lismo_fs_in_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor lismo_fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *lismo_fs_function[] = {
	(struct usb_descriptor_header *) &kmmo_interface_desc,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &mdlm_desc,
	(struct usb_descriptor_header *) &lismo_fs_in_desc,
	(struct usb_descriptor_header *) &lismo_fs_out_desc,
	NULL,
};

static struct usb_endpoint_descriptor lismo_hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor lismo_hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_descriptor_header *lismo_hs_function[] = {
	(struct usb_descriptor_header *) &kmmo_interface_desc,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &mdlm_desc,
	(struct usb_descriptor_header *) &lismo_hs_in_desc,
	(struct usb_descriptor_header *) &lismo_hs_out_desc,
	NULL,
};

/* string descriptors: */

static struct usb_string lismo_string_defs[] = {
	[0].s = "Lismo Serial Gadget Port",
	{  } /* end of list */
};

static struct usb_gadget_strings lismo_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		lismo_string_defs,
};

static struct usb_gadget_strings *lismo_strings[] = {
	&lismo_string_table,
	NULL,
};

static int lismo_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_gser		 *gser = func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	/* we know alt == 0, so this is an activation or a reset */

	if (gser->port.in->driver_data) {
		DBG(cdev, "reset lismo ttyGS%d\n", gser->port_num);
		printk("KDDI lismo_set_alt reset lismo data ttyGS%d\n", gser->port_num);
		gserial_disconnect(&gser->port);
	} else {
		DBG(cdev, "activate lismo data ttyGS%d\n", gser->port_num);
		printk("KDDI lismo_set_alt activate lismo ttyGS%d\n", gser->port_num);
	}
	gser->port.in_desc = ep_choose(cdev->gadget,
			gser->hs.in, gser->fs.in);
	gser->port.out_desc = ep_choose(cdev->gadget,
			gser->hs.out, gser->fs.out);
	gserial_connect(&gser->port, gser->port_num);
	gser->online = 1;
	serial_online=1;
/* FUJITSU:2011-04-26 USB_GB_LISMO start */
	serial_notification_set_state(serial_online);
/* FUJITSU:2011-04-26 USB_GB_LISMO end */

	return 0;
}

static void lismo_disable(struct usb_function *f)
{
	struct f_gser	         *gser = func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	serial_online=0;
/* FUJITSU:2011-04-26 USB_GB_LISMO start */
	serial_notification_set_state(serial_online);
/* FUJITSU:2011-04-26 USB_GB_LISMO end */

	DBG(cdev, "lismo ttyGS%d deactivated\n", gser->port_num);
	printk("KDDI lismo_disable lismo ttyGS%d deactivated\n", gser->port_num);
	gserial_disconnect(&gser->port);
	gser->online = 0;
}

/*-------------------------------------------------------------------------*/

/* serial function driver setup/binding */

static int
lismo_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_gser            *gser = func_to_gser(f);
	int			 status;
	struct usb_ep		 *ep;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	gser->data_id = status;
	kmmo_interface_desc.bInterfaceNumber = status;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &lismo_fs_in_desc);
	if (!ep)
		goto fail;
	gser->port.in = ep;
	ep->driver_data = cdev;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &lismo_fs_out_desc);
	if (!ep)
		goto fail;
	gser->port.out = ep;
	ep->driver_data = cdev;	/* claim */

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(lismo_fs_function);

	gser->fs.in = usb_find_endpoint(lismo_fs_function,
			f->descriptors, &lismo_fs_in_desc);
	gser->fs.out = usb_find_endpoint(lismo_fs_function,
			f->descriptors, &lismo_fs_out_desc);

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		lismo_hs_in_desc.bEndpointAddress =
				lismo_fs_in_desc.bEndpointAddress;
		lismo_hs_out_desc.bEndpointAddress =
				lismo_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(lismo_hs_function);

		gser->hs.in = usb_find_endpoint(lismo_hs_function,
				f->hs_descriptors, &lismo_hs_in_desc);
		gser->hs.out = usb_find_endpoint(lismo_hs_function,
				f->hs_descriptors, &lismo_hs_out_desc);
	}

	DBG(cdev, "lismo ttyGS%d: %s speed IN/%s OUT/%s\n",
			gser->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			gser->port.in->name, gser->port.out->name);
	printk( "KDDI lismo_bind lismo ttyGS%d: %s speed IN/%s OUT/%s\n",
                        gser->port_num,
                        gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
                        gser->port.in->name, gser->port.out->name);
	serial_online=1;

	return 0;

fail:
	/* we might as well release our claims on endpoints */
	if (gser->port.out)
		gser->port.out->driver_data = NULL;
	if (gser->port.in)
		gser->port.in->driver_data = NULL;

	//ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);
	printk("%s: can't bind, err %d\n", f->name, status);

	return status;
}

static void
lismo_unbind(struct usb_configuration *c, struct usb_function *f)
{
	/* WIP: Temp fix for freeze - start*/
	serial_online=0;
/* FUJITSU:2011-04-26 USB_GB_LISMO start */
	serial_notification_set_state(serial_online);
/* FUJITSU:2011-04-26 USB_GB_LISMO end */

	/* WIP: Temp fix for freeze - Finish*/
	
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
	kfree(func_to_gser(f));
}

/**
 * lismo_bind_config - add a generic serial function to a configuration
 * @c: the configuration to support the serial instance
 * @port_num: /dev/ttyGS* port this interface will use
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 *
 * Caller must have called @gserial_setup() with enough ports to
 * handle all the ones it binds.  Caller is also responsible
 * for calling @gserial_cleanup() before module unload.
 */
int lismo_bind_config(struct usb_configuration *c, u8 port_num)
{
	struct f_gser *gser;
	int		status;

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	/* maybe allocate device-global string ID */
	if (lismo_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		lismo_string_defs[0].id = status;
	}

	/* allocate and initialize one new instance */
	gser = kzalloc(sizeof *gser, GFP_KERNEL);
	if (!gser)
		return -ENOMEM;

	gser->port_num = port_num;

	gser->port.func.name = "lismo";
	gser->port.func.strings = lismo_strings;
	gser->port.func.bind = lismo_bind;
	gser->port.func.unbind = lismo_unbind;
	gser->port.func.set_alt = lismo_set_alt;
	gser->port.func.disable = lismo_disable;
	status = usb_add_function(c, &gser->port.func);
	printk( "KDDI lismo_bind_config lismo ttyGS%d\n",gser->port_num);
	if (status)
		kfree(gser);
	return status;
}
