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
/*
 * notification LED driver for GPIOs
 *
 * Base on leds-gpio.c, ledtrig-timer.c
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/module.h>
#include <asm/gpio.h>
#include "leds.h"
#include "../../arch/arm/mach-msm/smd_private.h"

#define CONFIG_NOTIFICATIN_GPIOS_LED_DEBUG

enum {
	DBG_FUNC_ENTER = 1 << 0,
	DBG_FUNC_LEAVE = 1 << 1,
	DBG_LED_SET = 1 << 2,
	DBG_SMEM = 1 << 3,
	DBG_ARGB = 1 << 4,
	DBG_DELAY_SET = 1 << 5,
	DBG_RESUME = 1 << 6,
	DBG_HAL_UT = 1 << 7,
	DBG_ANY = 1 << 31,
};
static u32 led_notification_debug_mask
//	= DBG_FUNC_ENTER | DBG_FUNC_LEAVE | DBG_LED_SET | DBG_SMEM
//	| DBG_ARGB | DBG_DELAY_SET | DBG_HAL_UT | DBG_RESUME | DBG_ANY;
	= DBG_SMEM | DBG_HAL_UT | DBG_RESUME;
module_param_named(debug_mask, led_notification_debug_mask, uint, S_IWUSR | S_IRUGO);

#if defined(CONFIG_NOTIFICATIN_GPIOS_LED_DEBUG)
#define	THIS_FILE	"led-notification"
#define FUNCENTER(fmt, args...) \
    do { if (led_notification_debug_mask & DBG_FUNC_ENTER) { \
        printk(KERN_INFO THIS_FILE "@%d %s(" fmt ")+\n", \
            __LINE__, __FUNCTION__, ## args); \
    } } while (0)
#define FUNCLEAVE(fmt, args...) \
    do { if (led_notification_debug_mask & DBG_FUNC_LEAVE) { \
        printk(KERN_INFO THIS_FILE "@%d %s()- " fmt "\n", \
            __LINE__, __FUNCTION__, ## args); \
    } } while (0)
#define LED_SET_DBG(dev, fmt, args...) \
    do { if (led_notification_debug_mask & DBG_LED_SET) { \
    	dev_info(dev, fmt, ## args); \
    } } while (0)
#define SMEM_DBG(dev, fmt, args...) \
    do { if (led_notification_debug_mask & DBG_SMEM) { \
    	dev_info(dev, fmt, ## args); \
    } } while (0)
#define ARGB_DBG(fmt, args...) \
    do { if (led_notification_debug_mask & DBG_ARGB) { \
        printk(KERN_INFO THIS_FILE "@%d %s() " fmt "\n", \
            __LINE__, __FUNCTION__, ## args); \
    } } while (0)
#define SET_DELAY_DBG(fmt, args...) \
    do { if (led_notification_debug_mask & DBG_DELAY_SET) { \
        printk(KERN_INFO THIS_FILE "@%d %s() " fmt "\n", \
            __LINE__, __FUNCTION__, ## args); \
    } } while (0)
#define RESUME_DBG(dev, fmt, args...) \
    do { if (led_notification_debug_mask & DBG_RESUME) { \
    	dev_info(dev, fmt, ## args); \
    } } while (0)
#define HAL_UT(dev, fmt, args...) \
    do { if (led_notification_debug_mask & DBG_HAL_UT) { \
    	dev_info(dev, fmt, ## args); \
    } } while (0)
#define ANY_DBG(fmt, args...) \
    do { if (led_notification_debug_mask & DBG_ANY) { \
        printk(KERN_INFO THIS_FILE "@%d %s() " fmt "\n", \
            __LINE__, __FUNCTION__, ## args); \
    } } while (0)
#else
#define FUNCENTER(fmt, args...) do {} while (0)
#define FUNCLEAVE(fmt, args...) do {} while (0)
#define LED_SET_DBG(dev, fmt, args...) do {} while (0)
#define SMEM_DBG(dev, fmt, args...) do {} while (0)
#define ARGB_DBG(fmt, args...) do {} while (0)
#define SET_DELAY_DBG(fmt, args...) do {} while (0)
#define RESUME_DBG(dev, fmt, args...) do {} while (0)
#define HAL_UT(dev, fmt, args...) do {} while (0)
#define ANY_DBG(fmt, args...) do {} while (0)
#endif

enum led_res_color {
	RES_LED_COLOR_NULL,
	RES_LED_COLOR_RED,
	RES_LED_COLOR_GREEN,
	RES_LED_COLOR_BLUE,
	RES_LED_COLOR_APPLICATION_COLOR,
	RES_LED_COLOR_MAX
};

enum led_res_charge {
	RES_CHARGE_STS_NULL,
	RES_CHARGE_STS_CHARGING,
	RES_CHARGE_STS_ERROR,
	RES_CHARGE_STS_MAX
};

enum led_res_notify {
	RES_NOTIFY_STS_NULL,
	RES_NOTIFY_STS_NORMAL,
	RES_NOTIFY_STS_3RDAPP,
	RES_NOTIFY_STS_MAX
};

static int iLedResColorTable[RES_NOTIFY_STS_MAX][RES_CHARGE_STS_MAX] = {
	{ RES_LED_COLOR_NULL             , RES_LED_COLOR_RED              , RES_LED_COLOR_RED },	/* NO_NOTIFY	: NULL, CHARGING, ERROR */
	{ RES_LED_COLOR_GREEN            , RES_LED_COLOR_GREEN            , RES_LED_COLOR_RED },	/* NORMAL_APP	: NULL, CHARGING, ERROR */
	{ RES_LED_COLOR_APPLICATION_COLOR, RES_LED_COLOR_APPLICATION_COLOR, RES_LED_COLOR_RED },	/* 3RD_APP	: NULL, CHARGING, ERROR */
};

#define RES_LED_NOT_3RD_MASK	(0x18)

struct smem_led_color_type {
	u8 led_alpha;
	u8 led_red;
	u8 led_green;
	u8 led_blue;
};

static volatile u8 * smem_led_ctrl_notify = NULL;
static volatile u8 * smem_led_ctrl_charge = NULL;
static volatile struct smem_led_color_type * smem_led_ctrl_color = NULL;

static u8 smem_led_notify;
static u8 smem_led_charge;
static struct smem_led_color_type smem_led_color;

struct notification_led_data {
	struct led_classdev cdev;
	u32 argb;
	unsigned int red_gpio;
	unsigned int green_gpio;
	unsigned int blue_gpio;
	unsigned long delay_on;
	unsigned long delay_off;
	struct timer_list timer;
	unsigned int brightness_on;
};

static ssize_t
notification_led_notify_store(
	struct device * dev, struct device_attribute * attr,
	const char * buf, size_t size)
{
	char * after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;
	u8 led_notify;
	u8 led_charge;
	ssize_t rc = -EINVAL;

	FUNCENTER("dev=%p,attr=%p,buf=%p,size=%u", dev, attr, buf, size);

	if (*after && isspace(*after))
		count++;
	if (count == size) {
		HAL_UT(dev, "store led_notify value %lu(%08lXh)", state, state);
		led_notify = (u8)(state & 0x000000ff);
		led_charge = (u8)((state & 0x00000700) >> 8);
		smem_led_notify = led_notify;
		smem_led_charge = led_charge;
		rc = count;
	}

	FUNCLEAVE("rc=%d", rc);

	return rc;
}

static ssize_t
notification_led_notify_show(
	struct device * dev, struct device_attribute * attr, char * buf)
{
	uint32_t state = smem_led_charge;

	state = (uint32_t)(state << 8);
	state = (uint32_t)(state + smem_led_notify);

	return sprintf(buf, "%u\n", state);
}

static ssize_t
notification_led_color_store(
	struct device * dev, struct device_attribute * attr,
	const char * buf, size_t size)
{
	char * after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;
	ssize_t rc = -EINVAL;

	FUNCENTER("dev=%p,attr=%p,buf=%p,size=%d", dev, attr, buf, size);

	if (*after && isspace(*after))
		count++;
	if (count == size) {
		HAL_UT(dev, "store led_color value %lu(%08lXh)", state, state);
		smem_led_color.led_blue = state & 255;
		state >>= 8;
		smem_led_color.led_green = state & 255;
		state >>= 8;
		smem_led_color.led_red = state & 255;
		state >>= 8;
		smem_led_color.led_alpha = state & 255;
		rc = count;
	}

	FUNCLEAVE("rc=%d", rc);

	return rc;
}

static ssize_t
notification_led_color_show(
	struct device * dev, struct device_attribute * attr, char * buf)
{
	unsigned int state = 0;

	state |= smem_led_color.led_alpha;
	state <<= 8;
	state |= smem_led_color.led_red;
	state <<= 8;
	state |= smem_led_color.led_green;
	state <<= 8;
	state |= smem_led_color.led_blue;

	return sprintf(buf, "%u\n", state);
}

static ssize_t
notification_led_argb_store(
	struct device * dev, struct device_attribute * attr,
	const char * buf, size_t size)
{
	struct led_classdev * led_cdev = dev_get_drvdata(dev);
	struct notification_led_data * led_data
		= container_of(led_cdev, struct notification_led_data, cdev);
	char * after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;
	ssize_t rc = -EINVAL;

	FUNCENTER("dev=%p,attr=%p,buf=%p,size=%d", dev, attr, buf, size);

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		HAL_UT(dev, "store argb value %lu(%08lXh)", state, state);
		led_data->argb = state;
		ARGB_DBG("colorRGB=%08Xh", led_data->argb);
		rc = count;
	}

	FUNCLEAVE("rc=%d", rc);

	return rc;
}

static ssize_t
notification_led_argb_show(struct device * dev, struct device_attribute * attr, char * buf)
{
	struct led_classdev * led_cdev = dev_get_drvdata(dev);
	struct notification_led_data * led_data
		= container_of(led_cdev, struct notification_led_data, cdev);

	return sprintf(buf, "%d\n", led_data->argb);
}

static ssize_t
notification_led_delay_on_store(
	struct device * dev, struct device_attribute * attr,
	const char * buf, size_t size)
{
	struct led_classdev * led_cdev = dev_get_drvdata(dev);
	struct notification_led_data * led_data
		= container_of(led_cdev, struct notification_led_data, cdev);
	int rc = -EINVAL;
	char * after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	FUNCENTER("dev=%p,attr=%p,buf=%p,size=%d", dev, attr, buf, size);

	if (isspace(*after))
		count++;

	if (count == size) {
		HAL_UT(dev, "store delay_on value %lu(%08lXh)", state, state);
		if (led_data->delay_on != state) {
			/* the new value differs from the previous */
			led_data->delay_on = state;

			/* deactivate previous settings */
			del_timer_sync(&led_data->timer);

			/* no hardware acceleration, blink via timer */
			mod_timer(&led_data->timer, jiffies + 1);

			SET_DELAY_DBG("delay_on=%lu", led_data->delay_on);
		}
		rc = count;
	}

	FUNCLEAVE("rc=%d", rc);

	return rc;
}

static ssize_t
notification_led_delay_on_show(struct device * dev, struct device_attribute * attr, char * buf)
{
	struct led_classdev * led_cdev = dev_get_drvdata(dev);
	struct notification_led_data * led_data
		= container_of(led_cdev, struct notification_led_data, cdev);

	return sprintf(buf, "%lu\n", led_data->delay_on);
}

static ssize_t
notification_led_delay_off_store(
	struct device * dev, struct device_attribute * attr,
	const char * buf, size_t size)
{
	struct led_classdev * led_cdev = dev_get_drvdata(dev);
	struct notification_led_data * led_data
		= container_of(led_cdev, struct notification_led_data, cdev);
	int rc = -EINVAL;
	char * after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	FUNCENTER("dev=%p,attr=%p,buf=%p,size=%d", dev, attr, buf, size);

	if (isspace(*after))
		count++;

	if (count == size) {
		HAL_UT(dev, "store delay_off value %lu(%08lXh)", state, state);
		if (led_data->delay_off != state) {
			/* the new value differs from the previous */
			led_data->delay_off = state;

			/* deactivate previous settings */
			del_timer_sync(&led_data->timer);

			/* no hardware acceleration, blink via timer */
			mod_timer(&led_data->timer, jiffies + 1);

			SET_DELAY_DBG("delay_off=%lu", led_data->delay_off);
		}
		rc = count;
	}

	FUNCLEAVE("rc=%d", rc);

	return rc;
}

static ssize_t
notification_led_delay_off_show(
	struct device * dev, struct device_attribute * attr, char * buf)
{
	struct led_classdev * led_cdev = dev_get_drvdata(dev);
	struct notification_led_data * led_data
		= container_of(led_cdev, struct notification_led_data, cdev);

	return sprintf(buf, "%lu\n", led_data->delay_off);
}

static const struct device_attribute notification_led_attrs[] = {
	__ATTR(argb, 0644, notification_led_argb_show, notification_led_argb_store),
	__ATTR(led_color, 0644, notification_led_color_show, notification_led_color_store),
	__ATTR(led_notify, 0644, notification_led_notify_show, notification_led_notify_store),
	__ATTR(delay_on, 0644, notification_led_delay_on_show, notification_led_delay_on_store),
	__ATTR(delay_off, 0644, notification_led_delay_off_show, notification_led_delay_off_store),
	__ATTR_NULL,
};

static int __devinit
notification_led_device_create_files(
	struct device * dev, const struct device_attribute * attrs)
{
	int rc = 0;
	int i;

	FUNCENTER("dev=%p,attrs=%p", dev, attrs);

	for (i = 0; attrs[i].attr.name != NULL ; i++) {
		rc = device_create_file(dev, &attrs[i]);
		if (rc) {
			dev_err(dev, "failure device_create_file \"%s\"\n",
				attrs[i].attr.name);
			while (--i >= 0)
				device_remove_file(dev, &attrs[i]);
			break;
		}
	}

	FUNCLEAVE("rc=%d", rc);

	return rc;
}

static void
notification_led_device_remove_files(
	struct device * dev, const struct device_attribute * attrs)
{
	FUNCENTER("dev=%p,attrs=%p", dev, attrs);

	for ( ; attrs->attr.name != NULL; attrs++)
		device_remove_file(dev, attrs);

	FUNCLEAVE("");
}

static void
notification_led_set(struct led_classdev * led_cdev, enum led_brightness value)
{
	struct notification_led_data * led_data
		= container_of(led_cdev, struct notification_led_data, cdev);
	int red, green, blue;

	FUNCENTER("led_cdev=%p,value=%u", led_cdev, value);

	if (value != LED_OFF) {
		red = ((led_data->argb >> 16) & 255) ? 1 : 0;
		green = ((led_data->argb >> 8) & 255) ? 1 : 0;
		blue = (led_data->argb & 255) ? 1 : 0;
	} else if (smem_led_charge != RES_CHARGE_STS_CHARGING) {
		red = 0;
		green = 0;
		blue = 0;
	} else {
		red = 1;
		green = 0;
		blue = 0;
	}

	LED_SET_DBG(led_data->cdev.dev, "RGB=%d%d%d", red, green, blue);

	gpio_set_value(led_data->red_gpio, red);
	gpio_set_value(led_data->green_gpio, green);
	gpio_set_value(led_data->blue_gpio, blue);

	FUNCLEAVE("");
}

static int __devinit
notification_led_setup_led_gpio(struct platform_device * pdev,
	unsigned int * gpio, unsigned int pin, const char * name)
{
	int rc;

	FUNCENTER("pdev=%p,gpio=%p,pin=%u,name=\"%s\"", pdev, gpio, pin, name);

	*gpio = pin;

	rc = gpio_request(*gpio, name);
	if (rc < 0) {
		dev_err(&pdev->dev, "failure gpio_request gpio[%u] rc=%d\n", pin, rc);
		return rc;
	}

	rc = gpio_direction_output(*gpio, 0);
	if (rc < 0) {
		dev_err(&pdev->dev, "failure gpio_direction_output gpio[%u] rc=%d\n",
			pin, rc);
		gpio_free(*gpio);
	}

	FUNCLEAVE("");

	return rc;
}

static void
notification_led_timer_function(unsigned long data)
{
	struct notification_led_data * led_data
		= (struct notification_led_data *)data;
	unsigned long brightness;
	unsigned long delay;

	FUNCENTER("data=%lx", data);

	if (!led_data->delay_on || !led_data->delay_off) {
		led_set_brightness(&led_data->cdev, LED_OFF);
		FUNCLEAVE("return");
		return;
	}

	brightness = led_get_brightness(&led_data->cdev);
	if (!brightness) {
		/* Time to switch the LED on. */
		brightness = led_data->brightness_on;
		delay = led_data->delay_on;
	} else {
		/* Store the current brightness value to be able
		 * to restore it when the delay_off period is over.
		 */
		led_data->brightness_on = brightness;
		brightness = LED_OFF;
		delay = led_data->delay_off;
	}

	led_set_brightness(&led_data->cdev, brightness);

	mod_timer(&led_data->timer, jiffies + msecs_to_jiffies(delay));

	FUNCLEAVE("mod_timer");
}

#ifdef CONFIG_PM
static int
notification_led_suspend(struct platform_device * pdev, pm_message_t state)
{
	FUNCENTER("pdev=%p", pdev);

	*smem_led_ctrl_notify = smem_led_notify;
	*smem_led_ctrl_charge = smem_led_charge;
	*smem_led_ctrl_color = smem_led_color;

	SMEM_DBG(&pdev->dev, "smem_led_ctrl_notify(@%p) %Xh\n",
		smem_led_ctrl_notify, *smem_led_ctrl_notify);
	SMEM_DBG(&pdev->dev, "smem_led_ctrl_charge(@%p) %Xh\n",
		smem_led_ctrl_charge, *smem_led_ctrl_charge);
	SMEM_DBG(&pdev->dev, "smem_led_ctrl_color(@%p) %u:%u:%u:%u\n",
		smem_led_ctrl_color, smem_led_ctrl_color->led_alpha,
		smem_led_ctrl_color->led_red, smem_led_ctrl_color->led_green,
		smem_led_ctrl_color->led_blue);

	FUNCLEAVE("%d", 0);

	return 0;
}

static int
notification_led_resume(struct platform_device * pdev)
{
	int notify_id;
	int charge_id = RES_CHARGE_STS_ERROR;
	struct notification_led_data * led_data = platform_get_drvdata(pdev);

	FUNCENTER("pdev=%p", pdev);

	if (!(*smem_led_ctrl_notify))
		notify_id = RES_NOTIFY_STS_NULL;
	else if (*smem_led_ctrl_notify & RES_LED_NOT_3RD_MASK)
		notify_id = RES_NOTIFY_STS_3RDAPP;
	else
		notify_id = RES_NOTIFY_STS_NORMAL;

	switch (*smem_led_ctrl_charge) {
	case RES_CHARGE_STS_NULL:
	case RES_CHARGE_STS_CHARGING:
	case RES_CHARGE_STS_ERROR:
		charge_id = *smem_led_ctrl_charge;
		break;
	default:
		dev_err(&pdev->dev, "Illegal charge status value [%d]",
			*smem_led_ctrl_charge);
		break;
	}

	switch (iLedResColorTable[notify_id][charge_id]) {
	case RES_LED_COLOR_APPLICATION_COLOR:
		led_data->argb = ((smem_led_ctrl_color->led_red) << 16)
				| ((smem_led_ctrl_color->led_green) << 8)
				| ((smem_led_ctrl_color->led_blue) << 0);
		led_data->cdev.brightness = led_data->argb
					  ? LED_FULL : LED_OFF;
		break;
	case RES_LED_COLOR_RED:
		led_data->argb = LED_FULL << 16;
		led_data->cdev.brightness = LED_FULL;
		break;
	case RES_LED_COLOR_GREEN:
		led_data->argb = LED_FULL << 8;
		led_data->cdev.brightness = LED_FULL;
		break;
	default:
		led_data->argb = 0;
		led_data->cdev.brightness = LED_OFF;
		break;
	}

	RESUME_DBG(&pdev->dev, "led_data->argb=%08Xh\n", led_data->argb);
	RESUME_DBG(&pdev->dev, "led_data->cdev.brightness=%d\n", led_data->cdev.brightness);

	smem_led_charge = *smem_led_ctrl_charge & 7;

	SMEM_DBG(&pdev->dev, "smem_led_charge %Xh\n", smem_led_charge);

	FUNCLEAVE("");

	return 0;
}
#else
#define notification_led_suspend	NULL
#define notification_led_resume		NULL
#endif

static int __devinit notification_led_probe(struct platform_device * pdev)
{
	struct notification_led_data * led_data;
	int rc = 0;

	FUNCENTER("pdev=%p", pdev);

	smem_led_ctrl_notify = smem_alloc_vendor1(SMEM_OEM_002);
	smem_led_ctrl_charge = smem_alloc_vendor1(SMEM_OEM_003);
	smem_led_ctrl_color = smem_alloc_vendor1(SMEM_OEM_010);
	if (!smem_led_ctrl_notify || !smem_led_ctrl_charge
	 || !smem_led_ctrl_color) {
		dev_err(&pdev->dev, "not found SMEM %p %p %p (probe)\n",
			smem_led_ctrl_notify, smem_led_ctrl_charge,
			smem_led_ctrl_color);
		FUNCLEAVE("%s", "-EFAULT");
		return -EFAULT;
	}

	SMEM_DBG(&pdev->dev, "smem_led_ctrl_notify=%p\n", smem_led_ctrl_notify);
	SMEM_DBG(&pdev->dev, "smem_led_ctrl_charge=%p\n", smem_led_ctrl_charge);
	SMEM_DBG(&pdev->dev, "smem_led_ctrl_color=%p\n", smem_led_ctrl_color);

	led_data = kzalloc(sizeof(*led_data), GFP_KERNEL);
	if (!led_data) {
		dev_err(&pdev->dev, "failure kzalloc\n");
		FUNCLEAVE("%s", "-ENOMEM");
		return -ENOMEM;
	}

	led_data->cdev.name = "notification";
	led_data->cdev.brightness_set = notification_led_set;
	led_data->cdev.brightness = LED_OFF;

	if (notification_led_setup_led_gpio(pdev, &led_data->red_gpio, 130, "led.red"))
		goto err_kmem_free;
	if (notification_led_setup_led_gpio(pdev, &led_data->green_gpio, 129, "led.green"))
		goto err_kmem_free;
	if (notification_led_setup_led_gpio(pdev, &led_data->blue_gpio, 128, "led.blue"))
		goto err_kmem_free;

	init_timer(&led_data->timer);
	led_data->timer.function = notification_led_timer_function;
	led_data->timer.data = (unsigned long)led_data;
	led_data->brightness_on = LED_FULL;

	rc = led_classdev_register(&pdev->dev, &led_data->cdev);
	if (rc < 0) {
		dev_err(&pdev->dev, "failure led_classdev_register rc=%d\n", rc);
		goto err_gpio_free;
	}

	rc = notification_led_device_create_files(led_data->cdev.dev, notification_led_attrs);
	if (rc)
		goto err_classdev_unregister;

	platform_set_drvdata(pdev, led_data);

	FUNCLEAVE("%d", 0);

	return 0;

err_classdev_unregister:
	led_classdev_unregister(&led_data->cdev);

err_gpio_free:
	gpio_free(led_data->blue_gpio);
	gpio_free(led_data->green_gpio);
	gpio_free(led_data->red_gpio);

err_kmem_free:
	kfree(led_data);

	dev_err(&pdev->dev, "failure notification_led_probe %d\n", rc);

	FUNCLEAVE("%d", rc);

	return rc;
}

static int __devexit notification_led_remove(struct platform_device * pdev)
{
	struct notification_led_data * led_data = platform_get_drvdata(pdev);

	FUNCENTER("pdev=%p", pdev);

	notification_led_device_remove_files(led_data->cdev.dev, notification_led_attrs);
	led_classdev_unregister(&led_data->cdev);
	gpio_free(led_data->blue_gpio);
	gpio_free(led_data->green_gpio);
	gpio_free(led_data->red_gpio);
	kfree(led_data);

	FUNCLEAVE("%d", 0);

	return 0;
}

static struct platform_driver notification_led_driver = {
	.probe		= notification_led_probe,
	.remove		= __devexit_p(notification_led_remove),
	.suspend	= notification_led_suspend,
	.resume		= notification_led_resume,
	.driver		= {
		.name	= "notification-led",
		.owner	= THIS_MODULE,
	},
};

static int __init notification_led_init(void)
{
	return platform_driver_register(&notification_led_driver);
}

static void __exit notification_led_exit(void)
{
	platform_driver_unregister(&notification_led_driver);
}

module_init(notification_led_init);
module_exit(notification_led_exit);

MODULE_ALIAS("platform:led-notification");
MODULE_DESCRIPTION("Notification LED driver");
MODULE_LICENSE("GPL v2");
