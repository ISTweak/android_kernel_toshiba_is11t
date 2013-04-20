/* 
  Light Sensor Driver

  Protocol driver for Light sensors.
  Copyiiright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
*/
/*
 * Copyright (c) 2010 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/


#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/platform_device.h>

/* for I2C remote mutex */
#include <linux/remote_spinlock.h>



#include <linux/moduleparam.h>

#include <linux/slab.h>     /* kmalloc() */
#include <linux/fs.h>       /* everything... */
#include <linux/errno.h>    /* error codes */
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <asm/page.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include <mach/vreg.h>
#include "light.h"

#include "../arch/arm/mach-msm/proc_comm.h"

/* for I2C remote mutex */
#include "../arch/arm/mach-msm/smd_private.h"

#include <linux/gpio_event.h>
#include <asm/uaccess.h> 

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#endif
//Flags for parallel userspace and driver handling

/* for debugging */
#define DEBUG 0

//Light Sensor
#define SENSOR_NAME "light"
#define LIGHT_GPIO	100
#define LIGHT_TEST_MODE		0

//changing as per spec
#define SENSOR_DEFAULT_DELAY            (1000)  /* 1000 ms */
#define SENSOR_LED_OFF_DELAY            (150)   /* 150 ms */
#define SENSOR_MAX_DELAY                (2000)  /* 2000 ms */
#define ABS_STATUS                      (ABS_BRAKE)
#define ABS_WAKE                        (ABS_MISC)
#define ABS_CONTROL_REPORT              (ABS_THROTTLE)
#define MAX_LUX_ENTRIES					(256)
#define OUT_OF_SCOPE_SLOPE				(0)
#define NUM_AVG              			(1)
#define LIGHT_DEFAULT_VAL				600

#if DEBUG
#define LIGHT_DBG(x...)  printk(x)
#else
#define LIGHT_DBG(x...) 
#endif

static struct input_dev *input_data = NULL;

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend  e_sus_fcn;
void light_early_suspend(struct early_suspend *h);
void light_late_resume(struct early_suspend *h);
static int light_suspended = 0;
#endif

static struct timer_list tmdrv_timer;
static struct work_struct g_light_work_data;
/* FUJITSU:2011-10-06 add start */
static struct workqueue_struct *g_light_wq;
/* FUJITSU:2011-10-06 add end */

static int g_ls_op_cnt = 0;
static int g_ls_probe_init = 0;
static int g_inside_sw = 0;
static int g_sensor_on_off = 0;
static bool g_gpio_sw_enable = false;

#define STATUATION_1000 				0
#define STATUATION_7000 				1
#define ILLUMINANCE_VAL 				30
#define GPIO_STANDBY_TIME 				5

#define LIGHT_VREG_NAME		"gp4"	/* L10 */
#define LIGHT_VREG_OFF		0
#define LIGHT_VREG_ON		1

static int sensor_get_illuminance(void);
static unsigned sensor_measure_illuminance(int);
static void sensor_vreg_on(void);
static void sensor_vreg_off(void);

struct sensor_data {
    struct input_dev *input_device;
    struct mutex mutex;
    int enabled;
    int delay;
#if DEBUG
    int suspend;
#endif
};

static const unsigned int adc_to_lux[MAX_LUX_ENTRIES][2] = {
	/* (index = adc), {1000LUX,7000LUX} */
	{0,0},
	{4,31},
	{9,62},
	{13,93},
	{17,124},
	{21,155},
	{26,186},
	{30,217},
	{34,248},
	{38,279},
	{43,310},
	{47,341},
	{51,372},
	{55,403},
	{60,434},
	{64,465},
	{68,496},
	{72,527},
	{77,558},
	{81,589},
	{85,620},
	{89,651},
	{94,682},
	{98,713},
	{102,744},
	{106,775},
	{111,806},
	{115,837},
	{119,868},
	{123,899},
	{128,930},
	{132,961},
	{136,992},
	{140,1023},
	{145,1054},
	{149,1085},
	{153,1116},
	{157,1147},
	{162,1178},
	{166,1209},
	{170,1240},
	{174,1271},
	{179,1302},
	{183,1333},
	{187,1364},
	{191,1395},
	{196,1426},
	{200,1457},
	{204,1488},
	{208,1519},
	{213,1550},
	{217,1581},
	{221,1612},
	{225,1643},
	{230,1674},
	{234,1705},
	{238,1736},
	{242,1767},
	{247,1798},
	{251,1829},
	{255,1860},
	{259,1891},
	{264,1922},
	{268,1953},
	{272,1984},
	{276,2015},
	{281,2046},
	{285,2077},
	{289,2108},
	{293,2139},
	{298,2170},
	{302,2201},
	{306,2232},
	{310,2263},
	{315,2294},
	{319,2325},
	{323,2356},
	{327,2387},
	{332,2418},
	{336,2449},
	{340,2480},
	{344,2511},
	{349,2542},
	{353,2573},
	{357,2604},
	{361,2635},
	{366,2666},
	{370,2697},
	{374,2728},
	{378,2759},
	{383,2790},
	{387,2821},
	{391,2852},
	{395,2883},
	{400,2914},
	{404,2945},
	{408,2976},
	{412,3007},
	{417,3038},
	{421,3069},
	{425,3100},
	{429,3131},
	{434,3162},
	{438,3193},
	{442,3224},
	{446,3255},
	{451,3286},
	{455,3317},
	{459,3348},
	{463,3379},
	{468,3410},
	{472,3441},
	{476,3472},
	{480,3503},
	{485,3534},
	{489,3565},
	{493,3596},
	{497,3627},
	{502,3658},
	{506,3689},
	{510,3720},
	{514,3751},
	{519,3782},
	{523,3813},
	{527,3844},
	{531,3875},
	{536,3906},
	{540,3937},
	{544,3968},
	{548,3999},
	{553,4030},
	{557,4061},
	{561,4092},
	{565,4123},
	{570,4154},
	{574,4185},
	{578,4216},
	{582,4247},
	{587,4278},
	{591,4309},
	{595,4340},
	{599,4371},
	{604,4402},
	{608,4433},
	{612,4464},
	{616,4495},
	{621,4526},
	{625,4557},
	{629,4588},
	{633,4619},
	{638,4650},
	{642,4681},
	{646,4712},
	{650,4743},
	{655,4774},
	{659,4805},
	{663,4836},
	{667,4867},
	{672,4898},
	{676,4929},
	{680,4960},
	{684,4991},
	{689,5022},
	{693,5053},
	{697,5084},
	{701,5115},
	{706,5146},
	{710,5177},
	{714,5208},
	{718,5239},
	{723,5270},
	{727,5301},
	{731,5332},
	{735,5363},
	{740,5394},
	{744,5425},
	{748,5456},
	{752,5487},
	{757,5518},
	{761,5549},
	{765,5580},
	{769,5611},
	{774,5642},
	{778,5673},
	{782,5704},
	{786,5735},
	{791,5766},
	{795,5797},
	{799,5828},
	{803,5859},
	{808,5890},
	{812,5921},
	{816,5952},
	{820,5983},
	{825,6014},
	{829,6045},
	{833,6076},
	{837,6107},
	{842,6138},
	{846,6169},
	{850,6200},
	{856,6232},
	{862,6264},
	{868,6296},
	{874,6328},
	{880,6360},
	{886,6392},
	{892,6424},
	{898,6456},
	{904,6488},
	{910,6520},
	{916,6552},
	{922,6584},
	{928,6616},
	{934,6648},
	{940,6680},
	{946,6712},
	{952,6744},
	{958,6776},
	{964,6808},
	{970,6840},
	{976,6872},
	{982,6904},
	{988,6936},
	{994,6968},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000},
	{1000,7000}
};

static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;
static struct sensor_data *data = NULL;
static struct vreg *light_vreg_l10;

#if LIGHT_TEST_MODE
/* 
 * For TEST_MODE
 */

#include <linux/miscdevice.h>
#include <linux/sensor_light_test_if.h>

static unsigned int light_test_msm_adc_read(void)
{
	unsigned int data1, data2;
	int ret;
	
	data1 = 6;
	data2 = 0;
	ret = msm_proc_comm(PCOM_OEM_007, &data1, &data2);
	LIGHT_DBG("***** LIGHT_DEBUG :msm adc=%02x, ret=%d\n", data2, ret);
	return data2;
}

static int light_test_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	LIGHT_DBG(KERN_INFO "***** LIGHT_DEBUG :ENTER %s\n", __func__);
	LIGHT_DBG(KERN_INFO "***** LIGHT_DEBUG :LEAVE %s\n", __func__);
	return ret;
}

static int light_test_release(struct inode *inode, struct file *file)
{
	LIGHT_DBG(KERN_INFO "***** LIGHT_DEBUG : ENTER %s \n", __func__);
	LIGHT_DBG(KERN_INFO "***** LIGHT_DEBUG : LEAVE %s \n", __func__);
	return 0;
}

static int
light_test_ioctl(struct inode *inode, struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int result = 0;
	unsigned int data/*, on_off*/;
	
	switch(cmd) {
		case SENSOR_LIGHT_TESTMODE_START:
		case SENSOR_LIGHT_TESTMODE_END:
		case SENSOR_LIGHT_TESTMODE_MSM_ADC:
		case SENSOR_LIGHT_TESTMODE_SET_1K:
		case SENSOR_LIGHT_TESTMODE_SET_7K:
			del_timer(&tmdrv_timer);
			break;
		
		default:
			LIGHT_DBG(KERN_INFO "default case\n");
			break;
	}
	
	LIGHT_DBG(KERN_INFO "***** LIGHT_DEBUG :ENTER %s cmd=%x\n", __func__, cmd);
	switch (cmd) {
		case SENSOR_LIGHT_TESTMODE_START:
			sensor_vreg_on();
			break;

		case SENSOR_LIGHT_TESTMODE_END:
			sensor_vreg_off();
			break;

		case SENSOR_LIGHT_TESTMODE_MSM_ADC:
			data = light_test_msm_adc_read();
			if (copy_to_user(argp, &data, sizeof(unsigned int))) {
				LIGHT_DBG(KERN_INFO "***** LIGHT_DEBUG :copy to user failed:%d\n",__LINE__);
				result = -EFAULT;
			}
			break;

		case SENSOR_LIGHT_TESTMODE_SET_1K:
			gpio_set_value(LIGHT_GPIO, STATUATION_1000);
			msleep(GPIO_STANDBY_TIME);
			LIGHT_DBG("[LIGHT]GPIO[%d] val= %d\n", LIGHT_GPIO, gpio_get_value(LIGHT_GPIO));
			break;
		
		case SENSOR_LIGHT_TESTMODE_SET_7K:
			gpio_set_value(LIGHT_GPIO, STATUATION_7000);
			msleep(GPIO_STANDBY_TIME);
			LIGHT_DBG("[LIGHT]GPIO[%d] val= %d\n", LIGHT_GPIO, gpio_get_value(LIGHT_GPIO));
			break;
		
		default:
			LIGHT_DBG(KERN_INFO "default case\n");
			break;
	}
	LIGHT_DBG(KERN_INFO "***** LIGHT_DEBUG :LEAVE %s -> %d \n", __func__, result);
	return result;
}

static struct file_operations light_fops = {
	.owner = THIS_MODULE,
	.open = light_test_open,
	.release = light_test_release,
	.ioctl = light_test_ioctl,
};

static struct miscdevice light_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "light",
	.fops = &light_fops,
};
#endif

unsigned
get_proc_comm_data(void)
{
    unsigned data1, data2;
	int ret;
    data1 = 6; //adc_read_lx
    data2 = 0;

    ret = msm_proc_comm(PCOM_OEM_007, &data1, &data2);
    LIGHT_DBG("[nak]get_proc_comm_data() - adc qsd %02x, ret=%d\n", data2,ret);
	if(ret < 0)
	{
		data2 = ret;
	}
    return data2;
}

unsigned int
get_lux_from_adc(unsigned data, int statuation)
{
	if(MAX_LUX_ENTRIES <= data)
	{
		return LIGHT_DEFAULT_VAL;
	}
	
	return adc_to_lux[data][statuation];
}

/*
 * checks if the sensor has been opened already
 * and only if everything has been initialized
 * before, it schedules the bottom half.
 *
*/
static int
light_open_sensor(void)
{
   LIGHT_DBG("\n[LIGHT] OPEN called  [%s] start, line [%d]===\n", __FUNCTION__,__LINE__);
   mutex_lock(&data->mutex);
   if (0 >= g_ls_op_cnt)
   {
	   if (0 == g_inside_sw)
	   {
/* FUJITSU:2011-10-31 mod start */
//	   		schedule_work(&g_light_work_data);
	   		if (g_light_wq != NULL) {
				queue_work(g_light_wq, &g_light_work_data);
	   		}
/* FUJITSU:2011-10-31 mod end */
	   }
   }
   g_ls_op_cnt ++;
   mutex_unlock(&data->mutex);
   return 0;
}
/*
 * checks if open count is >0 and only then
 * decrements the open counter
 * 
 *
*/
static int
light_close_sensor(void)
{
    LIGHT_DBG("\n[LIGHT] OPEN called  [%s] start, line [%d]===\n", __FUNCTION__,__LINE__);
	mutex_lock(&data->mutex);
	if (0 == g_ls_op_cnt) 
	{
		mutex_unlock(&data->mutex);
		return -1;
	}
	g_ls_op_cnt --;
	mutex_unlock(&data->mutex);
	return 0;
}

/* Sysfs interface */
static ssize_t
sensor_delay_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int delay;

    mutex_lock(&data->mutex);

    delay = data->delay;

    mutex_unlock(&data->mutex);

    return sprintf(buf, "%d\n", delay);
}

static ssize_t
sensor_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int value = simple_strtoul(buf, NULL, 10);

    if (value < 0) {
        return count;
    }

    if (SENSOR_MAX_DELAY < value) {
        value = SENSOR_MAX_DELAY;
    }

    mutex_lock(&data->mutex);

    data->delay = value;

    input_report_abs(input_data, ABS_CONTROL_REPORT, (data->enabled<<16) | value);

    mutex_unlock(&data->mutex);

    return count;
}

static ssize_t
sensor_enable_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int enabled;

    mutex_lock(&data->mutex);

    enabled = data->enabled;

    mutex_unlock(&data->mutex);

    return sprintf(buf, "%d\n", enabled);
}

static ssize_t
sensor_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int value = simple_strtoul(buf, NULL, 10);

    if (value != 0 && value != 1) {
        return count;
    }

    mutex_lock(&data->mutex);

    data->enabled = value;

    input_report_abs(input_data, ABS_CONTROL_REPORT, (value<<16) | data->delay);

    mutex_unlock(&data->mutex);

    return count;
}

static ssize_t
sensor_wake_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    static int cnt = 1;

    input_report_abs(input_data, ABS_WAKE, cnt++);

    return count;
}

static ssize_t
sensor_data_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
    int x;

    spin_lock_irqsave(&input_data->event_lock, flags);

    x = input_data->abs[ABS_X];

    spin_unlock_irqrestore(&input_data->event_lock, flags);

	return sprintf(buf, "%d\n", x);
}

static ssize_t
sensor_status_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    unsigned long flags;
    int status;

    spin_lock_irqsave(&input_data->event_lock, flags);

    status = input_data->abs[ABS_STATUS];

    spin_unlock_irqrestore(&input_data->event_lock, flags);

    return sprintf(buf, "%d\n", status);
}
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
        sensor_delay_show, sensor_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
        sensor_enable_show, sensor_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
        NULL, sensor_wake_store);
static DEVICE_ATTR(data, S_IRUGO, sensor_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, sensor_status_show, NULL);

static struct attribute *sensor_attributes[] = {
	&dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_wake.attr,
	&dev_attr_data.attr,
    &dev_attr_status.attr,
	NULL
};

static struct attribute_group sensor_attribute_group = {
    .attrs = sensor_attributes
};

/*
 * This is the bottom half which fires
 * at regular intervals and reads the ADC
 * value using msm_proc_comm and then converting
 *it to lux, finally syncing it to input subsystem
*/
static void 
light_work_bh(struct work_struct *work)
{
	mutex_lock(&data->mutex);
	g_inside_sw = 1;
	mutex_unlock(&data->mutex);

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (0 == light_suspended) 
#endif
	{
		input_report_abs(input_data, ABS_X, sensor_get_illuminance());
		input_sync(input_data);
	}

	mutex_lock(&data->mutex);
	if (g_ls_op_cnt > 0)
	{
		mod_timer(&tmdrv_timer,jiffies + msecs_to_jiffies(SENSOR_DEFAULT_DELAY));
	}
	g_inside_sw = 0;
	mutex_unlock(&data->mutex);
}

static void 
light_timer_func(unsigned long ptr)
{
	LIGHT_DBG("[LIGHT] schedule work is called===\n");
/* FUJITSU:2011-10-06 mod start */
//	schedule_work(&g_light_work_data);
	queue_work(g_light_wq, &g_light_work_data);
/* FUJITSU:2011-10-06 mod end */
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void light_early_suspend(struct early_suspend *h)
{
	light_suspended = 1;
	LIGHT_DBG("[LIGHT] light_early_suspend!!!!\n");
}
void light_late_resume(struct early_suspend *h)
{
	light_suspended = 0;
	LIGHT_DBG("[LIGHT] light_late_resume \n");
}
#endif

/*
 * open routine for input 
 * susbsystem
 * 
 *
*/
static int 
input_open (struct input_dev *dev)
{
	if (0 == g_ls_probe_init)
	{
   	    LIGHT_DBG("[LIGHT] Failed  [%s]:[%d]===\n", __FUNCTION__,__LINE__);
		return -1;
	}

	LIGHT_DBG("\n[LIGHT] in OPEN called  [%s] start, line [%d]===\n", __FUNCTION__,__LINE__);
	light_open_sensor();
	return 0;
}

/*
 * close routine for input 
 * susbsystem
 * 
 *
*/
static void 
input_close (struct input_dev *dev)
{
	if (0 == g_ls_probe_init)
	{
   	    LIGHT_DBG("[LIGHT] Failed  [%s]:[%d]===\n", __FUNCTION__,__LINE__);
		return;
	}

	LIGHT_DBG("\n[LIGHT] in CLOSE called  [%s] start, line [%d]===\n", __FUNCTION__,__LINE__);
	light_close_sensor();
	return;
}

static void sensor_vreg_on(void)
{
	int rc;
	pr_debug("%s\n", __func__);

	if(LIGHT_VREG_ON == g_sensor_on_off)
	{
		LIGHT_DBG("[LIGHT] VOLTAGE ALREADY ON!!!\n");
		return;
	}
	
	light_vreg_l10 = vreg_get(NULL, LIGHT_VREG_NAME);

	if (!light_vreg_l10) {
		pr_err("%s: VREG L10 get failed\n", __func__);
		return;
	}

	rc = vreg_set_level(light_vreg_l10, 3000);
	if (rc) {
		pr_err("%s: VREG L10 set failed\n", __func__);
		goto l10_put;
	}

	rc = vreg_enable(light_vreg_l10);
	if (rc) {
		pr_err("%s: VREG L10 enable failed\n", __func__);
		goto l10_put;
	}
	
	LIGHT_DBG("[LIGHT] VOLTAGE ON SUCCESS!!!\n");
	g_sensor_on_off = LIGHT_VREG_ON;
	
	/* FUJITSU:2011-07-15 add start */
	return;
	/* FUJITSU:2011-07-15 add end */
	
l10_put:
    vreg_put(light_vreg_l10);
    return;
}

static void sensor_vreg_off(void)
{
	int rc;
	LIGHT_DBG("[LIGHT] VOLTAGE OFF ENTER\n");

	if(LIGHT_VREG_OFF == g_sensor_on_off)
	{
		LIGHT_DBG("[LIGHT] VOLTAGE ALREADY OFF!!!\n");
		return;
	}
	
	if (IS_ERR(light_vreg_l10)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(light_vreg_l10));
		return;
	}
	rc = vreg_disable(light_vreg_l10);
	if (rc)
	{
		LIGHT_DBG("%s: vreg_disable(ncp) failed (%d)\n", __func__, rc);
	}
	else
	{
		LIGHT_DBG("[LIGHT] VOLTAGE OFF SUCCESS!!!\n");
	}
	vreg_put(light_vreg_l10);
	light_vreg_l10 = NULL;
	g_sensor_on_off = LIGHT_VREG_OFF;
}

static unsigned
sensor_measure_illuminance(int statuation)
{
	int adc;

	gpio_set_value(LIGHT_GPIO, statuation);
	
	msleep(GPIO_STANDBY_TIME);
	
	adc = get_proc_comm_data();
	
	return adc;
}

static int
sensor_get_illuminance(void)
{
	int adc1000 = 0;
	int adc7000 = 0;

	if((g_sensor_on_off == LIGHT_VREG_OFF) || !g_gpio_sw_enable)
	{
		/* if sensor power state is off return defautl value */
		return LIGHT_DEFAULT_VAL;
	}

	adc1000 = sensor_measure_illuminance(STATUATION_1000);
	adc7000 = sensor_measure_illuminance(STATUATION_7000);
	
	if(ILLUMINANCE_VAL < adc7000)
	{
		return get_lux_from_adc(adc7000, STATUATION_7000);
	}
	return get_lux_from_adc(adc1000, STATUATION_1000);
}

static void sensor_gpio_on(void)
{
	int rt;
	
	g_gpio_sw_enable = false;
	
	rt = gpio_request(LIGHT_GPIO, SENSOR_NAME);
	if(rt){
		return;
	}
	
	//added to set it to no pull as per spec
	rt = gpio_tlmm_config(GPIO_CFG(LIGHT_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	if(rt){
		return;
	}
	
	g_gpio_sw_enable = true;
}

static int
sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
    int rt = 0;

/* FUJITSU:2011-10-06 add start */
	cancel_work_sync(&g_light_work_data);
/* FUJITSU:2011-10-06 add end */
	del_timer(&tmdrv_timer);
	sensor_vreg_off();
    return rt;
}

static int
sensor_resume(struct platform_device *pdev)
{
    int rt = 0;

	sensor_vreg_on();
/* FUJITSU:2011-10-31 mod start */
//	setup_timer(&tmdrv_timer, light_timer_func, 0);
//	schedule_work(&g_light_work_data);
	if (g_light_wq != NULL) {
		queue_work(g_light_wq, &g_light_work_data);
	}
/* FUJITSU:2011-10-31 mod end */
    return rt;
}


/*
 * sensor probe routine : does all sensor 
 * initialization and input subsystem creation
 * 
 *
*/
static int
sensor_probe(struct platform_device *pdev)
{
	int input_registered = 0;
	int sysfs_created = 0;
	int rt;

	LIGHT_DBG("\n[LIGHT] in PROBE called  [%s] start, line [%d]===\n", __FUNCTION__,__LINE__);

	data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!data) {
		rt = -ENOMEM;
		goto err;
	}
	
	data->enabled = 1;
	data->delay = SENSOR_DEFAULT_DELAY;

	input_data = input_allocate_device();
	if (!input_data) {
		rt = -ENOMEM;
		printk(KERN_ERR
			   "sensor_probe: Failed to allocate input_data device\n");
		goto err;
	}
	data->input_device = input_data;

	set_bit(EV_ABS, input_data->evbit);
	set_bit(ABS_X,     input_data->absbit);
	input_set_abs_params(input_data, ABS_X, 0, 7000, 0, 0);

	input_data->name = SENSOR_NAME;
	input_data->open = input_open;
	input_data->close = input_close;

	rt = input_register_device(input_data);
	if (rt) {
		printk(KERN_ERR
			   "sensor_probe: Unable to register input_data device: %s\n",
			   input_data->name);
		goto err;
	}
	input_registered = 1;
	rt = sysfs_create_group(&input_data->dev.kobj,
			&sensor_attribute_group);
	if (rt) {
		printk(KERN_ERR
			   "sensor_probe: sysfs_create_group failed[%s]\n",
			   input_data->name);
		goto err;
	}
	sysfs_created = 1;
	mutex_init(&data->mutex);
	this_data = input_data;

	input_report_abs(input_data, ABS_X, LIGHT_DEFAULT_VAL);
	input_sync(input_data);	
	
	//initialize work queue
/* FUJITSU:2011-10-28 add start */
	g_light_wq = create_singlethread_workqueue( "light_wq" );
	if (g_light_wq == NULL) {
		printk(KERN_ERR
			   "sensor_probe: create work queue failed[%s]\n",
			   input_data->name);
		goto err;
	}
/* FUJITSU:2011-10-28 add end */
	INIT_WORK(&g_light_work_data, light_work_bh);
	setup_timer(&tmdrv_timer, light_timer_func, 0);
	g_ls_probe_init = 1;
#ifdef CONFIG_HAS_EARLYSUSPEND
	e_sus_fcn.suspend = light_early_suspend;
	e_sus_fcn.resume = light_late_resume;
	register_early_suspend(&e_sus_fcn);
#endif

	sensor_vreg_on();
	
	sensor_gpio_on();
	
	return 0;

	err:
	if (data != NULL) {
		printk(KERN_CRIT "sensor_probe: failed to initilie the light driver\n");
		if (input_data != NULL) {
			if (sysfs_created) {
				sysfs_remove_group(&input_data->dev.kobj,
						&sensor_attribute_group);
			}
			if (input_registered) {
				input_unregister_device(input_data);
			}
			else {
				input_free_device(input_data);
			}
			input_data = NULL;
		}
		kfree(data);
	}

	return rt;
}

static int
sensor_remove(struct platform_device *pdev)
{
    struct sensor_data *data;
	g_ls_probe_init = 0;
    if (this_data != NULL) {
        data = input_get_drvdata(this_data);
        sysfs_remove_group(&this_data->dev.kobj,
                &sensor_attribute_group);
        input_unregister_device(this_data);
        if (data != NULL) {
            kfree(data);
        }
    }
/* FUJITSU:2011-10-31 add start */
	if (g_light_wq != NULL) {
		destroy_workqueue(g_light_wq);
	}
/* FUJITSU:2011-10-31 add end */

    return 0;
}



/*
 * Module init and exit
 */
static struct platform_driver sensor_driver = {
    .probe      = sensor_probe,
    .remove     = sensor_remove,
    .suspend    = sensor_suspend,
    .resume     = sensor_resume,
    .driver = {
        .name   = SENSOR_NAME,
        .owner  = THIS_MODULE,
    },
};

/*
 * sensor module initialization
*/
static int __init sensor_init(void)
{
	sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
	if (IS_ERR(sensor_pdev)) {
		return -1;
	} 
	platform_driver_register(&sensor_driver);

#if LIGHT_TEST_MODE
	misc_register(&light_device);
#endif

	return 0;
}
module_init(sensor_init);

/*
 * exit sensor module
 * 
 * 
 *
*/
static void __exit sensor_exit(void)
{
    platform_driver_unregister(&sensor_driver);
    platform_device_unregister(sensor_pdev);
}
module_exit(sensor_exit);

MODULE_AUTHOR("Fujitsu");
MODULE_LICENSE( "GPL" );
MODULE_VERSION("1.2.0");
