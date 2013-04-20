/*
  Proximty Sensor Driver
  Copyright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

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
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA */
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
 * MA  02110-1301, USA. */
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
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/kthread.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include "proximity.h"
#include <asm/uaccess.h>
#include <asm/page.h>
#include <linux/i2c.h>
#include <mach/pmic.h>
#include <mach/vreg.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include "prox_i2c_gpio.h"
#include "../arch/arm/mach-msm/proc_comm.h"

/*===========================================================================

  MACROS

  ===========================================================================*/
/* for debugging */
#define DEBUG 0

#if DEBUG
#define PROX_DBG(x...)  printk(x)
#else
#define PROX_DBG(x...)  
#endif

/* for GPIO initialize */
#define SENSOR_NAME "proximity"
#define SENSOR_DEFAULT_DELAY	(200)   /* 200 ms */
#define SENSOR_MAX_DELAY		(2000)  /* 2000 ms */
#define ABS_STATUS				(ABS_BRAKE)
#define ABS_WAKE				(ABS_MISC)
#define ABS_CONTROL_REPORT		(ABS_THROTTLE)
#define PROX_GPIO_1				83
#define PROX_GPIO_2				142
#define WAKE_ON					1
/* FUJITSU:2011-06-17 GPIO SETTINGS del start */
//#define PROX_GPIO_OUT_LO		0
//#define PROX_GPIO_OUT_HI		1
/* FUJITSU:2011-06-17 GPIO SETTINGS del end */

/* for I2C */
#define SNS_STANDBY_TIME		10

#define I2C_WRITE_BUFSIZE		64
#define READ_SIZE				2

/* FUJITSU:2011-06-17 GPIO SETTINGS del start */
//#define I2C_TRANS_WRITE			1
//#define I2C_TARNS_READ			2
/* FUJITSU:2011-06-17 GPIO SETTINGS del end */

#define I2C_READ_POS			1

#define PROX_SLAVE_ADDRESS	  	(0xC8 >> 1)

/* FUJITSU:2011-06-17 GPIO SETTINGS del start */
//#define GPIOF_ENABLE_WAKE		0x40000000
/* FUJITSU:2011-06-17 GPIO SETTINGS del end */

#define	INT_CLEAR				0x80

#define	PROX_REG				0x00
#define	PROX_1					0x01
#define	PROX_VO_DET				0x01

#define	GAIN_REG				0x01
#define	GAIN_REG_INIT			0x00
#define	GAIN_LED				0x08

#define	HYS_REG					0x02
#define HYS_REG_INIT			0x00
#define HYS_HYSF				0x01
#define HYS_HYSF_1				0x02
#define HYS_HYSF_2				0x04
#define HYS_HYSF_3				0x08

#define	HYS_HYSC				0x20
#define	HYS_HYSC_1				0x40
#define	HYS_HYSD				0x80

#define	CYCLE_REG				0x03
#define	CYCLE_REG_INIT			0x00
#define	CYCLE_OSC				0x04
#define	CYCLE_CYCL				0x08
#define	CYCLE_CYCL_1			0x10
#define	CYCLE_CYCL_2			0x20
#define	CYCLE_256MS				0x2C

#define	OPMOD_REG				0x04
#define	OPMOD_REG_INIT			0x00
#define	OPMOD_SSD				0x01
#define	OPMOD_VCON				0x02
#define	OPMOD_ASD				0x10

#define	CON_REG					0x06
#define	CON_REG_INIT			0x00
#define	CON_OCON				0x08
#define	CON_OCON_1				0x10

#define PROX_REG_NEAR			1
#define PROX_REG_FAR			0

/* for notification */
#define PROX_VAL_NEAR			0
#define PROX_VAL_FAR			1

/* for sensor status */
#define PROX_STATE_WAKE			0
#define PROX_STATE_SUSPEND		1
#define PROX_STATE_WAIT_I2C		2

/* for sensor settings */
#define NV_OEM_TOP_ITEMS_I		10000
#define NV_PROX_SENSE_A_I		(NV_OEM_TOP_ITEMS_I+106)
#define NV_PROX_SENSE_B_I		(NV_OEM_TOP_ITEMS_I+107)

#define PROX_SENS_MAX_NUM		31
#define PROX_SENS_A_DEFAULT		12
#define PROX_SENS_B_DEFAULT		21
#define PROX_TEST_SENS_NUM		32

/* for sensor delay */
#define PROX_DETECT_NON			0
#define PROX_DETECTING			1
#define PROX_DETECTED			2

#define PROX_VREG_NAME			"gp7"	/* L8 */
#define PROX_CLASS_NAME			"proximity_dev"

#define FALSE false
#define TRUE true

#define PROX_VREG_ON			1
#define PROX_VREG_OFF			0

#define PROX_TEST_MODE			0	/* for test command */

/*===========================================================================

  STRUCTS

  ===========================================================================*/
struct sensor_data {
	struct mutex mutex;
	int enabled;
	int delay;
};

struct prox_detect_state {
	unsigned char proxy_val;
	unsigned char proxy_state;
};

/*===========================================================================

  GLOBAL VARIABLES

  ===========================================================================*/
static int g_prox_gpio = -1;

static atomic_t g_prox_val;
static atomic_t g_prox_state;

static unsigned int g_proxi_irq;
static struct work_struct g_proxi_work_data;
static unsigned char g_prox_sense_a;
static unsigned char g_prox_sense_b;

unsigned char prox_sensitivity_table[PROX_TEST_SENS_NUM] = {
	0x04,	/*  0 :0.25 */
	0x05,	/*  1 :0.31 */
	0x06,	/*  2 :0.38 */
	0x07,	/*  3 :0.44 */
	0x00,	/*  4 :0.50 */
	0x01,	/*  5 :0.56 */
	0x25,	/*  6 :0.63 */
	0x0D,	/*  7 :0.69 */
	0x26,	/*  8 :0.75 */
	0x0F,	/*  9 :0.81 */
	0x27,	/*  10:0.88 */
	0x09,	/*  11:0.94 */
	0x20,	/*  12:1.00 */
	0x0B,	/*  13:1.06 */
	0x21,	/*  14:1.13 */
	0x45,	/*  15:1.25 */
	0x2D,	/*  16:1.38 */
	0x46,	/*  17:1.50 */
	0x2F,	/*  18:1.63 */
	0x47,	/*  19:1.75 */
	0x29,	/*  20:1.88 */
	0x40,	/*  21:2.00 */
	0x2B,	/*  22:2.13 */
	0x41,	/*  23:2.25 */
	0x4C,	/*  24:2.50 */
	0x4D,	/*  25:2.75 */
	0x4E,	/*  26:3.00 */
	0x4F,	/*  27:3.25 */
	0x48,	/*  28:3.50 */
	0x49,	/*  29:3.75 */
	0x4A,	/*  30:4.00 */
	0x4B,	/*  31:4.25 */
};

#if PROX_TEST_MODE
#define PROX_SENS_DETECT_OFFSET 7
#define PROX_SENS_A_MAX_NUM 23
static struct class* proximity_class;
static int proximity_major = 0;

unsigned char prox_sens_a_b_table[PROX_SENS_A_MAX_NUM] = {
/*     a            b   */
	/* 0 :0.25 */	4,	/* 0.50 */
	/* 1 :0.31 */	6,	/* 0.63 */
	/* 2 :0.38 */	8,	/* 0.75 */
	/* 3 :0.44 */	10,	/* 0.88 */
	/* 4 :0.50 */	12,	/* 1.00 */
	/* 5 :0.56 */	14,	/* 1.13 */
	/* 6 :0.63 */	15,	/* 1.25 */
	/* 7 :0.69 */	16,	/* 1.38 */
	/* 8 :0.75 */	17,	/* 1.50 */
	/* 9 :0.81 */	18,	/* 1.63 */
	/* 10:0.88 */	19,	/* 1.75 */
	/* 11:0.94 */	20,	/* 1.88 */
	/* 12:1.00 */	21,	/* 2.00 */
	/* 13:1.06 */	22,	/* 2.13 */
	/* 14:1.13 */	23,	/* 2.25 */
	/* 15:1.25 */	24,	/* 2.50 */
	/* 16:1.38 */	25,	/* 2.75 */
	/* 17:1.50 */	26,	/* 3.00 */
	/* 18:1.63 */	27,	/* 3.25 */
	/* 19:1.75 */	28,	/* 3.50 */
	/* 20:1.88 */	29,	/* 3.75 */
	/* 21:2.00 */	30,	/* 4.00 */
	/* 22:2.13 */	31,	/* 4.25 */
};
#endif

static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;
static struct prox_detect_state prox_detect;
static struct timer_list prox_timer;
static struct vreg *prox_vreg_l8;
static int prox_wait = 150;
static int prox_vreg_on_off = PROX_VREG_OFF;

/*===========================================================================

  LOCAL FUNCTION PROTOTYPES

  ===========================================================================*/
static int prodrv_sns_reg_init(void);
static int  prodrv_sns_i2c_write( unsigned char reg, unsigned char data);
static int prodrv_sns_i2c_read(unsigned short reg, unsigned char *data, uint32_t len);
static void prodrv_sns_ssd(void);
int prodrv_sns_ON(void);
void prodrv_sns_OFF(void);
static int prodrv_sns_request_irqs(void);
int prox_read_nvitem(unsigned int id);

#if PROX_TEST_MODE 
int prox_write_nvitem(unsigned int id, unsigned char data);
#endif

/*===========================================================================

  LOCAL FUNCTIONS

  ===========================================================================*/
/* Sysfs interface */
static ssize_t
prodrv_sysfs_delay_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int delay;
	
	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_delay_show=====\n");
	
	if(NULL == data) {
		PROX_DBG("=====<PROXIMITY> prodrv_sysfs_delay_show NULL=====\n");
		return 0;
	}
	
	mutex_lock(&data->mutex);

	delay = data->delay;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", delay);
}

static ssize_t
prodrv_sysfs_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int value = simple_strtoul(buf, NULL, 10);
	int enabled;

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_delay_store=====\n");

	if(NULL == data) {
		PROX_DBG("=====<PROXIMITY> prodrv_sysfs_delay_store NULL=====\n");
		return 0;
	}
	
	if (value < 0) {
		return count;
	}

	if (SENSOR_MAX_DELAY < value) {
		value = SENSOR_MAX_DELAY;
	}

	mutex_lock(&data->mutex);

	enabled = data->enabled;
	data->delay = value;

	input_report_abs(input_data, ABS_CONTROL_REPORT, (enabled<<16) | value);

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t
prodrv_sysfs_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int enabled;

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_enable_show=====\n");
	
	if(NULL == data) {
		PROX_DBG("=====<PROXIMITY> prodrv_sysfs_enable_show NULL=====\n");
		return 0;
	}
	
	mutex_lock(&data->mutex);

	enabled = data->enabled;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t
prodrv_sysfs_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int value = simple_strtoul(buf, NULL, 10);

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_enable_store=====\n");
	
	if ((value != 0) && (value != 1)) {
		PROX_DBG("=====<PROXIMITY> prodrv_sysfs_enable_store return err=====\n");
		return count;
	}

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_enable_store value:%d=====\n", value);
	if(value) {
		/*if it has been set to 1, call is incoming*/
		prodrv_sns_ON();
	}
	else {
		prodrv_sns_OFF();
	}
		
	return count;
}

static ssize_t
prodrv_sysfs_wake_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	static int cnt = 1;

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_wake_store=====\n");

	input_report_abs(input_data, ABS_WAKE, cnt++);

	return count;
}

static ssize_t
prodrv_sysfs_data_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int x;

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_data_show=====\n");
	
	spin_lock_irqsave(&input_data->event_lock, flags);

	x = input_data->abs[ABS_X];

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_data_show:%d=====\n",x);
	
	return sprintf(buf, "%d\n", x);
}

static ssize_t
prodrv_sysfs_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int status;

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_status_show=====\n");
	
	spin_lock_irqsave(&input_data->event_lock, flags);

	status = input_data->abs[ABS_STATUS];

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_status_show:%d=====\n",status);
	
	return sprintf(buf, "%d\n", status);
}

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		prodrv_sysfs_delay_show, prodrv_sysfs_delay_store);
static DEVICE_ATTR(enable, S_IWUGO|S_IRUGO|S_IWUSR|S_IWGRP,
		prodrv_sysfs_enable_show, prodrv_sysfs_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
		NULL, prodrv_sysfs_wake_store);
static DEVICE_ATTR(dfdata, S_IRUGO, prodrv_sysfs_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, prodrv_sysfs_status_show, NULL);

static struct attribute *sensor_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_wake.attr,
	&dev_attr_dfdata.attr,
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group sensor_attribute_group = {
	.attrs = sensor_attributes
};

/********************************/
/* for test mode and sensor hal */
/********************************/
#if PROX_TEST_MODE
/*fops open to enable access via /dev/proximity*/
int
proximity_open(struct inode* inode,struct file* file)
{
	//int ret;

	PROX_DBG("proximity: proximity_open called\n");

	return 0;
}

/*fops read to enable access via /dev/proximity*/
static int
proximity_read(struct file * file, char * buff, size_t count, loff_t *pos)
{
	unsigned data1=0;

	data1 = gpio_get_value(g_prox_gpio);
	buff[0] = data1;
	PROX_DBG(KERN_CRIT "\n[PROXIMITY] in proximity_read--value read is [0x%x]===\n", data1);
	return 0;
}

/*fops release to enable access via /dev/proximity*/
int
proximity_release(struct inode *inode, struct file *file)
{
	PROX_DBG("proximity: proximity_release called\n");	
	return 0;
}

/*fops ioctl to enable access via /dev/proximity*/
static long
proximity_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int result = 0;

	unsigned char data[2];	
/* Android_PROX_T01_e */
/* Android_PROX_T02_s */
    int sampl_num;
    int test_num;
    unsigned long sampl_data[5] = {0};
/* Android_PROX_T02_e */
/* Android_PROX_T03_s */
    unsigned long ret_data[6] = {0};
	unsigned char detect_level = 0;
	unsigned char level_a;
	unsigned char level_b;
/* Android_PROX_T03_e */
	
	switch (cmd)
	{
		case IOCTL_PRODRV_ON:
			prodrv_sns_ON();
			break;

		case IOCTL_PRODRV_OFF:
			prodrv_sns_OFF();
			break;

/* Android_PROX_T01_s */
		case IOCTL_PRODRV_CHK:
			prodrv_sns_i2c_read(PROX_REG, data, 2);
			data[1] &= 0x01;
			PROX_DBG("[proximity]IOCTL_PRODRV_CHK: %d\n", data[1]);
			if (copy_to_user(argp, &data[1], sizeof(unsigned char))) {
				PROX_DBG("[proximity]copy to user failed:%d\n",__LINE__);
				result = -EFAULT;
			}
			break;
/* Android_PROX_T01_e */
/* Android_PROX_T02_s */
		case IOCTL_PRODRV_FAILDETECT_CHK:
			disable_irq_nosync(g_proxi_irq);

			for(test_num = 0;test_num < 5;test_num++)
			{
				for(sampl_num = 0;sampl_num < PROX_TEST_SENS_NUM ;sampl_num++)
				{
					prodrv_sns_i2c_write( HYS_REG, prox_sensitivity_table[sampl_num]);
					msleep(9); /* 8ms wait */
					prodrv_sns_i2c_read(PROX_REG, data, 2);
					if(data[1] & 0x01)
					{
						sampl_data[test_num] |= (0x0001 << sampl_num);
/* Android_PROX_T03_s */
						if(sampl_num > detect_level)
						{
							detect_level = sampl_num;
						}
/* Android_PROX_T03_e */
					}
				}
			}
			PROX_DBG("[proximity]IOCTL_PRODRV_FAILDETECT_CHK: %08lx %08lx %08lx %08lx %08lx\n",
			           sampl_data[0],sampl_data[1],sampl_data[2],sampl_data[3],sampl_data[4]);
/* Android_PROX_T03_s */
			memcpy(&ret_data[1],&sampl_data[0],sizeof(sampl_data));
			level_a = detect_level + PROX_SENS_DETECT_OFFSET;
		
			/* for debug print s */
			printk("[PROX] 7bit shift level_a = %d\n", level_a);
			/* foradebug print e */
		
			if(level_a < PROX_SENS_A_MAX_NUM)
			{
				/* setting success */
				ret_data[0] = 1;

				/* save NV data */
				prox_write_nvitem(NV_PROX_SENSE_A_I, level_a);
				level_b = prox_sens_a_b_table[level_a];
				prox_write_nvitem(NV_PROX_SENSE_B_I, level_b);

				/* Android_PROX_T04 s */
				g_prox_sense_a = prox_sensitivity_table[level_a];
				g_prox_sense_b = prox_sensitivity_table[level_b];
				/* Android_PROX_T04 e */
			}
    	    else
    	    {
				/* setting fail */
				ret_data[0] = 0;

    	        /* Android_PROX_T04 s */
				g_prox_sense_a = prox_sensitivity_table[PROX_SENS_A_DEFAULT];
				g_prox_sense_b = prox_sensitivity_table[PROX_SENS_B_DEFAULT];
				/* Android_PROX_T04 e */
    		}
			if (copy_to_user(argp, &ret_data[0], sizeof(ret_data))) {
				PROX_DBG("[proximity]copy to user failed:%d\n",__LINE__);
				result = -EFAULT;
			}
			//prodrv_sns_i2c_write( HYS_REG, HYS_HYSC);
			/* Android_PROX_T04 s */
			//prodrv_sns_i2c_write( HYS_REG, g_prox_sense_a);
			prodrv_sns_i2c_write( HYS_REG, g_prox_sense_b);
			/* Android_PROX_T04 e */
/* Android_PROX_T03_e */
			//interrupt clearing
			prodrv_sns_i2c_write( CON_REG | INT_CLEAR/*CON_REG_INT_CLEAR*/, CON_REG_INIT);

			enable_irq(g_proxi_irq);
			break;
/* Android_PROX_T02_e */
		default:
			break;
	}
	return result;
}

static struct file_operations proximity_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = proximity_read,
	//  .write = proximity_write,
	.open = proximity_open,
	.unlocked_ioctl = proximity_ioctl,
	.release = proximity_release,
};
#endif

static int
prodrv_sns_suspend(struct platform_device *pdev, pm_message_t state)
{
	PROX_DBG("=====<PROXIMITY> prodrv_sns_suspend=====\n");
	
	atomic_set(&g_prox_state,PROX_STATE_SUSPEND);

	return 0;
}

static int
prodrv_sns_resume(struct platform_device *pdev)
{
	PROX_DBG("=====<PROXIMITY> prodrv_sns_resume=====\n");

	if(atomic_read(&g_prox_state) == PROX_STATE_WAIT_I2C) {
		schedule_work(&g_proxi_work_data);
	}
	else {
		atomic_set(&g_prox_state,PROX_STATE_WAKE);
	}
	return 0;
}

static int
prodrv_sns_probe(struct platform_device *pdev)
{
	struct sensor_data *data = NULL;
	static struct input_dev *input_data = NULL;
	int input_registered = 0;
	int  sysfs_created = 0;
	int rt;

	PROX_DBG("=====<PROXIMITY> prodrv_sns_probe=====\n");

	rt = gpio_request(g_prox_gpio, SENSOR_NAME);
	if (rt) {
		goto err;
	}

/* FUJITSU:2011-06-17 GPIO SETTINGS add start */
	if((system_rev & 0xFE )) {
		gpio_tlmm_config(GPIO_CFG(g_prox_gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	}
	else {
		gpio_tlmm_config(GPIO_CFG(g_prox_gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	}
/* FUJITSU:2011-06-17 GPIO SETTINGS add end */

	rt = prodrv_sns_ON();
	if(rt != TRUE) {
		rt = -EBUSY;
		goto err;
	}
	
	data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!data) {
		rt = -ENOMEM;
		goto err;
	}
	data->enabled = 0;
	data->delay = SENSOR_DEFAULT_DELAY;

	input_data = input_allocate_device();
	if (!input_data) {
		rt = -ENOMEM;
		goto err;
	}

	set_bit(EV_ABS, input_data->evbit);
	input_set_capability(input_data, EV_ABS, ABS_X);

	input_data->name = SENSOR_NAME;
	rt = input_register_device(input_data);
	if (rt) {
		goto err;
	}
	input_set_drvdata(input_data, data);	
	input_registered = 1;
	rt = sysfs_create_group(&input_data->dev.kobj,
			&sensor_attribute_group);
	if (rt) {
		goto err;
	}
	sysfs_created = 1;
	mutex_init(&data->mutex);
	this_data = input_data;
	input_report_abs(input_data, ABS_X, PROX_VAL_FAR);
	input_sync(input_data);	
	prodrv_sns_request_irqs();
	prodrv_sns_OFF();
	return 0;

err:
	if (data != NULL) {
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
prodrv_sns_remove(struct platform_device *pdev)
{
	struct sensor_data *data;

	PROX_DBG("=====<PROXIMITY> prodrv_sns_remove=====\n");
	
	if (this_data != NULL) {
		data = input_get_drvdata(this_data);
		sysfs_remove_group(&this_data->dev.kobj,
				&sensor_attribute_group);
		input_unregister_device(this_data);
		if (data != NULL) {
			kfree(data);
		}
	}

	return 0;
}

/*
 * Module init and exit
 */
static struct platform_driver sensor_driver = {
	.probe	  = prodrv_sns_probe,
	.remove	 = prodrv_sns_remove,
	.suspend	= prodrv_sns_suspend,
	.resume	 = prodrv_sns_resume,
	.driver = {
		.name   = SENSOR_NAME,
		.owner  = THIS_MODULE,
	},
};

static int prodrv_vreg_on(void)
{
	int rc;
	int ret = FALSE;
	pr_debug("%s\n", __func__);

	prox_vreg_l8 = vreg_get(NULL, PROX_VREG_NAME);

	if (!prox_vreg_l8) {
		pr_err("%s: VREG L10 get failed\n", __func__);
		return FALSE;
	}

	rc = vreg_set_level(prox_vreg_l8, 2600);
	if (rc) {
		pr_err("%s: VREG L10 set failed\n", __func__);
		goto l8_put;
	}

	rc = vreg_enable(prox_vreg_l8);
	if (rc) {
		pr_err("%s: VREG L10 enable failed\n", __func__);
		goto l8_put;
	}
	
	PROX_DBG("[PROX] VREG ON SUCCESS!!!\n");
	ret = TRUE;

	/* FUJITSU:2011-07-15 add start */
	return ret;
	/* FUJITSU:2011-07-15 add end */
	
l8_put:
    vreg_put(prox_vreg_l8);
    return ret;
}

static void prodrv_vreg_off(void)
{
	int rc;
	pr_debug("%s\n", __func__);

	if (IS_ERR(prox_vreg_l8)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(prox_vreg_l8));
		return;
	}
	rc = vreg_disable(prox_vreg_l8);
	if (rc)
		pr_err("%s: vreg_disable(ncp) failed (%d)\n", __func__, rc);
	vreg_put(prox_vreg_l8);

	PROX_DBG("[PROX] VREG OFF SUCCESS!!!\n");

	prox_vreg_l8 = NULL;
}

/* FUJITSU:2011-06-17 GPIO SETTINGS del start */
///*powerup the PMIC, configure GPIO,
//set GPIO to NO_PULL, make it as input pin.
//Give sleep of 10 millisecs, call registry init
//*/
/* FUJITSU:2011-06-17 GPIO SETTINGS del end */
int
prodrv_sns_ON(void)
{
	int ok_flag = TRUE;

	PROX_DBG("=====<PROXIMITY> prodrv_sns_ON=====\n");

	if(PROX_VREG_ON == prox_vreg_on_off)
	{
		return ok_flag;
	}
	
	prox_detect.proxy_val = PROX_VAL_NEAR;
	prox_detect.proxy_state = PROX_DETECT_NON;

	ok_flag = prodrv_vreg_on();
	if(!ok_flag)
	{
		return ok_flag;
	}
	
	/* FUJITSU:2011-06-17 GPIO SETTINGS del start */
	//gpio_direction_input(g_prox_gpio);
	//
	////added to set it to no pull as per spec
	//gpio_tlmm_config(GPIO_CFG(g_prox_gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	/* FUJITSU:2011-06-17 GPIO SETTINGS del end */

	msleep(SNS_STANDBY_TIME);

	ok_flag = prodrv_sns_reg_init();
	/* FUJITSU:2011-06-17 GPIO SETTINGS mod start */
	//if(ok_flag!=TRUE) {
	if(ok_flag) {
	/* FUJITSU:2011-06-17 GPIO SETTINGS mod end */
	/* FUJITSU:2011-06-17 GPIO SETTINGS del start */
	//	gpio_direction_output(g_prox_gpio, PROX_GPIO_OUT_HI);
	//}
	//else
	//{
	/* FUJITSU:2011-06-17 GPIO SETTINGS del end */
		prox_vreg_on_off = PROX_VREG_ON;
	}
	return ok_flag;
}

/*power-down the device
configure GPIO 37 to out*/
void
prodrv_sns_OFF(void)
{
	PROX_DBG("=====<PROXIMITY> prodrv_sns_OFF=====\n");

	if(PROX_VREG_OFF == prox_vreg_on_off)
	{
		return;
	}
	
	/* FUJITSU:2011-06-17 GPIO SETTINGS del start */	
	//gpio_direction_output(g_prox_gpio, PROX_GPIO_OUT_HI);
	//
	//gpio_tlmm_config(GPIO_CFG(g_prox_gpio, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_DISABLE);
	/* FUJITSU:2011-06-17 GPIO SETTINGS del end */	

	prodrv_sns_ssd();

	prodrv_vreg_off();

	prox_detect.proxy_val = PROX_VAL_NEAR;
	prox_detect.proxy_state = PROX_DETECT_NON;
	
	del_timer(&prox_timer);
	
	prox_vreg_on_off = PROX_VREG_OFF;
}

/*Initialize all registry values of the proximity
sensor as per specifications, use i2c write to write
register values*/
static int
prodrv_sns_reg_init(void)
{
	int		init_sts;
	init_sts = prodrv_sns_i2c_write( GAIN_REG, GAIN_LED );
	
	if(0 <= init_sts) {
		init_sts &= prodrv_sns_i2c_write( HYS_REG, g_prox_sense_b);
	}
	
	if(0 <= init_sts) {
		init_sts &= prodrv_sns_i2c_write( CYCLE_REG, CYCLE_OSC);
	}

	if(0 <= init_sts) {
		init_sts &= prodrv_sns_i2c_write( OPMOD_REG, OPMOD_SSD | OPMOD_VCON);
	}

	if(0 <= init_sts) {
		init_sts &= prodrv_sns_i2c_write( CON_REG, CON_REG_INIT);
	}
	
	if(0 > init_sts) {
		PROX_DBG("=====<PROXIMITY> prodrv_sns_reg_init FALSE=====\n");
		return FALSE;
	}
	return TRUE;
}

/*change the register values so that it is ok to
shutdown*/
static void
prodrv_sns_ssd(void)
{
	PROX_DBG("=====<PROXIMITY> prodrv_sns_ssd=====\n");

	prodrv_sns_i2c_write( OPMOD_REG, OPMOD_VCON);
	prodrv_sns_i2c_write( CON_REG, CON_OCON | CON_OCON_1);
}

/*does i2c write to the proximity sensor*/
static int
prodrv_sns_i2c_write(unsigned char reg, unsigned char data)
{
    PROX_I2C_INFO  prox_i2c_info_data;
    bool retval;
	u_int8_t buf;
	int ret = TRUE;

    buf = data;
    prox_i2c_info_data.slave_addr = PROX_SLAVE_ADDRESS;
    prox_i2c_info_data.reg        = reg;
    prox_i2c_info_data.data       = &buf;
    prox_i2c_info_data.len        = 1;
    
    retval = prox_i2c_write(&prox_i2c_info_data);
    if( retval == 0 ) {
        PROX_DBG("[proximity] prodrv_sns_i2c_write falied! in %s()\n", __func__ );
        ret = FALSE;
    }    
    
	return ret;
}

/*does i2c read from the proximity sensor*/
static int
prodrv_sns_i2c_read(unsigned short reg, unsigned char *data, uint32_t len)
{
    PROX_I2C_INFO  prox_i2c_info_data;
    bool retval;
	int ret = TRUE;

    prox_i2c_info_data.slave_addr = PROX_SLAVE_ADDRESS;
    prox_i2c_info_data.reg        = reg;
    prox_i2c_info_data.data       = data;
    prox_i2c_info_data.len        = len;

    retval = prox_i2c_read(&prox_i2c_info_data);
    if( retval == 0 ) {
        PROX_DBG("[proximity] prodrv_sns_i2c_write falied! in %s()\n", __func__ );
        ret = FALSE;
    }    
    
	return ret;
}

#if PROX_TEST_MODE
/*
 * Set up the cdev structure for a device.
 */
static void
proximity_setup_cdev(struct cdev *dev, int minor,
		struct file_operations *fops)
{
	int err, devno = MKDEV(proximity_major, minor);

	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	dev->ops = fops;
	err = cdev_add (dev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		PROX_DBG (KERN_NOTICE "Error %d adding rfs%d", err, minor);

	if (IS_ERR(device_create(proximity_class, NULL, devno, NULL, SENSOR_NAME)))
		PROX_DBG(KERN_ERR "can't create device\n");
}
#endif

void prodrv_sns_report_abs(unsigned long data)
{
	struct prox_detect_state *state = (struct prox_detect_state *)data;
 
	if(state->proxy_val == PROX_VAL_NEAR) {
		state->proxy_state = PROX_DETECTED;
		PROX_DBG("=====<PROXIMITY> proxi_work_bh NEAR 2 delay:%d=====\n", prox_wait);
		input_report_abs(this_data, ABS_X, state->proxy_val);
		input_sync(this_data);	
		atomic_set(&g_prox_val,state->proxy_val);
	}
}

//bottom half of interrupt handling
static void
prodrv_sns_work_bh(struct work_struct *work)
{
	int init_sts;

	unsigned char data1[READ_SIZE];

	PROX_DBG("=====<PROXIMITY> proxi_work_bh=====\n");
	
	if(atomic_read(&g_prox_state) != PROX_STATE_SUSPEND) {
		prodrv_sns_i2c_read(PROX_REG, data1, READ_SIZE);

		if(data1[I2C_READ_POS] & PROX_REG_NEAR) {
			//changing as per new spec change
			//check if it is 01 or 10 as per spec
			init_sts = prodrv_sns_i2c_write( HYS_REG, g_prox_sense_a);
			PROX_DBG("=====<PROXIMITY> proxi_work_bh NEAR 1=====\n");

			// save state
			prox_detect.proxy_val = PROX_VAL_NEAR;
			prox_detect.proxy_state = PROX_DETECTING;
	
			// start timer
			init_timer(&prox_timer);
			prox_timer.function = prodrv_sns_report_abs;
			prox_timer.expires = jiffies + msecs_to_jiffies(prox_wait); //ms
			prox_timer.data = (unsigned long)&prox_detect;
			add_timer(&prox_timer);
		}
		else {
			init_sts = prodrv_sns_i2c_write( HYS_REG, g_prox_sense_b);
			PROX_DBG("=====<PROXIMITY> proxi_work_bh FAR 1=====\n");

			// save state
			prox_detect.proxy_val = PROX_VAL_FAR;
			
			if(prox_detect.proxy_state == PROX_DETECTING) {
				// If something leaves from the sensor within "prox_wait" ms,
				// the proximity driver cancel the timer. And it do NOT inform detecting.
				del_timer(&prox_timer);
				PROX_DBG("=====<PROXIMITY> proxi_work_bh FAR TimerDelete=====\n");
			}
			else {
				PROX_DBG("=====<PROXIMITY> proxi_work_bh FAR Notice=====\n");
				input_report_abs(this_data, ABS_X, prox_detect.proxy_val);
				input_sync(this_data);
				atomic_set(&g_prox_val, prox_detect.proxy_val);
			}

			prox_detect.proxy_state = PROX_DETECT_NON;
		}

		init_sts = prodrv_sns_i2c_write( CON_REG, CON_OCON | CON_OCON_1);

		//interrupt clearing
		init_sts = prodrv_sns_i2c_write( CON_REG | INT_CLEAR, CON_REG_INIT);

		enable_irq(g_proxi_irq);
	}
  	else {
		PROX_DBG("=====<PROXIMITY> proxi_work_bh I2C Wait=====\n");
		atomic_set(&g_prox_state,PROX_STATE_WAIT_I2C);
  	}
}

//irq handler
static irqreturn_t
prodrv_sns_irq_handler(int irq, void *p)
{
	disable_irq_nosync(g_proxi_irq);
	schedule_work(&g_proxi_work_data);
	return IRQ_HANDLED;
}

//irq request
static int
prodrv_sns_request_irqs(void)
{
	int err;
	unsigned long req_flags = IRQF_TRIGGER_FALLING;

	err = g_proxi_irq = gpio_to_irq(g_prox_gpio);
	err = request_irq(g_proxi_irq, prodrv_sns_irq_handler,
			req_flags, SENSOR_NAME, NULL);
	if (err) {
		return err;
	}
	PROX_DBG("=====<PROXIMITY> %s SUCCESS!!!=====\n", __func__);
	set_irq_wake(g_proxi_irq, WAKE_ON);
	return 0;
}

#if PROX_TEST_MODE
int prox_write_nvitem(unsigned int id, unsigned char data)
{
	int rc;

	/* write NV */
	rc = msm_proc_comm(PCOM_OEM_012, (unsigned *)&id, (unsigned *)&data);
	if( rc )
	{
		printk("PROX NV write error %d %d\n",rc,id);
	}
	return rc;
}
#endif

int prox_read_nvitem(unsigned int id)
{
	int rc;
	unsigned int prox_read_data;
        
	/* read NV*/
	rc = msm_proc_comm(PCOM_OEM_011, (unsigned *)&id, &prox_read_data);
	if(rc)
	{
		printk("PROX NV read error %d\n",rc);
	    return -1;
	}
    else
    {
        return prox_read_data;
    }
}

#if PROX_TEST_MODE
#define MAX_PROXIMITY_DEV 1
static struct cdev proximityDevs[MAX_PROXIMITY_DEV];
#endif

/*does sensor initialization*/
static int __init prodrv_sns_init(void)
{
	signed char level_a;
	signed char level_b;

#if PROX_TEST_MODE
	int result = 0;
	dev_t dev = MKDEV(proximity_major, 0);

	proximity_class = class_create(THIS_MODULE, PROX_CLASS_NAME);
	if(IS_ERR(proximity_class))
	{
		return PTR_ERR(proximity_class);
	}

	/* Figure out our device number. */
	if (proximity_major)
	{
		result = register_chrdev_region(dev, 2, PROX_CLASS_NAME);
	}
	else
	{
		result = alloc_chrdev_region(&dev, 0, 2, PROX_CLASS_NAME);
		proximity_major = MAJOR(dev);
	}
	if (result < 0) {
		PROX_DBG(KERN_WARNING "proximity: unable to get major %d\n", proximity_major);

		return result;
	}
	if (proximity_major == 0)
		proximity_major = result;

	/* Now set up two cdevis. */
	proximity_setup_cdev(proximityDevs, 0, &proximity_fops);
#endif

	PROX_DBG("=====<PROXIMITY> prodrv_sns_init=====\n");
	
	if((system_rev & 0xFE )) {
		g_prox_gpio = PROX_GPIO_2;
	}
	else {
		g_prox_gpio = PROX_GPIO_1;
	}
	PROX_DBG("=====<PROXIMITY> GPIO SELECT system_rev = %d, g_prox_gpio = %d\n", system_rev, g_prox_gpio);

	if(!prox_i2c_init()){
		return -1;
	}
	
	INIT_WORK(&g_proxi_work_data, prodrv_sns_work_bh);
	atomic_set(&g_prox_state,PROX_STATE_WAKE);
	atomic_set(&g_prox_val,PROX_VAL_FAR);
	
	sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
	if (IS_ERR(sensor_pdev)) {
		return -1;
	}
	g_prox_sense_a = prox_sensitivity_table[PROX_SENS_A_DEFAULT];
	g_prox_sense_b = prox_sensitivity_table[PROX_SENS_B_DEFAULT];
	
	level_a = prox_read_nvitem(NV_PROX_SENSE_A_I);
	if((level_a >= 0) && (level_a <= PROX_SENS_MAX_NUM)) {
		level_b = prox_read_nvitem(NV_PROX_SENSE_B_I);
		if((level_b >= 0 ) && (level_b <= PROX_SENS_MAX_NUM) && (level_a < level_b)) {
			/* NV value access success. */
			g_prox_sense_a = prox_sensitivity_table[level_a];
			g_prox_sense_b = prox_sensitivity_table[level_b];
		}
	}

	PROX_DBG("=====<PROXIMITY> sensor a:%d=====\n", g_prox_sense_a);
	PROX_DBG("=====<PROXIMITY> sensor b:%d=====\n", g_prox_sense_b);
	
	return platform_driver_register(&sensor_driver);
}
module_init(prodrv_sns_init);

/*does sensor shutdown*/
static void __exit prodrv_sns_exit(void)
{
	PROX_DBG("=====<PROXIMITY> prodrv_sns_exit=====\n");
	
	prodrv_sns_OFF();
	platform_driver_unregister(&sensor_driver);
	platform_device_unregister(sensor_pdev);
}
module_exit(prodrv_sns_exit);

MODULE_AUTHOR("Fujitsu");
MODULE_LICENSE( "GPL" );
MODULE_VERSION("1.0.0");
