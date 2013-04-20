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
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/kthread.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <mach/gpio.h>
#include "proximity.h"
#include <asm/uaccess.h>
#include <asm/page.h>
#include <linux/i2c.h>
#include <mach/pmic.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include "prox_gpio_hw.h"

/*===========================================================================
  MACROS
  ===========================================================================*/
/* for debugging */
#define DEBUG 0
#define DEBUG_SUPPORT_TEST 0	// tuning
#define DEBUG_MONITOR 0

#if DEBUG
#define PROX_DBG(x...)  printk(KERN_DEBUG x)
#else
#define PROX_DBG(x...)  
#endif

#if DEBUG_MONITOR
#define PROX_DBG_MONITOR(x...)  printk(x)
#else
#define PROX_DBG_MONITOR(x...)  
#endif

#define PROX_PRINT(x...) printk(x)

/* for device interface initialize */
#define SENSOR_NAME				"proximity"
#define SENSOR_DEFAULT_DELAY	(200)   /* 200 ms */
#define SENSOR_MAX_DELAY		(2000)  /* 2000 ms */
#define ABS_STATUS				(ABS_BRAKE)
#define ABS_WAKE				(ABS_MISC)
#define ABS_CONTROL_REPORT		(ABS_THROTTLE)
/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
#define PROX_GPIO_REV0			22		/* f11eif:GPIO_REV03_20110322 */
#define PROX_GPIO_REV1			112		
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */
#define WAKE_ON					1

//
/* for I2C */
#define PROX_I2C_ADAPTER		4			/* f11eif:1-X QUPB */

#define SNS_STANDBY_TIME		10

#define I2C_WRITE_BUFSIZE		64
#define READ_SIZE				2

#define I2C_TRANS_WRITE			1
#define I2C_TARNS_READ			2

#define I2C_READ_POS			1

#define PROX_SLAVE_ADDRESS	  	(0x38)		/* f11eif:BH1772GLC	*/

#define GPIOF_ENABLE_WAKE		0x40000000

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
#define	CYCLE_256MS				0x2C	/*K01_PROX_001*/

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
/* FUJITSU:2011-05-10 ALS/PS CORRECT start */
#define PROX_SENS_H				87		// default 104
#define PROX_SENS_L				66		// default 76
/* FUJITSU:2011-05-10 ALS/PS CORRECT end */
#define PROX_TEST_SENS_NUM		32

/* for sensor delay */
#define PROX_DETECT_NON			0
#define PROX_DETECTING			1
#define PROX_DETECTED			2



/*--------------------------------------------------------------------
I2C register (BH1772GLC)
--------------------------------------------------------------------*/
//#define I2C_CHANNEL					0
//#define I2C_WRITE_BIT					0x00
//#define I2C_READ_BIT					0x01
//#define I2C_PROX_ADDRESS				0x70

#define I2C_PROX_ALS_CONTROL			0x40
#define I2C_PROX_PS_CONTROL				0x41
#define I2C_PROX_I_LED					0x42
//
#define I2C_PROX_ALS_PS_MEAS			0x44
#define I2C_PROX_PS_MEAS_RATE			0x45
#define I2C_PROX_ALS_MEAS_RATE		0x46
//
//
#define I2C_PROX_ALS_DATA_0				0x4C
#define I2C_PROX_ALS_DATA_1				0x4D
#define I2C_PROX_ALS_PS_STATUS		0x4E
#define I2C_PROX_PS_DATA_LED1			0x4F
//
//
#define I2C_PROX_INTERRUPT				0x52
#define I2C_PROX_PS_TH_H_LED1			0x53
//
//
//#define I2C_PROX_ALS_TH_UP_0			0x56
//#define I2C_PROX_ALS_TH_UP_1			0x57
//#define I2C_PROX_ALS_TH_LOW_0			0x58
//#define I2C_PROX_ALS_TH_LOW_1			0x59
#define I2C_PROX_ALS_SENSITIVITY		0x5A
#define I2C_PROX_PERSISTENCE			0x5B
#define I2C_PROX_PS_TH_L_LED1			0x5C
//
//
/*--------------------------------------------------------------------
I2C status (BH1772GLC)
--------------------------------------------------------------------*/
// I2C_PROX_ALS_CONTROL MODE
#define ALS_MODE_STANDBY				0x00
#define ALS_MODE_FORCED					0x02
#define ALS_MODE_STANDALONE				0x03

// I2C_PROX_PS_CONTROL MODE
#define PS_MODE_STANDBY					0x00
#define PS_MODE_FORCED					0x02
#define PS_MODE_STANDALONE				0x03

// I2C_PROX_I_LED LED current
#define LED_CURRENT_5mA					0x18
#define LED_CURRENT_10mA				0x19
#define LED_CURRENT_20mA				0x1A
#define LED_CURRENT_50mA				0x1B
#define LED_CURRENT_100mA				0x1C
#define LED_CURRENT_150mA				0x1D
#define LED_CURRENT_200mA				0x1E

// I2C_PROX_ALS_PS_MEAS
#define PS_TRIGGER						0x01
#define ALS_TRIGGER						0x02

// I2C_PROX_ALS_MEAS_RATE
#define ALS_RATE_ENABLE					0x00
#define ALS_RATE_DISABLE				0x80

#define ALS_MEAS_RATE_100ms				0x00
#define ALS_MEAS_RATE_200ms				0x01
#define ALS_MEAS_RATE_500ms				0x02
#define ALS_MEAS_RATE_1000ms			0x03
#define ALS_MEAS_RATE_2000ms			0x04

// I2C_PROX_PS_MEAS_RATE
#define PS_MEAS_RATE_10ms				0x00
#define PS_MEAS_RATE_20ms				0x01
#define PS_MEAS_RATE_30ms				0x02
#define PS_MEAS_RATE_50ms				0x03
#define PS_MEAS_RATE_70ms				0x04
#define PS_MEAS_RATE_100ms				0x05
#define PS_MEAS_RATE_200ms				0x06
#define PS_MEAS_RATE_500ms				0x07
#define PS_MEAS_RATE_1000ms				0x08
#define PS_MEAS_RATE_2000ms				0x09

// I2C_PROX_INTERRUPT
#define INT_MODE_NONE					0x00
#define INT_MODE_PS						0x01
#define INT_MODE_ALS					0x02
#define INT_MODE_ALL					0x03

#define INT_POL_ACTLOW					0x00
#define INT_POL_INACTLOW				0x04

#define INT_OUTMODE_LATCH				0x00
#define INT_OUTMODE_NOLATCH				0x08

#define	INT_PSHIST_HI					0x00
#define INT_PSHIST_HILOW				0x10

#define	EIFFEL_INTERRUPT_SET			\
	(INT_PSHIST_HILOW | INT_OUTMODE_NOLATCH | INT_POL_ACTLOW | INT_MODE_PS)
#define	EIFFEL_INTERRUPT_RESET			\
	(INT_PSHIST_HILOW | INT_OUTMODE_NOLATCH | INT_POL_ACTLOW | INT_MODE_NONE)
#define	EIFFEL_INT_SET_F2N				\
	(INT_PSHIST_HILOW | INT_OUTMODE_NOLATCH | INT_POL_ACTLOW | INT_MODE_PS)
#define	EIFFEL_INT_SET_N2F				\
	(INT_PSHIST_HILOW | INT_OUTMODE_NOLATCH | INT_POL_INACTLOW | INT_MODE_PS)
#define	EIFFEL_INT_RESET_F2N			\
	(INT_PSHIST_HILOW | INT_OUTMODE_NOLATCH | INT_POL_ACTLOW | INT_MODE_NONE)
#define	EIFFEL_INT_RESET_N2F			\
	(INT_PSHIST_HILOW | INT_OUTMODE_NOLATCH | INT_POL_INACTLOW | INT_MODE_NONE)

// I2C_PROX_ALS_PS_STATUS
#define	STS_ALSINT_BIT					0x80
#define	STS_ALSDATA_BIT					0x40
#define	STS_PSINT_BIT					0x02
#define	STS_PSDATA_BIT					0x01

/* FUJITSU:2011-05-25 panel color read  start */
#define PROX_NVID_BASE		10031
#define PROX_NVDATA_BASE	0xf8;
/* FUJITSU:2011-05-25 panel color read  end */


/*===========================================================================

  GLOBAL VARIABLES

  ===========================================================================*/
struct i2c_adapter *i2c_proximity;		// i2c_adp desc.
int i2c_proximity_ready = 0;	// 1:ready , 0:not ready

static atomic_t g_prox_state;
static atomic_t g_wake;
static atomic_t g_suspend_off;

static unsigned int g_proxi_irq;
static struct work_struct g_proxi_work_data;

static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;

/* FUJITSU:2011-05-10 ALS/PS CORRECT start */
static unsigned char 	g_i_led;			// sensor register value
static unsigned char 	g_als_sens;			// sensor register value
static unsigned char 	g_ps_hi;			// sensor register value
static unsigned char 	g_ps_low;			// sensor register value
static int				g_als_correct;		// ALS sensitive lux x correct
/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
static int				g_gpio_number;		// sensor gpio num
static int				g_gpio_pullup;		// sensor gpio pullup
static int				g_gpio_strength;	// sensor gpio device strength
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

/* FUJITSU:2011-05-25 panel color read  start */
static unsigned g_prox_nvid;
static unsigned g_prox_nvdata;
/* FUJITSU:2011-05-25 panel color read end    */

struct prox_correct_t {
	unsigned char	index;
	unsigned char	i_led;
	unsigned char	ps_hi;
	unsigned char	ps_low;
	unsigned char	als_sens;
	int				als_correct;
};

struct prox_correct_t	prox_correct[3] = {
/* FUJITSU:2011-05-26 ALS sens default level change start */
	{0x00, LED_CURRENT_20mA, PROX_SENS_H, PROX_SENS_L, 0x6a, 5},// B(2x5)
	{0x01, LED_CURRENT_20mA, PROX_SENS_H, PROX_SENS_L, 0x42, 2},// P(1.25x2)
	{0xff, LED_CURRENT_20mA, PROX_SENS_H, PROX_SENS_L, 0x6a, 5} // o(2x5)
/* FUJITSU:2011-05-26 ALS sens default level change end */
};
/* FUJITSU:2011-05-10 ALS/PS CORRECT end */

/*===========================================================================

  STRUCTS

  ===========================================================================*/
struct sensor_data {
	struct mutex mutex;
	int enabled;
	int delay;
};

struct prox_detect_state {
	int           proxy_lux_save;
	int           proxy_distreg_save;
	char          proxy_distance_save[8];
	unsigned char proxy_val;
	unsigned char proxy_state;
	unsigned char proxy_intr_save;
};
static struct prox_detect_state prox_detect;


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
int prodrv_sns_als_data_read(int *lux);
void prodrv_sns_report_distance(void);
static int prodrv_sns_convert_data(int data, char *val);
struct i2c_adapter * prodev_i2c_search_adapter(void);
static void prodrv_sns_work_bh(struct work_struct *work);
static void prodrv_sns_work_bh_sub(void);


/* FUJITSU:2011-05-25 panel color read start  */
extern int msm_prodrv_read_nvitem(unsigned *id, unsigned *data);
/* FUJITSU:2011-05-25 panel color read end    */

/*===========================================================================

  LOCAL FUNCTIONS

  ===========================================================================*/
/*    Sysfs Interface  FUNCTIONS                                             */

/* ----------------------------------------------------------------- */
/*
 * sysfs: delay interface (proximity delay(output))
 */
static ssize_t
prodrv_sysfs_delay_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int delay;
	
	PROX_DBG("[PROXIMITY] prodrv_sysfs_delay_show \n");
	
	if(NULL == data) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_delay_show NULL \n");
		return 0;
	}
	
	mutex_lock(&data->mutex);

	delay = data->delay;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", delay);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: delay interface (proximity delay(input))
 */
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

	PROX_DBG("[PROXIMITY] prodrv_sysfs_delay_store \n");

	if(NULL == data) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_delay_store NULL \n");
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

/* ----------------------------------------------------------------- */
/*
 * sysfs: enable interface (proximity enable(output))
 */
static ssize_t
prodrv_sysfs_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int enabled;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_show \n");
	
	if(NULL == data) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_show NULL \n");
		return 0;
	}
	
	mutex_lock(&data->mutex);

	enabled = data->enabled;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", enabled);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: enable interface (proximity enable(input))
 */
static ssize_t
prodrv_sysfs_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int value = simple_strtoul(buf, NULL, 10);
	int		init_sts;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_store \n");
	
	if ((value != 0) && (value != 1)) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_store return err \n");
		return count;
	}

	PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_store value:%d \n", value);
	if(value) {
		/*if it has been set to 1, call is incoming*/
		atomic_set(&g_wake,true);
		PROX_DBG_MONITOR("[PROXIMITY] Sensor Enable set\n");
/* FUJITSU:2011-07-13 INTR change start */
		prox_detect.proxy_intr_save = EIFFEL_INTERRUPT_SET;
		init_sts = prodrv_sns_i2c_write(I2C_PROX_INTERRUPT,
				EIFFEL_INTERRUPT_SET);
/* FUJITSU:2011-07-13 INTR change end */
	}
	else {
		atomic_set(&g_wake,false);
		PROX_DBG_MONITOR("[PROXIMITY] Sensor Enable reset\n");
/* FUJITSU:2011-07-13 INTR change start */
		prox_detect.proxy_intr_save = EIFFEL_INTERRUPT_RESET;
		init_sts = prodrv_sns_i2c_write(I2C_PROX_INTERRUPT,
				EIFFEL_INTERRUPT_RESET);
	}
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_store I2C_PROX_INTERRUPT"
				" FALSE(%02x) \n", prox_detect.proxy_intr_save);
	}
/* FUJITSU:2011-07-13 INTR change end */
		
	return count;
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: wake interface (wake(input))
 */
static ssize_t
prodrv_sysfs_wake_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	static int cnt = 1;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_wake_store \n");

	input_report_abs(input_data, ABS_WAKE, cnt++);

	return count;
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: data interface (distance data(output))
 */
static ssize_t
prodrv_sysfs_data_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int x;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_data_show \n");
	
	spin_lock_irqsave(&input_data->event_lock, flags);

	x = input_data->abs[ABS_X];

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	//  report distance
	prodrv_sns_report_distance();

	PROX_DBG("[PROXIMITY] prodrv_sysfs_data_show(distance=%s) \n"
										,prox_detect.proxy_distance_save);

	return sprintf(buf, "%s\n",prox_detect.proxy_distance_save);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: dfdata interface (proximity flags(output))
 */
static ssize_t
prodrv_sysfs_dfdata_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int x;
#if DEBUG_MONITOR
	int	lux;
#endif

	PROX_DBG("[PROXIMITY] prodrv_sysfs_dfdata_show \n");

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
/*	prodrv_sns_work_bh_sub(); */
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */
	
	spin_lock_irqsave(&input_data->event_lock, flags);

	x = input_data->abs[ABS_X];

	spin_unlock_irqrestore(&input_data->event_lock, flags);

#if DEBUG_MONITOR
	// debug mode (for sensor HW test)
	prodrv_sns_als_data_read(&lux);
	prodrv_sns_report_distance();
	PROX_DBG_MONITOR("[PROXIMITY] Sensor status (Lux=%d distance=%s) \n",
										lux,prox_detect.proxy_distance_save);
#endif	

	PROX_DBG("[PROXIMITY] prodrv_sysfs_dfdata_show:%d \n",x);
	
	return sprintf(buf, "%d\n", x);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: drdata interface (prox-flags and dist-register(output))
 */
static ssize_t
prodrv_sysfs_drdata_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int x;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_drdata_show \n");

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	prodrv_sns_work_bh_sub();
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	spin_lock_irqsave(&input_data->event_lock, flags);

	x = input_data->abs[ABS_X];

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	//  report ps sensor register
	prodrv_sns_report_distance();

	PROX_DBG("[PROXIMITY] prodrv_sysfs_drdata_show:%d \n"
									,prox_detect.proxy_distreg_save);
	
	return sprintf(buf, "%02x%02x\n", x,prox_detect.proxy_distreg_save);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: status interface (status(output))
 */
static ssize_t
prodrv_sysfs_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int status;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_status_show \n");
	
	spin_lock_irqsave(&input_data->event_lock, flags);

	status = input_data->abs[ABS_STATUS];

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	PROX_DBG("[PROXIMITY] prodrv_sysfs_status_show:%d \n",status);
	
	return sprintf(buf, "%d\n", status);
}


/* ----------------------------------------------------------------- */
/*
 * sysfs: ktdata interface (koutei test data(output))
 */
static ssize_t
prodrv_sysfs_ktdata_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int x;
	int	lux;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_show \n");

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	prodrv_sns_work_bh_sub();
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	spin_lock_irqsave(&input_data->event_lock, flags);

	x = input_data->abs[ABS_X];

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	// debug mode (for sensor HW test)
	prodrv_sns_als_data_read(&lux);
	prodrv_sns_report_distance();
	PROX_DBG("[PROXIMITY] Sensor (Lux=%d dist=%s reg=%d I_LED=%02x ALS=%02x) \n",
		lux,prox_detect.proxy_distance_save,prox_detect.proxy_distreg_save,
		g_i_led,g_als_sens);

	PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_show:%d \n",x);
	
	return sprintf(buf, "%d %s %d %d %02x %02x\n", x,
		prox_detect.proxy_distance_save,prox_detect.proxy_distreg_save,
		lux, g_i_led, g_als_sens);
}

/* FUJITSU:2011-06-13 [CTS] start */
#if DEBUG_SUPPORT_TEST
/* ----------------------------------------------------------------- */
/*
 * sysfs: ktdata interface (koutei test data(input))
 */
static ssize_t
prodrv_sysfs_ktdata_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int value = simple_strtoul(buf, NULL, 10);
	unsigned char value_led;
	unsigned char value_sens;
	
	value_led  = (unsigned char)(value & 0x00ff);
	value_sens = (unsigned char)((value & 0xff00) >> 8);

	PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_store \n");
	
	if ((value_led < 0x18) || (value_led > 0x1e)) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_store return err \n");
		return count;
	}

	PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_store val:%d "
			"(led=%02x,sens=%02x)\n",
			 value,value_led,value_sens);

	g_i_led = value_led;
	g_als_sens = value_sens;
		
	return count;
}
#endif
/* FUJITSU:2011-06-13 [CTS] end */

/*--------------------------------------------------------------------
  device attribute (sysfs-interface) setting
  ------------------------------------------------------------------*/
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		prodrv_sysfs_delay_show, prodrv_sysfs_delay_store);
static DEVICE_ATTR(enable, S_IWUGO|S_IRUGO|S_IWUSR|S_IWGRP,
		prodrv_sysfs_enable_show, prodrv_sysfs_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
		NULL, prodrv_sysfs_wake_store);
/* FUJITSU:2011-06-13 [CTS] start */
static DEVICE_ATTR(data,   S_IRUGO, prodrv_sysfs_data_show,   NULL);
static DEVICE_ATTR(status, S_IRUGO, prodrv_sysfs_status_show, NULL);
static DEVICE_ATTR(dfdata, S_IRUGO, prodrv_sysfs_dfdata_show, NULL);
static DEVICE_ATTR(drdata, S_IRUGO, prodrv_sysfs_drdata_show, NULL);

#if DEBUG_SUPPORT_TEST
static DEVICE_ATTR(ktdata, S_IWUGO|S_IRUGO|S_IWUSR|S_IWGRP,
		prodrv_sysfs_ktdata_show, prodrv_sysfs_ktdata_store);
#else
static DEVICE_ATTR(ktdata, S_IRUGO, prodrv_sysfs_ktdata_show, NULL);
#endif
/* FUJITSU:2011-06-13 [CTS] end */

static struct attribute *sensor_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	&dev_attr_status.attr,
	&dev_attr_dfdata.attr,
	&dev_attr_drdata.attr,
	&dev_attr_ktdata.attr,
	NULL
};

static struct attribute_group sensor_attribute_group = {
	.attrs = sensor_attributes
};


/* ----------------------------------------------------------------- */
/*
 * suspend interface
 */
static int
prodrv_sns_suspend(struct platform_device *pdev, pm_message_t state)
{

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	if(atomic_read(&g_wake)) {

/* FUJITSU:2011-06-13 forced SUSPEND mode start */
		atomic_set(&g_suspend_off,true);
		atomic_set(&g_prox_state,PROX_STATE_SUSPEND);
		/**/
		if( prox_detect.proxy_val == PROX_VAL_NEAR ) {
			PROX_DBG("[PROXIMITY] prodrv_sns_suspend WAKE_ON / INTR-mode\n");
			PROX_PRINT(KERN_DEBUG "proximity: suspend(I)\n");
		}
		else {
			PROX_DBG("[PROXIMITY] prodrv_sns_suspend WAKE_ON "
					"FAR(forced INTR)\n");
			PROX_PRINT(KERN_DEBUG "proximity: suspend(I1)\n");
		}
		return 0;
/* FUJITSU:2011-06-13 forced SUSPEND mode end */
	} 
	else {
		PROX_DBG("[PROXIMITY] prodrv_sns_suspend WAKE_OFF (sleep)\n");
		PROX_PRINT(KERN_DEBUG "proximity: suspend(S2)\n");
	}
	// normal suspend mode (not interrupt)
	prodrv_sns_OFF();

/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	atomic_set(&g_prox_state,PROX_STATE_SUSPEND);

	return 0;
}


/* ----------------------------------------------------------------- */
/*
 * resume interface
 */
static int
prodrv_sns_resume(struct platform_device *pdev)
{
	int sts;

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	/* implement resume of the sensor */
	if(atomic_read(&g_suspend_off)) {
		// intr-mode

		atomic_set(&g_suspend_off,false);

/* FUJITSU:2011-06-15 no-irq suspend status change start */
		if(atomic_read(&g_prox_state) == PROX_STATE_SUSPEND) {
			PROX_PRINT(KERN_DEBUG "proximity: resume(Iw)\n");
			atomic_set(&g_prox_state,PROX_STATE_WAKE);
		} else {
			PROX_PRINT(KERN_DEBUG "proximity: resume(I)\n");
		}
/* FUJITSU:2011-06-15 no-irq suspend status change end */
	}
	else {
		// normal-suspend mode
		PROX_PRINT(KERN_DEBUG "proximity: resume(S)\n");

		sts = prodrv_sns_ON();
		atomic_set(&g_prox_state,PROX_STATE_WAKE);
		if(sts != true) {
			PROX_PRINT(KERN_ERR "proximity: resume prodrv_sns_ON error\n");
			return -EBUSY;
		}
	}
	i2c_proximity_ready = 1;		// I2C proximity ALS access ready
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	return 0;
}

/* ----------------------------------------------------------------- */
/*
 * probe interface
 */
static int
prodrv_sns_probe(struct platform_device *pdev)
{
	struct sensor_data *data = NULL;
	static struct input_dev *input_data = NULL;
	int input_registered = 0;
	int  sysfs_created = 0;
	int rt;
/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	int sts;
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */
/* FUJITSU:2011-05-26 ALS/PS CORRECT replace start */
	int idx;
	unsigned char nvindex = 0xff;
/* FUJITSU:2011-05-26 ALS/PS CORRECT replace end */

	PROX_DBG("[PROXIMITY] prodrv_sns_probe \n");

/* FUJITSU:2011-05-26 ALS/PS CORRECT replace start */
	// body color read
	g_prox_nvid = PROX_NVID_BASE;
	g_prox_nvdata = PROX_NVDATA_BASE;
	rt = msm_prodrv_read_nvitem(&g_prox_nvid, &g_prox_nvdata);
	if(rt == 0 )
	{
		/* success (nvid=0x08070605, nvdata=0x04030201) */
		nvindex = (unsigned char)((g_prox_nvid >> 24) & 0x000000ff );
		PROX_DBG("[PROXIMITY] prodrv_color_info : nv read success(%d)\n"
			,nvindex);
		PROX_DBG("[PROXIMITY] nv dump %08x %08x\n"
									,g_prox_nvid,g_prox_nvdata);
	}
	else
	{
		PROX_PRINT(KERN_ERR "proximity: probe NV read error\n");
	}

#if DEBUG_SUPPORT_TEST
	PROX_DBG("[PROXIMITY] prodrv_sns_init tune mode\n");
	nvindex = 0xff;	// default
#endif

	// select of housing
	for(idx=0;prox_correct[idx].index != 0xff;idx++) {
		if(prox_correct[idx].index == nvindex) {
			break;
		}
	}

	// register and correction initialize
	g_i_led = prox_correct[idx].i_led;
	g_ps_hi = prox_correct[idx].ps_hi;
	g_ps_low = prox_correct[idx].ps_low;
	g_als_sens = prox_correct[idx].als_sens;
	g_als_correct = prox_correct[idx].als_correct;
	PROX_DBG("[PROXIMITY] prodrv_sns_init global"
			"(led=%d,ps_hi=%d,ps_low=%d,als_sen=%d,als_corr=%d\n"
			,g_i_led,g_ps_hi,g_ps_low,g_als_sens,g_als_correct);

	PROX_PRINT(KERN_INFO "proximity: probe(%d)\n",nvindex);

/* FUJITSU:2011-05-26 ALS/PS CORRECT replace end */

	// GPIO initialize
/* FUJITSU:2011-05-26 checker start */
	rt = gpio_request(g_gpio_number, SENSOR_NAME);
	if (rt) {
		/* gpio error */
		PROX_DBG("[PROXIMITY] prodrv_sns_probe: gpio_request error \n");
		goto err;
	}
/* FUJITSU:2011-05-26 checker end */

	// I2C initialize
/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	sts = prodrv_sns_ON();
	if(sts != true) {
		rt = -EBUSY;
		goto err;
	}
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	// driver management memory allocate
	data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!data) {
		rt = -ENOMEM;
		goto err;
	}
	data->enabled = 0;
	data->delay = SENSOR_DEFAULT_DELAY;

	// proximity device allocate 
	input_data = input_allocate_device();
	if (!input_data) {
		rt = -ENOMEM;
		goto err;
	}

	set_bit(EV_ABS, input_data->evbit);
	input_set_capability(input_data, EV_ABS, ABS_X);

	input_data->name = SENSOR_NAME;

	// proximity device register
	rt = input_register_device(input_data);
	if (rt) {
		goto err;
	}
	input_registered = 1;

	// data store device member
	input_set_drvdata(input_data,data);

	// proximity sysfs initialize
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

	// proximity interrupt initialize
	prodrv_sns_request_irqs();

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
	PROX_PRINT(KERN_ERR "proximity: probe error (%d)\n",rt);

	return rt;
}

/* ----------------------------------------------------------------- */
/*
 * remove interface
 */
static int
prodrv_sns_remove(struct platform_device *pdev)
{
	struct sensor_data *data;

	PROX_DBG("[PROXIMITY] prodrv_sns_remove \n");
	
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

/*--------------------------------------------------------------------
  device module attribute (device desc.) setting
  ------------------------------------------------------------------*/
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

/* ----------------------------------------------------------------- */
/*
 * proximity sensor active-mode setting function
 */
int
prodrv_sns_ON(void)
{
	int ok_flag = true;

	PROX_DBG("[PROXIMITY] prodrv_sns_ON \n");

	// proximity status initialize
	prox_detect.proxy_val = PROX_VAL_NEAR;
	prox_detect.proxy_state = PROX_DETECT_NON;
	prox_detect.proxy_lux_save = 0;
	memset(prox_detect.proxy_distance_save,0,
		sizeof(prox_detect.proxy_distance_save));
	prox_detect.proxy_distreg_save =0;

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	// proximty interrupt (GPIO) signal setting
#ifndef CONFIG_GPIOLIB
	gpio_configure(g_gpio_number, GPIOF_ENABLE_WAKE | GPIOF_INPUT);
#else
	gpio_direction_input(g_gpio_number);
#endif
	
	//added to set it to no pull as per spec
	gpio_tlmm_config(
		GPIO_CFG(g_gpio_number, 0, GPIO_INPUT, g_gpio_pullup, g_gpio_strength)
																,GPIO_ENABLE);
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	msleep(SNS_STANDBY_TIME);

	// i2c register initialize
	ok_flag = prodrv_sns_reg_init();
	if(ok_flag!=true) {
#ifndef CONFIG_GPIOLIB
		gpio_configure(g_gpio_number, GPIOF_DRIVE_OUTPUT);
#else
		gpio_direction_output(g_gpio_number, 1);
#endif
	}
	return ok_flag;
}

/* ----------------------------------------------------------------- */
/*
 * proximity sensor standby-mode setting function
 */
void
prodrv_sns_OFF(void)
{
	PROX_DBG("[PROXIMITY] prodrv_sns_OFF \n");

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	// GPIO register finalize
#ifndef CONFIG_GPIOLIB
	gpio_configure(g_gpio_number, GPIOF_DRIVE_OUTPUT);
#else
	gpio_direction_output(g_gpio_number, 1);
#endif
	
	gpio_tlmm_config(
		GPIO_CFG(g_gpio_number, 0, GPIO_OUTPUT, g_gpio_pullup, g_gpio_strength)
																,GPIO_DISABLE);
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	// i2c register finalize
	prodrv_sns_ssd();

	// driver vars reset
	prox_detect.proxy_val = PROX_VAL_NEAR;
	prox_detect.proxy_state = PROX_DETECT_NON;
	prox_detect.proxy_lux_save = 0;
	memset(prox_detect.proxy_distance_save,0,
		sizeof(prox_detect.proxy_distance_save));
	prox_detect.proxy_distreg_save =0;
	
}

/* ----------------------------------------------------------------- */
/*
 * proximity sensor I2C active-mode setting function
 */
static int
prodrv_sns_reg_init(void)
{
	int		init_sts;

	// ALS setting (Illuminate setting)
	// proximity sensor device (ALS meas mode) initialize
	init_sts = prodrv_sns_i2c_write(I2C_PROX_ALS_SENSITIVITY,g_als_sens);
	PROX_DBG("[PROXIMITY] prodrv_sns_reg_init ALS_SENS = %02x\n",g_als_sens);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init ALS_CONTROL FALSE \n");
		return false;
	}

	// proximity sensor device (ALS meas mode) initialize
	init_sts = prodrv_sns_i2c_write(I2C_PROX_ALS_MEAS_RATE, 
						ALS_RATE_ENABLE | ALS_MEAS_RATE_500ms);	// ALS interval
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init ALS_CONTROL FALSE \n");
		return false;
	}

	// proximity sensor device (ALS control mode) initialize
	init_sts = prodrv_sns_i2c_write(I2C_PROX_ALS_CONTROL, ALS_MODE_STANDALONE);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init ALS_CONTROL FALSE \n");
		return false;
	}
	i2c_proximity_ready = 1;		// I2C proximity ALS access ready

	// PS setting (proximity setting)
	// proximity sensor device (LED current) initialize
	init_sts = prodrv_sns_i2c_write(I2C_PROX_I_LED, g_i_led);	
	PROX_DBG("[PROXIMITY] prodrv_sns_reg_init I_LED = %02x\n",g_i_led);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init I_LED FALSE \n");
		return false;
	}

	// proximity sensor device (PS interrupt threshold [HI]) initialize
/* FUJITSU:2011-05-10 ALS/PS CORRECT start */
	init_sts = prodrv_sns_i2c_write(I2C_PROX_PS_TH_H_LED1, g_ps_hi);
/* FUJITSU:2011-05-10 ALS/PS CORRECT end */
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init INTERRUPT HI TH  \n");
		return false;
	}

	// proximity sensor device (PS interrupt threshold [LOW]) initialize
/* FUJITSU:2011-05-10 ALS/PS CORRECT start */
	init_sts = prodrv_sns_i2c_write(I2C_PROX_PS_TH_L_LED1, g_ps_low);
/* FUJITSU:2011-05-10 ALS/PS CORRECT end */
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init INTERRUPT LO TH  \n");
		return false;
	}

	// proximity sensor device (PS meas rate) initialize
	init_sts = prodrv_sns_i2c_write(I2C_PROX_PS_MEAS_RATE, PS_MEAS_RATE_50ms);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init PS meas rate\n");
		return false;
	}

	// proximity sensor device (ALS/PS persistence) initialize
	// proximity detect rate = meas rate x persistence = 50ms x 3 = 150ms
	init_sts = prodrv_sns_i2c_write(I2C_PROX_PERSISTENCE, 0x03);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init PS presistence\n");
		return false;
	}

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	// proximity sensor device (interrupt mode) default set (default FAR->NEAR)
	prox_detect.proxy_intr_save = EIFFEL_INTERRUPT_RESET;

	// proximity sensor device (interrupt mode) initialize (disable intr.)
	init_sts = prodrv_sns_i2c_write(I2C_PROX_INTERRUPT, EIFFEL_INTERRUPT_RESET);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init INTERRUPT FALSE \n");
		return false;
	}
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	// proximity sensor device (PS control mode) initialize
	init_sts = prodrv_sns_i2c_write(I2C_PROX_PS_CONTROL, PS_MODE_STANDALONE);	
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init PS_CONTROL FALSE \n");
		return false;
	}

	return true;
}

/* ----------------------------------------------------------------- */
/*
 * proximity sensor I2C standby-mode setting function
 * @param   reg    [in] register address
 * @param   data   [in] write data buffer
 * @return  0      :    normal end
 *          !0     :    i2c_transfer error
 */
static void
prodrv_sns_ssd(void)
{
	int		final_sts;

	PROX_DBG("[PROXIMITY] prodrv_sns_ssd \n");

	// proximity sensor device (PS) standby
	final_sts = prodrv_sns_i2c_write(I2C_PROX_PS_CONTROL, PS_MODE_STANDBY);	
	if(0 > final_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_ssd PS_CONTROL FALSE \n");
	}

	i2c_proximity_ready = 0;	// I2C proximity ALS access not ready

	// proximity sensor device (ALS) standby
	final_sts = prodrv_sns_i2c_write(I2C_PROX_ALS_CONTROL, ALS_MODE_STANDBY);
	if(0 > final_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_ssd ALS_CONTROL FALSE \n");
	}
}

/* ----------------------------------------------------------------- */
/*
 * does i2c write to the proximity sensor
 * @param   reg    [in] register address
 * @param   data   [in] write data buffer
 * @return  0      :    normal end
 *          !0     :    i2c_transfer error
 */
static int
prodrv_sns_i2c_write(unsigned char reg, unsigned char data)
{
	struct i2c_msg msg;
	u_int8_t buf[I2C_WRITE_BUFSIZE];
	int ret = 0;

	msg.addr = PROX_SLAVE_ADDRESS;
	msg.flags = 0;
	buf[0] = reg;
	buf[1] = data;

	msg.buf  = buf;
	msg.len  = 2;

	ret = i2c_transfer(i2c_proximity, &msg, I2C_TRANS_WRITE);

	return ret;
}

/* ----------------------------------------------------------------- */
/*
 * does i2c read from the proximity sensor
 * @param   reg    [in] register address
 * @param   data   [out]read data buffer
 * @param   len    [out]read data buffer length
 * @return  0      :    normal end
 *          !0     :    i2c_transfer error
 */
static int
prodrv_sns_i2c_read(unsigned short reg, unsigned char *data, uint32_t len)
{
	struct i2c_msg msg[READ_SIZE];
	u_int8_t msgbuf[READ_SIZE];
	int ret = 0;

	memcpy(msgbuf, &reg, sizeof(reg));

	msg[0].addr  = PROX_SLAVE_ADDRESS;
	msg[0].flags = 0;
	msg[0].buf   = msgbuf;
	msg[0].len   = 1;

	msg[1].addr  = PROX_SLAVE_ADDRESS;
	msg[1].flags = I2C_M_RD;
	msg[1].buf   = data;
	msg[1].len   = len;

	ret = i2c_transfer(i2c_proximity, msg, I2C_TARNS_READ);

	return ret;
}

/* ----------------------------------------------------------------- */
/*
 * proximity ALS register get
 * @param   lux    [out]illumi sensor data (Lux)
 * @return  0      :    normal end
 *          !0     :    error
 */
int
prodrv_sns_als_data_read(int *lux)
{
	unsigned char data1[READ_SIZE];
	int 		res;

//	PROX_DBG("[PROXIMITY] prodrv_sns_als_data_read \n");
	
	/*--------------------------------------------------------------------
		ALS standalone mode
	--------------------------------------------------------------------*/
	*lux=0;

	res = prodrv_sns_i2c_read(I2C_PROX_ALS_PS_STATUS, data1, 1);
	if (res < 0){
		PROX_PRINT(KERN_ERR "proximity: als status I2C access error\n");
		PROX_DBG("[PROXIMITY] prodrv_sns_als_data_read PS status failed\n");
		return res;
	}

	if( data1[0] & STS_ALSDATA_BIT ) {
		// Illumi sensor value change
		
		// Illumi sensor value(Hi/Low) get
		res = prodrv_sns_i2c_read(I2C_PROX_ALS_DATA_0, data1, 2);
		if (res < 0){
			PROX_PRINT(KERN_ERR "proximity: als data I2C access error\n");
			PROX_DBG("[PROXIMITY] prodrv_sns_als_data_read ALS read failed\n");
			return res;
		}
		*lux = (data1[1] << 8) | data1[0];

/* FUJITSU:2011-05-10 ALS/PS CORRECT start */
		// Illumi sensor value software correct 
		*lux = *lux * g_als_correct;
/* FUJITSU:2011-05-10 ALS/PS CORRECT end */

		prox_detect.proxy_lux_save = *lux;

	}
	else {
		// Illumi sensor value not change (save data value)
		*lux = prox_detect.proxy_lux_save;

	}

	return 0;
}
EXPORT_SYMBOL(prodrv_sns_als_data_read);

/* ----------------------------------------------------------------- */
/*
 * does distance report save function
 */
void prodrv_sns_report_distance(void)
{
	int res;
	int dat;
	unsigned char data1[READ_SIZE];
 
	// proximity sensor value get
	res = prodrv_sns_i2c_read(I2C_PROX_PS_DATA_LED1, data1, 1);
	if (res < 0){
		PROX_DBG("[PROXIMITY] prodrv_sns_report_distance read failed\n");
		return;
	}
	dat = data1[0];

	prox_detect.proxy_distreg_save = dat;
	// conv. to distance[cm]
	res = prodrv_sns_convert_data(dat, prox_detect.proxy_distance_save);
	if (res < 0){
		PROX_PRINT(KERN_ERR "proximiy: distance convert error (%d)\n",dat);
		PROX_DBG("[PROXIMITY] prodrv_sns_report_distance data-conver failed\n");
		return;
	}

	PROX_DBG("[PROXIMITY] prodrv_sns_report_distance dat=%d dist=%s\n",dat,
			prox_detect.proxy_distance_save);

	return;
}

/* ----------------------------------------------------------------- */
/*
 * does proximiy sensor distance convert
 */
#define DIST_LIST_MAX 256
static int prodrv_sns_convert_data(int data, char *val)
{
	// scale:cm
	static char distance_list[DIST_LIST_MAX][8] = {
		"22.000","21.506","21.024","20.552","20.091","19.641","19.200","18.770",//   0-
		"18.349","17.937","17.535","17.142","16.758","16.382","16.014","15.655",//   8-
		"15.304","14.961","14.625","14.297","13.977","13.663","13.357","13.057",//  16-
		"12.765","12.478","12.198","11.925","11.657","11.396","11.140","10.891",//  24-
		"10.646","10.408","10.174", "9.946", "9.723", "9.505", "9.292", "9.083",//  32-
		 "8.880", "8.681", "8.486", "8.296", "8.110", "7.928", "7.750", "7.576",//  40-
		 "7.406", "7.240", "7.078", "6.919", "6.764", "6.612", "6.464", "6.319",//  48-
		 "6.177", "6.039", "5.903", "5.771", "5.641", "5.515", "5.391", "5.270",//  56-
		 "5.152", "5.037", "4.924", "4.813", "4.705", "4.600", "4.496", "4.396",//  64-
		 "4.297", "4.201", "4.106", "4.014", "3.924", "3.836", "3.750", "3.666",//  72-
		 "3.584", "3.504", "3.425", "3.348", "3.273", "3.200", "3.128", "3.058",//  80-
		 "2.989", "2.922", "2.857", "2.793", "2.730", "2.669", "2.609", "2.550",//  88-
		 "2.493", "2.437", "2.382", "2.329", "2.277", "2.226", "2.176", "2.127",//  96-
		 "2.079", "2.033", "1.987", "1.942", "1.899", "1.856", "1.815", "1.774",// 104-
		 "1.734", "1.695", "1.657", "1.620", "1.584", "1.548", "1.513", "1.480",// 112-
		 "1.446", "1.414", "1.382", "1.351", "1.321", "1.291", "1.262", "1.234",// 120-
		 "1.206", "1.179", "1.153", "1.127", "1.102", "1.077", "1.053", "1.029",// 128-
		 "1.006", "0.983", "0.961", "0.940", "0.919", "0.898", "0.878", "0.858",// 136-
		 "0.839", "0.820", "0.802", "0.784", "0.766", "0.749", "0.732", "0.716",// 144-
		 "0.700", "0.684", "0.669", "0.654", "0.639", "0.625", "0.611", "0.597",// 152-
		 "0.584", "0.570", "0.558", "0.545", "0.533", "0.521", "0.509", "0.498",// 160
		 "0.487", "0.476", "0.465", "0.455", "0.444", "0.434", "0.425", "0.415",// 168
		 "0.406", "0.397", "0.388", "0.379", "0.371", "0.362", "0.354", "0.346",// 176
		 "0.338", "0.331", "0.323", "0.316", "0.309", "0.302", "0.295", "0.289",// 184
		 "0.282", "0.276", "0.270", "0.264", "0.258", "0.252", "0.246", "0.241",// 192
		 "0.235", "0.230", "0.225", "0.220", "0.215", "0.210", "0.205", "0.201",// 200
		 "0.196", "0.192", "0.187", "0.183", "0.179", "0.175", "0.171", "0.167",// 208
		 "0.163", "0.160", "0.156", "0.153", "0.149", "0.146", "0.143", "0.139",// 216
		 "0.136", "0.133", "0.130", "0.127", "0.124", "0.122", "0.119", "0.116",// 224
		 "0.114", "0.111", "0.109", "0.106", "0.104", "0.101", "0.099", "0.097",// 232
		 "0.095", "0.093", "0.090", "0.088", "0.086", "0.084", "0.083", "0.081",// 240
		 "0.079", "0.077", "0.075", "0.074", "0.072", "0.070", "0.069", "0.067"	// 248
	};

	if((data >=  DIST_LIST_MAX) || (data < 0)){
		return -1;
	}

	memcpy(val,distance_list[data],8);
	return 0;
}

/* ----------------------------------------------------------------- */
/*
 * does proximity block hander
 */
/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
static void
prodrv_sns_work_bh(struct work_struct *work)
{
	int res;
	unsigned char data1[READ_SIZE];
	
	PROX_DBG("[PROXIMITY] prodrv_sns_work_bh \n");

	msleep(SNS_STANDBY_TIME);

	// proximity sensor device (interrupt mode) read-reset
	res = prodrv_sns_i2c_read(I2C_PROX_INTERRUPT, data1, 1);
	if(0 > res) {
		PROX_PRINT(KERN_ERR "proximity: intr-read I2C access error\n");
		PROX_DBG("[PROXIMITY] prodrv_sns_work_bh INTERRUPT read FALSE \n");
	}

	PROX_DBG("[PROXIMITY] prodrv_sns_work_bh WAKE \n");
	atomic_set(&g_prox_state,PROX_STATE_WAKE);

	// status setting
	prodrv_sns_work_bh_sub();

	enable_irq(g_proxi_irq);
}

static void
prodrv_sns_work_bh_sub(void)
{
	int res;
	unsigned char data1[READ_SIZE];

	PROX_DBG("[PROXIMITY] prodrv_sns_work_bh_sub \n");
	
	if(atomic_read(&g_prox_state) != PROX_STATE_SUSPEND) {
		// PS status read
		res = prodrv_sns_i2c_read(I2C_PROX_ALS_PS_STATUS, data1, 1);
		if (res < 0){
			PROX_PRINT(KERN_ERR "proximity: ps status I2C access error\n");
			PROX_DBG("[PROXIMITY] prodrv_sns_work_bh_sub PS status read  "
					"failed  \n");
			return;
		}

		// NEAR or FAR check
		if(data1[0] & STS_PSINT_BIT) {
			// bit ON (PS intrrupt ACTIVE) = NEAR
			PROX_DBG("[PROXIMITY] prodrv_sns_work_bh_sub NEAR (PS_STA=%02x) \n"
					,data1[0]);

			// save state
			prox_detect.proxy_val = PROX_VAL_NEAR;
			prox_detect.proxy_state = PROX_DETECTED;

			// proximity sensor device (interrupt mode) change
			prox_detect.proxy_intr_save = EIFFEL_INT_SET_N2F;
	
		}
		else {
			// bit OFF (PS intrrupt INACTIVE) = FAR
			PROX_DBG("[PROXIMITY] prodrv_sns_work_bh_sub FAR (PS_STA=%02x) \n"
					,data1[0]);

			// save state
			prox_detect.proxy_val = PROX_VAL_FAR;
			prox_detect.proxy_state = PROX_DETECT_NON;
			
			// proximity sensor device (interrupt mode) change
			prox_detect.proxy_intr_save = EIFFEL_INT_SET_F2N;
	
		}

		// distance data get
		prodrv_sns_report_distance();

		// report save
		input_report_abs(this_data, ABS_X, prox_detect.proxy_val);
		input_sync(this_data);

	}


}
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

/* ----------------------------------------------------------------- */
/*
 * does proximity irq handler
 */
static irqreturn_t
prodrv_sns_irq_handler(int irq, void *p)
{
/* FUJITSU:2011-07-13 INTR change start */
	if(atomic_read(&g_wake)) {
		disable_irq_nosync(g_proxi_irq);
		schedule_work(&g_proxi_work_data);
	}
/* FUJITSU:2011-07-13 INTR change end */
	return IRQ_HANDLED;
}


/* ----------------------------------------------------------------- */
/*
 * proximity irq  setting
 * @param   (none)
 * @return  0      :  normal end
 *          !0     :  irq initialize error
 */
static int
prodrv_sns_request_irqs(void)
{
	int err;
/* FUJITSU:2011-07-13 INTR change start */
	unsigned long req_flags = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING;
/* FUJITSU:2011-07-13 INTR change end */

	err = g_proxi_irq = gpio_to_irq(g_gpio_number);
	err = request_irq(g_proxi_irq, prodrv_sns_irq_handler,
			req_flags, SENSOR_NAME, NULL);
	if (err) {
		PROX_PRINT(KERN_ERR "proximity: irq init fail\n");
		return err;
	}
	set_irq_wake(g_proxi_irq, WAKE_ON);
	return 0;
}

/* ----------------------------------------------------------------- */
/*
 * i2c adapter setting
 * @param   (none)
 * @return  !0       : I2C adapter descriptor
 *           0(NULL) : ERROR
 */
struct i2c_adapter * prodev_i2c_search_adapter(void)
{
	struct i2c_adapter *	adptmp = NULL;
/* FUJITSU:2011-05-10 PROXIMITY I2C adapter-set change start */
#define PROX_I2C_ADAPTER_AUTO_SEARCH 0
#if PROX_I2C_ADAPTER_AUTO_SEARCH
	int res;
	unsigned char data1[READ_SIZE];
	int	i;
#endif

    //i2c qup adapter id=4
    PROX_DBG("[PROXIMITY] prodev_i2c_search_adapter system_rev(%d)\n"
			,system_rev);
    if(system_rev >= 0x0c) {
    	    PROX_DBG("[PROXIMITY] prodev_i2c_search_adapter adapter=4\n");
	    i2c_proximity = i2c_get_adapter(4);
	}
	else {
    	    PROX_DBG("[PROXIMITY] prodev_i2c_search_adapter adapter=0\n");
	    i2c_proximity = i2c_get_adapter(0);
	}
	adptmp = i2c_proximity;

#if PROX_I2C_ADAPTER_AUTO_SEARCH
	for(i=0;i<6;i++) {
		i2c_proximity = i2c_get_adapter(i);
		if(NULL != i2c_proximity) {
			res = prodrv_sns_i2c_read(I2C_PROX_ALS_CONTROL, data1, 1);
			if (res < 0){
				PROX_DBG("[PROXIMITY] prodev_i2c_search_adapter HW "
						"not-found(%d)\n",i);
			} 
			else {
				PROX_DBG("[PROXIMITY] prodev_i2c_search_adapter HW "
						"found (%d)\n",i);
				adptmp = i2c_proximity;
				break;
			}
		}
	}
	i2c_proximity = adptmp;
#endif	
/* FUJITSU:2011-05-10 PROXIMITY I2C adapter-set change end */

	return adptmp;
}


/* ----------------------------------------------------------------- */
/*
 * sensor initializeation
 * @param   (none)
 * @param   data   [in] write data buffer
 * @return  0      :    normal end
 *          !0     :    init error
 */
static int __init prodrv_sns_init(void)
{

	PROX_DBG("[PROXIMITY] prodrv_sns_init \n");
	
	i2c_proximity = prodev_i2c_search_adapter();
/* FUJITSU:2011-04-21 NULL_check start */
	if (NULL == i2c_proximity){
		PROX_PRINT(KERN_ERR "proximity: i2c init fail\n");
		return -1;
	}
/* FUJITSU:2011-04-21 NULL_check end */

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	// interrupt GPIO configure
	PROX_DBG("[PROXIMITY] prodrv_sns_init system_rev(%d)\n",system_rev);
	if(system_rev >= 0x0b) {
		g_gpio_number = PROX_GPIO_REV0;
		g_gpio_pullup = GPIO_NO_PULL;
		g_gpio_strength = GPIO_2MA;
	}
	else {
		g_gpio_number = PROX_GPIO_REV1;
		g_gpio_pullup = GPIO_PULL_UP;
		g_gpio_strength = GPIO_2MA;
	}
	PROX_DBG("[PROXIMITY] prodrv_sns_init config(num=%d,pull=%d,str=%d\n"
			,g_gpio_number,g_gpio_pullup,g_gpio_strength);
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */


	INIT_WORK(&g_proxi_work_data, prodrv_sns_work_bh);
	atomic_set(&g_wake,false);
	atomic_set(&g_suspend_off,false);
	atomic_set(&g_prox_state,PROX_STATE_WAKE);

	sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
	if (IS_ERR(sensor_pdev)) {
		PROX_PRINT(KERN_ERR "proximity: sensor device registraton error (%s)\n",SENSOR_NAME);
		return -1;
	}
	
	return platform_driver_register(&sensor_driver);
}
module_init(prodrv_sns_init);

/* ----------------------------------------------------------------- */
/*
 * sensor shutdown
 * @param   (none)
 * @return  (none)
 */
static void __exit prodrv_sns_exit(void)
{
	PROX_DBG("[PROXIMITY] prodrv_sns_exit \n");
	
	prodrv_sns_OFF();
	platform_driver_unregister(&sensor_driver);
	platform_device_unregister(sensor_pdev);
}
module_exit(prodrv_sns_exit);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_LICENSE( "GPL" );
MODULE_VERSION("1.0.0");
