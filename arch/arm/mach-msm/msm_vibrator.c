/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>

#include <mach/msm_rpcrouter.h>

#define PM_LIBPROG      0x30000061
/* FUJITSU:2011-04-19 f11k06 VIB start	*//* PM LibVersion Update	*/
#if 0
#if (CONFIG_MSM_AMSS_VERSION == 6220) || (CONFIG_MSM_AMSS_VERSION == 6225)
#define PM_LIBVERS      0xfb837d0b
#else
#define PM_LIBVERS      0x10001
#endif
#else
#define PM_LIBVERS      0x30005
#endif
/* FUJITSU:2011-04-19 f11k06 VIB  end	*/

/* FUJITSU:2011-04-19 f11k06 VIB start	*/
#if 0
#define HTC_PROCEDURE_SET_VIB_ON_OFF	21
#else
#define HTC_PROCEDURE_SET_VIB_ON_OFF	22
#endif
/* FUJITSU:2011-04-19 f11k06 VIB  end	*/

#define PMIC_VIBRATOR_LEVEL	(3000)

/* FUJITSU:2011-04-19 f11k06 VIB start	*/
static int vibrator_value;
static struct work_struct work_vibrator_on;
static struct delayed_work work_vibrator_off;
static struct hrtimer vibe_timer;
static struct mutex vib_mutex;
static void timed_vibrator_off(struct timed_output_dev *sdev, int delay);
static void timed_vibrator_on(struct timed_output_dev *sdev);
static int vibrator_get_time(struct timed_output_dev *dev);
/* FUJITSU:2011-04-19 f11k06 VIB  end	*/

/* FUJITSU:2011-04-19 f11k06 VIB start	*/
static int vibrator_timer_active(void)
{
   int ret = 0;
   mutex_lock(&vib_mutex);
   if (hrtimer_active(&vibe_timer)) {
	   ret = 1;
   }
   mutex_unlock(&vib_mutex);
   return ret;
}
/* FUJITSU:2011-04-19 f11k06 VIB  end	*/

static void set_pmic_vibrator(int on)
{
/* FUJITSU:2011-06-06 f11k06 VIB start	*/
	int rc = 0;
/* FUJITSU:2011-06-06 f11k06 VIB end	*/
	static struct msm_rpc_endpoint *vib_endpoint;
	struct set_vib_on_off_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	if (!vib_endpoint) {
		vib_endpoint = msm_rpc_connect(PM_LIBPROG, PM_LIBVERS, 0);
		if (IS_ERR(vib_endpoint)) {
			printk(KERN_ERR "init vib rpc failed!\n");
			vib_endpoint = 0;
			return;
		}
	}


	if (on)
		req.data = cpu_to_be32(PMIC_VIBRATOR_LEVEL);
	else
		req.data = cpu_to_be32(0);

/* FUJITSU:2011-06-06 f11k06 VIB start	*/
#if 0
	msm_rpc_call(vib_endpoint, HTC_PROCEDURE_SET_VIB_ON_OFF, &req,
		sizeof(req), 5 * HZ);
#else
	rc = msm_rpc_call(vib_endpoint, HTC_PROCEDURE_SET_VIB_ON_OFF, &req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "vibrator set failed!\n");
	else
		printk( KERN_INFO "*** VIB *** MSM Call\n" );
#endif
/* FUJITSU:2011-06-06 f11k06 VIB end	*/
}

static void pmic_vibrator_on(struct work_struct *work)
{
/* FUJITSU:2011-04-19 f11k06 VIB start	*/
    hrtimer_start(&vibe_timer,ktime_set(vibrator_value / 1000, (vibrator_value % 1000) * 1000000),HRTIMER_MODE_REL);
    set_pmic_vibrator(1);
	if (!vibrator_timer_active()) {
		/* No timer active, there may in infinite vibration, so switch off */
		timed_vibrator_off(NULL, 0);
	}
/* FUJITSU:2011-04-19 f11k06 VIB  end	*/
}

static void pmic_vibrator_off(struct work_struct *work)
{
	set_pmic_vibrator(0);
/* FUJITSU:2011-04-19 f11k06 VIB start	*/
	if (vibrator_timer_active()) {
		/* Make it on, timer will switch it off */
		set_pmic_vibrator(1);
	}
/* FUJITSU:2011-04-19 f11k06 VIB  end	*/
}

static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_on);
}

/* FUJITSU:2011-04-19 f11k06 VIB start	*/
static void timed_vibrator_off(struct timed_output_dev *sdev, int delay)
{
	schedule_delayed_work(&work_vibrator_off, msecs_to_jiffies(delay));
}
/* FUJITSU:2011-04-19 f11k06 VIB  end	*/

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
/* FUJITSU:2011-04-19 f11k06 VIB start	*/
	int vibrator_time = 0;
	if (value == 0) {
		if (vibrator_timer_active()) {
			vibrator_time = vibrator_get_time(dev);
printk( KERN_INFO "*** VIB *** vibrator Off(time[%d])\n", vibrator_time );
			/* Put a delay of 10 ms, in case the vibrator is off immediately */
			if (vibrator_time < 10 || vibrator_time > 50){
				hrtimer_cancel(&vibe_timer);
				vibrator_time = 0;
			}

			vibrator_value = value;
			timed_vibrator_off(dev, vibrator_time + 10);
		}
	}
	else {
printk( KERN_INFO "*** VIB *** vibrator On(time[%d])\n", value );
		vibrator_value = (value > 15000 ? 15000 : value);
		hrtimer_cancel(&vibe_timer);
/* FUJITSU:2011-04-19 f11k06 VIB  end	*/
		timed_vibrator_on(dev);
	}
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
/* FUJITSU:2011-04-19 f11k06 VIB start	*/
	timed_vibrator_off(NULL,0);
/* FUJITSU:2011-04-19 f11k06 VIB  end	*/
	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

void __init msm_init_pmic_vibrator(void)
{
/* FUJITSU:2011-04-19 f11k06 VIB start	*/
	mutex_init(&vib_mutex);
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_DELAYED_WORK(&work_vibrator_off, pmic_vibrator_off);
/* FUJITSU:2011-04-19 f11k06 VIB  end	*/

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&pmic_vibrator);
}

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

