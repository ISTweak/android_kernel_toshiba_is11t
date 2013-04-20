/* arch/arm/mach-msm/rpc_server_time_remote.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2011 Code Aurora Forum. All rights reserved.
 * Author: Iliyan Malchev <ibm@android.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <mach/msm_rpcrouter.h>
#include "rpc_server_time_remote.h"
#include <linux/rtc.h>

/* FUJITSU:2011/07/07 K06 start */
#ifdef CONFIG_MACH_F11K06
#include <linux/fs.h>
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/07/07 K06 end */

#include <linux/android_alarm.h>
#include <linux/rtc-msm.h>

/* FUJITSU:2011/07/07 K06 start */
#ifdef CONFIG_MACH_F11K06
#include "../../../kernel/power/power.h"
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/07/07 K06 end */

/* time_remote_mtoa server definitions. */

#define TIME_REMOTE_MTOA_PROG 0x3000005d
#define TIME_REMOTE_MTOA_VERS_OLD 0
#define TIME_REMOTE_MTOA_VERS 0x9202a8e4
#define TIME_REMOTE_MTOA_VERS_COMP 0x00010002
#define RPC_TIME_REMOTE_MTOA_NULL   0
#define RPC_TIME_TOD_SET_APPS_BASES 2
#define RPC_TIME_GET_APPS_USER_TIME 3

struct rpc_time_tod_set_apps_bases_args {
	uint32_t tick;
	uint64_t stamp;
};

static int read_rtc0_time(struct msm_rpc_server *server,
		   struct rpc_request_hdr *req,
		   unsigned len)
{
	int err;
	unsigned long tm_sec;
	uint32_t size = 0;
	void *reply;
	uint32_t output_valid;
	uint32_t rpc_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
	struct rtc_time tm;
	struct rtc_device *rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);

	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		goto send_reply;
	}

	err = rtc_read_time(rtc, &tm);
	if (err) {
		pr_err("%s: Error reading rtc device (%s) : %d\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE, err);
		goto close_dev;
	}

	err = rtc_valid_tm(&tm);
	if (err) {
		pr_err("%s: Invalid RTC time (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		goto close_dev;
	}

	rtc_tm_to_time(&tm, &tm_sec);
	rpc_status = RPC_ACCEPTSTAT_SUCCESS;

close_dev:
	rtc_class_close(rtc);

send_reply:
	reply = msm_rpc_server_start_accepted_reply(server, req->xid,
						    rpc_status);
	if (rpc_status == RPC_ACCEPTSTAT_SUCCESS) {
		output_valid = *((uint32_t *)(req + 1));
		*(uint32_t *)reply = output_valid;
		size = sizeof(uint32_t);
		if (be32_to_cpu(output_valid)) {
			reply += sizeof(uint32_t);
			*(uint32_t *)reply = cpu_to_be32(tm_sec);
			size += sizeof(uint32_t);
		}
	}
	err = msm_rpc_server_send_accepted_reply(server, size);
	if (err)
		pr_err("%s: send accepted reply failed: %d\n", __func__, err);

	return 1;
}

static int handle_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	struct timespec ts, tv;

/* FUJITSU:2011/07/15 K06 start */
#ifdef CONFIG_MACH_F11K06
    suspend_state_t suspend_state;
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/07/15 K06 end */

	switch (req->procedure) {
	case RPC_TIME_REMOTE_MTOA_NULL:
		return 0;

	case RPC_TIME_TOD_SET_APPS_BASES: {
		struct rpc_time_tod_set_apps_bases_args *args;

/* FUJITSU:2011/07/07 K06 start */
#ifdef CONFIG_MACH_F11K06
		struct kstat temp_state;

		if (vfs_stat("/data/data/com.android.settings/files"
				"/autotime.disabled", &temp_state) == 0) {
			printk(KERN_INFO "RPC_TIME_TOD_SET_APPS_BASES:\n"
				"\tautotime.disabled\n");
			return 0;
		}
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/07/07 K06 end */

		args = (struct rpc_time_tod_set_apps_bases_args *)(req + 1);
		args->tick = be32_to_cpu(args->tick);
		args->stamp = be64_to_cpu(args->stamp);
		printk(KERN_INFO "RPC_TIME_TOD_SET_APPS_BASES:\n"
		       "\ttick = %d\n"
		       "\tstamp = %lld\n",
		       args->tick, args->stamp);

		getnstimeofday(&ts);
		if (msmrtc_is_suspended()) {
			int64_t now, sleep, tick_at_suspend;
			now = msm_timer_get_sclk_time(NULL);
			tick_at_suspend = msmrtc_get_tickatsuspend();
			if (now && tick_at_suspend) {
				sleep = now - tick_at_suspend;
				timespec_add_ns(&ts, sleep);
			} else
				pr_err("%s: Invalid ticks from SCLK"
					"now=%lld tick_at_suspend=%lld",
					__func__, now, tick_at_suspend);

		}
		rtc_hctosys();
		getnstimeofday(&tv);
		/* Update the alarm information with the new time info. */

/* FUJITSU:2011/07/15 K06 start */
#ifdef CONFIG_MACH_F11K06

		suspend_state = get_suspend_state();
		if (suspend_state == PM_SUSPEND_ON) {

#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/07/15 K06 end */

		alarm_update_timedelta(ts, tv);
/* FUJITSU:2011/07/15 K06 start */
#ifdef CONFIG_MACH_F11K06
		}
		alarm_wake_up_executor();

#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/07/15 K06 end */

		return 0;
	}

	case RPC_TIME_GET_APPS_USER_TIME:
		return read_rtc0_time(server, req, len);

	default:
		return -ENODEV;
	}
}

static struct msm_rpc_server rpc_server[] = {
	{
		.prog = TIME_REMOTE_MTOA_PROG,
		.vers = TIME_REMOTE_MTOA_VERS_OLD,
		.rpc_call = handle_rpc_call,
	},
	{
		.prog = TIME_REMOTE_MTOA_PROG,
		.vers = TIME_REMOTE_MTOA_VERS,
		.rpc_call = handle_rpc_call,
	},
	{
		.prog = TIME_REMOTE_MTOA_PROG,
		.vers = TIME_REMOTE_MTOA_VERS_COMP,
		.rpc_call = handle_rpc_call,
	},
};

static int __init rpc_server_init(void)
{
	/* Dual server registration to support backwards compatibility vers */
	int ret;
	ret = msm_rpc_create_server(&rpc_server[2]);
	if (ret < 0)
		return ret;
	ret = msm_rpc_create_server(&rpc_server[1]);
	if (ret < 0)
		return ret;
	return msm_rpc_create_server(&rpc_server[0]);
}


module_init(rpc_server_init);
