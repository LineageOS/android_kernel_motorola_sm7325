/* Copyright (c) 2023, Motorola Mobility LLC. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt) "PM: " fmt

#include <linux/string.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/suspend.h>
#include <linux/timekeeping.h>

#include <trace/events/power.h>


bool entry = false;
static void suspend_resume_hook(void *data, const char *action, int val, bool start)
{
	struct rtc_time tm;
	struct timespec64 now;

	if (!strcmp(action, "suspend_enter") && start){
		entry = true;
	} else if (!strcmp(action, "thaw_processes") && !start && entry) {
		entry = false;
	}
	else {
		return;
	}
	ktime_get_real_ts64(&now);

	rtc_time64_to_tm(now.tv_sec, &tm);
	pr_info("suspend %s %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
		entry?"entry":"exit", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, now.tv_nsec);

}

static int __init suspend_marker_init(void)
{
	register_trace_suspend_resume(suspend_resume_hook, NULL);
	return 0;
}

static void __exit suspend_marker_exit(void)
{
	unregister_trace_suspend_resume(suspend_resume_hook, NULL);
}

late_initcall(suspend_marker_init);
module_exit(suspend_marker_exit);

MODULE_DESCRIPTION("Motorola Mobility LLC. suspend marker");
MODULE_LICENSE("GPL v2");
