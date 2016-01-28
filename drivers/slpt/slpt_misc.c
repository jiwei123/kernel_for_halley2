/*
 *  Copyright (C) 2014 Wu Jiao <jwu@ingenic.cn>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slpt.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/time.h>

#define TCSM_SAVE_SIZE   4096
#define TCSM_BASE        (0xb3422000)

#if defined(CONFIG_SOC_4775)
static char tcsm_back[TCSM_SAVE_SIZE] __attribute__ ((aligned (32)));
#endif

int slpt_suspend_in_kernel(suspend_state_t state) {
	int error = 0;

#if defined(CONFIG_SOC_4775)
	memcpy(tcsm_back, (void *)TCSM_BASE, TCSM_SAVE_SIZE);
	error = slpt_save_pm_enter_func(SLPTARG_LOAD_RESETDLL);
	memcpy((void *)TCSM_BASE, tcsm_back, TCSM_SAVE_SIZE);
#elif defined(CONFIG_SOC_M200)
	error = slpt_save_pm_enter_func(state);
#endif

	return error;
}

/* used by slpt app to call kernel printk */
int slpt_printf(const char *fmt, ...) {
	char buf[200];
	va_list args;
	int r;

	va_start(args, fmt);
	r = vsnprintf(buf, sizeof(buf) - 1, fmt, args);
	va_end(args);

	pr_info("%s", buf);

	return 0;
}
EXPORT_SYMBOL(slpt_printf);

void __weak wlan_pw_en_disable(void) {

}


static struct timespec suspend_time;

void slpt_set_suspend_time(void)
{
	getnstimeofday(&suspend_time);
}

int slpt_get_suspend_time(long int *tv_sec, long int *tv_nsec)
{
	if (tv_sec == NULL || tv_nsec == NULL)
		return -EINVAL;

	*tv_sec = suspend_time.tv_sec;
	*tv_nsec = suspend_time.tv_nsec;

	return 0;
}
EXPORT_SYMBOL(slpt_get_suspend_time);
