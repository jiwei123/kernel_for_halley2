
/*
 * Copyright (C) 2016 Ingenic Semiconductor
 *
 * SunWenZhong(Fighter) <wenzhong.sun@ingenic.com, wanmyqawdr@126.com>
 *
 * Copyright 2007, The Android Open Source Project
 *
 * For project-5
 *
 * Release under GPLv2
 *
 */

#include <linux/init.h>
#include <linux/suspend.h>

#include <asm/cacheops.h>
#include <soc/cache.h>
#include <asm/r4kcache.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#ifdef CONFIG_SLPT
#include <linux/slpt.h>
#endif

extern void show_wakeup_sources(void);
extern void record_suspend_time(void);

#define SLEEP_LIB_TCSM          0xb3426000
#define SLEEP_LIB_LPDDR2        0x8fff8000
#define SLEEP_LIB_SIZE          (16 * 1024)
#define SLEEP_LIB_EMMC_ADDRESS  (43 * 1024)

struct m200_sleep_lib_entry
{
	void (*enable_set_sleep_core_voltage)(int enable, unsigned int mV);
	void (*enable_sleep_pmu_low_power_mode)(int enable);
	void (*enable_sleep_poweroff_mode)(int enable);
	void (*restore_context)(void);
	int (*enter_sleep)(int state);
};

static int enter(suspend_state_t state)
{
	struct m200_sleep_lib_entry *sleep_entry = (void *)(SLEEP_LIB_TCSM);

#if !defined(CONFIG_SLPT)
	record_suspend_time();
#endif

	sleep_entry->enter_sleep(state);

#if !defined(CONFIG_SLPT)
	show_wakeup_sources();
#endif

	return 0;
}

static struct platform_suspend_ops pm_ops = {
	.valid = suspend_valid_only_mem,
	.enter = enter,
};

int __init pm_init(void)
{
#if defined(CONFIG_SLPT)
	slpt_set_suspend_ops(&pm_ops);
#else
	suspend_set_ops(&pm_ops);
#endif
	return 0;
}

arch_initcall(pm_init);
