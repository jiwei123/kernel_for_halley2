
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
#include <linux/gpio.h>

extern void show_wakeup_sources(void);
extern void record_suspend_time(void);

#define SLEEP_LIB_TCSM          0xb3426000
#define SLEEP_LIB_LPDDR2        0x8fff8000
#define SLEEP_LIB_SIZE          (16 * 1024)
#define SLEEP_LIB_EMMC_ADDRESS  (43 * 1024)

struct m200_sleep_lib_entry
{
	void (*enable_set_pmu_suspend_mode_voltage)(int enable, unsigned int mV);
	void (*enable_sleep_poweroff_mode)(int enable);
	void (*restore_context)(void);
	int (*enter_sleep)(int state);
	void (*select_pmu)(int pmu);
};

static int do_enter_sleep(suspend_state_t state)
{
	struct m200_sleep_lib_entry *sleep_entry = (void *)(SLEEP_LIB_TCSM);

	sleep_entry->enter_sleep(state);

	return 0;
}

static int enter(suspend_state_t state)
{
	int ret;

	record_suspend_time();

#ifdef CONFIG_SLPT
	ret = slpt_pm_enter(state);
#else
	ret = do_enter_sleep(state);
#endif

	show_wakeup_sources();

	return ret;
}

static struct platform_suspend_ops pm_ops = {
	.valid = suspend_valid_only_mem,
	.enter = enter,
};

int __init pm_init(void)
{
	suspend_set_ops(&pm_ops);
	slpt_set_suspend_ops(do_enter_sleep);

	return 0;
}

arch_initcall(pm_init);
