/*
 *  Copyright (C) 2015 Wu Jiao <jiao.wu@ingenic.com wujiaososo@qq.com>
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/suspend_state.h>
#include <linux/syscore_ops.h>
#include <linux/fb.h>
#include <linux/suspend.h>

static DEFINE_MUTEX(mlock);
static int suspend_state;

/* Routines for PM-transition notifications */

static BLOCKING_NOTIFIER_HEAD(suspend_chain_head);

int register_suspend_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&suspend_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_suspend_notifier);

int unregister_suspend_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&suspend_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_suspend_notifier);

static int suspend_notifier_call_chain(unsigned long event)
{
	return (blocking_notifier_call_chain(&suspend_chain_head, event, NULL)
			== NOTIFY_BAD) ? -EINVAL : 0;
}

void hold_suspend_lock(void) {
	mutex_lock(&mlock);
}

void release_suspend_lock(void) {
	mutex_unlock(&mlock);
}

unsigned int get_suspend_state_with_lock(void) {
	unsigned int state;

	hold_suspend_lock();
	state = suspend_state;
	release_suspend_lock();
	return state;
}

unsigned int get_suspend_state_no_lock(void) {
	return suspend_state;
}

static void the_first_early_suspend(void) {
	hold_suspend_lock();
	suspend_state = STATE_EARLYSUSPEND_ING;
	suspend_notifier_call_chain(suspend_state);
}

static void the_last_late_resume(void) {
	suspend_state = STATE_NO_SUSPEND;
	suspend_notifier_call_chain(suspend_state);
	release_suspend_lock();
}

static void the_last_early_suspend(void) {
	suspend_state = STATE_EARLYSUSPEND;
	suspend_notifier_call_chain(suspend_state);
	release_suspend_lock();
}

static void the_first_late_resume(void) {
	hold_suspend_lock();
	suspend_state = STATE_LATERESUME_ING;
	suspend_notifier_call_chain(suspend_state);
}

extern void fb_early_notify_suspend_state(int event) {
	printk(KERN_DEBUG "%s: evnet %d\n", __func__, event);
	if (event != FB_BLANK_UNBLANK)
		the_first_early_suspend();
	else
		the_first_late_resume();
}

extern void fb_late_notify_suspend_state(int event) {
	printk(KERN_DEBUG "%s: evnet %d\n", __func__, event);
	if (event != FB_BLANK_UNBLANK)
		the_last_early_suspend();
	else
		the_last_late_resume();
}

static int pm_notifier_call_func(struct notifier_block *nb, unsigned long msg, void *data) {
	hold_suspend_lock();

	if (msg == PM_SUSPEND_PREPARE) {
		suspend_state = STATE_SUSPEND_ING;
	}else if (msg == PM_POST_SUSPEND) {
		suspend_state = STATE_EARLYSUSPEND;
	}

	suspend_notifier_call_chain(suspend_state);

	release_suspend_lock();

	return NOTIFY_DONE;
}

static struct notifier_block pm_notifier_t = {
	.notifier_call = pm_notifier_call_func,
	.priority = 0,
};

int suspend_syscore_ops_syspend(void)
{
	int ret = 0;
	hold_suspend_lock();
	suspend_state = STATE_CORE_SYSPEND;
	ret = suspend_notifier_call_chain(suspend_state);
	release_suspend_lock();
	return ret;
}

void suspend_syscore_ops_resume(void)
{
	hold_suspend_lock();
	suspend_state = STATE_CORE_RESUME;
	suspend_notifier_call_chain(suspend_state);
	release_suspend_lock();
}

void suspend_syscore_ops_shutdown(void)
{
	hold_suspend_lock();
	suspend_state = STATE_CORE_SHUTDOWN;
	suspend_notifier_call_chain(suspend_state);
	release_suspend_lock();
}

static struct syscore_ops suspend_syscore_ops = {
	.suspend = suspend_syscore_ops_syspend,
	.resume = suspend_syscore_ops_resume,
	.shutdown = suspend_syscore_ops_shutdown,
};

int __init suspend_state_init(void) {
	int ret;

	ret = register_pm_notifier(&pm_notifier_t);
	BUG_ON(ret);
	register_syscore_ops(&suspend_syscore_ops);

	return 0;
}
arch_initcall(suspend_state_init);

void __exit suspend_state_exit(void) {
	unregister_pm_notifier(&pm_notifier_t);
	unregister_syscore_ops(&suspend_syscore_ops);
}
module_exit(suspend_state_exit);
