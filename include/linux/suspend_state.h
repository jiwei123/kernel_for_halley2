#ifndef _SUSPEND_STATE_H_
#define _SUSPEND_STATE_H_

enum {
	STATE_NO_SUSPEND,
	STATE_EARLYSUSPEND_ING,
	STATE_LATERESUME_ING,
	STATE_EARLYSUSPEND,
	STATE_SUSPEND_ING,
	STATE_RESUME_ING,
	STATE_SUSPEND,
	STATE_CORE_SYSPEND,
	STATE_CORE_RESUME,
	STATE_CORE_SHUTDOWN,
};

extern void hold_suspend_lock(void);
extern void release_suspend_lock(void);
extern unsigned int get_suspend_state_with_lock(void);
extern unsigned int get_suspend_state_no_lock(void);

/*
 * static int notifier_call_func(struct notifier_block *nb, unsigned long msg, void *data) {
 *     return NOTIFY_DONE;
 * };
 * static struct notifier_block sample_notifier = {
 *     .notifier_call = notifier_call_func,
 *     .priority = 0,
 * };
 * register_suspend_notifier(&sample_notifier);
 **/
int register_suspend_notifier(struct notifier_block *);
int unregister_suspend_notifier(struct notifier_block *);

#ifdef CONFIG_SUSPEND_STATE
extern void fb_early_notify_suspend_state(int event);
extern void fb_late_notify_suspend_state(int event);
#else
static inline void fb_early_notify_suspend_state(int event) {}
static inline void fb_late_notify_suspend_state(int event) {}
#endif

#endif /* _SUSPEND_STATE_H_ */

