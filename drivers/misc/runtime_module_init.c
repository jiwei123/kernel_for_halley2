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
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/kthread.h>

extern unsigned long __runtime_module_init_start;
extern unsigned long __runtime_module_init_end;
struct mutex runtime_module_init_lock;
initcall_t *runtime_module_init_next = NULL;

struct runtime_module_init_task {
	struct list_head	node;
	struct completion	done;
	struct task_struct *task;
};

static LIST_HEAD(task_list);

static int __init_or_module do_one_initcall_debug(initcall_t fn)
{
	ktime_t calltime, delta, rettime;
	unsigned long long duration;
	int ret;

	pr_debug("initcall calling  %pF @ %i\n", fn, task_pid_nr(current));
	calltime = ktime_get();
	ret = fn();
	rettime = ktime_get();
	delta = ktime_sub(rettime, calltime);
	duration = (unsigned long long) ktime_to_ns(delta) >> 10;
	pr_debug("initcall %pF returned %d after %lld msecs\n", fn,
		ret, duration >> 10);

	return ret;
}

static int do_one_runtime_module_initcall(void *data) {
	int ret = 0;
	initcall_t fn;
	struct runtime_module_init_task *runtime_task = data;

	while (1) {
		mutex_lock(&runtime_module_init_lock);

		if ((unsigned long)runtime_module_init_next >= (unsigned long)&__runtime_module_init_end) {
			mutex_unlock(&runtime_module_init_lock);
			break;
		}
		fn = *runtime_module_init_next;
		runtime_module_init_next++;
		mutex_unlock(&runtime_module_init_lock);

		if (initcall_debug) {
			ret = do_one_initcall_debug(fn);
		} else {
			ret = fn();
		}

		if (ret) {
			pr_err("runtime init %pF failed:%d\n", fn, ret);
		}
	}

	complete(&runtime_task->done);
	return 0;
}

int do_runtime_module_initcalls(void) {
	int initcall_num = 0;
	int i = 0;
	ktime_t calltime, delta, rettime;
	unsigned long long duration;
	struct runtime_module_init_task *task, *next;

	initcall_num = &__runtime_module_init_end - &__runtime_module_init_start;

	if (initcall_debug) {
		calltime = ktime_get();
		pr_debug("#####################################################\n");
		pr_debug("start runtime module init calls, total num:%d\n", initcall_num);
		pr_debug("calling  %pF @ %i\n", do_runtime_module_initcalls, task_pid_nr(current));
	}

	if (runtime_module_init_next == NULL) {
		runtime_module_init_next = (void *)&__runtime_module_init_start;
	}

	mutex_init(&runtime_module_init_lock);

	if (initcall_num > CONFIG_RUNTIME_MODULE_INIT_MAX_TASK) {
		initcall_num = CONFIG_RUNTIME_MODULE_INIT_MAX_TASK;
	}
	for (i = 0; i < initcall_num; i++) {
		task = kzalloc(sizeof(struct runtime_module_init_task), GFP_KERNEL);
		if (!task) {
			pr_err("alloc for runtime_module_init_task failed\n");
			break;
		}
		init_completion(&task->done);

		task->task = kthread_run(do_one_runtime_module_initcall, task, "runtime_init %d", i);
		if (IS_ERR(task->task)) {
			pr_err("failed to create runtime_module_init_task %d:err:%ld\n", i, PTR_ERR(task->task));
			kfree(task);
		} else {
			list_add(&task->node, &task_list);
		}
	}

	list_for_each_entry_safe(task, next, &task_list, node) {
		wait_for_completion(&task->done);
		list_del(&task->node);
		kfree(task);
	}
	runtime_module_init_next = NULL;

	if (initcall_debug) {
		rettime = ktime_get();
		delta = ktime_sub(rettime, calltime);
		duration = (unsigned long long) ktime_to_ns(delta) >> 10;
		pr_debug("initcall %pF returned %d after %lld msecs\n", do_runtime_module_initcalls,
				 0, duration >> 10);
		pr_debug("#####################################################\n");
	}
	return 0;
}

