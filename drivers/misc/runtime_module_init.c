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

struct runtime_module_init_task {
	struct list_head	node;
	initcall_t fn;
	struct completion	done;
	int ret;
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

	return 0;
}

static int do_one_runtime_module_initcall(void *data) {
	struct runtime_module_init_task *runtime_task = data;
	if (initcall_debug) {
		runtime_task->ret = !!do_one_initcall_debug(runtime_task->fn);
	} else {
		runtime_task->ret = !!runtime_task->fn();
	}
	complete(&runtime_task->done);
	return 0;
}

int do_runtime_module_initcalls(void) {
	initcall_t *fn;
	initcall_t *start = (void *)&__runtime_module_init_start;
	initcall_t *end = (void *)&__runtime_module_init_end;
	int i = 0;
	ktime_t calltime, delta, rettime;
	unsigned long long duration;
	struct runtime_module_init_task *task, *next;
	initcall_debug = 1;
	if (initcall_debug) {
		calltime = ktime_get();
		pr_debug("start runtime module init calls\n");
		pr_debug("calling  %pF @ %i\n", do_runtime_module_initcalls, task_pid_nr(current));
	}
	for (fn = start, i = 0; fn < end; fn++, i++) {
		task = kzalloc(sizeof(struct runtime_module_init_task), GFP_KERNEL);
		if (!task) {
			pr_err("alloc for runtime_module_init_task failed\n");
			break;
		}

		task->fn = *fn;
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
		if (task->ret) {
			pr_err("runtime init %pF failed:%d\n", task->fn, task->ret);
		}
		list_del(&task->node);
		kfree(task);
	}

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

