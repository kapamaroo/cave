#ifndef INCLUDE_LINUX_CAVE_H_
#define INCLUDE_LINUX_CAVE_H_

struct task_struct;

#ifdef CONFIG_UNISERVER_CAVE

void cave_exec_task(struct task_struct *p);
void cave_context_switch_voltage(struct task_struct *prev, struct task_struct *next);

#else
void cave_exec_task(struct task_struct *p)
{
}

static void cave_context_switch_voltage(struct task_struct *prev, struct task_struct *next)
{
}

#endif

#endif /* INCLUDE_LINUX_CAVE_H_ */
