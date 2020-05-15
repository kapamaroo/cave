#ifndef INCLUDE_LINUX_CAVE_H_
#define INCLUDE_LINUX_CAVE_H_

struct task_struct;

#ifdef CONFIG_UNISERVER_CAVE

void cave_exec_task(struct task_struct *p);

void cave_context_switch_voltage(struct task_struct *prev, struct task_struct *next);

void cave_fork_init(struct task_struct *p);

#else
void cave_exec_task(struct task_struct *p)
{
}

static void cave_context_switch_voltage(struct task_struct *prev, struct task_struct *next)
{
}

static void cave_fork_init(struct task_struct *p)
{
}

#endif

#endif /* INCLUDE_LINUX_CAVE_H_ */
