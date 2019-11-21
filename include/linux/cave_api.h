#ifndef INCLUDE_LINUX_CAVE_H_
#define INCLUDE_LINUX_CAVE_H_

struct task_struct;

#ifdef CONFIG_UNISERVER_CAVE

void cave_copy_task(struct task_struct *p, struct task_struct *parent);
void cave_init_userspace(void);
void cave_context_switch_voltage(struct task_struct *prev, struct task_struct *next);

#else
static void cave_copy_task(task_struct *p, struct task_struct *parent)
{
}

static void cave_init_userspace(void)
{
}

static void cave_context_switch_voltage(struct task_struct *prev, struct task_struct *next)
{
}

#endif

#endif /* INCLUDE_LINUX_CAVE_H_ */
