#ifndef INCLUDE_LINUX_CAVE_H_
#define INCLUDE_LINUX_CAVE_H_

struct task_struct;

#ifdef CONFIG_UNISERVER_CAVE

void cave_copy_task(struct task_struct *p, struct task_struct *parent);
void cave_init_userspace(void);

#else
static void cave_copy_task(task_struct *p, struct task_struct *parent)
{
}

static void cave_init_userspace(void)
{
}

#endif

#endif /* INCLUDE_LINUX_CAVE_H_ */
