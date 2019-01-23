#ifndef INCLUDE_LINUX_CAVE_H_
#define INCLUDE_LINUX_CAVE_H_

#ifdef CONFIG_UNISERVER_CAVE

struct seq_file;
struct task_struct;

#include <linux/cave_data.h>

#define CAVE_NOMINAL_VOLTAGE		4000ULL
#define CAVE_DEFAULT_KERNEL_VOFFSET	CONFIG_UNISERVER_CAVE_DEFAULT_KERNEL_VOFFSET

#define VOFFSET_OF(__voltage)	((long)CAVE_NOMINAL_VOLTAGE - (long)(__voltage))
#define VOLTAGE_OF(__voffset)	((long)CAVE_NOMINAL_VOLTAGE - (long)(__voffset))

#define INIT_TASK_CAVE							\
	.cave_data = { .voltage = VOLTAGE_OF(CAVE_DEFAULT_KERNEL_VOFFSET) },

void cave_set_task(struct task_struct *p, struct task_struct *parent);
void cave_set_init_task(void);

#else
#define INIT_TASK_CAVE

static void cave_set_task(task_struct *p)
{
}

static void cave_set_init_task(void)
{
}

#endif

#endif /* INCLUDE_LINUX_CAVE_H_ */
