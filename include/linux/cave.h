#ifndef INCLUDE_LINUX_CAVE_H_
#define INCLUDE_LINUX_CAVE_H_

#ifdef CONFIG_UNISERVER_CAVE

#include <linux/cave_data.h>

#define CAVE_DEFAULT_KERNEL_VOFFSET	CONFIG_UNISERVER_CAVE_DEFAULT_KERNEL_VOFFSET

#define INIT_TASK_CAVE							\
	.cave_data = { .voffset = CAVE_DEFAULT_KERNEL_VOFFSET },

#else
#define INIT_TASK_CAVE
#endif

#endif /* INCLUDE_LINUX_CAVE_H_ */
