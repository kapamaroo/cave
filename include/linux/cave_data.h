#ifndef INCLUDE_LINUX_CAVE_DATA_H_
#define INCLUDE_LINUX_CAVE_DATA_H_

#ifdef CONFIG_UNISERVER_CAVE

struct cave_context {
	long voffset;
};

typedef struct cave_data {
	struct cave_context context;
	struct cave_context active_context;
} cave_data_t;

#define CAVE_DEFAULT_KERNEL_VOFFSET	CONFIG_UNISERVER_CAVE_DEFAULT_KERNEL_VOFFSET

#define INIT_TASK_CAVE							\
	.cave_data = { \
		.context = { .voffset = CAVE_DEFAULT_KERNEL_VOFFSET },	\
		.active_context = { .voffset = CAVE_DEFAULT_KERNEL_VOFFSET } \
	},

#else
#define INIT_TASK_CAVE
#endif

#endif /* INCLUDE_LINUX_CAVE_DATA_H_ */
