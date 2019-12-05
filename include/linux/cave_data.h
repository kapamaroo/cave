#ifndef INCLUDE_LINUX_CAVE_DATA_H_
#define INCLUDE_LINUX_CAVE_DATA_H_

#ifdef CONFIG_UNISERVER_CAVE

struct cave_context {
	long voffset;
};

typedef struct cave_data {
	struct cave_context kernel_ctx;
	struct cave_context user_ctx;
	bool skip_default_user_context;
} cave_data_t;

#define CAVE_NOMINAL_VOFFSET	0
#define CAVE_DEFAULT_KERNEL_VOFFSET	CONFIG_UNISERVER_CAVE_DEFAULT_KERNEL_VOFFSET

#define INIT_TASK_CAVE							\
	.cave_data = {							\
		.kernel_ctx = { .voffset = CAVE_NOMINAL_VOFFSET },	\
		.user_ctx = { .voffset = CAVE_NOMINAL_VOFFSET },	\
		.skip_default_user_context = false			\
	},

#else
#define INIT_TASK_CAVE
#endif

#endif /* INCLUDE_LINUX_CAVE_DATA_H_ */
