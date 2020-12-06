#ifndef INCLUDE_LINUX_CAVE_DATA_H_
#define INCLUDE_LINUX_CAVE_DATA_H_

#ifdef CONFIG_CAVE

struct cave_context {
	long voffset;
};

typedef struct cave_data {
	struct cave_context kernel_ctx;
	struct cave_context user_ctx;
#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
	struct cave_context orig_kernel_ctx;
	unsigned long syscall_nr;
#endif
	bool user_defined;
	spinlock_t lock;
} cave_data_t;

#define CAVE_NOMINAL_VOFFSET	0

#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
#define INIT_TASK_SYSCALL_CONTEXT				\
	.orig_kernel_ctx = { .voffset = CAVE_NOMINAL_VOFFSET }, \
		.syscall_nr = 0,
#else
#define INIT_TASK_SYSCALL_CONTEXT
#endif

#define INIT_TASK_CAVE							\
	.cave_data = {							\
		.kernel_ctx = { .voffset = CAVE_NOMINAL_VOFFSET },	\
		.user_ctx = { .voffset = CAVE_NOMINAL_VOFFSET },	\
		INIT_TASK_SYSCALL_CONTEXT				\
		.user_defined = false,					\
		.lock = __SPIN_LOCK_UNLOCKED(tsk.cave_data.lock)	\
	},

#else
#define INIT_TASK_CAVE
#endif

#endif /* INCLUDE_LINUX_CAVE_DATA_H_ */
