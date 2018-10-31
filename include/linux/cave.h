#ifndef INCLUDE_LINUX_CAVE_H_
#define INCLUDE_LINUX_CAVE_H_

struct seq_file;
struct task_struct;

typedef struct cave_data {
	long voltage;
} cave_data_t;

void cave_set_task(struct task_struct *p);

#endif /* INCLUDE_LINUX_CAVE_H_ */
