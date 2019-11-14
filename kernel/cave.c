/* CAVE - Context Aware Voltage Elevator */

#include <asm/msr.h>
#include <asm-generic/delay.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/random.h>
#include <linux/cave.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <generated/asm-offsets.h>

#define CAVE_NOMINAL_VOFFSET	0

enum cave_switch_case {
	CAVE_INC = 0,
	CAVE_DEC,
	SKIP_FAST,
	SKIP_SLOW,
	SKIP_REPLAY,
	SKIP_RACE,
	CAVE_SWITCH_CASES
};

#ifdef CONFIG_UNISERVER_CAVE_STATS

enum cave_lock_case {
	LOCK_INC = 0,
	LOCK_DEC,
	CAVE_LOCK_CASES
};

enum cave_wait_cases {
	WAIT_TARGET = 0,
	WAIT_CURR,
	CAVE_WAIT_CASES
};

struct cave_stats {
	unsigned long long time[CAVE_SWITCH_CASES + CAVE_LOCK_CASES + CAVE_WAIT_CASES];
	unsigned long long counter[CAVE_SWITCH_CASES + CAVE_LOCK_CASES + CAVE_WAIT_CASES];
	unsigned long long cycles[CAVE_SWITCH_CASES + CAVE_LOCK_CASES + CAVE_WAIT_CASES];
};

static char *cave_stat_name[CAVE_SWITCH_CASES + CAVE_LOCK_CASES + CAVE_WAIT_CASES] = {
	__stringify(CAVE_INC),
	__stringify(CAVE_DEC),
	__stringify(SKIP_FAST),
	__stringify(SKIP_SLOW),
	__stringify(SKIP_REPLAY),
	__stringify(SKIP_RACE),

	__stringify(LOCK_INC),
	__stringify(LOCK_DEC),

	__stringify(WAIT_TARGET),
	__stringify(WAIT_CURR)
};

DEFINE_PER_CPU(struct cave_stats, time_stats);

static inline void _end_measure(unsigned long long start, enum cave_switch_case c)
{
	struct cave_stats *t = this_cpu_ptr(&time_stats);

	unsigned long long time = rdtsc() - start;

	/* WARN_ON_ONCE(t->time[c] >= ULLONG_MAX - time); */

	t->time[c] += time;
	t->counter[c]++;
}

static inline void _end_lock_measure(unsigned long long start, enum cave_lock_case c)
{
	struct cave_stats *t = this_cpu_ptr(&time_stats);

	unsigned long long time = rdtsc() - start;

	/* WARN_ON_ONCE(t->time[CAVE_SWITCH_CASES + c] > ULLONG_MAX - time); */

	t->time[CAVE_SWITCH_CASES + c] += time;
	t->counter[CAVE_SWITCH_CASES + c]++;
}

#define end_measure(start, c)	_end_measure(start, c)
#else
#define end_measure(start, c)
#endif

static struct kobject *cave_kobj;
static struct kobject *syscall_enabled_kobj;

static volatile int cave_enabled = 0;
static volatile int cave_random_vmin_enabled __read_mostly = 0;

static DEFINE_SPINLOCK(cave_lock);

#define _cave_lock_1(flags)	spin_lock_irqsave(&cave_lock, flags)
#ifndef CONFIG_UNISERVER_CAVE_STATS
#define _cave_lock_2(flags, lock_case)	spin_lock_irqsave(&cave_lock, flags)
#define _cave_lock_3(flags, lock_case, start)	spin_lock_irqsave(&cave_lock, flags)
#else
#define _cave_lock_2(flags, lock_case)					\
	do {								\
		bool done = false;					\
		unsigned long long __start = rdtsc();			\
		while (!spin_trylock_irqsave(&cave_lock, flags)) 	\
			if (!done) 					\
				done = true;				\
		if (done)						\
			_end_lock_measure(__start, lock_case);		\
	} while (0)

#define _cave_lock_3(flags, lock_case, __start)				\
	do {								\
		bool done = false;					\
		while (!spin_trylock_irqsave(&cave_lock, flags)) 	\
			if (!done) 					\
				done = true;				\
		if (done)						\
			_end_lock_measure(__start, lock_case);		\
	} while (0)
#endif

#define GET_MACRO(_1, _2, _3, NAME, ...) NAME

#define cave_lock(...)	GET_MACRO(__VA_ARGS__, _cave_lock_3, _cave_lock_2, _cave_lock_1, fn0)(__VA_ARGS__)
#define cave_unlock(flags)	spin_unlock_irqrestore(&cave_lock, flags)

static volatile int cave_max_voffset __read_mostly = 400;
static volatile int cave_kernel_voffset __read_mostly = CAVE_DEFAULT_KERNEL_VOFFSET;
static volatile int cave_syscall_voffset __read_mostly = CAVE_DEFAULT_KERNEL_VOFFSET;

#define CAVE_NOMINAL_CONTEXT	(cave_data_t){ .voffset = CAVE_NOMINAL_VOFFSET }
#define CAVE_KERNEL_CONTEXT (cave_data_t){ .voffset = cave_kernel_voffset }
#define CAVE_SYSCALL_CONTEXT (cave_data_t){ .voffset = cave_syscall_voffset }

#ifdef CONFIG_UNISERVER_CAVE_USERSPACE
#define CAVE_DEFAULT_USERSPACE_VOFFSET	CONFIG_UNISERVER_CAVE_DEFAULT_USERSPACE_VOFFSET
static volatile int cave_userspace_voffset __read_mostly = CAVE_DEFAULT_USERSPACE_VOFFSET;
#define CAVE_USERSPACE_CONTEXT	(cave_data_t){ .voffset = cave_userspace_voffset }
#else
#define CAVE_USERSPACE_CONTEXT	CAVE_NOMINAL_CONTEXT
#endif

DEFINE_PER_CPU(cave_data_t, context) = CAVE_NOMINAL_CONTEXT;

static volatile long target_voffset_cached = CAVE_NOMINAL_VOFFSET;
static volatile long curr_voffset = CAVE_NOMINAL_VOFFSET;

DECLARE_BITMAP(syscall_enabled, __NR_syscall_max);

#ifdef CONFIG_UNISERVER_CAVE_STATS
static DEFINE_SPINLOCK(cave_stat_avg_lock);
static struct cave_stats cave_stat_avg[4];
static int stat_samples[3] = { 0, 0, 0 };

static bool skip_msr __read_mostly = false;

#define FSHIFT	11
#define FIXED_1	(1 << FSHIFT)
#define STAT_INT(x)	((x) >> FSHIFT)
#define STAT_FRAC(x)	STAT_INT(((x) & (FIXED_1 - 1)) * 100)

#define CAVE_STATS_TIMER_PERIOD	1
#define CAVE_STATS_MINUTE	(60 / CAVE_STATS_TIMER_PERIOD)

static struct hrtimer stats_hrtimer;
static ktime_t stats_period_time;

static enum hrtimer_restart stats_gather(struct hrtimer *timer)
{
	unsigned long flags;
	struct cave_stats t;
	int i;
	int j;
	int cnt[CAVE_SWITCH_CASES + CAVE_LOCK_CASES + CAVE_WAIT_CASES] = { 0 };

	memset(&t, 0, sizeof(t));

	for_each_online_cpu(i) {
		struct cave_stats c;

		/* local copy */
		cave_lock(flags);
		c = *per_cpu_ptr(&time_stats, i);
		memset(per_cpu_ptr(&time_stats, i), 0, sizeof(struct cave_stats));
		cave_unlock(flags);

		for (j = 0; j < CAVE_SWITCH_CASES + CAVE_LOCK_CASES + CAVE_WAIT_CASES; j++) {
			/* WARN_ON_ONCE((c.time[j] == 0) ^ (c.counter[j] == 0)); */
			if (c.counter[j]) {
				t.time[j] += c.time[j];
				t.counter[j] += c.counter[j];
				t.cycles[j] += c.time[j] / c.counter[j];
				cnt[j]++;
			}
		}
	}

	for (j = 0; j < CAVE_SWITCH_CASES + CAVE_LOCK_CASES + CAVE_WAIT_CASES; j++) {
		if (cnt[j]) {
			t.time[j] /= cnt[j];
			t.counter[j] /= cnt[j];
			t.cycles[j] /= cnt[j];
		}
	}

#define __RUNNING_AVG_STAT(d, s, n, x)					\
	do {								\
		d.cycles[x] = (d.cycles[x] * (n) + s.cycles[x]) / ((n) + 1); \
		d.counter[x] = (d.counter[x] * (n) + s.counter[x]) / ((n) + 1); \
		d.time[x] = (d.time[x] * (n) + s.time[x]) / ((n) + 1);	\
	} while (0)

#define RUNNING_AVG_STAT(d, s, n, l)					\
	do {								\
		if (n == l)						\
			n--;						\
									\
		__RUNNING_AVG_STAT(d, s, n, CAVE_INC);			\
		__RUNNING_AVG_STAT(d, s, n, CAVE_DEC);			\
		__RUNNING_AVG_STAT(d, s, n, SKIP_FAST);			\
		__RUNNING_AVG_STAT(d, s, n, SKIP_SLOW);			\
		__RUNNING_AVG_STAT(d, s, n, SKIP_REPLAY);		\
		__RUNNING_AVG_STAT(d, s, n, SKIP_RACE);			\
		__RUNNING_AVG_STAT(d, s, n, CAVE_SWITCH_CASES + LOCK_INC); \
		__RUNNING_AVG_STAT(d, s, n, CAVE_SWITCH_CASES + LOCK_DEC); \
		__RUNNING_AVG_STAT(d, s, n, CAVE_SWITCH_CASES + CAVE_LOCK_CASES + WAIT_TARGET); \
		__RUNNING_AVG_STAT(d, s, n, CAVE_SWITCH_CASES + CAVE_LOCK_CASES + WAIT_CURR); \
		n++;							\
	} while (0)

	spin_lock_irqsave(&cave_stat_avg_lock, flags);
	cave_stat_avg[0] = t;
	RUNNING_AVG_STAT(cave_stat_avg[1], t, stat_samples[0], 1 * CAVE_STATS_MINUTE);
	/* RUNNING_AVG_STAT(cave_stat_avg[2], t, stat_samples[1], 5 * CAVE_STATS_MINUTE); */
	/* RUNNING_AVG_STAT(cave_stat_avg[3], t, stat_samples[2], 10 * CAVE_STATS_MINUTE); */
	spin_unlock_irqrestore(&cave_stat_avg_lock, flags);

#undef __RUNNING_AVG_STAT
#undef RUNNING_AVG_STAT

	hrtimer_forward_now(&stats_hrtimer, stats_period_time);

	return HRTIMER_RESTART;
}

static void stats_init(void)
{
	int i;

	for_each_possible_cpu(i) {
		struct cave_stats *t = per_cpu_ptr(&time_stats, i);
		memset(t, 0, sizeof(struct cave_stats));
	}

	memset(cave_stat_avg, 0, sizeof(cave_stat_avg));
	stat_samples[0] = 0;
	stat_samples[1] = 0;
	stat_samples[2] = 0;

	stats_period_time = ktime_set(CAVE_STATS_TIMER_PERIOD, 0);
	hrtimer_init(&stats_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stats_hrtimer.function = stats_gather;
	hrtimer_start(&stats_hrtimer, stats_period_time, HRTIMER_MODE_REL);
}

static void stats_clear(void)
{
	hrtimer_cancel(&stats_hrtimer);
}
#else

static void stats_init(void)
{
}

static void stats_clear(void)
{
}
#endif

#if 0
static void cave_check_tasks(void)
{
	struct task_struct *p;

	read_lock(&tasklist_lock);
	for_each_process(p) {
		struct cave_data *c = &p->cave_data;
		if (p->flags & PF_KTHREAD) {
			if (c->voffset != cave_kernel_voffset)
				pr_warn("cave: kthread %s with %d voffset\n",
					p->comm, c->voffset);
		}
		else {
			if (c->voffset == cave_kernel_voffset)
				pr_warn("cave: user thread %s with %d voffset\n",
					p->comm, c->voffset);
		}
	}
	read_unlock(&tasklist_lock);
}
#endif

/* Arch specific */

#define TO_VOFFSET_DATA(__val)	(__val ? (0x800ULL - (u64)__val) << 21 : 0ULL)
#define TO_VOFFSET_VAL(__data)    (__data ? (0x800ULL - ((u64)__data >> 21)) : 0ULL)

#define CORE_VOFFSET_VAL(__val)		(0x8000001100000000ULL | TO_VOFFSET_DATA(__val))
#define CACHE_VOFFSET_VAL(__val)	(0x8000021100000000ULL | TO_VOFFSET_DATA(__val))

static inline void write_voffset_msr(u64 voffset)
{
	if (unlikely(skip_msr))
		return;

	wrmsrl(0x150, CORE_VOFFSET_VAL(voffset));
	wrmsrl(0x150, CACHE_VOFFSET_VAL(voffset));
}

static inline u64 read_voffset_msr(void)
{
	u64 voffset;

	if (unlikely(skip_msr))
		return target_voffset_cached;

	wrmsrl(0x150, 0x8000001000000000);
	rdmsrl(0x150, voffset);

	return TO_VOFFSET_VAL(voffset);
}

/* ************************************************************************** */

static void write_target_voffset(long new_voffset)
{
	target_voffset_cached = new_voffset;
}

static void write_curr_voffset(long new_voffset)
{
	curr_voffset = new_voffset;
}

static void write_voffset(long voffset)
{
	write_target_voffset(voffset);
	write_curr_voffset(voffset);
	write_voffset_msr(voffset);
}

static long read_target_voffset(void)
{
	return target_voffset_cached;
}

static inline void wait_curr_voffset(long new_voffset)
{
#ifdef CONFIG_UNISERVER_CAVE_STATS
	unsigned long long start;
	struct cave_stats *t = this_cpu_ptr(&time_stats);

	if (new_voffset >= curr_voffset)
		return;

	start = rdtsc();
#endif

	while (new_voffset < curr_voffset)
		cpu_relax();

#ifdef CONFIG_UNISERVER_CAVE_STATS
	t->time[CAVE_SWITCH_CASES + CAVE_LOCK_CASES + WAIT_CURR] += rdtsc() - start;
	t->counter[CAVE_SWITCH_CASES + CAVE_LOCK_CASES + WAIT_CURR]++;
#endif
}

static inline void wait_target_voffset(long new_voffset)
{
#ifdef CONFIG_UNISERVER_CAVE_STATS
	unsigned long long start;
	struct cave_stats *t = this_cpu_ptr(&time_stats);

	start = rdtsc();
#endif

	while (new_voffset < (curr_voffset = read_voffset_msr()))
		cpu_relax();

#ifdef CONFIG_UNISERVER_CAVE_STATS
	t->time[CAVE_SWITCH_CASES + CAVE_LOCK_CASES + WAIT_TARGET] += rdtsc() - start;
	t->counter[CAVE_SWITCH_CASES + CAVE_LOCK_CASES + WAIT_TARGET]++;
#endif
}

static long select_voffset(void)
{
	long new_voffset = LONG_MAX;
	int i;

	for_each_online_cpu(i) {
		cave_data_t tmp = per_cpu(context, i);
		if(tmp.voffset < new_voffset)
			new_voffset = tmp.voffset;
	}

	return new_voffset;
}

static inline void _cave_switch(cave_data_t new_context)
{
	unsigned long flags;
	long new_voffset = new_context.voffset;
	long target_voffset;
	long updated_voffset;
	long selected_voffset;
	static int switch_path_contention = 0;

#ifdef CONFIG_UNISERVER_CAVE_STATS
	unsigned long long start;
#endif

	/* Fast path: check cave_enable outside of the cave_lock.
	 * It also covers the increase path if the system is subject to
	 * undervolting only.
	 */
	if (!cave_enabled)
		return;

#ifdef CONFIG_UNISERVER_CAVE_STATS
	start = rdtsc();
#endif

	cave_lock(flags, LOCK_INC, start);

	if (!cave_enabled) {
		cave_unlock(flags);
		return;
	}

	this_cpu_write(context, new_context);
	target_voffset = read_target_voffset();

	/* We may try to increase voltage just after cave is disabled,
	 * see cave_enabled comment above.
	 * It is safe to enter the increase path as soon as we undervolt only,
	 * if cave is disabled then target_voffset is nominal (lowest possible)
	 * and we skip the increase path.
	 */

	/* It is cleaner to just check cave_enabled in the cave_lock but prefer
	 * this detection method of cave disable as it has less overhead for the
	 * common case.
	 */

	/* increase voltage immediately */
	if (new_voffset < target_voffset) {
		write_target_voffset(new_voffset);
		write_voffset_msr(new_voffset);

		cave_unlock(flags);

		wait_target_voffset(new_voffset);
		end_measure(start, CAVE_INC);

		return;
	}

	if (new_voffset == target_voffset) {
		cave_unlock(flags);
		wait_curr_voffset(new_voffset);
		end_measure(start, SKIP_FAST);
		return;
	}

	switch_path_contention++;

	cave_unlock(flags);

	/* When more than one increases happen in a row, the smaller increases
	 * wait for the curr_voffset to match the new_voffset.
	 *
	 * This CPU wants to decrease the voltage but still has to wait for the
	 * increase to take place.
	 *
	 * It also considers the case where more than one CPUs try to set the
	 * same voltage.
	 */
	wait_curr_voffset(new_voffset);

	/* new_voffset > target_voffset */

	cave_lock(flags, LOCK_DEC);

	switch_path_contention--;

	if (!cave_enabled) {
		cave_unlock(flags);
		return;
	}

	/* Skip cascade decreases of voltage from many CPUs */
	if (switch_path_contention) {
		cave_unlock(flags);
		end_measure(start, SKIP_REPLAY);
		return;
	}

	selected_voffset = select_voffset();
	updated_voffset = read_target_voffset();

	if (selected_voffset == updated_voffset) {
		cave_unlock(flags);
		end_measure(start, SKIP_SLOW);
		return;
	}

	if (selected_voffset < updated_voffset) {
		cave_unlock(flags);
		end_measure(start, SKIP_RACE);
		return;
	}

	/* the curr_voffset is usefull only in the increase path to protect
	 * successive increases, set it to the target_voffset on decrease
	 * for cosistency (curr_voffset is always >= target_voffset)
	 *
	 * Note that curr_voffset also changes in wait_target_voffset() which is
	 * not lock protected. However, when a CPU waits for the voltage to
	 * increase, it prohibits another CPU from decreasing the voltage. Also
	 * when a CPU decreases the voltage it holds the lock, therefore it
	 * prohibits another CPU to increase the voltage.
	 */
	write_voffset(selected_voffset);
	cave_unlock(flags);
	end_measure(start, CAVE_DEC);
}

__visible void cave_syscall_entry_switch(unsigned long syscall_nr)
{
	cave_data_t syscall_entry_context = CAVE_KERNEL_CONTEXT;

	if (test_bit(syscall_nr, syscall_enabled))
		syscall_entry_context = CAVE_SYSCALL_CONTEXT;

	_cave_switch(syscall_entry_context);
}

__visible void cave_entry_switch(void)
{
	_cave_switch(CAVE_KERNEL_CONTEXT);
}

__visible void cave_exit_switch(void)
{
	_cave_switch(current->cave_data);
}

static void __cave_set_task(struct task_struct *p, long voffset)
{
	p->cave_data.voffset = voffset;
}

static void _cave_set_task(struct task_struct *p, long voffset)
{
	if (voffset < 0 || voffset > cave_max_voffset) {
		pr_warn("cave: voffset out of range (%ld) comm=%s\n", voffset, p->comm);
		voffset = 0;
	}

	__cave_set_task(p, voffset);
}

/* should be protected by tasklist_lock */
void cave_copy_task(struct task_struct *p, struct task_struct *parent)
{
	unsigned long voffset = parent->cave_data.voffset;

	if (voffset < 0 || voffset > cave_max_voffset)
		pr_warn("cave: pid %d comm=%s with no vmin", task_tgid_vnr(parent), parent->comm);

	if (cave_random_vmin_enabled && !(p->flags & PF_KTHREAD))
		voffset = get_random_long() % cave_max_voffset;

	__cave_set_task(p, voffset);
}

void cave_init_userspace(void)
{
	struct task_struct *p = current;

	p->cave_data = CAVE_USERSPACE_CONTEXT;
}

#ifdef CONFIG_UNISERVER_CAVE_STATS
static int _print_cave_stats(char *buf, struct cave_stats *t, const int t_min)
{
	int ret = 0;

#define SEPARATOR()	ret += sprintf(buf + ret, "\n")
#define F(x, t) 100 * ((x) << FSHIFT) / (t)
#define S(x, t)	STAT_INT(F(x, t)), STAT_FRAC(F(x, t))
#define FMT	"%llu.%02llu"

	int j;
	unsigned long long time = 0;
	unsigned long long counter = 0;
	unsigned long long cycles = 0;
	int cnt = 0;

	for (j = 0; j < CAVE_SWITCH_CASES; j++) {
		if (t->counter[j]) {
			time += t->time[j];
			counter += t->counter[j];
			cycles += t->cycles[j];
			cnt++;
		}
	}

	if (time == 0 || counter == 0)
	        return ret;

	ret += sprintf(buf + ret, "%dmin_stats\n", t_min);
	ret += sprintf(buf + ret, "\nOverhead_delay cycles_avg  time_avg%% counter_avg%%\n");
	ret += sprintf(buf + ret, "total_avg %llu 100.00 100.00\n", time / counter);

	for (j = 0; j < CAVE_SWITCH_CASES + CAVE_LOCK_CASES + CAVE_WAIT_CASES; j++) {
		if (j == SKIP_FAST || j == CAVE_SWITCH_CASES || j == CAVE_SWITCH_CASES + CAVE_LOCK_CASES)
			SEPARATOR();
		if (t->counter[j] == 0)
			continue;
		ret += sprintf(buf + ret, "%s %llu " FMT " " FMT
			       "\n", cave_stat_name[j],
			       t->cycles[j],
			       S(t->time[j], time),
			       S(t->counter[j], counter)
			       );
	}

	if (t->time[CAVE_INC] != 0 || time != t->time[CAVE_INC])
		ret += sprintf(buf + ret, "\nVoltage_delay time_avg%%\n");

	if (t->time[CAVE_INC] != 0) {
		ret += sprintf(buf + ret, "wait_target/inc " FMT "\n",
			       S(t->time[CAVE_SWITCH_CASES + CAVE_LOCK_CASES + WAIT_TARGET], t->time[CAVE_INC]));
	}

	if (time && time != t->time[CAVE_INC]) {
		ret += sprintf(buf + ret, "wait_curr/eq_dec " FMT "\n",
			       S(t->time[CAVE_SWITCH_CASES + CAVE_LOCK_CASES + WAIT_CURR], time - t->time[CAVE_INC]));
	}

	SEPARATOR();

#undef FMT
#undef S
#undef F
#undef SEPARATOR

	return ret;
}

static int print_cave_stats(char *buf)
{
	unsigned long flags;
	int ret = 0;
	int i;
	static struct cave_stats tmp_stat[4];
	const int w[4] = { 0, 1, 5, 10 };

	spin_lock_irqsave(&cave_stat_avg_lock, flags);
	tmp_stat[0] = cave_stat_avg[0];
	tmp_stat[1] = cave_stat_avg[1];
	/* tmp_stat[2] = cave_stat_avg[2]; */
	/* tmp_stat[3] = cave_stat_avg[3]; */
	spin_unlock_irqrestore(&cave_stat_avg_lock, flags);

	if (cave_enabled)
		for (i = 0; i < 2; i++)
			ret += _print_cave_stats(buf + ret, &tmp_stat[i], w[i]);

	return ret;
}
#else
static int print_cave_stats(char *buf, const bool raw)
{
	int ret = 0;
	ret += sprintf(buf + ret, "kernel build without stats\n");
	return ret;
}
#endif

/* sysfs interface */
#define KERNEL_ATTR_RW(_name)						\
	static struct kobj_attribute _name##_attr = __ATTR_RW(_name)

#define KERNEL_ATTR_RO(_name)						\
	static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

#define KERNEL_ATTR_WO(_name)						\
	static struct kobj_attribute _name##_attr = __ATTR_WO(_name)

static
ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%d\n", cave_enabled);

	return ret;
}

static
ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
		     const char *buf, size_t count)
{
	unsigned long flags;
	int enable;
	int err;
	bool show_msg = false;

	err = kstrtouint(buf, 10, &enable);
	if (err || (enable != 0 && enable != 1)) {
		pr_warn("cave: invalid %s value\n", attr->attr.name);
		return count;
	}

	if (enable) {
		cave_lock(flags);
		if (!cave_enabled) {
			// this_cpu_write(context, CAVE_NOMINAL_CONTEXT);
			write_voffset(CAVE_NOMINAL_VOFFSET);
			stats_init();
			bitmap_fill(syscall_enabled, __NR_syscall_max);
			cave_enabled = 1;
			show_msg = true;
		}
		cave_unlock(flags);
		if (show_msg)
			pr_info("cave: enabled\n");
	}
	else {
		cave_lock(flags);
		if (cave_enabled) {
			cave_enabled = 0;
			bitmap_zero(syscall_enabled, __NR_syscall_max);
			stats_clear();
			/* in case we race with a CPU on the decrease path,
			 * set context to nominal to avoid decrease in voltage
			 * below nominal. select_voffset() chooses the most
			 * constrained voffset (nominal).
			 */
			// this_cpu_write(context, CAVE_NOMINAL_CONTEXT);
			write_voffset(CAVE_NOMINAL_VOFFSET);
			show_msg = true;
		}
		cave_unlock(flags);
		if (show_msg)
			pr_info("cave: disabled\n");
	}

	return count;
}

static
ssize_t random_vmin_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%d\n", cave_random_vmin_enabled);

	return ret;
}

static
ssize_t random_vmin_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	int enable;
	int err;

	err = kstrtouint(buf, 10, &enable);
	if (err || (enable != 0 && enable != 1)) {
		pr_warn("cave: invalid %s value\n", attr->attr.name);
		return count;
	}

	if (enable) {
		if (!cave_random_vmin_enabled)
			cave_random_vmin_enabled = 1;
	}
	else {
		if (cave_random_vmin_enabled)
			cave_random_vmin_enabled = 0;
	}

	return count;
}

static
ssize_t max_voffset_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%d\n", cave_max_voffset);

	return ret;
}

static
ssize_t max_voffset_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t count)
{
	unsigned long flags;
	int voffset;
	int err;

	err = kstrtouint(buf, 10, &voffset);
	if (err) {
		pr_warn("cave: invalid %s value\n", attr->attr.name);
		return count;
	}

	cave_lock(flags);
	if (voffset < cave_kernel_voffset)
		pr_warn("cave: new value of max_voffset greater than kernel voffset\n");
#ifdef CONFIG_UNISERVER_CAVE_USERSPACE
	else if (voffset < cave_userspace_voffset)
		pr_warn("cave: new value of max_voffset greater than userspace voffset\n");
#endif
	else
		cave_max_voffset = voffset;
	cave_unlock(flags);
	return count;
}

static
ssize_t kernel_voffset_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%d\n", cave_kernel_voffset);

	return ret;
}

static
ssize_t kernel_voffset_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	struct task_struct *g, *p;
	unsigned long flags;
	int voffset;
	int err;
	int i;

	err = kstrtouint(buf, 10, &voffset);
	if (err) {
		pr_warn("cave: invalid %s value\n", attr->attr.name);
		return count;
	}

	if (voffset > cave_max_voffset) {
		pr_warn("cave: %s out of range\n", attr->attr.name);
		return count;
	}

	cave_lock(flags);

	if (cave_enabled) {
		pr_warn("cave: must be disabled to change kernel voffset\n");
		goto out;
	}
	cave_kernel_voffset = voffset;

	write_lock_irq(&tasklist_lock);
	for_each_possible_cpu(i)
		idle_task(i)->cave_data = CAVE_KERNEL_CONTEXT;
	for_each_process_thread(g, p)
		if (p->flags & PF_KTHREAD)
			p->cave_data = CAVE_KERNEL_CONTEXT;
	write_unlock_irq(&tasklist_lock);
 out:
	cave_unlock(flags);

	return count;
}

static
ssize_t syscall_voffset_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%d\n", cave_syscall_voffset);

	return ret;
}

static
ssize_t syscall_voffset_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned long flags;
	int voffset;
	int err;

	err = kstrtouint(buf, 10, &voffset);
	if (err) {
		pr_warn("cave: invalid %s value\n", attr->attr.name);
		return count;
	}

	if (voffset > cave_max_voffset) {
		pr_warn("cave: %s out of range\n", attr->attr.name);
		return count;
	}

	cave_lock(flags);

	if (cave_enabled)
		pr_warn("cave: must be disabled to change syscall voffset\n");
	else
		cave_syscall_voffset = voffset;

	cave_unlock(flags);

	return count;
}

#ifdef CONFIG_UNISERVER_CAVE_USERSPACE
static
ssize_t userspace_voffset_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%d\n", cave_userspace_voffset);

	return ret;
}

static
ssize_t userspace_voffset_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	struct task_struct *g, *p;
	unsigned long flags;
	int voffset;
	int err;

	err = kstrtouint(buf, 10, &voffset);
	if (err) {
		pr_warn("cave: invalid %s value\n", attr->attr.name);
		return count;
	}

	if (voffset > cave_max_voffset) {
		pr_warn("cave: %s out of range\n", attr->attr.name);
		return count;
	}

	cave_lock(flags);

	if (cave_enabled) {
		pr_warn("cave: must be disabled to change userspace voffset\n");
		goto out;
	}
	cave_userspace_voffset = voffset;

	write_lock_irq(&tasklist_lock);
	for_each_process_thread(g, p)
		if (!(p->flags & PF_KTHREAD))
			p->cave_data = CAVE_USERSPACE_CONTEXT;
	write_unlock_irq(&tasklist_lock);
 out:
	cave_unlock(flags);

	return count;
}
#endif

static
ssize_t reset_stats_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t count)
{
	unsigned long flags;

	cave_lock(flags);
	if (cave_enabled) {
		stats_clear();
		stats_init();
	}
	cave_unlock(flags);

	return count;
}

static
ssize_t stats_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += print_cave_stats(buf);

	return ret;
}

static
ssize_t voltage_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unsigned long flags;

	int ret = 0;
	long voffset;

	cave_lock(flags);
	voffset = read_target_voffset();
	cave_unlock(flags);

	ret += sprintf(buf + ret, "voffset %ld\n", -voffset);

	return ret;
}

static
ssize_t debug_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf + ret, "skip_msr = %s\n", skip_msr ? "true" : "false");

	return ret;
}

static
ssize_t debug_store(struct kobject *kobj, struct kobj_attribute *attr,
		    const char *buf, size_t count)
{
	int val = 0;

	sscanf(buf, "skip_msr = %d", &val);
	if ((val == 1 || val == 0) && skip_msr != val) {
		skip_msr = val;
		pr_info("cave: skip_msr = %s\n", val ? "true" : "false");
	}

	return count;
}

static
ssize_t ctl_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s;
	int ret;

	s = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!s)
		return 0;
	bitmap_print_to_pagebuf(false, s, syscall_enabled, __NR_syscall_max);
	ret = sprintf(buf,"cave: syscall bitmap: %s\n", s);
	kfree(s);

	return ret;
}

static
ssize_t ctl_store(struct kobject *kobj, struct kobj_attribute *attr,
		  const char *buf, size_t count)
{
	int ret = 0;
	DECLARE_BITMAP(tmp, __NR_syscall_max);

	if (strncmp(buf, "syscall:enable:", 15) == 0) {
		if (strncmp(buf + 15, "all", 3) == 0 && buf[18] == '\n') {
			bitmap_fill(syscall_enabled, __NR_syscall_max);
			return count;
		}

		if (buf[15] == '\n') {
			pr_warn("cave: ctl: invalid bitmap parselist\n");
			return count;
		}

		ret = bitmap_parselist(buf + 15, tmp, __NR_syscall_max);
		if (ret == 0) {
			bitmap_or(syscall_enabled,
				  syscall_enabled, tmp, __NR_syscall_max);
			return count;
		}
	}
	else if (strncmp(buf, "syscall:disable:", 16) == 0) {
		if (strncmp(buf + 16, "all", 3) == 0 && buf[19] == '\n') {
			bitmap_zero(syscall_enabled, __NR_syscall_max);
			return count;
		}

		if (buf[16] == '\n') {
			pr_warn("cave: ctl: invalid bitmap parselist\n");
			return count;
		}

		ret = bitmap_parselist(buf + 16, tmp, __NR_syscall_max);
		if (ret == 0) {
			bitmap_andnot(syscall_enabled,
				      syscall_enabled, tmp, __NR_syscall_max);
			return count;
		}
	}

	pr_warn("cave: ctl: invalid bitmap parselist\n");

	return count;
}

KERNEL_ATTR_RW(enable);
KERNEL_ATTR_RO(stats);
KERNEL_ATTR_WO(reset_stats);
KERNEL_ATTR_RO(voltage);
KERNEL_ATTR_RW(random_vmin_enable);
KERNEL_ATTR_RW(max_voffset);
KERNEL_ATTR_RW(kernel_voffset);
KERNEL_ATTR_RW(syscall_voffset);
#ifdef CONFIG_UNISERVER_CAVE_USERSPACE
KERNEL_ATTR_RW(userspace_voffset);
#endif
KERNEL_ATTR_RW(debug);
KERNEL_ATTR_RW(ctl);

#define HELPERS(__nr, __sys)						\
	static								\
	ssize_t __sys##_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) \
	{								\
		int ret = 0;						\
									\
		ret = sprintf(buf, "%d\n",				\
			      test_bit(__nr, syscall_enabled) ? 1 : 0);	\
									\
		return ret;						\
	}								\
									\
	static								\
	ssize_t __sys##_store(struct kobject *kobj, struct kobj_attribute *attr, \
			  const char *buf, size_t count)		\
	{								\
		int err;						\
		int enable;						\
									\
		err = kstrtouint(buf, 10, &enable);			\
		if (err || (enable != 0 && enable != 1)) {		\
			pr_warn("cave: syscall:enable:%s invalid %s value\n", #__sys, attr->attr.name); \
			return count;					\
		}							\
									\
									\
		if (enable)						\
			set_bit(__nr, syscall_enabled);			\
		else							\
			clear_bit(__nr, syscall_enabled);		\
		return count;						\
	}								\

#define __SYSCALL_64(__nr, __sys, __qual)	\
	HELPERS(__nr, __sys)			\
	KERNEL_ATTR_RW(__sys);			\

#include <generated/asm/syscalls_64.h>

#undef HELPERS
#undef __SYSCALL_64

static struct attribute_group syscall_enabled_attr_group = {
	.attrs = (struct attribute * []) {
#define __SYSCALL_64(__nr, __sys, __qual)	&__sys##_attr.attr,
#include <generated/asm/syscalls_64.h>
#undef __SYSCALL_64
		NULL
	}
};

static struct attribute_group attr_group = {
	.attrs = (struct attribute * []) {
		&enable_attr.attr,
		&reset_stats_attr.attr,
		&stats_attr.attr,
		&voltage_attr.attr,
		&random_vmin_enable_attr.attr,
		&max_voffset_attr.attr,
		&kernel_voffset_attr.attr,
		&syscall_voffset_attr.attr,
#ifdef CONFIG_UNISERVER_CAVE_USERSPACE
		&userspace_voffset_attr.attr,
#endif
		&debug_attr.attr,
		&ctl_attr.attr,
		NULL
	}
};

int cave_init(void)
{
	unsigned long flags;
	int err;
	long voffset;

	cave_kobj = kobject_create_and_add("cave", kernel_kobj);
	if (!cave_kobj) {
		pr_err("cave: failed\n");
		return -ENOMEM;
	}

	syscall_enabled_kobj = kobject_create_and_add("syscall_enabled", cave_kobj);
	if (!syscall_enabled_kobj) {
		pr_err("cave: failed\n");
		return -ENOMEM;
	}

	err = sysfs_create_group(cave_kobj, &attr_group);
	if (err) {
		pr_err("cave: failed\n");
		return err;
	}

	err = sysfs_create_group(syscall_enabled_kobj, &syscall_enabled_attr_group);
	if (err) {
		pr_err("cave: failed\n");
		return err;
	}

	cave_lock(flags);

	voffset = read_voffset_msr();
	write_voffset(CAVE_NOMINAL_VOFFSET);
	cave_unlock(flags);

	pr_info("cave: msr offset: %ld\n", -voffset);

	return 0;
}
late_initcall(cave_init);

#define CAVE_SET_TASK_VOFFSET	128

SYSCALL_DEFINE3(uniserver_ctl, int, action, int, op1, int, op2)
{
	struct task_struct *p = current;

	switch (action) {
	case CAVE_SET_TASK_VOFFSET:
		_cave_set_task(p, op2);
		/*
		pr_info("cave: pid %d voffset: %3ld\n",
		       task_tgid_vnr(p), -p->cave_data.voffset);
		*/
		return 0;
	}

	return -1;
}
