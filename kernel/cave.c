/* CAVE - Context Aware Voltage Elevator */

#include <asm/msr.h>
#include <asm-generic/delay.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/cave_data.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/vmalloc.h>
#include <generated/asm-offsets.h>

/* Arch specific */

#define TO_VOFFSET_DATA(__val)         (((0x800ULL - ((u64)__val)) & 0x7FF) << 21)
#define TO_VOFFSET_VAL(__data)         ((0x800ULL - ((u64)__data >> 21)) & 0x7FF)

#define CORE_VOFFSET_VAL(__val)		(0x8000001100000000ULL | TO_VOFFSET_DATA(__val))
#define CACHE_VOFFSET_VAL(__val)	(0x8000021100000000ULL | TO_VOFFSET_DATA(__val))

static inline void arch_write_voffset(u64 voffset)
{
	wrmsrl(0x150, CORE_VOFFSET_VAL(voffset));
	wrmsrl(0x150, CACHE_VOFFSET_VAL(voffset));
}

static inline u64 arch_read_voffset(void)
{
	u64 voffset;

	wrmsrl(0x150, 0x8000001000000000);
	rdmsrl(0x150, voffset);

	return TO_VOFFSET_VAL(voffset);
}

static u64 arch_read_voltage(void)
{
	u64 value;

	rdmsrl(0x198, value);
	/* Core Voltage (R/O)
	 * P-state core voltage can be computed by
	 * MSR_PERF_STATUS[47:32] * (float) 1/(2^13).
	 */
	value = (((value >> 32) & 0xFFFF) * 1000) >> 13;

	return value;
}

/* ************************************************************************** */

#define CAVE_CONTEXT(__v)	((struct cave_context){ .voffset = __v })

static volatile int cave_enabled = 0;
static bool cave_nowait __read_mostly = false;

#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
static volatile long target_voffset_cached = CAVE_NOMINAL_VOFFSET;
static volatile long curr_voffset = CAVE_NOMINAL_VOFFSET;
#endif

#ifdef CONFIG_CAVE_SKIP_ARCH_RW
static bool skip_arch __read_mostly = false;
#endif

#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
static volatile int cave_syscall_context_enabled = 0;
DECLARE_BITMAP(syscall_enabled, __NR_syscall_max);
static struct kobject *syscall_enabled_kobj;
static volatile struct cave_context cave_syscall_context __read_mostly = CAVE_CONTEXT(CAVE_NOMINAL_VOFFSET);
#endif

static inline void write_voffset(u64 voffset)
{
#ifdef CONFIG_CAVE_SKIP_ARCH_RW
	if (unlikely(skip_arch))
		return;
#endif

	arch_write_voffset(voffset);
}

static inline u64 read_voffset(void)
{
	u64 voffset;

#ifdef CONFIG_CAVE_SKIP_ARCH_RW
	if (unlikely(skip_arch)) {
#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
		return target_voffset_cached;
#else
		return this_cpu_read(context).voffset;
#endif
	}
#endif

	voffset = arch_read_voffset();

#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
	curr_voffset = voffset;
#endif

	return voffset;
}

enum reason {
	ENTRY = 0,
	EXIT,
	ENTRY_SYSCALL,
	EXIT_SYSCALL,
	CONTEXT_SWITCH,
	TRYLOCK
};

#ifdef CONFIG_CAVE_SYSCALL_RATELIMIT
struct syscall_ratelimit {
    unsigned int counter;
    int enabled;
};

DEFINE_PER_CPU(struct syscall_ratelimit, srl);

static unsigned int syscall_rate_period = MSEC_PER_SEC / 10;
static unsigned int syscall_rate_limit = 1000;
static bool cave_ratelimit __read_mostly = false;

static struct hrtimer ratelimit_hrtimer;
static ktime_t ratelimit_period_time;

static inline int __syscall_rate_work(struct syscall_ratelimit *p)
{
        unsigned int rate = p->counter * (MSEC_PER_SEC / syscall_rate_period);
        int enabled = p->enabled;

        p->counter = 0;

        if (rate > syscall_rate_limit) {
		if (enabled > 0) {
			/*
                         * transition value from user -> kernel context
                         * see entry point for details
                         */
			p->enabled = -1;

			return rate;
                }
        }
        else {
		if (enabled <= 0) {
			p->enabled = 1;

			return -rate;
                }
        }

	return 0;
}

static enum hrtimer_restart ratelimit_work(struct hrtimer *timer)
{
	int i;

	for_each_online_cpu(i) {
		struct syscall_ratelimit *p = per_cpu_ptr(&srl, i);
		int rate = __syscall_rate_work(p);
		pr_warn("cave: syscall rate: %s cave on cpu%d (limit=%d/%u / sec, period=%u ms)\n",
			(rate > 0) ? "disable" : "enable",
			i, rate, syscall_rate_limit, syscall_rate_period);
	}

	hrtimer_forward_now(&ratelimit_hrtimer, ratelimit_period_time);

	return HRTIMER_RESTART;
}

static void syscall_ratelimit_init(void)
{
	int i;

	if (!cave_ratelimit)
		return;

	for_each_online_cpu(i) {
		struct syscall_ratelimit *p = per_cpu_ptr(&srl, i);
		p->counter = 0;
		p->enabled = 1;
	}

	ratelimit_period_time = ms_to_ktime(syscall_rate_period);
	hrtimer_init(&ratelimit_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ratelimit_hrtimer.function = ratelimit_work;
	hrtimer_start(&ratelimit_hrtimer, ratelimit_period_time, HRTIMER_MODE_REL);

	pr_info("cave: ratelimit: enable (limit=%u, period=%d)\n",
		syscall_rate_limit, syscall_rate_period);
}

static void syscall_ratelimit_clear(void)
{
	int i;

	if (!cave_ratelimit)
		return;

	hrtimer_cancel(&ratelimit_hrtimer);

	for_each_online_cpu(i) {
		struct syscall_ratelimit *p = per_cpu_ptr(&srl, i);
		p->counter = 0;
		p->enabled = 1;
        }

	pr_info("cave: ratelimit: disable\n");
}
#else
static void syscall_ratelimit_init(void)
{
}

static void syscall_ratelimit_clear(void)
{
}
#endif

#ifdef CONFIG_CAVE_STATS

enum cave_stat_idx {
	C_STATS_START = 0,

	C_SWITCH_CASES_START = C_STATS_START,
	CAVE_INC = C_SWITCH_CASES_START,
	CAVE_DEC,
	SKIP_FAST,
#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
	SKIP_SLOW,
	SKIP_REPLAY,
	SKIP_RACE,
#endif
	C_SWITCH_CASES_END,

	C_TRYLOCK_CASES_START = C_SWITCH_CASES_END,
#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
	TRYLOCK_INC = C_TRYLOCK_CASES_START,
	TRYLOCK_DEC,
	C_TRYLOCK_CASES_END,
#else
	C_TRYLOCK_CASES_END = C_TRYLOCK_CASES_START,
#endif

	C_WAIT_CASES_START = C_TRYLOCK_CASES_END,
	WAIT_TARGET = C_WAIT_CASES_START,
#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
	WAIT_CURR,
#endif
	C_WAIT_CASES_END,

	C_STATS_END = C_WAIT_CASES_END
};

struct cave_stats {
	unsigned long long cycles[C_STATS_END];
	unsigned long long counter[C_STATS_END];
	unsigned long long duration[C_STATS_END];
};

static char *cave_stat_name[C_STATS_END] = {
	__stringify(CAVE_INC),
	__stringify(CAVE_DEC),
	__stringify(SKIP_FAST),
#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
	__stringify(SKIP_SLOW),
	__stringify(SKIP_REPLAY),
	__stringify(SKIP_RACE),

	__stringify(TRYLOCK_INC),
	__stringify(TRYLOCK_DEC),

#endif
	__stringify(WAIT_TARGET),
#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
	__stringify(WAIT_CURR)
#endif
};

DEFINE_PER_CPU(struct cave_stats, time_stats);

static inline unsigned long long _start_measure(const enum reason reason)
{
	unsigned long long ret;

	ret = rdtsc();

	return ret;
}

static inline void _end_measure(unsigned long long start, enum cave_stat_idx c,
				const enum reason reason)
{
	struct cave_stats *t = this_cpu_ptr(&time_stats);

	unsigned long long cycles = rdtsc() - start;

	/* WARN_ON_ONCE(t->cycles[c] >= ULLONG_MAX - cycles); */

	t->cycles[c] += cycles;
	t->counter[c]++;

#ifdef CONFIG_CAVE_RAW_VOLTAGE_LOGGING
	if (reason == EXIT_SYSCALL)
#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
		&& test_bit(current->cave.syscall_nr, syscall_enabled)
#endif
			trace_printk("%d %llu", smp_processor_id(), arch_read_voltage());
#endif
}

#define start_measure(r)		_start_measure(r)
#define end_measure(start, c, r)	_end_measure(start, c, r)
#else
#define start_measure(r)		0
#define end_measure(start, c, r)
#endif

static struct kobject *cave_kobj;

#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
static DEFINE_SPINLOCK(cave_lock);

#define _cave_lock_1(flags)	spin_lock_irqsave(&cave_lock, flags)
#ifndef CONFIG_CAVE_STATS
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
			_end_measure(__start, lock_case, TRYLOCK);	\
	} while (0)

#define _cave_lock_3(flags, lock_case, __start)				\
	do {								\
		bool done = false;					\
		while (!spin_trylock_irqsave(&cave_lock, flags)) 	\
			if (!done) 					\
				done = true;				\
		if (done)						\
			_end_measure(__start, lock_case, TRYLOCK);	\
	} while (0)
#endif

#define GET_MACRO(_1, _2, _3, NAME, ...) NAME

#define cave_lock(...)	GET_MACRO(__VA_ARGS__, _cave_lock_3, _cave_lock_2, _cave_lock_1, fn0)(__VA_ARGS__)
#define cave_unlock(flags)	spin_unlock_irqrestore(&cave_lock, flags)
#else
#define cave_lock(...)
#define cave_unlock(flags)	(void)(flags)
#endif

static volatile struct cave_context cave_max_context __read_mostly = CAVE_CONTEXT(400);
static volatile struct cave_context cave_kernel_context __read_mostly = CAVE_CONTEXT(CAVE_NOMINAL_VOFFSET);
static volatile struct cave_context cave_user_context __read_mostly = CAVE_CONTEXT(CAVE_NOMINAL_VOFFSET);

DEFINE_PER_CPU(struct cave_context, context) = CAVE_CONTEXT(CAVE_NOMINAL_VOFFSET);

#ifdef CONFIG_CAVE_STATS
static DEFINE_SPINLOCK(cave_stat_avg_lock);

#define FSHIFT	11
#define FIXED_1	(1 << FSHIFT)
#define STAT_INT(x)	((x) >> FSHIFT)
#define STAT_FRAC(x)	STAT_INT(((x) & (FIXED_1 - 1)) * 100)
#define _STAT_DIV(x, t) 100 * ((x) << FSHIFT) / (t)
#define STAT_DIV(x, t)	STAT_INT(_STAT_DIV(x, t)), STAT_FRAC(_STAT_DIV(x, t))
#define STAT_FMT	"%llu.%02llu"

#define CAVE_STATS_TIMER_PERIOD	1
#define CAVE_STATS_MINUTE	(60 / CAVE_STATS_TIMER_PERIOD)

struct cave_stats_data {
	int num;
	struct cave_stats buffer[1 * CAVE_STATS_MINUTE];
	struct cave_stats sum;
	int idx;
	int size;
};

static char *avg_names[3] = { "total", "1sec", "1min" };
static struct cave_stats cave_stat_avg[3];
static struct cave_stats_data avg_data[1];

static inline void add_stat(struct cave_stats *result, struct cave_stats *val)
{
	int i;

	for (i = C_STATS_START; i < C_STATS_END; i++) {
		result->cycles[i] += val->cycles[i];
		result->counter[i] += val->counter[i];
		result->duration[i] += val->duration[i];
	}
}

static inline void sub_stat(struct cave_stats *result, struct cave_stats *val)
{
	int i;

	for (i = C_STATS_START; i < C_STATS_END; i++) {
		result->cycles[i] -= val->cycles[i];
		result->counter[i] -= val->counter[i];
		result->duration[i] -= val->duration[i];
	}
}

static inline void div_stat(struct cave_stats *result, struct cave_stats *val, const int div)
{
	int i;

	for (i = C_STATS_START; i < C_STATS_END; i++) {
		result->cycles[i] = val->cycles[i] / div;
		result->counter[i] = val->counter[i] / div;
		result->duration[i] = val->duration[i] / div;
	}
}

static void calc_moving_average(struct cave_stats *avg, struct cave_stats *val,
				struct cave_stats_data *d)
{
	add_stat(&d->sum, val);
	if (d->num < d->size) {
		d->buffer[d->idx] = *val;
		d->idx = (d->idx + 1) % d->size;
		d->num++;
	}
	else {
		sub_stat(&d->sum, &d->buffer[d->idx]);
		d->buffer[d->idx] = *val;
		d->idx = (d->idx + 1) % d->size;
	}
	div_stat(avg, &d->sum, d->num);
}

static struct hrtimer stats_hrtimer;
static ktime_t stats_period_time;

static struct cave_stats __stats_gather(void)
{
	unsigned long flags;
	struct cave_stats t;
	int i;
	int j;
	int cpu_cnt[C_STATS_END] = { 0 };

	memset(&t, 0, sizeof(t));

	for_each_online_cpu(i) {
		struct cave_stats c;

		/* local copy */
		cave_lock(flags);
		c = *per_cpu_ptr(&time_stats, i);
		memset(per_cpu_ptr(&time_stats, i), 0, sizeof(struct cave_stats));
		cave_unlock(flags);

		for (j = C_STATS_START; j < C_STATS_END; j++) {
			/* WARN_ON_ONCE((c.cycles[j] == 0) ^ (c.counter[j] == 0)); */
			if (c.counter[j]) {
				t.cycles[j] += c.cycles[j];
				t.counter[j] += c.counter[j];
				t.duration[j] += c.cycles[j] / c.counter[j];
				cpu_cnt[j]++;
			}
		}
	}

	for (j = C_STATS_START; j < C_STATS_END; j++) {
		if (cpu_cnt[j]) {
			t.cycles[j] /= cpu_cnt[j];
			t.counter[j] /= cpu_cnt[j];
			t.duration[j] /= cpu_cnt[j];
		}
	}

	return t;
}

static void _stats_gather(void)
{
	unsigned long flags;
	struct cave_stats t;

	t = __stats_gather();

	spin_lock_irqsave(&cave_stat_avg_lock, flags);
	add_stat(&cave_stat_avg[0], &t);
	cave_stat_avg[1] = t;
	calc_moving_average(&cave_stat_avg[2], &t, &avg_data[0]);
	spin_unlock_irqrestore(&cave_stat_avg_lock, flags);
}

static enum hrtimer_restart stats_gather(struct hrtimer *timer)
{
	_stats_gather();
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

	memset(avg_data, 0, sizeof(avg_data));
	avg_data[0].size = 1 * CAVE_STATS_MINUTE;

	memset(cave_stat_avg, 0, sizeof(cave_stat_avg));

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

static inline void wait_target_voffset(long new_voffset)
{
#ifdef CONFIG_CAVE_STATS
	unsigned long long start;
	struct cave_stats *t = this_cpu_ptr(&time_stats);
#endif

	if (cave_nowait)
		return;

#ifdef CONFIG_CAVE_STATS
	start = rdtsc();
#endif

	while (new_voffset < read_voffset())
		cpu_relax();

#ifdef CONFIG_CAVE_STATS
	t->cycles[WAIT_TARGET] += rdtsc() - start;
	t->counter[WAIT_TARGET]++;
#endif
}

#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
static inline void wait_curr_voffset(long new_voffset)
{
#ifdef CONFIG_CAVE_STATS
	unsigned long long start;
	struct cave_stats *t = this_cpu_ptr(&time_stats);
#endif

	if (cave_nowait)
		return;

#ifdef CONFIG_CAVE_STATS
	if (new_voffset >= curr_voffset)
		return;

	start = rdtsc();
#endif

	while (new_voffset < curr_voffset)
		cpu_relax();

#ifdef CONFIG_CAVE_STATS
	t->cycles[WAIT_CURR] += rdtsc() - start;
	t->counter[WAIT_CURR]++;
#endif
}

static long select_voffset(void)
{
	long new_voffset = LONG_MAX;
	int i;

	for_each_online_cpu(i) {
		struct cave_context tmp = per_cpu(context, i);
		if(tmp.voffset < new_voffset)
			new_voffset = tmp.voffset;
	}

	return new_voffset;
}

static inline void _cave_switch(const volatile struct cave_context *next_ctx,
				const enum reason reason)
{
	unsigned long flags;
	long new_voffset;
	long target_voffset;
	long updated_voffset;
	long selected_voffset;
	static int switch_path_contention = 0;
	unsigned long long start;

	/* This fast path works after cave_apply_tasks() completes. Until then,
	 * it may take some time until the system transitions to cave mechanism.
	 */
	if (next_ctx->voffset == this_cpu_read(context).voffset)
		return;

	start = start_measure(reason);

	cave_lock(flags, TRYLOCK_INC, start);

	this_cpu_write(context, *next_ctx);
	target_voffset = target_voffset_cached;
	new_voffset = next_ctx->voffset;

	/* increase voltage immediately */
	if (new_voffset < target_voffset) {
		target_voffset_cached = new_voffset;
		write_voffset(new_voffset);

		cave_unlock(flags);

		wait_target_voffset(new_voffset);
		end_measure(start, CAVE_INC, reason);

		return;
	}

	if (new_voffset == target_voffset) {
		cave_unlock(flags);
		wait_curr_voffset(new_voffset);
		end_measure(start, SKIP_FAST, reason);
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

	cave_lock(flags, TRYLOCK_DEC);

	switch_path_contention--;

	/* Skip cascade decreases of voltage from many CPUs */
	if (switch_path_contention) {
		cave_unlock(flags);
		end_measure(start, SKIP_REPLAY, reason);
		return;
	}

	selected_voffset = select_voffset();
	updated_voffset = target_voffset_cached;

	if (selected_voffset == updated_voffset) {
		cave_unlock(flags);
		end_measure(start, SKIP_SLOW, reason);
		return;
	}

	if (selected_voffset < updated_voffset) {
		cave_unlock(flags);
		end_measure(start, SKIP_RACE, reason);
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
	target_voffset_cached = selected_voffset;
	curr_voffset = selected_voffset;
	write_voffset(selected_voffset);
	cave_unlock(flags);
	end_measure(start, CAVE_DEC, reason);
}
#else
static inline void _cave_switch(const volatile struct cave_context *next_ctx,
				const enum reason reason)
{
	long target_voffset = this_cpu_read(context).voffset;
	long new_voffset = next_ctx->voffset;
	unsigned long long start;

	start = start_measure(reason);

	/*
	 * This fast path works after cave_apply_tasks() completes. Until then,
	 * it may take some time until the system transitions to cave mechanism.
	 */
	if (new_voffset == target_voffset) {
		end_measure(start, SKIP_FAST, reason);
		return;
	}

	this_cpu_write(context, *next_ctx);
	write_voffset(new_voffset);

	if (new_voffset < target_voffset) {
		wait_target_voffset(new_voffset);
		end_measure(start, CAVE_INC, reason);
	}
	else {
		end_measure(start, CAVE_DEC, reason);
	}
}
#endif

__visible void cave_syscall_entry_switch(unsigned long syscall_nr)
{
	volatile struct cave_context *context = &current->cave.kernel_ctx;
#ifdef CONFIG_CAVE_SYSCALL_RATELIMIT
        struct syscall_ratelimit *p = this_cpu_ptr(&srl);
#endif

	if (!cave_enabled)
		return;

#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
	current->cave.syscall_nr = syscall_nr;
	if (cave_syscall_context_enabled && test_bit(syscall_nr, syscall_enabled)) {
		current->cave.orig_kernel_ctx = current->cave.kernel_ctx;
		current->cave.kernel_ctx = cave_syscall_context;
	}
#endif

#ifdef CONFIG_CAVE_SYSCALL_RATELIMIT
        /*
         * Any changes regarding rate limit take effect on the exit path and
         * affect the next entry point.
         */

	/* track cave tasks */
	if (current->cave.custom_kernel_ctx
	    || current->cave.custom_user_ctx)
		p->counter++;

        if (unlikely(!p->enabled))
		return;

        if (unlikely(p->enabled < 0))
		p->enabled = 0;
#endif

	_cave_switch(context, ENTRY_SYSCALL);
}

__visible void cave_entry_switch(void)
{
	volatile struct cave_context *context = &current->cave.kernel_ctx;

	if (!cave_enabled)
		return;

#ifdef CONFIG_CAVE_SYSCALL_RATELIMIT
	if (this_cpu_ptr(&srl)->enabled <= 0)
		return;
#endif

	_cave_switch(context, ENTRY);
}

/*
   No need to restore kernel_ctx on exit paths, next entry will handle it.
   kernel threads do not exit to user-space.
*/

__visible void cave_exit_switch(void)
{
	volatile struct cave_context *context = &current->cave.user_ctx;

	if (!cave_enabled)
		return;

#ifdef CONFIG_CAVE_SYSCALL_RATELIMIT
	if (this_cpu_ptr(&srl)->enabled <= 0)
		return;
#endif

	_cave_switch(context, EXIT);
}

__visible void cave_syscall_exit_switch(void)
{
	volatile struct cave_context *context = &current->cave.user_ctx;

	if (!cave_enabled)
		return;

#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
	/*
	 * If we have per system call voffsets which are different from the rest
	 * of the kernel, we need to restore the original kernel voffset.
	 *
	 * This happens for example, on system calls which may sleep,
	 * irq from kernel, syscall slowpath.
	 */
	if (cave_syscall_context_enabled && test_bit(current->cave.syscall_nr, syscall_enabled))
		current->cave.kernel_ctx = current->cave.orig_kernel_ctx;
#endif

#ifdef CONFIG_CAVE_SYSCALL_RATELIMIT
	if (this_cpu_ptr(&srl)->enabled <= 0)
		return;
#endif

	_cave_switch(context, EXIT_SYSCALL);
}

void cave_guest_entry(void)
{
	volatile struct cave_context *context = &current->cave.user_ctx;

	if (!cave_enabled)
		return;

	_cave_switch(context, ENTRY);
}
EXPORT_SYMBOL_GPL(cave_guest_entry);

void cave_guest_exit(void)
{
	volatile struct cave_context *context = &current->cave.kernel_ctx;

	if (!cave_enabled)
		return;

	_cave_switch(context, EXIT);
}
EXPORT_SYMBOL_GPL(cave_guest_exit);

/*
 * The current task may be either a kernel or a user task.
 *
 *	               exec()
 *	   user task    -->    user task
 *	   kernel task  -->    user task
 *
 * Kernel threads do not care about user context.
 * As exec() converts a kernel task to a user task, we need
 * to set user context accordingly.
 *
 * see fs/exec.c for more details about do_execveat_common()
 */
void cave_exec_task(struct task_struct *p)
{
	unsigned long flags;

	if (p->flags & PF_KTHREAD) {
		spin_lock_irqsave(&p->cave.lock, flags);
		p->cave.user_ctx = cave_user_context;
		p->cave.custom_kernel_ctx= false;
		p->cave.custom_user_ctx = false;
		spin_unlock_irqrestore(&p->cave.lock, flags);
	}
}

/* The task is not visible to the rest of the system yet.
 * cave_apply_tasks() may set again the appropriate contexts.
 */
void cave_fork_init(struct task_struct *p)
{
	spin_lock_init(&p->cave.lock);

	if (!p->cave.custom_kernel_ctx)
		p->cave.kernel_ctx = cave_kernel_context;
	if (!p->cave.custom_user_ctx)
		p->cave.user_ctx = cave_user_context;
}

/*
 * Context switch takes place in kernel mode
 * switch voltage again on the user exit path.
 */
void cave_context_switch_voltage(struct task_struct *prev, struct task_struct *next)
{
	volatile struct cave_context *next_ctx = &next->cave.kernel_ctx;

	if (!cave_enabled)
		return;

	_cave_switch(next_ctx, CONTEXT_SWITCH);
}

/* protected by cave_lock to avoid races with new voffset values */
static void cave_apply_tasks(void)
{
	struct task_struct *g, *p;
	unsigned long flags;

	int i;

	read_lock(&tasklist_lock);
	for_each_process_thread(g, p) {
		spin_lock_irqsave(&p->cave.lock, flags);
		if (!p->cave.custom_kernel_ctx)
			p->cave.kernel_ctx = cave_kernel_context;
		if (!p->cave.custom_user_ctx)
			p->cave.user_ctx = cave_user_context;
		spin_unlock_irqrestore(&p->cave.lock, flags);
	}

	/* Idle tasks do not run on user mode, i.e. they don't enter kernel
	 * through entry points, but from schedule() context_switch().
	 * Therefore it is safe to set kernel_ctx for them here.
	 */
	for_each_possible_cpu(i) {
		p = idle_task(i);
		spin_lock_irqsave(&p->cave.lock, flags);
		p->cave.kernel_ctx = cave_user_context;
		p->cave.user_ctx = cave_user_context;
		spin_unlock_irqrestore(&p->cave.lock, flags);
	}
	read_unlock(&tasklist_lock);
}

#ifdef CONFIG_CAVE_STATS
static int _print_cave_stats(char *buf, struct cave_stats *t, char *name)
{
	int ret = 0;

	int j;
	unsigned long long cycles = 0;
	unsigned long long counter = 0;

	for (j = C_SWITCH_CASES_START; j < C_SWITCH_CASES_END; j++) {
		if (t->counter[j]) {
			cycles += t->cycles[j];
			counter += t->counter[j];
		}
	}

	if (cycles == 0 || counter == 0)
	        return ret;

	ret += sprintf(buf + ret, "%s_stats %llu %llu\n", name, cycles, counter);

	for (j = C_STATS_START; j < C_STATS_END; j++) {
		ret += sprintf(buf + ret, "%s %llu %llu %llu\n",
			       cave_stat_name[j],
			       t->cycles[j],
			       t->counter[j],
			       t->duration[j]
			       );
	}

#define T_WAIT_TARGET(__m)	(t->__m[WAIT_TARGET])

#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
#define T_WAIT_CURR(__m)	(t->__m[WAIT_CURR])
#define T_TRYLOCK(__m)		(t->__m[TRYLOCK_INC] + t->__m[TRYLOCK_DEC])
#else
#define T_WAIT_CURR(__m)	(0)
#define T_TRYLOCK(__m)		(0)
#endif

#define T_WAIT(__m)	(T_WAIT_TARGET(__m) + T_WAIT_CURR(__m))

	ret += sprintf(buf + ret, "%s " STAT_FMT "\n",
		       "wait",
		       STAT_DIV(T_WAIT(cycles), cycles)
		       );

	ret += sprintf(buf + ret, "%s " STAT_FMT "\n",
		       "trylock",
		       STAT_DIV(T_TRYLOCK(cycles), cycles)
		       );

	ret += sprintf(buf + ret, "%s " STAT_FMT "\n",
		       "decide",
		       STAT_DIV(cycles - T_WAIT(cycles) + T_TRYLOCK(cycles), cycles)
		       );

#undef T_WAIT
#undef T_TRYLOCK

	return ret;
}

static int print_cave_stats(char *buf)
{
	unsigned long flags;
	int ret = 0;
	int i;
	static struct cave_stats tmp_stat[3];

	spin_lock_irqsave(&cave_stat_avg_lock, flags);
	memcpy(tmp_stat, cave_stat_avg, sizeof(tmp_stat));
	spin_unlock_irqrestore(&cave_stat_avg_lock, flags);

	if (cave_enabled) {
		ret += sprintf(buf + ret, "average cycles counter duration\n");
		for (i = 0; i < 3; i++) {
			ret += _print_cave_stats(buf + ret, &tmp_stat[i], avg_names[i]);
			ret += sprintf(buf + ret, "\n");
		}

	}

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

static void cave_cpu_enable(void *data)
{
	struct cave_context *context = (struct cave_context *)data;

	if (!cave_enabled)
		return;

	_cave_switch(context, CONTEXT_SWITCH);
}

static void cave_cpu_disable(void *data)
{
	struct cave_context *context = (struct cave_context *)data;

	if (!cave_enabled)
		return;

	_cave_switch(context, CONTEXT_SWITCH);
}

static
ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
		     const char *buf, size_t count)
{
	unsigned long flags;
	int enable;
	int err;

	err = kstrtouint(buf, 10, &enable);
	if (err || (enable != 0 && enable != 1)) {
		pr_warn("cave: invalid %s value\n", attr->attr.name);
		return count;
	}

	if (enable && !cave_enabled) {
		struct cave_context context;
		cave_lock(flags);
		/* local copy to avoid races after unlock */
		context = cave_kernel_context;
		cave_apply_tasks();
		stats_init();
		syscall_ratelimit_init();
		cave_enabled = 1;
		cave_unlock(flags);
		on_each_cpu(cave_cpu_enable, &context, 1);
		pr_info("cave: enabled\n");
	}
	else if (!enable && cave_enabled) {
		struct cave_context nominal = CAVE_CONTEXT(CAVE_NOMINAL_VOFFSET);
		on_each_cpu(cave_cpu_disable, &nominal, 1);
		cave_lock(flags);
		cave_enabled = 0;
#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
		cave_syscall_context_enabled = 0;
#endif
		syscall_ratelimit_clear();
		stats_clear();
		cave_unlock(flags);
		pr_info("cave: disabled\n");
	}

	return count;
}

static
ssize_t max_voffset_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%ld\n", cave_max_context.voffset);

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
	if (voffset < cave_kernel_context.voffset)
		pr_warn("cave: new value of max_voffset less than kernel voffset\n");
#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
	else if (voffset < cave_syscall_context.voffset)
		pr_warn("cave: new value of max_voffset less than syscall voffset\n");
#endif
	else if (voffset < cave_user_context.voffset)
		pr_warn("cave: new value of max_voffset less than userspace voffset\n");
	else
		cave_max_context.voffset = voffset;
	cave_unlock(flags);
	return count;
}

#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
static
ssize_t enable_syscall_voffset_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%d\n", cave_syscall_context_enabled);

	return ret;
}

static
ssize_t enable_syscall_voffset_store(struct kobject *kobj, struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long flags;
	int enable;
	int err;

	err = kstrtouint(buf, 10, &enable);
	if (err || (enable != 0 && enable != 1)) {
		pr_warn("cave: invalid %s value\n", attr->attr.name);
		return count;
	}

	cave_lock(flags);
	if (cave_enabled)
		pr_warn("cave: must be disabled to enable / disable syscall voffset\n");
	else if (cave_syscall_context_enabled != enable) {
		cave_syscall_context_enabled = enable;
		pr_info("cave: %s syscall voffset\n", enable ? "enable" : "disable");
	}
	cave_unlock(flags);
	return count;
}
#endif

static
ssize_t kernel_voffset_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%ld\n", cave_kernel_context.voffset);

	return ret;
}

static
ssize_t kernel_voffset_store(struct kobject *kobj, struct kobj_attribute *attr,
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

	if (voffset > cave_max_context.voffset) {
		pr_warn("cave: %s out of range\n", attr->attr.name);
		return count;
	}

	cave_lock(flags);

	if (cave_enabled)
		pr_warn("cave: must be disabled to change kernel voffset\n");
	else
		cave_kernel_context.voffset = voffset;

	cave_unlock(flags);

	return count;
}

#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
static
ssize_t syscall_voffset_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%ld\n", cave_syscall_context.voffset);

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

	if (voffset > cave_max_context.voffset) {
		pr_warn("cave: %s out of range\n", attr->attr.name);
		return count;
	}

	cave_lock(flags);

	if (cave_enabled)
		pr_warn("cave: must be disabled to change syscall voffset\n");
	else
		cave_syscall_context.voffset = voffset;

	cave_unlock(flags);

	return count;
}
#endif

static
ssize_t userspace_voffset_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%ld\n", cave_user_context.voffset);

	return ret;
}

static
ssize_t userspace_voffset_store(struct kobject *kobj, struct kobj_attribute *attr,
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

	if (voffset > cave_max_context.voffset) {
		pr_warn("cave: %s out of range\n", attr->attr.name);
		return count;
	}

	cave_lock(flags);

	if (cave_enabled)
		pr_warn("cave: must be disabled to change userspace voffset\n");
	else
		cave_user_context.voffset = voffset;

	cave_unlock(flags);

	return count;
}

#ifdef CONFIG_CAVE_STATS
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
#endif

static
ssize_t voltage_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	long voffset;

#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
	unsigned long flags;

	cave_lock(flags);
	voffset = target_voffset_cached;
	cave_unlock(flags);
#else
	voffset = this_cpu_read(context).voffset;
#endif

	ret += sprintf(buf + ret, "voffset %ld\n", -voffset);
	ret += sprintf(buf + ret, "voltage %lld\n", arch_read_voltage());
	return ret;
}

#ifdef CONFIG_CAVE_SYSCALL_RATELIMIT
static
ssize_t syscall_rate_limit_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%u\n", syscall_rate_limit);

	return ret;
}

static
ssize_t syscall_rate_limit_store(struct kobject *kobj, struct kobj_attribute *attr,
                                 const char *buf, size_t count)
{
	unsigned int limit;
	int err;

	err = kstrtouint(buf, 10, &limit);
	if (err || limit == 0) {
		pr_warn("cave: invalid %s value\n", attr->attr.name);
		return count;
	}

	syscall_rate_limit = limit;

	return count;
}

static
ssize_t syscall_rate_period_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%u\n", syscall_rate_period);

	return ret;
}

static
ssize_t syscall_rate_period_store(struct kobject *kobj, struct kobj_attribute *attr,
                                  const char *buf, size_t count)
{
	unsigned int time;
	int err;

	err = kstrtouint(buf, 10, &time);
	if (err || time == 0) {
		pr_warn("cave: invalid %s value\n", attr->attr.name);
		return count;
	}

	syscall_rate_period = time;
	ratelimit_period_time = ms_to_ktime(syscall_rate_period);

	return count;
}
#endif

static
ssize_t debug_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

#ifdef CONFIG_CAVE_SKIP_ARCH_RW
	ret += sprintf(buf + ret, "option:skip_arch = %s\n", skip_arch ? "true" : "false");
#endif
#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
	ret += sprintf(buf + ret, "config:one_voltage_domain\n");
#endif
#ifdef CONFIG_CAVE_STATS
	ret += sprintf(buf + ret, "config:stats\n");
#endif
#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
	ret += sprintf(buf + ret, "config:syscall\n");
#endif

	return ret;
}

static
ssize_t debug_store(struct kobject *kobj, struct kobj_attribute *attr,
		    const char *buf, size_t count)
{
#ifdef CONFIG_CAVE_SKIP_ARCH_RW
	int val = 0;

	sscanf(buf, "skip_arch = %d", &val);
	if ((val == 1 || val == 0) && skip_arch != val) {
		skip_arch = val;
		pr_info("cave: skip_arch = %s\n", val ? "true" : "false");
	}
#endif

	return count;
}

static
ssize_t ctl_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
	static char *s = (void *)-1;

	if (s == (void *)-1)
		s = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!s)
		return 0;

	bitmap_print_to_pagebuf(false, s, syscall_enabled, __NR_syscall_max);
	ret += snprintf(buf + ret, PAGE_SIZE, "cave: syscall bitmap: %s\n", s);

	// kfree(s);
#endif

	ret += sprintf(buf + ret, "nowait=%s\n", cave_nowait ? "true" : "false");
#ifdef CONFIG_CAVE_SYSCALL_RATELIMIT
	ret += sprintf(buf + ret, "ratelimit=%s\n", cave_ratelimit ? "true" : "false");
#endif

	return ret;
}

static
ssize_t ctl_store(struct kobject *kobj, struct kobj_attribute *attr,
		  const char *buf, size_t count)
{
#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
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
                if (ret) {
			pr_warn("cave: ctl: invalid bitmap parselist\n");
			return count;
		}

		bitmap_or(syscall_enabled, syscall_enabled, tmp, __NR_syscall_max);
		return count;
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
                if (ret) {
			pr_warn("cave: ctl: invalid bitmap parselist\n");
			return count;
		}

		bitmap_andnot(syscall_enabled, syscall_enabled, tmp, __NR_syscall_max);
		return count;
	}
#endif

	if (strncmp(buf, "nowait:true", 11) == 0) {
		if (!cave_nowait) {
			cave_nowait = true;
			pr_info("cave: nowait = true\n");
		}
		return count;
	}
	else if (strncmp(buf, "nowait:false", 12) == 0) {
		if (cave_nowait) {
			cave_nowait = false;
			pr_info("cave: nowait = false\n");
		}
		return count;
	}

#ifdef CONFIG_CAVE_SYSCALL_RATELIMIT
	if (strncmp(buf, "ratelimit:true", 14) == 0) {
		if (!cave_ratelimit) {
			cave_ratelimit = true;
			if (cave_enabled)
				syscall_ratelimit_init();
		}
		return count;
	}
	else if (strncmp(buf, "ratelimit:false", 15) == 0) {
		if (cave_ratelimit) {
			cave_ratelimit = false;
			if (cave_enabled)
				syscall_ratelimit_clear();
		}
		return count;
	}
#endif

	return count;
}

KERNEL_ATTR_RW(enable);
#ifdef CONFIG_CAVE_STATS
KERNEL_ATTR_RO(stats);
KERNEL_ATTR_WO(reset_stats);
#endif
KERNEL_ATTR_RO(voltage);
KERNEL_ATTR_RW(max_voffset);
KERNEL_ATTR_RW(kernel_voffset);
#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
KERNEL_ATTR_RW(syscall_voffset);
KERNEL_ATTR_RW(enable_syscall_voffset);
#endif
KERNEL_ATTR_RW(userspace_voffset);
KERNEL_ATTR_RW(debug);
KERNEL_ATTR_RW(ctl);
#ifdef CONFIG_CAVE_SYSCALL_RATELIMIT
KERNEL_ATTR_RW(syscall_rate_limit);
KERNEL_ATTR_RW(syscall_rate_period);
#endif

#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
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

#include "../arch/x86/include/generated/asm/syscalls_64.h"

#undef HELPERS
#undef __SYSCALL_64

static struct attribute_group syscall_enabled_attr_group = {
	.attrs = (struct attribute * []) {
#define __SYSCALL_64(__nr, __sys, __qual)	&__sys##_attr.attr,
#include "../arch/x86/include/generated/asm/syscalls_64.h"
#undef __SYSCALL_64
		NULL
	}
};
#endif

static struct attribute_group attr_group = {
	.attrs = (struct attribute * []) {
		&enable_attr.attr,
#ifdef CONFIG_CAVE_STATS
		&reset_stats_attr.attr,
		&stats_attr.attr,
#endif
		&voltage_attr.attr,
		&max_voffset_attr.attr,
		&kernel_voffset_attr.attr,
#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
		&syscall_voffset_attr.attr,
		&enable_syscall_voffset_attr.attr,
#endif
		&userspace_voffset_attr.attr,
		&debug_attr.attr,
		&ctl_attr.attr,
#ifdef CONFIG_CAVE_SYSCALL_RATELIMIT
                &syscall_rate_limit_attr.attr,
                &syscall_rate_period_attr.attr,
#endif
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

#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
	syscall_enabled_kobj = kobject_create_and_add("syscall_enabled", cave_kobj);
	if (!syscall_enabled_kobj) {
		pr_err("cave: failed\n");
		return -ENOMEM;
	}
#endif

	err = sysfs_create_group(cave_kobj, &attr_group);
	if (err) {
		pr_err("cave: failed\n");
		return err;
	}

#ifdef CONFIG_CAVE_SYSCALL_CONTEXT
	err = sysfs_create_group(syscall_enabled_kobj, &syscall_enabled_attr_group);
	if (err) {
		pr_err("cave: failed\n");
		return err;
	}

	bitmap_fill(syscall_enabled, __NR_syscall_max);
#endif

	cave_lock(flags);

#ifdef CONFIG_CAVE_COMMON_VOLTAGE_DOMAIN
	target_voffset_cached = CAVE_NOMINAL_VOFFSET;
	curr_voffset = CAVE_NOMINAL_VOFFSET;
#endif
	arch_write_voffset(CAVE_NOMINAL_VOFFSET);
	voffset = arch_read_voffset();
	cave_unlock(flags);

	pr_info("cave: msr offset: %ld\n", -voffset);

	return 0;
}
late_initcall(cave_init);

#define CAVE_KEEP_VOFFSET	(0xFFFF)
#define CAVE_SET_VOFFSET	128

SYSCALL_DEFINE4(cave_ctl, int, action, int, pid, int, kernel_voffset, int, user_voffset)
{
	struct task_struct *p;
	unsigned long flags;

	switch (action) {
	case CAVE_SET_VOFFSET:
		if (pid == 0)
			p = current;
		else if (pid > 0 && (p = find_task_by_vpid(pid)))
			;  /* ok */
		else
			return -EINVAL;

		if (kernel_voffset != CAVE_KEEP_VOFFSET
		    && (kernel_voffset < 0 || kernel_voffset > cave_max_context.voffset))
			return -EINVAL;
		if (user_voffset != CAVE_KEEP_VOFFSET
		    && (user_voffset < 0 || user_voffset > cave_max_context.voffset))
			return -EINVAL;

		spin_lock_irqsave(&p->cave.lock, flags);
		if (kernel_voffset != CAVE_KEEP_VOFFSET) {
			p->cave.kernel_ctx = CAVE_CONTEXT(kernel_voffset);
			p->cave.custom_kernel_ctx = true;
		}
		if (user_voffset != CAVE_KEEP_VOFFSET) {
			p->cave.user_ctx = CAVE_CONTEXT(user_voffset);
			p->cave.custom_user_ctx = true;
		}
		spin_unlock_irqrestore(&p->cave.lock, flags);

		if (p->cave.custom_kernel_ctx && p->cave.custom_user_ctx)
			pr_info("cave: %s [pid=%d] set voffset: kernel=%ld, user=%ld\n",
				p->comm, task_pid_vnr(p),
				p->cave.kernel_ctx.voffset, p->cave.user_ctx.voffset);
		else if (p->cave.custom_kernel_ctx)
			pr_info("cave: %s [pid=%d] set voffset: kernel=%ld\n",
				p->comm, task_pid_vnr(p), p->cave.kernel_ctx.voffset);
		else if (p->cave.custom_user_ctx)
			pr_info("cave: %s [pid=%d] set voffset: user=%ld\n",
				p->comm, task_pid_vnr(p), p->cave.user_ctx.voffset);

		return 0;
	}

	return -EINVAL;
}
