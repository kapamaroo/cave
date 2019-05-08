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
#include <linux/syscalls.h>

#define CAVE_KERNEL_CONTEXT (cave_data_t){ .voltage = VOLTAGE_OF(cave_kernel_voffset) }
#define CAVE_NOMINAL_CONTEXT	(cave_data_t){ .voltage = CAVE_NOMINAL_VOLTAGE }
#ifdef CONFIG_UNISERVER_CAVE_USERSPACE
#define CAVE_USERSPACE_CONTEXT	(cave_data_t){ .voltage = VOLTAGE_OF(cave_userspace_voffset) }
#endif

#define TO_VOFFSET_DATA(__val)	(__val ? (0x800ULL - (u64)__val) << 21 : 0ULL)
#define TO_VOFFSET_VAL(__data)    (__data ? (0x800ULL - ((u64)__data >> 21)) : 0ULL)

#define CORE_VOFFSET_VAL(__val)		(0x8000001100000000ULL | TO_VOFFSET_DATA(__val))
#define CACHE_VOFFSET_VAL(__val)	(0x8000021100000000ULL | TO_VOFFSET_DATA(__val))

enum cave_case {
	CAVE_INC = 0,
	SKIP_INC_WAIT,
	SKIP_FAST,
	SKIP_SLOW,
	SKIP_REPLAY,
	SKIP_RACE,
	CAVE_DEC,
	CAVE_CASES
};

static char *cave_stat_name[CAVE_CASES] = {
	"inc",
	"skip_inc_wait",
	"skip_fast",
	"skip_slow",
	"skip_replay",
	"skip_race",
	"dec"
};

struct cave_stat {
	long inc;
	long dec;
	long skip_inc_wait;
	long skip_fast;
	long skip_slow;
	long skip_replay;
	long skip_race;
	long skip;
	long locked;
	long locked_inc;
	long locked_dec;
	long total;
};

#define CAVE_TIME_STATS

#ifdef CAVE_TIME_STATS
struct cave_time {
	unsigned long long time[CAVE_CASES];
	unsigned long long counter[CAVE_CASES];
	unsigned long long wait_target_time;
	unsigned long long wait_target_counter;
	unsigned long long wait_curr_time;
	unsigned long long wait_curr_counter;
};

DEFINE_PER_CPU(struct cave_time, time_stats);
#endif

static inline void end_measure(unsigned long long start, enum cave_case c)
{
#ifdef CAVE_TIME_STATS
	struct cave_time *t = this_cpu_ptr(&time_stats);

	t->time[c] += rdtsc() - start;
	t->counter[c]++;
#endif
}

static volatile int cave_enabled = 0;
static DEFINE_SPINLOCK(cave_lock);
DEFINE_PER_CPU(cave_data_t, context) = CAVE_NOMINAL_CONTEXT;
static volatile int cave_random_vmin_enabled __read_mostly = 0;
static volatile int cave_kernel_voffset __read_mostly = CAVE_DEFAULT_KERNEL_VOFFSET;
#ifdef CONFIG_UNISERVER_CAVE_USERSPACE
#define CAVE_DEFAULT_USERSPACE_VOFFSET	CONFIG_UNISERVER_CAVE_DEFAULT_USERSPACE_VOFFSET
static volatile int cave_userspace_voffset __read_mostly = CAVE_DEFAULT_USERSPACE_VOFFSET;
#endif
static volatile int cave_max_voffset __read_mostly = 400;
static volatile long target_voltage_cached = CAVE_NOMINAL_VOLTAGE;
static volatile long curr_voltage = CAVE_NOMINAL_VOLTAGE;

#ifdef CONFIG_UNISERVER_CAVE_STATS
static DEFINE_SPINLOCK(cave_stat_avg_lock);
static struct cave_stat cave_stat;
static struct cave_stat cave_stat_avg[3];
static int stat_samples[3] = { 0, 0, 0 };

#define FSHIFT	11
#define FIXED_1	(1 << FSHIFT)
#define STAT_INT(x)	((x) >> FSHIFT)
#define STAT_FRAC(x)	STAT_INT(((x) & (FIXED_1 - 1)) * 100)

#define CAVE_STATS_TIMER_PERIOD	1
#define CAVE_STATS_MINUTE	(60 / CAVE_STATS_TIMER_PERIOD)

#define INC(x)	do { x++; } while (0)

static struct hrtimer stats_hrtimer;
static ktime_t stats_period_time;

static enum hrtimer_restart stats_gather(struct hrtimer *timer)
{
	unsigned long flags;
	struct cave_stat new_stat;

	spin_lock_irqsave(&cave_lock, flags);
	WARN_ON(!cave_enabled);
	new_stat = cave_stat;
	memset(&cave_stat, 0, sizeof(struct cave_stat));
	spin_unlock_irqrestore(&cave_lock, flags);

#define __RUNNING_AVG_STAT(d, s, n, x)		\
	d.x = (d.x * (n) + s.x) / ((n) + 1)
#define RUNNING_AVG_STAT(d, s, n, l)			\
	if (n == l)					\
		n--;					\
							\
	__RUNNING_AVG_STAT(d, s, n, locked_inc);	\
	__RUNNING_AVG_STAT(d, s, n, locked_dec);	\
	__RUNNING_AVG_STAT(d, s, n, inc);		\
	__RUNNING_AVG_STAT(d, s, n, dec);		\
	__RUNNING_AVG_STAT(d, s, n, skip_inc_wait);	\
	__RUNNING_AVG_STAT(d, s, n, skip_fast);		\
	__RUNNING_AVG_STAT(d, s, n, skip_slow);		\
	__RUNNING_AVG_STAT(d, s, n, skip_replay);	\
	__RUNNING_AVG_STAT(d, s, n, skip_race);		\
	n++;

	spin_lock_irqsave(&cave_stat_avg_lock, flags);
	RUNNING_AVG_STAT(cave_stat_avg[0], new_stat, stat_samples[0], 1 * CAVE_STATS_MINUTE);
	RUNNING_AVG_STAT(cave_stat_avg[1], new_stat, stat_samples[1], 5 * CAVE_STATS_MINUTE);
	RUNNING_AVG_STAT(cave_stat_avg[2], new_stat, stat_samples[2], 10 * CAVE_STATS_MINUTE);
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
		struct cave_time *t = per_cpu_ptr(&time_stats, i);
		memset(t, 0, sizeof(struct cave_time));
	}

	memset(&cave_stat, 0, sizeof(struct cave_stat));
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

#define INC(x)

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
			if (c->voltage != VOLTAGE_OF(cave_kernel_voffset))
				pr_warn("cave: kthread %s with %d voltage\n",
					p->comm, c->voltage);
		}
		else {
			if (c->voltage == VOLTAGE_OF(cave_kernel_voffset))
				pr_warn("cave: user thread %s with %d voltage\n",
					p->comm, c->voltage);
		}
	}
	read_unlock(&tasklist_lock);
}
#endif

static inline void write_voffset_msr(u64 voffset)
{
	wrmsrl(0x150, CORE_VOFFSET_VAL(voffset));
	wrmsrl(0x150, CACHE_VOFFSET_VAL(voffset));
}

static inline u64 read_voffset_msr(void)
{
	u64 voffset;

	wrmsrl(0x150, 0x8000001000000000);
	rdmsrl(0x150, voffset);

	return TO_VOFFSET_VAL(voffset);
}

static void write_target_voltage(long new_voltage)
{
	target_voltage_cached = new_voltage;
}

static void write_voltage_msr(long new_voltage)
{
	u64 new_voffset;

	new_voffset = VOFFSET_OF(new_voltage);
	write_voffset_msr(new_voffset);

	if (unlikely(new_voltage != target_voltage_cached)) {
		WARN_ON_ONCE(1);
		target_voltage_cached = new_voltage;
	}
}

static long read_target_voltage(void)
{
	return target_voltage_cached;
}

static long read_voltage_msr(void)
{
	long voffset_msr, voltage_msr;

	voffset_msr = read_voffset_msr();
	voltage_msr = VOLTAGE_OF(voffset_msr);

	return voltage_msr;
}

static void wait_curr_voltage(long new_voltage)
{
#ifdef CAVE_TIME_STATS
	unsigned long long start;
	struct cave_time *t = this_cpu_ptr(&time_stats);

	start = rdtsc();
#endif

	while (new_voltage > curr_voltage)
		cpu_relax();

#ifdef CAVE_TIME_STATS
	t->wait_curr_time += rdtsc() - start;
	t->wait_curr_counter++;
#endif
}

static void wait_target_voltage(long new_voltage)
{
#ifdef CAVE_TIME_STATS
	unsigned long long start;
	struct cave_time *t = this_cpu_ptr(&time_stats);

	start = rdtsc();
#endif

	while (new_voltage > (curr_voltage = read_voltage_msr()))
		cpu_relax();

#ifdef CAVE_TIME_STATS
	t->wait_target_time += rdtsc() - start;
	t->wait_target_counter++;
#endif
}

static long select_voltage(void)
{
	long new_voltage = 0;
	int i;

	for_each_possible_cpu(i) {
		cave_data_t tmp = per_cpu(context, i);
		if(tmp.voltage > new_voltage)
			new_voltage = tmp.voltage;
	}

	return new_voltage;
}

static inline void _cave_switch(cave_data_t new_context)
{
	unsigned long flags;
	long new_voltage = new_context.voltage;
	long target_voltage;
	long updated_voltage;
	long selected_voltage;
	unsigned long long my_ref;
	static unsigned long long ref = 0;

#if 1
	bool done_inc = false;
	bool done_dec = false;
#endif

#ifdef CAVE_TIME_STATS
	unsigned long long start;

	start = rdtsc();
#else
#define start	0
#endif

	if (!cave_enabled)
		return;

#if 0
	spin_lock_irqsave(&cave_lock, flags);
#else
	while (!spin_trylock_irqsave(&cave_lock, flags)) {
		if (!done_inc) {
			done_inc = true;
			INC(cave_stat.locked_inc);
		}
	}
#endif

	this_cpu_write(context, new_context);
	target_voltage = read_target_voltage();

	/* increase voltage immediately */
	if (new_voltage > target_voltage) {
		write_target_voltage(new_voltage);
		write_voltage_msr(new_voltage);

		spin_unlock_irqrestore(&cave_lock, flags);

		wait_target_voltage(new_voltage);

		INC(cave_stat.inc);
		end_measure(start, CAVE_INC);
		return;
	}
	my_ref = ref++;
	spin_unlock_irqrestore(&cave_lock, flags);

	/* When more than one increases happen in a row, the smaller increases
	 * wait for the curr_voltage to match the new_voltage.
	 *
	 * This is a light-weight increase which merely waits for the voltage to
	 * increase.
	 */

	if (new_voltage >= curr_voltage) {
		wait_curr_voltage(new_voltage);
		INC(cave_stat.skip_inc_wait);
		end_measure(start, SKIP_INC_WAIT);
		return;
	}

	/* if another CPU managed to change voltage before the following check,
	 * we may keep an out-of-date target_voltage value. In any case the other
	 * CPU has observed / considered our new_voltage during its decision.
	 * The out-of-date target_voltage is covered in all of the following
	 * possible cases:
	 *
	 * increased target_voltage:
	 * 	The other CPU holds the most constrained voltage value, we can
	 * 	simply skip any voltage change.
	 *
	 * decreased target_voltage:
	 * 	The other CPU has released the last most constrained voltage
	 * 	value and went through the lockless selection. The decreased
	 * 	voltage is >= to our new_voltage, we can simply skip any voltage
	 * 	changes.
	 */
	if (new_voltage == target_voltage) {
		INC(cave_stat.skip_fast);
		end_measure(start, SKIP_FAST);
		return;
	}

	/*
	 * new_voltage < target_voltage
	 *
	 * We may observe newer contexts from other CPUs that may lead to
	 * increase of voltage instead of decrease. Other CPUs that may try to
	 * change voltage are going to decide upon the newer voltage values.
	 */
	selected_voltage = select_voltage();

	/* The constrained voltage resides on another CPU and remains the same.
	 * In the case we observe a selection greater than the current voltage,
	 * another CPU has set the voltage constraint higher.
	 *
	 * skip voltage decrease, the other CPU increases the voltage.
	 */

	if (selected_voltage >= target_voltage) {
		INC(cave_stat.skip_slow);
		end_measure(start, SKIP_SLOW);
		return;
	}

	/* selected_voltage < target_voltage */

#if 0
	spin_lock_irqsave(&cave_lock, flags);
#else
	while (!spin_trylock_irqsave(&cave_lock, flags)) {
		if (!done_dec) {
			done_dec = true;
			INC(cave_stat.locked_dec);
		}
	}
#endif

	/* We want to decrease voltage (this CPU was the most constrained).
	 *
	 * updated_voltage > target_voltage
	 * 	another CPU has the highest voltage constraint and increased it.
	 * 	skip decrease
	 *
	 * updated_voltage < target_voltage
	 * 	another CPU has decreased the voltage considering our constraint
	 * 	on the decision.
	 * 	skip decrease
	 *
	 * updated_voltage == target_voltage
	 * 	1. None of the other CPUs crossed an entry / exit point.
	 * 	   selected_voltage is valid
	 * 	2. Some other CPUs have skipped decreasing the voltage or
	 * 	   increased it. This means that another CPU is the most
	 * 	   constrained now with >= voltage value.
	 * 	   selected_voltage is out-of-date
	 */
	updated_voltage = read_target_voltage();

	if (updated_voltage != target_voltage) {
		spin_unlock_irqrestore(&cave_lock, flags);
		INC(cave_stat.skip_race);
		end_measure(start, SKIP_RACE);
		return;
	}

	if (my_ref != ref) {
		/* case 2 */
		selected_voltage = select_voltage();
		if (selected_voltage >= updated_voltage) {
			spin_unlock_irqrestore(&cave_lock, flags);
			INC(cave_stat.skip_replay);
			end_measure(start, SKIP_REPLAY);
			return;
		}
	}

	write_target_voltage(selected_voltage);
	write_voltage_msr(selected_voltage);
	spin_unlock_irqrestore(&cave_lock, flags);
	INC(cave_stat.dec);
	end_measure(start, CAVE_DEC);

#ifdef CAVE_TIME_STATS
#undef start
#endif
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
	p->cave_data.voltage = VOLTAGE_OF(voffset);
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
void cave_set_task(struct task_struct *p, struct task_struct *parent)
{
	unsigned long voffset = 0;

	p->cave_data = parent->cave_data;

	if (p->cave_data.voltage <= 0)
		pr_warn("cave: pid %d comm=%s with no vmin", task_tgid_vnr(p), p->comm);

	if (cave_random_vmin_enabled && !(p->flags & PF_KTHREAD)) {
		voffset = get_random_long() % cave_max_voffset;
		__cave_set_task(p, voffset);
	}
}

void cave_set_init_task(void)
{
	struct task_struct *p = current;

#ifdef CONFIG_UNISERVER_CAVE_USERSPACE
	p->cave_data = CAVE_USERSPACE_CONTEXT;
#else
	p->cave_data = CAVE_NOMINAL_CONTEXT;
#endif
}

#ifdef CONFIG_UNISERVER_CAVE_STATS
static int _print_cave_stats(char *buf, struct cave_stat *stat, const bool raw)
{
	int ret = 0;

#define SKIP(x)					\
	stat[x].skip = stat[x].skip_inc_wait + stat[x].skip_fast + stat[x].skip_slow + stat[x].skip_replay + stat[x].skip_race

	SKIP(0);
	SKIP(1);
	SKIP(2);
	SKIP(3);

#undef SKIP

#define LOCKED(x)				\
	stat[x].locked = stat[x].locked_inc + stat[x].locked_dec

	LOCKED(0);
	LOCKED(1);
	LOCKED(2);
	LOCKED(3);

#undef LOCKED

	/* +1 in case everything is 0 */
	stat[0].total = stat[0].inc + stat[0].dec + stat[0].skip + 1;
	stat[1].total = stat[1].inc + stat[1].dec + stat[1].skip + 1;
	stat[2].total = stat[2].inc + stat[2].dec + stat[2].skip + 1;
	stat[3].total = stat[3].inc + stat[3].dec + stat[3].skip + 1;

	if (raw) {
#define S(x)					\
		stat[0].x,			\
			stat[1].x,		\
			stat[2].x,		\
			stat[3].x
#define PRINT(x)							\
		ret += sprintf(buf + ret, #x " %ld %ld %ld %ld\n", S(x))

		PRINT(locked);
		PRINT(locked_inc);
		PRINT(locked_dec);
		PRINT(inc);
		PRINT(dec);
		PRINT(skip);
		PRINT(skip_inc_wait);
		PRINT(skip_fast);
		PRINT(skip_slow);
		PRINT(skip_replay);
		PRINT(skip_race);
#undef PRINT
#undef S
	}
	else {
#define __FIXED_STAT(d, x, __s) d.x = 100 * (d.x << __s) / d.total
#define FIXED_STAT(d, __s)				\
		__FIXED_STAT(d, locked,    __s);	\
		__FIXED_STAT(d, locked_inc,    __s);	\
		__FIXED_STAT(d, locked_dec,    __s);	\
		__FIXED_STAT(d, inc,       __s);	\
		__FIXED_STAT(d, dec,       __s);	\
		__FIXED_STAT(d, skip,      __s);	\
		__FIXED_STAT(d, skip_inc_wait, __s);	\
		__FIXED_STAT(d, skip_fast, __s);	\
		__FIXED_STAT(d, skip_slow, __s);	\
		__FIXED_STAT(d, skip_replay, __s);	\
		__FIXED_STAT(d, skip_race, __s);

		FIXED_STAT(stat[0], FSHIFT);
		FIXED_STAT(stat[1], FSHIFT);
		FIXED_STAT(stat[2], FSHIFT);
		FIXED_STAT(stat[3], FSHIFT);

#define S(x)	STAT_INT(x), STAT_FRAC(x)
#define PRINT(x)							\
		ret += sprintf(buf + ret, #x " "			\
			"%2ld.%02ld %2ld.%02ld %2ld.%02ld %2ld.%02ld\n", \
			S(stat[0].x), S(stat[1].x), S(stat[2].x), S(stat[3].x))

		PRINT(locked);
		PRINT(locked_inc);
		PRINT(locked_dec);
		PRINT(inc);
		PRINT(dec);
		PRINT(skip);
		PRINT(skip_inc_wait);
		PRINT(skip_fast);
		PRINT(skip_slow);
		PRINT(skip_replay);
		PRINT(skip_race);

#undef PRINT
#undef S
#undef __FIXED_STAT
#undef FIXED_STAT
	}

	return ret;
}

static int print_cave_stats(char *buf, const bool raw)
{
	int ret = 0;

	unsigned long flags;
	int enabled;

	struct cave_stat stat[4];

	spin_lock_irqsave(&cave_lock, flags);
	enabled = cave_enabled;
	if (enabled)
		stat[0] = cave_stat;
	spin_unlock_irqrestore(&cave_lock, flags);

	spin_lock_irqsave(&cave_stat_avg_lock, flags);
	stat[1] = cave_stat_avg[0];
	stat[2] = cave_stat_avg[1];
	stat[3] = cave_stat_avg[2];
	spin_unlock_irqrestore(&cave_stat_avg_lock, flags);

	if (enabled)
		ret += _print_cave_stats(buf + ret, stat, raw);

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
		spin_lock_irqsave(&cave_lock, flags);
		if (!cave_enabled) {
			write_target_voltage(CAVE_NOMINAL_VOLTAGE);
			write_voltage_msr(CAVE_NOMINAL_VOLTAGE);
			stats_init();
			cave_enabled = 1;
			show_msg = true;
		}
		spin_unlock_irqrestore(&cave_lock, flags);
		if (show_msg)
			printk(KERN_WARNING "cave: enabled\n");
	}
	else {
		spin_lock_irqsave(&cave_lock, flags);
		if (cave_enabled) {
			cave_enabled = 0;
			stats_clear();
			write_target_voltage(CAVE_NOMINAL_VOLTAGE);
			write_voltage_msr(CAVE_NOMINAL_VOLTAGE);
			show_msg = true;
		}
		spin_unlock_irqrestore(&cave_lock, flags);
		if (show_msg)
			printk(KERN_WARNING "cave: disabled\n");
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

	spin_lock_irqsave(&cave_lock, flags);
	if (voffset < cave_kernel_voffset)
		pr_warn("cave: new value of max_voffset greater than kernel voffset\n");
#ifdef CONFIG_UNISERVER_CAVE_USERSPACE
	else if (voffset < cave_userspace_voffset)
		pr_warn("cave: new value of max_voffset greater than userspace voffset\n");
#endif
	else
		cave_max_voffset = voffset;
	spin_unlock_irqrestore(&cave_lock, flags);
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

	spin_lock_irqsave(&cave_lock, flags);

	if (cave_enabled) {
		pr_warn("cave: must be disabled to change kernel voffset\n");
		goto out;
	}
	cave_kernel_voffset = voffset;

	write_lock_irq(&tasklist_lock);
	for_each_possible_cpu(i)
		idle_task(i)->cave_data = CAVE_KERNEL_CONTEXT;
	for_each_process_thread(g, p)
		if (p->flags & (PF_KTHREAD | PF_WQ_WORKER))
			p->cave_data = CAVE_KERNEL_CONTEXT;
	write_unlock_irq(&tasklist_lock);
 out:
	spin_unlock_irqrestore(&cave_lock, flags);

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

	spin_lock_irqsave(&cave_lock, flags);

	if (cave_enabled) {
		pr_warn("cave: must be disabled to change userspace voffset\n");
		goto out;
	}
	cave_userspace_voffset = voffset;

	write_lock_irq(&tasklist_lock);
	for_each_process_thread(g, p)
		if (!(p->flags & (PF_KTHREAD | PF_WQ_WORKER)))
			p->cave_data = CAVE_USERSPACE_CONTEXT;
	write_unlock_irq(&tasklist_lock);
 out:
	spin_unlock_irqrestore(&cave_lock, flags);

	return count;
}
#endif

static
ssize_t reset_stats_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t count)
{
	unsigned long flags;

	spin_lock_irqsave(&cave_lock, flags);
	if (cave_enabled) {
		stats_clear();
		stats_init();
	}
	spin_unlock_irqrestore(&cave_lock, flags);

	return count;
}

static
ssize_t stats_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += print_cave_stats(buf, false);

	return ret;
}

static
ssize_t raw_stats_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += print_cave_stats(buf, true);

	return ret;
}

static
ssize_t voltage_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unsigned long flags;

	int ret = 0;
	long veff;

	spin_lock_irqsave(&cave_lock, flags);
	veff = read_target_voltage();
	spin_unlock_irqrestore(&cave_lock, flags);

	ret += sprintf(buf + ret, "vmin %ld\n", veff);
	ret += sprintf(buf + ret, "voff_cached %3ld\n", -VOFFSET_OF(veff));

	return ret;
}

static
ssize_t debug_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

#ifdef CAVE_TIME_STATS
	int i;
	int j;

	unsigned long long time = 0;
	unsigned long long counter = 0;

	struct cave_time t;

	memset(&t, 0, sizeof(t));

	for_each_possible_cpu(i) {
		/* local copy */
		struct cave_time c = *per_cpu_ptr(&time_stats, i);

		for (j = 0; j < CAVE_CASES; j++) {
			t.time[j] += c.time[j];
			t.counter[j] += c.counter[j];
			time += c.time[j];
			counter += c.counter[j];
		}
		t.wait_target_time += c.wait_target_time;
		t.wait_target_counter += c.wait_target_counter;
		t.wait_curr_time += c.wait_curr_time;
		t.wait_curr_counter += c.wait_curr_counter;
	}

#define F(x, t) 100 * ((x) << FSHIFT) / (t)
#define S(x, t)	STAT_INT(F(x, t)), STAT_FRAC(F(x, t))
#define FMT	"%2llu.%02llu"

	if (time == 0 || counter == 0)
		return ret;

	ret += sprintf(buf + ret, "avg/total_cycles %llu\n", time / counter);

	for (j = 0; j < CAVE_CASES; j++) {
		ret += sprintf(buf + ret, "%s %llu " FMT "\n", cave_stat_name[j],
			       t.time[j] / (t.counter[j] + 1),
			       S(t.time[j], time));
	}

	if (t.wait_target_counter != 0) {
		ret += sprintf(buf + ret, "wait_target/cycles %llu\n", t.wait_target_time / t.wait_target_counter);
		ret += sprintf(buf + ret, "wait_target/total%% " FMT "\n", S(t.wait_target_time, time));
		ret += sprintf(buf + ret, "wait_target/inc%% " FMT "\n", S(t.wait_target_time, t.time[CAVE_INC]));
	}

	if (t.wait_curr_counter != 0) {
		ret += sprintf(buf + ret, "wait_curr/cycles %llu\n", t.wait_curr_time / t.wait_curr_counter);
		ret += sprintf(buf + ret, "wait_curr/total%% " FMT "\n", S(t.wait_curr_time, time));
		ret += sprintf(buf + ret, "wait_curr/inc_wait%% " FMT "\n", S(t.wait_curr_time, t.time[SKIP_INC_WAIT]));
	}

#undef FMT
#undef S
#undef F
#endif

	return ret;
}

static
ssize_t debug_store(struct kobject *kobj, struct kobj_attribute *attr,
		    const char *buf, size_t count)
{
	return count;
}

KERNEL_ATTR_RW(enable);
KERNEL_ATTR_RO(stats);
KERNEL_ATTR_RO(raw_stats);
KERNEL_ATTR_WO(reset_stats);
KERNEL_ATTR_RO(voltage);
KERNEL_ATTR_RW(random_vmin_enable);
KERNEL_ATTR_RW(max_voffset);
KERNEL_ATTR_RW(kernel_voffset);
#ifdef CONFIG_UNISERVER_CAVE_USERSPACE
KERNEL_ATTR_RW(userspace_voffset);
#endif
KERNEL_ATTR_RW(debug);

static struct attribute_group attr_group = {
	.name = "cave",
	.attrs = (struct attribute * []) {
		&enable_attr.attr,
		&reset_stats_attr.attr,
		&stats_attr.attr,
		&raw_stats_attr.attr,
		&voltage_attr.attr,
		&random_vmin_enable_attr.attr,
		&max_voffset_attr.attr,
		&kernel_voffset_attr.attr,
#ifdef CONFIG_UNISERVER_CAVE_USERSPACE
		&userspace_voffset_attr.attr,
#endif
		&debug_attr.attr,
		NULL
	}
};

int cave_init(void)
{
	unsigned long flags;
	int err;
	long voltage;

	err = sysfs_create_group(kernel_kobj, &attr_group);
	if (err) {
		pr_err("cave: failed\n");
		return err;
	}

	spin_lock_irqsave(&cave_lock, flags);

	voltage = read_voltage_msr();
	write_target_voltage(CAVE_NOMINAL_VOLTAGE);
	write_voltage_msr(CAVE_NOMINAL_VOLTAGE);
	spin_unlock_irqrestore(&cave_lock, flags);

	pr_warn("cave: msr voltage: %ld offset: %ld\n", voltage, -VOFFSET_OF(voltage));

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
		printk(KERN_WARNING "cave: pid %d vmin: %ld voff: %3ld\n",
		       task_tgid_vnr(p), p->cave_data.voltage,
		       -VOFFSET_OF(p->cave_data.voltage));
		*/
		return 0;
	}

	return -1;
}
