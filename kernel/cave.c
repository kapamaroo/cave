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

#define TO_VOFFSET_DATA(__val)	(__val ? (0x800ULL - (u64)__val) << 21 : 0ULL)
#define TO_VOFFSET_VAL(__data)    (__data ? (0x800ULL - ((u64)__data >> 21)) : 0ULL)

#define CORE_VOFFSET_VAL(__val)		(0x8000001100000000ULL | TO_VOFFSET_DATA(__val))
#define CACHE_VOFFSET_VAL(__val)	(0x8000021100000000ULL | TO_VOFFSET_DATA(__val))

struct cave_stat {
	long inc;
	long dec;
	long skip_fast;
	long skip_slow;
	long skip;
	long locked;
	long total;
};

static volatile int cave_enabled = 0;
static DEFINE_SPINLOCK(cave_lock);
static DEFINE_SPINLOCK(cave_stat_avg_lock);
DEFINE_PER_CPU(cave_data_t, context);
static volatile int cave_random_vmin_enabled __read_mostly = 0;
static volatile int cave_kernel_voffset __read_mostly = CAVE_DEFAULT_KERNEL_VOFFSET;
static volatile int cave_max_voffset __read_mostly = CAVE_DEFAULT_MAX_VOFFSET;

static volatile long effective_voltage = CAVE_NOMINAL_VOLTAGE;

static struct cave_stat cave_stat;
static struct cave_stat cave_stat_avg[3];
static int stat_samples[3] = { 1, 1, 1 };

#define FSHIFT	11
#define FIXED_1	(1 << FSHIFT)
#define STAT_INT(x)	((x) >> FSHIFT)
#define STAT_FRAC(x)	STAT_INT(((x) & (FIXED_1 - 1)) * 100)

#define __RUNNING_AVG_STAT(d, s, n, x)		\
	d.x = (d.x * (n - 1) + s.x) / n
#define RUNNING_AVG_STAT(d, s, n, l)		\
	__RUNNING_AVG_STAT(d, s, n, inc);	\
	__RUNNING_AVG_STAT(d, s, n, dec);	\
	__RUNNING_AVG_STAT(d, s, n, skip_fast);	\
	__RUNNING_AVG_STAT(d, s, n, skip_slow);	\
	__RUNNING_AVG_STAT(d, s, n, locked);	\
	if (n < l)				\
		n++;

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

	spin_lock_irqsave(&cave_stat_avg_lock, flags);
	RUNNING_AVG_STAT(cave_stat_avg[0], new_stat, stat_samples[0], 60);
	RUNNING_AVG_STAT(cave_stat_avg[1], new_stat, stat_samples[1], 5 * 60);
	RUNNING_AVG_STAT(cave_stat_avg[2], new_stat, stat_samples[2], 10 * 60);
	spin_unlock_irqrestore(&cave_stat_avg_lock, flags);

	hrtimer_forward_now(&stats_hrtimer, stats_period_time);

	return HRTIMER_RESTART;
}

static void stats_init(void)
{
	memset(&cave_stat, 0, sizeof(struct cave_stat));
	memset(cave_stat_avg, 0, sizeof(cave_stat_avg));
	stat_samples[0] = 1;
	stat_samples[1] = 1;
	stat_samples[2] = 1;

	stats_period_time = ktime_set(1, 0);
	hrtimer_init(&stats_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stats_hrtimer.function = stats_gather;
	hrtimer_start(&stats_hrtimer, stats_period_time, HRTIMER_MODE_REL);
}

static void stats_clear(void)
{
	hrtimer_cancel(&stats_hrtimer);
}

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

static void write_voltage_cached(long new_voltage)
{
	if (unlikely(new_voltage < 0 || new_voltage > CAVE_NOMINAL_VOLTAGE))
		return;

	effective_voltage = new_voltage;
}

static void write_voltage_msr(long new_voltage)
{
	u64 new_voffset;

	if (unlikely(new_voltage < 0 || new_voltage > CAVE_NOMINAL_VOLTAGE))
		return;

	if (unlikely(new_voltage != effective_voltage)) {
		WARN_ON_ONCE(1);
		effective_voltage = new_voltage;
	}

	if (!new_voltage)
		pr_warn("cave: pid %d (%s) fix", task_tgid_vnr(current), current->comm);

	new_voffset = VOFFSET_OF(new_voltage);
	write_voffset_msr(new_voffset);
}

static long read_voltage_cached(void)
{
	return effective_voltage;
}

static long read_voltage_msr(void)
{
	long voffset_msr, voltage_msr, voltage_cached;

	voffset_msr = read_voffset_msr();
	voltage_cached = effective_voltage;

	voltage_msr = VOLTAGE_OF(voffset_msr);

	WARN_ON_ONCE(cave_enabled && voltage_msr != voltage_cached);

	return voltage_msr;
}

static void wait_voltage(long new_voltage)
{
	long voltage;

	while(new_voltage > (voltage = read_voltage_msr()))
		cpu_relax();
}

static long select_voltage(long prev_vmin, cave_data_t my_context)
{
	long new_vmin = my_context.voltage;
	int i;

	if (new_vmin == prev_vmin) {
		cave_stat.skip_fast++;
		return -1;
	}
	else if (new_vmin > prev_vmin) {
		cave_stat.inc++;
		return new_vmin;
	}

	for_each_possible_cpu(i) {
		cave_data_t tmp = per_cpu(context, i);
		if(tmp.voltage > new_vmin)
			new_vmin = tmp.voltage;
	}

	if (new_vmin == prev_vmin) {
		new_vmin = -1;
		cave_stat.skip_slow++;
	}
	else if (new_vmin < prev_vmin)
		cave_stat.dec++;
	else
		WARN_ON(1);

	return new_vmin;
}

static void _cave_switch(cave_data_t new_context)
{
	unsigned long flags;
	long new_vmin;
	long prev_vmin;
	int done = 0;

	if (!cave_enabled)
		return;

	while (!spin_trylock_irqsave(&cave_lock, flags)) {
		done++;
		cpu_relax();
	}

	if (done)
		cave_stat.locked++;

	this_cpu_write(context, new_context);
	prev_vmin = read_voltage_cached();
	new_vmin = select_voltage(prev_vmin, new_context);
	write_voltage_cached(new_vmin);
	write_voltage_msr(new_vmin);
	if (new_vmin > prev_vmin)
		wait_voltage(new_context.voltage);

	spin_unlock_irqrestore(&cave_lock, flags);
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

void cave_set_task(struct task_struct *p)
{
	unsigned long voffset = 0;

	if (p->cave_data.voltage == 0)
		pr_warn("cave: pid %d comm=%s with no vmin", task_tgid_vnr(p), p->comm);

	if (cave_random_vmin_enabled && !(p->flags & PF_KTHREAD)) {
		voffset = get_random_long() % cave_max_voffset;
		__cave_set_task(p, voffset);
	}
}

void cave_set_init_task(void)
{
	struct task_struct *p = current;

	p->cave_data = CAVE_NOMINAL_CONTEXT;
}

static int _print_cave_stats(char *buf, struct cave_stat *stat, const bool raw)
{
	int ret = 0;

	stat[0].skip = stat[0].skip_fast + stat[0].skip_slow;
	stat[1].skip = stat[1].skip_fast + stat[1].skip_slow;
	stat[2].skip = stat[2].skip_fast + stat[2].skip_slow;
	stat[3].skip = stat[3].skip_fast + stat[3].skip_slow;

	/* +1 in case everything is 0 */
	stat[0].total = stat[0].inc + stat[0].dec + stat[0].skip + 1;
	stat[1].total = stat[1].inc + stat[1].dec + stat[1].skip + 1;
	stat[2].total = stat[2].inc + stat[2].dec + stat[2].skip + 1;
	stat[3].total = stat[3].inc + stat[3].dec + stat[3].skip + 1;

	if (raw) {
#define S(x)		   \
		stat[0].x, \
		stat[1].x, \
		stat[2].x, \
		stat[3].x

		ret += sprintf(buf + ret, "locked %ld %ld %ld %ld\n", S(locked));
		ret += sprintf(buf + ret, "inc %ld %ld %ld %ld\n", S(inc));
		ret += sprintf(buf + ret, "dec %ld %ld %ld %ld\n", S(dec));
		ret += sprintf(buf + ret, "skip %ld %ld %ld %ld\n", S(skip));
		ret += sprintf(buf + ret, "skip_fast %ld %ld %ld %ld\n", S(skip_fast));
		ret += sprintf(buf + ret, "skip_slow %ld %ld %ld %ld\n", S(skip_slow));
#undef S
	}
	else {
#define __FIXED_STAT(d, x, __s) d.x = 100 * (d.x << __s) / d.total
#define FIXED_STAT(d, __s)				\
		__FIXED_STAT(d, inc,       __s);	\
		__FIXED_STAT(d, dec,       __s);	\
		__FIXED_STAT(d, skip_fast, __s);	\
		__FIXED_STAT(d, skip_slow, __s);	\
		__FIXED_STAT(d, skip,      __s);	\
		__FIXED_STAT(d, locked,    __s);

		FIXED_STAT(stat[0], FSHIFT);
		FIXED_STAT(stat[1], FSHIFT);
		FIXED_STAT(stat[2], FSHIFT);
		FIXED_STAT(stat[3], FSHIFT);

#define S(x)	STAT_INT(x), STAT_FRAC(x)

		ret += sprintf(buf + ret, "locked "
			       "%2ld.%02ld %2ld.%02ld %2ld.%02ld %2ld.%02ld\n",
			       S(stat[0].locked), S(stat[1].locked),
			       S(stat[2].locked), S(stat[3].locked));
		ret += sprintf(buf + ret, "inc "
			       "%2ld.%02ld %2ld.%02ld %2ld.%02ld %2ld.%02ld\n",
			       S(stat[0].inc), S(stat[1].inc),
			       S(stat[2].inc), S(stat[3].inc));
		ret += sprintf(buf + ret, "dec "
			       "%2ld.%02ld %2ld.%02ld %2ld.%02ld %2ld.%02ld\n",
			       S(stat[0].dec), S(stat[1].dec),
			       S(stat[2].dec), S(stat[3].dec));
		ret += sprintf(buf + ret, "skip "
			       "%2ld.%02ld %2ld.%02ld %2ld.%02ld %2ld.%02ld\n",
			       S(stat[0].skip), S(stat[1].skip),
			       S(stat[2].skip), S(stat[3].skip));
		ret += sprintf(buf + ret, "skip_fast "
			       "%2ld.%02ld %2ld.%02ld %2ld.%02ld %2ld.%02ld\n",
			       S(stat[0].skip_fast), S(stat[1].skip_fast),
			       S(stat[2].skip_fast), S(stat[3].skip_fast));
		ret += sprintf(buf + ret, "skip_slow "
			       "%2ld.%02ld %2ld.%02ld %2ld.%02ld %2ld.%02ld\n",
			       S(stat[0].skip_slow), S(stat[1].skip_slow),
			       S(stat[2].skip_slow), S(stat[3].skip_slow));
#undef __FIXED_STAT
#undef FIXED_STAT
#undef S
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

/* sysfs interface */
#define KERNEL_ATTR_RW(_name) \
static struct kobj_attribute _name##_attr = __ATTR_RW(_name)

#define KERNEL_ATTR_RO(_name) \
static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

#define KERNEL_ATTR_WO(_name) \
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
			write_voltage_cached(CAVE_NOMINAL_VOLTAGE);
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
			write_voltage_cached(CAVE_NOMINAL_VOLTAGE);
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
	cave_kernel_voffset = voffset;
	for_each_possible_cpu(i)
		idle_task(i)->cave_data = CAVE_KERNEL_CONTEXT;
	spin_unlock_irqrestore(&cave_lock, flags);

	return count;
}

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
	veff = read_voltage_cached();
	spin_unlock_irqrestore(&cave_lock, flags);

	ret += sprintf(buf + ret, "vmin %ld\n", veff);
	ret += sprintf(buf + ret, "voff_cached %3ld\n", -VOFFSET_OF(veff));

	return ret;
}

KERNEL_ATTR_RW(enable);
KERNEL_ATTR_RO(stats);
KERNEL_ATTR_RO(raw_stats);
KERNEL_ATTR_WO(reset_stats);
KERNEL_ATTR_RO(voltage);
KERNEL_ATTR_RW(random_vmin_enable);
KERNEL_ATTR_RW(max_voffset);
KERNEL_ATTR_RW(kernel_voffset);

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
		NULL
	}
};

int cave_init(void)
{
	unsigned long flags;
	int i;
	int err;
	long voltage;

        err = sysfs_create_group(kernel_kobj, &attr_group);
        if (err) {
                pr_err("cave: failed\n");
                return err;
        }

	spin_lock_irqsave(&cave_lock, flags);
	for_each_possible_cpu(i) {
		per_cpu(context, i) = CAVE_KERNEL_CONTEXT;
		idle_task(i)->cave_data = CAVE_KERNEL_CONTEXT;
	}

        voltage = read_voltage_msr();
	write_voltage_cached(voltage);
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
