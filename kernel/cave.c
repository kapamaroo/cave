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

#define CAVE_NOMINAL_VOLTAGE		4000ULL
#define CAVE_DEFAULT_MAX_VOFFSET	 250ULL
#define CAVE_DEFAULT_KERNEL_VOFFSET	 111ULL

#define CAVE_DEFAULT_MAX_VMIN		VOLTAGE_OF(CAVE_DEFAULT_MAX_VOFFSET)
#define CAVE_DEFAULT_KERNEL_VMIN	VOLTAGE_OF(CAVE_DEFAULT_KERNEL_VOFFSET)

#define VOFFSET_OF(voltage)	((long)CAVE_NOMINAL_VOLTAGE - (long)(voltage))
#define VOLTAGE_OF(voffset)	((long)CAVE_NOMINAL_VOLTAGE - (long)(voffset))

#define TO_VOFFSET_DATA(val)	(val ? (0x800ULL - (u64)val) << 21 : 0ULL)
#define TO_VOFFSET_VAL(data)    (data ? (0x800ULL - ((u64)data >> 21)) : 0ULL)

#define CORE_VOFFSET_VAL(val)		(0x8000001100000000ULL | TO_VOFFSET_DATA(val))
#define CACHE_VOFFSET_VAL(val)		(0x8000021100000000ULL | TO_VOFFSET_DATA(val))

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
DEFINE_PER_CPU(cave_data_t, context);
static volatile int cave_random_vmin_enabled __read_mostly = 0;
static volatile int cave_kernel_vmin __read_mostly = CAVE_DEFAULT_KERNEL_VMIN;
static volatile int cave_max_vmin __read_mostly = CAVE_DEFAULT_MAX_VMIN;

static volatile long effective_voltage = CAVE_NOMINAL_VOLTAGE;

#define CAVE_KERNEL_CONTEXT (cave_data_t){ .voltage = cave_kernel_vmin }
#define CAVE_NOMINAL_CONTEXT	(cave_data_t){ .voltage = CAVE_NOMINAL_VOLTAGE }

static struct cave_stat cave_stat;
static struct cave_stat cave_stat_avg[3];
static int stat_samples[3] = { 1, 1, 1 };

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

	WARN_ON(!cave_enabled);

	spin_lock_irqsave(&cave_lock, flags);
	new_stat = cave_stat;
	memset(&cave_stat, 0, sizeof(struct cave_stat));
	spin_unlock_irqrestore(&cave_lock, flags);

	RUNNING_AVG_STAT(cave_stat_avg[0], new_stat, stat_samples[0], 60);
	RUNNING_AVG_STAT(cave_stat_avg[1], new_stat, stat_samples[1], 5 * 60);
	RUNNING_AVG_STAT(cave_stat_avg[2], new_stat, stat_samples[2], 10 * 60);

	hrtimer_forward_now(&stats_hrtimer, stats_period_time);

	return HRTIMER_RESTART;
}

static void stats_init(void)
{
	stats_period_time = ktime_set(1, 0);
	hrtimer_init(&stats_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stats_hrtimer.function = stats_gather;
	hrtimer_start(&stats_hrtimer, stats_period_time, HRTIMER_MODE_REL);
}

static void stats_clear(void)
{
	hrtimer_cancel(&stats_hrtimer);
}

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
	if (unlikely(new_voltage < 0 || new_voltage > CAVE_NOMINAL_VOLTAGE))
		return;

	if (unlikely(new_voltage != effective_voltage)) {
		WARN_ON_ONCE(1);
		effective_voltage = new_voltage;
	}

	write_voffset_msr(VOFFSET_OF(new_voltage));
}

static long read_voltage_cached(void)
{
	return effective_voltage;
}

static long read_voltage_msr(void)
{
	long voffset = read_voffset_msr();
	long voltage = VOLTAGE_OF(voffset);

	WARN_ON_ONCE(voltage != effective_voltage);

	return voltage;
}

static void wait_voltage(long new_voltage)
{
	while(new_voltage > read_voltage_msr())
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
	bool done = false;

	if (!cave_enabled)
		return;

	while (!spin_trylock_irqsave(&cave_lock, flags)) {
		if (!done) {
			cave_stat.locked++;
			done = true;
		}
	}

	this_cpu_write(context, new_context);
	prev_vmin = read_voltage_cached();
	new_vmin = select_voltage(prev_vmin, new_context);
	write_voltage_cached(new_vmin);
	write_voltage_msr(new_vmin);

	spin_unlock_irqrestore(&cave_lock, flags);

	if(new_vmin > prev_vmin)
		wait_voltage(new_vmin);
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
	if (voffset < 0 || voffset > cave_max_vmin) {
		pr_warn("cave: voffset out of range (%ld) comm=%s\n", voffset, p->comm);
		voffset = 0;
	}

	__cave_set_task(p, voffset);
}

void cave_set_task(struct task_struct *p)
{
	if (cave_random_vmin_enabled) {
		unsigned long voffset = 0;
		voffset = get_random_long() % cave_max_vmin;
		__cave_set_task(p, voffset);
	}
}

#define FSHIFT	11
#define FIXED_1	(1 << FSHIFT)
#define STAT_INT(x)	((x) >> FSHIFT)
#define STAT_FRAC(x)	STAT_INT(((x) & (FIXED_1 - 1)) * 100)

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
		ret += sprintf(buf + ret, "locked %ld %ld %ld %ld\n",
			       stat[0].locked, stat[1].locked,
			       stat[2].locked, stat[3].locked);
		ret += sprintf(buf + ret, "inc %ld %ld %ld %ld\n",
			       stat[0].inc, stat[1].inc, stat[2].inc, stat[3].inc);
		ret += sprintf(buf + ret, "dec %ld %ld %ld %ld\n",
			       stat[0].dec, stat[1].dec, stat[2].dec, stat[3].dec);
		ret += sprintf(buf + ret, "skip %ld %ld %ld %ld\n",
			       stat[0].skip, stat[1].skip, stat[2].skip, stat[3].skip);
		ret += sprintf(buf + ret, "skip_fast %ld %ld %ld %ld\n",
			       stat[0].skip_fast, stat[1].skip_fast,
			       stat[2].skip_fast, stat[3].skip_fast);
		ret += sprintf(buf + ret, "skip_slow %ld %ld %ld %ld\n",
			       stat[0].skip_slow, stat[1].skip_slow,
			       stat[2].skip_slow, stat[3].skip_slow);
	}
	else {
#define __FIXED_STAT(d, x) d.x = (d.x << FSHIFT) / d.total
#define FIXED_STAT(d) \
		__FIXED_STAT(d, inc); \
		__FIXED_STAT(d, dec); \
		__FIXED_STAT(d, skip_fast); \
		__FIXED_STAT(d, skip_slow); \
		__FIXED_STAT(d, skip); \
		__FIXED_STAT(d, locked);

		FIXED_STAT(stat[0]);
		FIXED_STAT(stat[1]);
		FIXED_STAT(stat[2]);
		FIXED_STAT(stat[3]);

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
	if (enabled) {
		stat[0] = cave_stat;
		memcpy(stat + 1, cave_stat_avg, sizeof(cave_stat_avg));
	}
	spin_unlock_irqrestore(&cave_lock, flags);

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

	if (strncmp(buf, "1", 1) == 0) {
		spin_lock_irqsave(&cave_lock, flags);
		if (!cave_enabled) {
			memset(&cave_stat, 0, sizeof(struct cave_stat));
			memset(cave_stat_avg, 0, sizeof(cave_stat_avg));
			cave_enabled = 1;
		}
		spin_unlock_irqrestore(&cave_lock, flags);
		stats_init();
		printk(KERN_WARNING "cave: enabled\n");
	}
	else {
		stats_clear();
		spin_lock_irqsave(&cave_lock, flags);
		if (cave_enabled) {
			cave_enabled = 0;
			memset(&cave_stat, 0, sizeof(struct cave_stat));
			memset(cave_stat_avg, 0, sizeof(cave_stat_avg));
		}
		write_voltage_cached(CAVE_NOMINAL_VOLTAGE);
		write_voltage_msr(CAVE_NOMINAL_VOLTAGE);
		spin_unlock_irqrestore(&cave_lock, flags);
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
	if (strncmp(buf, "1", 1) == 0) {
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
ssize_t max_vmin_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%d\n", cave_max_vmin);

	return ret;
}

static
ssize_t max_vmin_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
	int val;
	int err;

	err = kstrtouint(buf, 10, &val);
	if (err) {
		pr_warn("cave: invalid max_vmin value\n");
		return count;
	}

	cave_max_vmin = val;

	return count;
}

static
ssize_t kernel_vmin_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%d\n", cave_kernel_vmin);

	return ret;
}

static
ssize_t kernel_vmin_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t count)
{
	unsigned long flags;
	int val;
	int err;
	int i;

	err = kstrtouint(buf, 10, &val);
	if (err) {
		pr_warn("cave: invalid kernel_vmin value\n");
		return count;
	}

	if (val > cave_max_vmin) {
		pr_warn("cave: kernel_vmin out of range\n");
		return count;
	}

	spin_lock_irqsave(&cave_lock, flags);
	cave_kernel_vmin = val;
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
		memset(&cave_stat, 0, sizeof(struct cave_stat));
		memset(cave_stat_avg, 0, sizeof(cave_stat_avg));
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
KERNEL_ATTR_RW(max_vmin);
KERNEL_ATTR_RW(kernel_vmin);

static struct attribute_group attr_group = {
	.name = "cave",
	.attrs = (struct attribute * []) {
		&enable_attr.attr,
		&reset_stats_attr.attr,
		&stats_attr.attr,
		&raw_stats_attr.attr,
		&voltage_attr.attr,
		&random_vmin_enable_attr.attr,
		&max_vmin_attr.attr,
		&kernel_vmin_attr.attr,
		NULL
	}
};

int cave_init(void)
{
	int i;
	int err;
	long voltage;

        err = sysfs_create_group(kernel_kobj, &attr_group);
        if (err) {
                pr_err("cave: failed\n");
                return err;
        }

	for_each_possible_cpu(i) {
		per_cpu(context, i) = CAVE_KERNEL_CONTEXT;
		idle_task(i)->cave_data = CAVE_KERNEL_CONTEXT;
	}

        voltage = read_voltage_msr();
	write_voltage_cached(voltage);

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
		printk(KERN_WARNING "cave: pid %d vmin: %ld voff: %3ld\n",
		       task_tgid_vnr(p), p->cave_data.voltage,
		       -VOFFSET_OF(p->cave_data.voltage));
		return 0;
	}

	return -1;
}
