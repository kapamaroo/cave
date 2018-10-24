/* CAVE - Context Aware Voltage Elevator */

#include <asm/msr.h>
#include <asm-generic/delay.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/random.h>
#include <linux/cave.h>

#define NOMINAL_VOLTAGE	4000ULL
#define VOFFSET_OF(voltage)	((long)NOMINAL_VOLTAGE - (long)(voltage))
#define VOLTAGE_OF(voffset)	((long)NOMINAL_VOLTAGE - (long)(voffset))

#define KERNEL_VOLTAGE	(NOMINAL_VOLTAGE - 111)

#define TO_VOFFSET_DATA(val)	(val ? (0x800ULL - (u64)val) << 21 : 0ULL)
#define TO_VOFFSET_VAL(data)    (data ? (0x800ULL - ((u64)data >> 21)) : 0ULL)

#define CORE_VOFFSET_VAL(val)		(0x8000001100000000ULL | TO_VOFFSET_DATA(val))
#define CACHE_VOFFSET_VAL(val)		(0x8000021100000000ULL | TO_VOFFSET_DATA(val))

#if 0
typedef atomic_long_t stat_cnt_t;
#define STAT_INC(x)	atomic_long_inc(&(x))
#define STAT_READ(x)	atomic_long_read(&(x))
#define STAT_INIT(val)	ATOMIC_LONG_INIT(val)
#define STAT_SET(x, v)	atomic_long_set(&x, v)
#else
typedef long stat_cnt_t;
#define STAT_INC(x)	(x)++
#define STAT_READ(x)	(x)
#define STAT_INIT(val)	(val)
#define STAT_SET(x, v)	x = v
#endif

struct cave_stat {
	stat_cnt_t inc;
	stat_cnt_t dec;
	stat_cnt_t skip_fast;
	stat_cnt_t skip_slow;
	stat_cnt_t locked;
};

#define KERNEL_CONTEXT (cave_data_t){ .voltage = KERNEL_VOLTAGE }
#define NOMINAL_CONTEXT	(cave_data_t){ .voltage = NOMINAL_VOLTAGE }

static volatile int cave_enabled = 0;
static DEFINE_SPINLOCK(cave_lock);
DEFINE_PER_CPU(cave_data_t, context);
static struct cave_stat cave_stat;

static stat_cnt_t effective_voltage = STAT_INIT(NOMINAL_VOLTAGE);

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
	if (unlikely(new_voltage < 0 || new_voltage > NOMINAL_VOLTAGE))
		return;

	STAT_SET(effective_voltage, new_voltage);
}

static void write_voltage_msr(long new_voltage)
{
	if (unlikely(new_voltage < 0 || new_voltage > NOMINAL_VOLTAGE))
		return;

	if (unlikely(new_voltage != STAT_READ(effective_voltage))) {
		WARN_ON(1);
		STAT_SET(effective_voltage, new_voltage);
	}

	write_voffset_msr(VOFFSET_OF(new_voltage));
}

static long read_voltage_cached(void)
{
	return STAT_READ(effective_voltage);
}

static long read_voltage_msr(void)
{
	long voffset = read_voffset_msr();
	return VOLTAGE_OF(voffset);
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
		STAT_INC(cave_stat.skip_fast);
		return -1;
	}
	else if (new_vmin > prev_vmin) {
		STAT_INC(cave_stat.inc);
		return new_vmin;
	}

	for_each_possible_cpu(i) {
		cave_data_t tmp = per_cpu(context, i);
		if(tmp.voltage > new_vmin)
			new_vmin = tmp.voltage;
	}

	if (new_vmin == prev_vmin) {
		new_vmin = -1;
		STAT_INC(cave_stat.skip_slow);
	}
	else if (new_vmin < prev_vmin)
		STAT_INC(cave_stat.dec);
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

	if (unlikely(!cave_enabled))
		return;

	while (!spin_trylock_irqsave(&cave_lock, flags)) {
		if (!done) {
			STAT_INC(cave_stat.locked);
			done = true;
		}
	}

	this_cpu_write(context, new_context);
	prev_vmin = read_voltage_cached();
	new_vmin = select_voltage(prev_vmin, new_context);
	write_voltage_cached(new_vmin);

	spin_unlock_irqrestore(&cave_lock, flags);

	write_voltage_msr(new_vmin);
	if(new_vmin > prev_vmin)
		wait_voltage(new_vmin);
}

__visible void cave_entry_switch(void)
{
	_cave_switch(KERNEL_CONTEXT);
}

__visible void cave_exit_switch(void)
{
	_cave_switch(current->cave_data);
}

void cave_set_task(struct task_struct *p)
{
	p->cave_data.voltage = VOLTAGE_OF((get_random_long() % 250));

	/*
	if (cave_enabled)
		printk(KERN_WARNING "cave: pid %d vmin: %ld voff: %3ld\n",
				task_tgid_vnr(p), p->cave_data.voltage,
				VOFFSET_OF(p->cave_data.voltage));
	 */
}

void print_cave(struct seq_file *p)
{
	long inc = STAT_READ(cave_stat.inc);
	long dec = STAT_READ(cave_stat.dec);
	long skip_fast = STAT_READ(cave_stat.skip_fast);
	long skip_slow = STAT_READ(cave_stat.skip_slow);
	long locked = STAT_READ(cave_stat.locked);
	long total = inc + dec + skip_fast + skip_slow;

	long veff = read_voltage_cached();

	if (!total)
		total = 100;

	seq_printf(p, "\n");
	seq_printf(p, "cave: vmin: %ld voff: %3ld\n",
			veff, VOFFSET_OF(veff));
	seq_printf(p, "cave: locked %ld %%\n",
			100 * locked / total);
	seq_printf(p, "cave: inc %ld %%, dec %ld %%\n",
			100 * inc / total,
			100 * dec / total);
	seq_printf(p, "cave: skip %ld %% (fast %ld %%, slow %ld %%)\n",
			100 * (skip_fast + skip_slow) / total,
			100 * skip_fast / total,
			100 * skip_slow / total);
}

/* sysfs interface */
#define KERNEL_ATTR_RW(_name) \
static struct kobj_attribute _name##_attr = \
       __ATTR(_name, 0644, _name##_show, _name##_store)

ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf, "%d\n", cave_enabled);

	return ret;
}

ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
	               const char *buf, size_t count)
{
	if (strncmp(buf, "1", 1) == 0) {
		cave_enabled = 1;
		printk(KERN_WARNING "cave: enabled\n");
	}
	else {
		cave_enabled = 0;
		printk(KERN_WARNING "cave: disabled\n");
	}

	return count;
}

KERNEL_ATTR_RW(enable);

static struct attribute_group attr_group = {
	.name = "cave",
	.attrs = (struct attribute * []) {
		&enable_attr.attr,
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
		per_cpu(context, i) = KERNEL_CONTEXT;
		idle_task(i)->cave_data = KERNEL_CONTEXT;
	}

        voltage = read_voltage_msr();

        pr_warn("cave: msr voltage: %ld offset: %ld\n", voltage, VOFFSET_OF(voltage));

	return 0;
}
late_initcall(cave_init);
