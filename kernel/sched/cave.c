/* CAVE - Context Aware Voltage Elevator */

#include <asm/msr.h>
#include <asm-generic/delay.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/random.h>
#include <linux/cave.h>

#define NOMINAL_VOLTAGE	4000
#define KERNEL_VOLTAGE	(NOMINAL_VOLTAGE - 100)

#define TO_VOFFSET_DATA(val)	(val ? (0x800ULL - (u64)val) << 21 : 0ULL)
#define TO_VOFFSET_VAL(data)    (data ? (0x800ULL - ((u64)data >> 21)) : 0ULL)

#define CORE_VOFFSET_VAL(val)		(0x8000001100000000ULL | TO_VOFFSET_DATA(val))
#define CACHE_VOFFSET_VAL(val)		(0x8000021100000000ULL | TO_VOFFSET_DATA(val))

struct cave_stat {
	atomic_long_t inc;
	atomic_long_t dec;
	atomic_long_t skip_fast;
	atomic_long_t skip_slow;
	atomic_long_t locked;
};

#define KERNEL_CONTEXT (cave_data_t){ .voltage = KERNEL_VOLTAGE }
#define NOMINAL_CONTEXT	(cave_data_t){ .voltage = NOMINAL_VOLTAGE }

static volatile int cave_enabled = 0;
static DEFINE_SPINLOCK(cave_lock);
DEFINE_PER_CPU(cave_data_t, context);
static struct cave_stat cave_stat;

#ifdef CONFIG_UNISERVER_CAVE_TEST_MODE
static atomic_long_t effective_voltage = ATOMIC_LONG_INIT(0);
#endif

static inline void write_voffset(u64 voffset)
{
#ifdef CONFIG_UNISERVER_CAVE_TEST_MODE
	udelay(155);
	atomic_long_set(&effective_voltage, NOMINAL_VOLTAGE - voffset);
	return;
#else
	wrmsrl(0x150, CORE_VOFFSET_VAL(voffset));
	wrmsrl(0x150, CACHE_VOFFSET_VAL(voffset));
#endif
}

static inline u64 read_voffset(void)
{
#ifdef CONFIG_UNISERVER_CAVE_TEST_MODE
	return NOMINAL_VOLTAGE - (u64)atomic_long_read(&effective_voltage);
#else
	u64 voffset;

	wrmsrl(0x150, 0x8000001000000000);
	rdmsrl(0x150, voffset);

	return TO_VOFFSET_VAL(voffset);
#endif
}

static void write_voltage(long new_voltage)
{
	if (new_voltage < 0 || new_voltage > NOMINAL_VOLTAGE)
		return;

	write_voffset(NOMINAL_VOLTAGE - new_voltage);
}

static long read_voltage(void)
{
	long voffset = read_voffset();
	return NOMINAL_VOLTAGE - voffset;
}

static void wait_voltage(long new_voltage)
{
	udelay(155);
	return;
	while(new_voltage > read_voltage())
		cpu_relax();
}

static long select_voltage(long prev_vmin, cave_data_t my_context)
{
	long new_vmin = my_context.voltage;
	int i;

	if (new_vmin == prev_vmin) {
		atomic_long_inc(&cave_stat.skip_fast);
		return -1;
	}
	else if (new_vmin > prev_vmin) {
		atomic_long_inc(&cave_stat.inc);
		return new_vmin;
	}

	for_each_possible_cpu(i) {
		cave_data_t tmp = per_cpu(context, i);
		if(tmp.voltage > new_vmin)
			new_vmin = tmp.voltage;
	}

	if (new_vmin == prev_vmin) {
		new_vmin = -1;
		atomic_long_inc(&cave_stat.skip_slow);
	}
	else if (new_vmin < prev_vmin)
		atomic_long_inc(&cave_stat.dec);
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
			atomic_long_inc(&cave_stat.locked);
			done = true;
		}
	}

	this_cpu_write(context, new_context);
	prev_vmin = read_voltage();
	new_vmin = select_voltage(prev_vmin, new_context);
	write_voltage(new_vmin);

	spin_unlock_irqrestore(&cave_lock, flags);

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
	p->cave_data.voltage = NOMINAL_VOLTAGE - (get_random_long() % 250);

	/*
	if (cave_enabled)
		printk(KERN_WARNING "cave: pid %d vmin: %ld voff: %3ld\n",
				task_tgid_vnr(p), p->cave_data,
				NOMINAL_VOLTAGE - p->cave_data);
	 */
}

void print_cave(struct seq_file *p)
{
	long inc = atomic_long_read(&cave_stat.inc);
	long dec = atomic_long_read(&cave_stat.dec);
	long skip_fast = atomic_long_read(&cave_stat.skip_fast);
	long skip_slow = atomic_long_read(&cave_stat.skip_slow);
	long locked = atomic_long_read(&cave_stat.locked);
	long total = inc + dec + skip_fast + skip_slow;

#ifdef CONFIG_UNISERVER_CAVE_TEST_MODE
	long veff = atomic_long_read(&effective_voltage);
#else
	long veff = read_voltage();
#endif

	if (!total)
		total = 100;

	seq_printf(p, "\n");
	seq_printf(p, "cave: vmin: %ld voff: %3ld\n",
			veff, NOMINAL_VOLTAGE - veff);
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

	for_each_possible_cpu(i) {
		per_cpu(context, i) = KERNEL_CONTEXT;
		idle_task(i)->cave_data = KERNEL_CONTEXT;
	}

#ifdef CONFIG_UNISERVER_CAVE_TEST_MODE
	atomic_long_set(&effective_voltage, NOMINAL_VOLTAGE);
#endif

        err = sysfs_create_group(kernel_kobj, &attr_group);
        if (err) {
                pr_err("cave: failed\n");
                return err;
        }

        voltage = read_voltage();
        pr_warn("cave: nominal voltage: %ld - %ld\n", voltage, NOMINAL_VOLTAGE - voltage);

	return 0;
}
late_initcall(cave_init);
