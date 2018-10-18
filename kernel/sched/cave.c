/* CAVE - Context Aware Voltage Elevator */

#include <asm/msr.h>
#include <asm-generic/delay.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/random.h>
#include <linux/cave.h>

#define NOMINAL_VOLTAGE	0x1000
#define KERNEL_VOLTAGE	(NOMINAL_VOLTAGE - 100)

#define TO_VOFFSET_DATA(val)	((0x1000ULL - 2 * (u64)(val)) << 20)
#define TO_VOFFSET_VAL(data)    ((0x1000ULL - ((data) >> 20)) / 2)

#define CORE_VOFFSET_VAL(val)		(0x8000001100000000ULL | TO_VOFFSET_DATA(val))
#define CACHE_VOFFSET_VAL(val)		(0x8000021100000000ULL | TO_VOFFSET_DATA(val))

struct cave_stat {
	atomic_long_t inc;
	atomic_long_t dec;
	atomic_long_t skip;
	atomic_long_t locked;
};

static volatile int cave_enabled = 0;
static DEFINE_SPINLOCK(cave_lock);
DEFINE_PER_CPU(cave_data_t, context);
static cave_data_t kernel_context = { .voltage = KERNEL_VOLTAGE };
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
	u64 voltage;

	wrmsrl(0x150, 0x8000001000000000);
	rdmsrl(0x150, voltage);

	return TO_VOFFSET_VAL(voltage);
#endif
}

static void write_voltage(long new_voltage)
{
	if(!new_voltage)
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
	while(new_voltage > read_voltage())
		cpu_relax();
}

static long select_voltage(long prev_vmin)
{
	long new_vmin = 0;
	int i;

	for_each_possible_cpu(i) {
		cave_data_t tmp = per_cpu(context, i);
		if(tmp.voltage > new_vmin)
			new_vmin = tmp.voltage;
	}

	if (new_vmin == prev_vmin) {
		new_vmin = 0;
		atomic_long_inc(&cave_stat.skip);
	}
	else if (new_vmin > prev_vmin)
		atomic_long_inc(&cave_stat.inc);
	else
		atomic_long_inc(&cave_stat.dec);

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
	new_vmin = select_voltage(prev_vmin);
	write_voltage(new_vmin);

	spin_unlock_irqrestore(&cave_lock, flags);

	if(new_vmin > prev_vmin)
		wait_voltage(new_vmin);
}

__visible void cave_entry_switch(void)
{
	_cave_switch(kernel_context);
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
	long skip = atomic_long_read(&cave_stat.skip);
	long locked = atomic_long_read(&cave_stat.locked);
	long total = inc + dec + skip;

#ifdef CONFIG_UNISERVER_CAVE_TEST_MODE
	long veff = atomic_long_read(&effective_voltage);
#else
	long veff = read_voltage();
#endif

	seq_printf(p, "\n");
	seq_printf(p, "cave: locked %ld %%\n", 100 * locked / total);
	seq_printf(p, "cave: vmin: %ld voff: %3ld\n", veff, NOMINAL_VOLTAGE - veff);
	seq_printf(p, "cave: inc %ld %%, dec %ld %%, skip %ld %%\n",
			100 * inc / total, 100 * dec / total, 100 * skip / total);
}

int cave_init(void)
{
	int i;
	cave_data_t nominal = { .voltage = NOMINAL_VOLTAGE };

	for_each_possible_cpu(i) {
		per_cpu(context, i) = nominal;
		idle_task(i)->cave_data = kernel_context;
	}

#ifdef CONFIG_UNISERVER_CAVE_TEST_MODE
	atomic_long_set(&effective_voltage, NOMINAL_VOLTAGE);
#endif
	cave_enabled = 1;

	printk(KERN_WARNING "cave: enabled\n");

	return 0;
}
late_initcall(cave_init);
