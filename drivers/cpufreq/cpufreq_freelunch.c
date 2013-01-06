/*
 *  drivers/cpufreq/cpufreq_freelunch.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (C)  2009 Alexander Clouter <alex@digriz.org.uk>
 *            (C)  2012 Ryan Pennucci <decimalman@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/slab.h>

// {{{1 tuner crap
static void do_dbs_timer(struct work_struct *work);

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	unsigned int requested_freq;
	int cpu;
	unsigned int enable:1;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;

	int hotplug_cycle;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, cs_cpu_dbs_info);

static unsigned int dbs_enable;	/* number of CPUs using this policy */

/* Hotplug stuff */
static struct workqueue_struct *hotplug_wq;
static struct work_struct cpu_up_work;
static struct work_struct cpu_down_work;

/*
 * dbs_mutex protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(dbs_mutex);

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int ignore_nice;

	unsigned int hotplug_up_cycles;
	unsigned int hotplug_down_cycles;
	unsigned int hotplug_up_load;
	unsigned int hotplug_up_usage;
	unsigned int hotplug_down_usage;
	unsigned int overestimate_khz;
} dbs_tuners_ins = {
#if 0
	/* Crazy-aggressive */
	.sampling_rate = 20000,
	.ignore_nice = 0,
	.hotplug_up_cycles = 2,
	.hotplug_down_cycles = 1,
	.hotplug_up_load = 2,
	.hotplug_up_usage = 40,
	.hotplug_down_usage = 10,
	.overestimate_khz = 125000,
#elif 1
	/* Crazy-conservative */
	.sampling_rate = 20000,
	.ignore_nice = 0,
	.hotplug_up_cycles = 3,
	.hotplug_down_cycles = 1,
	.hotplug_up_load = 3,
	.hotplug_up_usage = 50,
	.hotplug_down_usage = 20,
	.overestimate_khz = 25000,
#else
	/* Original settings */
	.sampling_rate = 20000,
	.ignore_nice = 0,
	.hotplug_up_cycles = 2,
	.hotplug_down_cycles = 2,
	.hotplug_up_load = 3,
	.hotplug_up_usage = 60,
	.hotplug_down_usage = 25,
	.overestimate_khz = 75000,
#endif
};
// }}}
// {{{2 support function crap
static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
							cputime64_t *wall)
{
	cputime64_t idle_time;
	cputime64_t cur_wall_time;
	cputime64_t busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());
	busy_time = cputime64_add(kstat_cpu(cpu).cpustat.user,
			kstat_cpu(cpu).cpustat.system);

	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.irq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.softirq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.steal);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.nice);

	idle_time = cputime64_sub(cur_wall_time, busy_time);
	if (wall)
		*wall = (cputime64_t)jiffies_to_usecs(cur_wall_time);

	return (cputime64_t)jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, wall);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);

	return idle_time;
}

/* keep track of frequency transitions */
static int
dbs_cpufreq_notifier(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpu_dbs_info_s *this_dbs_info = &per_cpu(cs_cpu_dbs_info,
							freq->cpu);

	struct cpufreq_policy *policy;

	if (!this_dbs_info->enable)
		return 0;

	policy = this_dbs_info->cur_policy;

	/*
	 * we only care if our internally tracked freq moves outside
	 * the 'valid' ranges of freqency available to us otherwise
	 * we do not change it
	*/
	if (this_dbs_info->requested_freq > policy->max
			|| this_dbs_info->requested_freq < policy->min)
		this_dbs_info->requested_freq = freq->new;

	return 0;
}

static struct notifier_block dbs_cpufreq_notifier_block = {
	.notifier_call = dbs_cpufreq_notifier
};
// }}}
// {{{2 sysfs crap
/************************** sysfs interface ************************/
static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", 10000);
}

define_one_global_ro(sampling_rate_min);

/* cpufreq_freelunch Governor Tunables */
#define show_one(file_name, object)							\
static ssize_t show_##file_name								\
(struct kobject *kobj, struct attribute *attr, char *buf)	\
{															\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
#define i_am_lazy(f, min, max)						\
show_one(f,f)										\
static ssize_t store_##f							\
(struct kobject *a, struct attribute *b,			\
				   const char *buf, size_t count)	\
{													\
	unsigned int input; int ret;					\
	ret = sscanf(buf, "%u", &input);				\
	if (ret != 1 || input < min || input > max) return -EINVAL;	\
	dbs_tuners_ins.f = input;						\
	return count;									\
}													\
define_one_global_rw(f);

show_one(sampling_rate, sampling_rate);
show_one(ignore_nice_load, ignore_nice);
i_am_lazy(hotplug_up_cycles, 0, 10)
i_am_lazy(hotplug_down_cycles, 0, 10)
i_am_lazy(hotplug_up_load, 0, 10)
i_am_lazy(hotplug_up_usage, 0, 100)
i_am_lazy(hotplug_down_usage, 0, 100)
i_am_lazy(overestimate_khz, 0, 1000000)

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.sampling_rate = max(input, (unsigned int)10000);
	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_tuners_ins.ignore_nice) /* nothing to do */
		return count;

	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(cs_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kstat_cpu(j).cpustat.nice;
	}
	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(ignore_nice_load);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&ignore_nice_load.attr,
	&hotplug_up_cycles.attr,
	&hotplug_down_cycles.attr,
	&hotplug_up_load.attr,
	&hotplug_up_usage.attr,
	&hotplug_down_usage.attr,
	&overestimate_khz.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "freelunch",
};

/************************** sysfs end ************************/
// }}}
// {{{1 useful crap
static void do_cpu_up(struct work_struct *work) {
	if (num_online_cpus() == 1) cpu_up(1);
}
static void do_cpu_down(struct work_struct *work) { 
	if (num_online_cpus() > 1) cpu_down(1);
}

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int load;
	cputime64_t cur_wall_time, cur_idle_time;
	unsigned int idle_time, wall_time;

	struct cpufreq_policy *policy;

	/* Calculate load for this processor only.  The assumption is that we're
	 * running on an aSMP processor where each core has its own instance.
	 *
	 * XXX To use this on non-Krait processors, it would be wise to wrap this
	 * in the usual for_each_cpu loop and rethink the load estimation.
	 */
	policy = this_dbs_info->cur_policy;

	cur_idle_time = get_cpu_idle_time(policy->cpu, &cur_wall_time);

	wall_time = (unsigned int) cputime64_sub(cur_wall_time,
		this_dbs_info->prev_cpu_wall);
	this_dbs_info->prev_cpu_wall = cur_wall_time;
	
	idle_time = (unsigned int) cputime64_sub(cur_idle_time,
		this_dbs_info->prev_cpu_idle);
	this_dbs_info->prev_cpu_idle = cur_idle_time;

	if (dbs_tuners_ins.ignore_nice) {
		cputime64_t cur_nice;
		unsigned long cur_nice_jiffies;

		cur_nice = cputime64_sub(kstat_cpu(policy->cpu).cpustat.nice,
			this_dbs_info->prev_cpu_nice);
		cur_nice_jiffies = (unsigned long)
			cputime64_to_jiffies64(cur_nice);

		this_dbs_info->prev_cpu_nice = kstat_cpu(policy->cpu).cpustat.nice;
		idle_time += jiffies_to_usecs(cur_nice_jiffies);
	}

	/* Apparently, this happens. */
	if (idle_time > wall_time) return;

	/* Use an accurate load estimate for hotplug, and overestimate for freq */
	load = policy->cur / wall_time * (wall_time - idle_time);
	this_dbs_info->requested_freq = (policy->cur + dbs_tuners_ins.overestimate_khz) /
		wall_time * (wall_time - idle_time);

	/* Hotplug? */
	if (num_online_cpus() == 1) {
		if (nr_running() >= dbs_tuners_ins.hotplug_up_load) {
			if (this_dbs_info->hotplug_cycle++ >= dbs_tuners_ins.hotplug_up_cycles &&
				load > policy->max * dbs_tuners_ins.hotplug_up_usage / 100) {
				queue_work_on(0, hotplug_wq, &cpu_up_work);
				this_dbs_info->hotplug_cycle = 0;
			}
		} else this_dbs_info->hotplug_cycle = 0;
	} else {
		if (load < policy->max * dbs_tuners_ins.hotplug_down_usage / 100) {
			if (this_dbs_info->hotplug_cycle++ >= dbs_tuners_ins.hotplug_down_cycles) {
				queue_work_on(0, hotplug_wq, &cpu_down_work);
				this_dbs_info->hotplug_cycle = 0;
			}
		} else this_dbs_info->hotplug_cycle = 0;
	}

	/* Set! */
	__cpufreq_driver_target(policy, this_dbs_info->requested_freq,
		CPUFREQ_RELATION_L);
}
// }}}
// {{{2 cpufreq crap
static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;

	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	delay -= jiffies % delay;

	mutex_lock(&dbs_info->timer_mutex);

	dbs_check_cpu(dbs_info);

	schedule_delayed_work_on(cpu, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	delay -= jiffies % delay;

	dbs_info->enable = 1;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	schedule_delayed_work_on(dbs_info->cpu, &dbs_info->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	dbs_info->enable = 0;
	cancel_delayed_work_sync(&dbs_info->work);
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(cs_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice) {
				j_dbs_info->prev_cpu_nice =
						kstat_cpu(j).cpustat.nice;
			}
		}
		this_dbs_info->requested_freq = policy->cur;
		this_dbs_info->hotplug_cycle = 0;

		mutex_init(&this_dbs_info->timer_mutex);
		dbs_enable++;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			cpufreq_register_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
		}
		mutex_unlock(&dbs_mutex);

		dbs_timer_init(this_dbs_info);

		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		dbs_enable--;
		mutex_destroy(&this_dbs_info->timer_mutex);

		/*
		 * Stop the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 0)
			cpufreq_unregister_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);

		mutex_unlock(&dbs_mutex);
		if (!dbs_enable)
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&this_dbs_info->timer_mutex);

		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_FREELUNCH
static
#endif
struct cpufreq_governor cpufreq_gov_freelunch = {
	.name			= "freelunch",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= 10000000,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	hotplug_wq = create_workqueue("flhotplug");
	if (!hotplug_wq) {
		printk(KERN_ERR "Couldn't create freelunch hotplug wq\n");
		return -EFAULT;
	}
	INIT_WORK(&cpu_up_work, do_cpu_up);
	INIT_WORK(&cpu_down_work, do_cpu_down);
	return cpufreq_register_governor(&cpufreq_gov_freelunch);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_freelunch);
}

MODULE_AUTHOR("Ryan Pennucci <decimalman@gmail.com>");
MODULE_DESCRIPTION("'cpufreq_freelunch' -- an incredibly simple hotplugging governor.");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_FREELUNCH
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
// }}}
// vim:ts=4:sw=4:fdm=marker:fdl=1
