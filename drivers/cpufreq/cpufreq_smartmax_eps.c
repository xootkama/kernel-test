/*
 * drivers/cpufreq/cpufreq_smartmax_eps.c
 *
 * Copyright (C) 2013 maxwen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Author: maxwen
 *
 * Based on the ondemand and smartassV2 governor
 *
 * ondemand:
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *
 * smartassV2:
 * Author: Erasmux
 *
 * For a general overview of CPU governors see the relavent part in
 * Documentation/cpu-freq/governors.txt
 *
 */

#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/kernel_stat.h>

#ifdef CONFIG_CPU_FREQ_GOV_SMARTMAX_EPS_TEGRA
extern int tegra_input_boost (struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation);
#endif

/******************** Tunable parameters: ********************/

/*
 * The "ideal" frequency to use. The governor will ramp up faster
 * towards the ideal frequency and slower after it has passed it. Similarly,
 * lowering the frequency towards the ideal frequency is faster than below it.
 */


#ifdef CONFIG_CPU_FREQ_GOV_SMARTMAX_EPS_ENRC2B
#define DEFAULT_SUSPEND_IDEAL_FREQ 475000
#define DEFAULT_AWAKE_IDEAL_FREQ 475000
#define DEFAULT_RAMP_UP_STEP 300000
#define DEFAULT_RAMP_DOWN_STEP 150000
#define DEFAULT_MAX_CPU_LOAD 80
#define DEFAULT_MIN_CPU_LOAD 50
#define DEFAULT_UP_RATE 30000
#define DEFAULT_DOWN_RATE 60000
#define DEFAULT_SAMPLING_RATE 30000
// default to 3 * sampling_rate
#define DEFAULT_INPUT_BOOST_DURATION 90000
#define DEFAULT_TOUCH_POKE_FREQ 910000
#define DEFAULT_BOOST_FREQ 910000
/*
 * from cpufreq_wheatley.c
 * Not all CPUs want IO time to be accounted as busy; this dependson how
 * efficient idling at a higher frequency/voltage is.
 * Pavel Machek says this is not so for various generations of AMD and old
 * Intel systems.
 * Mike Chan (androidlcom) calis this is also not true for ARM.
 */
#define DEFAULT_IO_IS_BUSY 0
#define DEFAULT_IGNORE_NICE 1
#endif

// msm8974 platform
#ifdef CONFIG_CPU_FREQ_GOV_SMARTMAX_EPS
#define DEFAULT_SUSPEND_IDEAL_FREQ 400000
#define DEFAULT_AWAKE_IDEAL_FREQ 652800
#define DEFAULT_RAMP_UP_STEP 200000
#define DEFAULT_RAMP_DOWN_STEP 200000
#define DEFAULT_MAX_CPU_LOAD 100
#define DEFAULT_MIN_CPU_LOAD 70
#define DEFAULT_UP_RATE 30000
#define DEFAULT_DOWN_RATE 60000
#define DEFAULT_SAMPLING_RATE 30000
#define DEFAULT_INPUT_BOOST_DURATION 110000
#define DEFAULT_TOUCH_POKE_FREQ 1036900
#define DEFAULT_BOOST_FREQ 1497600
#define DEFAULT_IO_IS_BUSY 0
#define DEFAULT_IGNORE_NICE 1
#endif

#ifdef CONFIG_CPU_FREQ_GOV_SMARTMAX_EPS_PRIMOU
#define DEFAULT_SUSPEND_IDEAL_FREQ 368000
#define DEFAULT_AWAKE_IDEAL_FREQ 806000
#define DEFAULT_RAMP_UP_STEP 200000
#define DEFAULT_RAMP_DOWN_STEP 200000
#define DEFAULT_MAX_CPU_LOAD 60
#define DEFAULT_MIN_CPU_LOAD 30
#define DEFAULT_UP_RATE 30000
#define DEFAULT_DOWN_RATE 60000
#define DEFAULT_SAMPLING_RATE 30000
#define DEFAULT_INPUT_BOOST_DURATION 90000
#define DEFAULT_TOUCH_POKE_FREQ 1200000
#define DEFAULT_BOOST_FREQ 1200000
#define DEFAULT_IO_IS_BUSY 0
#define DEFAULT_IGNORE_NICE 1
#endif

#ifdef CONFIG_CPU_FREQ_GOV_SMARTMAX_EPS_M7
#define DEFAULT_SUSPEND_IDEAL_FREQ 384000
#define DEFAULT_AWAKE_IDEAL_FREQ 594000
#define DEFAULT_RAMP_UP_STEP 200000
#define DEFAULT_RAMP_DOWN_STEP 200000
#define DEFAULT_MAX_CPU_LOAD 70
#define DEFAULT_MIN_CPU_LOAD 40
#define DEFAULT_UP_RATE 30000
#define DEFAULT_DOWN_RATE 60000
#define DEFAULT_SAMPLING_RATE 30000
#define DEFAULT_INPUT_BOOST_DURATION 90000
#define DEFAULT_TOUCH_POKE_FREQ 1134000
#define DEFAULT_BOOST_FREQ 1134000
#define DEFAULT_IO_IS_BUSY 0
#define DEFAULT_IGNORE_NICE 1
#endif

#ifdef CONFIG_CPU_FREQ_GOV_SMARTMAX_EPS_FIND5
#define DEFAULT_SUSPEND_IDEAL_FREQ 384000
#define DEFAULT_AWAKE_IDEAL_FREQ 594000
#define DEFAULT_RAMP_UP_STEP 200000
#define DEFAULT_RAMP_DOWN_STEP 200000
#define DEFAULT_MAX_CPU_LOAD 90
#define DEFAULT_MIN_CPU_LOAD 60
#define DEFAULT_UP_RATE 30000
#define DEFAULT_DOWN_RATE 60000
#define DEFAULT_SAMPLING_RATE 30000
#define DEFAULT_INPUT_BOOST_DURATION 900000
#define DEFAULT_TOUCH_POKE_FREQ 1134000
#define DEFAULT_BOOST_FREQ 1134000
#define DEFAULT_IO_IS_BUSY 0
#define DEFAULT_IGNORE_NICE 1
#endif

static unsigned int suspend_ideal_freq;
static unsigned int awake_ideal_freq;
/*
 * Freqeuncy delta when ramping up above the ideal freqeuncy.
 * Zero disables and causes to always jump straight to max frequency.
 * When below the ideal freqeuncy we always ramp up to the ideal freq.
 */
static unsigned int ramp_up_step;

/*
 * Freqeuncy delta when ramping down below the ideal freqeuncy.
 * Zero disables and will calculate ramp down according to load heuristic.
 * When above the ideal freqeuncy we always ramp down to the ideal freq.
 */
static unsigned int ramp_down_step;

/*
 * CPU freq will be increased if measured load > max_cpu_load;
 */
static unsigned int max_cpu_load;

/*
 * CPU freq will be decreased if measured load < min_cpu_load;
 */
static unsigned int min_cpu_load;

/*
 * The minimum amount of time in usecs to spend at a frequency before we can ramp up.
 * Notice we ignore this when we are below the ideal frequency.
 */
static unsigned int up_rate;

/*
 * The minimum amount of time in usecs to spend at a frequency before we can ramp down.
 * Notice we ignore this when we are above the ideal frequency.
 */
static unsigned int down_rate;

/* in usecs */
static unsigned int sampling_rate;

/* in usecs */
static unsigned int input_boost_duration;

static unsigned int touch_poke_freq;
static bool touch_poke = true;

/*
 * should ramp_up steps during boost be possible
 */
static bool ramp_up_during_boost = true;

/*
 * external boost interface - boost if duration is written
 * to sysfs for boost_duration
 */
static unsigned int boost_freq;
static bool boost = true;

/* in usecs */
static unsigned int boost_duration = 0;

/* Consider IO as busy */
static unsigned int io_is_busy;

static unsigned int ignore_nice;

/*************** End of tunables ***************/

static unsigned int dbs_enable; /* number of CPUs using this policy */

static void do_dbs_timer(struct work_struct *work);

struct smartmax_eps_info_s {
	struct cpufreq_policy *cur_policy;
	struct cpufreq_frequency_table *freq_table;
	struct delayed_work work;
	u64 prev_cpu_idle;
	u64 prev_cpu_iowait;
	u64 prev_cpu_wall;
	u64 prev_cpu_nice;
	u64 freq_change_time;
	unsigned int cur_cpu_load;
	unsigned int old_freq;
	int ramp_dir;
	unsigned int ideal_speed;
	unsigned int cpu;
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct smartmax_eps_info_s, smartmax_eps_info);

#define dprintk(flag,msg...) do { \
	if (debug_mask & flag) pr_info("[smartmax_eps]" ":" msg); \
	} while (0)

enum {
	SMARTMAX_EPS_DEBUG_JUMPS = 1,
	SMARTMAX_EPS_DEBUG_LOAD = 2,
	SMARTMAX_EPS_DEBUG_ALG = 4,
	SMARTMAX_EPS_DEBUG_BOOST = 8,
	SMARTMAX_EPS_DEBUG_INPUT = 16,
	SMARTMAX_EPS_DEBUG_SUSPEND = 32
};

/*
 * Combination of the above debug flags.
 */
//static unsigned long debug_mask = SMARTMAX_EPS_DEBUG_LOAD|SMARTMAX_EPS_DEBUG_JUMPS|SMARTMAX_EPS_DEBUG_ALG|SMARTMAX_EPS_DEBUG_BOOST|SMARTMAX_EPS_DEBUG_INPUT|SMARTMAX_EPS_DEBUG_SUSPEND;
static unsigned long debug_mask;

#define SMARTMAX_EPS_STAT 0
#if SMARTMAX_EPS_STAT
static u64 timer_stat[4] = {0, 0, 0, 0};
#endif

/*
 * dbs_mutex protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(dbs_mutex);
static struct workqueue_struct *smartmax_eps_wq;

static bool boost_task_alive = false;
static struct task_struct *boost_task;
static u64 boost_end_time = 0ULL;
static unsigned int cur_boost_freq = 0;
static unsigned int cur_boost_duration = 0;
static bool boost_running = false;
static unsigned int ideal_freq;
static bool is_suspended = false;
static unsigned int min_sampling_rate;

#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate sampling
 * rate.
 * For CPUs with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work.
 * All times here are in uS.
 */
#define MIN_SAMPLING_RATE_RATIO			(2)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)

static int cpufreq_governor_smartmax_eps(struct cpufreq_policy *policy,
		unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SMARTMAX_EPS
static
#endif
struct cpufreq_governor cpufreq_gov_smartmax_eps = { 
    .name = "smartmax_eps", 
    .governor = cpufreq_smartmax_eps, 
    .max_transition_latency = TRANSITION_LATENCY_LIMIT, 
    .owner = THIS_MODULE,
    };

static inline u64 get_cpu_iowait_time(unsigned int cpu, u64 *wall) {
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

inline static void smartmax_eps_update_min_max(
		struct smartmax_eps_info_s *this_smartmax_eps, struct cpufreq_policy *policy) {
	this_smartmax_eps->ideal_speed = // ideal_freq; but make sure it obeys the policy min/max
			policy->min < ideal_freq ?
					(ideal_freq < policy->max ? ideal_freq : policy->max) :
					policy->min;

}

inline static void smartmax_eps_update_min_max_allcpus(void) {
	unsigned int cpu;

	for_each_online_cpu(cpu)
	{
		struct smartmax_eps_info_s *this_smartmax_eps = &per_cpu(smartmax_eps_info, cpu);
		if (this_smartmax_eps->cur_policy){
//			if (lock_policy_rwsem_write(cpu) < 0)
//				continue;

			smartmax_eps_update_min_max(this_smartmax_eps, this_smartmax_eps->cur_policy);
			
//			unlock_policy_rwsem_write(cpu);
		}
	}
}

inline static unsigned int validate_freq(struct cpufreq_policy *policy,
		int freq) {
	if (freq > (int) policy->max)
		return policy->max;
	if (freq < (int) policy->min)
		return policy->min;
	return freq;
}
/* We use the same work function to sale up and down */
static void cpufreq_smartmax_eps_freq_change(struct smartmax_eps_info_s *this_smartmax_eps) {
	unsigned int cpu;
	unsigned int new_freq = 0;
	unsigned int old_freq;
	int ramp_dir;
	struct cpufreq_policy *policy;
	unsigned int relation = CPUFREQ_RELATION_L;

	ramp_dir = this_smartmax_eps->ramp_dir;
	old_freq = this_smartmax_eps->old_freq;
	policy = this_smartmax_eps->cur_policy;
	cpu = this_smartmax_eps->cpu;

	dprintk(SMARTMAX_EPS_DEBUG_ALG, "%d: %s\n", old_freq, __func__);
	
	if (old_freq != policy->cur) {
		// frequency was changed by someone else?
		dprintk(SMARTMAX_EPS_DEBUG_ALG, "%d: frequency changed by 3rd party to %d\n",
				old_freq, policy->cur);
		new_freq = old_freq;
	} else if (ramp_dir > 0 && nr_running() > 1) {
		// ramp up logic:
		if (old_freq < this_smartmax_eps->ideal_speed)
			new_freq = this_smartmax_eps->ideal_speed;
		else if (ramp_up_step) {
			new_freq = old_freq + ramp_up_step;
			relation = CPUFREQ_RELATION_H;
		} else {
			new_freq = policy->max;
			relation = CPUFREQ_RELATION_H;
		}
	} else if (ramp_dir < 0) {
		// ramp down logic:
		if (old_freq > this_smartmax_eps->ideal_speed) {
			new_freq = this_smartmax_eps->ideal_speed;
			relation = CPUFREQ_RELATION_H;
		} else if (ramp_down_step)
			new_freq = old_freq - ramp_down_step;
		else {
			// Load heuristics: Adjust new_freq such that, assuming a linear
			// scaling of load vs. frequency, the load in the new frequency
			// will be max_cpu_load:
			new_freq = old_freq * this_smartmax_eps->cur_cpu_load / max_cpu_load;
			if (new_freq > old_freq) // min_cpu_load > max_cpu_load ?!
				new_freq = old_freq - 1;
		}
	}

	if (new_freq!=0){
		target_freq(policy, this_smartmax_eps, new_freq, old_freq, relation);
	}
	
	this_smartmax_eps->ramp_dir = 0;
}

static inline void cpufreq_smartmax_eps_get_ramp_direction(struct smartmax_eps_info_s *this_smartmax_eps, u64 now)
{
	unsigned int cur_load = this_smartmax_eps->cur_cpu_load;
	unsigned int cur = this_smartmax_eps->old_freq;
	struct cpufreq_policy *policy = this_smartmax_eps->cur_policy;
	
	// Scale up if load is above max or if there where no idle cycles since coming out of idle,
	// additionally, if we are at or above the ideal_speed, verify we have been at this frequency
	// for at least up_rate:
	if (cur_load > max_cpu_load && cur < policy->max
			&& (cur < this_smartmax_eps->ideal_speed
				|| (now - this_smartmax_eps->freq_change_time) >= up_rate)) {
		dprintk(SMARTMAX_EPS_DEBUG_ALG,
				"%d: ramp up: load %d\n", cur, cur_load);
		this_smartmax_eps->ramp_dir = 1;
	}
	// Similarly for scale down: load should be below min and if we are at or below ideal
	// frequency we require that we have been at this frequency for at least down_rate:
	else if (cur_load < min_cpu_load && cur > policy->min
			&& (cur > this_smartmax_eps->ideal_speed
				|| (now - this_smartmax_eps->freq_change_time) >= down_rate)) {
		dprintk(SMARTMAX_EPS_DEBUG_ALG,
				"%d: ramp down: load %d\n", cur, cur_load);
		this_smartmax_eps->ramp_dir = -1;
	}
}

static void inline cpufreq_smartmax_eps_calc_load(int j)
{
	struct smartmax_eps_info_s *j_this_smartmax_eps;
	u64 cur_wall_time, cur_idle_time, cur_iowait_time;
	unsigned int idle_time, wall_time, iowait_time;
	unsigned int cur_load;
		
	j_this_smartmax_eps = &per_cpu(smartmax_eps_info, j);

	cur_idle_time = get_cpu_idle_time(j, &cur_wall_time, 0);
	cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

	wall_time = cur_wall_time - j_this_smartmax_eps->prev_cpu_wall;
	j_this_smartmax_eps->prev_cpu_wall = cur_wall_time;

	idle_time = cur_idle_time - j_this_smartmax_eps->prev_cpu_idle;
	j_this_smartmax_eps->prev_cpu_idle = cur_idle_time;

	iowait_time = cur_iowait_time - j_this_smartmax_eps->prev_cpu_iowait;
	j_this_smartmax_eps->prev_cpu_iowait = cur_iowait_time;

	if (ignore_nice) {
		u64 cur_nice;
		unsigned long cur_nice_jiffies;

#ifdef CONFIG_CPU_FREQ_GOV_SMARTMAX_EPS_30
		cur_nice = kstat_cpu(j).cpustat.nice - j_this_smartmax_eps->prev_cpu_nice;
		cur_nice_jiffies = (unsigned long) cputime64_to_jiffies64(cur_nice);

		j_this_smartmax_eps->prev_cpu_nice = kstat_cpu(j).cpustat.nice;
#else
		cur_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE] - j_this_smartmax_eps->prev_cpu_nice;
		cur_nice_jiffies = (unsigned long) cputime64_to_jiffies64(cur_nice);

		j_this_smartmax_eps->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];

#endif

		idle_time += jiffies_to_usecs(cur_nice_jiffies);
	}

	/*
	 * For the purpose of ondemand, waiting for disk IO is an
	 * indication that you're performance critical, and not that
	 * the system is actually idle. So subtract the iowait time
	 * from the cpu idle time.
	 */
	if (io_is_busy && idle_time >= iowait_time)
		idle_time -= iowait_time;

	if (unlikely(!wall_time || wall_time < idle_time))
		return;

	cur_load = 100 * (wall_time - idle_time) / wall_time;
	j_this_smartmax_eps->cur_cpu_load = cur_load;
}

static void cpufreq_smartmax_eps_timer(struct smartmax_eps_info_s *this_smartmax_eps) {
	unsigned int cur;
	struct cpufreq_policy *policy = this_smartmax_eps->cur_policy;
	u64 now = ktime_to_us(ktime_get());
	/* Extrapolated load of this CPU */
	//unsigned int load_at_max_freq = 0;
	unsigned int cpu = this_smartmax_eps->cpu;

#if SMARTMAX_EPS_STAT 
	u64 diff = 0;

	if (timer_stat[cpu])
		diff = now - timer_stat[cpu];

	timer_stat[cpu] = now;
	printk(KERN_DEBUG "[smartmax_eps]:cpu %d %lld\n", cpu, diff);
#endif

	cur = policy->cur;

	dprintk(SMARTMAX_EPS_DEBUG_ALG, "%d: %s cpu %d %lld\n", cur, __func__, cpu, now);

	cpufreq_smartmax_eps_calc_load(cpu);

	/* calculate the scaled load across CPU */
	//load_at_max_freq = (this_smartmax_eps->cur_cpu_load * policy->cur)/policy->cpuinfo.max_freq;

	//cpufreq_notify_utilization(policy, load_at_max_freq);

	dprintk(SMARTMAX_EPS_DEBUG_LOAD, "%d: load %d\n", cpu, this_smartmax_eps->cur_cpu_load);

	this_smartmax_eps->old_freq = cur;
	this_smartmax_eps->ramp_dir = 0;

	cpufreq_smartmax_eps_get_ramp_direction(this_smartmax_eps, now);

	// no changes
	if (this_smartmax_eps->ramp_dir == 0)		
		return;

	// boost - but not block ramp up steps based on load if requested
	if (boost_running){
		if (now < boost_end_time) {
			dprintk(SMARTMAX_EPS_DEBUG_BOOST, "%d: cpu %d boost running %llu %llu\n", cur, cpu, now, boost_end_time);
		
			if (this_smartmax_eps->ramp_dir == -1)
				return;
			else {
				if (ramp_up_during_boost)
					dprintk(SMARTMAX_EPS_DEBUG_BOOST, "%d: cpu %d boost running but ramp_up above boost freq requested\n", cur, cpu);
				else
					return;
			}
		} else
			boost_running = false;
	}

	cpufreq_smartmax_eps_freq_change(this_smartmax_eps);
}

static void do_dbs_timer(struct work_struct *work) {
	struct smartmax_eps_info_s *this_smartmax_eps =
			container_of(work, struct smartmax_eps_info_s, work.work);
	unsigned int cpu = this_smartmax_eps->cpu;
	int delay = get_timer_delay();

	mutex_lock(&this_smartmax_eps->timer_mutex);

	cpufreq_smartmax_eps_timer(this_smartmax_eps);

	queue_delayed_work_on(cpu, smartmax_eps_wq, &this_smartmax_eps->work, delay);
	mutex_unlock(&this_smartmax_eps->timer_mutex);
}

static void update_idle_time(bool online) {
	int j = 0;

	for_each_possible_cpu(j)
	{
		struct smartmax_eps_info_s *j_this_smartmax_eps;

		if (online && !cpu_online(j)) {
			continue;
		}
		j_this_smartmax_eps = &per_cpu(smartmax_eps_info, j);

		j_this_smartmax_eps->prev_cpu_idle = get_cpu_idle_time(j,
				&j_this_smartmax_eps->prev_cpu_wall, 0);
				
		if (ignore_nice)
#ifdef CONFIG_CPU_FREQ_GOV_SMARTMAX_EPS_30
			j_this_smartmax_eps->prev_cpu_nice = kstat_cpu(j) .cpustat.nice;
#else
			j_this_smartmax_eps->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
#endif
	}
}

static ssize_t show_debug_mask(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%lu\n", debug_mask);
}

static ssize_t store_debug_mask(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0)
		debug_mask = input;
	else
		return -EINVAL;
	return count;
}

static ssize_t show_up_rate(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%u\n", up_rate);
}

static ssize_t store_up_rate(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input <= 100000000)
		up_rate = input;
	else
		return -EINVAL;
	return count;
}

static ssize_t show_down_rate(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%u\n", down_rate);
}

static ssize_t store_down_rate(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input <= 100000000)
		down_rate = input;
	else
		return -EINVAL;
	return count;
}

static ssize_t show_awake_ideal_freq(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%u\n", awake_ideal_freq);
}

static ssize_t store_awake_ideal_freq(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
		awake_ideal_freq = input;
		if (!is_suspended){
			ideal_freq = awake_ideal_freq;
			smartmax_eps_update_min_max_allcpus();
		}
	} else
		return -EINVAL;
	return count;
}

static ssize_t show_suspend_ideal_freq(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%u\n", suspend_ideal_freq);
}

static ssize_t store_suspend_ideal_freq(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
		suspend_ideal_freq = input;
		if (is_suspended){
			ideal_freq = suspend_ideal_freq;
			smartmax_eps_update_min_max_allcpus();
		}
	} else
		return -EINVAL;
	return count;
}

static ssize_t show_ramp_up_step(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%u\n", ramp_up_step);
}

static ssize_t store_ramp_up_step(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0 && input >= 0)
		ramp_up_step = input;
	else
		return -EINVAL;
	return count;
}

static ssize_t show_ramp_down_step(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%u\n", ramp_down_step);
}

static ssize_t store_ramp_down_step(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0 && input >= 0)
		ramp_down_step = input;
	else
		return -EINVAL;
	return count;
}

static ssize_t show_max_cpu_load(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%u\n", max_cpu_load);
}

static ssize_t store_max_cpu_load(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0 && input > 0 && input <= 100)
		max_cpu_load = input;
	else
		return -EINVAL;
	return count;
}

static ssize_t show_min_cpu_load(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%u\n", min_cpu_load);
}

static ssize_t store_min_cpu_load(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0 && input > 0 && input < 100)
		min_cpu_load = input;
	else
		return -EINVAL;
	return count;
}

static ssize_t show_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%u\n", sampling_rate);
}

static ssize_t store_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0 && input >= min_sampling_rate)
		sampling_rate = input;
	else
		return -EINVAL;
	return count;
}

static ssize_t show_touch_poke_freq(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf) {
	return sprintf(buf, "%u\n", touch_poke_freq);
}

static ssize_t store_touch_poke_freq(struct kobject *a, struct kobj_attribute *b,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0){
		touch_poke_freq = input;
	
		if (touch_poke_freq == 0)
			touch_poke = false;
		else
			touch_poke = true;
	} else
		return -EINVAL;	
	return count;
}

static ssize_t show_input_boost_duration(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf) {
	return sprintf(buf, "%u\n", input_boost_duration);
}

static ssize_t store_input_boost_duration(struct kobject *a,
		struct kobj_attribute *b, const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0 && input > 10000)
		input_boost_duration = input;
	else
		return -EINVAL;
	return count;
}

static ssize_t show_ramp_up_during_boost(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf) {
	return sprintf(buf, "%d\n", ramp_up_during_boost);
}

static ssize_t store_ramp_up_during_boost(struct kobject *a, struct kobj_attribute *b,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0) {
		if (input == 0)
			ramp_up_during_boost = false;
		else if (input == 1)
			ramp_up_during_boost = true;
		else
			return -EINVAL;
	} else
		return -EINVAL;
	return count;
}

static ssize_t show_boost_freq(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%u\n", boost_freq);
}

static ssize_t store_boost_freq(struct kobject *a, struct kobj_attribute *b,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0) {
		boost_freq = input;
		if (boost_freq == 0)
			boost = false;
		else
			boost = true;
	} else
		return -EINVAL;	
	return count;
}

static ssize_t show_boost_duration(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%d\n", boost_running);
}

static ssize_t store_boost_duration(struct kobject *a, struct kobj_attribute *b,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;
	res = kstrtoul(buf, 0, &input);
	if (res >= 0 && input > 10000){
		boost_duration = input;
		if (boost) {
			// no need to bother if currently a boost is running anyway
			if (boost_task_alive && boost_running)
				return count;

			if (boost_task_alive) {
				cur_boost_freq = boost_freq;
				cur_boost_duration = boost_duration;
				wake_up_process(boost_task);
			}
		}
	} else
		return -EINVAL;
	return count;
}

static ssize_t show_io_is_busy(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%d\n", io_is_busy);
}

static ssize_t store_io_is_busy(struct kobject *a, struct kobj_attribute *b,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;

	res = kstrtoul(buf, 0, &input);
	if (res >= 0) {
		if (input > 1)
			input = 1;
		if (input == io_is_busy) { /* nothing to do */
			return count;
		}
		io_is_busy = input;
	} else
		return -EINVAL;	
	return count;
}

static ssize_t show_ignore_nice(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%d\n", ignore_nice);
}

static ssize_t store_ignore_nice(struct kobject *a, struct kobj_attribute *b,
		const char *buf, size_t count) {
	ssize_t res;
	unsigned long input;

	res = kstrtoul(buf, 0, &input);
	if (res >= 0) {
		if (input > 1)
			input = 1;
		if (input == ignore_nice) { /* nothing to do */
			return count;
		}
		ignore_nice = input;
		/* we need to re-evaluate prev_cpu_idle */
		update_idle_time(true);
	} else
		return -EINVAL;	
	return count;
}

static ssize_t show_min_sampling_rate(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return sprintf(buf, "%d\n", min_sampling_rate);
}

static ssize_t store_min_sampling_rate(struct kobject *a, struct kobj_attribute *b,
		const char *buf, size_t count) {
	return -EINVAL;	
}

#define define_global_rw_attr(_name)		\
static struct kobj_attribute _name##_attr =	\
	__ATTR(_name, 0644, show_##_name, store_##_name)

#define define_global_ro_attr(_name)		\
static struct kobj_attribute _name##_attr =	\
	__ATTR(_name, 0444, show_##_name, store_##_name)

define_global_rw_attr(debug_mask);
define_global_rw_attr(up_rate);
define_global_rw_attr(down_rate);
define_global_rw_attr(ramp_up_step);
define_global_rw_attr(ramp_down_step);
define_global_rw_attr(max_cpu_load);
define_global_rw_attr(min_cpu_load);
define_global_rw_attr(sampling_rate);
define_global_rw_attr(touch_poke_freq);
define_global_rw_attr(input_boost_duration);
define_global_rw_attr(boost_freq);
define_global_rw_attr(boost_duration);
define_global_rw_attr(io_is_busy);
define_global_rw_attr(ignore_nice);
define_global_rw_attr(ramp_up_during_boost);
define_global_rw_attr(awake_ideal_freq);
define_global_rw_attr(suspend_ideal_freq);
define_global_ro_attr(min_sampling_rate);

static struct attribute * smartmax_eps_attributes[] = { 
	&debug_mask_attr.attr,
	&up_rate_attr.attr, 
	&down_rate_attr.attr, 
	&ramp_up_step_attr.attr, 
	&ramp_down_step_attr.attr,
	&max_cpu_load_attr.attr, 
	&min_cpu_load_attr.attr,
	&sampling_rate_attr.attr, 
	&touch_poke_freq_attr.attr,
	&input_boost_duration_attr.attr, 
	&boost_freq_attr.attr, 
	&boost_duration_attr.attr, 
	&io_is_busy_attr.attr,
	&ignore_nice_attr.attr, 
	&ramp_up_during_boost_attr.attr, 
	&awake_ideal_freq_attr.attr,
	&suspend_ideal_freq_attr.attr,		
	&min_sampling_rate_attr.attr,
	NULL , };

static struct attribute_group smartmax_eps_attr_group = { 
	.attrs = smartmax_eps_attributes, 
	.name = "smartmax_eps", 
	};

static int cpufreq_smartmax_eps_boost_task(void *data) {
	struct smartmax_eps_info_s *this_smartmax_eps;
	u64 now;
	struct cpufreq_policy *policy;
#ifndef CONFIG_CPU_FREQ_GOV_SMARTMAX_EPS_TEGRA
	unsigned int cpu;
	bool start_boost = false;
#endif
	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		if (kthread_should_stop())
			break;

		set_current_state(TASK_RUNNING);

		if (boost_running)
			continue;

#ifdef CONFIG_CPU_FREQ_GOV_SMARTMAX_EPS_TEGRA
		/* on tegra there is only one cpu clock so we only need to boost cpu 0 
		   all others will run at the same speed */
		this_smartmax_eps = &per_cpu(smartmax_eps_info, 0);
		if (!this_smartmax_eps)
			continue;

		policy = this_smartmax_eps->cur_policy;
		if (!policy)
			continue;

//		if (lock_policy_rwsem_write(0) < 0)
//			continue;
		
		tegra_input_boost(policy, cur_boost_freq, CPUFREQ_RELATION_H);
	
        this_smartmax_eps->prev_cpu_idle = get_cpu_idle_time(0,
						&this_smartmax_eps->prev_cpu_wall, 0);

//	unlock_policy_rwsem_write(0);
#else		
		for_each_online_cpu(cpu){
			this_smartmax_eps = &per_cpu(smartmax_eps_info, cpu);
			if (!this_smartmax_eps)
				continue;

//			if (lock_policy_rwsem_write(cpu) < 0)
//				continue;

			policy = this_smartmax_eps->cur_policy;
			if (!policy){
//				unlock_policy_rwsem_write(cpu);
				continue;
			}

			mutex_lock(&this_smartmax_eps->timer_mutex);

			if (policy->cur < cur_boost_freq) {
				start_boost = true;
				dprintk(SMARTMAX_EPS_DEBUG_BOOST, "input boost cpu %d to %d\n", cpu, cur_boost_freq);
				target_freq(policy, this_smartmax_eps, cur_boost_freq, this_smartmax_eps->old_freq, CPUFREQ_RELATION_H);
				this_smartmax_eps->prev_cpu_idle = get_cpu_idle_time(cpu, &this_smartmax_eps->prev_cpu_wall, 0);
			}
			mutex_unlock(&this_smartmax_eps->timer_mutex);
			
/*
 * In no_hz/micro accounting case we set the minimum frequency
 * not depending on HZ, but fixed (very low). The deferred
 * timer might skip some samples if idle/sleeping as needed.
*/
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		/* For correct statistics, we need 10 ticks for each measure */
		min_sampling_rate = MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(10);
	}

	smartmax_eps_wq = alloc_workqueue("smartmax_eps_wq", WQ_HIGHPRI, 0);
	if (!smartmax_eps_wq) {
		printk(KERN_ERR "Failed to create smartmax_eps_wq workqueue\n");
		return -EFAULT;
	}

	up_rate = DEFAULT_UP_RATE;
	down_rate = DEFAULT_DOWN_RATE;
	suspend_ideal_freq = DEFAULT_SUSPEND_IDEAL_FREQ;
	awake_ideal_freq = DEFAULT_AWAKE_IDEAL_FREQ;
	ideal_freq = awake_ideal_freq;
	ramp_up_step = DEFAULT_RAMP_UP_STEP;
	ramp_down_step = DEFAULT_RAMP_DOWN_STEP;
	max_cpu_load = DEFAULT_MAX_CPU_LOAD;
	min_cpu_load = DEFAULT_MIN_CPU_LOAD;
	sampling_rate = DEFAULT_SAMPLING_RATE;
	input_boost_duration = DEFAULT_INPUT_BOOST_DURATION;
	io_is_busy = DEFAULT_IO_IS_BUSY;
	ignore_nice = DEFAULT_IGNORE_NICE;
	touch_poke_freq = DEFAULT_TOUCH_POKE_FREQ;
	boost_freq = DEFAULT_BOOST_FREQ;

	/* Initalize per-cpu data: */
	for_each_possible_cpu(i)
	{
		this_smartmax_eps = &per_cpu(smartmax_eps_info, i);
		this_smartmax_eps->cur_policy = NULL;
		this_smartmax_eps->ramp_dir = 0;
		this_smartmax_eps->freq_change_time = 0;
		this_smartmax_eps->cur_cpu_load = 0;
		mutex_init(&this_smartmax_eps->timer_mutex);
	}

	return cpufreq_register_governor(&cpufreq_gov_smartmax_eps);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SMARTMAX_EPS
fs_initcall(cpufreq_smartmax_eps_init);
#else
module_init(cpufreq_smartmax_eps_init);
#endif

static void __exit cpufreq_smartmax_eps_exit(void) {
	unsigned int i;
	struct smartmax_eps_info_s *this_smartmax_eps;

	cpufreq_unregister_governor(&cpufreq_gov_smartmax_eps);

	for_each_possible_cpu(i)
	{
		this_smartmax_eps = &per_cpu(smartmax_eps_info, i);
		mutex_destroy(&this_smartmax_eps->timer_mutex);
	}
	destroy_workqueue(smartmax_eps_wq);
}

module_exit(cpufreq_smartmax_eps_exit);

MODULE_AUTHOR("maxwen");
MODULE_DESCRIPTION("'cpufreq_smartmax_eps' - A smart cpufreq governor");
MODULE_LICENSE("GPL");

