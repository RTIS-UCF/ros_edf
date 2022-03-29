// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * rt-sched.h - sched_setattr() and sched_getattr() API
 * (C) Dario Faggioli <raistlin@linux.it>, 2009, 2010
 * Copyright (C) 2014 BMW Car IT GmbH, Daniel Wagner <daniel.wagner@bmw-carit.de
 */

/* This file is based on Dario Faggioli's libdl. Eventually it will be
   replaced by a proper implemenation of this API. */

#ifndef __RT_SCHED_H__
#define __RT_SCHED_H__

#include <stdint.h>
#include <sys/types.h>
#include <mutex>
#include <vector>
#include <memory>

#ifndef SCHED_DEADLINE
#define SCHED_DEADLINE 6
#endif

#ifdef __x86_64__
#define __NR_sched_setattr		314
#define __NR_sched_getattr		315
#endif

#ifdef __i386__
#define __NR_sched_setattr		351
#define __NR_sched_getattr		352
#endif

#ifdef __arm__
#ifndef __NR_sched_setattr
#define __NR_sched_setattr		380
#endif
#ifndef __NR_sched_getattr
#define __NR_sched_getattr		381
#endif
#endif

#ifdef __tilegx__
#define __NR_sched_setattr		274
#define __NR_sched_getattr		275
#endif

typedef unsigned long long u64;
#define NS_TO_MS 1000000
struct sched_attr {
	uint32_t size;
	uint32_t sched_policy;
	uint64_t sched_flags;

	/* SCHED_NORMAL, SCHED_BATCH */
	int32_t sched_nice;

	/* SCHED_FIFO, SCHED_RR */
	uint32_t sched_priority;

	/* SCHED_DEADLINE */
	uint64_t sched_runtime;
	uint64_t sched_deadline;
	uint64_t sched_period;
};

int sched_setattr(pid_t pid,
		  const struct sched_attr *attr,
		  unsigned int flags);

int sched_getattr(pid_t pid,
		  struct sched_attr *attr,
		  unsigned int size,
		  unsigned int flags);

u64 get_time_us(void);

typedef struct node_time_logger
{
	std::shared_ptr<std::vector<std::pair<std::string, u64>>> recorded_times;
} node_time_logger;

void log_entry(node_time_logger logger, std::string text);
node_time_logger create_logger();

inline u64 get_time_us(void)
{
	struct timespec ts;
	unsigned long long time;

	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	time = ts.tv_sec * 1000000;
	time += ts.tv_nsec / 1000;

	return time;
}

#endif /* __RT_SCHED_H__ */
