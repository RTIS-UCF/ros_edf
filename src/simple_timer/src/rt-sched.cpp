// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * rt-sched.h - sched_setattr() and sched_getattr() API
 *
 * (C) Dario Faggioli <raistlin@linux.it>, 2009, 2010
 * Copyright (C) 2014 BMW Car IT GmbH, Daniel Wagner <daniel.wagner@bmw-carit.de
 */

/* This file is based on Dario Faggioli's libdl. Eventually it will be
   replaced by a proper implemenation of this API. */

#include <unistd.h>
#include <sys/syscall.h>
#include <time.h>

#include "simple_timer/rt-sched.hpp"

int sched_setattr(pid_t pid,
		  const struct sched_attr *attr,
		  unsigned int flags)
{
	return syscall(__NR_sched_setattr, pid, attr, flags);
}

int sched_getattr(pid_t pid,
		  struct sched_attr *attr,
		  unsigned int size,
		  unsigned int flags)
{
	return syscall(__NR_sched_getattr, pid, attr, size, flags);
}

void log_entry(node_time_logger logger, std::string text)
{
    if (logger.recorded_times != nullptr)
    {
        logger.recorded_times->push_back(std::make_pair(text, get_time_us()));
    }
}

node_time_logger create_logger()
{
    node_time_logger logger;
    logger.recorded_times = std::make_shared<std::vector<std::pair<std::string, u64>>>();
    return logger;
}