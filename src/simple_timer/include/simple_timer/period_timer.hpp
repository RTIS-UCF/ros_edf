#ifndef __PERIOD_TIMER__
#define __PERIOD_TIMER__

#include <memory>
#include "simple_timer/rt-sched.hpp"
namespace simple_timer
{
    class PeriodTimer
    {
    public:
        PeriodTimer(long start_delay = 0);
        void start();
        void stop();
        const u64 start_delay_time;
        u64 start_time = 0;

        u64 last_period_time = 0;
        unsigned long max_period = 0;
        unsigned long min_period = 0;
        unsigned long last_period = 0;
        bool recording = false;
    };
} // namespace simple_timer

#endif