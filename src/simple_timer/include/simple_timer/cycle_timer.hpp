#ifndef __CYCLE_TIMER__
#define __CYCLE_TIMER__

#include <memory>
#include "simple_timer/rt-sched.hpp"
namespace simple_timer
{
    class CycleTimer
    {
    public:
        CycleTimer(long start_delay=0);
        void tick() ;
        const u64 start_delay_time;
        u64 start_time = 0;
        u64 last_cycle_time = 0;
        unsigned long max_diff = 0;
        unsigned long min_diff = 0;
        unsigned long last_diff = 0;
        bool recording = false;
    };
} // namespace simple_timer

#endif