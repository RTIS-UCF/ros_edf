
#include "simple_timer/period_timer.hpp"

namespace simple_timer
{
    PeriodTimer::PeriodTimer(long start_delay) : start_delay_time(start_delay * 1000)
    {
    }

    void PeriodTimer::start()
    {
        u64 current_wall_time = get_time_us();

        if (!recording)
        {

            if (start_time == 0)
            {
                start_time = current_wall_time;
            }
            else if (current_wall_time - start_time > start_delay_time)
            {
                recording = true;
                start_time = current_wall_time;
                last_period_time = current_wall_time;
            }
        }
        else
        {
            last_period_time = current_wall_time;
        }
    }
  void PeriodTimer::stop()
  {
    u64 current_wall_time = get_time_us();
    u64 time_diff = 0;

    if (!recording)
    {
        return;
    }
    else
    {
      time_diff = current_wall_time - last_period_time;
      if (time_diff < min_period || min_period == 0)
      {
        min_period = time_diff;
      }
      if (time_diff > max_period || max_period == 0)
      {
        max_period = time_diff;
      }
      last_period = time_diff;
    }
  }
} // namespace simple_timer
