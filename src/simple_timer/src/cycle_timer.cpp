#include "simple_timer/cycle_timer.hpp"

namespace simple_timer
{
  CycleTimer::CycleTimer(long start_delay) : start_delay_time(start_delay * 1000)
  {
  }

  void CycleTimer::tick()
  {
    u64 current_wall_time = get_time_us();
    u64 time_diff = 0;

    if (!recording)
    {

      if (start_time == 0)
      {
        start_time = current_wall_time;
      }
      else if (current_wall_time - start_time > start_delay_time)
      {
        recording = true;
        last_cycle_time = current_wall_time;
        start_time = current_wall_time;
      }
    }
    else
    {
      time_diff = current_wall_time - last_cycle_time;
      if (time_diff < min_diff || min_diff == 0)
      {
        min_diff = time_diff;
      }
      if (time_diff > max_diff || max_diff == 0)
      {
        max_diff = time_diff;
      }
      last_cycle_time = current_wall_time;
      last_diff = time_diff;
    }
  }
} // namespace simple_timer
