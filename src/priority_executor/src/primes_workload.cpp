#include "priority_executor/primes_workload.hpp"
#include <iostream>
ktimeunit nth_prime_silly(int n, double millis)
{
  // struct tms this_thread_times;
  struct timespec currTime;
  int sum = 0;
  int i;
  int j;
  ktimeunit const start_cpu_time = get_thread_time(&currTime);
  ktimeunit last_iter_time = 0;
  ktimeunit last_iter_start_time = start_cpu_time;
  for (i = 2; i < 4294967296 - 1; i++)
  {
    // times(&this_thread_times);
    ktimeunit cum_time = get_thread_time(&currTime);
    last_iter_time = cum_time - last_iter_start_time;
    last_iter_start_time = cum_time;
    if ((cum_time - start_cpu_time + last_iter_time) > millis)
    {
      break;
    }
    for (j = 2; j < i; j++)
    {
      sum += j;
    }
    if (cum_time - start_cpu_time > millis)
    {
      std::cout << "Warning: Time limit exceeded" << std::endl;
    }
  }
  return get_thread_time(&currTime) - start_cpu_time;
}
ktimeunit get_thread_time(struct timespec *currTime)
{
  clockid_t threadClockId;
  pthread_getcpuclockid(pthread_self(), &threadClockId);
  clock_gettime(threadClockId, currTime);
  return currTime->tv_nsec / 1000000.0 + currTime->tv_sec * 1000.0;
}
