#ifndef RTIS_PRIMES_WORKLOAD
#define RTIS_PRIMES_WORKLOAD
#include <time.h>
#include <sys/time.h>
#include <thread>
#include <cstdio>
typedef double ktimeunit;
ktimeunit nth_prime_silly(int n, double millis = 100);
ktimeunit get_thread_time(struct timespec *currTime);
// int main()
// {
//   printf("%lf\n", nth_prime_silly(100000, 750));
// }
#endif