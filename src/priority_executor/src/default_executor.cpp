// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rcpputils/scope_exit.hpp"

#include "rclcpp/any_executable.hpp"
#include "priority_executor/priority_memory_strategy.hpp"
#include "priority_executor/default_executor.hpp"
#include "priority_executor/primes_workload.hpp"

ROSDefaultExecutor::ROSDefaultExecutor(const rclcpp::ExecutorOptions &options)
    : rclcpp::Executor(options) {}

ROSDefaultExecutor::~ROSDefaultExecutor() {}

bool ROSDefaultExecutor::get_next_executable(rclcpp::AnyExecutable &any_executable, std::chrono::nanoseconds timeout)
{
  bool success = false;
  // Check to see if there are any subscriptions or timers needing service
  // TODO(wjwwood): improve run to run efficiency of this function
  // sched_yield();
  wait_for_work(std::chrono::milliseconds(1));
  success = get_next_ready_executable(any_executable);
  return success;
}

void ROSDefaultExecutor::spin()
{
  if (spinning.exchange(true))
  {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););
  while (rclcpp::ok(this->context_) && spinning.load())
  {
    rclcpp::AnyExecutable any_executable;
    if (get_next_executable(any_executable))
    {
      if (any_executable.timer)
      {
        if (priority_map.find(any_executable.timer->get_timer_handle()) != priority_map.end())
        {
          timespec current_time;
          clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);
          uint64_t millis = (current_time.tv_sec * (uint64_t)1000) + (current_time.tv_nsec / 1000000);
          PriorityExecutable next_exec = priority_map[any_executable.timer->get_timer_handle()];

          auto timer = next_exec.timer_handle;
          // TODO: this is really a fire
          log_entry(logger, "timer_" + std::to_string(next_exec.chain_id) + "_release_" + std::to_string(millis + (timer->time_until_trigger().count() / 1000000)));
        }
      }
      execute_any_executable(any_executable);
    }
  }
}
