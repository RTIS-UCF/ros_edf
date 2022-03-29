// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RTIS_TIMED_EXECUTOR
#define RTIS_TIMED_EXECUTOR

#include <rmw/rmw.h>

#include <cassert>
#include <cstdlib>
#include <memory>
#include <vector>
#include <time.h>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/visibility_control.hpp"
namespace timed_executor
{

  /// Single-threaded executor implementation.
  /**
 * This is the default executor created by rclcpp::spin.
 */
  class TimedExecutor : public rclcpp::Executor
  {
  public:
    RCLCPP_SMART_PTR_DEFINITIONS(TimedExecutor)

    /// Default constructor. See the default constructor for Executor.
    RCLCPP_PUBLIC
    explicit TimedExecutor(
        const rclcpp::ExecutorOptions &options = rclcpp::ExecutorOptions(), std::string name = "unnamed executor");

    /// Default destructor.
    RCLCPP_PUBLIC
    virtual ~TimedExecutor();

    /// Single-threaded implementation of spin.
    /**
   * This function will block until work comes in, execute it, and then repeat
   * the process until canceled.
   * It may be interrupt by a call to rclcpp::Executor::cancel() or by ctrl-c
   * if the associated context is configured to shutdown on SIGINT.
   * \throws std::runtime_error when spin() called while already spinning
   */
    RCLCPP_PUBLIC
    void
    spin() override;
    unsigned long long get_max_runtime(void);
    std::string name;

    void set_use_priorities(bool use_prio);

  private:
    RCLCPP_DISABLE_COPY(TimedExecutor)
    // TODO: remove these
    unsigned long long maxRuntime = 0;
    unsigned long long start_time = 0;
    int recording = 0;
    void execute_subscription(rclcpp::AnyExecutable subscription);
    bool
    get_next_executable(rclcpp::AnyExecutable &any_executable, std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    void
    wait_for_work(std::chrono::nanoseconds timeout);

    bool
    get_next_ready_executable(rclcpp::AnyExecutable &any_executable);

    bool use_priorities = true;
  };

} // namespace timed_executor

static void
take_and_do_error_handling(
    const char *action_description,
    const char *topic_or_service_name,
    std::function<bool()> take_action,
    std::function<void()> handle_action)
{
  bool taken = false;
  try
  {
    taken = take_action();
  }
  catch (const rclcpp::exceptions::RCLError &rcl_error)
  {
    RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"),
        "executor %s '%s' unexpectedly failed: %s",
        action_description,
        topic_or_service_name,
        rcl_error.what());
  }
  if (taken)
  {
    handle_action();
  }
  else
  {
    // Message or Service was not taken for some reason.
    // Note that this can be normal, if the underlying middleware needs to
    // interrupt wait spuriously it is allowed.
    // So in that case the executor cannot tell the difference in a
    // spurious wake up and an entity actually having data until trying
    // to take the data.
    RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "executor %s '%s' failed to take anything",
        action_description,
        topic_or_service_name);
  }
}
#endif // RCLCPP__EXECUTORS__SINGLE_THREADED_EXECUTOR_HPP_
