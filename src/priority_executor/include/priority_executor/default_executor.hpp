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

#ifndef RTIS_DEFAULT_EXECUTOR
#define RTIS_DEFAULT_EXECUTOR

#include <rmw/rmw.h>

#include <cassert>
#include <cstdlib>
#include <memory>
#include <vector>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/visibility_control.hpp"
#include "priority_executor/priority_memory_strategy.hpp"

/// Single-threaded executor implementation.
/**
 * This is the default executor created by rclcpp::spin.
 */
class ROSDefaultExecutor : public rclcpp::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ROSDefaultExecutor)
  std::unordered_map<std::shared_ptr<const void>, PriorityExecutable> priority_map;
  node_time_logger logger;

  /// Default constructor. See the default constructor for Executor.
  RCLCPP_PUBLIC
  explicit ROSDefaultExecutor(
      const rclcpp::ExecutorOptions &options = rclcpp::ExecutorOptions());

  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~ROSDefaultExecutor();

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
  bool get_next_executable(rclcpp::AnyExecutable &any_executable, std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

private:
  RCLCPP_DISABLE_COPY(ROSDefaultExecutor)
};

#endif // RCLCPP__EXECUTORS__SINGLE_THREADED_EXECUTOR_HPP_
