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

#include "priority_executor/priority_executor.hpp"
#include "priority_executor/priority_memory_strategy.hpp"
#include "rclcpp/any_executable.hpp"
#include "rclcpp/scope_exit.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/exceptions.hpp"
#include <memory>
#include <sched.h>
namespace timed_executor
{

  TimedExecutor::TimedExecutor(const rclcpp::ExecutorOptions &options, std::string name)
      : rclcpp::Executor(options)
  {
    this->name = name;
  }

  TimedExecutor::~TimedExecutor() {}

  void
  TimedExecutor::spin()
  {
    if (spinning.exchange(true))
    {
      throw std::runtime_error("spin() called while already spinning");
    }
    RCLCPP_SCOPE_EXIT(this->spinning.store(false););
    while (rclcpp::ok(this->context_) && spinning.load())
    {
      rclcpp::AnyExecutable any_executable;
      // std::cout<<memory_strategy_->number_of_ready_timers()<<std::endl;
      // std::cout << "spinning " << this->name << std::endl;
      // size_t ready = memory_strategy_->number_of_ready_subscriptions();
      // std::cout << "ready:" << ready << std::endl;

      if (get_next_executable(any_executable))
      {
        if (any_executable.subscription)
        {
          execute_subscription(any_executable);
        }
        else
        {
          execute_any_executable(any_executable);
        }
      }
    }
    std::cout << "shutdown" << std::endl;
  }

  unsigned long long TimedExecutor::get_max_runtime(void)
  {
    return maxRuntime;
  }

  void
  TimedExecutor::execute_subscription(rclcpp::AnyExecutable executable)
  {
    rclcpp::SubscriptionBase::SharedPtr subscription = executable.subscription;

    rclcpp::MessageInfo message_info;
    message_info.get_rmw_message_info().from_intra_process = false;

    if (subscription->is_serialized())
    {
      // This is the case where a copy of the serialized message is taken from
      // the middleware via inter-process communication.

      // if this should happen on another thread,  we'd pass it to a thread here
      std::shared_ptr<rclcpp::SerializedMessage> serialized_msg = subscription->create_serialized_message();
      take_and_do_error_handling(
          "taking a serialized message from topic",
          subscription->get_topic_name(),
          [&]()
          {
            auto result = subscription->take_serialized(*serialized_msg.get(), message_info);
            // RCLCPP_INFO(rclcpp::get_logger(this->name), "at topic %s, serialized msg sent at %ld, and recieved at %ld", executable.node_base->get_name(), message_info.get_rmw_message_info().source_timestamp, message_info.get_rmw_message_info().received_timestamp);
            return result;
          },
          [&]()
          {
            auto void_serialized_msg = std::static_pointer_cast<void>(serialized_msg);
            subscription->handle_message(void_serialized_msg, message_info);
          });
      subscription->return_serialized_message(serialized_msg);
    }
    else if (subscription->can_loan_messages())
    {
      // This is the case where a loaned message is taken from the middleware via
      // inter-process communication, given to the user for their callback,
      // and then returned.
      void *loaned_msg = nullptr;
      // TODO(wjwwood): refactor this into methods on subscription when LoanedMessage
      //   is extened to support subscriptions as well.
      take_and_do_error_handling(
          "taking a loaned message from topic",
          subscription->get_topic_name(),
          [&]()
          {
            rcl_ret_t ret = rcl_take_loaned_message(
                subscription->get_subscription_handle().get(),
                &loaned_msg,
                &message_info.get_rmw_message_info(),
                nullptr);
            if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret)
            {
              return false;
            }
            else if (RCL_RET_OK != ret)
            {
              rclcpp::exceptions::throw_from_rcl_error(ret);
            }
            // RCLCPP_INFO(rclcpp::get_logger(this->name), "at topic %s, loaned msg sent at %ld, and recieved at %ld", executable.node_base->get_name(), message_info.get_rmw_message_info().source_timestamp, message_info.get_rmw_message_info().received_timestamp);
            return true;
          },
          [&]()
          { subscription->handle_loaned_message(loaned_msg, message_info); });
      rcl_ret_t ret = rcl_return_loaned_message_from_subscription(
          subscription->get_subscription_handle().get(),
          loaned_msg);
      if (RCL_RET_OK != ret)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "rcl_return_loaned_message_from_subscription() failed for subscription on topic '%s': %s",
            subscription->get_topic_name(), rcl_get_error_string().str);
      }
      loaned_msg = nullptr;
    }
    else
    {
      // This case is taking a copy of the message data from the middleware via
      // inter-process communication.
      std::shared_ptr<void> message = subscription->create_message();
      take_and_do_error_handling(
          "taking a message from topic",
          subscription->get_topic_name(),
          [&]()
          {
            auto result = subscription->take_type_erased(message.get(), message_info);
            // RCLCPP_INFO(rclcpp::get_logger(this->name), "at topic %s, IPC msg sent at %ld, and recieved at %ld", executable.node_base->get_name(), message_info.get_rmw_message_info().source_timestamp, message_info.get_rmw_message_info().received_timestamp);
            return result;
          },
          [&]()
          { subscription->handle_message(message, message_info); });
      // this just deallocates
      subscription->return_message(message);
    }
  }
  bool TimedExecutor::get_next_executable(rclcpp::AnyExecutable &any_executable, std::chrono::nanoseconds timeout)
  {
    bool success = false;
    // Check to see if there are any subscriptions or timers needing service
    // TODO(wjwwood): improve run to run efficiency of this function
    // sched_yield();
    wait_for_work(std::chrono::milliseconds(1));
    success = get_next_ready_executable(any_executable);
    return success;
  }

  // TODO: since we're calling this more often, clean it up a bit
  void
  TimedExecutor::wait_for_work(std::chrono::nanoseconds timeout)
  {
    {
      std::unique_lock<std::mutex> lock(memory_strategy_mutex_);

      // Collect the subscriptions and timers to be waited on
      memory_strategy_->clear_handles();
      bool has_invalid_weak_nodes = memory_strategy_->collect_entities(weak_nodes_);

      // Clean up any invalid nodes, if they were detected
      if (has_invalid_weak_nodes)
      {
        auto node_it = weak_nodes_.begin();
        auto gc_it = guard_conditions_.begin();
        while (node_it != weak_nodes_.end())
        {
          if (node_it->expired())
          {
            node_it = weak_nodes_.erase(node_it);
            memory_strategy_->remove_guard_condition(*gc_it);
            gc_it = guard_conditions_.erase(gc_it);
          }
          else
          {
            ++node_it;
            ++gc_it;
          }
        }
      }
      // clear wait set
      rcl_ret_t ret = rcl_wait_set_clear(&wait_set_);
      if (ret != RCL_RET_OK)
      {
        rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't clear wait set");
      }

      // The size of waitables are accounted for in size of the other entities
      ret = rcl_wait_set_resize(
          &wait_set_, memory_strategy_->number_of_ready_subscriptions(),
          memory_strategy_->number_of_guard_conditions(), memory_strategy_->number_of_ready_timers(),
          memory_strategy_->number_of_ready_clients(), memory_strategy_->number_of_ready_services(),
          memory_strategy_->number_of_ready_events());
      if (RCL_RET_OK != ret)
      {
        rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't resize the wait set");
      }

      if (!memory_strategy_->add_handles_to_wait_set(&wait_set_))
      {
        throw std::runtime_error("Couldn't fill wait set");
      }
    }
    rcl_ret_t status =
        rcl_wait(&wait_set_, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count());
    if (status == RCL_RET_WAIT_SET_EMPTY)
    {
      RCUTILS_LOG_WARN_NAMED(
          "rclcpp",
          "empty wait set received in rcl_wait(). This should never happen.");
    }
    else if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT)
    {
      using rclcpp::exceptions::throw_from_rcl_error;
      throw_from_rcl_error(status, "rcl_wait() failed");
    }

    // check the null handles in the wait set and remove them from the handles in memory strategy
    // for callback-based entities
    memory_strategy_->remove_null_handles(&wait_set_);
  }
  bool
  TimedExecutor::get_next_ready_executable(rclcpp::AnyExecutable &any_executable)
  {
    bool success = false;
    if (use_priorities)
    {
      std::shared_ptr<PriorityMemoryStrategy<>> strat = std::dynamic_pointer_cast<PriorityMemoryStrategy<>>(memory_strategy_);
      strat->get_next_executable(any_executable, weak_nodes_);
      if (any_executable.timer || any_executable.subscription || any_executable.service || any_executable.client || any_executable.waitable)
      {
        success = true;
      }
    }
    else
    {
      // Check the timers to see if there are any that are ready
      memory_strategy_->get_next_timer(any_executable, weak_nodes_);
      if (any_executable.timer)
      {
        std::cout << "got timer" << std::endl;
        success = true;
      }
      if (!success)
      {
        // Check the subscriptions to see if there are any that are ready
        memory_strategy_->get_next_subscription(any_executable, weak_nodes_);
        if (any_executable.subscription)
        {
          // std::cout << "got subs" << std::endl;
          success = true;
        }
      }
      if (!success)
      {
        // Check the services to see if there are any that are ready
        memory_strategy_->get_next_service(any_executable, weak_nodes_);
        if (any_executable.service)
        {
          std::cout << "got serv" << std::endl;
          success = true;
        }
      }
      if (!success)
      {
        // Check the clients to see if there are any that are ready
        memory_strategy_->get_next_client(any_executable, weak_nodes_);
        if (any_executable.client)
        {
          std::cout << "got client" << std::endl;
          success = true;
        }
      }
      if (!success)
      {
        // Check the waitables to see if there are any that are ready
        memory_strategy_->get_next_waitable(any_executable, weak_nodes_);
        if (any_executable.waitable)
        {
          std::cout << "got wait" << std::endl;
          success = true;
        }
      }
    }
    // At this point any_exec should be valid with either a valid subscription
    // or a valid timer, or it should be a null shared_ptr
    if (success)
    {
      // If it is valid, check to see if the group is mutually exclusive or
      // not, then mark it accordingly
      using rclcpp::callback_group::CallbackGroupType;
      if (
          any_executable.callback_group &&
          any_executable.callback_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive)
      {
        // It should not have been taken otherwise
        assert(any_executable.callback_group->can_be_taken_from().load());
        // Set to false to indicate something is being run from this group
        // This is reset to true either when the any_exec is executed or when the
        // any_exec is destructued
        any_executable.callback_group->can_be_taken_from().store(false);
      }
    }
    // If there is no ready executable, return false
    return success;
  }

  void TimedExecutor::set_use_priorities(bool use_prio)
  {
    use_priorities = use_prio;
  }

} // namespace timed_executor
