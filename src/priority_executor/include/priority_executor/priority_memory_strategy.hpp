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

#ifndef RTIS_PRIORITY_STRATEGY
#define RTIS_PRIORITY_STRATEGY

#include <memory>
#include <vector>
#include <queue>
#include <time.h>

#include "rcl/allocator.h"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcutils/logging_macros.h"

#include "rmw/types.h"

#include "simple_timer/rt-sched.hpp"

/// Delegate for handling memory allocations while the Executor is executing.
/**
 * By default, the memory strategy dynamically allocates memory for structures that come in from
 * the rmw implementation after the executor waits for work, based on the number of entities that
 * come through.
 */

enum ExecutableType
{
    SUBSCRIPTION,
    SERVICE,
    CLIENT,
    TIMER,
    WAITABLE
};

enum ExecutableScheduleType
{
    CHAIN_INDEPENDENT_PRIORITY, // not used here
    CHAIN_AWARE_PRIORITY,
    DEADLINE,
    DEFAULT, // not used here
};

class PriorityExecutable
{
public:
    std::shared_ptr<const void> handle;
    ExecutableType type;
    bool can_be_run = true;
    std::shared_ptr<rclcpp::Waitable> waitable;
    ExecutableScheduleType sched_type;

    int priority;
    long period = 1000; // milliseconds

    bool is_first_in_chain = false;
    bool is_last_in_chain = false;
    // chain aware deadlines
    std::deque<uint> *deadlines = nullptr;
    std::shared_ptr<rclcpp::TimerBase> timer_handle;
    // just used for logging
    int chain_id = 0;

    // chain aware priority
    int counter = 0;

    PriorityExecutable(std::shared_ptr<const void> h, int p, ExecutableType t, ExecutableScheduleType sched_type = CHAIN_INDEPENDENT_PRIORITY);

    void dont_run();

    void allow_run();

    PriorityExecutable();

    void increment_counter();
};

class PriorityExecutableComparator
{
public:
    bool operator()(const PriorityExecutable *p1, const PriorityExecutable *p2);
};

template <typename Alloc = std::allocator<void>>
class PriorityMemoryStrategy : public rclcpp::memory_strategy::MemoryStrategy
{
public:
    RCLCPP_SMART_PTR_DEFINITIONS(PriorityMemoryStrategy<Alloc>)

    using VoidAllocTraits = typename rclcpp::allocator::AllocRebind<void *, Alloc>;
    using VoidAlloc = typename VoidAllocTraits::allocator_type;

    explicit PriorityMemoryStrategy(std::shared_ptr<Alloc> allocator)
    {
        allocator_ = std::make_shared<VoidAlloc>(*allocator.get());
        logger_ = create_logger();
    }

    PriorityMemoryStrategy()
    {
        allocator_ = std::make_shared<VoidAlloc>();
        logger_ = create_logger();
    }
    node_time_logger logger_;

    void add_guard_condition(const rcl_guard_condition_t *guard_condition) override;

    void remove_guard_condition(const rcl_guard_condition_t *guard_condition) override;

    void clear_handles() override;

    void remove_null_handles(rcl_wait_set_t *wait_set) override;

    bool collect_entities(const WeakNodeList &weak_nodes) override;

    void add_waitable_handle(const rclcpp::Waitable::SharedPtr &waitable) override;

    bool add_handles_to_wait_set(rcl_wait_set_t *wait_set) override;

    void
    get_next_executable(
        rclcpp::AnyExecutable &any_exec,
        const WeakNodeList &weak_nodes);

    void post_execute(rclcpp::AnyExecutable any_exec);

    void
    get_next_subscription(
        rclcpp::AnyExecutable &any_exec,
        const WeakNodeList &weak_nodes) override;

    void
    get_next_service(
        rclcpp::AnyExecutable &any_exec,
        const WeakNodeList &weak_nodes) override;

    void
    get_next_client(rclcpp::AnyExecutable &any_exec, const WeakNodeList &weak_nodes) override;

    void
    get_next_timer(
        rclcpp::AnyExecutable &any_exec,
        const WeakNodeList &weak_nodes) override;

    void
    get_next_waitable(rclcpp::AnyExecutable &any_exec, const WeakNodeList &weak_nodes) override;

    PriorityExecutable *get_priority_settings(std::shared_ptr<const void> executable)
    {
        auto search = priority_map.find(executable);
        if (search != priority_map.end())
        {
            return &(search->second);
        }
        else
        {
            return nullptr;
        }
    }

    rcl_allocator_t get_allocator() override
    {
        return rclcpp::allocator::get_rcl_allocator<void *, VoidAlloc>(*allocator_.get());
    }

    size_t number_of_ready_subscriptions() const override
    {
        size_t number_of_subscriptions = subscription_handles_.size();
        // std::cout << "ready_raw: " << number_of_subscriptions << std::endl;
        for (auto waitable : waitable_handles_)
        {
            number_of_subscriptions += waitable->get_number_of_ready_subscriptions();
        }
        return number_of_subscriptions;
    }

    size_t number_of_ready_services() const override
    {
        size_t number_of_services = service_handles_.size();
        for (auto waitable : waitable_handles_)
        {
            number_of_services += waitable->get_number_of_ready_services();
        }
        return number_of_services;
    }

    size_t number_of_ready_events() const override
    {
        size_t number_of_events = 0;
        for (auto waitable : waitable_handles_)
        {
            number_of_events += waitable->get_number_of_ready_events();
        }
        return number_of_events;
    }

    size_t number_of_ready_clients() const override
    {
        size_t number_of_clients = client_handles_.size();
        for (auto waitable : waitable_handles_)
        {
            number_of_clients += waitable->get_number_of_ready_clients();
        }
        return number_of_clients;
    }

    size_t number_of_guard_conditions() const override
    {
        size_t number_of_guard_conditions = guard_conditions_.size();
        for (auto waitable : waitable_handles_)
        {
            number_of_guard_conditions += waitable->get_number_of_ready_guard_conditions();
        }
        return number_of_guard_conditions;
    }

    size_t number_of_ready_timers() const override
    {
        size_t number_of_timers = timer_handles_.size();
        for (auto waitable : waitable_handles_)
        {
            number_of_timers += waitable->get_number_of_ready_timers();
        }
        return number_of_timers;
    }

    size_t number_of_waitables() const override
    {
        return waitable_handles_.size();
    }

    void set_executable_priority(std::shared_ptr<const void> handle, int priority, ExecutableType t)
    {
        // TODO: any sanity checks should go here
        // priority_map.insert(executable, priority);
        priority_map[handle] = PriorityExecutable(handle, priority, t);
    }
    void set_executable_priority(std::shared_ptr<const void> handle, int priority, ExecutableType t, ExecutableScheduleType sc, int chain_index)
    {
        // TODO: any sanity checks should go here
        // priority_map.insert(executable, priority);
        priority_map[handle] = PriorityExecutable(handle, priority, t, sc);
        priority_map[handle].chain_id = chain_index;
    }

    void set_executable_deadline(std::shared_ptr<const void> handle, int period, ExecutableType t, int chain_id = 0)
    {
        // TODO: any sanity checks should go here
        // priority_map.insert(executable, priority);
        priority_map[handle] = PriorityExecutable(handle, period, t, DEADLINE);
        priority_map[handle].chain_id = chain_id;
    }

    int get_priority(std::shared_ptr<const void> executable)
    {
        auto search = priority_map.find(executable);
        if (search != priority_map.end())
        {
            return search->second.priority;
        }
        else
        {
            return 0;
        }
    }


    void set_first_in_chain(std::shared_ptr<const void> exec_handle)
    {
        PriorityExecutable *settings = get_priority_settings(exec_handle);
        settings->is_first_in_chain = true;
    }

    void set_last_in_chain(std::shared_ptr<const void> exec_handle)
    {
        PriorityExecutable *settings = get_priority_settings(exec_handle);
        settings->is_last_in_chain = true;
    }

    void assign_deadlines_queue(std::shared_ptr<const void> exec_handle, std::deque<uint> *deadlines)
    {
        PriorityExecutable *settings = get_priority_settings(exec_handle);
        settings->deadlines = deadlines;
    }

private:
    PriorityExecutable *get_and_reset_priority(std::shared_ptr<const void> executable, ExecutableType t)
    {
        PriorityExecutable *p = get_priority_settings(executable);
        if (p == nullptr)
        {
            priority_map[executable] = PriorityExecutable(executable, 0, t);
            p = &(priority_map[executable]);
        }
        // p->can_be_run = true;
        return p;
    }

    template <typename T>
    using VectorRebind =
        std::vector<T, typename std::allocator_traits<Alloc>::template rebind_alloc<T>>;

    VectorRebind<const rcl_guard_condition_t *> guard_conditions_;

    VectorRebind<std::shared_ptr<const rcl_subscription_t>> subscription_handles_;
    VectorRebind<std::shared_ptr<const rcl_service_t>> service_handles_;
    VectorRebind<std::shared_ptr<const rcl_client_t>> client_handles_;
    VectorRebind<std::shared_ptr<const rcl_timer_t>> timer_handles_;
    VectorRebind<std::shared_ptr<rclcpp::Waitable>> waitable_handles_;

    std::shared_ptr<VoidAlloc> allocator_;

    // TODO: evaluate using node/subscription namespaced strings as keys

    // holds *all* handle->priority mappings
    std::unordered_map<std::shared_ptr<const void>, PriorityExecutable> priority_map;

    // hold *only valid* executable+priorities
    std::priority_queue<const PriorityExecutable *, std::vector<const PriorityExecutable *>, PriorityExecutableComparator> all_executables_;
};

#endif // RCLCPP__STRATEGIES__ALLOCATOR_MEMORY_STRATEGY_HPP_
