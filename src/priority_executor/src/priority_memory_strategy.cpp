#include "priority_executor/priority_memory_strategy.hpp"
#include "simple_timer/rt-sched.hpp"
#include <sstream>

PriorityExecutable::PriorityExecutable(std::shared_ptr<const void> h, int p, ExecutableType t, ExecutableScheduleType sched_type)
{
    handle = h;
    type = t;
    if (sched_type == CHAIN_INDEPENDENT_PRIORITY || sched_type == CHAIN_AWARE_PRIORITY)
    {
        priority = p;
    }
    else if (sched_type == DEADLINE)
    {
        period = p;
    }
    this->sched_type = sched_type;
}

PriorityExecutable::PriorityExecutable()
{
    handle = nullptr;
    priority = 0;
    type = SUBSCRIPTION;
}

void PriorityExecutable::dont_run()
{

    this->can_be_run = false;
}

void PriorityExecutable::allow_run()
{
    if (this->can_be_run)
    {
        // release has been detected
    }
    else
    {
        this->can_be_run = true;
    }
}

void PriorityExecutable::increment_counter()
{
    this->counter += 1;
}

bool PriorityExecutableComparator::operator()(const PriorityExecutable *p1, const PriorityExecutable *p2)
{
    if (p1 == nullptr || p2 == nullptr)
    {
        // TODO: realistic value
        return 0;
    }
    if (p1->sched_type != p2->sched_type)
    {
        if (p1->sched_type == DEADLINE)
        {
            return false;
        }
        else if (p2->sched_type == DEADLINE)
        {
            return true;
        }
        if (p1->sched_type == CHAIN_INDEPENDENT_PRIORITY)
        {
            return false;
        }
        else if (p2->sched_type == CHAIN_INDEPENDENT_PRIORITY)
        {
            return true;
        }
        else if (p1->sched_type == CHAIN_AWARE_PRIORITY)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    if (p1->sched_type == CHAIN_INDEPENDENT_PRIORITY)
    {
        // lower value runs first
        return p1->priority > p2->priority;
    }
    if (p1->sched_type == CHAIN_AWARE_PRIORITY)
    {
        if (p1->priority != p2->priority)
        {
            return p1->priority > p2->priority;
        }
        // return p1->counter > p2->counter;
        return 0;
    }
    if (p1->sched_type == DEADLINE)
    {
        // TODO: use the counter logic here as well

        uint p1_deadline = 0;
        uint p2_deadline = 0;
        if (p1->deadlines != nullptr && !p1->deadlines->empty())
        {
            p1_deadline = p1->deadlines->front();
        }
        if (p2->deadlines != nullptr && !p2->deadlines->empty())
        {
            p2_deadline = p2->deadlines->front();
        }
        if (p1_deadline == 0)
        {
            return true;
        }
        if (p2_deadline == 0)
        {
            return false;
        }
        if (p1_deadline == p2_deadline)
        {
            // these must be from the same chain
            // settle the difference with the counter
            return p1->counter > p2->counter;
        }
        return p1_deadline > p2_deadline;
    }
    else
    {
        std::cout << "invalid compare opration on priority_exec" << std::endl;
        return 0;
    }
}

template <>
void PriorityMemoryStrategy<>::add_guard_condition(const rcl_guard_condition_t *guard_condition)
{
    for (const auto &existing_guard_condition : guard_conditions_)
    {
        if (existing_guard_condition == guard_condition)
        {
            return;
        }
    }
    guard_conditions_.push_back(guard_condition);
}

template <>
void PriorityMemoryStrategy<>::remove_guard_condition(const rcl_guard_condition_t *guard_condition)
{
    for (auto it = guard_conditions_.begin(); it != guard_conditions_.end(); ++it)
    {
        if (*it == guard_condition)
        {
            guard_conditions_.erase(it);
            break;
        }
    }
}

template <>
void PriorityMemoryStrategy<>::clear_handles()
{
    subscription_handles_.clear();
    service_handles_.clear();
    client_handles_.clear();
    timer_handles_.clear();
    waitable_handles_.clear();

    // priority_queue doesn't have a clear function, so we swap it with an empty one. `empty` will go out of scope, and be cleared
    // all_executables_ = std::priority_queue<const PriorityExecutable *, std::vector<const PriorityExecutable *>, PriorityExecutableComparator>();
    std::priority_queue<const PriorityExecutable *, std::vector<const PriorityExecutable *>, PriorityExecutableComparator> empty;
    std::swap(all_executables_, empty);
    if (!all_executables_.empty())
    {
        std::cout << "couldn't clear all exec" << std::endl;
    }
}

template <>
void PriorityMemoryStrategy<>::remove_null_handles(rcl_wait_set_t *wait_set)
{
    // TODO(jacobperron): Check if wait set sizes are what we expect them to be?
    //                    e.g. wait_set->size_of_clients == client_handles_.size()

    // Important to use subscription_handles_.size() instead of wait set's size since
    // there may be more subscriptions in the wait set due to Waitables added to the end.
    // The same logic applies for other entities.
    for (size_t i = 0; i < subscription_handles_.size(); ++i)
    {
        if (!wait_set->subscriptions[i])
        {
            priority_map[subscription_handles_[i]].dont_run();
            subscription_handles_[i].reset();
        }
        else
        {
            priority_map[subscription_handles_[i]].allow_run();
        }
    }
    for (size_t i = 0; i < service_handles_.size(); ++i)
    {
        if (!wait_set->services[i])
        {
            priority_map[service_handles_[i]].dont_run();
            service_handles_[i].reset();
        }
        else
        {
            priority_map[service_handles_[i]].allow_run();
        }
    }
    for (size_t i = 0; i < client_handles_.size(); ++i)
    {
        if (!wait_set->clients[i])
        {
            priority_map[client_handles_[i]].dont_run();
            client_handles_[i].reset();
        }
        else
        {
            priority_map[client_handles_[i]].allow_run();
        }
    }
    for (size_t i = 0; i < timer_handles_.size(); ++i)
    {
        if (!wait_set->timers[i])
        {
            priority_map[timer_handles_[i]].dont_run();
            timer_handles_[i].reset();
        }
        else
        {
            priority_map[timer_handles_[i]].allow_run();
        }
    }
    for (size_t i = 0; i < waitable_handles_.size(); ++i)
    {
        if (!waitable_handles_[i]->is_ready(wait_set))
        {
            priority_map[waitable_handles_[i]].dont_run();
            waitable_handles_[i].reset();
        }
        else
        {
            priority_map[waitable_handles_[i]].allow_run();
        }
    }

    subscription_handles_.erase(
        std::remove(subscription_handles_.begin(), subscription_handles_.end(), nullptr),
        subscription_handles_.end());

    service_handles_.erase(
        std::remove(service_handles_.begin(), service_handles_.end(), nullptr),
        service_handles_.end());

    client_handles_.erase(
        std::remove(client_handles_.begin(), client_handles_.end(), nullptr),
        client_handles_.end());

    timer_handles_.erase(
        std::remove(timer_handles_.begin(), timer_handles_.end(), nullptr),
        timer_handles_.end());

    waitable_handles_.erase(
        std::remove(waitable_handles_.begin(), waitable_handles_.end(), nullptr),
        waitable_handles_.end());
}

template <>
bool PriorityMemoryStrategy<>::collect_entities(const WeakNodeList &weak_nodes)
{

    bool has_invalid_weak_nodes = false;
    for (auto &weak_node : weak_nodes)
    {
        auto node = weak_node.lock();
        if (!node)
        {
            has_invalid_weak_nodes = true;
            continue;
        }
        for (auto &weak_group : node->get_callback_groups())
        {
            auto group = weak_group.lock();
            if (!group || !group->can_be_taken_from().load())
            {
                continue;
            }
            group->find_subscription_ptrs_if(
                [this](const rclcpp::SubscriptionBase::SharedPtr &subscription)
                {
                    auto subscription_handle = subscription->get_subscription_handle();
                    subscription_handles_.push_back(subscription_handle);
                    all_executables_.push(get_and_reset_priority(subscription_handle, SUBSCRIPTION));

                    return false;
                });
            group->find_service_ptrs_if(
                [this](const rclcpp::ServiceBase::SharedPtr &service)
                {
                    all_executables_.push(get_and_reset_priority(service->get_service_handle(), SERVICE));
                    service_handles_.push_back(service->get_service_handle());
                    return false;
                });
            group->find_client_ptrs_if(
                [this](const rclcpp::ClientBase::SharedPtr &client)
                {
                    all_executables_.push(get_and_reset_priority(client->get_client_handle(), CLIENT));
                    client_handles_.push_back(client->get_client_handle());
                    return false;
                });
            group->find_timer_ptrs_if(
                [this](const rclcpp::TimerBase::SharedPtr &timer)
                {
                    all_executables_.push(get_and_reset_priority(timer->get_timer_handle(), TIMER));
                    timer_handles_.push_back(timer->get_timer_handle());
                    return false;
                });
            group->find_waitable_ptrs_if(
                [this](const rclcpp::Waitable::SharedPtr &waitable)
                {
                    all_executables_.push(get_and_reset_priority(waitable, WAITABLE));
                    waitable_handles_.push_back(waitable);
                    return false;
                });
        }
    }
    return has_invalid_weak_nodes;
}

template <>
void PriorityMemoryStrategy<>::add_waitable_handle(const rclcpp::Waitable::SharedPtr &waitable)
{

    if (nullptr == waitable)
    {
        throw std::runtime_error("waitable object unexpectedly nullptr");
    }
    waitable_handles_.push_back(waitable);
}

template <>
bool PriorityMemoryStrategy<>::add_handles_to_wait_set(rcl_wait_set_t *wait_set)
{
    for (auto subscription : subscription_handles_)
    {
        if (rcl_wait_set_add_subscription(wait_set, subscription.get(), NULL) != RCL_RET_OK)
        {
            RCUTILS_LOG_ERROR_NAMED(
                "rclcpp",
                "Couldn't add subscription to wait set: %s", rcl_get_error_string().str);
            return false;
        }
    }

    for (auto client : client_handles_)
    {
        if (rcl_wait_set_add_client(wait_set, client.get(), NULL) != RCL_RET_OK)
        {
            RCUTILS_LOG_ERROR_NAMED(
                "rclcpp",
                "Couldn't add client to wait set: %s", rcl_get_error_string().str);
            return false;
        }
    }

    for (auto service : service_handles_)
    {
        if (rcl_wait_set_add_service(wait_set, service.get(), NULL) != RCL_RET_OK)
        {
            RCUTILS_LOG_ERROR_NAMED(
                "rclcpp",
                "Couldn't add service to wait set: %s", rcl_get_error_string().str);
            return false;
        }
    }

    for (auto timer : timer_handles_)
    {
        if (rcl_wait_set_add_timer(wait_set, timer.get(), NULL) != RCL_RET_OK)
        {
            RCUTILS_LOG_ERROR_NAMED(
                "rclcpp",
                "Couldn't add timer to wait set: %s", rcl_get_error_string().str);
            return false;
        }
    }

    for (auto guard_condition : guard_conditions_)
    {
        if (rcl_wait_set_add_guard_condition(wait_set, guard_condition, NULL) != RCL_RET_OK)
        {
            RCUTILS_LOG_ERROR_NAMED(
                "rclcpp",
                "Couldn't add guard_condition to wait set: %s",
                rcl_get_error_string().str);
            return false;
        }
    }

    for (auto waitable : waitable_handles_)
    {
        if (!waitable->add_to_wait_set(wait_set))
        {
            RCUTILS_LOG_ERROR_NAMED(
                "rclcpp",
                "Couldn't add waitable to wait set: %s", rcl_get_error_string().str);
            return false;
        }
    }
    return true;
}

template <>
void PriorityMemoryStrategy<>::get_next_executable(
    rclcpp::AnyExecutable &any_exec,
    const WeakNodeList &weak_nodes)
{

    const PriorityExecutable *next_exec = nullptr;
    while (!all_executables_.empty())
    {
        next_exec = all_executables_.top();
        all_executables_.pop();
        if (!next_exec->can_be_run)
        {
            continue;
        }
        ExecutableType type = next_exec->type;
        switch (type)
        {
        case SUBSCRIPTION:
        {
            std::shared_ptr<const rcl_subscription_t> subs_handle = std::static_pointer_cast<const rcl_subscription_t>(next_exec->handle);
            auto subscription = get_subscription_by_handle(subs_handle, weak_nodes);
            if (subscription)
            {
                auto group = get_group_by_subscription(subscription, weak_nodes);
                if (!group)
                {
                    // Group was not found, meaning the waitable is not valid...
                    // Remove it from the ready list and continue looking
                    // it = subscription_handles_.erase(it);
                    continue;
                }
                if (!group->can_be_taken_from().load())
                {
                    // Group is mutually exclusive and is being used, so skip it for now
                    // Leave it to be checked next time, but continue searching
                    // ++it;
                    continue;
                }
                any_exec.callback_group = group;
                any_exec.subscription = subscription;
                any_exec.node_base = get_node_by_group(group, weak_nodes);
                // std::cout << "Using new priority sub " << subscription->get_topic_name() << std::endl;
            }
        }
        break;
        case SERVICE:
        {
            std::shared_ptr<const rcl_service_t> service_handle = std::static_pointer_cast<const rcl_service_t>(next_exec->handle);
            auto service = get_service_by_handle(service_handle, weak_nodes);
            if (service)
            {
                auto group = get_group_by_service(service, weak_nodes);
                if (!group)
                {
                    // Group was not found, meaning the waitable is not valid...
                    // Remove it from the ready list and continue looking
                    // it = subscription_handles_.erase(it);
                    continue;
                }
                if (!group->can_be_taken_from().load())
                {
                    // Group is mutually exclusive and is being used, so skip it for now
                    // Leave it to be checked next time, but continue searching
                    // ++it;
                    continue;
                }
                any_exec.callback_group = group;
                any_exec.service = service;
                any_exec.node_base = get_node_by_group(group, weak_nodes);
                // std::cout << "Using new priority service " << service->get_service_name() << std::endl;
            }
        }
        break;
        case CLIENT:
        {
            std::shared_ptr<const rcl_client_t> client_handle = std::static_pointer_cast<const rcl_client_t>(next_exec->handle);
            auto client = get_client_by_handle(client_handle, weak_nodes);
            if (client)
            {
                auto group = get_group_by_client(client, weak_nodes);
                if (!group)
                {
                    // Group was not found, meaning the waitable is not valid...
                    // Remove it from the ready list and continue looking
                    // it = subscription_handles_.erase(it);
                    continue;
                }
                if (!group->can_be_taken_from().load())
                {
                    // Group is mutually exclusive and is being used, so skip it for now
                    // Leave it to be checked next time, but continue searching
                    // ++it;
                    continue;
                }
                any_exec.callback_group = group;
                any_exec.client = client;
                any_exec.node_base = get_node_by_group(group, weak_nodes);
                // std::cout << "Using new priority client " << client->get_service_name() << std::endl;
            }
        }
        break;
        case TIMER:
        {
            std::shared_ptr<const rcl_timer_t> timer_handle = std::static_pointer_cast<const rcl_timer_t>(next_exec->handle);
            auto timer = get_timer_by_handle(timer_handle, weak_nodes);
            if (timer)
            {
                auto group = get_group_by_timer(timer, weak_nodes);
                if (!group)
                {
                    // Group was not found, meaning the waitable is not valid...
                    // Remove it from the ready list and continue looking
                    // it = subscription_handles_.erase(it);
                    continue;
                }
                if (!group->can_be_taken_from().load())
                {
                    // Group is mutually exclusive and is being used, so skip it for now
                    // Leave it to be checked next time, but continue searching
                    // ++it;
                    continue;
                }
                any_exec.callback_group = group;
                any_exec.timer = timer;
                any_exec.node_base = get_node_by_group(group, weak_nodes);
            }
        }
        break;
        case WAITABLE:
        {
            std::shared_ptr<rclcpp::Waitable> waitable_handle = std::static_pointer_cast<rclcpp::Waitable>(next_exec->waitable);
            auto waitable = waitable_handle;
            if (waitable)
            {
                auto group = get_group_by_waitable(waitable, weak_nodes);
                if (!group)
                {
                    // Group was not found, meaning the waitable is not valid...
                    // Remove it from the ready list and continue looking
                    // it = subscription_handles_.erase(it);
                    continue;
                }
                if (!group->can_be_taken_from().load())
                {
                    // Group is mutually exclusive and is being used, so skip it for now
                    // Leave it to be checked next time, but continue searching
                    // ++it;
                    continue;
                }
                any_exec.callback_group = group;
                any_exec.waitable = waitable;
                any_exec.node_base = get_node_by_group(group, weak_nodes);
                // std::cout << "Using new priority waitable" << std::endl;
            }
        }
        break;
        default:
            // std::cout << "Unknown type from priority!!!" << std::endl;
            break;
        }
        if (next_exec->is_first_in_chain && next_exec->sched_type == DEADLINE)
        {
            // std::cout << "running first in chain deadline" << std::endl;
        }
        return;
    }
}

template <>
void PriorityMemoryStrategy<>::post_execute(rclcpp::AnyExecutable any_exec)
{

    PriorityExecutable *next_exec = nullptr;
    if (any_exec.subscription)
    {
        next_exec = get_priority_settings(any_exec.subscription->get_subscription_handle());
    }
    else if (any_exec.service)
    {
        next_exec = get_priority_settings(any_exec.service->get_service_handle());
    }
    else if (any_exec.client)
    {
        next_exec = get_priority_settings(any_exec.client->get_client_handle());
    }
    else if (any_exec.timer)
    {
        next_exec = get_priority_settings(any_exec.timer->get_timer_handle());
    }
    else if (any_exec.waitable)
    {
        next_exec = get_priority_settings(any_exec.waitable);
    }
    if (next_exec == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Could not find priority settings for handle");
    }
    // callback has executed
    // std::cout<< "running callback. first? " << next_exec->is_first_in_chain << " type " << next_exec->sched_type << std::endl;
    if (next_exec->is_first_in_chain && next_exec->sched_type != DEADLINE)
    {
        timespec current_time;
        clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);
        uint64_t millis = (current_time.tv_sec * (uint64_t)1000) + (current_time.tv_nsec / 1000000);

        auto timer = next_exec->timer_handle;
        int64_t time_until_next_call = timer->time_until_trigger().count() / 1000000;
        // std::cout << "end of chain. time until trigger: " << std::to_string(time_until_next_call) << std::endl;
    }
    if (next_exec->is_last_in_chain && next_exec->sched_type == DEADLINE)
    {
        // did we make the deadline?
        timespec current_time;
        clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);
        uint64_t millis = (current_time.tv_sec * (uint64_t)1000) + (current_time.tv_nsec / 1000000);
        uint64_t this_deadline = next_exec->deadlines->front();
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "last, chain id: %d", next_exec->chain_id);
        // for (auto deadline : *next_exec->deadlines)
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "deadline: %d", deadline);
        // }
        std::ostringstream oss;
        // oss << "{\"operation\": \"deadline_check\", \"chain_id\": " << next_exec->chain_id << ", \"deadline\": " << next_exec->deadlines->front() << ", \"current_time\": " << millis << "}";
        // log_entry(logger_, oss.str());

        if (next_exec->timer_handle == nullptr)
        {
            std::cout << "tried to use a chain without a timer handle!!!" << std::endl;
        }
        auto timer = next_exec->timer_handle;
        if (timer == nullptr)
        {
            std::cout << "somehow, this timer handle didn't have an associated timer" << std::endl;
        }
        // timespec current_time;
        // clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);
        // uint64_t millis = (current_time.tv_sec * (uint64_t)1000) + (current_time.tv_nsec / 1000000);
        // uint64_t this_deadline = next_exec->deadlines->front();
        uint64_t next_deadline = 0;
        // if (millis < this_deadline)
        // {
        //     next_deadline = this_deadline + next_exec->period;
        // }
        // else
        {
            int periods_late = std::ceil((millis - this_deadline) / (double)next_exec->period);
            if (periods_late < 1)
            {
                periods_late = 1;
            }
            next_deadline = this_deadline + (periods_late) * next_exec->period;
        }
        // std::chrono::nanoseconds time_until = next_exec->timer_handle->time_until_trigger();
        // next_deadline = millis + (time_until.count() / 1000000) + next_exec->period;
        // the next deadline is the current time plus the period, skipping periods that have already passed

        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "next deadline: %d", next_deadline);
        oss << "{\"operation\": \"next_deadline\", \"chain_id\": " << next_exec->chain_id << ", \"deadline\": " << next_deadline << "}";
        // oss << "{\"operation\": \"next_deadline\", \"chain_id\": " << next_exec->chain_id << ", \"deadline\": " << next_exec->deadlines->front() << "}";
        log_entry(logger_, oss.str());
        next_exec->deadlines->push_back(next_deadline);

        // print chain id and deadline contents
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "chain id: %d", next_exec->chain_id);
        // for (auto deadline : *next_exec->deadlines)
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "deadline: %d", deadline);
        // }
        // std::cout << "deadline set" << std::endl;

        if (!next_exec->deadlines->empty())
            next_exec->deadlines->pop_front();
    }
    if (next_exec->sched_type == CHAIN_AWARE_PRIORITY || next_exec->sched_type == DEADLINE)
    {
        // this is safe, since we popped it earlier
        // get a mutable reference
        // TODO: find a cleaner way to do this
        PriorityExecutable *mut_executable = get_priority_settings(next_exec->handle);
        // std::cout << "running chain aware cb" << std::endl;
        mut_executable->increment_counter();
    }
}

template <>
void PriorityMemoryStrategy<>::
    get_next_subscription(
        rclcpp::AnyExecutable &any_exec,
        const WeakNodeList &weak_nodes)
{

    auto it = subscription_handles_.begin();
    while (it != subscription_handles_.end())
    {
        auto subscription = get_subscription_by_handle(*it, weak_nodes);
        if (subscription)
        {
            // Find the group for this handle and see if it can be serviced
            auto group = get_group_by_subscription(subscription, weak_nodes);
            if (!group)
            {
                // Group was not found, meaning the subscription is not valid...
                // Remove it from the ready list and continue looking
                it = subscription_handles_.erase(it);
                continue;
            }
            if (!group->can_be_taken_from().load())
            {
                // Group is mutually exclusive and is being used, so skip it for now
                // Leave it to be checked next time, but continue searching
                ++it;
                continue;
            }
            // Otherwise it is safe to set and return the any_exec
            any_exec.subscription = subscription;
            any_exec.callback_group = group;
            any_exec.node_base = get_node_by_group(group, weak_nodes);
            subscription_handles_.erase(it);
            return;
        }
        // Else, the subscription is no longer valid, remove it and continue
        it = subscription_handles_.erase(it);
    }
}

template <>
void PriorityMemoryStrategy<>::
    get_next_service(
        rclcpp::AnyExecutable &any_exec,
        const WeakNodeList &weak_nodes)
{
    auto it = service_handles_.begin();
    while (it != service_handles_.end())
    {
        auto service = get_service_by_handle(*it, weak_nodes);
        if (service)
        {
            // Find the group for this handle and see if it can be serviced
            auto group = get_group_by_service(service, weak_nodes);
            if (!group)
            {
                // Group was not found, meaning the service is not valid...
                // Remove it from the ready list and continue looking
                it = service_handles_.erase(it);
                continue;
            }
            if (!group->can_be_taken_from().load())
            {
                // Group is mutually exclusive and is being used, so skip it for now
                // Leave it to be checked next time, but continue searching
                ++it;
                continue;
            }
            // Otherwise it is safe to set and return the any_exec
            any_exec.service = service;
            any_exec.callback_group = group;
            any_exec.node_base = get_node_by_group(group, weak_nodes);
            service_handles_.erase(it);
            return;
        }
        // Else, the service is no longer valid, remove it and continue
        it = service_handles_.erase(it);
    }
}

template <>
void PriorityMemoryStrategy<>::
    get_next_client(
        rclcpp::AnyExecutable &any_exec,
        const WeakNodeList &weak_nodes)
{

    auto it = client_handles_.begin();
    while (it != client_handles_.end())
    {
        auto client = get_client_by_handle(*it, weak_nodes);
        if (client)
        {
            // Find the group for this handle and see if it can be serviced
            auto group = get_group_by_client(client, weak_nodes);
            if (!group)
            {
                // Group was not found, meaning the service is not valid...
                // Remove it from the ready list and continue looking
                it = client_handles_.erase(it);
                continue;
            }
            if (!group->can_be_taken_from().load())
            {
                // Group is mutually exclusive and is being used, so skip it for now
                // Leave it to be checked next time, but continue searching
                ++it;
                continue;
            }
            // Otherwise it is safe to set and return the any_exec
            any_exec.client = client;
            any_exec.callback_group = group;
            any_exec.node_base = get_node_by_group(group, weak_nodes);
            client_handles_.erase(it);
            return;
        }
        // Else, the service is no longer valid, remove it and continue
        it = client_handles_.erase(it);
    }
}

template <>
void PriorityMemoryStrategy<>::
    get_next_timer(
        rclcpp::AnyExecutable &any_exec,
        const WeakNodeList &weak_nodes)
{

    auto it = timer_handles_.begin();
    while (it != timer_handles_.end())
    {
        auto timer = get_timer_by_handle(*it, weak_nodes);
        if (timer)
        {
            // Find the group for this handle and see if it can be serviced
            auto group = get_group_by_timer(timer, weak_nodes);
            if (!group)
            {
                // Group was not found, meaning the timer is not valid...
                // Remove it from the ready list and continue looking
                it = timer_handles_.erase(it);
                continue;
            }
            if (!group->can_be_taken_from().load())
            {
                // Group is mutually exclusive and is being used, so skip it for now
                // Leave it to be checked next time, but continue searching
                ++it;
                continue;
            }
            // Otherwise it is safe to set and return the any_exec
            any_exec.timer = timer;
            any_exec.callback_group = group;
            any_exec.node_base = get_node_by_group(group, weak_nodes);
            timer_handles_.erase(it);
            return;
        }
        // Else, the service is no longer valid, remove it and continue
        it = timer_handles_.erase(it);
    }
}

template <>
void PriorityMemoryStrategy<>::
    get_next_waitable(
        rclcpp::AnyExecutable &any_exec,
        const WeakNodeList &weak_nodes)
{
    auto it = waitable_handles_.begin();
    while (it != waitable_handles_.end())
    {
        auto waitable = *it;
        if (waitable)
        {
            // Find the group for this handle and see if it can be serviced
            auto group = get_group_by_waitable(waitable, weak_nodes);
            if (!group)
            {
                // Group was not found, meaning the waitable is not valid...
                // Remove it from the ready list and continue looking
                it = waitable_handles_.erase(it);
                continue;
            }
            if (!group->can_be_taken_from().load())
            {
                // Group is mutually exclusive and is being used, so skip it for now
                // Leave it to be checked next time, but continue searching
                ++it;
                continue;
            }
            // Otherwise it is safe to set and return the any_exec
            any_exec.waitable = waitable;
            any_exec.callback_group = group;
            any_exec.node_base = get_node_by_group(group, weak_nodes);
            waitable_handles_.erase(it);
            return;
        }
        // Else, the waitable is no longer valid, remove it and continue
        it = waitable_handles_.erase(it);
    }
}