#include "rclcpp/rclcpp.hpp"
#include "priority_executor/priority_executor.hpp"
#include "priority_executor/priority_memory_strategy.hpp"
#include "priority_executor/test_nodes.hpp"
#include <string>
#include <fstream>
#include "simple_timer/rt-sched.hpp"
#include "priority_executor/default_executor.hpp"
#include <unistd.h>

// clock_t times(struct tms *buf);

std::vector<int64_t> get_parameter_array(std::shared_ptr<rclcpp::Node> node, std::string name, std::vector<int64_t> default_val)
{
  rclcpp::Parameter param_result(name, default_val);
  node->get_parameter_or(name, param_result, param_result);
  return param_result.as_integer_array();
}

int main(int argc, char **argv)
{
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(0, &cpuset);

  pthread_t current_thread = pthread_self();
  if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset))
  {
    std::cout << "problem setting cpu core" << std::endl;
  }
  sched_param sch_params;
  sch_params.sched_priority = 98;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch_params))
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "spin_rt thread has an error.");
  }
  rclcpp::init(argc, argv);

  // https://design.ros2.org/articles/ros_command_line_arguments.html#multiple-parameter-assignments
  auto node = rclcpp::Node::make_shared("experiment_parameters");
  node->declare_parameter("experiment_name");
  node->declare_parameter("count_max");
  node->declare_parameter("schedule_type");

  node->declare_parameter("chain_lengths");
  node->declare_parameter("chain_periods");
  node->declare_parameter("chain_deadlines");
  node->declare_parameter("chain_runtimes");
  node->declare_parameter("chain_priorities");
  node->declare_parameter("chain_timer_runtimes");

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  parameters_client->wait_for_service();
  const std::string schedule_type_str = parameters_client->get_parameter("schedule_type", std::string("deadline"));
  std::cout << schedule_type_str << std::endl;
  ExecutableScheduleType schedule_type = DEFAULT;
  if (schedule_type_str == "deadline")
  {
    schedule_type = DEADLINE;
  }
  else if (schedule_type_str == "chain_priority")
  {
    schedule_type = CHAIN_AWARE_PRIORITY;
  }
  else
  {
    schedule_type = DEFAULT;
  }

  const std::vector<int64_t> chain_lengths = get_parameter_array(node, "chain_lengths", std::vector<int64_t>({3, 7}));

  const std::vector<int64_t> chain_periods = get_parameter_array(node, "chain_periods", std::vector<int64_t>({1000, 1000}));
  const std::vector<int64_t> chain_deadlines = get_parameter_array(node, "chain_deadlines", std::vector<int64_t>({1000, 1000}));
  const std::vector<int64_t> chain_runtimes = get_parameter_array(node, "chain_runtimes", std::vector<int64_t>({131, 131}));
  const std::vector<int64_t> chain_timer_runtimes = get_parameter_array(node, "chain_timer_runtimes", std::vector<int64_t>({109, 109}));

  const std::vector<int64_t> chain_priorities = get_parameter_array(node, "chain_priorities", std::vector<int64_t>({1, 2}));
  const uint NUM_CHAINS = chain_lengths.size();
  if (chain_lengths.size() > chain_periods.size())
  {
    std::cout << "chain_periods shorter than chain_lengths" << std::endl;
    exit(-1);
  }

  if (chain_lengths.size() > chain_runtimes.size())
  {
    std::cout << "chain_runtimes shorter than chain_lengths" << std::endl;
    exit(-1);
  }
  if (chain_lengths.size() > chain_timer_runtimes.size())
  {
    std::cout << "chain_timer_runtimes shorter than chain_lengths" << std::endl;
    exit(-1);
  }
  if (schedule_type == DEADLINE)
  {
    if (chain_lengths.size() > chain_deadlines.size())
    {
      std::cout << "chain_deadlines shorter than chain_lengths" << std::endl;
      exit(-1);
    }
  }
  else if (schedule_type == CHAIN_AWARE_PRIORITY)
  {
    if (chain_lengths.size() > chain_priorities.size())
    {
      std::cout << "chain_priorities shorter than chain_lengths" << std::endl;
      exit(-1);
    }
  }

  const uint COUNT_MAX = parameters_client->get_parameter("count_max", 20);
  const std::string experiment_name = parameters_client->get_parameter("experiment_name", std::string("unnamed_experiment"));

  rclcpp::ExecutorOptions options;
  // use a modified memorystrategy
  std::shared_ptr<PriorityMemoryStrategy<>> strat = std::make_shared<PriorityMemoryStrategy<>>();
  strat->logger = create_logger();
  // publisher
  options.memory_strategy = strat;
  rclcpp::Executor *sub1_executor = nullptr;

  ROSDefaultExecutor *default_executor = nullptr;
  if (schedule_type != DEFAULT)
  {
    timed_executor::TimedExecutor *rtis_executor = new timed_executor::TimedExecutor(options, "short_executor");
    rtis_executor->set_use_priorities(true);
    sub1_executor = rtis_executor;
  }
  else if (schedule_type == DEFAULT)
  {
    default_executor = new ROSDefaultExecutor();
    default_executor->logger = create_logger();
  }
  // stock ROS executor
  // rclcpp::executors::SingleThreadedExecutor sub1_executor;
  std::vector<std::shared_ptr<DummyWorker>> workers;
  std::vector<std::deque<uint> *> chain_deadlines_deque;
  std::vector<std::shared_ptr<PublisherNode>> timers;
  // TODO: make the chain layout configurable via rosparam
  for (uint chain_index = 0; chain_index < NUM_CHAINS; chain_index++)
  {
    std::shared_ptr<rclcpp::TimerBase> this_chain_timer_handle;
    std::deque<uint> *this_chain_deadlines_deque = new std::deque<uint>();
    for (int cb_index = 0; cb_index < chain_lengths[chain_index]; cb_index++)
    {
      int total_prio = 0;
      int this_chain_prio = 0;
      if (schedule_type == CHAIN_AWARE_PRIORITY)
      {
        this_chain_prio = chain_priorities[chain_index];
        total_prio = chain_lengths[chain_index];
        for (uint eval_chain = 0; eval_chain < NUM_CHAINS; eval_chain++)
        {
          if (eval_chain == chain_index)
          {
            continue;
          }
          if (chain_priorities[eval_chain] < this_chain_prio)
          {
            total_prio += chain_lengths[eval_chain] * chain_priorities[eval_chain];
          }
        }
      }
      if (cb_index == 0)
      {
        std::shared_ptr<PublisherNode> pubnode = std::make_shared<PublisherNode>("topic_" + std::to_string(chain_index), chain_index, chain_periods[chain_index], chain_timer_runtimes[chain_index]);
        pubnode->count_max = COUNT_MAX;
        if (schedule_type == DEADLINE)
        {
          strat->set_executable_deadline(pubnode->timer_->get_timer_handle(), chain_deadlines[chain_index], TIMER, chain_index);
        }
        else if (schedule_type == CHAIN_AWARE_PRIORITY)
        {
          std::cout << "creating prio timer on chain " << std::to_string(chain_index) << " with prio " << std::to_string(chain_index) << std::endl;
          strat->set_executable_priority(pubnode->timer_->get_timer_handle(), chain_index, TIMER, CHAIN_AWARE_PRIORITY, chain_index);
        }
        if (schedule_type != DEFAULT)
        {
          strat->set_first_in_chain(pubnode->timer_->get_timer_handle());
          strat->assign_deadlines_queue(pubnode->timer_->get_timer_handle(), this_chain_deadlines_deque);
          strat->get_priority_settings(pubnode->timer_->get_timer_handle())->timer_handle = pubnode->timer_;
          this_chain_timer_handle = pubnode->timer_;
        }
        if (schedule_type == DEFAULT)
        {
          PriorityExecutable e;
          e.chain_id = chain_index;
          e.timer_handle = pubnode->timer_;
          default_executor->priority_map[pubnode->timer_->get_timer_handle()] = e;
          default_executor->add_node(pubnode);
        }
        else
        {
          sub1_executor->add_node(pubnode);
        }
        timers.push_back(pubnode);
      }
      else
      {
        auto sub1node = std::make_shared<DummyWorker>("chain_" + std::to_string(chain_index) + "_worker_" + std::to_string(cb_index), chain_runtimes[chain_index], chain_index, cb_index);
        if (schedule_type == DEFAULT)
        {
          default_executor->add_node(sub1node);
        }
        else
        {
          sub1_executor->add_node(sub1node);
        }
        if (schedule_type == DEADLINE)
        {
          strat->set_executable_deadline(sub1node->subscription_->get_subscription_handle(), chain_deadlines[chain_index], SUBSCRIPTION, chain_index);
        }
        else if (schedule_type == CHAIN_AWARE_PRIORITY)
        {
          std::cout << "creating prio cb with prio " << std::to_string(chain_index) << std::endl;
          strat->set_executable_priority(sub1node->subscription_->get_subscription_handle(), (chain_index), SUBSCRIPTION, CHAIN_AWARE_PRIORITY, chain_index);
        }
        if (schedule_type != DEFAULT)
        {
          strat->assign_deadlines_queue(sub1node->subscription_->get_subscription_handle(), this_chain_deadlines_deque);
          if (cb_index == chain_lengths[chain_index] - 1)
          {
            strat->set_last_in_chain(sub1node->subscription_->get_subscription_handle());
            strat->get_priority_settings(sub1node->subscription_->get_subscription_handle())->timer_handle = this_chain_timer_handle;
          }
        }
        workers.push_back(sub1node);
      }
    }
    chain_deadlines_deque.push_back(this_chain_deadlines_deque);
  }
  std::cout << "initialized nodes" << std::endl;
  node_time_logger logger = create_logger();
  // initialize first deadlines
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);
  uint64_t millis = (current_time.tv_sec * (uint64_t)1000) + (current_time.tv_nsec / 1000000);
  for (uint chain_index = 0; chain_index < NUM_CHAINS; chain_index++)
  {
    log_entry(logger, "deadline_" + std::to_string(chain_index) + "_" + std::to_string(millis + chain_deadlines[chain_index]));
    log_entry(logger, "timer_" + std::to_string(chain_index) + "_release_" + std::to_string(millis));
    chain_deadlines_deque[chain_index]->push_back(millis + chain_deadlines[chain_index]);
    // chain_deadlines_deque[chain_index]->push_back(0);
  }
  if (schedule_type == DEFAULT)
  {
    default_executor->spin();
  }
  else
  {
    sub1_executor->spin();
  }
  rclcpp::shutdown();

  // combine logs from all chains
  std::vector<std::pair<std::string, u64>> combined_logs;
  for (auto &worker : workers)
  {
    for (auto &log : *(worker->logger_.recorded_times))
    {
      combined_logs.push_back(log);
    }
  }
  for (auto &timer : timers)
  {
    for (auto &log : *(timer->logger_.recorded_times))
    {
      combined_logs.push_back(log);
    }
  }
  // add logs from strategy
  for (auto &log : *(strat->logger.recorded_times))
  {
    combined_logs.push_back(log);
  }

  // if using the "default" executor, grab those logs too
  if (schedule_type == DEFAULT)
  {
    for (auto &log : *(default_executor->logger.recorded_times))
    {
      combined_logs.push_back(log);
    }
  }

  // sort logs
  std::sort(combined_logs.begin(), combined_logs.end(), [](const std::pair<std::string, u64> &a, const std::pair<std::string, u64> &b)
            { return a.second < b.second; });

  std::ofstream output_file;
  output_file.open("experiments/results/" + experiment_name + ".txt");
  for (auto p : combined_logs)
  {
    output_file << p.second << " " << p.first << std::endl;
  }
  output_file.close();
}
