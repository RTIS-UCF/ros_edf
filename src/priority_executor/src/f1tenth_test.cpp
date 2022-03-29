#include "rclcpp/rclcpp.hpp"
#include "simple_timer/rt-sched.hpp"
#include "priority_executor/priority_memory_strategy.hpp"
#include "priority_executor/priority_executor.hpp"
#include "priority_executor/test_nodes.hpp"
#include "priority_executor/default_executor.hpp"
#include <vector>
#include <fstream>
#include <unistd.h>

typedef struct
{
  std::shared_ptr<timed_executor::TimedExecutor> executor;
  std::shared_ptr<PriorityMemoryStrategy<>> strat;

  std::shared_ptr<ROSDefaultExecutor> default_executor;
} executor_strat;

void spin_exec(executor_strat strat, int id, int index)
{
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(id, &cpuset);

  pthread_t current_thread = pthread_self();
  int result;
  if (result = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset))
  {
    std::cout << "problem setting cpu core" << std::endl;
    std::cout << strerror(result) << std::endl;
  }
  sched_param sch_params;

  // experiment: RT threads have priority 99, all others 98
  if (index < 4)
  {
    sch_params.sched_priority = 99;
  }
  else
  {
    sch_params.sched_priority = 98;
  }
  // sch_params.sched_priority = 99 - index;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch_params))
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "spin_rt thread has an error.");
  }
  if (strat.executor != nullptr)
  {
    strat.executor->spin();
  }
  else if (strat.default_executor != nullptr)
  {
    strat.default_executor->spin();
  }
  else
  {
    std::cout << "spin_exec got a executor_strat with null values!" << std::endl;
  }
}

int main(int argc, char **argv)
{
  // read parameters
  rclcpp::init(argc, argv);
  std::cout << "starting..." << std::endl;
  auto node = rclcpp::Node::make_shared("experiment_parameters");
  node->declare_parameter("experiment_name");
  node->declare_parameter("count_max");
  node->declare_parameter("schedule_type");

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  // parameters_client->wait_for_service();
  const std::string schedule_type_str = parameters_client->get_parameter("schedule_type", std::string("deadline"));
  std::cout << schedule_type_str << std::endl;
  int COUNT_MAX = parameters_client->get_parameter("count_max", 500);
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

  // create executors
  std::vector<executor_strat> executors;
  const int NUM_EXECUTORS = 8;
  std::cout << "creating executors" << std::endl;
  for (int i = 0; i < NUM_EXECUTORS; i++)
  {
    executor_strat executor;
    if (schedule_type == DEFAULT)
    {
      executor.default_executor = std::make_shared<ROSDefaultExecutor>();
      executor.default_executor->logger = create_logger();
    }
    else
    {

      executor.strat = std::make_shared<PriorityMemoryStrategy<>>();
      rclcpp::ExecutorOptions options;
      options.memory_strategy = executor.strat;
      executor.strat->logger = create_logger();
      executor.strat->is_f1tenth = true;

      executor.executor = std::make_shared<timed_executor::TimedExecutor>(options);
      executor.executor->set_use_priorities(true);
    }
    executors.push_back(executor);
  }
  std::cout << "executors created" << std::endl;

  std::vector<uint64_t> chain_lengths = {2, 4, 4, 3, 4, 2, 2, 2, 2, 2, 2, 2};
  std::vector<std::vector<uint64_t>> chain_member_ids = {{1, 2}, {1, 3, 4, 5}, {6, 7, 8, 9}, {10, 11, 12}, {13, 14, 15, 16}, {17, 18}, {19, 20}, {21, 22}, {23, 24}, {25, 26}, {27, 28}, {29, 30}};
  std::vector<std::vector<uint64_t>> chain_priorities = {{1, 0}, {5, 4, 3, 2, 1}, {9, 8, 7, 6}, {12, 11, 10}, {16, 15, 14, 13}, {18, 17}, {20, 19}, {22, 21}, {24, 23}, {26, 25}, {28, 27}, {30, 29}};
  // assignments for ROS and EDF
  std::vector<std::vector<uint64_t>> node_executor_assignment = {{0, 0}, {0, 1, 1, 1}, {2, 2, 2, 2}, {3, 3, 3}, {0, 0, 0, 0}, {1, 1}, {4, 4}, {5, 5}, {6, 6}, {7, 7}, {4, 4}, {5, 5}};
  std::vector<uint64_t> executor_cpu_assignment = {0, 1, 2, 3, 0, 1, 2, 3};
  std::vector<double_t> node_runtimes = {2.3, 16.1, 2.3, 2.2, 18.4, 9.1, 23.1, 7.9, 14.2, 17.9, 20.6, 17.9, 6.6, 1.7, 11.0, 6.6, 7.9, 1.7, 195.9, 33.2, 2.2, 33.2, 2.2, 33.2, 2.2, 33.2, 2.2, 33.2, 2.2, 33.2, 2.2};
  std::cout << std::to_string(node_runtimes.size()) << std::endl;
  std::vector<uint64_t> chain_periods = {80, 80, 100, 100, 160, 1000, 120, 120, 120, 120, 120, 120};
  std::vector<uint64_t> chain_deadlines = {80, 80, 100, 100, 160, 1000, 120, 120, 120, 120, 120, 120};

  std::vector<std::vector<std::shared_ptr<rclcpp::Node>>> nodes;
  std::vector<std::shared_ptr<PublisherNode>> publishers;
  std::vector<std::shared_ptr<DummyWorker>> workers;
  std::vector<std::deque<uint> *> chain_deadlines_deque;
  // create nodes and assign to executors
  uint64_t current_node_id = 0;
  for (uint chain_index = 0; chain_index < chain_lengths.size(); chain_index++)
  {
    std::cout << "making chain " << std::to_string(chain_index) << std::endl;
    std::shared_ptr<rclcpp::TimerBase> this_chain_timer_handle;
    std::deque<uint> *this_chain_deadlines_deque = new std::deque<uint>();
    nodes.push_back(std::vector<std::shared_ptr<rclcpp::Node>>());
    for (uint cb_index = 0; cb_index < chain_lengths[chain_index]; cb_index++)
    {
      std::cout << "making node " << std::to_string(current_node_id) << " with runtime " << node_runtimes[current_node_id] << std::endl;

      executor_strat this_executor = executors[node_executor_assignment[chain_index][cb_index] % NUM_EXECUTORS];
      if (cb_index == 0)
      {
        // this should be a timer
        std::shared_ptr<PublisherNode> publisher_node;
        if (chain_index == 1)
        {
          // special case, re-use timer from index 0
          publisher_node = std::static_pointer_cast<PublisherNode>(nodes[0][0]);
          this_chain_timer_handle = publisher_node->timer_;
          // current_node_id--;
        }
        else
        {
          publisher_node = std::make_shared<PublisherNode>("topic_" + std::to_string(chain_index), chain_index, chain_periods[chain_index], node_runtimes[current_node_id]);
          publishers.push_back(publisher_node);
          publisher_node->count_max = COUNT_MAX;
          if (schedule_type == DEADLINE)
          {
            assert(this_executor.strat != nullptr);
            auto timer_handle = publisher_node->timer_->get_timer_handle();
            assert(timer_handle != nullptr);
            this_executor.strat->set_executable_deadline(publisher_node->timer_->get_timer_handle(), chain_deadlines[chain_index], TIMER, chain_index);
          }
          else if (schedule_type == CHAIN_AWARE_PRIORITY)
          {
            this_executor.strat->set_executable_priority(publisher_node->timer_->get_timer_handle(), chain_priorities[chain_index][cb_index], TIMER, CHAIN_AWARE_PRIORITY, chain_index);
          }

          if (schedule_type == DEFAULT)
          {
            this_executor.default_executor->add_node(publisher_node);
            PriorityExecutable e;
            e.chain_id = chain_index;
            e.timer_handle = publisher_node->timer_;
            this_executor.default_executor->priority_map[publisher_node->timer_->get_timer_handle()] = e;
          }
          else
          {
            this_executor.strat->set_first_in_chain(publisher_node->timer_->get_timer_handle());
            this_executor.strat->assign_deadlines_queue(publisher_node->timer_->get_timer_handle(), this_chain_deadlines_deque);
            this_chain_timer_handle = publisher_node->timer_;
            this_executor.strat->get_priority_settings(publisher_node->timer_->get_timer_handle())->timer_handle = this_chain_timer_handle;
            this_executor.executor->add_node(publisher_node);
          }
        }
        nodes[chain_index].push_back(std::static_pointer_cast<rclcpp::Node>(publisher_node));
      }
      else
      {
        // this is a worker node
        std::shared_ptr<DummyWorker> sub_node;

        if (chain_index == 1 && cb_index == 1)
        {
          sub_node = std::make_shared<DummyWorker>("chain_" + std::to_string(chain_index) + "_worker_" + std::to_string(cb_index), node_runtimes[current_node_id], chain_index, cb_index, true);
        }
        else
        {
          sub_node = std::make_shared<DummyWorker>("chain_" + std::to_string(chain_index) + "_worker_" + std::to_string(cb_index), node_runtimes[current_node_id], chain_index, cb_index);
        }
        workers.push_back(sub_node);
        if (schedule_type == DEADLINE)
        {
          this_executor.strat->set_executable_deadline(sub_node->subscription_->get_subscription_handle(), chain_deadlines[chain_index], SUBSCRIPTION, chain_index);
        }
        else if (schedule_type == CHAIN_AWARE_PRIORITY)
        {
          this_executor.strat->set_executable_priority(sub_node->subscription_->get_subscription_handle(), chain_priorities[chain_index][cb_index], SUBSCRIPTION, CHAIN_AWARE_PRIORITY, chain_index);
        }

        if (schedule_type == DEFAULT)
        {
          this_executor.default_executor->add_node(sub_node);
        }
        else
        {
          this_executor.executor->add_node(sub_node);
          this_executor.strat->assign_deadlines_queue(sub_node->subscription_->get_subscription_handle(), this_chain_deadlines_deque);
          if (cb_index == chain_lengths[chain_index] - 1)
          {
            this_executor.strat->set_last_in_chain(sub_node->subscription_->get_subscription_handle());
            this_executor.strat->get_priority_settings(sub_node->subscription_->get_subscription_handle())->timer_handle = this_chain_timer_handle;
          }
        }
        nodes[chain_index].push_back(std::static_pointer_cast<rclcpp::Node>(sub_node));
      }
      current_node_id++;
    }
    chain_deadlines_deque.push_back(this_chain_deadlines_deque);
  }
  std::cout << "initialized nodes" << std::endl;
  node_time_logger logger = create_logger();
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);
  uint64_t millis = (current_time.tv_sec * (uint64_t)1000) + (current_time.tv_nsec / 1000000);
  for (uint chain_index = 0; chain_index < chain_lengths.size(); chain_index++)
  {
    if (schedule_type == DEADLINE)
    {

      log_entry(logger, "deadline_" + std::to_string(chain_index) + "_" + std::to_string(millis + chain_deadlines[chain_index]));
    }
    log_entry(logger, "timer_" + std::to_string(chain_index) + "_release_" + std::to_string(millis));
    chain_deadlines_deque[chain_index]->push_back(millis + chain_deadlines[chain_index]);
    // chain_deadlines_deque[chain_index]->push_back(0);
  }

  std::vector<std::thread> threads;
  // start each executor on it's own thread
  for (int i = 0; i < NUM_EXECUTORS; i++)
  {
    executor_strat strat = executors[i];
    auto func = std::bind(spin_exec, strat, executor_cpu_assignment[i], i);
    threads.emplace_back(func);
  }
  for (auto &thread : threads)
  {
    thread.join();
  }
  rclcpp::shutdown();

  std::ofstream output_file;
  std::string suffix = "_rtis_alloc";
  if (schedule_type == DEADLINE)
  {
    output_file.open("experiments/results/f1tenth_full" + std::to_string(NUM_EXECUTORS) + "c" + suffix + ".txt");
  }
  else if (schedule_type == CHAIN_AWARE_PRIORITY)
  {
    output_file.open("experiments/results/f1tenth_full_chain" + std::to_string(NUM_EXECUTORS) + "c" + suffix + ".txt");
  }
  else
  {
    output_file.open("experiments/results/f1tenth_default" + std::to_string(NUM_EXECUTORS) + "c" + suffix + ".txt");
  }

  std::vector<std::pair<std::string, u64>> combined_logs;
  for (auto &publisher : publishers)
  {
    for (auto &log : *(publisher->logger_.recorded_times))
    {
      combined_logs.push_back(log);
    }
  }

  for (auto &worker : workers)
  {
    for (auto &log : *(worker->logger_.recorded_times))
    {
      combined_logs.push_back(log);
    }
  }

  if (schedule_type == DEFAULT)
  {
    for (auto &executor : executors)
    {
      for (auto &log : *(executor.default_executor->logger.recorded_times))
      {
        combined_logs.push_back(log);
      }
    }
  }
  else
  {
    for (auto &executor : executors)
    {
      for (auto &log : *(executor.strat->logger.recorded_times))
      {
        combined_logs.push_back(log);
      }
    }
  }

  std::sort(combined_logs.begin(), combined_logs.end(), [](const std::pair<std::string, u64> &a, const std::pair<std::string, u64> &b)
            { return a.second < b.second; });

  for (auto p : combined_logs)
  {
    output_file << p.second << " " << p.first << std::endl;
  }
  output_file.close();
  std::cout<<"data written"<<std::endl;
}
