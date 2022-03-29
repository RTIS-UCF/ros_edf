#include "priority_executor/test_nodes.hpp"
#include "priority_executor/primes_workload.hpp"
#include "simple_timer/rt-sched.hpp"
#include <string>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
PublisherNode::PublisherNode(std::string publish_topic, int chain, int period, double runtime)
    : Node("PublisherNode_" + publish_topic), count_(0)
{
  logger_ = create_logger();
  publisher_ = this->create_publisher<std_msgs::msg::String>(publish_topic, 1);
  this->chain = chain;
  this->runtime = runtime;
  this->period = period;
  std::cout << "creating timer "
            << publish_topic << std::endl;
  auto timer_callback =
      [this]() -> void
  {
    if (this->count_ > this->count_max)
    {
      rclcpp::shutdown();
      return;
    }
    this->logger_.recorded_times->push_back(std::make_pair(std::string(this->get_name()) + "_publish_" + std::to_string(this->count_), get_time_us()));
    this->logger_.recorded_times->push_back(std::make_pair("chain_" + std::to_string(this->chain) + "_worker_0_recv_MESSAGE" + std::to_string(this->count_), get_time_us()));

    double result = nth_prime_silly(100000, this->runtime);

    this->logger_.recorded_times->push_back(std::make_pair("chain_" + std::to_string(this->chain) + "_worker_0_processed_MESSAGE" + std::to_string(this->count_), get_time_us()));
    auto message = std_msgs::msg::String();
    message.data = "MESSAGE" + std::to_string(this->count_++);
    this->publisher_->publish(message);

    // RCLCPP_INFO(this->get_logger(), "I did work on: '%s', taking %lf ms", message.data.c_str(), result);
    // usleep(600 * 1000);
  };
  timer_ = this->create_wall_timer(std::chrono::milliseconds(period), timer_callback);
}

DummyWorker::DummyWorker(const std::string &name, double runtime, int chain, int number, bool is_multichain)
    : Node(name)
{
  this->runtime = runtime;
  this->number = number;
  this->chain = chain;
  this->logger_ = create_logger();
  std::cout << "creating dummy worker "
            << name << std::endl;
  if (is_multichain)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic_" + std::to_string(chain - 1), 1, std::bind(&DummyWorker::topic_callback, this, _1));
  }
  else if (number == 1)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic_" + std::to_string(chain), 1, std::bind(&DummyWorker::topic_callback, this, _1));
  }
  else
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "chain_" + std::to_string(chain) + "_topic_" + std::to_string(number), 1, std::bind(&DummyWorker::topic_callback, this, _1));
  }
  this->publisher_ = this->create_publisher<std_msgs::msg::String>("chain_" + std::to_string(chain) + "_topic_" + std::to_string(number + 1), 1);
}
void DummyWorker::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
  this->logger_.recorded_times->push_back(std::make_pair(std::string(this->get_name()) + "_recv_" + msg->data, get_time_us()));

  double result = nth_prime_silly(100000, runtime);

  this->logger_.recorded_times->push_back(std::make_pair(std::string(this->get_name()) + "_processed_" + msg->data, get_time_us()));
  // RCLCPP_INFO(this->get_logger(), "I did work on: '%s', taking %lf ms", msg->data.c_str(), result);
  auto message = std_msgs::msg::String();
  message.data = msg->data;
  this->publisher_->publish(message);
  // usleep(600 * 1000);
}
