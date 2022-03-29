#include "rclcpp/rclcpp.hpp"
#include "simple_timer/rt-sched.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode(std::string publish_topic, int chain, int period, double runtime);

  rclcpp::TimerBase::SharedPtr timer_;
  uint count_max = 20;
  node_time_logger logger_;

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  uint count_;
  int chain;
  double runtime;
  int period;
};
class DummyWorker : public rclcpp::Node
{
public:
  DummyWorker(const std::string &name, double runtime, int chain, int number, bool is_multichain = false);
  node_time_logger logger_;

private:
  double runtime;
  int number;
  int chain;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;

public:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};