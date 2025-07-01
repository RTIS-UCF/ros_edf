#include "priority_executor/priority_executor.hpp"
#include "priority_executor/priority_memory_strategy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <fstream>
#include <string>
#include <unistd.h>

// not used by the executor internals, but just convenient for the example
#define CHAIN_0_PERIOD_MS 1000
#define CHAIN_1_PERIOD_MS 500

// re-create the classic talker-listener example with two listeners
class Talker : public rclcpp::Node {
public:
  Talker(std::string name, std::string publish_to, std::chrono::milliseconds period)
      : Node(name) {
        pub_ = this->create_publisher<std_msgs::msg::String>(publish_to, 10);
        timer_ = this->create_wall_timer(
            period, std::bind(&Talker::timer_callback, this));
  }
  // the timer must be public
  rclcpp::TimerBase::SharedPtr timer_;

private:
  void timer_callback() {
    // Create a message and publish it 10 times.
    std_msgs::msg::String msg;
    msg.data = std::string("sending from ") + this->get_name();
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    pub_->publish(msg);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class Listener : public rclcpp::Node {
public:
  Listener(std::string name, std::string subscribe_to, std::string publish_to)
      : Node(name) {
    // Create a subscription on the "chatter" topic with the default callback
    // method.
    sub_ = this->create_subscription<std_msgs::msg::String>(
        subscribe_to, 10,
        std::bind(&Listener::callback, this, std::placeholders::_1));
    // Create a publisher on the "chatter" topic with 10 msg queue size.
    pub_ = this->create_publisher<std_msgs::msg::String>(publish_to, 10);
  }
  // the publisher must be public
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

private:
  void callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    std_msgs::msg::String new_msg;
    new_msg.data = msg->data;
    pub_->publish(new_msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto talker = std::make_shared<Talker>("talker1", "talker1",
                                         std::chrono::milliseconds(CHAIN_0_PERIOD_MS));
  auto listener1 =
      std::make_shared<Listener>("listener1", "talker1", "listener1_out");
  auto listener2 =
      std::make_shared<Listener>("listener2", "listener1_out", "listener2_out");
  rclcpp::ExecutorOptions options;

  auto strategy = std::make_shared<PriorityMemoryStrategy<>>();
  options.memory_strategy = strategy;
  auto executor = new timed_executor::TimedExecutor(options);

  strategy->set_executable_deadline(talker->timer_->get_timer_handle(),
                                    CHAIN_0_PERIOD_MS, TIMER, 0);
  // must label the first callback in the chain
  strategy->set_first_in_chain(talker->timer_->get_timer_handle());
  strategy->set_executable_deadline(listener1->sub_->get_subscription_handle(),
                                    CHAIN_0_PERIOD_MS, SUBSCRIPTION, 0);
  strategy->set_executable_deadline(listener2->sub_->get_subscription_handle(),
                                    CHAIN_0_PERIOD_MS, SUBSCRIPTION, 0);
  // must label the last callback in the chain
  strategy->set_last_in_chain(listener2->sub_->get_subscription_handle());
  // also inform the scheduler which timer to use when calculating the next
  // deadline timer_handle must be set on the last callback in the chain
  strategy->get_priority_settings(listener2->sub_->get_subscription_handle())
      ->timer_handle = talker->timer_;


  executor->add_node(talker);
  executor->add_node(listener1);
  executor->add_node(listener2);


  // same thing for the second chain
  auto talker2 = std::make_shared<Talker>("talker2", "talker2",
                                           std::chrono::milliseconds(CHAIN_1_PERIOD_MS));
    auto listener3 =
      std::make_shared<Listener>("listener3", "talker2", "listener3_out");
    auto listener4 =
      std::make_shared<Listener>("listener4", "listener3_out", "listener4_out");

    // for a separate chain, use a different chain index
    strategy->set_executable_deadline(talker2->timer_->get_timer_handle(),
                                        CHAIN_1_PERIOD_MS, TIMER, 1);
    strategy->set_first_in_chain(talker2->timer_->get_timer_handle());
    strategy->set_executable_deadline(listener3->sub_->get_subscription_handle(),
                                        CHAIN_1_PERIOD_MS, SUBSCRIPTION, 1);
    strategy->set_executable_deadline(listener4->sub_->get_subscription_handle(),
                                        CHAIN_1_PERIOD_MS, SUBSCRIPTION, 1);
    strategy->set_last_in_chain(listener4->sub_->get_subscription_handle());
    strategy->get_priority_settings(listener4->sub_->get_subscription_handle())
        ->timer_handle = talker2->timer_;
    executor->add_node(talker2);
    executor->add_node(listener3);
    executor->add_node(listener4);



  executor->spin();
}
