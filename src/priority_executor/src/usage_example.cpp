#include "rclcpp/rclcpp.hpp"
#include "priority_executor/priority_executor.hpp"
#include "priority_executor/priority_memory_strategy.hpp"
#include "priority_executor/test_nodes.hpp"
#include <string>
#include <fstream>
#include "simple_timer/rt-sched.hpp"
#include "priority_executor/default_executor.hpp"
#include <unistd.h>

// re-create the classic talker-listener example with two listeners
class Talker : public rclcpp::Node
{
public:
    Talker() : Node("talker")
    {
        // Create a publisher on the "chatter" topic with 10 msg queue size.
        pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        // Create a timer of period 1s that calls our callback member function.
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Talker::timer_callback, this));
    }
    // the timer must be public
    rclcpp::TimerBase::SharedPtr timer_;

private:
    void timer_callback()
    {
        // Create a message and publish it 10 times.
        std_msgs::msg::String msg;
        msg.data = "Hello World!";
        for (int i = 0; i < 10; ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
            pub_->publish(msg);
        }
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class Listener : public rclcpp::Node
{
public:
    Listener(std::string name) : Node(name)
    {
        // Create a subscription on the "chatter" topic with the default callback method.
        sub_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&Listener::callback, this, std::placeholders::_1));
    }
    // the publisher must be public
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

private:
    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto talker = std::make_shared<Talker>();
    auto listener1 = std::make_shared<Listener>("listener1");
    auto listener2 = std::make_shared<Listener>("listener2");
    rclcpp::ExecutorOptions options;

    auto strategy = std::make_shared<PriorityMemoryStrategy<>>();
    options.memory_strategy = strategy;
    auto executor = new timed_executor::TimedExecutor(options);
    // replace the above line with the following line to use the default executor
    // which will intermix the execution of listener1 and listener2
    // auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(options);

    strategy->set_executable_deadline(talker->timer_->get_timer_handle(), 100, TIMER);
    strategy->set_executable_deadline(listener1->sub_->get_subscription_handle(), 100, SUBSCRIPTION);
    strategy->set_executable_deadline(listener2->sub_->get_subscription_handle(), 50, SUBSCRIPTION);
    executor->add_node(talker);
    executor->add_node(listener1);
    executor->add_node(listener2);

    executor->spin();
}
