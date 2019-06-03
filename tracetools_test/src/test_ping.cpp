#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


class PingNode : public rclcpp::Node
{
public:
    PingNode(rclcpp::NodeOptions options) : Node("ping_node", options)
    {
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "pong",
            rclcpp::QoS(10),
            std::bind(&PingNode::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<std_msgs::msg::String>(
            "ping",
            rclcpp::QoS(10));
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&PingNode::timer_callback, this));
    }


private:
    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "[output] %s", msg->data.c_str());
        rclcpp::shutdown();
    }

    void timer_callback()
    {
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = "some random ping string";
        pub_->publish(*msg);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ping_node = std::make_shared<PingNode>(rclcpp::NodeOptions());
    exec.add_node(ping_node);

    printf("spinning\n");
    exec.spin();

    // Will actually be called inside the node's callback
    rclcpp::shutdown();
    return 0;
}
