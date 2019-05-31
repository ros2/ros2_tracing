#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class SubNode : public rclcpp::Node
{
public:
    SubNode(rclcpp::NodeOptions options) : Node("sub_node", options)
    {
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "the_topic",
            rclcpp::QoS(10),
            std::bind(&SubNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Nothing
        (void)msg;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto sub_node = std::make_shared<SubNode>(rclcpp::NodeOptions());
    exec.add_node(sub_node);

    printf("spinning once\n");
    exec.spin_once();

    rclcpp::shutdown();
    return 0;
}
