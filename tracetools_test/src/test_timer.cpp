#include <chrono>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


class TimerNode : public rclcpp::Node
{
public:
    TimerNode(rclcpp::NodeOptions options) : Node("timer_node", options)
    {
        is_done_ = false;
        timer_ = this->create_wall_timer(
            1ms,
            std::bind(&TimerNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        if (is_done_) {
            rclcpp::shutdown();
        } else {
            is_done_ = true;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    bool is_done_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto timer_node = std::make_shared<TimerNode>(rclcpp::NodeOptions());
    exec.add_node(timer_node);

    printf("spinning\n");
    exec.spin();

    // Will actually be called inside the timer's callback
    rclcpp::shutdown();
    return 0;
}
