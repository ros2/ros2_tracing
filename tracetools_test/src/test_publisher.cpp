#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PubNode : public rclcpp::Node
{
public:
  explicit PubNode(rclcpp::NodeOptions options)
  : Node("pub_node", options)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>(
      "the_topic",
      rclcpp::QoS(10));
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto pub_node = std::make_shared<PubNode>(rclcpp::NodeOptions());
  exec.add_node(pub_node);

  printf("spinning once\n");
  exec.spin_once();

  rclcpp::shutdown();
  return 0;
}
