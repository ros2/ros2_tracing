// Copyright 2019 Robert Bosch GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define NODE_NAME "test_subscription"
#define TOPIC_NAME "the_topic"
#define QUEUE_DEPTH 10

class SubNode : public rclcpp::Node
{
public:
  explicit SubNode(rclcpp::NodeOptions options)
  : Node(NODE_NAME, options)
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      TOPIC_NAME,
      rclcpp::QoS(QUEUE_DEPTH),
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

int main(int argc, char * argv[])
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
