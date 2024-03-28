// Copyright 2019 Robert Bosch GmbH
// Copyright 2024 TIER IV, Inc.
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
#include "test_tracetools/mark_process.hpp"

#define NODE_NAME "test_generic_pong"
#define SUB_TOPIC_NAME "ping"
#define PUB_TOPIC_NAME "pong"

class PongNode : public rclcpp::Node
{
public:
  PongNode(rclcpp::NodeOptions options, bool do_only_one)
  : Node(NODE_NAME, options), do_only_one_(do_only_one)
  {
    sub_ = this->create_generic_subscription(
      SUB_TOPIC_NAME,
      "std_msgs/msg/String",
      rclcpp::QoS(10),
      std::bind(&PongNode::callback, this, std::placeholders::_1));
    pub_ = this->create_generic_publisher(
      PUB_TOPIC_NAME,
      "std_msgs/msg/String",
      rclcpp::QoS(10));
    serializer_ = std::make_shared<rclcpp::Serialization<std_msgs::msg::String>>();
  }

  explicit PongNode(rclcpp::NodeOptions options)
  : PongNode(options, true) {}

private:
  void callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    std_msgs::msg::String deserialized_msg;
    serializer_->deserialize_message(msg.get(), &deserialized_msg);
    RCLCPP_INFO(this->get_logger(), "[output] %s", deserialized_msg.data.c_str());
    std_msgs::msg::String next_msg;
    next_msg.data = "some random pong string";
    rclcpp::SerializedMessage serialized_msg;
    serialized_msg.reserve(1024);
    serializer_->serialize_message(&next_msg, &serialized_msg);
    pub_->publish(serialized_msg);
    if (do_only_one_) {
      rclcpp::shutdown();
    }
  }

  rclcpp::GenericSubscription::SharedPtr sub_;
  rclcpp::GenericPublisher::SharedPtr pub_;
  std::shared_ptr<rclcpp::Serialization<std_msgs::msg::String>> serializer_;
  bool do_only_one_;
};

int main(int argc, char * argv[])
{
  test_tracetools::mark_trace_test_process();

  bool do_only_one = true;
  for (int i = 0; i < argc; ++i) {
    if (strncmp(argv[i], "do_more", 7) == 0) {
      do_only_one = false;
    }
  }

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto pong_node = std::make_shared<PongNode>(rclcpp::NodeOptions(), do_only_one);
  exec.add_node(pong_node);

  printf("spinning\n");
  exec.spin();

  // Will actually be called inside the node's callback
  rclcpp::shutdown();
  return 0;
}
