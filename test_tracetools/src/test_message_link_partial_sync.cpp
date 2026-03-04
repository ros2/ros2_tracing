// Copyright 2026 Raphael van Kempen
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
#include "tracetools/tracetools.h"

#define NODE_NAME "test_message_link_partial_sync"
#define SUB_TOPIC_NAME_1 "ping"
#define SUB_TOPIC_NAME_2 "pong"
#define PUB_TOPIC_NAME "combined"

class MessageLinkPartialNode : public rclcpp::Node
{
public:
  MessageLinkPartialNode(rclcpp::NodeOptions options, bool do_only_one)
  : Node(NODE_NAME, options), do_only_one_(do_only_one)
  {
    sub1_ = this->create_subscription<std_msgs::msg::String>(
      SUB_TOPIC_NAME_1,
      rclcpp::QoS(10).transient_local(),
      std::bind(&MessageLinkPartialNode::callback1, this, std::placeholders::_1));
    sub2_ = this->create_subscription<std_msgs::msg::String>(
      SUB_TOPIC_NAME_2,
      rclcpp::QoS(10),
      std::bind(&MessageLinkPartialNode::callback2, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::String>(
      PUB_TOPIC_NAME,
      rclcpp::QoS(10));

    // Manually link sub1_ as additional input to pub_ for tracing. sub2_ is
    // directly traced via publication during callback function.
    std::vector<const void *> link_subs = {
      static_cast<const void *>(sub1_->get_subscription_handle().get())
    };
    std::vector<const void *> link_pubs = {
      static_cast<const void *>(pub_->get_publisher_handle().get())
    };
    TRACETOOLS_TRACEPOINT(
      message_link_partial_sync, link_subs.data(), link_subs.size(), link_pubs.data(),
      link_pubs.size()
    );
  }

  explicit MessageLinkPartialNode(rclcpp::NodeOptions options)
  : MessageLinkPartialNode(options, true) {}

private:
  void callback1(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[sub1] %s", msg->data.c_str());
    sub1_message = msg;
  }

  void callback2(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[sub2] %s", msg->data.c_str());
    if(!sub1_message) {
      RCLCPP_WARN(this->get_logger(), "Did not receive message from sub1 yet.");
      return;
    }
    auto next_msg = std::make_shared<std_msgs::msg::String>();
    next_msg->data = "combined string including both: " +
      sub1_message->data + " and: " + msg->data;
    RCLCPP_INFO(this->get_logger(), "%s", next_msg->data.c_str());
    pub_->publish(*next_msg);
    if (do_only_one_) {
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  bool do_only_one_;
  std_msgs::msg::String::ConstSharedPtr sub1_message = nullptr;
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
  auto partial_node =
    std::make_shared<MessageLinkPartialNode>(rclcpp::NodeOptions(), do_only_one);
  exec.add_node(partial_node);

  printf("spinning\n");
  exec.spin();

  // Will actually be called inside the node's callback
  rclcpp::shutdown();
  return 0;
}
