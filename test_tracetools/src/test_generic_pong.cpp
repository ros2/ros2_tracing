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

#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/string.hpp"
#include "test_tracetools/mark_process.hpp"

using namespace std::chrono_literals;

#define NODE_NAME "test_generic_pong"
#define SUB_TOPIC_NAME "ping"
#define PUB_TOPIC_NAME "pong"
// In the default mode, give up and exit with an error if the exchange isn't done by then
#define EXCHANGE_TIMEOUT 30s
// In the default mode, give up and exit with an error if test_generic_ping's subscription isn't
// matched by then
#define MATCH_TIMEOUT 10s

class PongNode : public rclcpp::Node
{
public:
  PongNode(rclcpp::NodeOptions options, bool do_only_one)
  : Node(NODE_NAME, options), do_only_one_(do_only_one)
  {
    sub_ = this->create_generic_subscription(
      SUB_TOPIC_NAME,
      "std_msgs/msg/String",
      rclcpp::QoS(10).transient_local(),
      std::bind(&PongNode::callback, this, std::placeholders::_1));
    pub_ = this->create_generic_publisher(
      PUB_TOPIC_NAME,
      "std_msgs/msg/String",
      rclcpp::QoS(10));
    serializer_ = std::make_shared<rclcpp::Serialization<std_msgs::msg::String>>();
    if (do_only_one_) {
      // If we exit right after replying, the reply might not be delivered, so instead exit
      // once test_generic_ping has received the reply and gone away.
      timer_ = this->create_wall_timer(
        100ms,
        std::bind(&PongNode::timer_callback, this));
      deadline_timer_ = this->create_wall_timer(
        EXCHANGE_TIMEOUT,
        [this]() {fail("timed out waiting for the exchange with test_generic_ping to finish");});
    }
  }

  explicit PongNode(rclcpp::NodeOptions options)
  : PongNode(options, true) {}

  bool failed() const {return failed_;}

private:
  void fail(const char * reason)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", reason);
    failed_ = true;
    rclcpp::shutdown();
  }

  void callback(std::shared_ptr<const rclcpp::SerializedMessage> msg)
  {
    std_msgs::msg::String deserialized_msg;
    serializer_->deserialize_message(msg.get(), &deserialized_msg);
    RCLCPP_INFO(this->get_logger(), "[output] %s", deserialized_msg.data.c_str());
    std_msgs::msg::String next_msg;
    next_msg.data = "some random pong string";
    rclcpp::SerializedMessage serialized_msg;
    serialized_msg.reserve(1024);
    serializer_->serialize_message(&next_msg, &serialized_msg);
    if (do_only_one_) {
      // The pong publisher is volatile, so make sure test_generic_ping's subscription is matched
      // before replying; otherwise the reply could be lost and test_generic_ping would wait
      // forever. In do_more mode, a lost reply doesn't matter, since the next ping will trigger
      // another one.
      const auto match_deadline = std::chrono::steady_clock::now() + MATCH_TIMEOUT;
      while (rclcpp::ok() && 0u == pub_->get_subscription_count()) {
        if (std::chrono::steady_clock::now() >= match_deadline) {
          fail("timed out waiting for test_generic_ping's subscription to match");
          return;
        }
        std::this_thread::sleep_for(10ms);
      }
    }
    RCLCPP_INFO(this->get_logger(), "pong");
    pub_->publish(serialized_msg);
    got_ping_ = true;
  }

  void timer_callback()
  {
    if (got_ping_ && 0u == sub_->get_publisher_count()) {
      rclcpp::shutdown();
    }
  }

  rclcpp::GenericSubscription::SharedPtr sub_;
  rclcpp::GenericPublisher::SharedPtr pub_;
  std::shared_ptr<rclcpp::Serialization<std_msgs::msg::String>> serializer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr deadline_timer_;
  bool do_only_one_;
  bool got_ping_ = false;
  bool failed_ = false;
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
  return pong_node->failed() ? 1 : 0;
}
