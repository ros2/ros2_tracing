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

#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/string.hpp"
#include "test_tracetools/mark_process.hpp"

using namespace std::chrono_literals;

#define NODE_NAME "test_pong"
#define SUB_TOPIC_NAME "ping"
#define PUB_TOPIC_NAME "pong"
// In the default mode, give up and exit with an error if the exchange isn't done by then
#define EXCHANGE_TIMEOUT 30s
// Give up and exit with an error if test_ping's subscription isn't matched by then
#define MATCH_TIMEOUT 10s

class PongNode : public rclcpp::Node
{
public:
  PongNode(rclcpp::NodeOptions options, bool do_only_one)
  : Node(NODE_NAME, options), do_only_one_(do_only_one)
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      SUB_TOPIC_NAME,
      rclcpp::QoS(10).transient_local(),
      std::bind(&PongNode::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::String>(
      PUB_TOPIC_NAME,
      rclcpp::QoS(10));
    if (do_only_one_) {
      // If we exit right after replying, the reply might not be delivered, so instead exit
      // once test_ping has received the reply and gone away.
      timer_ = this->create_wall_timer(
        100ms,
        std::bind(&PongNode::timer_callback, this));
      deadline_timer_ = this->create_wall_timer(
        EXCHANGE_TIMEOUT,
        [this]() {fail("timed out waiting for the exchange with test_ping to finish");});
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

  void callback(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[output] %s", msg->data.c_str());
    auto next_msg = std::make_shared<std_msgs::msg::String>();
    next_msg->data = "some random pong string";
    // The pong publisher is volatile, so make sure test_ping's subscription is matched before
    // replying; otherwise the reply could be lost and test_ping would wait forever
    const auto match_deadline = std::chrono::steady_clock::now() + MATCH_TIMEOUT;
    while (rclcpp::ok() && 0u == pub_->get_subscription_count()) {
      if (std::chrono::steady_clock::now() >= match_deadline) {
        fail("timed out waiting for test_ping's subscription to match");
        return;
      }
      std::this_thread::sleep_for(10ms);
    }
    RCLCPP_INFO(this->get_logger(), "pong");
    pub_->publish(*next_msg);
    got_ping_ = true;
  }

  void timer_callback()
  {
    if (got_ping_ && 0u == sub_->get_publisher_count()) {
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
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
