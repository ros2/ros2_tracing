// Copyright 2019 Robert Bosch GmbH
// Copyright 2026 Open Source Robotics Foundation, Inc.
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
#include <cstring>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "test_tracetools/mark_process.hpp"

using namespace std::chrono_literals;

namespace
{
using Msg = std_msgs::msg::UInt32;
}  // namespace

#define NODE_NAME "test_ping_loaned"
#define SUB_TOPIC_NAME "pong"
#define PUB_TOPIC_NAME "ping"
#define QUEUE_DEPTH 10

class PingLoanedNode : public rclcpp::Node
{
public:
  PingLoanedNode(rclcpp::NodeOptions options, bool do_only_one)
  : Node(NODE_NAME, options), do_only_one_(do_only_one)
  {
    sub_ = this->create_subscription<Msg>(
      SUB_TOPIC_NAME,
      rclcpp::QoS(QUEUE_DEPTH),
      std::bind(&PingLoanedNode::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<Msg>(
      PUB_TOPIC_NAME,
      rclcpp::QoS(QUEUE_DEPTH).transient_local());

    if (!sub_->can_loan_messages()) {
      throw std::runtime_error(
              "message loaning is not available (subscription cannot loan messages)");
    }

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&PingLoanedNode::timer_callback, this));
  }

  explicit PingLoanedNode(rclcpp::NodeOptions options)
  : PingLoanedNode(options, true) {}

private:
  void callback(const Msg::ConstSharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[output] %u", msg->data);
    if (do_only_one_) {
      rclcpp::shutdown();
    }
  }

  void timer_callback()
  {
    // If ping publishes before pong exists, the first message can be dropped and both nodes
    // can wait forever. Only publish once a subscriber is matched.
    if (pub_->get_subscription_count() == 0) {
      return;
    }
    auto out = pub_->borrow_loaned_message();
    out.get().data = 1;
    RCLCPP_INFO(this->get_logger(), "ping");
    pub_->publish(std::move(out));
  }

  rclcpp::Subscription<Msg>::SharedPtr sub_;
  rclcpp::Publisher<Msg>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
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
  auto ping_node = std::make_shared<PingLoanedNode>(rclcpp::NodeOptions(), do_only_one);
  exec.add_node(ping_node);

  printf("spinning\n");
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
