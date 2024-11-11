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
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "test_tracetools/mark_process.hpp"

int main(int argc, char * argv[])
{
  test_tracetools::mark_trace_test_process();

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test_service_ping");

  auto client = node->create_client<std_srvs::srv::Empty>("ping");
  RCLCPP_INFO(node->get_logger(), "waiting for service");
  while (!client->wait_for_service(std::chrono::seconds(10))) {
    if (!rclcpp::ok()) {
      return 1;
    }
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result_future = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "sent request");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    client->remove_pending_request(result_future);
    return 1;
  }
  auto result = result_future.get();
  RCLCPP_INFO(node->get_logger(), "got response");
  rclcpp::shutdown();
  return 0;
}
