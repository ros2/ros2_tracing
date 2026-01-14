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
#include "std_srvs/srv/empty.hpp"
#include "test_tracetools/mark_process.hpp"

int main(int argc, char * argv[])
{
  test_tracetools::mark_trace_test_process();

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = rclcpp::Node::make_shared("test_service_pong");

  auto client = node->create_service<std_srvs::srv::Empty>(
    "ping",
    [&](const std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> service,
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request) {
      (void)request;
      RCLCPP_INFO(node->get_logger(), "got request");
      std_srvs::srv::Empty::Response response;
      service->send_response(*request_header, response);
      RCLCPP_INFO(node->get_logger(), "sent response");
      // Stop executor after this callback is done
      exec.cancel();
    });

  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
