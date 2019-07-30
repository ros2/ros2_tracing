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

#include "tracetools/tracetools.h"

#ifndef TRACETOOLS_DISABLED

#if defined(TRACETOOLS_LTTNG_ENABLED)
# include "tracetools/tp_call.h"
# define CONDITIONAL_TP(...) \
  tracepoint(__VA_ARGS__)
#else
# define CONDITIONAL_TP(...)
#endif

bool ros_trace_compile_status()
{
#if defined(TRACETOOLS_LTTNG_ENABLED)
  return true;
#else
  return false;
#endif
}

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

void TRACEPOINT(
  rcl_init,
  const void * context_handle)
{
  CONDITIONAL_TP(
    ros2,
    rcl_init,
    context_handle,
    tracetools_VERSION);
}

void TRACEPOINT(
  rcl_node_init,
  const void * node_handle,
  const void * rmw_handle,
  const char * node_name,
  const char * node_namespace)
{
  CONDITIONAL_TP(
    ros2,
    rcl_node_init,
    node_handle,
    rmw_handle,
    node_name,
    node_namespace);
}

void TRACEPOINT(
  rcl_publisher_init,
  const void * publisher_handle,
  const void * node_handle,
  const void * rmw_publisher_handle,
  const char * topic_name,
  const size_t queue_depth)
{
  CONDITIONAL_TP(
    ros2,
    rcl_publisher_init,
    publisher_handle,
    node_handle,
    rmw_publisher_handle,
    topic_name,
    queue_depth);
}

void TRACEPOINT(
  rcl_subscription_init,
  const void * subscription_handle,
  const void * node_handle,
  const void * rmw_subscription_handle,
  const char * topic_name,
  const size_t queue_depth)
{
  CONDITIONAL_TP(
    ros2,
    rcl_subscription_init,
    subscription_handle,
    node_handle,
    rmw_subscription_handle,
    topic_name,
    queue_depth);
}

void TRACEPOINT(
  rclcpp_subscription_callback_added,
  const void * subscription_handle,
  const void * callback)
{
  CONDITIONAL_TP(
    ros2,
    rclcpp_subscription_callback_added,
    subscription_handle,
    callback);
}

void TRACEPOINT(
  rcl_service_init,
  const void * service_handle,
  const void * node_handle,
  const void * rmw_service_handle,
  const char * service_name)
{
  CONDITIONAL_TP(
    ros2,
    rcl_service_init,
    service_handle,
    node_handle,
    rmw_service_handle,
    service_name);
}

void TRACEPOINT(
  rclcpp_service_callback_added,
  const void * service_handle,
  const void * callback)
{
  CONDITIONAL_TP(
    ros2,
    rclcpp_service_callback_added,
    service_handle,
    callback);
}

void TRACEPOINT(
  rcl_client_init,
  const void * client_handle,
  const void * node_handle,
  const void * rmw_client_handle,
  const char * service_name)
{
  CONDITIONAL_TP(
    ros2,
    rcl_client_init,
    client_handle,
    node_handle,
    rmw_client_handle,
    service_name);
}

void TRACEPOINT(
  rcl_timer_init,
  const void * timer_handle,
  int64_t period)
{
  CONDITIONAL_TP(
    ros2,
    rcl_timer_init,
    timer_handle,
    period);
}

void TRACEPOINT(
  rclcpp_timer_callback_added,
  const void * timer_handle,
  const void * callback)
{
  CONDITIONAL_TP(
    ros2,
    rclcpp_timer_callback_added,
    timer_handle,
    callback);
}

void TRACEPOINT(
  rclcpp_callback_register,
  const void * callback,
  const char * function_symbol)
{
  CONDITIONAL_TP(
    ros2,
    rclcpp_callback_register,
    callback,
    function_symbol);
}

void TRACEPOINT(
  callback_start,
  const void * callback,
  const bool is_intra_process)
{
  CONDITIONAL_TP(
    ros2,
    callback_start,
    callback,
    (is_intra_process ? 1 : 0));
}

void TRACEPOINT(
  callback_end,
  const void * callback)
{
  CONDITIONAL_TP(
    ros2,
    callback_end,
    callback);
}

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#endif  // TRACETOOLS_DISABLED
