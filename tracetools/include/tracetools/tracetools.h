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

#ifndef TRACETOOLS__TRACETOOLS_H_
#define TRACETOOLS__TRACETOOLS_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "tracetools/config.h"
#include "tracetools/visibility_control.hpp"

#ifndef TRACETOOLS_DISABLED
#  define TRACEPOINT(event_name, ...) \
  (ros_trace_ ## event_name)(__VA_ARGS__)
#  define DECLARE_TRACEPOINT(event_name, ...) \
  TRACETOOLS_PUBLIC void(ros_trace_ ## event_name)(__VA_ARGS__);
#else
#  define TRACEPOINT(event_name, ...)
#  define DECLARE_TRACEPOINT(event_name, ...)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * Report whether tracing is compiled in
 */
TRACETOOLS_PUBLIC bool ros_trace_compile_status();

/**
 * tp: rcl_init
 */
DECLARE_TRACEPOINT(
  rcl_init,
  const void * context_handle)

/**
 * tp: rcl_node_init
 */
DECLARE_TRACEPOINT(
  rcl_node_init,
  const void * node_handle,
  const void * rmw_handle,
  const char * node_name,
  const char * node_namespace)

/**
 * tp: rcl_publisher_init
 */
DECLARE_TRACEPOINT(
  rcl_publisher_init,
  const void * publisher_handle,
  const void * node_handle,
  const void * rmw_publisher_handle,
  const char * topic_name,
  const size_t queue_depth)

/**
 * tp: rcl_subscription_init
 */
DECLARE_TRACEPOINT(
  rcl_subscription_init,
  const void * subscription_handle,
  const void * node_handle,
  const void * rmw_subscription_handle,
  const char * topic_name,
  const size_t queue_depth)

/**
 * tp: rclcpp_subscription_callback_added
 */
DECLARE_TRACEPOINT(
  rclcpp_subscription_callback_added,
  const void * subscription_handle,
  const void * callback)

/**
 * tp: rcl_service_init
 */
DECLARE_TRACEPOINT(
  rcl_service_init,
  const void * service_handle,
  const void * node_handle,
  const void * rmw_service_handle,
  const char * service_name)

/**
 * tp: rclcpp_service_callback_added
 */
DECLARE_TRACEPOINT(
  rclcpp_service_callback_added,
  const void * service_handle,
  const void * callback)

/**
 * tp: rcl_client_init
 */
DECLARE_TRACEPOINT(
  rcl_client_init,
  const void * client_handle,
  const void * node_handle,
  const void * rmw_client_handle,
  const char * service_name)

/**
 * tp: rcl_timer_init
 */
DECLARE_TRACEPOINT(
  rcl_timer_init,
  const void * timer_handle,
  int64_t period)

/**
 * tp: rclcpp_timer_callback_added
 */
DECLARE_TRACEPOINT(
  rclcpp_timer_callback_added,
  const void * timer_handle,
  const void * callback)

/**
 * tp: rclcpp_callback_register
 */
DECLARE_TRACEPOINT(
  rclcpp_callback_register,
  const void * callback,
  const char * function_symbol)

/**
 * tp: callback_start
 */
DECLARE_TRACEPOINT(
  callback_start,
  const void * callback,
  const bool is_intra_process)

/**
 * tp: callback_end
 */
DECLARE_TRACEPOINT(
  callback_end,
  const void * callback)

#ifdef __cplusplus
}
#endif

#endif  // TRACETOOLS__TRACETOOLS_H_
