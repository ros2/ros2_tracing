// Copyright 2019 Robert Bosch GmbH
// Copyright 2020 Christophe Bedard
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

/**
 * TODO(christophebedard) need to use "ros2" directly instead of "TRACEPOINT_PROVIDER" in some
 * places here due to a bug with LTTng. Try again when bumping to a newer LTTng-UST version once
 * this fix gets merged: https://review.lttng.org/c/lttng-ust/+/9001
 */
#ifndef TRACETOOLS_TRACEPOINTS_EXCLUDED
# include "tracetools/tp_call.h"
// *INDENT-OFF*
# define _CONDITIONAL_TP(...) \
  tracepoint(TRACEPOINT_PROVIDER, __VA_ARGS__)
# define _CONDITIONAL_TP_ENABLED(event_name) \
  0 != tracepoint_enabled(ros2, event_name)
# define _CONDITIONAL_DO_TP(...) \
  do_tracepoint(ros2, __VA_ARGS__)
#else
# define _CONDITIONAL_TP(...) ((void) (0))
# define _CONDITIONAL_TP_ENABLED(...) false
# define _CONDITIONAL_DO_TP(...) ((void) (0))
#endif

#define TRACEPOINT_ARGS(...) __VA_ARGS__
#define TRACEPOINT_PARAMS(...) __VA_ARGS__

#define DEFINE_TRACEPOINT(event_name, _TP_PARAMS, _TP_ARGS) \
  void TRACETOOLS_TRACEPOINT(event_name, _TP_PARAMS) \
  { \
    _CONDITIONAL_TP(event_name, _TP_ARGS); \
  } \
  bool TRACETOOLS_TRACEPOINT_ENABLED(event_name) \
  { \
    return _CONDITIONAL_TP_ENABLED(event_name); \
  } \
  void TRACETOOLS_DO_TRACEPOINT(event_name, _TP_PARAMS) \
  { \
    _CONDITIONAL_DO_TP(event_name, _TP_ARGS); \
  }
#define DEFINE_TRACEPOINT_NO_ARGS(event_name) \
  void TRACETOOLS_TRACEPOINT(event_name) \
  { \
    _CONDITIONAL_TP(event_name); \
  } \
  bool TRACETOOLS_TRACEPOINT_ENABLED(event_name) \
  { \
    return _CONDITIONAL_TP_ENABLED(event_name); \
  } \
  void TRACETOOLS_DO_TRACEPOINT(event_name) \
  { \
    _CONDITIONAL_DO_TP(event_name); \
  }
// *INDENT-ON*

bool ros_trace_compile_status()
{
#ifndef TRACETOOLS_TRACEPOINTS_EXCLUDED
  return true;
#else
  return false;
#endif
}

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#else
# pragma warning(push)
# pragma warning(disable: 4100)
#endif

DEFINE_TRACEPOINT(
  rcl_init,
  TRACEPOINT_PARAMS(
    const void * context_handle),
  TRACEPOINT_ARGS(
    context_handle))

DEFINE_TRACEPOINT(
  rcl_node_init,
  TRACEPOINT_PARAMS(
    const void * node_handle,
    const void * rmw_handle,
    const char * node_name,
    const char * node_namespace),
  TRACEPOINT_ARGS(
    node_handle,
    rmw_handle,
    node_name,
    node_namespace))

DEFINE_TRACEPOINT(
  rmw_publisher_init,
  TRACEPOINT_PARAMS(
    const void * rmw_publisher_handle,
    const uint8_t * gid),
  TRACEPOINT_ARGS(
    rmw_publisher_handle,
    gid))

DEFINE_TRACEPOINT(
  rcl_publisher_init,
  TRACEPOINT_PARAMS(
    const void * publisher_handle,
    const void * node_handle,
    const void * rmw_publisher_handle,
    const char * topic_name,
    const size_t queue_depth),
  TRACEPOINT_ARGS(
    publisher_handle,
    node_handle,
    rmw_publisher_handle,
    topic_name,
    queue_depth))

DEFINE_TRACEPOINT(
  rclcpp_publish,
  TRACEPOINT_PARAMS(
    const void * publisher_handle,
    const void * message),
  TRACEPOINT_ARGS(
    message))

DEFINE_TRACEPOINT(
  rclcpp_intra_publish,
  TRACEPOINT_PARAMS(
    const void * publisher_handle,
    const void * message),
  TRACEPOINT_ARGS(
    publisher_handle,
    message))

DEFINE_TRACEPOINT(
  rcl_publish,
  TRACEPOINT_PARAMS(
    const void * publisher_handle,
    const void * message),
  TRACEPOINT_ARGS(
    publisher_handle,
    message))

DEFINE_TRACEPOINT(
  rmw_publish,
  TRACEPOINT_PARAMS(
    const void * rmw_publisher_handle,
    const void * message,
    int64_t timestamp),
  TRACEPOINT_ARGS(
    rmw_publisher_handle,
    message,
    timestamp))

DEFINE_TRACEPOINT(
  rmw_subscription_init,
  TRACEPOINT_PARAMS(
    const void * rmw_subscription_handle,
    const uint8_t * gid),
  TRACEPOINT_ARGS(
    rmw_subscription_handle,
    gid))

DEFINE_TRACEPOINT(
  rcl_subscription_init,
  TRACEPOINT_PARAMS(
    const void * subscription_handle,
    const void * node_handle,
    const void * rmw_subscription_handle,
    const char * topic_name,
    const size_t queue_depth),
  TRACEPOINT_ARGS(
    subscription_handle,
    node_handle,
    rmw_subscription_handle,
    topic_name,
    queue_depth))

DEFINE_TRACEPOINT(
  rclcpp_subscription_init,
  TRACEPOINT_PARAMS(
    const void * subscription_handle,
    const void * subscription),
  TRACEPOINT_ARGS(
    subscription_handle,
    subscription))

DEFINE_TRACEPOINT(
  rclcpp_subscription_callback_added,
  TRACEPOINT_PARAMS(
    const void * subscription,
    const void * callback),
  TRACEPOINT_ARGS(
    subscription,
    callback))

DEFINE_TRACEPOINT(
  rmw_take,
  TRACEPOINT_PARAMS(
    const void * rmw_subscription_handle,
    const void * message,
    int64_t source_timestamp,
    const bool taken),
  TRACEPOINT_ARGS(
    rmw_subscription_handle,
    message,
    source_timestamp,
    taken))

DEFINE_TRACEPOINT(
  rcl_take,
  TRACEPOINT_PARAMS(
    const void * message),
  TRACEPOINT_ARGS(
    message))

DEFINE_TRACEPOINT(
  rclcpp_take,
  TRACEPOINT_PARAMS(
    const void * message),
  TRACEPOINT_ARGS(
    message))

DEFINE_TRACEPOINT(
  rcl_service_init,
  TRACEPOINT_PARAMS(
    const void * service_handle,
    const void * node_handle,
    const void * rmw_service_handle,
    const char * service_name),
  TRACEPOINT_ARGS(
    service_handle,
    node_handle,
    rmw_service_handle,
    service_name))

DEFINE_TRACEPOINT(
  rclcpp_service_callback_added,
  TRACEPOINT_PARAMS(
    const void * service_handle,
    const void * callback),
  TRACEPOINT_ARGS(
    service_handle,
    callback))

DEFINE_TRACEPOINT(
  rcl_client_init,
  TRACEPOINT_PARAMS(
    const void * client_handle,
    const void * node_handle,
    const void * rmw_client_handle,
    const char * service_name),
  TRACEPOINT_ARGS(
    client_handle,
    node_handle,
    rmw_client_handle,
    service_name))

DEFINE_TRACEPOINT(
  rcl_timer_init,
  TRACEPOINT_PARAMS(
    const void * timer_handle,
    int64_t period),
  TRACEPOINT_ARGS(
    timer_handle,
    period))

DEFINE_TRACEPOINT(
  rclcpp_timer_callback_added,
  TRACEPOINT_PARAMS(
    const void * timer_handle,
    const void * callback),
  TRACEPOINT_ARGS(
    timer_handle,
    callback))

DEFINE_TRACEPOINT(
  rclcpp_timer_link_node,
  TRACEPOINT_PARAMS(
    const void * timer_handle,
    const void * node_handle),
  TRACEPOINT_ARGS(
    timer_handle,
    node_handle))

DEFINE_TRACEPOINT(
  rclcpp_callback_register,
  TRACEPOINT_PARAMS(
    const void * callback,
    const char * function_symbol),
  TRACEPOINT_ARGS(
    callback,
    function_symbol))

DEFINE_TRACEPOINT(
  callback_start,
  TRACEPOINT_PARAMS(
    const void * callback,
    const bool is_intra_process),
  TRACEPOINT_ARGS(
    callback,
    is_intra_process))

DEFINE_TRACEPOINT(
  callback_end,
  TRACEPOINT_PARAMS(
    const void * callback),
  TRACEPOINT_ARGS(
    callback))

DEFINE_TRACEPOINT(
  rcl_lifecycle_state_machine_init,
  TRACEPOINT_PARAMS(
    const void * node_handle,
    const void * state_machine),
  TRACEPOINT_ARGS(
    node_handle,
    state_machine))

DEFINE_TRACEPOINT(
  rcl_lifecycle_transition,
  TRACEPOINT_PARAMS(
    const void * state_machine,
    const char * start_label,
    const char * goal_label),
  TRACEPOINT_ARGS(
    state_machine,
    start_label,
    goal_label))

DEFINE_TRACEPOINT_NO_ARGS(
  rclcpp_executor_get_next_ready)

DEFINE_TRACEPOINT(
  rclcpp_executor_wait_for_work,
  TRACEPOINT_PARAMS(
    const int64_t timeout),
  TRACEPOINT_ARGS(
    timeout))

DEFINE_TRACEPOINT(
  rclcpp_executor_execute,
  TRACEPOINT_PARAMS(
    const void * handle),
  TRACEPOINT_ARGS(
    handle))

DEFINE_TRACEPOINT(
  rclcpp_ipb_to_subscription,
  TRACEPOINT_PARAMS(
    const void * ipb,
    const void * subscription),
  TRACEPOINT_ARGS(
    ipb,
    subscription))

DEFINE_TRACEPOINT(
  rclcpp_buffer_to_ipb,
  TRACEPOINT_PARAMS(
    const void * buffer,
    const void * ipb),
  TRACEPOINT_ARGS(
    buffer,
    ipb))

DEFINE_TRACEPOINT(
  rclcpp_construct_ring_buffer,
  TRACEPOINT_PARAMS(
    const void * buffer,
    const uint64_t capacity),
  TRACEPOINT_ARGS(
    buffer,
    capacity))

DEFINE_TRACEPOINT(
  rclcpp_ring_buffer_enqueue,
  TRACEPOINT_PARAMS(
    const void * buffer,
    const uint64_t index,
    const uint64_t size,
    const bool overwritten),
  TRACEPOINT_ARGS(
    buffer,
    index,
    size,
    overwritten))

DEFINE_TRACEPOINT(
  rclcpp_ring_buffer_dequeue,
  TRACEPOINT_PARAMS(
    const void * buffer,
    const uint64_t index,
    const uint64_t size),
  TRACEPOINT_ARGS(
    buffer,
    index,
    size))

DEFINE_TRACEPOINT(
  rclcpp_ring_buffer_clear,
  TRACEPOINT_PARAMS(
    const void * buffer),
  TRACEPOINT_ARGS(
    buffer))

#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

#endif  // TRACETOOLS_DISABLED
