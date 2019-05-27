#ifndef __TRACETOOLS_TRACETOOLS_H_
#define __TRACETOOLS_TRACETOOLS_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define TRACEPOINT(event_name, ...) \
  (ros_trace_##event_name)(__VA_ARGS__)

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * Report whether tracing is compiled in
 */
bool ros_trace_compile_status();

/**
 * tp: rcl_init
 */
void TRACEPOINT(
  rcl_init,
  const void * context);

/**
 * tp: rcl_node_init
 */
void TRACEPOINT(
  rcl_node_init,
  const void * node_handle,
  const void * rmw_handle,
  const char * node_name,
  const char * node_namespace);

/**
 * tp: rcl_publisher_init
 */
void TRACEPOINT(
  rcl_publisher_init,
  const void * node_handle,
  const void * rmw_handle,
  const void * publisher_handle,
  const char * topic_name,
  const size_t depth);

/**
 * tp: rcl_subscription_init
 */
void TRACEPOINT(
  rcl_subscription_init,
  const void * node_handle,
  const void * rmw_handle,
  const void * subscription_handle,
  const char * topic_name);

/**
 * tp: rclcpp_subscription_callback_added
 */
void TRACEPOINT(
  rclcpp_subscription_callback_added,
  const void * subscription_handle,
  const void * callback);

/**
 * tp: rclcpp_subscription_callback_start
 */
void TRACEPOINT(
  rclcpp_subscription_callback_start,
  const void * callback,
  const bool is_intra_process);

/**
 * tp: rclcpp_subscription_callback_end
 */
void TRACEPOINT(
  rclcpp_subscription_callback_end,
  const void * callback);

/**
 * tp: rcl_service_init
 */
void TRACEPOINT(
  rcl_service_init,
  const void * node_handle,
  const void * rmw_handle,
  const void * service_handle,
  const char * service_name);

/**
 * tp: rclcpp_service_callback_added
 */
void TRACEPOINT(
  rclcpp_service_callback_added,
  const void * service_handle,
  const void * callback);

/**
 * tp: rclcpp_service_callback_start
 */
void TRACEPOINT(
  rclcpp_service_callback_start,
  const void * callback);

/**
 * tp: rclcpp_service_callback_end
 */
void TRACEPOINT(
  rclcpp_service_callback_end,
  const void * callback);

#ifdef __cplusplus
}
#endif

#endif /* __TRACETOOLS_TRACETOOLS_H_ */
