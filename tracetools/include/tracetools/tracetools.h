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
void ros_trace_rcl_init();

/**
 * tp: rcl_node_init
 */
void ros_trace_rcl_node_init(const char * node_name, const char * node_namespace, const void * rmw_handle);

/**
 * tp: rcl_publisher_init
 */
void ros_trace_rcl_publisher_init(const char * node_name, const char * node_namespace);

/**
 * tp: rcl_subscription_init
 */
void ros_trace_rcl_subscription_init(const char * node_name, const char * topic_name);

/**
 * tp: rclcpp_callback_start
 */
void ros_trace_rclcpp_callback_start(const void * callback, const bool is_intra_process);

/**
 * tp: rclcpp_callback_end
 */
void ros_trace_rclcpp_callback_end(const void * callback);

#ifdef __cplusplus
}
#endif

#endif /* __TRACETOOLS_TRACETOOLS_H_ */
