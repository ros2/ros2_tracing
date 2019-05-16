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
void ros_trace_rcl_node_init(const char * node_name, const char * namespace);

/**
 * tp: rcl_publisher_init
 */
void ros_trace_rcl_publisher_init(const char * node_name, const char * namespace);

/**
 * tp: rcl_subscription_init
 */
void ros_trace_rcl_subscription_init(const char * node_name, const char * topic_name);

#ifdef __cplusplus
}
#endif

#endif /* __TRACETOOLS_TRACETOOLS_H_ */
