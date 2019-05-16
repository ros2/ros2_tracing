#include "tracetools/tracetools.h"

#ifdef WITH_LTTNG
#include "tp_call.h"
#endif


bool ros_trace_compile_status() {
#ifdef WITH_LTTNG
    return true;
#else
    return false;
#endif
}

void ros_trace_rcl_init() {
#ifdef WITH_LTTNG
    tracepoint(ros2, rcl_init);
#endif
}

void ros_trace_rcl_node_init(const char * node_name, const char * namespace) {
#ifdef WITH_LTTNG
    tracepoint(ros2, rcl_node_init, node_name, namespace);
#endif
}

void ros_trace_rcl_publisher_init(const char * node_name, const char * namespace) {
#ifdef WITH_LTTNG
    tracepoint(ros2, rcl_publisher_init, node_name, namespace);
#endif
}

void ros_trace_rcl_subscription_init(const char * node_name, const char * topic_name) {
#ifdef WITH_LTTNG
    tracepoint(ros2, rcl_subscription_init, node_name, topic_name);
#endif
}
