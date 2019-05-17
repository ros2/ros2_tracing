#include "tracetools/tracetools.h"

#ifdef WITH_LTTNG
#include "tp_call.h"
#define CONDITIONAL_TP(...) \
  tracepoint(__VA_ARGS__);
#else
  #define CONDITIONAL_TP(...)
#endif


bool ros_trace_compile_status()
{
#ifdef WITH_LTTNG
  return true;
#else
  return false;
#endif
}

void TRACEPOINT(rcl_init)
{
  CONDITIONAL_TP(ros2, rcl_init);
}

void TRACEPOINT(
  rcl_node_init,
  const char * node_name,
  const char * node_namespace,
  const void * rmw_handle)
{
  CONDITIONAL_TP(ros2, rcl_node_init, node_name, node_namespace, rmw_handle);
}

void TRACEPOINT(
  rcl_publisher_init,
  const char * node_name,
  const char * node_namespace)
{
  CONDITIONAL_TP(ros2, rcl_publisher_init, node_name, node_namespace);
}

void TRACEPOINT(
  rcl_subscription_init,
  const void * subscription_handle,
  const char * node_name,
  const char * topic_name)
{
  CONDITIONAL_TP(ros2, rcl_subscription_init, subscription_handle, node_name, topic_name);
}

void TRACEPOINT(
    rclcpp_subscription_callback_added,
    const void * subscription_handle,
    const void * callback)
{
  CONDITIONAL_TP(ros2, rclcpp_subscription_callback_added, subscription_handle, callback);
}

void TRACEPOINT(
  rclcpp_subscription_callback_start,
  const void * callback,
  const bool is_intra_process)
{
  CONDITIONAL_TP(ros2, rclcpp_subscription_callback_start, callback, (is_intra_process ? 1 : 0));
}

void TRACEPOINT(
  rclcpp_subscription_callback_end,
  const void * callback)
{
  CONDITIONAL_TP(ros2, rclcpp_subscription_callback_end, callback);
}

void TRACEPOINT(
  rclcpp_service_callback_start,
  const void * callback)
{
  CONDITIONAL_TP(ros2, rclcpp_service_callback_start, callback);
}

void TRACEPOINT(
  rclcpp_service_callback_end,
  const void * callback)
{
  CONDITIONAL_TP(ros2, rclcpp_service_callback_end, callback);
}
