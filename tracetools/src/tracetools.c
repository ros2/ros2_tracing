#include "tracetools/tracetools.h"

#if defined(WITH_LTTNG) && !defined(_WIN32)
# include "tp_call.h"
# define CONDITIONAL_TP(...) \
  tracepoint(__VA_ARGS__)
#else
# define CONDITIONAL_TP(...)
#endif

bool ros_trace_compile_status()
{
#if defined(WITH_LTTNG) && !defined(_WIN32)
  return true;
#else
  return false;
#endif
}

void TRACEPOINT(
  rcl_init,
  const void * context_handle)
{
  CONDITIONAL_TP(
    ros2,
    rcl_init,
    context_handle);
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
  const void * node_handle,
  const void * publisher_handle,
  const void * rmw_publisher_handle,
  const char * topic_name,
  const size_t depth)
{
  CONDITIONAL_TP(
    ros2,
    rcl_publisher_init,
    node_handle,
    publisher_handle,
    rmw_publisher_handle,
    topic_name,
    depth);
}

void TRACEPOINT(
  rcl_subscription_init,
  const void * node_handle,
  const void * subscription_handle,
  const void * rmw_subscription_handle,
  const char * topic_name,
  const size_t depth)
{
  CONDITIONAL_TP(
    ros2,
    rcl_subscription_init,
    node_handle,
    subscription_handle,
    rmw_subscription_handle,
    topic_name,
    depth);
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
  rclcpp_subscription_callback_start,
  const void * callback,
  const bool is_intra_process)
{
  CONDITIONAL_TP(
    ros2,
    rclcpp_subscription_callback_start,
    callback,
    (is_intra_process ? 1 : 0));
}

void TRACEPOINT(
  rclcpp_subscription_callback_end,
  const void * callback)
{
  CONDITIONAL_TP(
    ros2,
    rclcpp_subscription_callback_end,
    callback);
}

void TRACEPOINT(
  rcl_service_init,
  const void * node_handle,
  const void * service_handle,
  const void * rmw_service_handle,
  const char * service_name)
{
  CONDITIONAL_TP(
    ros2,
    rcl_service_init,
    node_handle,
    service_handle,
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
  rclcpp_service_callback_start,
  const void * callback)
{
  CONDITIONAL_TP(
    ros2,
    rclcpp_service_callback_start,
    callback);
}

void TRACEPOINT(
  rclcpp_service_callback_end,
  const void * callback)
{
  CONDITIONAL_TP(
    ros2,
    rclcpp_service_callback_end,
    callback);
}

void TRACEPOINT(
  rcl_client_init,
  const void * node_handle,
  const void * client_handle,
  const void * rmw_client_handle,
  const char * service_name)
{
  CONDITIONAL_TP(
    ros2,
    rcl_client_init,
    node_handle,
    client_handle,
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
  rclcpp_timer_callback_start,
  const void * callback)
{
  CONDITIONAL_TP(
    ros2,
    rclcpp_timer_callback_start,
    callback);
}

void TRACEPOINT(
  rclcpp_timer_callback_end,
  const void * callback)
{
  CONDITIONAL_TP(
    ros2,
    rclcpp_timer_callback_end,
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
