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
