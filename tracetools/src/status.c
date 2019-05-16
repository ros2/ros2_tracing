#include <stdio.h>
#include "tracetools/tracetools.h"

int main()
{
  printf("Tracing ");
  if (ros_trace_compile_status()) {
    printf("enabled\n");
    return 0;
  } else {
    printf("disabled\n");
    return 1;
  }
}
