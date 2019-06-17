#if defined(WITH_LTTNG) && !defined(_WIN32)
#include <dlfcn.h>
#include <cxxabi.h>
#endif
#include "tracetools/utils.hpp"

const char * get_symbol(void * funptr)
{
#define SYMBOL_UNKNOWN "UNKNOWN"
#if defined(WITH_LTTNG) && !defined(_WIN32)
#define SYMBOL_LAMBDA "[lambda]"
  if (funptr == 0) {
    return SYMBOL_LAMBDA;
  }

  Dl_info info;
  if (dladdr(funptr, &info) == 0) {
    return SYMBOL_UNKNOWN;
  }

  char * demangled = nullptr;
  int status;
  demangled = abi::__cxa_demangle(info.dli_sname, NULL, 0, &status);
  // Use demangled symbol if possible
  const char * demangled_val = (status == 0 ? demangled : info.dli_sname);
  return demangled_val != 0 ? demangled_val : SYMBOL_UNKNOWN;
#else
  (void)funptr;
  return SYMBOL_UNKNOWN;
#endif
}
