// Copyright 2019 Robert Bosch GmbH
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

#include <iostream>

#if defined(TRACETOOLS_LTTNG_ENABLED) && !defined(_WIN32)
#include <dlfcn.h>
#include <cxxabi.h>
#endif
#include "tracetools/utils.hpp"

const char * get_symbol(void * funptr)
{
#define SYMBOL_UNKNOWN "UNKNOWN"
#if defined(TRACETOOLS_LTTNG_ENABLED) && !defined(_WIN32)
#define SYMBOL_LAMBDA "[lambda]"
  if (funptr == 0) {
    std::cout << "lamba!" << std::endl;
    return SYMBOL_LAMBDA;
  }

  Dl_info info;
  if (dladdr(funptr, &info) == 0) {
    std::cout << "unknown!" << std::endl;
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
