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

#include "tracetools/config.h"

#ifndef TRACETOOLS_DISABLED

#include <dlfcn.h>
#include <cxxabi.h>

#include "tracetools/utils.hpp"

namespace tracetools
{

namespace detail
{

char * demangle_symbol(const char * mangled)
{
  int status = 1;
  char * demangled = abi::__cxa_demangle(mangled, NULL, NULL, &status);
  return status == 0 ? demangled : nullptr;
}

char * get_symbol_funcptr(const void * funcptr)
{
  Dl_info info;
  if (dladdr(funcptr, &info) == 0) {
    return nullptr;
  }
  return demangle_symbol(info.dli_sname);
}

}  // namespace detail

}  // namespace tracetools

#endif  // TRACETOOLS_DISABLED
