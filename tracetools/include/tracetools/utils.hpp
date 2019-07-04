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

#ifndef TRACETOOLS__UTILS_HPP_
#define TRACETOOLS__UTILS_HPP_

#include <stddef.h>
#include <functional>

#define SYMBOL_UNKNOWN "UNKNOWN"

const char * _demangle_symbol(const char * mangled);

const char * _get_symbol_funcptr(void * funcptr);

template<typename T, typename ... U>
const char * get_symbol(std::function<T(U...)> f)
{
  typedef T (fnType)(U...);
  fnType ** fnPointer = f.template target<fnType *>();
  // If we get an actual address
  if (fnPointer != nullptr) {
    void * funcptr = reinterpret_cast<void *>(*fnPointer);
    return _get_symbol_funcptr(funcptr);
  }
  // Otherwise we have to go through target_type()
  return _demangle_symbol(f.target_type().name());
}

#endif  // TRACETOOLS__UTILS_HPP_
