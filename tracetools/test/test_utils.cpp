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

#include <gtest/gtest.h>

#include <functional>
#include <memory>

#include "tracetools/utils.hpp"
#include "test_utils.hpp"

int function_int_int(int num)
{
  return num + 1;
}

void function_generic_shared(const std::shared_ptr<int> p)
{
  (void)p;
}

void function_generic_unique(const std::unique_ptr<int> p)
{
  (void)p;
}

/*
   Testing address and symbol resolution for std::function objects.
 */
TEST(TestUtils, valid_address_symbol) {
  std::function<int(int)> f = &function_int_int;
  std::function<int(int)> lambda = [](int num) {return num + 1;};
  std::function<int(int)> l = lambda;

  ASSERT_EQ(f(69), l(69));

  // Address for an std::function with an underlying lambda should be nullptr
  ASSERT_EQ(get_address(l), nullptr) << "get_address() for lambda std::function not 0";
  // But address for one with an actual underlying function should be non-zero
  ASSERT_GT(get_address(f), (void *)0) << "get_address() for function not valid";

  ASSERT_STREQ(get_symbol(get_address(f)), "function_int_int(int)") << "invalid function name";

  // Generic
  std::function<void (const std::shared_ptr<int>)> fg_shared = &function_generic_shared;
  SomeGenericClass<int> gc_shared;
  gc_shared.set(fg_shared);
  ASSERT_GT(gc_shared.get_address_(), (void *)0) << "generic -- address invalid";
  ASSERT_STREQ(gc_shared.get_symbol_(), "function_generic_shared(std::shared_ptr<int>)") << "generic -- symbol invalid";

  std::function<void (const std::unique_ptr<int>)> fg_unique = &function_generic_unique;
  SomeGenericClass<int> gc_unique;
  gc_unique.set(fg_unique);
  ASSERT_GT(gc_unique.get_address_(), (void *)0) << "generic -- address invalid";
  ASSERT_STREQ(gc_unique.get_symbol_(), "function_generic_unique(std::unique_ptr<int, std::default_delete<int> >)") << "generic -- symbol invalid";
}
