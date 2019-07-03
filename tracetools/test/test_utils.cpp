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

class SomeClassWithCallback
{
public:
  SomeClassWithCallback() {}

  void my_callback(int some_number, std::string some_string)
  {
    (void)some_number;
    (void)some_string;
  }
};

void function_shared(const std::shared_ptr<int> p)
{
  (void)p;
}

/*
   Testing address and symbol resolution for std::function objects.
 */
TEST(TestUtils, valid_address_symbol) {
  // Function pointer
  std::function<void(std::shared_ptr<int>)> f = &function_shared;
  // Address for one with an actual underlying function should be non-zero
  ASSERT_GT(get_address(f), (void *)0) << "get_address() for function not valid";
  ASSERT_STREQ(get_symbol(get_address(f)), "function_shared(std::shared_ptr<int>)") << "invalid function name";

  // Lambda
  std::function<int(int)> l = [](int num) {return num + 1;};
  // Address for an std::function with an underlying lambda should be nullptr
  ASSERT_EQ(get_address(l), nullptr) << "get_address() for lambda std::function not 0";
  // TODO symbol

  // Bind (to member function)
  SomeClassWithCallback scwc;
  std::function<void(int, std::string)> fscwc = std::bind(
    &SomeClassWithCallback::my_callback,
    &scwc,
    std::placeholders::_1,
    std::placeholders::_2
  );
  ASSERT_EQ(get_address(fscwc), nullptr) << "get_address() for std::bind std::function not 0";
  // TODO symbol
}
