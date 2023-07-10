// Copyright 2023 Apex.AI, Inc.
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
#include <lttng/event.h>

#include <optional>

#include "lttngpy/context_lttng.hpp"

TEST(TestContextLttng, invalid_context_name) {
  EXPECT_EQ(std::nullopt, lttngpy::get_lttng_context_type(""));
  EXPECT_EQ(std::nullopt, lttngpy::get_lttng_context_type("unknown"));
}

TEST(TestContextLttng, valid_context_name) {
  const auto opt = lttngpy::get_lttng_context_type("pid");
  ASSERT_TRUE(opt.has_value());
  EXPECT_EQ(LTTNG_EVENT_CONTEXT_PID, opt.value());
}
