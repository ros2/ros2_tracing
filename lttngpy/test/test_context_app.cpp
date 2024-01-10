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

#include <variant>

#include "lttngpy/context_app.hpp"
#include "lttngpy/utils.hpp"

TEST(TestContextApp, invalid_app_context_name) {
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_app_context("")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_app_context("$app.")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_app_context(":")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_app_context("$app.p")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_app_context("$app.p:")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_app_context("$app.:")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_app_context("$app.:c")));
}

TEST(TestContextApp, valid_app_context_name) {
  const auto var = lttngpy::get_app_context("$app.provider:type");
  ASSERT_TRUE(!std::holds_alternative<int>(var));
  const auto context = std::get<lttngpy::lttng_event_context_app_ctx>(var);
  EXPECT_EQ("provider", context.provider_name);
  EXPECT_EQ("type", context.ctx_name);
}
