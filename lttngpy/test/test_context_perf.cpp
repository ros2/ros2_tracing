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

#include "lttngpy/context_perf.hpp"
#include "lttngpy/utils.hpp"

TEST(TestContextPerf, invalid_context_name) {
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("unknown")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:cpu")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:cpu:")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:cpu:unknown")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:thread")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:thread:")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:thread:unknown")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:thread:raw")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:thread:raw:")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:thread:raw::")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:thread:raw:r")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:thread:raw:r:")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:thread:raw:r1:")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:thread:raw:r:name")));
  EXPECT_TRUE(lttngpy::is_int(lttngpy::get_perf_counter_context("perf:thread:raw:b1234:name")));
  EXPECT_TRUE(
    lttngpy::is_int(lttngpy::get_perf_counter_context("perf:thread:raw:r0110:name:extra")));
}

TEST(TestContextPerf, valid_context_name) {
  auto var = lttngpy::get_perf_counter_context("perf:thread:task-clock");
  ASSERT_TRUE(!std::holds_alternative<int>(var));
  auto context = std::get<lttng_event_perf_counter_ctx>(var);
  EXPECT_EQ(lttngpy::PERF_TYPE_SOFTWARE, context.type);
  EXPECT_EQ(lttngpy::PERF_COUNT_SW_TASK_CLOCK, context.config);
  EXPECT_STREQ("perf_thread_task_clock", context.name);

  var = lttngpy::get_perf_counter_context("perf:cpu:dTLB-loads");
  ASSERT_TRUE(!std::holds_alternative<int>(var));
  context = std::get<lttng_event_perf_counter_ctx>(var);
  EXPECT_EQ(lttngpy::PERF_TYPE_HW_CACHE, context.type);
  // See equation
  EXPECT_EQ(
    (lttngpy::PERF_COUNT_HW_CACHE_DTLB) |
    (lttngpy::PERF_COUNT_HW_CACHE_OP_READ << 8) |
    (lttngpy::PERF_COUNT_HW_CACHE_RESULT_ACCESS << 16),
    context.config);
  EXPECT_STREQ("perf_cpu_dTLB_loads", context.name);

  var = lttngpy::get_perf_counter_context("perf:thread:raw:r0110:name");
  ASSERT_TRUE(!std::holds_alternative<int>(var));
  context = std::get<lttng_event_perf_counter_ctx>(var);
  EXPECT_EQ(lttngpy::PERF_TYPE_RAW, context.type);
  EXPECT_EQ(272, context.config);
  EXPECT_STREQ("name", context.name);
}
