// Copyright 2024 Apex.AI, Inc.
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

#ifndef TRACETOOLS_DISABLED
#include <lttng/tracef.h>
#include <lttng/ust-version.h>

#include <string>

#include "rcpputils/env.hpp"
#endif  // TRACETOOLS_DISABLED

#include <string_view>

#include "test_tracetools/mark_process.hpp"

namespace test_tracetools
{

namespace
{

/**
 * \see tracetools_test
 */
constexpr std::string_view trace_test_id_env_var = "TRACETOOLS_TEST_TRACE_TEST_ID";

}  // namespace

void mark_trace_test_process()
{
#ifndef TRACETOOLS_DISABLED
  // See tracetools_test.mark_process for more details
  const std::string env_var{trace_test_id_env_var};
  const auto test_id = rcpputils::get_env_var(env_var.c_str());
  if (!test_id.empty()) {
#if LTTNG_UST_MINOR_VERSION <= 12
    tracef("%s=%s", env_var.c_str(), test_id.c_str());
#else
    lttng_ust_tracef("%s=%s", env_var.c_str(), test_id.c_str());
#endif
  }
#endif  // TRACETOOLS_DISABLED
}

}  // namespace test_tracetools
