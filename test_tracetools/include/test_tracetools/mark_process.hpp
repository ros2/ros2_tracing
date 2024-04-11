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

#ifndef TEST_TRACETOOLS__MARK_PROCESS_HPP_
#define TEST_TRACETOOLS__MARK_PROCESS_HPP_

namespace test_tracetools
{

/// Mark process to link it to a trace test.
/**
 * This should be called once by each test application (process) to mark the process so that it can
 * be linked to the trace test that spawned it.
 *
 * The process responsible for spawning the current process is responsible for setting the
 * `TRACETOOLS_TEST_TRACE_TEST_ID` environment variable to a unique value.
 *
 * If the `TRACETOOLS_TEST_TRACE_TEST_ID` environment variable is set and not empty, this marks the
 * current process by triggering an `lttng_ust_tracef:event` trace event with a `msg` field value
 * equal to: `"TRACETOOLS_TEST_TRACE_TEST_ID=<value of TRACETOOLS_TEST_TRACE_TEST_ID>"`.
 *
 * \see tracetools_test
 */
void mark_trace_test_process();

}  // namespace test_tracetools

#endif  // TEST_TRACETOOLS__MARK_PROCESS_HPP_
