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

#ifndef LTTNGPY__CONTEXT_PERF_HPP_
#define LTTNGPY__CONTEXT_PERF_HPP_

#include <lttng/event.h>

#include <string>
#include <variant>

namespace lttngpy
{

/**
 * See perf ABI (include/uapi/linux/perf_event.h).
 *
 * We/LTTng do not support all of the types here.
 */
enum perf_type_id
{
  PERF_TYPE_HARDWARE = 0,
  PERF_TYPE_SOFTWARE = 1,
  PERF_TYPE_TRACEPOINT = 2,  // Unused here
  PERF_TYPE_HW_CACHE = 3,
  PERF_TYPE_RAW = 4,  // Unused here
  PERF_TYPE_BREAKPOINT = 5,  // Unused here
  PERF_TYPE_MAX, /* non-ABI */
};

/**
 * See perf ABI (include/uapi/linux/perf_event.h).
 *
 * We/LTTng do not support all of the types here.
 */
enum perf_hw_cache_id
{
  PERF_COUNT_HW_CACHE_L1D = 0,
  PERF_COUNT_HW_CACHE_L1I = 1,
  PERF_COUNT_HW_CACHE_LL = 2,
  PERF_COUNT_HW_CACHE_DTLB = 3,
  PERF_COUNT_HW_CACHE_ITLB = 4,
  PERF_COUNT_HW_CACHE_BPU = 5,
  PERF_COUNT_HW_CACHE_NODE = 6,  // Unused here
  PERF_COUNT_HW_CACHE_MAX, /* non-ABI */
};

/**
 * See perf ABI (include/uapi/linux/perf_event.h).
 *
 * We/LTTng do not support all of the types here.
 */
enum perf_hw_cache_op_id
{
  PERF_COUNT_HW_CACHE_OP_READ = 0,
  PERF_COUNT_HW_CACHE_OP_WRITE = 1,
  PERF_COUNT_HW_CACHE_OP_PREFETCH = 2,
  PERF_COUNT_HW_CACHE_OP_MAX, /* non-ABI */
};

/**
 * See perf ABI (include/uapi/linux/perf_event.h).
 *
 * We/LTTng do not support all of the types here.
 */
enum perf_hw_cache_op_result_id
{
  PERF_COUNT_HW_CACHE_RESULT_ACCESS = 0,
  PERF_COUNT_HW_CACHE_RESULT_MISS = 1,
  PERF_COUNT_HW_CACHE_RESULT_MAX, /* non-ABI */
};

/**
 * See perf ABI (include/uapi/linux/perf_event.h).
 *
 * We/LTTng do not support all of the types here.
 */
enum perf_hw_id
{
  PERF_COUNT_HW_CPU_CYCLES = 0,
  PERF_COUNT_HW_INSTRUCTIONS = 1,
  PERF_COUNT_HW_CACHE_REFERENCES = 2,
  PERF_COUNT_HW_CACHE_MISSES = 3,
  PERF_COUNT_HW_BRANCH_INSTRUCTIONS = 4,
  PERF_COUNT_HW_BRANCH_MISSES = 5,
  PERF_COUNT_HW_BUS_CYCLES = 6,
  PERF_COUNT_HW_STALLED_CYCLES_FRONTEND = 7,
  PERF_COUNT_HW_STALLED_CYCLES_BACKEND = 8,
  PERF_COUNT_HW_REF_CPU_CYCLES = 9,  // Unused
  PERF_COUNT_HW_MAX, /* non-ABI */
};

/**
 * See perf ABI (include/uapi/linux/perf_event.h).
 *
 * We/LTTng do not support all of the types here.
 */
enum perf_sw_ids
{
  PERF_COUNT_SW_CPU_CLOCK = 0,
  PERF_COUNT_SW_TASK_CLOCK = 1,
  PERF_COUNT_SW_PAGE_FAULTS = 2,
  PERF_COUNT_SW_CONTEXT_SWITCHES = 3,
  PERF_COUNT_SW_CPU_MIGRATIONS = 4,
  PERF_COUNT_SW_PAGE_FAULTS_MIN = 5,
  PERF_COUNT_SW_PAGE_FAULTS_MAJ = 6,
  PERF_COUNT_SW_ALIGNMENT_FAULTS = 7,
  PERF_COUNT_SW_EMULATION_FAULTS = 8,
  PERF_COUNT_SW_DUMMY = 9,  // Unused
  PERF_COUNT_SW_BPF_OUTPUT = 10,  // Unused
  PERF_COUNT_SW_CGROUP_SWITCHES = 11,  // Unused
  PERF_COUNT_SW_MAX, /* non-ABI */
};

constexpr std::string_view perf_counter_prefix = "perf:";
constexpr std::string_view perf_counter_cpu_prefix = "perf:cpu:";
constexpr std::string_view perf_counter_thread_prefix = "perf:thread:";
constexpr std::string_view perf_counter_raw_prefix = "raw:";
constexpr std::string_view perf_counter_mask_prefix = "r";
constexpr std::string_view perf_counter_mask_name_sep = ":";

/**
 * Get perf counter context from name.
 *
 * \return the perf counter context, else a negative LTTng error code (if the name isn't known)
*/
std::variant<int, struct lttng_event_perf_counter_ctx> get_perf_counter_context(
  const std::string & perf_counter_name);

}  // namespace lttngpy

#endif  // LTTNGPY__CONTEXT_PERF_HPP_
