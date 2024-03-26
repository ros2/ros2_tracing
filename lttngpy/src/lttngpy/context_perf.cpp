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

#include <lttng/event.h>
#include <lttng/lttng-error.h>

#include <algorithm>
#include <map>
#include <string>
#include <variant>

#include "lttngpy/context_perf.hpp"
#include "lttngpy/utils.hpp"

namespace lttngpy
{

/**
 * Leave the name as an empty string here, we'll fill it in automatically afterwards.
 *
 * The last item is the padding.
 */
#define _CTX(ctx_type, counter_id) lttng_event_perf_counter_ctx{ctx_type, counter_id, "", {}}
#define _CTX_HW(counter_id) _CTX(PERF_TYPE_HARDWARE, counter_id)
#define _CTX_HW_CACHE(cache_id) _CTX(PERF_TYPE_HW_CACHE, cache_id)
#define _CTX_SW(counter_id) _CTX(PERF_TYPE_SOFTWARE, counter_id)
#define CTX_HW(name, counter_id) {name, _CTX_HW(counter_id)}
#define CTX_SW(name, counter_id) {name, _CTX_SW(counter_id)}

// *INDENT-OFF*
/**
 * Get 'config' value for a cache/op/result combination.
 *
 * See perf_event_open(2) for 'config' equation for type PERF_TYPE_HW_CACHE.
 */
#define _CTX_HW_CACHE_OP_RESULT(cache_id, op_id, result_id) \
  (cache_id) | (op_id << 8) | (result_id << 16)
#define CTX_HW_CACHE_OP_RESULT(name, cache_id, op_id, result_id) \
  {name, _CTX_HW_CACHE(_CTX_HW_CACHE_OP_RESULT(cache_id, op_id, result_id))}

/**
 * Each cache counter has 6 different variants: read/write/prefetch and access/miss (3 * 2 = 6).
 */
#define CTX_HW_CACHE(name, cache_id) \
  CTX_HW_CACHE_OP_RESULT( \
    name "-loads", cache_id, \
    PERF_COUNT_HW_CACHE_OP_READ, PERF_COUNT_HW_CACHE_RESULT_ACCESS), \
  CTX_HW_CACHE_OP_RESULT( \
    name "-load-misses", cache_id, \
    PERF_COUNT_HW_CACHE_OP_READ, PERF_COUNT_HW_CACHE_RESULT_MISS), \
  CTX_HW_CACHE_OP_RESULT( \
    name "-stores", cache_id, \
    PERF_COUNT_HW_CACHE_OP_WRITE, PERF_COUNT_HW_CACHE_RESULT_ACCESS), \
  CTX_HW_CACHE_OP_RESULT( \
    name "-store-misses", cache_id, \
    PERF_COUNT_HW_CACHE_OP_WRITE, PERF_COUNT_HW_CACHE_RESULT_MISS), \
  CTX_HW_CACHE_OP_RESULT( \
    name "-prefetches", cache_id, \
    PERF_COUNT_HW_CACHE_OP_PREFETCH, PERF_COUNT_HW_CACHE_RESULT_ACCESS), \
  CTX_HW_CACHE_OP_RESULT( \
    name "-prefetch-misses", cache_id, \
    PERF_COUNT_HW_CACHE_OP_PREFETCH, PERF_COUNT_HW_CACHE_RESULT_MISS)
// *INDENT-ON*

/**
 * LTTng perf counter name to perf counter context struct.
 *
 * For more information, refer to the lttng-ctl API and perf API.
 *
 * LTTng suppports 'perf:[counter name]' counters as old/backwards-compatible names for
 * 'perf:{cpu,thread}:[counter name]', but we leave them out here.
 */
std::map<std::string, struct lttng_event_perf_counter_ctx> perf_counter_name_to_perf_context = {
  // Per-CPU counters: hardware
  CTX_HW("perf:cpu:cpu-cycles", PERF_COUNT_HW_CPU_CYCLES),
  CTX_HW("perf:cpu:cycles", PERF_COUNT_HW_CPU_CYCLES),  // Alias
  CTX_HW("perf:cpu:instructions", PERF_COUNT_HW_INSTRUCTIONS),
  CTX_HW("perf:cpu:cache-references", PERF_COUNT_HW_CACHE_REFERENCES),
  CTX_HW("perf:cpu:cache-misses", PERF_COUNT_HW_CACHE_MISSES),
  CTX_HW("perf:cpu:branch-instructions", PERF_COUNT_HW_BRANCH_INSTRUCTIONS),
  CTX_HW("perf:cpu:branches", PERF_COUNT_HW_BRANCH_INSTRUCTIONS),  // Alias
  CTX_HW("perf:cpu:branch-misses", PERF_COUNT_HW_BRANCH_MISSES),
  CTX_HW("perf:cpu:bus-cycles", PERF_COUNT_HW_BUS_CYCLES),
  CTX_HW("perf:cpu:stalled-cycles-frontend", PERF_COUNT_HW_STALLED_CYCLES_FRONTEND),
  CTX_HW("perf:cpu:idle-cycles-frontend", PERF_COUNT_HW_STALLED_CYCLES_FRONTEND),  // Alias
  CTX_HW("perf:cpu:stalled-cycles-backend", PERF_COUNT_HW_STALLED_CYCLES_BACKEND),
  CTX_HW("perf:cpu:idle-cycles-backend", PERF_COUNT_HW_STALLED_CYCLES_BACKEND),

  // Per-CPU counters: hardware cache
  CTX_HW_CACHE("perf:cpu:L1-dcache", PERF_COUNT_HW_CACHE_L1D),
  CTX_HW_CACHE("perf:cpu:L1-icache", PERF_COUNT_HW_CACHE_L1I),
  CTX_HW_CACHE("perf:cpu:LLC", PERF_COUNT_HW_CACHE_LL),
  CTX_HW_CACHE("perf:cpu:dTLB", PERF_COUNT_HW_CACHE_DTLB),
  CTX_HW_CACHE_OP_RESULT(
    "perf:cpu:iTLB-loads",
    PERF_COUNT_HW_CACHE_ITLB,
    PERF_COUNT_HW_CACHE_OP_READ,
    PERF_COUNT_HW_CACHE_RESULT_ACCESS),
  CTX_HW_CACHE_OP_RESULT(
    "perf:cpu:iTLB-load-misses",
    PERF_COUNT_HW_CACHE_ITLB,
    PERF_COUNT_HW_CACHE_OP_READ,
    PERF_COUNT_HW_CACHE_RESULT_MISS),
  CTX_HW_CACHE_OP_RESULT(
    "perf:cpu:branch-loads",
    PERF_COUNT_HW_CACHE_BPU,
    PERF_COUNT_HW_CACHE_OP_READ,
    PERF_COUNT_HW_CACHE_RESULT_ACCESS),
  CTX_HW_CACHE_OP_RESULT(
    "perf:cpu:branch-load-misses",
    PERF_COUNT_HW_CACHE_BPU,
    PERF_COUNT_HW_CACHE_OP_READ,
    PERF_COUNT_HW_CACHE_RESULT_MISS),

  // Per-CPU counters: software
  CTX_SW("perf:cpu:cpu-clock", PERF_COUNT_SW_CPU_CLOCK),
  CTX_SW("perf:cpu:task-clock", PERF_COUNT_SW_TASK_CLOCK),
  CTX_SW("perf:cpu:page-fault", PERF_COUNT_SW_PAGE_FAULTS),
  CTX_SW("perf:cpu:faults", PERF_COUNT_SW_PAGE_FAULTS),  // Alias
  CTX_SW("perf:cpu:context-switches", PERF_COUNT_SW_CONTEXT_SWITCHES),
  CTX_SW("perf:cpu:cs", PERF_COUNT_SW_CONTEXT_SWITCHES),  // Alias
  CTX_SW("perf:cpu:cpu-migrations", PERF_COUNT_SW_CPU_MIGRATIONS),
  CTX_SW("perf:cpu:migrations", PERF_COUNT_SW_CPU_MIGRATIONS),  // Alias
  CTX_SW("perf:cpu:minor-faults", PERF_COUNT_SW_PAGE_FAULTS_MIN),
  CTX_SW("perf:cpu:major-faults", PERF_COUNT_SW_PAGE_FAULTS_MAJ),
  CTX_SW("perf:cpu:alignment-faults", PERF_COUNT_SW_ALIGNMENT_FAULTS),
  CTX_SW("perf:cpu:emulation-faults", PERF_COUNT_SW_EMULATION_FAULTS),

  // Per-thread counters: hardware
  CTX_HW("perf:thread:cpu-cycles", PERF_COUNT_HW_CPU_CYCLES),
  CTX_HW("perf:thread:clock", PERF_COUNT_HW_CPU_CYCLES),
  CTX_HW("perf:thread:instructions", PERF_COUNT_HW_INSTRUCTIONS),
  CTX_HW("perf:thread:cache-references", PERF_COUNT_HW_CACHE_REFERENCES),
  CTX_HW("perf:thread:cache-misses", PERF_COUNT_HW_CACHE_MISSES),
  CTX_HW("perf:thread:branch-instructions", PERF_COUNT_HW_BRANCH_INSTRUCTIONS),
  CTX_HW("perf:thread:branches", PERF_COUNT_HW_BRANCH_INSTRUCTIONS),  // Alias
  CTX_HW("perf:thread:branch-misses", PERF_COUNT_HW_BRANCH_MISSES),
  CTX_HW("perf:thread:bus-cycles", PERF_COUNT_HW_BUS_CYCLES),
  CTX_HW("perf:thread:stalled-cycles-frontend", PERF_COUNT_HW_STALLED_CYCLES_FRONTEND),
  CTX_HW("perf:thread:idle-cycles-frontend", PERF_COUNT_HW_STALLED_CYCLES_FRONTEND),  // Alias
  CTX_HW("perf:thread:stalled-cycles-backend", PERF_COUNT_HW_STALLED_CYCLES_BACKEND),
  CTX_HW("perf:thread:idle-cycles-backend", PERF_COUNT_HW_STALLED_CYCLES_BACKEND),  // Alias

  // Per-thread counters: hardware cache
  CTX_HW_CACHE("perf:thread:L1-dcache", PERF_COUNT_HW_CACHE_L1D),
  CTX_HW_CACHE("perf:thread:L1-icache", PERF_COUNT_HW_CACHE_L1I),
  CTX_HW_CACHE("perf:thread:LLC", PERF_COUNT_HW_CACHE_LL),
  CTX_HW_CACHE("perf:thread:dTLB", PERF_COUNT_HW_CACHE_DTLB),
  CTX_HW_CACHE_OP_RESULT(
    "perf:thread:iTLB-loads",
    PERF_COUNT_HW_CACHE_ITLB,
    PERF_COUNT_HW_CACHE_OP_READ,
    PERF_COUNT_HW_CACHE_RESULT_ACCESS),
  CTX_HW_CACHE_OP_RESULT(
    "perf:thread:iTLB-load-misses",
    PERF_COUNT_HW_CACHE_ITLB,
    PERF_COUNT_HW_CACHE_OP_READ,
    PERF_COUNT_HW_CACHE_RESULT_MISS),
  CTX_HW_CACHE_OP_RESULT(
    "perf:thread:branch-loads",
    PERF_COUNT_HW_CACHE_BPU,
    PERF_COUNT_HW_CACHE_OP_READ,
    PERF_COUNT_HW_CACHE_RESULT_ACCESS),
  CTX_HW_CACHE_OP_RESULT(
    "perf:thread:branch-load-misses",
    PERF_COUNT_HW_CACHE_BPU,
    PERF_COUNT_HW_CACHE_OP_READ,
    PERF_COUNT_HW_CACHE_RESULT_MISS),

  // Per-thread counters: software
  CTX_SW("perf:thread:cpu-clock", PERF_COUNT_SW_CPU_CLOCK),
  CTX_SW("perf:thread:task-clock", PERF_COUNT_SW_TASK_CLOCK),
  CTX_SW("perf:thread:page-fault", PERF_COUNT_SW_PAGE_FAULTS),
  CTX_SW("perf:thread:faults", PERF_COUNT_SW_PAGE_FAULTS),  // Alias
  CTX_SW("perf:thread:context-switches", PERF_COUNT_SW_CONTEXT_SWITCHES),
  CTX_SW("perf:thread:cs", PERF_COUNT_SW_CONTEXT_SWITCHES),  // Alias
  CTX_SW("perf:thread:cpu-migrations", PERF_COUNT_SW_CPU_MIGRATIONS),
  CTX_SW("perf:thread:migrations", PERF_COUNT_SW_CPU_MIGRATIONS),  // Alias
  CTX_SW("perf:thread:minor-faults", PERF_COUNT_SW_PAGE_FAULTS_MIN),
  CTX_SW("perf:thread:major-faults", PERF_COUNT_SW_PAGE_FAULTS_MAJ),
  CTX_SW("perf:thread:alignment-faults", PERF_COUNT_SW_ALIGNMENT_FAULTS),
  CTX_SW("perf:thread:emulation-faults", PERF_COUNT_SW_EMULATION_FAULTS),
};

std::variant<int, struct lttng_event_perf_counter_ctx> _get_perf_raw_counter_context(
  const std::string & perf_counter_name,
  const bool is_cpu_raw_counter,
  const bool is_thread_raw_counter)
{
  struct lttng_event_perf_counter_ctx context = {};
  context.type = lttngpy::PERF_TYPE_RAW;

  // Extract mask (NNN) and name: perf:{cpu,thread}:raw:rNNN:NAME
  std::string mask_prefix;
  if (is_cpu_raw_counter) {
    mask_prefix =
      std::string(perf_counter_cpu_prefix) +
      std::string(perf_counter_raw_prefix) +
      std::string(perf_counter_mask_prefix);
  } else if (is_thread_raw_counter) {
    mask_prefix =
      std::string(perf_counter_thread_prefix) +
      std::string(perf_counter_raw_prefix) +
      std::string(perf_counter_mask_prefix);
  } else {
    return -LTTNG_ERR_UST_CONTEXT_INVAL;
  }
  std::string::size_type prefix_start = perf_counter_name.find(mask_prefix);
  if (std::string::npos == prefix_start) {
    return -LTTNG_ERR_UST_CONTEXT_INVAL;
  }
  std::string::size_type mask_start = prefix_start + mask_prefix.length();

  std::string::size_type mask_name_sep =
    perf_counter_name.find(perf_counter_mask_name_sep, mask_start);
  if (std::string::npos == mask_name_sep) {
    return -LTTNG_ERR_UST_CONTEXT_INVAL;
  }
  std::string::size_type mask_len = mask_name_sep - mask_start;
  // Mask should not be empty
  if (0 == mask_len) {
    return -LTTNG_ERR_UST_CONTEXT_INVAL;
  }

  std::string::size_type name_start = mask_name_sep + perf_counter_mask_name_sep.length();
  std::string::size_type name_len = perf_counter_name.length() - name_start;
  // Name should not be empty
  if (0 == name_len) {
    return -LTTNG_ERR_UST_CONTEXT_INVAL;
  }

  std::string mask_str = perf_counter_name.substr(mask_start, mask_len);
  std::string name = perf_counter_name.substr(name_start, name_len);
  // Should not have another ':' in the name, after the sep between mask and name
  if (std::string::npos != name.find(':')) {
    return -LTTNG_ERR_UST_CONTEXT_INVAL;
  }

  // Mask value should be hex (see perf-record(1))
  const auto mask_opt = lttngpy::optional_stoull(mask_str, 16);
  if (!mask_opt.has_value()) {
    return -LTTNG_ERR_UST_CONTEXT_INVAL;
  }
  context.config = mask_opt.value();
  name.copy(context.name, LTTNG_SYMBOL_NAME_LEN);
  return context;
}

std::variant<int, struct lttng_event_perf_counter_ctx> _get_perf_hw_sw_counter_context(
  const std::string & perf_counter_name)
{
  const auto & it = lttngpy::perf_counter_name_to_perf_context.find(perf_counter_name);
  if (it == std::end(lttngpy::perf_counter_name_to_perf_context)) {
    return -LTTNG_ERR_UST_CONTEXT_INVAL;
  }

  /**
   * lttng_event_perf_counter_ctx.name should be the LTTng perf context field name with ':' and '-'
   * replaced with '_'.
   */
  struct lttng_event_perf_counter_ctx context = it->second;
  std::string name(perf_counter_name);
  std::replace(std::begin(name), std::end(name), ':', '_');
  std::replace(std::begin(name), std::end(name), '-', '_');
  name.copy(context.name, LTTNG_SYMBOL_NAME_LEN);
  return context;
}

std::variant<int, struct lttng_event_perf_counter_ctx> get_perf_counter_context(
  const std::string & perf_counter_name)
{
  // Check if it's a raw counter first
  const bool is_cpu_raw_counter = lttngpy::starts_with(
    perf_counter_name,
    std::string(perf_counter_cpu_prefix) + std::string(perf_counter_raw_prefix));
  const bool is_thread_raw_counter = lttngpy::starts_with(
    perf_counter_name,
    std::string(perf_counter_thread_prefix) + std::string(perf_counter_raw_prefix));
  if (is_cpu_raw_counter || is_thread_raw_counter) {
    return _get_perf_raw_counter_context(
      perf_counter_name, is_cpu_raw_counter, is_thread_raw_counter);
  }
  // Otherwise it's a normal hardware or software counter
  return _get_perf_hw_sw_counter_context(perf_counter_name);
}

}  // namespace lttngpy
