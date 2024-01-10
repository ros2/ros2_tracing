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

#include <map>
#include <optional>
#include <string>

#include "lttngpy/config.hpp"
#include "lttngpy/context_lttng.hpp"

namespace lttngpy
{

/**
 * LTTng context name to context type.
 *
 * The perf counter context types are skipped and have to be handled separately.
 * Some other context types are skipped, since they are aliases/equal to another type.
 * Finally, the app context type is skipped for now.
 *
 * For more information, refer to the lttng-ctl API.
 */
std::map<std::string, enum lttng_event_context_type> context_name_to_context_type = {
  {"pid", LTTNG_EVENT_CONTEXT_PID},
  // LTTNG_EVENT_CONTEXT_PERF_COUNTER is equal to LTTNG_EVENT_CONTEXT_PERF_CPU_COUNTER
  {"procname", LTTNG_EVENT_CONTEXT_PROCNAME},
  {"prio", LTTNG_EVENT_CONTEXT_PRIO},
  {"nice", LTTNG_EVENT_CONTEXT_NICE},
  {"vpid", LTTNG_EVENT_CONTEXT_VPID},
  {"tid", LTTNG_EVENT_CONTEXT_TID},
  {"vtid", LTTNG_EVENT_CONTEXT_VTID},
  {"ppid", LTTNG_EVENT_CONTEXT_PPID},
  {"vppid", LTTNG_EVENT_CONTEXT_VPPID},
  {"pthread_id", LTTNG_EVENT_CONTEXT_PTHREAD_ID},
  {"hostname", LTTNG_EVENT_CONTEXT_HOSTNAME},
  {"ip", LTTNG_EVENT_CONTEXT_IP},
  // LTTNG_EVENT_CONTEXT_PERF_CPU_COUNTER is handled separately
  // LTTNG_EVENT_CONTEXT_PERF_THREAD_COUNTER is handled separately
  // LTTNG_EVENT_CONTEXT_APP_CONTEXT is handled separately
  {"interruptible", LTTNG_EVENT_CONTEXT_INTERRUPTIBLE},
  {"preemptible", LTTNG_EVENT_CONTEXT_PREEMPTIBLE},
  {"need_reschedule", LTTNG_EVENT_CONTEXT_NEED_RESCHEDULE},
  {"migratable", LTTNG_EVENT_CONTEXT_MIGRATABLE},
  {"callstack-kernel", LTTNG_EVENT_CONTEXT_CALLSTACK_KERNEL},
  {"callstack-user", LTTNG_EVENT_CONTEXT_CALLSTACK_USER},
  {"cgroup_ns", LTTNG_EVENT_CONTEXT_CGROUP_NS},
  {"ipc_ns", LTTNG_EVENT_CONTEXT_IPC_NS},
  {"mnt_ns", LTTNG_EVENT_CONTEXT_MNT_NS},
  {"net_ns", LTTNG_EVENT_CONTEXT_NET_NS},
  {"pid_ns", LTTNG_EVENT_CONTEXT_PID_NS},
  {"user_ns", LTTNG_EVENT_CONTEXT_USER_NS},
  {"uts_ns", LTTNG_EVENT_CONTEXT_UTS_NS},
  {"uid", LTTNG_EVENT_CONTEXT_UID},
  {"euid", LTTNG_EVENT_CONTEXT_EUID},
  {"suid", LTTNG_EVENT_CONTEXT_SUID},
  {"gid", LTTNG_EVENT_CONTEXT_GID},
  {"egid", LTTNG_EVENT_CONTEXT_EGID},
  {"sgid", LTTNG_EVENT_CONTEXT_SGID},
  {"vuid", LTTNG_EVENT_CONTEXT_VUID},
  {"veuid", LTTNG_EVENT_CONTEXT_VEUID},
  {"vsuid", LTTNG_EVENT_CONTEXT_VSUID},
  {"vgid", LTTNG_EVENT_CONTEXT_VGID},
  {"vegid", LTTNG_EVENT_CONTEXT_VEGID},
  {"vsgid", LTTNG_EVENT_CONTEXT_VSGID},
#if (LTTNG_CTL_VERSION_MAJOR >= 2) && (LTTNG_CTL_VERSION_MINOR >= 13)
  {"time_ns", LTTNG_EVENT_CONTEXT_TIME_NS},
#endif  // (LTTNG_CTL_VERSION_MAJOR >= 2) && (LTTNG_CTL_VERSION_MINOR >= 13)
};

std::optional<enum lttng_event_context_type> get_lttng_context_type(
  const std::string & lttng_context_field_name)
{
  const auto & it = context_name_to_context_type.find(lttng_context_field_name);
  if (it == std::end(lttngpy::context_name_to_context_type)) {
    return std::nullopt;
  }
  return it->second;
}

}  // namespace lttngpy
