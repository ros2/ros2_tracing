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

#ifndef LTTNGPY__EVENT_HPP_
#define LTTNGPY__EVENT_HPP_

#include <lttng/domain.h>
#include <lttng/event.h>

#include <set>
#include <string>
#include <variant>

namespace lttngpy
{

/**
 * Enable events.
 *
 * \param session_name the session name
 * \param domain_type the domain type
 * \param channel_name the channel name
 * \param events the set of event names
 * \return 0 on success, else a negative LTTng error code
 */
int enable_events(
  const std::string & session_name,
  const enum lttng_domain_type domain_type,
  const std::string & channel_name,
  const std::set<std::string> & events);

/**
 * Get tracepoints.
 *
 * \param domain_type the domain type
 * \return the set of tracepoints, else a negative LTTng error code (e.g., if kernel tracer is not
 *   available when providing the kernel domain)
 */
std::variant<int, std::set<std::string>> get_tracepoints(const enum lttng_domain_type domain_type);

/**
 * Add contexts.
 *
 * \param session_name the session name
 * \param domain_type the domain type
 * \param channel_name the channel name
 * \param context_fields the set of context field names
 * \return 0 on success, else a negative LTTng error code
 */
int add_contexts(
  const std::string & session_name,
  const enum lttng_domain_type domain_type,
  const std::string & channel_name,
  const std::set<std::string> & context_fields);

}  // namespace lttngpy

#endif  // LTTNGPY__EVENT_HPP_
