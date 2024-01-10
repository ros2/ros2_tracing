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

#ifndef LTTNGPY__CHANNEL_HPP_
#define LTTNGPY__CHANNEL_HPP_

#include <lttng/domain.h>

#include <string>

namespace lttngpy
{

/**
 * Enable channel.
 *
 * \param session_name the session name
 * \param domain_type the domain type
 * \param buffer_type the buffer type
 * \param channel_name the channel name
 * \param overwrite the overwrite attribute (-1: session default, 1: overwrite, 0: discard), or
 *   `std::nullopt` for the default value
 * \param subbuf_size the subbuffer size in bytes (power of 2), or `std::nullopt` for the default
 *   value
 * \param num_subbuf the number of subbuffers (power of 2), or `std::nullopt` for the default value
 * \param switch_timer_interval the switch timer internal in usec (0 to disable), or `std::nullopt`
 *   for the default value
 * \param read_timer_interval the read timer internal in usec (0 to disable), or `std::nullopt` for
 *   the default value
 * \return 0 on success, else a negative LTTng error code
 */
int enable_channel(
  const std::string & session_name,
  const enum lttng_domain_type domain_type,
  const enum lttng_buffer_type buffer_type,
  const std::string & channel_name,
  const std::optional<int> overwrite,
  const std::optional<uint64_t> subbuf_size,
  const std::optional<uint64_t> num_subbuf,
  const std::optional<uint64_t> switch_timer_interval,
  const std::optional<uint64_t> read_timer_interval,
  const std::optional<enum lttng_event_output> output);

}  // namespace lttngpy

#endif  // LTTNGPY__CHANNEL_HPP_
