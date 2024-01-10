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

#ifndef LTTNGPY__CONTEXT_LTTNG_HPP_
#define LTTNGPY__CONTEXT_LTTNG_HPP_

#include <lttng/event.h>

#include <optional>
#include <string>

namespace lttngpy
{

/**
 * Get LTTng context type from name.
 *
 * \return the context type, or `std::nullopt` on failure (if the name isn't known)
*/
std::optional<enum lttng_event_context_type> get_lttng_context_type(
  const std::string & lttng_context_field_name);

}  // namespace lttngpy

#endif  // LTTNGPY__CONTEXT_LTTNG_HPP_
