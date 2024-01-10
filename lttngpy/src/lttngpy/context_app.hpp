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

#ifndef LTTNGPY__CONTEXT_APP_HPP_
#define LTTNGPY__CONTEXT_APP_HPP_

#include <string>
#include <variant>

namespace lttngpy
{

constexpr std::string_view app_context_prefix = "$app.";
constexpr std::string_view app_context_provider_ctx_sep = ":";

/**
 * See struct lttng_event_context.u.app_ctx.
 */
struct lttng_event_context_app_ctx
{
  std::string provider_name;
  std::string ctx_name;
};

/**
 * Get provider name and context name from app context type name.
 *
 * The expected format is: '$app.PROVIDER:TYPE' (see lttng-add-context(1)).
 *
 * \return the provider name and context name, else a negative LTTng error code (if the name isn't
 *   using the expected format)
*/
std::variant<int, struct lttng_event_context_app_ctx> get_app_context(
  const std::string & app_context_name);

}  // namespace lttngpy

#endif  // LTTNGPY__CONTEXT_APP_HPP_
