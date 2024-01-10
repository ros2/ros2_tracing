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

#ifndef LTTNGPY__SESSION_HPP_
#define LTTNGPY__SESSION_HPP_

#include <set>
#include <string>
#include <variant>

namespace lttngpy
{

/**
 * Get the currently-existing session names.
 *
 * \return the set of session names, else a negative LTTng error code
 */
std::variant<int, std::set<std::string>> get_session_names();

/**
 * Destroy all sessions.
 *
 * Tries to destroy all sessions, and reports an error if any session destruction was unsuccessful.
 *
 * \return 0 on success, else a negative LTTng error code
 */
int destroy_all_sessions();

}  // namespace lttngpy

#endif  // LTTNGPY__SESSION_HPP_
