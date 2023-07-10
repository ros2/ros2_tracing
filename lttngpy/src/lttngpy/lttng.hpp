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

#ifndef LTTNGPY__LTTNG_HPP_
#define LTTNGPY__LTTNG_HPP_

#include <string>

namespace lttngpy
{

/**
 * Check if session daemon is alive.
 *
 * \return true if alive, false otherwise
 */
bool is_lttng_session_daemon_alive();

}  // namespace lttngpy

#endif  // LTTNGPY__LTTNG_HPP_
