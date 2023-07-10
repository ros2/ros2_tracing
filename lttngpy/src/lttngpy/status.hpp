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

#ifndef LTTNGPY__STATUS_HPP_
#define LTTNGPY__STATUS_HPP_

namespace lttngpy
{

/**
 * Check if lttng-ctl is available.
 *
 * This is the only function guaranteed to exist in this Python module. If this returns false, then
 * it means that no other functions are available.
 *
 * This is false on non-Linux platforms, or if it was explicitly disabled during build.
 *
 * \return true if available, false otherwise
 */
bool is_available();

}  // namespace lttngpy

#endif  // LTTNGPY__STATUS_HPP_
