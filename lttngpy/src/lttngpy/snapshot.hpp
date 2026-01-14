// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef LTTNGPY__SNAPSHOT_HPP_
#define LTTNGPY__SNAPSHOT_HPP_

#include <string>

namespace lttngpy
{

/**
 * Add an output object to the snapshot session.
 *
 * \param session_name the session name
 * \param max_size the maximum size of the snapshot output in bytes
 * \param name the name of the snapshot output
 * \param url the destination URL for the snapshot output
 * \return 0 on success, else a negative LTTng error code
 */
int add_snapshot_output(
  const std::string & session_name,
  const uint64_t max_size,
  const std::string & name,
  const std::string & url);

/**
 * Record a snapshot session.
 *
 * \param session_name the session name
 * \return 0 on success, else a negative LTTng error code
 */
int record_snapshot(const std::string & session_name);

}  // namespace lttngpy

#endif  // LTTNGPY__SNAPSHOT_HPP_
