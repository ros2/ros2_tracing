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
 * Record a snapshot session.
 * 
 * \param session_name the session name
 * \param id the snapshot output ID
 * \param max_size the maximum size of the snapshot output in bytes
 * \param name the name of the snapshot output
 * \param url the destination URL for the snapshot output
 */
int lttng_record_snapshot(
  const std::string & session_name,
  const uint32_t id,
  const uint64_t max_size,
  const std::string & name,
  const std::string & url);

}  // namespace lttngpy

#endif  // LTTNGPY__SNAPSHOT_HPP_
