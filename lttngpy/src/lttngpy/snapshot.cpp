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

#include <lttng/snapshot.h>
#include <lttng/lttng-error.h>

#include <string>

#include "lttngpy/snapshot.hpp"

namespace lttngpy
{

int lttng_add_snapshot_output(
  const std::string & session_name,
  const uint64_t max_size,
  const std::string & name,
  const std::string & url)
{
  struct lttng_snapshot_output * output = lttng_snapshot_output_create();
  if (nullptr == output) {
    return -LTTNG_ERR_UNK;
  }

  int ret = 0;
  auto try_or_cleanup = [&](int function_call) -> bool {
      ret = function_call;
      if (0 != ret) {
        lttng_snapshot_output_destroy(output);
        return false;
      }
      return true;
    };

  // Set snapshot output attributes.
  if (!try_or_cleanup(lttng_snapshot_output_set_size(max_size, output))) {return ret;}
  if (!try_or_cleanup(lttng_snapshot_output_set_name(name.c_str(), output))) {return ret;}
  if (!try_or_cleanup(lttng_snapshot_output_set_local_path(url.c_str(), output))) {return ret;}

  // Add output object to the session.
  ret = lttng_snapshot_add_output(session_name.c_str(), output);
  return ret;
}

int lttng_record_snapshot(const std::string & session_name)
{
  int ret = lttng_snapshot_record(session_name.c_str(), NULL, 1);
  return ret;
}

}  // namespace lttngpy
