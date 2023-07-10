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

#include <lttng/lttng-error.h>
#include <lttng/session.h>

#include <set>
#include <string>
#include <variant>

#include "lttngpy/session.hpp"

namespace lttngpy
{

std::variant<int, std::set<std::string>> get_session_names()
{
  struct lttng_session * sessions = nullptr;
  int ret = lttng_list_sessions(&sessions);
  if (0 > ret) {
    std::free(sessions);
    return ret;
  }

  std::set<std::string> session_names = {};
  const int num_sessions = ret;
  for (int i = 0; i < num_sessions; i++) {
    session_names.insert(sessions[i].name);
  }
  std::free(sessions);
  return session_names;
}

int destroy_all_sessions()
{
  const auto & session_names_opt = lttngpy::get_session_names();
  if (std::holds_alternative<int>(session_names_opt)) {
    return std::get<int>(session_names_opt);
  }

  int overall_ret = 0;
  const auto session_names = std::get<std::set<std::string>>(session_names_opt);
  for (const auto & session_name : session_names) {
    /**
     * Try to destroy all sessions, but return the last error code if any session destruction was
     * unsuccessful.
     */
    int ret = lttng_destroy_session(session_name.c_str());
    if (0 != ret) {
      overall_ret = ret;
    }
  }
  return overall_ret;
}

}  // namespace lttngpy
