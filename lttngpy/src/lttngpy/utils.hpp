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

#ifndef LTTNGPY__UTILS_HPP_
#define LTTNGPY__UTILS_HPP_

#include <optional>
#include <string>
#include <variant>

namespace lttngpy
{

inline bool starts_with(const std::string & str, const std::string & substr)
{
  return std::equal(substr.begin(), substr.end(), str.begin());
}

inline std::optional<uint64_t> optional_stoull(const std::string & str, int base)
{
  try {
    return static_cast<uint64_t>(std::stoull(str, nullptr, base));
  } catch (...) {
  }
  return std::nullopt;
}

template<typename T>
inline bool is_int(const std::variant<int, T> & var)
{
  return std::holds_alternative<int>(var);
}

}  // namespace lttngpy

#endif  // LTTNGPY__UTILS_HPP_
