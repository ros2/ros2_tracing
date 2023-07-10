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

#include <string>
#include <variant>

#include "lttngpy/context_app.hpp"

namespace lttngpy
{

std::variant<int, struct lttng_event_context_app_ctx> get_app_context(
  const std::string & app_context_name)
{
  // Extract provider name and type name: $app.PROVIDER:TYPE
  std::string::size_type prefix_start = app_context_name.find(app_context_prefix);
  if (std::string::npos == prefix_start) {
    return -LTTNG_ERR_UST_CONTEXT_INVAL;
  }
  std::string::size_type provider_name_start = prefix_start + app_context_prefix.length();

  std::string::size_type provider_ctx_sep = app_context_name.find(app_context_provider_ctx_sep);
  if (std::string::npos == provider_ctx_sep) {
    return -LTTNG_ERR_UST_CONTEXT_INVAL;
  }
  std::string::size_type provider_name_len = provider_ctx_sep - provider_name_start;
  if (0 == provider_name_len) {
    return -LTTNG_ERR_UST_CONTEXT_INVAL;
  }
  std::string::size_type ctx_name_start = provider_ctx_sep + app_context_provider_ctx_sep.length();
  std::string::size_type ctx_name_len = app_context_name.length() - ctx_name_start;
  if (0 == ctx_name_len) {
    return -LTTNG_ERR_UST_CONTEXT_INVAL;
  }

  std::string provider_name = app_context_name.substr(provider_name_start, provider_name_len);
  std::string ctx_name = app_context_name.substr(ctx_name_start, ctx_name_len);

  struct lttng_event_context_app_ctx app_ctx = {};
  app_ctx.provider_name = provider_name;
  app_ctx.ctx_name = ctx_name;
  return app_ctx;
}

}  // namespace lttngpy
