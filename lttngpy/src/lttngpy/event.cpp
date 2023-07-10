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

#include <lttng/domain.h>
#include <lttng/event.h>
#include <lttng/lttng-error.h>
#include <string.h>

#include <map>
#include <optional>
#include <set>
#include <string>
#include <variant>

#include "lttngpy/context_app.hpp"
#include "lttngpy/context_lttng.hpp"
#include "lttngpy/context_perf.hpp"
#include "lttngpy/event.hpp"
#include "lttngpy/utils.hpp"

namespace lttngpy
{

int enable_events(
  const std::string & session_name,
  const enum lttng_domain_type domain_type,
  const std::string & channel_name,
  const std::set<std::string> & events)
{
  if (events.empty()) {
    return 0;
  }

  // We do not actually need to specify a buffer type for this
  struct lttng_domain domain {};
  domain.type = domain_type;

  struct lttng_handle * handle = lttng_create_handle(session_name.c_str(), &domain);
  if (nullptr == handle) {
    return -LTTNG_ERR_UNK;
  }

  int ret = 0;
  for (const auto & event_name : events) {
    struct lttng_event * event = lttng_event_create();
    if (nullptr == event) {
      ret = -LTTNG_ERR_UNK;
      break;
    }
    event_name.copy(event->name, LTTNG_SYMBOL_NAME_LEN);
    event->type = LTTNG_EVENT_TRACEPOINT;

    ret = lttng_enable_event(handle, event, channel_name.c_str());
    lttng_event_destroy(event);
    if (0 != ret) {
      break;
    }
  }

  lttng_destroy_handle(handle);
  return ret;
}

std::variant<int, std::set<std::string>> get_tracepoints(const enum lttng_domain_type domain_type)
{
  // We do not actually need to specify a buffer type for this
  struct lttng_domain domain {};
  domain.type = domain_type;

  struct lttng_handle * handle = lttng_create_handle(nullptr, &domain);
  if (nullptr == handle) {
    // This seems to be what lttng-ctl itself returns in this case
    return -LTTNG_ERR_NOMEM;
  }

  struct lttng_event * events = nullptr;
  int ret = lttng_list_tracepoints(handle, &events);
  std::variant<int, std::set<std::string>> tracepoints_var = ret;
  if (0 <= ret) {
    std::set<std::string> tracepoints = {};
    const int num_events = ret;
    for (int i = 0; i < num_events; i++) {
      tracepoints.insert(events[i].name);
    }
    tracepoints_var = tracepoints;
  }

  std::free(events);
  lttng_destroy_handle(handle);
  return tracepoints_var;
}

int _fill_in_event_context(
  const std::string & context_field,
  const enum lttng_domain_type domain_type,
  struct lttng_event_context * context)
{
  // Context type
  enum lttng_event_context_type ctx;
  if (lttngpy::starts_with(context_field, std::string(lttngpy::app_context_prefix))) {
    // App context
    ctx = LTTNG_EVENT_CONTEXT_APP_CONTEXT;
    const auto app_context_var = lttngpy::get_app_context(context_field);
    if (std::holds_alternative<int>(app_context_var)) {
      return std::get<int>(app_context_var);
    }
    const auto app_context = std::get<lttngpy::lttng_event_context_app_ctx>(app_context_var);
    // The caller must free the char arrays after use
    context->u.app_ctx.provider_name = strdup(app_context.provider_name.c_str());
    context->u.app_ctx.ctx_name = strdup(app_context.ctx_name.c_str());
  } else if (lttngpy::starts_with(context_field, std::string(lttngpy::perf_counter_prefix))) {
    // Perf counter context
    const bool is_cpu_counter =
      lttngpy::starts_with(context_field, std::string(lttngpy::perf_counter_cpu_prefix));
    const bool is_thread_counter =
      lttngpy::starts_with(context_field, std::string(lttngpy::perf_counter_thread_prefix));
    if (is_cpu_counter) {
      ctx = LTTNG_EVENT_CONTEXT_PERF_CPU_COUNTER;
    } else if (is_thread_counter) {
      ctx = LTTNG_EVENT_CONTEXT_PERF_THREAD_COUNTER;
    } else {
      return -LTTNG_ERR_UNK;
    }
  } else {
    // Normal LTTng context
    const auto context_type_opt = lttngpy::get_lttng_context_type(context_field);
    if (!context_type_opt.has_value()) {
      switch (domain_type) {
        case LTTNG_DOMAIN_KERNEL:
          return -LTTNG_ERR_KERN_CONTEXT_FAIL;
        case LTTNG_DOMAIN_UST:
          return -LTTNG_ERR_UST_CONTEXT_INVAL;
        case LTTNG_DOMAIN_NONE:  // Fall through
        case LTTNG_DOMAIN_JUL:  // Fall through
        case LTTNG_DOMAIN_LOG4J:  // Fall through
        case LTTNG_DOMAIN_PYTHON:  // Fall through
        default:
          return -LTTNG_ERR_UNK;
      }
    }
    ctx = context_type_opt.value();
  }
  context->ctx = ctx;

  // Perf counters
  if (context->ctx == LTTNG_EVENT_CONTEXT_PERF_CPU_COUNTER ||
    context->ctx == LTTNG_EVENT_CONTEXT_PERF_THREAD_COUNTER)
  {
    const auto counter_context_type_var = lttngpy::get_perf_counter_context(context_field);
    if (std::holds_alternative<int>(counter_context_type_var)) {
      return std::get<int>(counter_context_type_var);
    }
    context->u.perf_counter = std::get<lttng_event_perf_counter_ctx>(counter_context_type_var);
  }
  return 0;
}

int add_contexts(
  const std::string & session_name,
  const enum lttng_domain_type domain_type,
  const std::string & channel_name,
  const std::set<std::string> & context_fields)
{
  if (context_fields.empty()) {
    return 0;
  }

  // We do not actually need to specify a buffer type for this
  struct lttng_domain domain {};
  domain.type = domain_type;

  struct lttng_handle * handle = lttng_create_handle(session_name.c_str(), &domain);
  if (nullptr == handle) {
    return -LTTNG_ERR_UNK;
  }

  int ret = 0;
  for (const auto & context_field : context_fields) {
    struct lttng_event_context context = {};
    ret = _fill_in_event_context(context_field, domain_type, &context);
    if (0 == ret) {
      ret = lttng_add_context(handle, &context, nullptr, channel_name.c_str());
    }

    // Free app context strings
    if (LTTNG_EVENT_CONTEXT_APP_CONTEXT == context.ctx) {
      std::free(context.u.app_ctx.provider_name);
      std::free(context.u.app_ctx.ctx_name);
    }

    if (0 != ret) {
      break;
    }
  }

  lttng_destroy_handle(handle);
  return ret;
}

}  // namespace lttngpy
