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

#include <lttng/channel.h>
#include <lttng/constant.h>
#include <lttng/domain.h>
#include <lttng/event.h>
#include <lttng/handle.h>
#include <lttng/lttng-error.h>

#include <optional>
#include <string>

#include "lttngpy/channel.hpp"

namespace lttngpy
{

int enable_channel(
  const std::string & session_name,
  const enum lttng_domain_type domain_type,
  const enum lttng_buffer_type buffer_type,
  const std::string & channel_name,
  const std::optional<int> overwrite,
  const std::optional<uint64_t> subbuf_size,
  const std::optional<uint64_t> num_subbuf,
  const std::optional<uint64_t> switch_timer_interval,
  const std::optional<uint64_t> read_timer_interval,
  const std::optional<enum lttng_event_output> output)
{
  struct lttng_domain domain {};
  domain.type = domain_type;
  domain.buf_type = buffer_type;

  struct lttng_handle * handle = lttng_create_handle(session_name.c_str(), &domain);
  if (nullptr == handle) {
    return -LTTNG_ERR_UNK;
  }

  struct lttng_channel * chan = lttng_channel_create(&domain);
  if (nullptr == chan) {
    return -LTTNG_ERR_UNK;
  }
  /**
   * Configure channel.
   *
   * lttng_channel_create() sets default values for the attributes depending on the domain type
   * and/or buffer type, so set them only if needed.
   */
  channel_name.copy(chan->name, LTTNG_SYMBOL_NAME_LEN);
  if (overwrite.has_value()) {
    chan->attr.overwrite = overwrite.value();
  }
  if (subbuf_size.has_value()) {
    chan->attr.subbuf_size = subbuf_size.value();
  }
  if (num_subbuf.has_value()) {
    chan->attr.num_subbuf = num_subbuf.value();
  }
  if (switch_timer_interval.has_value()) {
    chan->attr.switch_timer_interval = switch_timer_interval.value();
  }
  if (read_timer_interval.has_value()) {
    chan->attr.read_timer_interval = read_timer_interval.value();
  }
  if (output.has_value()) {
    chan->attr.output = output.value();
  }

  int ret = lttng_enable_channel(handle, chan);
  lttng_channel_destroy(chan);
  lttng_destroy_handle(handle);
  return ret;
}

}  // namespace lttngpy
