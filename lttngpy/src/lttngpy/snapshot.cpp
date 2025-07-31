#include <lttng/snapshot.h>
#include <lttng/lttng-error.h>

#include <string>

#include "lttngpy/snapshot.hpp"

namespace lttngpy
{

int lttng_record_snapshot(
  const std::string & session_name,
  const uint32_t id,
  const uint64_t max_size,
  const std::string & name,
  const std::string & url,
  const int wait)
{
  struct lttng_snapshot_output * output = lttng_snapshot_output_create();
  if (nullptr == output) {
    return -LTTNG_ERR_UNK;
  }

  // Set snapshot output attributes.
  lttng_snapshot_output_set_id(id, output);
  lttng_snapshot_output_set_size(max_size, output);
  lttng_snapshot_output_set_name(name.c_str(), output);
  lttng_snapshot_output_set_local_path(url.c_str(), output);

  // Add snapshot output to the session.
  lttng_snapshot_add_output(session_name.c_str(), output);

  int ret = lttng_snapshot_record(session_name.c_str(), output, wait);
  return ret;
}

}  // namespace lttngpy
