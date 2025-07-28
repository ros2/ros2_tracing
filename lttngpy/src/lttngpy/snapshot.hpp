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
  const std::string & url,
  const int wait);

} // namespace lttngpy

#endif  // LTTNGPY__SNAPSHOT_HPP_