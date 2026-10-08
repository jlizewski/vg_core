#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

#include "vg_core/io/session.hpp"

namespace vg::io {

struct McapRepairResult {
  std::size_t messages = 0;
  std::size_t channels = 0;
  std::size_t metadata = 0;
  std::size_t attachments = 0;
  // Log time of the first and last message recovered (0 if none).
  Timestamp start_time = 0;
  Timestamp end_time = 0;
  // True if the input was read to its end; false if reading stopped at a
  // damaged or cut-off record, described by problem.
  bool complete = true;
  std::string problem;
};

// Copies the readable part of an MCAP file into a new, properly indexed file.
// Meant for recordings whose writer never closed them (app killed, crash,
// phone out of space): those have no summary or index, and their last chunk
// may be cut off. Messages, metadata and attachments are copied front to back
// up to the first damaged record, so everything up to the last complete chunk
// is kept. Throws std::runtime_error if the input isn't an MCAP file, the
// output can't be written, or both name the same file.
McapRepairResult repair_mcap(const std::filesystem::path& input,
                             const std::filesystem::path& output);

}  // namespace vg::io
