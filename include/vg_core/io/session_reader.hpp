#pragma once

#include <filesystem>
#include <memory>
#include <optional>

#include "vg_core/io/session.hpp"

namespace vg::io {

// Reads a capture session in timestamp order. Depth images come with their
// intrinsics and confidence attached. Topics this version doesn't know are
// skipped. A recording that was never closed (no index) is still read, up to
// its last complete chunk. Throws std::runtime_error if the file can't be read.
class SessionReader {
 public:
  explicit SessionReader(const std::filesystem::path& path);
  ~SessionReader();
  SessionReader(const SessionReader&) = delete;
  SessionReader& operator=(const SessionReader&) = delete;

  // The next message, or nullopt at the end of the file.
  std::optional<SessionMessage> next();

  // False if the file has no message index, i.e. its writer never closed it.
  bool indexed() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vg::io
