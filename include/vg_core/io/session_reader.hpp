#pragma once

#include <filesystem>
#include <memory>
#include <optional>

#include "vg_core/io/session.hpp"

namespace vg::io {

// Reads a capture session in timestamp order. Depth images come with their
// intrinsics and confidence attached. Topics this version doesn't know are
// skipped. Throws std::runtime_error if the file can't be read.
class SessionReader {
 public:
  explicit SessionReader(const std::filesystem::path& path);
  ~SessionReader();
  SessionReader(const SessionReader&) = delete;
  SessionReader& operator=(const SessionReader&) = delete;

  // The next message, or nullopt at the end of the file.
  std::optional<SessionMessage> next();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vg::io
