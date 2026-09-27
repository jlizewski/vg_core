#pragma once

#include <filesystem>
#include <memory>

#include "vg_core/io/session.hpp"

namespace vg::io {

// Records a capture session to an MCAP file (zstd-compressed, protobuf
// messages). Messages should be written in timestamp order. Throws
// std::runtime_error on I/O errors.
class SessionWriter {
 public:
  SessionWriter(const std::filesystem::path& path, const SessionInfo& info);
  ~SessionWriter();
  SessionWriter(const SessionWriter&) = delete;
  SessionWriter& operator=(const SessionWriter&) = delete;

  void write(const CameraCalibration& calibration);
  void write(const StaticTransforms& transforms);
  void write(const CameraImage& image);
  // Writes the depth calibration and confidence (if any) alongside the depth.
  void write(const DepthImage& depth);
  void write(const PoseStamped& pose);
  void write(const TrackingStatus& status);
  void write(const ImuSample& imu);
  void write(const GpsFix& fix);
  // Any message except SessionInfo, which the constructor writes.
  void write(const SessionMessage& message);

  // Finishes the file. Called by the destructor if not called explicitly.
  void close();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vg::io
