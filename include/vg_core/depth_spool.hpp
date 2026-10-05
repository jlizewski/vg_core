#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "vg_core/sensor_data.hpp"

namespace vg {

// A first-in, first-out queue of posed depth frames, held in memory or spilled
// to a file. MapBuilder parks the frames it doesn't integrate live here, so a
// long walk's worth of them can be fused later without holding them all in RAM.
//
// Depth is stored as whole millimeters (0 for no data, capped at 65.535 m), so
// a frame read back matches the one pushed to within half a millimeter. That is
// lossless for platforms that report millimeters, such as ARCore.
class DepthSpool {
 public:
  struct Entry {
    std::string camera;
    DepthFrame frame;
    Eigen::Isometry3d world_from_camera = Eigen::Isometry3d::Identity();
  };

  // Kept in memory.
  DepthSpool() = default;
  // Kept in a file at path, created (or truncated) now and deleted when the
  // spool is destroyed. Throws std::runtime_error if it can't be created.
  explicit DepthSpool(std::string path);
  ~DepthSpool();

  DepthSpool(const DepthSpool&) = delete;
  DepthSpool& operator=(const DepthSpool&) = delete;

  // Throws std::runtime_error if writing the file fails.
  void push(const std::string& camera, const DepthFrame& frame,
            const Eigen::Isometry3d& world_from_camera);

  // The oldest entry, or nullopt when empty. Throws std::runtime_error if the
  // file can't be read back.
  std::optional<Entry> pop();

  std::size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }

 private:
  struct Stored {
    std::string camera;
    Timestamp timestamp = 0;
    CameraIntrinsics intrinsics;
    Eigen::Isometry3d world_from_camera = Eigen::Isometry3d::Identity();
    std::vector<std::uint16_t> depth_mm;
    std::vector<std::uint8_t> confidence;
  };

  static Entry to_entry(Stored stored);

  std::string path_;
  std::ofstream out_;
  std::ifstream in_;
  std::deque<Stored> memory_;
  std::size_t size_ = 0;
};

}  // namespace vg
