#pragma once

// Plain sensor data types the map builder consumes. They mirror the capture
// format (docs/capture-format.md) but have no file-format dependency, so a host
// app can fill them from live sensors or vg_core_io can fill them from a file.
//
// Conventions: SI units, world frame gravity-aligned with z up, camera frames
// in OpenCV convention (x right, y down, z forward).

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace vg {

// Nanoseconds on the session's monotonic clock.
using Timestamp = std::int64_t;

// Pinhole intrinsics, in pixels. (cx, cy) is the principal point with pixel
// centers at integer coordinates.
struct CameraIntrinsics {
  int width = 0;
  int height = 0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
};

// Depth image registered to a camera. Row-major, depth along the optical (z)
// axis in meters; 0 or NaN means no data.
struct DepthFrame {
  Timestamp timestamp = 0;
  CameraIntrinsics intrinsics;
  std::vector<float> depth;

  // Optional per-pixel confidence, 0 (none) to 255 (full). Empty if the source
  // has none; otherwise the same size as depth.
  std::vector<std::uint8_t> confidence;

  std::size_t index(int u, int v) const {
    return static_cast<std::size_t>(v) * static_cast<std::size_t>(intrinsics.width) +
           static_cast<std::size_t>(u);
  }
};

// Pose of a frame (e.g. body or camera) in the world frame.
struct PoseStamped {
  Timestamp timestamp = 0;
  Eigen::Isometry3d world_from_frame = Eigen::Isometry3d::Identity();
};

// One gyroscope + accelerometer sample in the body frame.
struct ImuSample {
  Timestamp timestamp = 0;
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();     // rad/s
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();  // m/s^2
};

// GNSS fix. WGS84 degrees, altitude in meters above the ellipsoid.
struct GpsFix {
  Timestamp timestamp = 0;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  // East-North-Up position covariance in m^2.
  Eigen::Matrix3d position_covariance = Eigen::Matrix3d::Zero();
};

}  // namespace vg
