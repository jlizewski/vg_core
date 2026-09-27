#pragma once

// Messages of a capture session (docs/capture-format.md), as plain structs.

#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "vg_core/sensor_data.hpp"

namespace vg::io {

// Capture format version this library writes.
inline constexpr const char* kFormatVersion = "0.1";

// /vg/session: one per session, written first.
struct SessionInfo {
  std::string format_version = kFormatVersion;
  std::string producer;
  std::string platform;  // "arcore", "arkit", "rig", ...
  std::string device_model;
  Timestamp start_time = 0;
  // UTC time = session clock time + utc_offset_ns.
  std::int64_t utc_offset_ns = 0;
  std::string notes;
};

// /cam/<camera>/calibration: intrinsics of a camera's color image.
struct CameraCalibration {
  Timestamp timestamp = 0;
  std::string camera;  // Short name such as "rgb", "left", "right".
  CameraIntrinsics intrinsics;
};

// One fixed transform: pose of child_frame in parent_frame.
struct StaticTransform {
  std::string parent_frame;  // e.g. "body"
  std::string child_frame;   // e.g. "cam/rgb"
  Eigen::Isometry3d parent_from_child = Eigen::Isometry3d::Identity();
};

// /tf_static: fixed extrinsics between body, cameras and GPS antenna.
struct StaticTransforms {
  Timestamp timestamp = 0;
  std::vector<StaticTransform> transforms;
};

// /cam/<camera>/image: an encoded color image.
struct CameraImage {
  Timestamp timestamp = 0;
  std::string camera;
  std::string format;  // "jpeg" or "png"
  std::vector<std::uint8_t> data;
};

// /cam/<camera>/depth (+ /depth/calibration, /depth/confidence).
// frame.timestamp is the message time.
struct DepthImage {
  std::string camera;
  DepthFrame frame;
};

enum class TrackingState : std::uint8_t { Unknown = 0, Normal = 1, Limited = 2, Lost = 3 };

// /pose/status: platform tracker state, written when it changes.
struct TrackingStatus {
  Timestamp timestamp = 0;
  TrackingState state = TrackingState::Unknown;
  std::string reason;
};

// /pose carries PoseStamped (body in world), /imu ImuSample, /gps GpsFix.
using SessionMessage = std::variant<SessionInfo, CameraCalibration, StaticTransforms, CameraImage,
                                    DepthImage, PoseStamped, TrackingStatus, ImuSample, GpsFix>;

// Time of a message on the session clock.
Timestamp timestamp_of(const SessionMessage& message);

}  // namespace vg::io
