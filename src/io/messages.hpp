#pragma once

// Protobuf encoding of capture-session messages (docs/capture-format.md).

#include <string>
#include <string_view>

#include "vg_core/io/session.hpp"

namespace vg::io::detail {

std::string encode(const SessionInfo& info);
std::string encode_calibration(Timestamp timestamp, const std::string& frame_id,
                               const CameraIntrinsics& intrinsics);
std::string encode(const StaticTransforms& transforms);
std::string encode(const CameraImage& image);
// Depth as 32FC1 meters.
std::string encode_depth(const DepthImage& depth);
// Confidence as mono8. Requires depth.frame.confidence to be non-empty.
std::string encode_confidence(const DepthImage& depth);
std::string encode(const PoseStamped& pose);
std::string encode(const TrackingStatus& status);
std::string encode(const ImuSample& imu);
std::string encode(const GpsFix& fix);

SessionInfo decode_session_info(std::string_view data);
CameraIntrinsics decode_calibration(std::string_view data);
StaticTransforms decode_static_transforms(std::string_view data);
CameraImage decode_image(std::string_view data);

// A decoded foxglove.RawImage.
struct RawImage {
  Timestamp timestamp = 0;
  int width = 0;
  int height = 0;
  std::string encoding;
  std::uint32_t step = 0;
  std::string_view data;  // Points into the decoded buffer.
};
RawImage decode_raw_image(std::string_view data);
// Converts a 32FC1 (meters) or 16UC1 (millimeters) image to meters.
std::vector<float> depth_from_raw(const RawImage& image);
// Converts a mono8 image.
std::vector<std::uint8_t> confidence_from_raw(const RawImage& image);

PoseStamped decode_pose(std::string_view data);
TrackingStatus decode_tracking_status(std::string_view data);
ImuSample decode_imu(std::string_view data);
GpsFix decode_gps(std::string_view data);

}  // namespace vg::io::detail
