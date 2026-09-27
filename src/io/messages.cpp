#include "messages.hpp"

#include <cstring>
#include <stdexcept>

#include "protobuf.hpp"

namespace vg::io::detail {
namespace {

ProtoWriter vector3(const Eigen::Vector3d& v) {
  ProtoWriter w;
  w.double_field(1, v.x());
  w.double_field(2, v.y());
  w.double_field(3, v.z());
  return w;
}

ProtoWriter quaternion(const Eigen::Quaterniond& q) {
  ProtoWriter w;
  w.double_field(1, q.x());
  w.double_field(2, q.y());
  w.double_field(3, q.z());
  w.double_field(4, q.w());
  return w;
}

Eigen::Vector3d parse_vector3(std::string_view data) {
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  ProtoReader r(data);
  ProtoField f;
  while (r.next(f)) {
    if (f.number >= 1 && f.number <= 3) {
      v[f.number - 1] = f.as_double();
    }
  }
  return v;
}

Eigen::Quaterniond parse_quaternion(std::string_view data) {
  // Fields default to 0 in proto3, so an omitted w must be read as 0 too.
  double c[4] = {0.0, 0.0, 0.0, 0.0};
  ProtoReader r(data);
  ProtoField f;
  while (r.next(f)) {
    if (f.number >= 1 && f.number <= 4) {
      c[f.number - 1] = f.as_double();
    }
  }
  Eigen::Quaterniond q(c[3], c[0], c[1], c[2]);
  if (q.norm() < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return q.normalized();
}

Eigen::Isometry3d isometry(const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() = t;
  return pose;
}

std::string_view bytes_of(const void* data, std::size_t size) {
  return {static_cast<const char*>(data), size};
}

ProtoWriter raw_image(Timestamp timestamp, const std::string& frame_id, const CameraIntrinsics& k,
                      const char* encoding, std::uint32_t bytes_per_pixel,
                      std::string_view pixels) {
  ProtoWriter w;
  w.timestamp_field(1, timestamp);
  w.bytes_field(7, frame_id);
  w.fixed32_field(2, static_cast<std::uint32_t>(k.width));
  w.fixed32_field(3, static_cast<std::uint32_t>(k.height));
  w.bytes_field(4, encoding);
  w.fixed32_field(5, static_cast<std::uint32_t>(k.width) * bytes_per_pixel);
  w.bytes_field(6, pixels);
  return w;
}

std::string camera_frame(const std::string& camera) { return "cam/" + camera; }

}  // namespace

std::string encode(const SessionInfo& info) {
  ProtoWriter w;
  w.bytes_field(1, info.format_version);
  w.bytes_field(2, info.producer);
  w.bytes_field(3, info.platform);
  w.bytes_field(4, info.device_model);
  w.timestamp_field(5, info.start_time);
  w.int64_field(6, info.utc_offset_ns);
  w.bytes_field(7, info.notes);
  return w.data();
}

SessionInfo decode_session_info(std::string_view data) {
  SessionInfo info;
  info.format_version.clear();
  ProtoReader r(data);
  ProtoField f;
  while (r.next(f)) {
    switch (f.number) {
      case 1:
        info.format_version = f.bytes;
        break;
      case 2:
        info.producer = f.bytes;
        break;
      case 3:
        info.platform = f.bytes;
        break;
      case 4:
        info.device_model = f.bytes;
        break;
      case 5:
        info.start_time = parse_timestamp(f.bytes);
        break;
      case 6:
        info.utc_offset_ns = f.as_int64();
        break;
      case 7:
        info.notes = f.bytes;
        break;
      default:
        break;
    }
  }
  return info;
}

std::string encode_calibration(Timestamp timestamp, const std::string& frame_id,
                               const CameraIntrinsics& k) {
  const double K[9] = {k.fx, 0.0, k.cx, 0.0, k.fy, k.cy, 0.0, 0.0, 1.0};
  const double R[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  const double P[12] = {k.fx, 0.0, k.cx, 0.0, 0.0, k.fy, k.cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  ProtoWriter w;
  w.timestamp_field(1, timestamp);
  w.bytes_field(9, frame_id);
  w.fixed32_field(2, static_cast<std::uint32_t>(k.width));
  w.fixed32_field(3, static_cast<std::uint32_t>(k.height));
  w.bytes_field(4, "plumb_bob");  // With no D coefficients: no distortion.
  w.packed_doubles(6, K, 9);
  w.packed_doubles(7, R, 9);
  w.packed_doubles(8, P, 12);
  return w.data();
}

CameraIntrinsics decode_calibration(std::string_view data) {
  CameraIntrinsics k;
  std::vector<double> K;
  ProtoReader r(data);
  ProtoField f;
  while (r.next(f)) {
    switch (f.number) {
      case 2:
        k.width = static_cast<int>(f.as_uint32());
        break;
      case 3:
        k.height = static_cast<int>(f.as_uint32());
        break;
      case 6:
        f.append_doubles(K);
        break;
      default:
        break;
    }
  }
  if (K.size() != 9) {
    throw std::runtime_error("session: camera calibration without a 3x3 K");
  }
  k.fx = K[0];
  k.cx = K[2];
  k.fy = K[4];
  k.cy = K[5];
  return k;
}

std::string encode(const StaticTransforms& transforms) {
  ProtoWriter w;
  for (const StaticTransform& t : transforms.transforms) {
    ProtoWriter ft;
    ft.timestamp_field(1, transforms.timestamp);
    ft.bytes_field(2, t.parent_frame);
    ft.bytes_field(3, t.child_frame);
    ft.message_field(4, vector3(t.parent_from_child.translation()));
    ft.message_field(5, quaternion(Eigen::Quaterniond(t.parent_from_child.rotation())));
    w.message_field(1, ft);
  }
  return w.data();
}

StaticTransforms decode_static_transforms(std::string_view data) {
  StaticTransforms out;
  ProtoReader r(data);
  ProtoField f;
  while (r.next(f)) {
    if (f.number != 1) {
      continue;
    }
    StaticTransform t;
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    ProtoReader tr(f.bytes);
    ProtoField g;
    while (tr.next(g)) {
      switch (g.number) {
        case 1:
          out.timestamp = parse_timestamp(g.bytes);
          break;
        case 2:
          t.parent_frame = g.bytes;
          break;
        case 3:
          t.child_frame = g.bytes;
          break;
        case 4:
          translation = parse_vector3(g.bytes);
          break;
        case 5:
          rotation = parse_quaternion(g.bytes);
          break;
        default:
          break;
      }
    }
    t.parent_from_child = isometry(translation, rotation);
    out.transforms.push_back(std::move(t));
  }
  return out;
}

std::string encode(const CameraImage& image) {
  ProtoWriter w;
  w.timestamp_field(1, image.timestamp);
  w.bytes_field(4, camera_frame(image.camera));
  w.bytes_field(2, bytes_of(image.data.data(), image.data.size()));
  w.bytes_field(3, image.format);
  return w.data();
}

CameraImage decode_image(std::string_view data) {
  CameraImage image;
  ProtoReader r(data);
  ProtoField f;
  while (r.next(f)) {
    switch (f.number) {
      case 1:
        image.timestamp = parse_timestamp(f.bytes);
        break;
      case 2:
        image.data.assign(f.bytes.begin(), f.bytes.end());
        break;
      case 3:
        image.format = f.bytes;
        break;
      default:
        break;
    }
  }
  return image;
}

std::string encode_depth(const DepthImage& depth) {
  const DepthFrame& d = depth.frame;
  return raw_image(d.timestamp, camera_frame(depth.camera), d.intrinsics, "32FC1", 4,
                   bytes_of(d.depth.data(), d.depth.size() * sizeof(float)))
      .data();
}

std::string encode_confidence(const DepthImage& depth) {
  const DepthFrame& d = depth.frame;
  return raw_image(d.timestamp, camera_frame(depth.camera), d.intrinsics, "mono8", 1,
                   bytes_of(d.confidence.data(), d.confidence.size()))
      .data();
}

RawImage decode_raw_image(std::string_view data) {
  RawImage image;
  ProtoReader r(data);
  ProtoField f;
  while (r.next(f)) {
    switch (f.number) {
      case 1:
        image.timestamp = parse_timestamp(f.bytes);
        break;
      case 2:
        image.width = static_cast<int>(f.as_uint32());
        break;
      case 3:
        image.height = static_cast<int>(f.as_uint32());
        break;
      case 4:
        image.encoding = f.bytes;
        break;
      case 5:
        image.step = f.as_uint32();
        break;
      case 6:
        image.data = f.bytes;
        break;
      default:
        break;
    }
  }
  return image;
}

namespace {

// Copies each row's pixels (skipping any row padding) into a packed vector.
template <typename T>
std::vector<T> unpack_rows(const RawImage& image) {
  const auto width = static_cast<std::size_t>(image.width);
  const auto height = static_cast<std::size_t>(image.height);
  const std::size_t row_bytes = width * sizeof(T);
  const std::size_t step = image.step == 0 ? row_bytes : image.step;
  if (step < row_bytes || (height > 0 && image.data.size() < step * (height - 1) + row_bytes)) {
    throw std::runtime_error("session: raw image data too short for its size");
  }
  std::vector<T> out(width * height);
  for (std::size_t y = 0; y < height; ++y) {
    std::memcpy(out.data() + y * width, image.data.data() + y * step, row_bytes);
  }
  return out;
}

}  // namespace

std::vector<float> depth_from_raw(const RawImage& image) {
  if (image.encoding == "32FC1") {
    return unpack_rows<float>(image);
  }
  if (image.encoding == "16UC1") {
    const auto mm = unpack_rows<std::uint16_t>(image);
    std::vector<float> meters(mm.size());
    for (std::size_t i = 0; i < mm.size(); ++i) {
      meters[i] = static_cast<float>(mm[i]) * 0.001f;
    }
    return meters;
  }
  throw std::runtime_error("session: unsupported depth encoding " + image.encoding);
}

std::vector<std::uint8_t> confidence_from_raw(const RawImage& image) {
  if (image.encoding != "mono8" && image.encoding != "8UC1") {
    throw std::runtime_error("session: unsupported confidence encoding " + image.encoding);
  }
  return unpack_rows<std::uint8_t>(image);
}

std::string encode(const PoseStamped& pose) {
  ProtoWriter p;
  p.message_field(1, vector3(pose.world_from_frame.translation()));
  p.message_field(2, quaternion(Eigen::Quaterniond(pose.world_from_frame.rotation())));
  ProtoWriter w;
  w.timestamp_field(1, pose.timestamp);
  w.bytes_field(2, "world");
  w.message_field(3, p);
  return w.data();
}

PoseStamped decode_pose(std::string_view data) {
  PoseStamped pose;
  ProtoReader r(data);
  ProtoField f;
  while (r.next(f)) {
    if (f.number == 1) {
      pose.timestamp = parse_timestamp(f.bytes);
    } else if (f.number == 3) {
      Eigen::Vector3d t = Eigen::Vector3d::Zero();
      Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
      ProtoReader pr(f.bytes);
      ProtoField g;
      while (pr.next(g)) {
        if (g.number == 1) {
          t = parse_vector3(g.bytes);
        } else if (g.number == 2) {
          q = parse_quaternion(g.bytes);
        }
      }
      pose.world_from_frame = isometry(t, q);
    }
  }
  return pose;
}

std::string encode(const TrackingStatus& status) {
  ProtoWriter w;
  w.timestamp_field(1, status.timestamp);
  w.varint_field(2, static_cast<std::uint64_t>(status.state));
  w.bytes_field(3, status.reason);
  return w.data();
}

TrackingStatus decode_tracking_status(std::string_view data) {
  TrackingStatus status;
  ProtoReader r(data);
  ProtoField f;
  while (r.next(f)) {
    switch (f.number) {
      case 1:
        status.timestamp = parse_timestamp(f.bytes);
        break;
      case 2:
        status.state = f.value <= 3 ? static_cast<TrackingState>(f.value) : TrackingState::Unknown;
        break;
      case 3:
        status.reason = f.bytes;
        break;
      default:
        break;
    }
  }
  return status;
}

std::string encode(const ImuSample& imu) {
  ProtoWriter w;
  w.timestamp_field(1, imu.timestamp);
  w.bytes_field(2, "body");
  w.message_field(3, vector3(imu.angular_velocity));
  w.message_field(4, vector3(imu.linear_acceleration));
  return w.data();
}

ImuSample decode_imu(std::string_view data) {
  ImuSample imu;
  ProtoReader r(data);
  ProtoField f;
  while (r.next(f)) {
    switch (f.number) {
      case 1:
        imu.timestamp = parse_timestamp(f.bytes);
        break;
      case 3:
        imu.angular_velocity = parse_vector3(f.bytes);
        break;
      case 4:
        imu.linear_acceleration = parse_vector3(f.bytes);
        break;
      default:
        break;
    }
  }
  return imu;
}

std::string encode(const GpsFix& fix) {
  // Eigen is column-major; LocationFix wants row-major. The matrix is
  // symmetric, but transpose anyway so asymmetric input round-trips.
  const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cov = fix.position_covariance;
  const bool known = !cov.isZero();
  ProtoWriter w;
  w.timestamp_field(6, fix.timestamp);
  w.bytes_field(7, "gps");
  w.double_field(1, fix.latitude);
  w.double_field(2, fix.longitude);
  w.double_field(3, fix.altitude);
  w.packed_doubles(4, cov.data(), 9);
  w.varint_field(5, known ? 3u : 0u);  // KNOWN or UNKNOWN
  return w.data();
}

GpsFix decode_gps(std::string_view data) {
  GpsFix fix;
  std::vector<double> cov;
  ProtoReader r(data);
  ProtoField f;
  while (r.next(f)) {
    switch (f.number) {
      case 1:
        fix.latitude = f.as_double();
        break;
      case 2:
        fix.longitude = f.as_double();
        break;
      case 3:
        fix.altitude = f.as_double();
        break;
      case 4:
        f.append_doubles(cov);
        break;
      case 6:
        fix.timestamp = parse_timestamp(f.bytes);
        break;
      default:
        break;
    }
  }
  if (cov.size() == 9) {
    fix.position_covariance =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(cov.data());
  }
  return fix;
}

}  // namespace vg::io::detail
