// The C API's library functions, and the helpers in common.hpp.

#include <cmath>
#include <string>

#include "common.hpp"
#include "vg_core/version.hpp"

namespace vg::c_api {

namespace {

thread_local std::string last_error;

}  // namespace

void set_error(const char* message) noexcept {
  try {
    last_error = message != nullptr ? message : "";
  } catch (...) {
    last_error.clear();  // Out of memory: no message rather than none at all.
  }
}

void clear_error() noexcept { last_error.clear(); }

std::vector<std::string> strings(const char* const* items, std::size_t count, const char* what) {
  std::vector<std::string> out;
  if (count == 0) {
    return out;
  }
  require(items, what);
  out.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (items[i] == nullptr) {
      throw std::invalid_argument(std::string(what) + "[" + std::to_string(i) + "] is NULL");
    }
    out.emplace_back(items[i]);
  }
  return out;
}

Eigen::Isometry3d to_isometry(const vg_pose& pose) {
  Eigen::Quaterniond q(pose.orientation[3], pose.orientation[0], pose.orientation[1],
                       pose.orientation[2]);
  const double norm = q.norm();
  if (!std::isfinite(norm) || norm < 1e-9) {
    throw std::invalid_argument("pose orientation is not a rotation (zero or non-finite)");
  }
  q.coeffs() /= norm;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = q.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(pose.position[0], pose.position[1], pose.position[2]);
  return transform;
}

CameraIntrinsics to_intrinsics(const vg_intrinsics& intrinsics) {
  if (intrinsics.width <= 0 || intrinsics.height <= 0) {
    throw std::invalid_argument("intrinsics: width and height must be positive");
  }
  CameraIntrinsics k;
  k.width = intrinsics.width;
  k.height = intrinsics.height;
  k.fx = intrinsics.fx;
  k.fy = intrinsics.fy;
  k.cx = intrinsics.cx;
  k.cy = intrinsics.cy;
  return k;
}

DepthFrame to_depth_frame(const vg_depth_frame& frame) {
  if ((frame.depth_m == nullptr) == (frame.depth_mm == nullptr)) {
    throw std::invalid_argument("depth frame: give exactly one of depth_m and depth_mm");
  }
  DepthFrame out;
  out.timestamp = frame.timestamp_ns;
  out.intrinsics = to_intrinsics(frame.intrinsics);
  const std::size_t n = static_cast<std::size_t>(out.intrinsics.width) *
                        static_cast<std::size_t>(out.intrinsics.height);
  if (frame.depth_m != nullptr) {
    out.depth.assign(frame.depth_m, frame.depth_m + n);
  } else {
    out.depth.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      out.depth[i] = static_cast<float>(frame.depth_mm[i]) * 0.001f;
    }
  }
  if (frame.confidence != nullptr) {
    out.confidence.assign(frame.confidence, frame.confidence + n);
  }
  if (frame.labels != nullptr) {
    out.labels.assign(frame.labels, frame.labels + n);
  }
  return out;
}

Log::LineBuffer::int_type Log::LineBuffer::overflow(int_type ch) {
  if (traits_type::eq_int_type(ch, traits_type::eof())) {
    return traits_type::not_eof(ch);
  }
  const char c = traits_type::to_char_type(ch);
  if (c == '\n') {
    emit();
  } else {
    line_.push_back(c);
  }
  return ch;
}

std::streamsize Log::LineBuffer::xsputn(const char* s, std::streamsize n) {
  for (std::streamsize i = 0; i < n; ++i) {
    overflow(traits_type::to_int_type(s[i]));
  }
  return n;
}

void Log::LineBuffer::emit() {
  if (fn_ != nullptr && !line_.empty()) {
    fn_(user_data_, line_.c_str());
  }
  line_.clear();
}

}  // namespace vg::c_api

int32_t vg_api_version(void) { return VG_C_API_VERSION; }

const char* vg_version_string(void) {
  // A string literal underneath, so NUL-terminated.
  return vg::version_string().data();
}

const char* vg_last_error(void) { return vg::c_api::last_error.c_str(); }
