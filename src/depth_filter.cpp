#include "vg_core/depth_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace vg {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool has_depth(float d) { return std::isfinite(d) && d > 0.0f; }

Eigen::Vector3d back_project(const DepthFrame& frame, int u, int v, float d) {
  const CameraIntrinsics& k = frame.intrinsics;
  return {(u - k.cx) / k.fx * d, (v - k.cy) / k.fy * d, d};
}

}  // namespace

bool is_depth_edge(const DepthFrame& frame, int u, int v, double ratio) {
  const CameraIntrinsics& k = frame.intrinsics;
  const float d = frame.depth[frame.index(u, v)];
  const double limit = ratio * d;
  for (int dv = -1; dv <= 1; ++dv) {
    for (int du = -1; du <= 1; ++du) {
      const int nu = u + du;
      const int nv = v + dv;
      if ((du == 0 && dv == 0) || nu < 0 || nv < 0 || nu >= k.width || nv >= k.height) {
        continue;
      }
      const float n = frame.depth[frame.index(nu, nv)];
      if (has_depth(n) && std::abs(n - d) > limit) {
        return true;
      }
    }
  }
  return false;
}

double incidence_deg(const DepthFrame& frame, int u, int v) {
  const CameraIntrinsics& k = frame.intrinsics;
  const auto point = [&](int pu, int pv) -> std::optional<Eigen::Vector3d> {
    if (pu < 0 || pv < 0 || pu >= k.width || pv >= k.height) {
      return std::nullopt;
    }
    const float d = frame.depth[frame.index(pu, pv)];
    if (!has_depth(d)) {
      return std::nullopt;
    }
    return back_project(frame, pu, pv, d);
  };
  const auto center = point(u, v);
  if (!center) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // Tangents from whichever neighbors exist, preferring central differences.
  const auto tangent = [&](int du, int dv) -> std::optional<Eigen::Vector3d> {
    const auto next = point(u + du, v + dv);
    const auto prev = point(u - du, v - dv);
    if (next && prev) return *next - *prev;
    if (next) return *next - *center;
    if (prev) return *center - *prev;
    return std::nullopt;
  };
  const auto tu = tangent(1, 0);
  const auto tv = tangent(0, 1);
  if (!tu || !tv) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const Eigen::Vector3d normal = tu->cross(*tv);
  const double norm = normal.norm() * center->norm();
  if (norm <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double cosine = std::abs(normal.dot(*center)) / norm;
  return std::acos(std::min(1.0, cosine)) * 180.0 / kPi;
}

DepthFrame filter_depth(const DepthFrame& frame, const DepthFilterConfig& config) {
  DepthFrame out = frame;
  const CameraIntrinsics& k = frame.intrinsics;
  const bool edges = config.edge_ratio > 0.0;
  const bool angles = config.max_incidence_deg < 90.0;
  for (int v = 0; v < k.height; ++v) {
    for (int u = 0; u < k.width; ++u) {
      const std::size_t i = frame.index(u, v);
      const float d = frame.depth[i];
      if (!has_depth(d)) {
        continue;
      }
      // Tests read the unfiltered frame, so the result doesn't depend on
      // the order pixels are visited in.
      const bool keep = d >= config.min_depth && d <= config.max_depth &&
                        !(edges && is_depth_edge(frame, u, v, config.edge_ratio)) &&
                        !(angles && incidence_deg(frame, u, v) > config.max_incidence_deg);
      if (!keep) {
        out.depth[i] = 0.0f;
      }
    }
  }
  return out;
}

}  // namespace vg
