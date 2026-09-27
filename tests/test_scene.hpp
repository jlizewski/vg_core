#pragma once

// Synthetic scenes for mapping tests: analytic depth rendering of a ground
// plane with an optional box (e.g. a raised bed) on it.

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include "vg_core/sensor_data.hpp"

namespace vg::test {

struct Box {
  Eigen::Vector3d min;
  Eigen::Vector3d max;
};

struct Scene {
  std::optional<Box> box;

  // Distance along dir (not normalized) from origin to the first hit, if any.
  std::optional<double> intersect(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir) const {
    double best = std::numeric_limits<double>::infinity();
    if (dir.z() < 0.0) {
      best = -origin.z() / dir.z();  // Ground plane z = 0.
    }
    if (box) {
      double t_near = -std::numeric_limits<double>::infinity();
      double t_far = std::numeric_limits<double>::infinity();
      bool hit = true;
      for (int a = 0; a < 3 && hit; ++a) {
        if (std::abs(dir[a]) < 1e-12) {
          hit = origin[a] >= box->min[a] && origin[a] <= box->max[a];
          continue;
        }
        double t0 = (box->min[a] - origin[a]) / dir[a];
        double t1 = (box->max[a] - origin[a]) / dir[a];
        if (t0 > t1) {
          std::swap(t0, t1);
        }
        t_near = std::max(t_near, t0);
        t_far = std::min(t_far, t1);
      }
      if (hit && t_near <= t_far && t_near > 0.0) {
        best = std::min(best, t_near);
      }
    }
    if (!std::isfinite(best)) {
      return std::nullopt;
    }
    return best;
  }
};

inline CameraIntrinsics test_intrinsics() {
  CameraIntrinsics k;
  k.width = 160;
  k.height = 120;
  k.fx = 100.0;
  k.fy = 100.0;
  k.cx = 79.5;
  k.cy = 59.5;
  return k;
}

// Camera at eye looking at target, OpenCV axes (x right, y down, z forward).
inline Eigen::Isometry3d look_at(const Eigen::Vector3d& eye, const Eigen::Vector3d& target,
                                 const Eigen::Vector3d& up) {
  const Eigen::Vector3d z = (target - eye).normalized();
  const Eigen::Vector3d x = z.cross(up).normalized();
  const Eigen::Vector3d y = z.cross(x);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear().col(0) = x;
  pose.linear().col(1) = y;
  pose.linear().col(2) = z;
  pose.translation() = eye;
  return pose;
}

inline DepthFrame render_depth(const Scene& scene, const Eigen::Isometry3d& world_from_camera,
                               const CameraIntrinsics& k = test_intrinsics()) {
  DepthFrame frame;
  frame.intrinsics = k;
  frame.depth.assign(static_cast<std::size_t>(k.width) * static_cast<std::size_t>(k.height), 0.0f);
  for (int v = 0; v < k.height; ++v) {
    for (int u = 0; u < k.width; ++u) {
      // With a unit z component, the hit distance equals the depth.
      const Eigen::Vector3d ray((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
      const auto t =
          scene.intersect(world_from_camera.translation(), world_from_camera.linear() * ray);
      if (t) {
        frame.depth[frame.index(u, v)] = static_cast<float>(*t);
      }
    }
  }
  return frame;
}

}  // namespace vg::test
