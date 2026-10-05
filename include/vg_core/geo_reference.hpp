#pragma once

// Where the map's world frame sits on the Earth.
//
// The world frame is gravity aligned (z up) but its origin and heading are set
// arbitrarily by the tracker at session start. GPS fixes recorded alongside
// the tracker's poses place it: the fixes give the latitude and longitude, and
// comparing the GPS track with the pose track gives the heading.

#include <Eigen/Core>
#include <optional>
#include <vector>

#include "vg_core/sensor_data.hpp"

namespace vg {

struct GeoReference {
  // WGS84 position of the world origin: degrees, and meters above the
  // ellipsoid.
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  // Rotation about up taking world axes to East-North-Up, in radians
  // counter-clockwise: enu = Rz(heading) * world. 0 means world +x is east.
  double heading = 0.0;
  // 1-sigma uncertainty of heading, in radians. Infinite when the heading was
  // not observed (too little movement, or no poses) and was left at 0.
  double heading_sigma = 0.0;

  // Direction in the world frame of a vector given in East-North-Up.
  Eigen::Vector3d world_from_enu(const Eigen::Vector3d& enu) const;
  Eigen::Vector3d enu_from_world(const Eigen::Vector3d& world) const;
};

struct GeoReferenceConfig {
  // A fix is used for the heading only if poses exist within this time of it
  // on both sides, in nanoseconds.
  Timestamp max_pose_gap = 500'000'000;
  // Horizontal 1-sigma GPS error assumed when a fix reports no covariance, in
  // meters.
  double default_gps_sigma = 5.0;
};

// Places the world frame using GPS fixes and the tracker's body poses
// (world_from_body), all on the session clock. Returns nullopt without fixes.
//
// The heading is the rotation that best lines up the GPS track with the pose
// track (weighted least squares); it is only as good as the walk was long
// compared with the GPS error, which heading_sigma reports. Over a few meters
// of walking with phone GPS it can be off by tens of degrees.
std::optional<GeoReference> estimate_geo_reference(const std::vector<GpsFix>& fixes,
                                                   const std::vector<PoseStamped>& poses,
                                                   const GeoReferenceConfig& config = {});

// East-North-Up offset in meters of a WGS84 point from a reference point, on
// the local tangent plane. Accurate to well under a meter within a few km.
Eigen::Vector3d enu_offset(double latitude, double longitude, double altitude, double ref_latitude,
                           double ref_longitude, double ref_altitude);

}  // namespace vg
