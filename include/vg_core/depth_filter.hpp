#pragma once

// Cleaning a depth image before it's fused into the map. Phone depth is least
// reliable far away, across depth discontinuities (where it smears between
// the near and far surface: "flying pixels") and on surfaces seen edge-on;
// left in, these become floating blobs and ghost surfaces in the map.

#include "vg_core/sensor_data.hpp"

namespace vg {

struct DepthFilterConfig {
  // Readings outside this range are dropped.
  double min_depth = 0.1;
  double max_depth = 5.0;
  // A reading differing from a valid 8-neighbor by more than this fraction of
  // its depth is on a depth edge and dropped. 0 keeps edges.
  double edge_ratio = 0.0;
  // Readings of a surface seen more than this far from head-on (angle between
  // its normal and the viewing ray, in degrees) are dropped. 90 keeps all.
  double max_incidence_deg = 90.0;
};

// Whether pixel (u, v) of a depth image sits on a depth discontinuity: a valid
// 8-neighbor differs from it by more than ratio times its depth.
bool is_depth_edge(const DepthFrame& frame, int u, int v, double ratio);

// Angle in degrees between the surface normal at pixel (u, v), estimated from
// its 4-neighbors, and the viewing ray; NaN if the neighbors have no depth.
double incidence_deg(const DepthFrame& frame, int u, int v);

// frame with the readings config rejects set to 0 (no data). Confidence is
// kept as is.
DepthFrame filter_depth(const DepthFrame& frame, const DepthFilterConfig& config);

}  // namespace vg
