#include "vg_core/features.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>

namespace vg {
namespace {

constexpr double kPi = 3.14159265358979323846;

// An angle as a line's direction, in [0, pi).
double wrap_axis(double a) {
  a = std::fmod(a, kPi);
  return a < 0.0 ? a + kPi : a;
}

// The smaller angle between two line directions, in [0, pi/2].
double axis_difference(double a, double b) {
  const double d = std::abs(wrap_axis(a) - wrap_axis(b));
  return std::min(d, kPi - d);
}

Eigen::Vector2d direction(double axis) { return {std::cos(axis), std::sin(axis)}; }

// How far a sighting at p is from the element: across it, for a line (walks
// see different stretches of the same fence); else from its center.
double offset(const Feature& element, const Eigen::Vector2d& p) {
  const Eigen::Vector2d d = p - element.center;
  if (element.kind == FeatureKind::Line) {
    const Eigen::Vector2d u = direction(element.axis);
    return std::abs(d.x() * u.y() - d.y() * u.x());
  }
  return d.norm();
}

// How far apart two sightings of the same kind are, if they can be the same
// element; negative if they can't.
double distance(const Feature& a, const Feature& b, const FeatureConfig& config) {
  // A post seen whole in one walk can look like a small block in another.
  const auto small = [&](const Feature& f) {
    return f.kind != FeatureKind::Line && f.length <= config.small_block_size;
  };
  if (a.kind != b.kind && !(small(a) && small(b))) {
    return -1.0;
  }
  if (a.kind == FeatureKind::Line) {
    if (axis_difference(a.axis, b.axis) > config.match_angle_deg * kPi / 180.0) {
      return -1.0;
    }
    // Across the line, and overlapping along it.
    const Eigen::Vector2d u = direction(a.axis);
    const Eigen::Vector2d d = b.center - a.center;
    const double across = std::abs(d.x() * u.y() - d.y() * u.x());
    const double along = std::abs(d.dot(u));
    if (across > config.match_distance || along > (a.length + b.length) / 2.0) {
      return -1.0;
    }
    return across;
  }
  const double d = (a.center - b.center).norm();
  return d <= config.match_distance ? d : -1.0;
}

}  // namespace

std::string_view feature_kind_name(FeatureKind kind) {
  switch (kind) {
    case FeatureKind::Post:
      return "post";
    case FeatureKind::Line:
      return "line";
    case FeatureKind::Block:
      return "block";
  }
  return "post";
}

std::vector<Feature> walk_features(const std::vector<WalkObject>& objects,
                                   const FeatureConfig& config) {
  std::vector<Feature> features;
  for (const WalkObject& o : objects) {
    const bool stands = o.grounded && o.label != SemanticClass::Ground &&
                        o.label != SemanticClass::Sky && o.label != SemanticClass::Dynamic;
    if (!stands || !std::isfinite(o.center.x()) || !(o.top_above_ground >= config.min_height)) {
      continue;
    }
    Feature f;
    if (o.length <= config.post_max_size && o.width <= config.post_max_size) {
      f.kind = FeatureKind::Post;
    } else if (o.length >= config.line_min_length && o.width <= config.line_max_aspect * o.length) {
      f.kind = FeatureKind::Line;
    } else if (o.length * o.width >= config.block_min_area) {
      f.kind = FeatureKind::Block;
    } else {
      continue;
    }
    f.label = o.label;
    f.center = o.center;
    f.length = o.length;
    f.width = o.width;
    f.axis = wrap_axis(o.axis);
    f.height = o.top_above_ground;
    features.push_back(f);
  }
  return features;
}

Feature to_site(const Feature& f, const MapTransform& site_from_map) {
  Feature out = f;
  out.center = site_from_map.apply_xy(f.center);
  out.axis = wrap_axis(f.axis + site_from_map.yaw);
  return out;
}

FeatureLayer match_features(const std::vector<WalkFeatures>& walks, const FeatureConfig& config) {
  FeatureLayer layer;
  std::array<int, 3> made{};  // Elements made so far, by kind: their ids.
  // Running sums for each element's mean direction (lines): doubled angles.
  std::vector<Eigen::Vector2d> axis_sum;
  for (const WalkFeatures& walk : walks) {
    // Every pairing of a site element with one of this walk's, closest first.
    std::vector<std::tuple<double, std::size_t, std::size_t>> pairs;
    for (std::size_t e = 0; e < layer.elements.size(); ++e) {
      if (layer.elements[e].retired) {
        continue;
      }
      for (std::size_t f = 0; f < walk.features.size(); ++f) {
        const double d = distance(layer.elements[e].feature, walk.features[f], config);
        if (d >= 0.0) {
          pairs.emplace_back(d, e, f);
        }
      }
    }
    std::sort(pairs.begin(), pairs.end());
    std::vector<char> element_taken(layer.elements.size(), 0);
    std::vector<int> match(walk.features.size(), -1);
    for (const auto& [d, e, f] : pairs) {
      if (!element_taken[e] && match[f] < 0) {
        element_taken[e] = 1;
        match[f] = static_cast<int>(e);
      }
    }

    for (std::size_t f = 0; f < walk.features.size(); ++f) {
      const Feature& seen = walk.features[f];
      if (match[f] >= 0) {
        // The element's position and size: the mean of its sightings.
        const auto e = static_cast<std::size_t>(match[f]);
        SiteFeature& element = layer.elements[e];
        Feature& mean = element.feature;
        const double n = element.seen;
        mean.center = (mean.center * n + seen.center) / (n + 1.0);
        mean.length = (mean.length * n + seen.length) / (n + 1.0);
        mean.width = (mean.width * n + seen.width) / (n + 1.0);
        mean.height = (mean.height * n + seen.height) / (n + 1.0);
        axis_sum[e] += direction(2.0 * seen.axis);
        mean.axis = wrap_axis(std::atan2(axis_sum[e].y(), axis_sum[e].x()) / 2.0);
        if (mean.label == SemanticClass::Unknown) {
          mean.label = seen.label;
        }
        ++element.seen;
        element.missed = 0;
        element.last_seen = walk.recorded;
        layer.sightings.push_back({walk.id, element.id, seen});
      } else {
        SiteFeature element;
        const auto kind = static_cast<std::size_t>(seen.kind);
        element.id = std::string(feature_kind_name(seen.kind)) + "-" + std::to_string(++made[kind]);
        element.feature = seen;
        element.seen = 1;
        element.first_seen = walk.recorded;
        element.last_seen = walk.recorded;
        layer.elements.push_back(element);
        axis_sum.push_back(direction(2.0 * seen.axis));
        element_taken.push_back(1);
        layer.sightings.push_back({walk.id, element.id, seen});
      }
    }

    // Elements this walk covered but didn't find.
    for (std::size_t e = 0; e < element_taken.size(); ++e) {
      SiteFeature& element = layer.elements[e];
      if (element_taken[e] || element.retired || !walk.covers ||
          !walk.covers(element.feature.center)) {
        continue;
      }
      if (++element.missed >= config.retire_after_misses) {
        element.retired = true;
      }
    }
  }

  // How far each element's sightings lie from its position.
  for (SiteFeature& element : layer.elements) {
    for (const FeatureSighting& s : layer.sightings) {
      if (s.element == element.id) {
        element.spread = std::max(element.spread, offset(element.feature, s.feature.center));
      }
    }
  }
  return layer;
}

}  // namespace vg
