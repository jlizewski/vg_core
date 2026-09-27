#include "vg_core/io/session.hpp"

namespace vg::io {

Timestamp timestamp_of(const SessionMessage& message) {
  struct Visitor {
    Timestamp operator()(const SessionInfo& m) const { return m.start_time; }
    Timestamp operator()(const DepthImage& m) const { return m.frame.timestamp; }
    Timestamp operator()(const CameraCalibration& m) const { return m.timestamp; }
    Timestamp operator()(const StaticTransforms& m) const { return m.timestamp; }
    Timestamp operator()(const CameraImage& m) const { return m.timestamp; }
    Timestamp operator()(const PoseStamped& m) const { return m.timestamp; }
    Timestamp operator()(const TrackingStatus& m) const { return m.timestamp; }
    Timestamp operator()(const ImuSample& m) const { return m.timestamp; }
    Timestamp operator()(const GpsFix& m) const { return m.timestamp; }
  };
  return std::visit(Visitor{}, message);
}

}  // namespace vg::io
