#pragma once

// A recording made ready to watch in Foxglove (https://foxglove.dev), like a
// bag in RViz: the walk's poses as transforms and each depth image as a point
// cloud, optionally placed over a site's 3D model so you can see how the walk
// lines up with the map.
//
// Frames (a transform tree, as in ROS):
//
//   site ─(static: where the walk sits in the site)─ world
//   world ─(/tf, every pose)─ body ─(/tf_static)─ cam/<name>
//
// Topics:
//
//   /tf_static                 foxglove.FrameTransforms   recording's extrinsics, site -> world
//   /tf                        foxglove.FrameTransforms   world -> body at every pose
//   /cam/<name>/points         foxglove.PointCloud        each depth image, in cam/<name>
//   /cam/<name>/image          foxglove.CompressedImage   from the recording, thinned
//   /cam/<name>/calibration    foxglove.CameraCalibration copied from the recording
//   /walk/path                 foxglove.SceneUpdate       the whole walk as a line, in world
//   /site/model                foxglove.SceneUpdate       the site's surface and canopy, in site
//
// The poses are the ones the map was built from: with remove_pose_jumps, the
// tracker's jumps are undone as vg replay does, so world is the walk's map
// frame and the site's transform for it applies.

#include <cstdint>
#include <filesystem>
#include <optional>
#include <ostream>
#include <vector>

#include "vg_core/pose_jumps.hpp"
#include "vg_core/registration.hpp"
#include "vg_core/surface_mesh.hpp"

namespace vg::io {

struct FoxgloveExportOptions {
  // Every Nth depth pixel in each direction becomes a point.
  int pixel_stride = 2;
  // Readings beyond this range (m), below this confidence, or on depth edges
  // (see depth_filter.hpp) are left out.
  double max_depth = 3.5;
  std::uint8_t min_confidence = 0;
  double edge_ratio = 0.05;
  // Copy the camera images (most of the file's size), at most this many a
  // second per camera (0: every one).
  bool images = true;
  double image_rate = 10.0;
  // Undo the tracker's pose jumps, as vg replay does by default.
  bool remove_pose_jumps = true;
  PoseJumpConfig pose_jumps;
  // Where the walk's map frame sits in the site; with it, site_model is
  // drawn and the site frame is the root of the tree.
  std::optional<MapTransform> site_from_world;
  // The site in 3D (Site::model), in the site frame.
  std::vector<MeshPart> site_model;
};

struct FoxgloveExportSummary {
  std::size_t poses = 0;
  std::size_t clouds = 0;
  std::size_t points = 0;
  std::size_t images = 0;
  std::size_t pose_jumps = 0;
};

// Writes the viewer file for the recording at session to out. Progress goes to
// log. Throws std::runtime_error if the recording can't be read or out can't be
// written.
FoxgloveExportSummary export_foxglove(const std::filesystem::path& session,
                                      const std::filesystem::path& out,
                                      const FoxgloveExportOptions& options, std::ostream& log);

}  // namespace vg::io
