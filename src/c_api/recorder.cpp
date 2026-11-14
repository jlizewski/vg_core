// The C API's recorder: vg::io::SessionWriter behind a handle and a mutex.

#include <memory>
#include <mutex>
#include <stdexcept>

#include "common.hpp"
#include "vg_core/io/session_writer.hpp"

using vg::c_api::guard;
using vg::c_api::require;
using vg::c_api::text;

struct vg_recorder {
  std::mutex mutex;
  std::unique_ptr<vg::io::SessionWriter> writer;  // Null once closed.
};

namespace {

// Writes one message, holding the recorder's lock: capture plugins write
// from several threads (camera, IMU, location).
template <typename Message>
void write_locked(vg_recorder* recorder, const Message& message) {
  std::lock_guard<std::mutex> lock(recorder->mutex);
  if (!recorder->writer) {
    throw std::runtime_error("the recording is closed");
  }
  recorder->writer->write(message);
}

}  // namespace

vg_status vg_recorder_open(const char* path, const vg_session_info* info, vg_recorder** out) {
  return guard([&] {
    const auto file = vg::c_api::path_of(path, "path");
    require(info, "info");
    require(out, "out");
    vg::io::SessionInfo session;
    session.producer = text(info->producer);
    session.platform = text(info->platform);
    session.device_model = text(info->device_model);
    session.start_time = info->start_time_ns;
    session.utc_offset_ns = info->utc_offset_ns;
    session.notes = text(info->notes);
    auto recorder = std::make_unique<vg_recorder>();
    recorder->writer = std::make_unique<vg::io::SessionWriter>(file, session);
    *out = recorder.release();
  });
}

vg_status vg_recorder_write_calibration(vg_recorder* recorder, int64_t timestamp_ns,
                                        const char* camera, const vg_intrinsics* intrinsics) {
  return guard([&] {
    require(recorder, "recorder");
    require(camera, "camera");
    require(intrinsics, "intrinsics");
    vg::io::CameraCalibration calibration;
    calibration.timestamp = timestamp_ns;
    calibration.camera = camera;
    calibration.intrinsics = vg::c_api::to_intrinsics(*intrinsics);
    write_locked(recorder, calibration);
  });
}

vg_status vg_recorder_write_static_transforms(vg_recorder* recorder, int64_t timestamp_ns,
                                              const vg_static_transform* transforms,
                                              size_t count) {
  return guard([&] {
    require(recorder, "recorder");
    if (count > 0) {
      require(transforms, "transforms");
    }
    vg::io::StaticTransforms message;
    message.timestamp = timestamp_ns;
    for (size_t i = 0; i < count; ++i) {
      require(transforms[i].parent_frame, "parent_frame");
      require(transforms[i].child_frame, "child_frame");
      vg::io::StaticTransform t;
      t.parent_frame = transforms[i].parent_frame;
      t.child_frame = transforms[i].child_frame;
      t.parent_from_child = vg::c_api::to_isometry(transforms[i].parent_from_child);
      message.transforms.push_back(t);
    }
    write_locked(recorder, message);
  });
}

vg_status vg_recorder_write_image(vg_recorder* recorder, int64_t timestamp_ns,
                                  const char* camera, const char* format, const uint8_t* data,
                                  size_t size) {
  return guard([&] {
    require(recorder, "recorder");
    require(camera, "camera");
    require(format, "format");
    if (size > 0) {
      require(data, "data");
    }
    vg::io::CameraImage image;
    image.timestamp = timestamp_ns;
    image.camera = camera;
    image.format = format;
    if (image.format != "jpeg" && image.format != "png") {
      throw std::invalid_argument("image format must be \"jpeg\" or \"png\", not \"" +
                                  image.format + "\"");
    }
    if (size > 0) {
      image.data.assign(data, data + size);
    }
    write_locked(recorder, image);
  });
}

vg_status vg_recorder_write_depth(vg_recorder* recorder, const char* camera,
                                  const vg_depth_frame* frame) {
  return guard([&] {
    require(recorder, "recorder");
    require(camera, "camera");
    require(frame, "frame");
    vg::io::DepthImage depth;
    depth.camera = camera;
    depth.frame = vg::c_api::to_depth_frame(*frame);
    write_locked(recorder, depth);
  });
}

vg_status vg_recorder_write_pose(vg_recorder* recorder, int64_t timestamp_ns,
                                 const vg_pose* world_from_body) {
  return guard([&] {
    require(recorder, "recorder");
    require(world_from_body, "world_from_body");
    vg::PoseStamped pose;
    pose.timestamp = timestamp_ns;
    pose.world_from_frame = vg::c_api::to_isometry(*world_from_body);
    write_locked(recorder, pose);
  });
}

vg_status vg_recorder_write_tracking(vg_recorder* recorder, int64_t timestamp_ns, int32_t state,
                                     const char* reason) {
  return guard([&] {
    require(recorder, "recorder");
    if (state < VG_TRACKING_UNKNOWN || state > VG_TRACKING_LOST) {
      throw std::invalid_argument("unknown tracking state " + std::to_string(state));
    }
    vg::io::TrackingStatus status;
    status.timestamp = timestamp_ns;
    status.state = static_cast<vg::io::TrackingState>(state);
    status.reason = text(reason);
    write_locked(recorder, status);
  });
}

vg_status vg_recorder_write_imu(vg_recorder* recorder, const vg_imu_sample* sample) {
  return guard([&] {
    require(recorder, "recorder");
    require(sample, "sample");
    vg::ImuSample imu;
    imu.timestamp = sample->timestamp_ns;
    imu.angular_velocity = Eigen::Vector3d(sample->angular_velocity[0],
                                           sample->angular_velocity[1],
                                           sample->angular_velocity[2]);
    imu.linear_acceleration = Eigen::Vector3d(sample->linear_acceleration[0],
                                              sample->linear_acceleration[1],
                                              sample->linear_acceleration[2]);
    write_locked(recorder, imu);
  });
}

vg_status vg_recorder_write_gps(vg_recorder* recorder, const vg_gps_fix* fix) {
  return guard([&] {
    require(recorder, "recorder");
    require(fix, "fix");
    vg::GpsFix gps;
    gps.timestamp = fix->timestamp_ns;
    gps.latitude = fix->latitude;
    gps.longitude = fix->longitude;
    gps.altitude = fix->altitude;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        gps.position_covariance(r, c) = fix->position_covariance[r * 3 + c];
      }
    }
    write_locked(recorder, gps);
  });
}

vg_status vg_recorder_close(vg_recorder* recorder) {
  return guard([&] {
    require(recorder, "recorder");
    std::lock_guard<std::mutex> lock(recorder->mutex);
    if (recorder->writer) {
      recorder->writer->close();
      recorder->writer.reset();
    }
  });
}

void vg_recorder_free(vg_recorder* recorder) {
  if (recorder != nullptr) {
    vg_recorder_close(recorder);
    delete recorder;
  }
}
