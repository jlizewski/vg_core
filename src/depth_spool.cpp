#include "vg_core/depth_spool.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

namespace vg {
namespace {

// Records are written in host byte order: a spool never leaves the device that
// wrote it.
template <typename T>
void write_value(std::ofstream& out, const T& value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

template <typename T>
void read_value(std::ifstream& in, T& value) {
  in.read(reinterpret_cast<char*>(&value), sizeof(T));
}

template <typename T>
void write_vector(std::ofstream& out, const std::vector<T>& values) {
  write_value(out, static_cast<std::uint64_t>(values.size()));
  out.write(reinterpret_cast<const char*>(values.data()),
            static_cast<std::streamsize>(values.size() * sizeof(T)));
}

template <typename T>
void read_vector(std::ifstream& in, std::vector<T>& values) {
  std::uint64_t size = 0;
  read_value(in, size);
  values.resize(static_cast<std::size_t>(size));
  in.read(reinterpret_cast<char*>(values.data()),
          static_cast<std::streamsize>(values.size() * sizeof(T)));
}

std::uint16_t to_millimeters(float meters) {
  if (!std::isfinite(meters) || meters <= 0.0f) {
    return 0;
  }
  return static_cast<std::uint16_t>(std::min(std::lround(meters * 1000.0f), 65535L));
}

}  // namespace

DepthSpool::DepthSpool(std::string path) : path_(std::move(path)) {
  out_.open(path_, std::ios::binary | std::ios::trunc);
  if (!out_) {
    throw std::runtime_error("Could not create depth spool " + path_);
  }
}

DepthSpool::~DepthSpool() {
  if (!path_.empty()) {
    out_.close();
    in_.close();
    std::remove(path_.c_str());
  }
}

void DepthSpool::push(const std::string& camera, const DepthFrame& frame,
                      const Eigen::Isometry3d& world_from_camera) {
  Stored stored;
  stored.camera = camera;
  stored.timestamp = frame.timestamp;
  stored.intrinsics = frame.intrinsics;
  stored.world_from_camera = world_from_camera;
  stored.depth_mm.resize(frame.depth.size());
  std::transform(frame.depth.begin(), frame.depth.end(), stored.depth_mm.begin(), to_millimeters);
  stored.confidence = frame.confidence;

  if (path_.empty()) {
    memory_.push_back(std::move(stored));
  } else {
    write_value(out_, static_cast<std::uint32_t>(camera.size()));
    out_.write(camera.data(), static_cast<std::streamsize>(camera.size()));
    write_value(out_, stored.timestamp);
    write_value(out_, stored.intrinsics);
    const Eigen::Matrix4d matrix = stored.world_from_camera.matrix();
    out_.write(reinterpret_cast<const char*>(matrix.data()), sizeof(double) * 16);
    write_vector(out_, stored.depth_mm);
    write_vector(out_, stored.confidence);
    if (!out_) {
      throw std::runtime_error("Could not write to depth spool " + path_);
    }
  }
  ++size_;
}

std::optional<DepthSpool::Entry> DepthSpool::pop() {
  if (size_ == 0) {
    return std::nullopt;
  }
  if (path_.empty()) {
    Stored stored = std::move(memory_.front());
    memory_.pop_front();
    --size_;
    return to_entry(std::move(stored));
  }

  // Reads trail writes, so make sure what was written has reached the file.
  out_.flush();
  if (!in_.is_open()) {
    in_.open(path_, std::ios::binary);
  }
  // A read may have buffered up to the old end of the file, so seek to where the next record
  // starts each time; seeking drops the stale buffer and any end-of-file state.
  in_.clear();
  in_.seekg(read_offset_);
  Stored stored;
  std::uint32_t camera_size = 0;
  read_value(in_, camera_size);
  stored.camera.resize(camera_size);
  in_.read(stored.camera.data(), static_cast<std::streamsize>(camera_size));
  read_value(in_, stored.timestamp);
  read_value(in_, stored.intrinsics);
  Eigen::Matrix4d matrix;
  in_.read(reinterpret_cast<char*>(matrix.data()), sizeof(double) * 16);
  stored.world_from_camera.matrix() = matrix;
  read_vector(in_, stored.depth_mm);
  read_vector(in_, stored.confidence);
  if (!in_) {
    throw std::runtime_error("Could not read back depth spool " + path_);
  }
  read_offset_ = in_.tellg();
  --size_;
  return to_entry(std::move(stored));
}

DepthSpool::Entry DepthSpool::to_entry(Stored stored) {
  Entry entry;
  entry.camera = std::move(stored.camera);
  entry.world_from_camera = stored.world_from_camera;
  entry.frame.timestamp = stored.timestamp;
  entry.frame.intrinsics = stored.intrinsics;
  entry.frame.depth.resize(stored.depth_mm.size());
  std::transform(stored.depth_mm.begin(), stored.depth_mm.end(), entry.frame.depth.begin(),
                 [](std::uint16_t mm) { return static_cast<float>(mm) * 0.001f; });
  entry.frame.confidence = std::move(stored.confidence);
  return entry;
}

}  // namespace vg
