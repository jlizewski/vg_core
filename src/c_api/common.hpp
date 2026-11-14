#pragma once

// Helpers shared by the C API's source files: turning C++ exceptions into
// status codes, and converting C structs to vg_core's types.

#include <Eigen/Geometry>
#include <cstddef>
#include <filesystem>
#include <new>
#include <ostream>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <vector>

#include "vg_core/config/params.hpp"
#include "vg_core/height_map.hpp"
#include "vg_core/sensor_data.hpp"
#include "vg_core_c/vg_core_c.h"

// A 2D map behind a vg_grid handle (made by the grid and map builder files).
struct vg_grid {
  vg::HeightMap map;
};

namespace vg::c_api {

// The calling thread's last error message (vg_last_error).
void set_error(const char* message) noexcept;
void clear_error() noexcept;

// Runs body, turning any exception into a status and the thread's error
// message. Nothing may throw across the C boundary.
template <typename Body>
vg_status guard(Body&& body) noexcept {
  clear_error();
  try {
    body();
    return VG_OK;
  } catch (const std::invalid_argument& e) {
    set_error(e.what());
    return VG_ERROR_INVALID_ARGUMENT;
  } catch (const config::ConfigError& e) {
    set_error(e.what());
    return VG_ERROR_CONFIG;
  } catch (const std::bad_alloc&) {
    set_error("out of memory");
    return VG_ERROR_INTERNAL;
  } catch (const std::runtime_error& e) {
    set_error(e.what());
    return VG_ERROR_FAILED;
  } catch (const std::exception& e) {
    set_error(e.what());
    return VG_ERROR_INTERNAL;
  } catch (...) {
    set_error("unknown error");
    return VG_ERROR_INTERNAL;
  }
}

// Throws std::invalid_argument naming what if pointer is NULL.
inline void require(const void* pointer, const char* what) {
  if (pointer == nullptr) {
    throw std::invalid_argument(std::string(what) + " is NULL");
  }
}

inline std::string text(const char* s) { return s != nullptr ? std::string(s) : std::string(); }

inline std::filesystem::path path_of(const char* utf8, const char* what) {
  require(utf8, what);
  return std::filesystem::u8path(utf8);
}

// count strings, none NULL (items may be NULL when count is 0).
std::vector<std::string> strings(const char* const* items, std::size_t count, const char* what);

Eigen::Isometry3d to_isometry(const vg_pose& pose);
CameraIntrinsics to_intrinsics(const vg_intrinsics& intrinsics);
DepthFrame to_depth_frame(const vg_depth_frame& frame);

// An ostream passing each line it receives to a vg_log_fn (or dropping it).
class Log {
 public:
  Log(vg_log_fn fn, void* user_data) : buffer_(fn, user_data), stream_(&buffer_) {}
  Log(const Log&) = delete;
  Log& operator=(const Log&) = delete;
  std::ostream& stream() { return stream_; }

 private:
  class LineBuffer : public std::streambuf {
   public:
    LineBuffer(vg_log_fn fn, void* user_data) : fn_(fn), user_data_(user_data) {}
    ~LineBuffer() override { emit(); }

   protected:
    int_type overflow(int_type ch) override;
    std::streamsize xsputn(const char* s, std::streamsize n) override;

   private:
    void emit();
    vg_log_fn fn_;
    void* user_data_;
    std::string line_;
  };

  LineBuffer buffer_;  // Before stream_, which writes to it.
  std::ostream stream_;
};

}  // namespace vg::c_api
