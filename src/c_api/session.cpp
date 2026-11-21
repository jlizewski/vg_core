// The C API's reading of a recording's session info (what it's for, which
// tracking session made it, how long it is), without reading its frames.

#include <memory>
#include <stdexcept>
#include <string>
#include <variant>

#include "common.hpp"
#include "vg_core/io/session_reader.hpp"

using vg::c_api::guard;
using vg::c_api::require;

struct vg_session {
  vg::io::SessionInfo info;
  bool indexed = false;
  bool has_time_range = false;
  int64_t first_ns = 0;
  int64_t last_ns = 0;
};

vg_status vg_session_open(const char* path, vg_session** out) {
  return guard([&] {
    const auto file = vg::c_api::path_of(path, "path");
    require(out, "out");
    auto session = std::make_unique<vg_session>();
    vg::io::SessionReader reader(file);
    // The session info is written first.
    const auto first = reader.next();
    const auto* info = first ? std::get_if<vg::io::SessionInfo>(&*first) : nullptr;
    if (info == nullptr) {
      throw std::runtime_error(file.string() + ": doesn't start with its session info");
    }
    session->info = *info;
    session->indexed = reader.indexed();
    if (const auto range = reader.time_range()) {
      session->has_time_range = true;
      session->first_ns = range->first;
      session->last_ns = range->second;
    }
    *out = session.release();
  });
}

void vg_session_free(vg_session* session) { delete session; }

vg_status vg_session_get_summary(const vg_session* session, vg_session_summary* out) {
  return guard([&] {
    require(session, "session");
    require(out, "out");
    const vg::io::SessionInfo& s = session->info;
    vg_session_summary summary{};
    summary.format_version = s.format_version.c_str();
    summary.info.producer = s.producer.c_str();
    summary.info.platform = s.platform.c_str();
    summary.info.device_model = s.device_model.c_str();
    summary.info.start_time_ns = s.start_time;
    summary.info.utc_offset_ns = s.utc_offset_ns;
    summary.info.notes = s.notes.c_str();
    summary.info.session_type = s.session_type.c_str();
    summary.info.tracking_frame = s.tracking_frame.c_str();
    summary.indexed = session->indexed ? 1 : 0;
    summary.has_time_range = session->has_time_range ? 1 : 0;
    summary.first_ns = session->first_ns;
    summary.last_ns = session->last_ns;
    *out = summary;
  });
}
