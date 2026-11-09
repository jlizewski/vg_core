// vg_segment: label a recording's depth with a segmentation model on this computer (ONNX Runtime).
//
//   vg_segment <session.mcap> <out.mcap> [--model NAME] [--model-file PATH] [--every N]
//              [--threads N] [--force]
//
// Copies the recording to out.mcap with every depth image labeled by the
// model, from the camera image nearest in time, turned upright from the
// phone's pose as the app does (vg_core/segmentation.hpp). Labels already in
// the recording are replaced, so the same walk can be relabeled with another
// model or class mapping and compared (vg labels compare) or mapped (vg replay).
//
//   --model NAME       deeplabv3_mnv2_ade20k (default) or segformer_b0_ade20k: the
//                      model's normalization and classes
//   --model-file PATH  the .onnx file (default: models/<NAME>.onnx in vg_core, as
//                      `vg models export` writes it)
//   --every N          label only every Nth depth image (default 1); the rest get none
//   --threads N        CPU threads for the model (default: ONNX Runtime's)
//   --force            replace out.mcap if it exists

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "vg_core/io/image_decode.hpp"
#include "vg_core/io/session_reader.hpp"
#include "vg_core/io/session_writer.hpp"
#include "vg_core/onnx_backend.hpp"
#include "vg_core/segmentation.hpp"

namespace {

int usage() {
  std::cerr << "usage: vg_segment <session.mcap> <out.mcap> [--model NAME] [--model-file PATH]\n"
               "                  [--every N] [--threads N] [--force]\n"
               "  models: deeplabv3_mnv2_ade20k (default), segformer_b0_ade20k\n";
  return 2;
}

// What the second pass needs from a first look at the recording.
struct Index {
  std::vector<vg::Timestamp> image_times;  // Of the camera's images, sorted.
  std::vector<vg::PoseStamped> poses;      // Sorted.
  Eigen::Isometry3d body_from_camera = Eigen::Isometry3d::Identity();
  std::string camera = "rgb";
};

Index index_session(const std::filesystem::path& path) {
  Index index;
  vg::io::SessionReader reader(path);
  while (auto m = reader.next()) {
    if (const auto* image = std::get_if<vg::io::CameraImage>(&*m)) {
      index.image_times.push_back(image->timestamp);
    } else if (const auto* pose = std::get_if<vg::PoseStamped>(&*m)) {
      index.poses.push_back(*pose);
    } else if (const auto* tf = std::get_if<vg::io::StaticTransforms>(&*m)) {
      for (const auto& t : tf->transforms) {
        if (t.parent_frame == "body" && t.child_frame == "cam/" + index.camera) {
          index.body_from_camera = t.parent_from_child;
        }
      }
    }
  }
  std::sort(index.image_times.begin(), index.image_times.end());
  std::sort(index.poses.begin(), index.poses.end(),
            [](const auto& a, const auto& b) { return a.timestamp < b.timestamp; });
  return index;
}

template <typename T, typename Time>
const T* nearest(const std::vector<T>& items, vg::Timestamp t, Time time_of) {
  if (items.empty()) {
    return nullptr;
  }
  const auto it = std::lower_bound(items.begin(), items.end(), t,
                                   [&](const T& item, vg::Timestamp v) { return time_of(item) < v; });
  if (it == items.end()) {
    return &items.back();
  }
  if (it != items.begin() && t - time_of(*std::prev(it)) < time_of(*it) - t) {
    return &*std::prev(it);
  }
  return &*it;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    return usage();
  }
  const std::filesystem::path session = argv[1];
  const std::filesystem::path out = argv[2];
  std::string model_name = "deeplabv3_mnv2_ade20k";
  std::filesystem::path model_file;
  int every = 1;
  int threads = 0;
  bool force = false;
  for (int i = 3; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--force") {
      force = true;
      continue;
    }
    if (i + 1 >= argc) {
      return usage();
    }
    const std::string value = argv[++i];
    if (arg == "--model") {
      model_name = value;
    } else if (arg == "--model-file") {
      model_file = value;
    } else if (arg == "--every") {
      every = std::max(1, std::atoi(value.c_str()));
    } else if (arg == "--threads") {
      threads = std::atoi(value.c_str());
    } else {
      return usage();
    }
  }

  vg::SegmentationModelInfo model;
  if (model_name == "deeplabv3_mnv2_ade20k") {
    model = vg::deeplabv3_mnv2_ade20k();
  } else if (model_name == "segformer_b0_ade20k") {
    model = vg::segformer_b0_ade20k();
  } else {
    std::cerr << "vg_segment: unknown model " << model_name << "\n";
    return usage();
  }
  if (model_file.empty()) {
    model_file = std::filesystem::path(VG_CORE_MODELS_DIR) / (model.name + ".onnx");
  }
  if (std::filesystem::exists(out) && !force) {
    std::cerr << "vg_segment: " << out.string() << " exists (--force to replace it)\n";
    return 1;
  }
  if (std::filesystem::exists(out) && std::filesystem::equivalent(out, session)) {
    std::cerr << "vg_segment: the output must be a new file\n";
    return 1;
  }

  try {
    const auto start = std::chrono::steady_clock::now();
    vg::Segmenter segmenter(vg::make_onnx_backend(model_file, threads), model);
    std::cout << model.name << " on " << segmenter.backend().name() << ", input "
              << segmenter.backend().input().width << " x " << segmenter.backend().input().height
              << "\n";
    const Index index = index_session(session);
    if (index.image_times.empty()) {
      throw std::runtime_error("no camera images in " + session.string());
    }

    vg::io::SessionReader reader(session);
    std::optional<vg::io::SessionWriter> writer;
    // Recent camera images, and depth images waiting for the image nearest them.
    std::deque<vg::io::CameraImage> images;
    std::deque<std::pair<vg::io::DepthImage, vg::Timestamp>> waiting;
    std::size_t depth_frames = 0;
    std::size_t labeled = 0;
    std::size_t undecodable = 0;
    double inference_ms = 0.0;
    std::array<std::size_t, vg::kSemanticClasses> counts{};

    const auto write = [&](const vg::io::SessionMessage& m) {
      if (!writer) {
        // A recording starts with its session info; make one if it doesn't.
        writer.emplace(out, vg::io::SessionInfo{});
      }
      writer->write(m);
    };
    const auto label_and_write = [&](vg::io::DepthImage depth, const vg::io::CameraImage* image) {
      depth.frame.labels.clear();
      if (image != nullptr) {
        if (const auto rgb = vg::io::decode_image(*image)) {
          const auto* pose = nearest(index.poses, depth.frame.timestamp,
                                     [](const vg::PoseStamped& p) { return p.timestamp; });
          const int rotation =
              pose != nullptr
                  ? vg::upright_rotation(pose->world_from_frame * index.body_from_camera)
                  : 0;
          auto result = segmenter.segment(rgb->view(), rotation, depth.frame.intrinsics.width,
                                          depth.frame.intrinsics.height);
          depth.frame.labels = std::move(result.labels);
          inference_ms += result.inference_ms;
          for (std::size_t c = 0; c < counts.size(); ++c) {
            counts[c] += result.counts[c];
          }
          ++labeled;
          if (labeled % 100 == 0) {
            std::cout << "  " << labeled << " labeled ("
                      << std::setprecision(3) << inference_ms / static_cast<double>(labeled)
                      << " ms per image)" << std::endl;
          }
        } else {
          ++undecodable;
        }
      }
      write(depth);
    };
    // Labels and writes the waiting depth images whose image has arrived.
    const auto drain = [&](bool all) {
      while (!waiting.empty()) {
        const vg::Timestamp want = waiting.front().second;
        const auto it = std::find_if(images.begin(), images.end(),
                                     [&](const auto& im) { return im.timestamp == want; });
        if (it == images.end() && !all) {
          break;
        }
        label_and_write(std::move(waiting.front().first), it == images.end() ? nullptr : &*it);
        waiting.pop_front();
      }
    };

    while (auto m = reader.next()) {
      if (const auto* info = std::get_if<vg::io::SessionInfo>(&*m)) {
        if (!writer) {
          writer.emplace(out, *info);
        }
        continue;
      }
      if (auto* depth = std::get_if<vg::io::DepthImage>(&*m)) {
        if (depth->camera != index.camera || depth_frames++ % static_cast<std::size_t>(every) != 0) {
          depth->frame.labels.clear();  // Not labeled by this model: no labels.
          write(*m);
          continue;
        }
        const vg::Timestamp want = *nearest(index.image_times, depth->frame.timestamp,
                                            [](vg::Timestamp t) { return t; });
        waiting.emplace_back(std::move(*depth), want);
        drain(false);
        continue;
      }
      if (const auto* image = std::get_if<vg::io::CameraImage>(&*m)) {
        if (image->camera == index.camera) {
          images.push_back(*image);
          if (images.size() > 30) {
            images.pop_front();
          }
          drain(false);
        }
      }
      // Everything else is copied as it comes; a depth image waiting for its
      // camera image follows a few milliseconds later in the file.
      write(*m);
    }
    drain(true);
    writer.reset();

    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::size_t total = 0;
    for (const auto n : counts) {
      total += n;
    }
    std::cout << std::fixed << std::setprecision(1) << labeled << " of " << depth_frames
              << " depth images labeled (" << undecodable << " images undecodable), "
              << (labeled ? inference_ms / static_cast<double>(labeled) : 0.0)
              << " ms inference per image, " << seconds << " s in all\n  classes:";
    for (std::size_t c = 0; c < counts.size(); ++c) {
      std::cout << ' ' << vg::semantic_class_name(static_cast<vg::SemanticClass>(c)) << ' '
                << (total ? 100.0 * static_cast<double>(counts[c]) / static_cast<double>(total)
                          : 0.0)
                << '%';
    }
    std::cout << "\n-> " << out.string() << "\n";
  } catch (const std::exception& e) {
    std::cerr << "vg_segment: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
