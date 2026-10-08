#include "vg_core/io/mcap_repair.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vg_core/io/session_reader.hpp"
#include "vg_core/io/session_writer.hpp"

namespace {

constexpr vg::Timestamp kMs = 1'000'000;

// Unique temp file, removed when the test ends.
class TempFile {
 public:
  explicit TempFile(const std::string& name)
      : path_(std::filesystem::temp_directory_path() /
              (name + "_" +
               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
               ".mcap")) {}
  ~TempFile() {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }
  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

vg::io::SessionInfo test_info() {
  vg::io::SessionInfo info;
  info.producer = "vg_core tests";
  info.start_time = 0;
  return info;
}

// Writes a session of IMU samples 1 ms apart, large enough to span several
// chunks.
void write_imu_session(const std::filesystem::path& path, int samples) {
  vg::io::SessionWriter writer(path, test_info());
  for (int i = 1; i <= samples; ++i) {
    vg::ImuSample imu;
    imu.timestamp = i * kMs;
    imu.angular_velocity = {0.1 * i, 0.0, 0.0};
    writer.write(imu);
  }
}

std::vector<vg::Timestamp> imu_times(const std::filesystem::path& path, bool* indexed) {
  vg::io::SessionReader reader(path);
  *indexed = reader.indexed();
  std::vector<vg::Timestamp> times;
  while (auto m = reader.next()) {
    if (std::holds_alternative<vg::ImuSample>(*m)) {
      times.push_back(vg::io::timestamp_of(*m));
    }
  }
  return times;
}

TEST(McapRepair, IndexesFileThatWasNeverClosed) {
  TempFile in("vg_repair_unclosed");
  TempFile out("vg_repair_unclosed_out");
  write_imu_session(in.path(), 100);
  // Drop the footer, as when the recorder is killed before close().
  std::filesystem::resize_file(in.path(), std::filesystem::file_size(in.path()) - 37);

  const vg::io::McapRepairResult result = vg::io::repair_mcap(in.path(), out.path());
  EXPECT_EQ(result.messages, 101u);  // Session info and every sample.
  EXPECT_EQ(result.channels, 2u);
  EXPECT_EQ(result.metadata, 1u);
  EXPECT_EQ(result.end_time, 100 * kMs);

  bool indexed = false;
  const std::vector<vg::Timestamp> times = imu_times(out.path(), &indexed);
  EXPECT_TRUE(indexed);
  ASSERT_EQ(times.size(), 100u);
  EXPECT_EQ(times.front(), 1 * kMs);
  EXPECT_EQ(times.back(), 100 * kMs);
}

TEST(McapRepair, KeepsEverythingBeforeACutOffChunk) {
  TempFile in("vg_repair_cut");
  TempFile out("vg_repair_cut_out");
  constexpr int kSamples = 50'000;  // Several chunks.
  write_imu_session(in.path(), kSamples);
  // Cut the file in the middle, through a chunk.
  std::filesystem::resize_file(in.path(), std::filesystem::file_size(in.path()) / 2);

  const vg::io::McapRepairResult result = vg::io::repair_mcap(in.path(), out.path());
  EXPECT_FALSE(result.complete);
  EXPECT_FALSE(result.problem.empty());

  bool indexed = false;
  const std::vector<vg::Timestamp> times = imu_times(out.path(), &indexed);
  EXPECT_TRUE(indexed);
  // Some whole chunks survive, and they are exactly the first samples written.
  ASSERT_GT(times.size(), 0u);
  ASSERT_LT(times.size(), static_cast<std::size_t>(kSamples));
  for (std::size_t i = 0; i < times.size(); ++i) {
    ASSERT_EQ(times[i], static_cast<vg::Timestamp>(i + 1) * kMs);
  }
}

TEST(McapRepair, ClosedFileCopiesUnchanged) {
  TempFile in("vg_repair_closed");
  TempFile out("vg_repair_closed_out");
  write_imu_session(in.path(), 10);
  const vg::io::McapRepairResult result = vg::io::repair_mcap(in.path(), out.path());
  EXPECT_TRUE(result.complete);
  EXPECT_EQ(result.messages, 11u);
  bool indexed = false;
  EXPECT_EQ(imu_times(out.path(), &indexed).size(), 10u);
}

TEST(McapRepair, RefusesToOverwriteInput) {
  TempFile in("vg_repair_same");
  write_imu_session(in.path(), 1);
  EXPECT_THROW(vg::io::repair_mcap(in.path(), in.path()), std::runtime_error);
}

TEST(McapRepair, RejectsNonMcapInput) {
  TempFile in("vg_repair_text");
  TempFile out("vg_repair_text_out");
  std::ofstream(in.path()) << "not an mcap file";
  EXPECT_THROW(vg::io::repair_mcap(in.path(), out.path()), std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(out.path()));
}

}  // namespace
