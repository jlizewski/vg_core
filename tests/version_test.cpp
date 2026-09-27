#include "vg_core/version.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(Version, StringMatchesComponents) {
  const auto v = vg::version();
  const std::string expected =
      std::to_string(v.major) + "." + std::to_string(v.minor) + "." + std::to_string(v.patch);
  EXPECT_EQ(vg::version_string(), expected);
}
