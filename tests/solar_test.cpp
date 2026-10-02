#include "vg_core/solar.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

double azimuth_difference(double a, double b) {
  const double d = std::fmod(std::abs(a - b), 360.0);
  return d > 180.0 ? 360.0 - d : d;
}

TEST(Solar, YearStart) {
  EXPECT_EQ(vg::unix_time_of_year_start(1970), 0.0);
  EXPECT_EQ(vg::unix_time_of_year_start(2000), 946684800.0);
  EXPECT_EQ(vg::unix_time_of_year_start(2026), 1767225600.0);
  EXPECT_EQ(vg::unix_time_of_year_start(1969), -31536000.0);
  for (const int year : {1900, 1969, 1970, 2000, 2024, 2026, 2100}) {
    const double start = vg::unix_time_of_year_start(year);
    EXPECT_EQ(vg::utc_year(start), year);
    EXPECT_EQ(vg::utc_year(start - 1.0), year - 1);
    EXPECT_EQ(vg::utc_year(start + 200 * 86400.0), year);
  }
}

TEST(Solar, MatchesSpaReferenceCase) {
  // NREL SPA paper (Reda & Andreas 2004) example: Golden, Colorado,
  // 2003-10-17 12:30:30 local time (UTC-7). Zenith 50.11162, azimuth 194.34024.
  const double t =
      vg::unix_time_of_year_start(2003) + 289 * 86400.0 + 19 * 3600.0 + 30 * 60.0 + 30.0;
  const auto sun = vg::sun_position(t, 39.742476, -105.1786);
  EXPECT_NEAR(sun.elevation_deg, 90.0 - 50.11162, 0.05);
  EXPECT_NEAR(azimuth_difference(sun.azimuth_deg, 194.34024), 0.0, 0.05);
  EXPECT_NEAR(sun.direction_enu.norm(), 1.0, 1e-12);
  // South-southwest and up.
  EXPECT_LT(sun.direction_enu.x(), 0.0);
  EXPECT_LT(sun.direction_enu.y(), 0.0);
  EXPECT_GT(sun.direction_enu.z(), 0.0);
}

TEST(Solar, DayAndNight) {
  // Greenwich at the June solstice: high at noon, below the horizon at midnight.
  const double day = vg::unix_time_of_year_start(2026) + 171 * 86400.0;
  const auto noon = vg::sun_position(day + 12 * 3600.0, 51.4769, 0.0);
  EXPECT_NEAR(noon.elevation_deg, 90.0 - 51.4769 + 23.44, 0.3);
  EXPECT_NEAR(azimuth_difference(noon.azimuth_deg, 180.0), 0.0, 1.0);
  const auto midnight = vg::sun_position(day, 51.4769, 0.0);
  EXPECT_LT(midnight.elevation_deg, 0.0);
  // Morning sun is in the east.
  const auto morning = vg::sun_position(day + 7 * 3600.0, 51.4769, 0.0);
  EXPECT_GT(morning.direction_enu.x(), 0.5);
}

TEST(Solar, ClearSkyIrradiance) {
  EXPECT_EQ(vg::clear_sky_direct_irradiance(-5.0), 0.0);
  EXPECT_EQ(vg::clear_sky_direct_irradiance(0.0), 0.0);
  EXPECT_NEAR(vg::clear_sky_direct_irradiance(90.0), 953.0, 5.0);
  EXPECT_LT(vg::clear_sky_direct_irradiance(10.0), vg::clear_sky_direct_irradiance(30.0));
  EXPECT_LT(vg::clear_sky_direct_irradiance(30.0), vg::clear_sky_direct_irradiance(60.0));
}

}  // namespace
