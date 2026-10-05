#pragma once

// Position of the sun in the sky, and clear-sky direct sunlight.
//
// Uses the NOAA solar position algorithm (Meeus, "Astronomical Algorithms"),
// accurate to about 0.01 degrees for years 1800-2100, with atmospheric
// refraction applied near the horizon.

#include <Eigen/Core>
#include <cstdint>

namespace vg {

struct SunPosition {
  // Degrees clockwise from true north (90 = east).
  double azimuth_deg = 0.0;
  // Degrees above the horizon, refraction corrected. Negative below it.
  double elevation_deg = 0.0;
  // Unit vector toward the sun in East-North-Up.
  Eigen::Vector3d direction_enu = Eigen::Vector3d::UnitZ();
};

// Sun position at a UTC time (seconds since the Unix epoch) seen from a WGS84
// latitude and longitude in degrees.
SunPosition sun_position(double unix_time_s, double latitude_deg, double longitude_deg);

// Clear-sky direct normal irradiance in W/m^2 for a sun at elevation_deg: the
// sunlight falling on a surface facing the sun, after the atmosphere (Meinel
// model with the Kasten-Young air mass). 0 when the sun is below the horizon.
double clear_sky_direct_irradiance(double elevation_deg);

// UTC seconds since the Unix epoch of midnight starting January 1 of year.
double unix_time_of_year_start(int year);

// The UTC calendar year containing a time (seconds since the Unix epoch).
int utc_year(double unix_time_s);

}  // namespace vg
