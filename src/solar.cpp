#include "vg_core/solar.hpp"

#include <algorithm>
#include <cmath>

namespace vg {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kSolarConstant = 1361.0;  // W/m^2 above the atmosphere.

double wrap_degrees(double a) {
  a = std::fmod(a, 360.0);
  return a < 0.0 ? a + 360.0 : a;
}

// Atmospheric refraction in degrees for a geometric elevation (NOAA
// approximation).
double refraction_deg(double elevation_deg) {
  if (elevation_deg > 85.0) {
    return 0.0;
  }
  const double te = std::tan(elevation_deg * kDeg);
  double arcsec;
  if (elevation_deg > 5.0) {
    arcsec = 58.1 / te - 0.07 / (te * te * te) + 0.000086 / std::pow(te, 5);
  } else if (elevation_deg > -0.575) {
    const double e = elevation_deg;
    arcsec = 1735.0 + e * (-518.2 + e * (103.4 + e * (-12.79 + e * 0.711)));
  } else {
    arcsec = -20.772 / te;
  }
  return arcsec / 3600.0;
}

}  // namespace

SunPosition sun_position(double unix_time_s, double latitude_deg, double longitude_deg) {
  const double julian_day = unix_time_s / 86400.0 + 2440587.5;
  const double t = (julian_day - 2451545.0) / 36525.0;  // Julian centuries since J2000.

  const double mean_longitude = wrap_degrees(280.46646 + t * (36000.76983 + t * 0.0003032));
  const double mean_anomaly = 357.52911 + t * (35999.05029 - 0.0001537 * t);
  const double eccentricity = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
  const double m = mean_anomaly * kDeg;
  const double center = std::sin(m) * (1.914602 - t * (0.004817 + 0.000014 * t)) +
                        std::sin(2.0 * m) * (0.019993 - 0.000101 * t) +
                        std::sin(3.0 * m) * 0.000289;
  const double true_longitude = mean_longitude + center;
  const double omega = (125.04 - 1934.136 * t) * kDeg;
  const double apparent_longitude = (true_longitude - 0.00569 - 0.00478 * std::sin(omega)) * kDeg;
  const double mean_obliquity =
      23.0 + (26.0 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60.0) / 60.0;
  const double obliquity = (mean_obliquity + 0.00256 * std::cos(omega)) * kDeg;
  const double declination = std::asin(std::sin(obliquity) * std::sin(apparent_longitude));

  // Equation of time, in minutes.
  const double y = std::pow(std::tan(obliquity / 2.0), 2);
  const double l0 = mean_longitude * kDeg;
  const double e = eccentricity;
  const double equation_of_time =
      4.0 / kDeg *
      (y * std::sin(2.0 * l0) - 2.0 * e * std::sin(m) +
       4.0 * e * y * std::sin(m) * std::cos(2.0 * l0) - 0.5 * y * y * std::sin(4.0 * l0) -
       1.25 * e * e * std::sin(2.0 * m));

  const double minutes_of_day = std::fmod(unix_time_s, 86400.0) / 60.0;
  const double true_solar_time = minutes_of_day + equation_of_time + 4.0 * longitude_deg;
  const double hour_angle = (true_solar_time / 4.0 - 180.0) * kDeg;

  // Direction to the sun in East-North-Up from the local hour angle.
  const double lat = latitude_deg * kDeg;
  const double cd = std::cos(declination);
  const double sd = std::sin(declination);
  const Eigen::Vector3d geometric(-cd * std::sin(hour_angle),
                                  sd * std::cos(lat) - cd * std::cos(hour_angle) * std::sin(lat),
                                  sd * std::sin(lat) + cd * std::cos(hour_angle) * std::cos(lat));

  SunPosition sun;
  sun.azimuth_deg = wrap_degrees(std::atan2(geometric.x(), geometric.y()) / kDeg);
  const double geometric_elevation = std::asin(std::clamp(geometric.z(), -1.0, 1.0)) / kDeg;
  sun.elevation_deg = geometric_elevation + refraction_deg(geometric_elevation);
  const double az = sun.azimuth_deg * kDeg;
  const double el = sun.elevation_deg * kDeg;
  sun.direction_enu =
      Eigen::Vector3d(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el));
  return sun;
}

double clear_sky_direct_irradiance(double elevation_deg) {
  if (elevation_deg <= 0.0) {
    return 0.0;
  }
  const double zenith = 90.0 - elevation_deg;
  const double air_mass =
      1.0 / (std::cos(zenith * kDeg) + 0.50572 * std::pow(96.07995 - zenith, -1.6364));
  return kSolarConstant * std::pow(0.7, std::pow(air_mass, 0.678));
}

double unix_time_of_year_start(int year) {
  // Days from 1970-01-01 to year-01-01 (Howard Hinnant's days_from_civil).
  const int y = year - 1;  // January counts as part of the previous March-based year.
  const int era = (y >= 0 ? y : y - 399) / 400;
  const int yoe = y - era * 400;
  const int doy = 306;  // Days from March 1 to January 1.
  const int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const long long days = static_cast<long long>(era) * 146097 + doe - 719468;
  return static_cast<double>(days) * 86400.0;
}

int utc_year(double unix_time_s) {
  // Howard Hinnant's civil_from_days, keeping only the year.
  const long long z = static_cast<long long>(std::floor(unix_time_s / 86400.0)) + 719468;
  const long long era = (z >= 0 ? z : z - 146096) / 146097;
  const long long doe = z - era * 146097;
  const long long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const long long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const long long mp = (5 * doy + 2) / 153;  // Month, counted from March.
  const long long year = yoe + era * 400 + (mp >= 10 ? 1 : 0);
  return static_cast<int>(year);
}

}  // namespace vg
