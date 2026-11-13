#include "vg_core/site.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

#include <Eigen/Geometry>

#include "vg_core/height_map_io.hpp"
#include "vg_core/map_package.hpp"
#include "vg_core/voxel_map_io.hpp"

namespace vg {
namespace {

constexpr double kPi = 3.14159265358979323846;
// Cell size of the site's maps when no visit has any (m).
constexpr double kDefaultCell = 0.1524;
constexpr const char* kHeader =
    "package,recorded,yaw_deg,x,y,z,used_gps,peak,rival_ratio,overlap,inliers_10cm,inliers_30cm,"
    "median_m";

// The site's maps: file, and whether its values are heights (moved by the
// visit's z shift) rather than classes or heights above the terrain.
constexpr std::array<std::pair<const char*, bool>, 5> kMaps = {{
    {package_file::kSurface, true},
    {package_file::kGround, true},
    {package_file::kTerrain, true},
    {package_file::kFeatures, false},
    {package_file::kSemantic, false},
}};

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  std::istringstream in(line);
  while (std::getline(in, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

double number(const std::string& text) {
  std::istringstream in(text);
  in.imbue(std::locale::classic());
  double value = 0.0;
  if (!(in >> value)) {
    throw std::runtime_error("site: bad number " + text + " in " + std::string(site_file::kVisits));
  }
  return value;
}

double seconds_since(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

// A visit's maps in the site frame, the latest visit first per cell.
struct Layered {
  HeightMap latest;
  HeightMap previous;  // The value from the visit before, where one saw it too.
  std::vector<int> visit;  // 1-based visit of each latest value, 0 for none.
};

Layered resample(const std::vector<std::pair<HeightMap, MapTransform>>& maps,
                 bool heights) {
  Layered out;
  // The site grid: the first map's cell size, covering every map.
  double cell = 0.0;
  Eigen::Vector2d lo = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d hi = -lo;
  for (const auto& [map, t] : maps) {
    if (map.width == 0 || map.height == 0) {
      continue;
    }
    if (cell == 0.0) {
      cell = map.cell_size;
    }
    const Eigen::Vector2d size = Eigen::Vector2d(map.width, map.height) * map.cell_size;
    for (const Eigen::Vector2d corner :
         {Eigen::Vector2d(0, 0), Eigen::Vector2d(size.x(), 0), Eigen::Vector2d(0, size.y()),
          size}) {
      const Eigen::Vector2d p = t.apply_xy(map.origin + corner);
      lo = lo.cwiseMin(p);
      hi = hi.cwiseMax(p);
    }
  }
  if (cell == 0.0) {
    out.latest.cell_size = out.previous.cell_size = kDefaultCell;
    return out;
  }
  HeightMap& latest = out.latest;
  latest.cell_size = cell;
  latest.origin = (lo / cell).array().floor().matrix() * cell;
  latest.width = static_cast<int>(std::ceil((hi.x() - latest.origin.x()) / cell));
  latest.height = static_cast<int>(std::ceil((hi.y() - latest.origin.y()) / cell));
  latest.heights.assign(static_cast<std::size_t>(latest.width) *
                            static_cast<std::size_t>(latest.height),
                        std::numeric_limits<float>::quiet_NaN());
  out.previous = latest;
  out.visit.assign(latest.heights.size(), 0);

  std::vector<MapTransform> inverses;
  for (const auto& entry : maps) {
    inverses.push_back(entry.second.inverse());
  }
  for (int y = 0; y < latest.height; ++y) {
    for (int x = 0; x < latest.width; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(latest.width) +
                            static_cast<std::size_t>(x);
      const Eigen::Vector2d center = latest.origin + Eigen::Vector2d(x + 0.5, y + 0.5) * cell;
      int found = 0;
      for (std::size_t k = maps.size(); k-- > 0 && found < 2;) {
        const HeightMap& map = maps[k].first;
        if (map.width == 0) {
          continue;
        }
        const Eigen::Vector2d f = (inverses[k].apply_xy(center) - map.origin) / map.cell_size;
        const int mx = static_cast<int>(std::floor(f.x()));
        const int my = static_cast<int>(std::floor(f.y()));
        if (mx < 0 || my < 0 || mx >= map.width || my >= map.height || !map.has_data(mx, my)) {
          continue;
        }
        const float value =
            map.at(mx, my) + (heights ? static_cast<float>(maps[k].second.translation.z()) : 0.0f);
        if (found == 0) {
          latest.heights[i] = value;
          out.visit[i] = static_cast<int>(k) + 1;
        } else {
          out.previous.heights[i] = value;
        }
        ++found;
      }
    }
  }
  return out;
}

std::string format_fit(const SiteVisit& v) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out.precision(9);
  const MapTransform& t = v.site_from_map;
  out << v.package.generic_string() << ',' << v.recorded << ',' << t.yaw * 180.0 / kPi << ','
      << t.translation.x() << ',' << t.translation.y() << ',' << t.translation.z();
  if (v.fit) {
    out.precision(4);
    const RegistrationResult& f = *v.fit;
    out << ',' << (f.used_prior ? 1 : 0) << ',' << f.peak << ',' << f.rival_ratio << ','
        << f.overlap << ',' << f.inliers_10cm << ',' << f.inliers_30cm << ','
        << f.median_distance;
  } else {
    out << ",,,,,,,";
  }
  return out.str();
}

}  // namespace

std::optional<std::string> recorded_time(const std::string& session) {
  const auto digits = [&](std::size_t at, std::size_t n) {
    return std::all_of(session.begin() + static_cast<std::ptrdiff_t>(at),
                       session.begin() + static_cast<std::ptrdiff_t>(at + n),
                       [](char c) { return c >= '0' && c <= '9'; });
  };
  for (std::size_t i = 0; i + 15 <= session.size(); ++i) {
    if (digits(i, 8) && session[i + 8] == '-' && digits(i + 9, 6)) {
      return session.substr(i, 15);
    }
  }
  return std::nullopt;
}

Site::Site(std::filesystem::path dir) : dir_(std::move(dir)) {
  std::filesystem::create_directories(dir_);
  std::ifstream in(dir_ / site_file::kVisits);
  std::string line;
  if (!std::getline(in, line)) {
    return;
  }
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    const auto f = split(line);
    if (f.size() != 13) {
      throw std::runtime_error("site: bad line in " + (dir_ / site_file::kVisits).string() +
                               ": " + line);
    }
    SiteVisit v;
    v.package = std::filesystem::path(f[0]);
    v.recorded = f[1];
    v.site_from_map.yaw = number(f[2]) * kPi / 180.0;
    v.site_from_map.translation = {number(f[3]), number(f[4]), number(f[5])};
    if (!f[6].empty()) {
      RegistrationResult fit;
      fit.reference_from_map = v.site_from_map;
      fit.used_prior = f[6] == "1";
      fit.peak = number(f[7]);
      fit.rival_ratio = number(f[8]);
      fit.overlap = number(f[9]);
      fit.inliers_10cm = number(f[10]);
      fit.inliers_30cm = number(f[11]);
      fit.median_distance = number(f[12]);
      fit.accepted = true;
      v.fit = fit;
    }
    visits_.push_back(v);
  }
}

std::filesystem::path Site::package_dir(const SiteVisit& visit) const {
  return visit.package.is_absolute() ? visit.package : dir_ / visit.package;
}

void Site::save() const {
  std::ofstream out(dir_ / site_file::kVisits, std::ios::binary);
  out << kHeader << '\n';
  for (const SiteVisit& v : visits_) {
    out << format_fit(v) << '\n';
  }
  if (!out) {
    throw std::runtime_error("site: cannot write " + (dir_ / site_file::kVisits).string());
  }
}

RegistrationResult Site::add(const std::filesystem::path& package, const RegistrationConfig& config,
                             std::ostream& log, bool force) {
  const auto start = std::chrono::steady_clock::now();
  if (package.generic_string().find(',') != std::string::npos) {
    throw std::runtime_error("site: package paths can't contain commas: " + package.string());
  }
  for (const char* file : {package_file::kFeatures, package_file::kTerrain}) {
    if (!std::filesystem::is_regular_file(package / file)) {
      throw std::runtime_error("site: " + (package / file).string() +
                               " is missing (run the surface, ground and terrain stages)");
    }
  }
  MapPackage map(package, log);
  RegistrationInput input;
  input.points = map.occupied_centers();
  input.features = load_height_map(package / package_file::kFeatures);
  input.terrain = load_height_map(package / package_file::kTerrain);

  SiteVisit visit;
  // Kept relative to the site when it's inside it, so the site can move.
  const auto absolute = std::filesystem::absolute(package).lexically_normal();
  const auto relative = absolute.lexically_relative(std::filesystem::absolute(dir_));
  visit.package = !relative.empty() && *relative.begin() != ".." ? relative : absolute;
  visit.recorded = recorded_time(map.metadata().session).value_or("");
  for (const SiteVisit& v : visits_) {
    if (std::filesystem::exists(package_dir(v)) &&
        std::filesystem::equivalent(package_dir(v), package)) {
      throw std::runtime_error("site: " + package.string() + " is already a visit");
    }
  }

  RegistrationResult result;
  if (visits_.empty()) {
    result.accepted = true;
    log << "site: " << package.string() << " is the first visit: it sets the site's frame"
        << std::endl;
  } else {
    // The reference: every visit so far, in the site frame.
    RegistrationInput reference;
    std::vector<std::pair<HeightMap, MapTransform>> features;
    std::vector<std::pair<HeightMap, MapTransform>> terrains;
    // A prior from GPS, through the visit with the best GPS heading: the
    // site's yaw of that visit plus the turn from its heading to this
    // package's, and this package's origin placed from that visit's by their
    // GPS positions.
    RegistrationPrior prior;
    const auto& geo = map.metadata().geo;
    for (const SiteVisit& v : visits_) {
      const auto dir = package_dir(v);
      MapPackage earlier(dir, log);
      const auto& other = earlier.metadata().geo;
      if (config.use_gps && geo && other && std::isfinite(geo->heading_sigma) &&
          std::isfinite(other->heading_sigma)) {
        const double sigma = std::hypot(geo->heading_sigma, other->heading_sigma);
        if (!prior.yaw || sigma < prior.yaw_sigma) {
          prior.yaw = v.site_from_map.yaw + geo->heading - other->heading;
          prior.yaw_sigma = sigma;
          // This package's origin east and north of the visit's (m), then
          // in the visit's frame (enu = Rz(heading) world), then the site's.
          constexpr double kMetersPerDegree = 111'320.0;
          const Eigen::Vector2d enu(
              (geo->longitude - other->longitude) * kMetersPerDegree *
                  std::cos(other->latitude * kPi / 180.0),
              (geo->latitude - other->latitude) * kMetersPerDegree);
          const Eigen::Vector2d world = Eigen::Rotation2Dd(-other->heading) * enu;
          prior.origin = v.site_from_map.apply_xy(world);
          prior.origin_sigma = config.gps_position_sigma;
        }
      }
      for (const Eigen::Vector3d& p : earlier.occupied_centers()) {
        reference.points.push_back(v.site_from_map.apply(p));
      }
      features.emplace_back(load_height_map(dir / package_file::kFeatures), v.site_from_map);
      terrains.emplace_back(load_height_map(dir / package_file::kTerrain), v.site_from_map);
    }
    reference.features = resample(features, false).latest;
    reference.terrain = resample(terrains, true).latest;

    result = register_map(reference, input, config, prior);
    visit.site_from_map = result.reference_from_map;
    visit.fit = result;
    const MapTransform& t = result.reference_from_map;
    log << "site: " << package.string() << " -> yaw " << t.yaw * 180.0 / kPi << " deg, shift ("
        << t.translation.x() << ", " << t.translation.y() << ", " << t.translation.z()
        << ") m; heading " << (result.used_prior ? "searched near GPS" : "searched all round")
        << ", peak " << result.peak;
    if (result.rival_ratio > 0.0) {
      log << ", lines up " << result.rival_ratio << "x as well as turned "
          << result.rival_yaw * 180.0 / kPi << " deg";
    }
    log << "; overlap " << 100.0 * result.overlap
        << "%, of it within 10 cm " << 100.0 * result.inliers_10cm << "%, within 30 cm "
        << 100.0 * result.inliers_30cm << "%, median " << 100.0 * result.median_distance
        << " cm (" << seconds_since(start) << " s)" << std::endl;
    if (!result.accepted) {
      log << "site: " << (force ? "adding anyway (forced): " : "not added: ") << result.problem
          << std::endl;
      if (!force) {
        return result;
      }
    }
  }

  // Oldest first; visits with no known time keep the order added, last.
  const auto at = std::find_if(visits_.begin(), visits_.end(), [&](const SiteVisit& v) {
    return !visit.recorded.empty() && (v.recorded.empty() || v.recorded > visit.recorded);
  });
  visits_.insert(at, visit);
  save();
  return result;
}

void Site::make_maps(std::ostream& log, const CanopyConfig& canopy_config) const {
  const auto start = std::chrono::steady_clock::now();
  const auto out = dir_ / site_file::kCurrent;
  std::filesystem::create_directories(out);
  for (const auto& [file, heights] : kMaps) {
    std::vector<std::pair<HeightMap, MapTransform>> maps;
    for (const SiteVisit& v : visits_) {
      const auto path = package_dir(v) / file;
      maps.emplace_back(std::filesystem::is_regular_file(path) ? load_height_map(path) : HeightMap{},
                        v.site_from_map);
    }
    const Layered layered = resample(maps, heights);
    save_height_map(layered.latest, out / file);
    if (std::string(file) != package_file::kSurface) {
      continue;
    }
    HeightMap visit = layered.latest;
    HeightMap changes = layered.latest;
    for (std::size_t i = 0; i < visit.heights.size(); ++i) {
      visit.heights[i] = layered.visit[i] > 0 ? static_cast<float>(layered.visit[i])
                                              : std::numeric_limits<float>::quiet_NaN();
      changes.heights[i] = layered.latest.heights[i] - layered.previous.heights[i];
    }
    save_height_map(visit, out / site_file::kVisit);
    save_height_map(changes, out / site_file::kChanges);
  }
  // The canopy: every visit's sky evidence in the site frame, together.
  std::optional<SkyEvidence> sky;
  for (const SiteVisit& v : visits_) {
    const auto path = package_dir(v) / package_file::kSky;
    if (std::filesystem::is_regular_file(path)) {
      const SkyEvidence evidence = load_sky(path);
      if (!sky) {
        sky.emplace(evidence.cell_size());
      }
      sky->add(evidence, v.site_from_map);
    }
  }
  if (sky) {
    const Canopy canopy =
        make_canopy(*sky, load_height_map(out / package_file::kTerrain), canopy_config);
    save_height_map(canopy.bottom, out / package_file::kCanopy);
    save_height_map(canopy.top, out / package_file::kCanopyTop);
    save_voxels_ply(canopy.cells, canopy.cell_size, out / package_file::kCanopyCells);
    log << "site: canopy " << canopy.cells.size() << " cells of " << canopy.observed
        << " judged, from " << sky->frames() << " labeled frames" << std::endl;
  }
  log << "site: " << visits_.size() << " visits -> " << site_file::kCurrent << "/ ("
      << seconds_since(start) << " s)" << std::endl;
}

}  // namespace vg
