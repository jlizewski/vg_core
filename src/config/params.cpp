#include "vg_core/config/params.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>

namespace vg::config {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::string trim(const std::string& s) {
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

std::string lowercase(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

bool is_auto(const std::string& text) {
  const std::string t = lowercase(trim(text));
  return t.empty() || t == "auto" || t == "null" || t == "~";
}

[[noreturn]] void bad_value(const std::string& text, const char* expected) {
  throw ConfigError("'" + text + "' is not " + expected);
}

double parse_double(const std::string& text) {
  const std::string t = trim(text);
  std::istringstream in(t);
  in.imbue(std::locale::classic());
  double value = 0.0;
  in >> value;
  if (t.empty() || in.fail() || !in.eof() || !std::isfinite(value)) {
    bad_value(text, "a number");
  }
  return value;
}

long long parse_integer(const std::string& text, long long min, long long max) {
  const std::string t = trim(text);
  char* end = nullptr;
  errno = 0;
  const long long value = std::strtoll(t.c_str(), &end, 10);
  if (t.empty() || *end != '\0' || errno != 0) {
    bad_value(text, "a whole number");
  }
  if (value < min || value > max) {
    throw ConfigError(t + " is out of range (" + std::to_string(min) + " to " +
                      std::to_string(max) + ")");
  }
  return value;
}

bool parse_bool(const std::string& text) {
  const std::string t = lowercase(trim(text));
  if (t == "true" || t == "yes" || t == "on" || t == "1") return true;
  if (t == "false" || t == "no" || t == "off" || t == "0") return false;
  bad_value(text, "true or false");
}

std::vector<std::string> parse_list(const std::string& text) {
  std::string t = trim(text);
  if (!t.empty() && t.front() == '[') {
    if (t.back() != ']') {
      bad_value(text, "a list like [a, b]");
    }
    t = t.substr(1, t.size() - 2);
  }
  std::vector<std::string> items;
  std::istringstream in(t);
  std::string item;
  while (std::getline(in, item, ',')) {
    item = trim(item);
    if (!item.empty()) {
      items.push_back(item);
    }
  }
  return items;
}

std::string format(double value, int precision) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out.precision(precision);
  out << value;
  return out.str();
}

std::string format_list(const std::vector<std::string>& items) {
  std::string out = "[";
  for (std::size_t i = 0; i < items.size(); ++i) {
    out += (i > 0 ? ", " : "") + items[i];
  }
  return out + "]";
}

std::size_t edit_distance(const std::string& a, const std::string& b) {
  std::vector<std::size_t> row(b.size() + 1);
  for (std::size_t j = 0; j <= b.size(); ++j) row[j] = j;
  for (std::size_t i = 1; i <= a.size(); ++i) {
    std::size_t diagonal = row[0];
    row[0] = i;
    for (std::size_t j = 1; j <= b.size(); ++j) {
      const std::size_t above = row[j];
      row[j] = std::min({row[j] + 1, row[j - 1] + 1, diagonal + (a[i - 1] == b[j - 1] ? 0 : 1)});
      diagonal = above;
    }
  }
  return row[b.size()];
}

std::string last_component(const std::string& path) {
  const auto dot = path.rfind('.');
  return dot == std::string::npos ? path : path.substr(dot + 1);
}

std::vector<std::string> components(const std::string& path) {
  std::vector<std::string> parts;
  std::istringstream in(path);
  std::string part;
  while (std::getline(in, part, '.')) {
    parts.push_back(part);
  }
  return parts;
}

// Splits help text into lines of at most width characters.
std::vector<std::string> wrap(const std::string& text, std::size_t width) {
  std::vector<std::string> lines;
  std::istringstream in(text);
  std::string word;
  std::string line;
  while (in >> word) {
    if (!line.empty() && line.size() + 1 + word.size() > width) {
      lines.push_back(line);
      line.clear();
    }
    line += (line.empty() ? "" : " ") + word;
  }
  if (!line.empty()) {
    lines.push_back(line);
  }
  return lines;
}

// The lines of text, keeping blank ones; none for empty text.
std::vector<std::string> wrap_lines(const std::string& text) {
  std::vector<std::string> lines;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    lines.push_back(trim(line));
  }
  return lines;
}

}  // namespace

void ParamSet::section(const std::string& path, const std::string& help) {
  entries_.push_back({true, path, help, 0});
}

void ParamSet::add_param(const std::string& path, const std::string& help,
                         const std::string& flag, std::function<void(const std::string&)> set,
                         std::function<std::string()> get) {
  if (has(path)) {
    throw std::logic_error("setting " + path + " added twice");
  }
  entries_.push_back({false, path, help, params_.size()});
  params_.push_back({path, flag, help, std::move(set), std::move(get)});
}

void ParamSet::add(const std::string& path, double& value, const std::string& help,
                   const std::string& flag) {
  add_param(
      path, help, flag, [&value](const std::string& t) { value = parse_double(t); },
      [&value] { return format(value, 15); });
}

void ParamSet::add(const std::string& path, float& value, const std::string& help,
                   const std::string& flag) {
  add_param(
      path, help, flag,
      [&value](const std::string& t) { value = static_cast<float>(parse_double(t)); },
      [&value] { return format(value, 7); });
}

void ParamSet::add(const std::string& path, int& value, const std::string& help,
                   const std::string& flag) {
  add_param(
      path, help, flag,
      [&value](const std::string& t) {
        value = static_cast<int>(parse_integer(t, std::numeric_limits<int>::min(),
                                               std::numeric_limits<int>::max()));
      },
      [&value] { return std::to_string(value); });
}

void ParamSet::add(const std::string& path, bool& value, const std::string& help,
                   const std::string& flag) {
  add_param(
      path, help, flag, [&value](const std::string& t) { value = parse_bool(t); },
      [&value] { return std::string(value ? "true" : "false"); });
}

void ParamSet::add(const std::string& path, std::uint8_t& value, const std::string& help,
                   const std::string& flag) {
  add_param(
      path, help, flag,
      [&value](const std::string& t) { value = static_cast<std::uint8_t>(parse_integer(t, 0, 255)); },
      [&value] { return std::to_string(static_cast<int>(value)); });
}

void ParamSet::add(const std::string& path, std::optional<double>& value, const std::string& help,
                   const std::string& flag) {
  add_param(
      path, help, flag,
      [&value](const std::string& t) {
        value = is_auto(t) ? std::nullopt : std::optional<double>(parse_double(t));
      },
      [&value] { return value ? format(*value, 15) : std::string("auto"); });
}

void ParamSet::add(const std::string& path, std::optional<float>& value, const std::string& help,
                   const std::string& flag) {
  add_param(
      path, help, flag,
      [&value](const std::string& t) {
        value = is_auto(t) ? std::nullopt
                           : std::optional<float>(static_cast<float>(parse_double(t)));
      },
      [&value] { return value ? format(*value, 7) : std::string("auto"); });
}

void ParamSet::add(const std::string& path, std::optional<int>& value, const std::string& help,
                   const std::string& flag) {
  add_param(
      path, help, flag,
      [&value](const std::string& t) {
        value = is_auto(t) ? std::nullopt
                           : std::optional<int>(static_cast<int>(parse_integer(
                                 t, std::numeric_limits<int>::min(),
                                 std::numeric_limits<int>::max())));
      },
      [&value] { return value ? std::to_string(*value) : std::string("auto"); });
}

void ParamSet::add(const std::string& path, std::vector<std::string>& value,
                   const std::string& help, const std::string& flag) {
  add_param(
      path, help, flag, [&value](const std::string& t) { value = parse_list(t); },
      [&value] { return format_list(value); });
}

void ParamSet::add_degrees(const std::string& path, double& radians, const std::string& help,
                           const std::string& flag) {
  add_param(
      path, help, flag,
      [&radians](const std::string& t) { radians = parse_double(t) * kPi / 180.0; },
      [&radians] { return format(radians * 180.0 / kPi, 12); });
}

bool ParamSet::has(const std::string& path) const {
  return std::any_of(params_.begin(), params_.end(),
                     [&](const Param& p) { return p.path == path; });
}

const ParamSet::Param& ParamSet::find(const std::string& path) const {
  for (const Param& p : params_) {
    if (p.path == path) {
      return p;
    }
  }
  throw ConfigError("unknown setting " + path + suggest(path));
}

std::string ParamSet::suggest(const std::string& path) const {
  const std::string* best = nullptr;
  std::size_t best_distance = std::numeric_limits<std::size_t>::max();
  for (const Param& p : params_) {
    // A misspelled key, or the right key under the wrong section.
    const std::size_t d = std::min(edit_distance(path, p.path),
                                   last_component(path) == last_component(p.path)
                                       ? std::size_t{1}
                                       : std::numeric_limits<std::size_t>::max());
    if (d < best_distance) {
      best_distance = d;
      best = &p.path;
    }
  }
  if (best == nullptr || best_distance > std::max<std::size_t>(2, path.size() / 4)) {
    return "";
  }
  return " (did you mean " + *best + "?)";
}

void ParamSet::set(const std::string& path, const std::string& text) {
  const Param& param = find(path);
  try {
    param.set(text);
  } catch (const ConfigError& e) {
    throw ConfigError(path + ": " + e.what());
  }
}

std::string ParamSet::get(const std::string& path) const { return find(path).get(); }

std::vector<std::string> ParamSet::paths() const {
  std::vector<std::string> out;
  for (const Param& p : params_) {
    out.push_back(p.path);
  }
  return out;
}

void ParamSet::load_yaml(const std::string& text, const std::string& source) {
  YAML::Node root;
  try {
    root = YAML::Load(text);
  } catch (const YAML::Exception& e) {
    throw ConfigError(source + ": " + e.what());
  }
  if (root.IsNull()) {
    return;  // An empty file changes nothing.
  }
  if (!root.IsMap()) {
    throw ConfigError(source + ": expected settings (key: value), found something else");
  }
  const auto where = [&](const YAML::Node& node) {
    return source + ":" + std::to_string(node.Mark().line + 1) + ": ";
  };
  std::function<void(const YAML::Node&, const std::string&)> walk;
  walk = [&](const YAML::Node& map, const std::string& prefix) {
    for (const auto& item : map) {
      const std::string key = item.first.as<std::string>();
      const std::string path = prefix.empty() ? key : prefix + "." + key;
      const YAML::Node& value = item.second;
      if (value.IsMap()) {
        const bool known_section =
            std::any_of(params_.begin(), params_.end(),
                        [&](const Param& p) { return p.path.rfind(path + ".", 0) == 0; });
        if (!known_section) {
          throw ConfigError(where(item.first) + "unknown section " + path +
                            (has(path) ? " (it's a setting: give it a value)" : suggest(path)));
        }
        walk(value, path);
        continue;
      }
      if (!has(path)) {
        throw ConfigError(where(item.first) + "unknown setting " + path + suggest(path));
      }
      std::string text_value;
      if (value.IsSequence()) {
        std::vector<std::string> items;
        for (const auto& element : value) {
          items.push_back(element.as<std::string>());
        }
        text_value = format_list(items);
      } else if (!value.IsNull()) {
        text_value = value.Scalar();
      }
      try {
        set(path, text_value);
      } catch (const ConfigError& e) {
        throw ConfigError(where(item.first) + e.what());
      }
    }
  };
  walk(root, "");
}

void ParamSet::load_yaml_file(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw ConfigError("cannot open settings file " + path);
  }
  std::ostringstream text;
  text << in.rdbuf();
  load_yaml(text.str(), path);
}

void ParamSet::apply_args(const std::vector<std::string>& args) {
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string& arg = args[i];
    const auto value = [&]() -> const std::string& {
      if (i + 1 >= args.size()) {
        throw ConfigError(arg + " needs a value");
      }
      return args[++i];
    };
    if (arg == "--config") {
      load_yaml_file(value());
    } else if (arg == "--set") {
      const std::string& assignment = value();
      const auto eq = assignment.find('=');
      if (eq == std::string::npos) {
        throw ConfigError("--set takes PATH=VALUE, e.g. --set ground.max_slope=0.6");
      }
      set(trim(assignment.substr(0, eq)), assignment.substr(eq + 1));
    } else {
      const auto it = std::find_if(params_.begin(), params_.end(),
                                   [&](const Param& p) { return !p.flag.empty() && p.flag == arg; });
      if (it == params_.end()) {
        throw ConfigError("unknown option " + arg);
      }
      set(it->path, value());
    }
  }
}

void ParamSet::write_yaml(std::ostream& out, const std::string& header) const {
  for (const std::string& line : wrap_lines(header)) {
    out << (line.empty() ? "#" : "# " + line) << '\n';
  }
  std::vector<std::string> open;  // Sections currently written, outermost first.
  const auto open_to = [&](const std::vector<std::string>& target, const std::string& help) {
    std::size_t common = 0;
    while (common < open.size() && common < target.size() && open[common] == target[common]) {
      ++common;
    }
    open.resize(common);
    for (std::size_t i = common; i < target.size(); ++i) {
      const std::string indent(2 * i, ' ');
      if (i == 0) {
        out << '\n';
      }
      if (i + 1 == target.size()) {
        for (const std::string& line : wrap(help, 78 - indent.size())) {
          out << indent << "# " << line << '\n';
        }
      }
      out << indent << target[i] << ":\n";
      open.push_back(target[i]);
    }
  };
  for (const Entry& entry : entries_) {
    if (entry.is_section) {
      open_to(components(entry.path), entry.help);
      continue;
    }
    const Param& param = params_[entry.param];
    std::vector<std::string> parts = components(param.path);
    const std::string key = parts.back();
    parts.pop_back();
    open_to(parts, "");
    const std::string indent(2 * parts.size(), ' ');
    if (parts.empty()) {
      out << '\n';
    }
    for (const std::string& line : wrap(param.help, 78 - indent.size())) {
      out << indent << "# " << line << '\n';
    }
    out << indent << key << ": " << param.get() << '\n';
  }
}

std::string ParamSet::flags_help() const {
  std::ostringstream out;
  out << "settings (applied in order; see the YAML for what each does):\n"
         "  --config FILE          a YAML settings file (repeatable)\n"
         "  --set PATH=VALUE       any setting, e.g. --set ground.max_slope=0.6\n";
  for (const Param& p : params_) {
    if (p.flag.empty()) {
      continue;
    }
    std::string name = "  " + p.flag + " V";
    name.resize(std::max<std::size_t>(name.size() + 1, 25), ' ');
    const auto lines = wrap(p.help + " [" + p.path + "]", 74);
    for (std::size_t i = 0; i < lines.size(); ++i) {
      out << (i == 0 ? name : std::string(25, ' ')) << lines[i] << '\n';
    }
  }
  return out.str();
}

}  // namespace vg::config
