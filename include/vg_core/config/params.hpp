#pragma once

// A registry of tunable settings, so one list drives the YAML settings files,
// the command-line flags, their help, and the commented defaults file.
//
// Each setting has a path in the YAML ("ground.max_slope" is `max_slope:`
// under `ground:`), is bound to the variable it sets, and may have a short
// command-line flag ("--max-slope"). Optional settings take `auto` (or an
// empty / null value) to leave the choice to the code.
//
// Settings are applied in order, later ones winning: the defaults the bound
// variables start with, then YAML files, then command-line flags. A YAML
// file only needs the settings it changes; an unknown key is an error (with
// the nearest known key suggested), so a typo can't silently do nothing.

#include <cstdint>
#include <functional>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace vg::config {

// A bad setting: unknown, or a value that doesn't parse. The message names
// the setting and, for files, where it is.
class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class ParamSet {
 public:
  // Starts a YAML section (e.g. "build.depth"), with a comment for it in
  // the written YAML. The settings added after it, until the next section,
  // should be in it.
  void section(const std::string& path, const std::string& help);

  // Binds the setting at path to value. flag is an optional command-line
  // alias such as "--max-slope".
  void add(const std::string& path, double& value, const std::string& help,
           const std::string& flag = {});
  void add(const std::string& path, float& value, const std::string& help,
           const std::string& flag = {});
  void add(const std::string& path, int& value, const std::string& help,
           const std::string& flag = {});
  void add(const std::string& path, bool& value, const std::string& help,
           const std::string& flag = {});
  void add(const std::string& path, std::uint8_t& value, const std::string& help,
           const std::string& flag = {});
  // `auto` leaves these unset.
  void add(const std::string& path, std::optional<double>& value, const std::string& help,
           const std::string& flag = {});
  void add(const std::string& path, std::optional<float>& value, const std::string& help,
           const std::string& flag = {});
  void add(const std::string& path, std::optional<int>& value, const std::string& help,
           const std::string& flag = {});
  // A list of names, e.g. [voxels, surface, ground].
  void add(const std::string& path, std::vector<std::string>& value, const std::string& help,
           const std::string& flag = {});
  // An angle given in degrees, kept in radians.
  void add_degrees(const std::string& path, double& radians, const std::string& help,
                   const std::string& flag = {});

  // Sets a setting from its text as in YAML ("0.5", "true", "auto", "[a, b]"
  // or "a,b"). Throws ConfigError if there is no such setting or the text
  // doesn't parse.
  void set(const std::string& path, const std::string& text);
  // The setting's current value, as YAML text.
  std::string get(const std::string& path) const;
  bool has(const std::string& path) const;
  // Every setting's path, in order.
  std::vector<std::string> paths() const;

  // Applies a YAML document of settings. source names it in errors (e.g.
  // the file name). Throws ConfigError.
  void load_yaml(const std::string& text, const std::string& source = "<yaml>");
  void load_yaml_file(const std::string& path);

  // Applies command-line arguments, in order:
  //   --config FILE     a YAML settings file
  //   --set PATH=VALUE  any setting
  //   --FLAG VALUE      a setting's short flag
  // Throws ConfigError on anything else.
  void apply_args(const std::vector<std::string>& args);

  // Writes every setting as YAML, each with its help as a comment, after
  // header (each line of which becomes a comment).
  void write_yaml(std::ostream& out, const std::string& header = {}) const;

  // Help for apply_args(): the generic options and every short flag.
  std::string flags_help() const;

 private:
  struct Param {
    std::string path;
    std::string flag;
    std::string help;
    std::function<void(const std::string&)> set;
    std::function<std::string()> get;
  };
  struct Entry {
    bool is_section = false;
    std::string path;  // Section path, or the param's path.
    std::string help;
    std::size_t param = 0;  // Index into params_, for a param.
  };

  void add_param(const std::string& path, const std::string& help, const std::string& flag,
                 std::function<void(const std::string&)> set, std::function<std::string()> get);
  const Param& find(const std::string& path) const;
  std::string suggest(const std::string& path) const;

  std::vector<Param> params_;
  std::vector<Entry> entries_;
};

}  // namespace vg::config
