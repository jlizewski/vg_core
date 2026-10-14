// vg_config: show or check mapping settings (the YAML files vg replay and vg map take).
//
//   vg_config defaults                  every setting with its default and help, as YAML
//   vg_config show [settings]           the settings that result from --config files,
//                                       --set and flags (as vg replay would use them)
//   vg_config check FILE...             check settings files for unknown keys and bad values
//
// config/mapping.yaml in the repository is `vg_config defaults`.

#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "vg_core/config/mapping_config.hpp"

namespace {

int usage() {
  std::cerr << "usage: vg_config defaults\n"
               "       vg_config show [--config FILE] [--set PATH=VALUE] [flags]\n"
               "       vg_config check FILE...\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty()) {
    return usage();
  }
  const std::string& command = args[0];

  if (command == "defaults" && args.size() == 1) {
    vg::config::MappingConfig config;
    vg::config::bind(config).write_yaml(std::cout, vg::config::kDefaultsHeader);
    return 0;
  }
  if (command == "show") {
    vg::config::MappingConfig config;
    vg::config::ParamSet params = vg::config::bind(config);
    try {
      params.apply_args({args.begin() + 1, args.end()});
      config.validate();
    } catch (const vg::config::ConfigError& e) {
      std::cerr << "vg_config: " << e.what() << "\n";
      return 2;
    }
    params.write_yaml(std::cout, "Settings from the defaults, then the given files and flags.");
    return 0;
  }
  if (command == "check" && args.size() > 1) {
    int failed = 0;
    for (std::size_t i = 1; i < args.size(); ++i) {
      vg::config::MappingConfig config;
      vg::config::ParamSet params = vg::config::bind(config);
      try {
        params.load_yaml_file(args[i]);
        config.validate();
        std::cout << args[i] << ": ok\n";
      } catch (const vg::config::ConfigError& e) {
        std::cout << e.what() << "\n";
        ++failed;
      }
    }
    return failed == 0 ? 0 : 1;
  }
  return usage();
}
