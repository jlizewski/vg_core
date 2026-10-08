// vg_mcap_repair: rewrite an unclosed or damaged MCAP recording as a new, indexed file.
//
//   vg_mcap_repair <in.mcap> [out.mcap] [--force]
//
// A recording whose writer never closed it (app killed, crash, phone out of
// space) has no index, so tools such as Foxglove can't seek in it, and its
// last chunk may be cut off. This copies everything readable, up to the last
// complete chunk, into out.mcap (default: in.repaired.mcap next to the input)
// and writes the index. The input is never modified; an existing output is
// only replaced with --force.

#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include "vg_core/io/mcap_repair.hpp"

namespace {

int usage() {
  std::cerr << "usage: vg_mcap_repair <in.mcap> [out.mcap] [--force]\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path input;
  std::filesystem::path output;
  bool force = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--force") {
      force = true;
    } else if (!arg.empty() && arg[0] == '-') {
      return usage();
    } else if (input.empty()) {
      input = arg;
    } else if (output.empty()) {
      output = arg;
    } else {
      return usage();
    }
  }
  if (input.empty()) {
    return usage();
  }
  if (output.empty()) {
    output = input;
    output.replace_extension(".repaired.mcap");
  }
  std::error_code ec;
  if (std::filesystem::equivalent(input, output, ec)) {
    std::cerr << "error: the output would overwrite the input " << input.string() << "\n";
    return 1;
  }
  if (!force && std::filesystem::exists(output, ec)) {
    std::cerr << "error: " << output.string() << " already exists (--force to replace it)\n";
    return 1;
  }

  try {
    const vg::io::McapRepairResult result = vg::io::repair_mcap(input, output);
    if (!result.complete) {
      std::cerr << "warning: stopped at a damaged or cut-off record (" << result.problem
                << "); kept everything before it\n";
    }
    std::cout << result.messages << " messages on " << result.channels << " channels";
    if (result.messages > 0) {
      std::cout << " (" << static_cast<double>(result.end_time - result.start_time) * 1e-9 << " s)";
    }
    std::cout << ", " << result.metadata << " metadata and " << result.attachments
              << " attachment records written to " << output.string() << "\n";
    if (result.messages == 0) {
      std::cerr << "warning: no messages could be recovered\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
