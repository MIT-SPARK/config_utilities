#include "config_utilities/test/cli_args.h"

#include <sstream>

namespace config::test {

std::string CliArgs::Args::get_cmd() const {
  std::stringstream ss;
  for (int i = 0; i < argc; ++i) {
    ss << argv[i];
    if (i < argc - 1) {
      ss << " ";
    }
  }

  return ss.str();
}

CliArgs::CliArgs(const std::vector<std::string>& args) : original_args(args) {
  for (auto& str : original_args) {
    arg_pointers.push_back(str.data());
  }
}

CliArgs::Args CliArgs::get() { return {static_cast<int>(arg_pointers.size()), arg_pointers.data()}; }

}  // namespace config::test