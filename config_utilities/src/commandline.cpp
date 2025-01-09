/** -----------------------------------------------------------------------------
 * Copyright (c) 2023 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:     Lukas Schmid <lschmid@mit.edu>, Nathan Hughes <na26933@mit.edu>
 * AFFILIATION: MIT-SPARK Lab, Massachusetts Institute of Technology
 * YEAR:        2023
 * LICENSE:     BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

#include "config_utilities/parsing/commandline.h"

#include <iostream>

#include <boost/program_options.hpp>

#include "config_utilities/internal/context.h"
#include "config_utilities/internal/yaml_utils.h"

namespace config {
namespace internal {

namespace po = boost::program_options;

YAML::Node loadFromArguments(int argc, char* argv[], bool remove_args) {
  YAML::Node node;

  po::options_description desc("config_utilities options");
  // clang-format off
  desc.add_options()
    ("config-utilities-file", po::value<std::vector<std::string>>(), "file(s) to load")
    ("config-utilities-yaml", po::value<std::vector<std::string>>()->multitoken(), "yaml to load");
  // clang-format on

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).allow_unregistered().run(), vm);
  po::notify(vm);

  if (vm.count("config-utilities-file")) {
    const auto files = vm["config-utilities-file"].as<std::vector<std::string>>();
    for (const auto& file : files) {
      YAML::Node file_node;
      try {
        file_node = YAML::LoadFile(file);
      } catch (const std::exception& e) {
        std::cerr << "Failure for '" << file << "': " << e.what() << std::endl;
      }
      internal::mergeYamlNodes(node, file_node, true);
    }
  }

  if (vm.count("config-utilities-yaml")) {
    const auto entries = vm["config-utilities-yaml"].as<std::vector<std::string>>();
    for (const auto& entry : entries) {
      YAML::Node cli_node;
      try {
        cli_node = YAML::Load(entry);
      } catch (const std::exception& e) {
        std::cerr << "Failure for '" << entry << "': " << e.what() << std::endl;
      }

      internal::mergeYamlNodes(node, cli_node, true);
    }
  }

  return node;
}

}  // namespace internal

void initContext(int argc, char* argv[], bool remove_arguments) {
  const auto node = internal::loadFromArguments(argc, argv, remove_arguments);
  internal::Context::update(node, "");
}

}  // namespace config
