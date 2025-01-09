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

#include <filesystem>
#include <iostream>

#include "config_utilities/internal/context.h"
#include "config_utilities/internal/yaml_utils.h"

namespace config {
namespace internal {

namespace fs = std::filesystem;

struct CliParser {
  static constexpr auto FILE_OPT = "--config-utilities-file";
  static constexpr auto YAML_OPT = "--config-utilities-yaml";

  CliParser() = default;
  CliParser& parse(int& argc, char* argv[], bool remove_args);

  std::vector<std::string> files;
  std::vector<std::string> yaml_entries;
};

struct Span {
  int pos = 0;
  int num_tokens = 1;
  std::string extractTokens(int argc, char* argv[]) const;
};

std::string Span::extractTokens(int argc, char* argv[]) const {
  if (pos < 0) {
    return "";
  }

  std::stringstream ss;
  const auto total = std::min(argc, pos + num_tokens + 1);
  for (int i = pos + 1; i < total; ++i) {
    ss << argv[i];
    if (i < total - 1) {
      ss << " ";
    }
  }

  return ss.str();
}

std::ostream& operator<<(std::ostream& out, const Span& span) {
  out << "[" << span.pos << ", " << span.pos + span.num_tokens << "]";
  return out;
}

std::optional<Span> getSpan(int argc, char* argv[], int pos, bool parse_all, std::string& error) {
  int index = pos + 1;
  while (index < argc) {
    const std::string curr_opt = argv[index];
    const bool is_flag = !curr_opt.empty() && curr_opt[0] == '-';
    if (is_flag && index == pos + 1) {
      error = parse_all ? "at least one value required!" : "missing required value!";
      return std::nullopt;
    }

    if (!parse_all) {
      return Span{pos, 1};
    }

    if (is_flag) {
      break;  // stop parsing the span
    }

    ++index;
  }

  // return multi-token span
  return Span{pos, index - pos - 1};
}

void removeSpan(int& argc, char* argv[], const Span& span) {
  for (int token = span.num_tokens; token >= 0; --token) {
    for (int i = span.pos + token; i < argc - (span.num_tokens - token); ++i) {
      // bubble-sort esque shuffle to move args to end
      std::swap(argv[i], argv[i + 1]);
    }
  }

  argc -= span.num_tokens + 1;
}

void removeSpans(int& argc, char* argv[], const std::map<std::string, std::vector<Span>>& arg_spans) {
  std::vector<Span> spans;
  for (const auto& [key, key_spans] : arg_spans) {
    for (const auto& span : key_spans) {
      spans.push_back(span);
    }
  }

  std::sort(spans.begin(), spans.end(), [](const auto& lhs, const auto& rhs) { return lhs.pos > rhs.pos; });
  for (const auto& span : spans) {
    removeSpan(argc, argv, span);
  }
}

CliParser& CliParser::parse(int& argc, char* argv[], bool remove_args) {
  std::map<std::string, std::vector<Span>> spans;

  int i = 0;
  while (i < argc) {
    const std::string curr_opt(argv[i]);
    std::string error;
    std::optional<Span> curr_span;
    if (curr_opt == FILE_OPT || curr_opt == YAML_OPT) {
      curr_span = getSpan(argc, argv, i, curr_opt == YAML_OPT, error);
    }

    if (curr_span) {
      auto iter = spans.find(curr_opt);
      if (iter == spans.end()) {
        iter = spans.emplace(curr_opt, std::vector<Span>()).first;
      }

      iter->second.emplace_back(*curr_span);
      i += curr_span->num_tokens;
      continue;
    }

    if (!error.empty()) {
      // TODO(nathan) log instead of manual print
      std::cerr << "Parse issue for '" << curr_opt << "': " << error << std::endl;
    }

    ++i;
  }

  for (const auto& [key, key_spans] : spans) {
    for (const auto& span : key_spans) {
      if (key == FILE_OPT) {
        files.push_back(span.extractTokens(argc, argv));
      } else {
        yaml_entries.push_back(span.extractTokens(argc, argv));
      }
    }
  }

  if (remove_args) {
    removeSpans(argc, argv, spans);
  }

  return *this;
}

YAML::Node loadFromArguments(int& argc, char* argv[], bool remove_args) {
  YAML::Node node;

  const auto parser = CliParser().parse(argc, argv, remove_args);

  for (const auto& entry : parser.files) {
    auto filepath = entry;
    std::string ns;
    const auto index = filepath.find("@");
    if (index != std::string::npos) {
      ns = filepath.substr(index + 1);
      filepath = filepath.substr(0, index);
    }

    std::filesystem::path file(filepath);
    if (!fs::exists(file)) {
      // TODO(nathan) log instead of manual print
      std::cerr << "File " << file << " does not exist!" << std::endl;
      continue;
    }

    YAML::Node file_node;
    try {
      file_node = YAML::LoadFile(file);
    } catch (const std::exception& e) {
      // TODO(nathan) log instead of manual print
      std::cerr << "Failure for " << file << ": " << e.what() << std::endl;
      continue;
    }

    if (!ns.empty()) {
      internal::moveDownNamespace(file_node, ns);
    }

    internal::mergeYamlNodes(node, file_node, true);
  }

  for (const auto& entry : parser.yaml_entries) {
    YAML::Node cli_node;
    try {
      cli_node = YAML::Load(entry);
    } catch (const std::exception& e) {
      // TODO(nathan) log instead of manual print
      std::cerr << "Failure for '" << entry << "': " << e.what() << std::endl;
    }

    internal::mergeYamlNodes(node, cli_node, true);
  }

  return node;
}

}  // namespace internal

void initContext(int& argc, char* argv[], bool remove_arguments) {
  const auto node = internal::loadFromArguments(argc, argv, remove_arguments);
  internal::Context::update(node, "");
}

}  // namespace config
