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
#include <sstream>

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/yaml_utils.h"

namespace config::internal {

namespace fs = std::filesystem;

struct Span {
  int pos = 0;
  int num_tokens = 1;
  std::string key;

  std::string extractTokens(int argc, char* argv[]) const;
};

struct CliParser {
  struct Entry {
    std::string value;
    bool is_file;
  };

  static constexpr auto FILE_OPT = "--config-utilities-file";
  static constexpr auto YAML_OPT = "--config-utilities-yaml";

  CliParser() = default;
  CliParser& parse(int& argc, char* argv[], bool remove_args);

  std::vector<Entry> entries;
};

std::string Span::extractTokens(int argc, char* argv[]) const {
  if (pos < 0) {
    return "";  // NOTE(nathan) unreachable
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

std::optional<Span> getSpan(int argc, char* argv[], int pos, bool parse_all, std::string& error) {
  int index = pos + 1;
  while (index < argc) {
    const std::string curr_opt = argv[index];
    const bool is_flag = !curr_opt.empty() && curr_opt[0] == '-';
    if (is_flag) {
      break;  // stop parsing the span
    }

    if (!parse_all) {
      return Span{pos, 1, argv[pos]};
    }

    ++index;
  }

  if (index == pos + 1) {
    error = parse_all ? "at least one value required!" : "missing required value!";
    return std::nullopt;
  }

  // return multi-token span
  return Span{pos, index - pos - 1, argv[pos]};
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

CliParser& CliParser::parse(int& argc, char* argv[], bool remove_args) {
  std::vector<Span> spans;

  int i = 0;
  while (i < argc) {
    const std::string curr_opt(argv[i]);
    std::string error;
    std::optional<Span> curr_span;
    if (curr_opt == FILE_OPT || curr_opt == YAML_OPT) {
      curr_span = getSpan(argc, argv, i, curr_opt == YAML_OPT, error);
    }

    if (curr_span) {
      spans.emplace_back(*curr_span);
      i += curr_span->num_tokens;
      continue;
    }

    if (!error.empty()) {
      std::stringstream ss;
      ss << "Parse issue for '" << curr_opt << "': " << error;
      Logger::logError(ss.str());
    }

    ++i;
  }

  for (const auto& span : spans) {
    entries.push_back(Entry{span.extractTokens(argc, argv), span.key == FILE_OPT});
  }

  if (remove_args) {
    std::sort(spans.begin(), spans.end(), [](const auto& lhs, const auto& rhs) { return lhs.pos > rhs.pos; });
    for (const auto& span : spans) {
      removeSpan(argc, argv, span);
    }
  }

  return *this;
}

YAML::Node nodeFromFileEntry(const CliParser::Entry& entry) {
  auto filepath = entry.value;

  std::string ns;
  const auto index = filepath.find("@");
  if (index != std::string::npos) {
    ns = filepath.substr(index + 1);
    filepath = filepath.substr(0, index);
  }

  YAML::Node node;
  std::filesystem::path file(filepath);
  if (!fs::exists(file)) {
    std::stringstream ss;
    ss << "File " << file << " does not exist!";
    Logger::logError(ss.str());
    return node;
  }

  try {
    node = YAML::LoadFile(file);
  } catch (const std::exception& e) {
    std::stringstream ss;
    ss << "Failure for " << file << ": " << e.what();
    Logger::logError(ss.str());
    return node;
  }

  if (!ns.empty()) {
    internal::moveDownNamespace(node, ns);
  }

  return node;
}

YAML::Node nodeFromLiteralEntry(const CliParser::Entry& entry) {
  YAML::Node node;
  try {
    node = YAML::Load(entry.value);
  } catch (const std::exception& e) {
    std::stringstream ss;
    ss << "Failure for '" << entry.value << "': " << e.what();
    Logger::logError(ss.str());
  }

  return node;
}

YAML::Node loadFromArguments(int& argc, char* argv[], bool remove_args) {
  const auto parser = CliParser().parse(argc, argv, remove_args);

  YAML::Node node;
  for (const auto& entry : parser.entries) {
    YAML::Node parsed_node;
    if (entry.is_file) {
      parsed_node = nodeFromFileEntry(entry);
    } else {
      parsed_node = nodeFromLiteralEntry(entry);
    }

    // no-op for invalid parsed node
    internal::mergeYamlNodes(node, parsed_node, true);
  }

  return node;
}

}  // namespace config::internal
