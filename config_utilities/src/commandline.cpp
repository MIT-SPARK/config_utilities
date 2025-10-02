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
#include <regex>
#include <sstream>

#include "config_utilities/internal/introspection.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/substitutions.h"
#include "config_utilities/update.h"

namespace config {
namespace internal {

namespace fs = std::filesystem;

struct Span {
  int pos = 0;
  int num_tokens = 1;
  std::string key;

  std::string extractTokens(int argc, char* argv[]) const;
};

struct CliParser {
  struct Entry {
    enum class Type { File, Yaml, Var, Flag, Arg } type;
    std::string value;
  };

  ParserInfo info;

  struct Opt {
    const std::string name;
    Entry::Type type;
    enum class NumArgs { Zero, One, ZeroOrOne, ZeroOrMore, OneOrMore } nargs = NumArgs::One;
    const std::string short_name;

    bool matches(const std::string& option) const;
    std::string getFlagToken(const std::string& option) const;
  };

  const std::vector<Opt> opts{{"config-utilities-file", Entry::Type::File, Opt::NumArgs::One, "f"},
                              {"config-utilities-yaml", Entry::Type::Yaml, Opt::NumArgs::OneOrMore, "c"},
                              {"config-utilities-var", Entry::Type::Var, Opt::NumArgs::One, "v"},
                              {"disable-substitutions", Entry::Type::Flag, Opt::NumArgs::Zero, "d"},
                              {"config-utilities-introspect", Entry::Type::Arg, Opt::NumArgs::ZeroOrOne, "i"}};

  CliParser() = default;
  CliParser& parse(int& argc, char* argv[], bool remove_args);
  std::optional<Opt> matches(const std::string& option);

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

bool checkIfFlag(const std::string& opt) {
  std::regex flag_regex(R"regex(^-{1,2}[\w-]+(=.+)?$)regex");
  std::smatch m;
  return std::regex_match(opt, m, flag_regex);
}

std::optional<Span> getSpan(int argc, char* argv[], int pos, CliParser::Opt::NumArgs nargs, std::string& error) {
  // a flag is one of:
  //   -some-option_name
  //   --some_0ption-name
  //   --some_option_name=random_characters{}
  //
  // This presents some ambiguous situations where a yaml value
  // can be misinterpreted as a new command line flag, such as
  // ["--config-utilities-yaml", "{a:", "--some_value=value}"]
  // which gets parsed as the (invalid) yaml input string
  // "{a:" and the argument "--some_value=value}"
  // If you want to specify the yaml input string
  // "{a: --some_value=value}"
  // you should escape it when passing the argument, i.e.,
  // --config-utilities-yaml '{a: --some_value=value}'
  if (nargs == CliParser::Opt::NumArgs::Zero) {
    return Span{pos, 0, argv[pos]};
  }

  int index = pos + 1;
  while (index < argc) {
    const std::string curr_opt = argv[index];
    if (checkIfFlag(curr_opt)) {
      break;  // stop parsing the span
    }

    if (nargs == CliParser::Opt::NumArgs::One) {
      return Span{pos, 1, argv[pos]};
    }

    ++index;
  }

  if (index == pos + 1 && (nargs == CliParser::Opt::NumArgs::One || nargs == CliParser::Opt::NumArgs::OneOrMore)) {
    error = nargs == CliParser::Opt::NumArgs::OneOrMore ? "at least one value required!" : "missing required value!";
    return std::nullopt;
  }
  if (nargs == CliParser::Opt::NumArgs::ZeroOrOne && index > pos + 2) {
    error = "At most one value allowed!";
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

bool CliParser::Opt::matches(const std::string& option) const {
  std::string flags = "^--" + name + "$";
  if (!short_name.empty()) {
    flags += "|^-" + short_name + "$";
  }

  if (type == Entry::Type::Flag) {
    flags += "|^--no-" + name + "$";
  }

  std::smatch match;
  const std::regex matcher(flags);
  return std::regex_match(option, match, matcher);
}

std::string CliParser::Opt::getFlagToken(const std::string& option) const {
  const auto is_true = option == "--" + name || (!short_name.empty() && option == "-" + short_name);
  return name + "=" + (is_true ? "true" : "false");
}

std::optional<CliParser::Opt> CliParser::matches(const std::string& option) {
  for (const auto& opt : opts) {
    if (opt.matches(option)) {
      return opt;
    }
  }

  return std::nullopt;
}

CliParser& CliParser::parse(int& argc, char* argv[], bool remove_args) {
  int i = 0;
  bool found_separator = false;
  std::vector<Span> spans;
  while (i < argc) {
    const std::string curr_opt(argv[i]);
    if (!checkIfFlag(curr_opt)) {
      ++i;  // skip non-flags from parsing
      continue;
    }

    std::string error;
    std::optional<Span> curr_span;
    if ((curr_opt == "-h" || curr_opt == "--help") && !found_separator) {
      info.help_present = true;
      spans.emplace_back(Span{i, 0, curr_opt});
      ++i;
      continue;
    }

    if ((curr_opt == "--force-block-style") && !found_separator) {
      info.force_block_style = true;
      spans.emplace_back(Span{i, 0, curr_opt});
      ++i;
      continue;
    }

    if (curr_opt == "--" && !found_separator) {
      found_separator = true;
      spans.emplace_back(Span{i, 0, curr_opt});
      break;
    }

    const auto opt_match = matches(curr_opt);
    if (opt_match) {
      curr_span = getSpan(argc, argv, i, opt_match->nargs, error);
    }

    if (curr_span) {
      spans.emplace_back(*curr_span);
      i += curr_span->num_tokens + 1;
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
    const auto opt = matches(span.key);
    if (!opt) {
      continue;  // skip any spans for single options
    }

    switch (opt->type) {
      case Entry::Type::File:
      case Entry::Type::Yaml:
      case Entry::Type::Var:
        entries.push_back(Entry{opt->type, span.extractTokens(argc, argv)});
        break;
      case Entry::Type::Flag:
        entries.push_back(Entry{opt->type, opt->getFlagToken(span.key)});
        break;
      case Entry::Type::Arg:
        const auto tokens = span.extractTokens(argc, argv);
        entries.push_back(Entry{opt->type, opt->name + (tokens.empty() ? "" : " " + tokens)});
        break;
    }
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
    ss << "File '" << file.string() << "' does not exist!";
    Logger::logError(ss.str());
    return node;
  }

  try {
    node = YAML::LoadFile(file);
  } catch (const std::exception& e) {
    std::stringstream ss;
    ss << "Failure for '" << file.string() << "': " << e.what();
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

void parseEntryVar(const CliParser::Entry& entry, std::map<std::string, std::string>& vars) {
  auto pos = entry.value.find("=");
  if (pos == std::string::npos) {
    Logger::logError("Invalid variable specification '" + entry.value + "'. Must be 'name=value'");
    return;
  }

  vars[entry.value.substr(0, pos)] = entry.value.substr(pos + 1);
}

void updateContextFromFlag(const CliParser::Entry& entry, ParserContext& context) {
  auto pos = entry.value.find("=");
  if (pos == std::string::npos) {
    Logger::logError("Invalid flag '" + entry.value + "'");
    return;
  }

  const auto name = entry.value.substr(0, pos);
  const bool value = entry.value.substr(pos + 1) == "true";
  if (name == "disable-substitutions") {
    context.allow_substitutions = !value;
  }
}

void setupIntrospectionFromParser(const CliParser& parser) {
  for (const auto& entry : parser.entries) {
    if (entry.type != CliParser::Entry::Type::Arg) {
      continue;
    }
    if (entry.value.rfind("config-utilities-introspect", 0) == 0) {
      const auto pos = entry.value.find(" ");
      internal::Settings::instance().introspection.output =
          pos != std::string::npos ? entry.value.substr(pos + 1) : "config_introspection_output";
      return;
    }
  }
}

YAML::Node loadFromArguments(int& argc, char* argv[], bool remove_args, ParserInfo* info) {
  const auto parser = CliParser().parse(argc, argv, remove_args);
  if (info) {
    *info = parser.info;
    if (info->help_present) {
      return YAML::Node();
    }
  }

  // Check for introspection first.
  setupIntrospectionFromParser(parser);
  const bool introspection_enabled = Settings::instance().introspection.enabled();

  // Populate the context.
  ParserContext context;
  YAML::Node node = YAML::Node(YAML::NodeType::Null);
  for (const auto& entry : parser.entries) {
    YAML::Node parsed_node;
    switch (entry.type) {
      case CliParser::Entry::Type::File:
        parsed_node = nodeFromFileEntry(entry);
        break;
      case CliParser::Entry::Type::Yaml:
        parsed_node = nodeFromLiteralEntry(entry);
        break;
      case CliParser::Entry::Type::Var:
        parseEntryVar(entry, context.vars);
        continue;
      case CliParser::Entry::Type::Flag:
        updateContextFromFlag(entry, context);
        continue;
      case CliParser::Entry::Type::Arg:
        continue;
    }

    internal::mergeYamlNodes(node, parsed_node, MergeMode::APPEND);
    if (introspection_enabled) {
      const auto by = entry.type == CliParser::Entry::Type::File
                          ? Introspection::By::file(fs::absolute(entry.value).string())
                          : Introspection::By::arg(entry.value);
      Introspection::logMerge(node, parsed_node, by);
    }
  }

  resolveSubstitutions(node, context);
  if (introspection_enabled) {
    Introspection::logDiff(node, Introspection::By::substitution());
  }
  return node;
}

YAML::Node loadFromArguments(const std::vector<std::string>& _args) {
  std::vector<std::string> args(_args.begin(), _args.end());

  int argc = args.size();
  std::vector<char*> argv;
  std::transform(args.begin(), args.end(), std::back_inserter(argv), [](auto& m) { return m.data(); });
  // no need to remove arguments given that the original vector won't be modified
  return loadFromArguments(argc, argv.data(), false);
}

}  // namespace internal

void setConfigSettingsFromCLI(int argc, char* argv[], const std::string& name_space) {
  auto node = internal::loadFromArguments(argc, argv, false);
  internal::Visitor::setValues(Settings(), internal::lookupNamespace(node, name_space), true);
}

void setConfigSettingsFromCLI(const std::vector<std::string>& argv, const std::string& name_space) {
  auto node = internal::loadFromArguments(argv);
  internal::Visitor::setValues(Settings(), internal::lookupNamespace(node, name_space), true);
}

}  // namespace config
