#include "config_utilities/internal/introspection.h"

#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>

#ifdef CONFIG_UTILS_ENABLE_JSON
#include <nlohmann/json.hpp>
#endif  // CONFIG_UTILS_ENABLE_JSON

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/yaml_utils.h"

// TMP
#include <iostream>

namespace config::internal {

using Event = Introspection::Event;
using By = Introspection::By;
using Node = Introspection::Node;

By::By(Type type, const std::string& value) : type(type), index(registerSource(type, value)) {}

size_t By::registerSource(Type type, const std::string& value) {
  auto& sources = Introspection::instance().sources_[type];
  const auto it = sources.find(value);
  if (it != sources.end()) {
    return it->second;
  }
  const size_t new_index = sources.size();
  sources[value] = new_index;
  return new_index;
}

By By::file(const std::string& filename) { return By(Type::File, filename); }

By By::arg(const std::string& args) { return By(Type::Arg, args); }

By By::substitution(const std::string& substitution_details) { return By(Type::Substitution, substitution_details); }

By By::programmatic(const std::string& call) { return By(Type::Programmatic, call); }

By By::config(const std::string& config_name) { return By(Type::Config, config_name); }

Event::Event(Type type, const By& by, const std::string& value)
    : type(type), by(by), value(value), sequence_id(instance().sequence_id_) {}

bool Event::isDeleteEvent() const { return type == Type::Remove; }

bool Event::isSetEvent() const {
  return type == Type::Set || type == Type::Update || type == Type::SetNonModified || type == Type::SetFailed;
}

bool Event::isGetEvent() const { return type == Type::Get || type == Type::GetDefault || type == Type::GetError; }

bool Event::valueModified() const {
  if (type != Type::Set && type == Type::Update) {
    return false;
  }
  return !value.empty();
}

const std::string& Node::lastValue() const { return last_value_; }

void Node::addEvent(const Event& event) {
  history.push_back(event);
  if (event.type == Event::Type::Remove) {
    last_value_.clear();
    clearDownstream();
    return;
  }
  if (event.valueModified()) {
    last_value_ = event.value;
    clearDownstream();
  }
}

void Node::clearDownstream() {
  for (auto& child : list) {
    child.last_value_.clear();
    child.clearDownstream();
  }
  for (auto& [key, child] : map) {
    child.last_value_.clear();
    child.clearDownstream();
  }
}

Node& Node::at(const std::string& key) { return map[key]; }

Node& Node::at(size_t index) {
  if (index >= list.size()) {
    list.resize(index + 1);
  }
  return list[index];
}

void Node::clear() {
  history.clear();
  last_value_.clear();
  list.clear();
  map.clear();
}

bool Node::empty() const { return history.empty() && list.empty() && map.empty(); }

YAML::Node Node::toYaml(size_t at_sequence_id) const { return toYamlRec(at_sequence_id).first; }

std::pair<YAML::Node, size_t> Node::toYamlRec(size_t at_sequence_id) const {
  // 1. Compute the last set or removed value and the corresponding sequence id.
  std::string value;
  size_t value_last_set = 0;
  for (auto& event : history) {
    if (event.sequence_id > at_sequence_id) {
      break;
    }
    value_last_set = event.sequence_id;
    if (event.valueModified()) {
      value = event.value;
      continue;
    }
    if (event.isDeleteEvent()) {
      value.clear();
    }
  }

  // 2. Get the list values if present.
  auto yaml_list = YAML::Node(YAML::NodeType::Sequence);
  size_t list_last_set = 0;
  for (const auto& child : list) {
    const auto [child_node, child_last_set] = child.toYamlRec(at_sequence_id);
    if (child_node.IsNull()) {
      continue;
    }
    yaml_list.push_back(child_node);
    list_last_set = std::max(list_last_set, child_last_set);
  }

  // 3. Get the map values if present.
  auto yaml_map = YAML::Node(YAML::NodeType::Map);
  size_t map_last_set = 0;
  for (const auto& [key, child] : map) {
    const auto [child_node, child_last_set] = child.toYamlRec(at_sequence_id);
    if (child_node.IsNull()) {
      continue;
    }
    yaml_map[key] = child_node;
    map_last_set = std::max(map_last_set, child_last_set);
  }

  // 4. Compute the state of the node.
  if (value_last_set >= list_last_set && value_last_set >= map_last_set) {
    // Scalar was set or the node was deleted, no downstream members. This also covers the case where everything is
    // empty.
    if (value.empty()) {
      return {YAML::Node(YAML::NodeType::Null), value_last_set};
    }
    return {YamlParser::toYaml(value), value_last_set};
  }
  if (map_last_set >= list_last_set) {
    // Map was set or has the most recent change.
    return {yaml_map, map_last_set};
  }
  return {yaml_list, list_last_set};
}

void Introspection::logMerge(const YAML::Node& merged, const YAML::Node& input, const By& by) {
  auto& instance = Introspection::instance();
  instance.initLog();
  instance.logMergeRec(merged, input, by, instance.data_);
}

void Introspection::logMergeRec(const YAML::Node& merged, const YAML::Node& input, const By& by, Node& node) {
  if (input.IsScalar()) {
    // Compare actual values in the leaf nodes.
    const std::string input_value = yamlToString(input, false);
    if (!merged.IsScalar()) {
      // If the resulting node is not a scalar, the set failed.
      node.addEvent(Event(Event::Type::SetFailed, by, input_value));
      return;
    }
    const std::string merged_value = yamlToString(merged, false);
    const std::string& previous_value = node.lastValue();
    if (merged_value == previous_value) {
      // Value is present in set but not modified.
      node.addEvent(Event(Event::Type::SetNonModified, by));
      return;
    }
    if (merged_value == input_value) {
      // Overwritten or new.
      node.addEvent(Event(Event::Type::Set, by, merged_value));
      return;
    }
    // Modified value in other ways.
    node.addEvent(Event(Event::Type::Update, by, merged_value));
    return;
  }
  if (input.IsSequence()) {
    for (size_t i = 0; i < input.size(); ++i) {
      logMergeRec(merged[i], input[i], by, node[i]);
    }
  } else if (input.IsMap()) {
    for (const auto& kv : input) {
      const std::string key = kv.first.Scalar();
      logMergeRec(merged[key], kv.second, by, node[key]);
    }
  }
}

void Introspection::logDiff(const YAML::Node& before,
                            const YAML::Node& after,
                            const By& by,
                            const Event::Type log_diff_as) {
  auto& instance = Introspection::instance();
  instance.initLog();
  instance.logDiffRec(before, after, by, instance.data_, log_diff_as);
}

void Introspection::logDiffRec(const YAML::Node& before,
                               const YAML::Node& after,
                               const By& by,
                               Node& node,
                               const Event::Type log_diff_as) {
  if (after.IsNull()) {
    if (!before.IsNull()) {
      node.addEvent(Event(Event::Type::Remove, by));
    }
    return;
  }
  if (after.IsScalar()) {
    // Compare actual values in the leaf nodes.
    const std::string after_value = yamlToString(after, false);
    if (!before.IsScalar()) {
      // Now a scalar: was newly set / overridden.
      node.addEvent(Event(Event::Type::Set, by, after_value));
      return;
    }
    const std::string before_value = yamlToString(before, false);
    if (after_value != before_value) {
      node.addEvent(Event(log_diff_as, by, after_value));
      return;
    }
    // Value is not modified, no need to log diff.
    return;
  }
  if (after.IsSequence()) {
    for (size_t i = 0; i < after.size(); ++i) {
      logDiffRec(before[i], after[i], by, node[i], log_diff_as);
    }
  } else if (after.IsMap()) {
    for (const auto& kv : after) {
      const std::string key = kv.first.Scalar();
      logDiffRec(before[key], kv.second, by, node[key], log_diff_as);
    }
  }
}

void Introspection::logSetValue(const MetaData& meta_data, const std::string& ns) {}

void Introspection::logClear(const By& by) {
  auto& instance = Introspection::instance();
  instance.initLog();
  instance.data_.addEvent(Event(Event::Type::Remove, by));
}

void Introspection::clear() {
  data_.clear();
  sources_.clear();
  sequence_id_ = 0;
}

void Introspection::initLog() {
  // Increment the sequence id for the next events.
  ++sequence_id_;
}

// Serialization: conditional compilation.
#ifdef CONFIG_UTILS_ENABLE_JSON

nlohmann::json toJson(const Event& event) {
  nlohmann::json j;
  j["type"] = std::string(1, static_cast<char>(event.type));
  j["by"] = std::string(1, static_cast<char>(event.by.type)) + std::to_string(event.by.index);
  j["seq"] = event.sequence_id;
  if (!event.value.empty()) {
    j["val"] = event.value;
  }
  return j;
}

nlohmann::json toJson(const Node& node) {
  nlohmann::json j;
  auto hist = nlohmann::json::array();
  for (const auto& event : node.history) {
    hist.push_back(toJson(event));
  }
  j["history"] = hist;
  if (!node.list.empty()) {
    auto list = nlohmann::json::array();
    for (const auto& child : node.list) {
      list.push_back(toJson(child));
    }
    j["list"] = list;
  }
  if (!node.map.empty()) {
    nlohmann::json map;
    for (const auto& [key, child] : node.map) {
      map[key] = toJson(child);
    }
    j["map"] = map;
  }
  return j;
}

nlohmann::json toJson(const Introspection::Sources& sources) {
  nlohmann::json j;
  for (const auto& [type, entries] : sources) {
    nlohmann::json entry;
    for (const auto& [value, index] : entries) {
      entry[index] = value;
    }
    j[std::string(1, static_cast<char>(type))] = entry;
  }
  return j;
}

void writeOutputDataImpl(const Node& data, const Introspection::Sources& sources, const std::string& output_directory) {
  const auto output_dir = std::filesystem::path(output_directory);
  // Setup and clear the output directory if it exists.
  if (std::filesystem::exists(output_dir)) {
    std::filesystem::remove_all(output_dir);
  }
  std::filesystem::create_directories(output_dir);

  const auto json_path = output_dir / "data.json";
  std::ofstream json_file(json_path);
  if (!json_file.is_open()) {
    Logger::logError("Failed to open introspection output file '" + json_path.string() + "'.");
    return;
  }
  nlohmann::json output_data;
  output_data["data"] = toJson(data);
  output_data["sources"] = toJson(sources);
  json_file << output_data.dump(2);
  json_file.close();
}

#endif  // CONFIG_UTILS_ENABLE_JSON

void Introspection::writeOutputData(const std::string& output_dir) {
#ifdef CONFIG_UTILS_ENABLE_JSON
  writeOutputDataImpl(data_, sources_, output_dir);
#else
  Logger::logWarning(
      "Introspection requires 'nlohmann_json' to be available at compile time. No output written. Try installing it "
      "via 'sudo apt install nlohmann-json3-dev' and recompile config utilities.");
#endif  // CONFIG_UTILS_ENABLE_JSON
}

Introspection::~Introspection() {
  // On teardown write all the output.
  if (data_.empty()) {
    return;
  }

  if (!Settings::instance().introspection.enabled()) {
    Logger::logWarning(
        "Introspection data is present but introspection has been disabled, not writing any introspection output.");
    return;
  }

  writeOutputData(Settings::instance().introspection.output);
}

Introspection& Introspection::instance() {
  static Introspection instance;
  return instance;
}

std::map<std::string, std::string> flatten(const YAML::Node& node, const std::string& ns_separator) {
  std::map<std::string, std::string> result;

  // Helper function to recursively flatten the node.
  std::function<void(const YAML::Node&, const std::string&)> flatten = [&](const YAML::Node& n,
                                                                           const std::string& path) {
    if (n.IsMap()) {
      for (const auto& kv : n) {
        flatten(kv.second, path.empty() ? kv.first.Scalar() : path + ns_separator + kv.first.Scalar());
      }
    } else {
      result[path] = yamlToString(n, false);
    }
  };

  flatten(node, "");
  return result;
}

}  // namespace config::internal