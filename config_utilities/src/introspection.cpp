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
  if (type == Type::Remove) {
    return true;
  }
  if (type != Type::Set && type != Type::Update) {
    return false;
  }
  return !value.empty();
}

std::string Event::display() const {
  std::stringstream ss;
  // <type><index>@<by>[:<value>]
  ss << "'" << type << sequence_id << "@" << by.type << by.index;
  if (!value.empty()) {
    ss << ":" << value;
  }
  ss << "'";
  return ss.str();
}

const std::string& Node::lastValue() const { return last_value_; }

void Node::addEvent(const Event& event) {
  history.push_back(event);
  // If this event unset downstream values, log this as separate delete events for easier in-leave tracking.
  if (event.valueModified()) {
    last_value_ = event.value;
  }
}

Node& Node::at(const std::string& key) {
  auto it = std::find_if(map.begin(), map.end(), [&](const auto& pair) { return pair.first == key; });
  if (it == map.end()) {
    map.emplace_back(key, Node());
    return map.back().second;
  }
  return it->second;
}

Node& Node::at(size_t index) {
  if (index >= list.size()) {
    list.resize(index + 1);
  }
  return list[index];
}

Node& Node::atNamespace(const std::string& name_space) {
  const auto parts = splitNamespace(name_space);
  Node* current = this;
  for (const auto& part : parts) {
    current = &current->at(part);
  }
  return *current;
}

void Node::clear() {
  history.clear();
  last_value_.clear();
  list.clear();
  map.clear();
}

bool Node::empty() const { return history.empty() && list.empty() && map.empty(); }

YAML::Node Node::toYaml(size_t at_sequence_id) const { return toYamlRec(at_sequence_id).value_or(YAML::Node()); }

std::optional<YAML::Node> Node::toYamlRec(size_t at_sequence_id) const {
  // All nodes should log correct delete events, so it is fine to just go through the history.
  std::string value;
  for (auto& event : history) {
    if (event.sequence_id > at_sequence_id) {
      break;
    }
    if (event.valueModified()) {
      value = event.value;
    }
  }
  if (!value.empty()) {
    return YAML::Node(value);
  }

  // Check list.
  YAML::Node node(YAML::NodeType::Sequence);
  for (const auto& child : list) {
    const auto child_node = child.toYamlRec(at_sequence_id);
    if (child_node) {
      node.push_back(*child_node);
    }
  }
  if (node.size() != 0) {
    return node;
  }

  // Check map.
  node = YAML::Node(YAML::NodeType::Map);
  for (const auto& [key, child] : map) {
    const auto child_node = child.toYamlRec(at_sequence_id);
    if (child_node) {
      node[key] = *child_node;
    }
  }
  if (node.size() != 0) {
    return node;
  }
  return std::nullopt;
}

std::string Node::display(size_t indent) const {
  if (list.empty() && map.empty() && history.empty()) {
    return "";
  }
  std::stringstream ss;
  const std::string indent_str(indent, ' ');
  if (!history.empty()) {
    ss << " [";
    for (size_t i = 0; i < history.size(); ++i) {
      ss << history[i].display();
      if (i + 1 < history.size()) {
        ss << ", ";
      }
    }
    ss << "]";
  }
  for (size_t i = 0; i < list.size(); ++i) {
    ss << "\n" << indent_str << "[" << i << "]:" << list[i].display(indent + 2);
  }
  for (const auto& [key, child] : map) {
    ss << "\n" << indent_str << key << ":" << child.display(indent + 2);
  }
  return ss.str();
}

void Introspection::logMerge(const YAML::Node& merged, const YAML::Node& input, const By& by) {
  auto& instance = Introspection::instance();
  instance.initLog();
  auto& node = instance.currentNode();
  instance.logMergeRec(merged, input, by, node);
  instance.logRemovesRec(merged, node, by);
  instance.finishLog();
}

void Introspection::logMergeRec(const YAML::Node& merged, const YAML::Node& input, const By& by, Node& node) {
  // Only log 'positive' changes here, i.e. sets, modified values, and failed sets. Removed values will be handled in a
  // separate pass.
  if (input.IsScalar()) {
    // Compare actual values in the leaf nodes.
    const std::string input_value = yamlToString(input, true);
    if (!merged.IsScalar()) {
      // If the resulting node is not a scalar, the set failed.
      node.addEvent(Event(Event::Type::SetFailed, by, input_value));
      return;
    }
    const std::string merged_value = yamlToString(merged, true);
    if (merged_value != input_value) {
      // Overwritten or new.
      node.addEvent(Event(Event::Type::SetFailed, by, input_value));
      return;
    }
    const std::string& previous_value = node.lastValue();
    if (merged_value == previous_value) {
      // Value is present in set but not modified.
      node.addEvent(Event(Event::Type::SetNonModified, by, merged_value));
      return;
    }
    if (merged_value == input_value) {
      // Overwritten or new.
      node.addEvent(Event(Event::Type::Set, by, merged_value));
      return;
    }
  } else if (input.IsSequence()) {
    // NOTE(lschmid): Lists are tricky as indexing happens by position. This means the lists have been merged (in
    // potentially arbitrary ways). Currently cover the tail of the merged sequence, as this will log appends and
    // likely the full sequence. This will miss introspection for changes in the middle of lists though.
    const size_t offset = merged.IsSequence() && merged.size() > input.size() ? merged.size() - input.size() : 0;
    for (size_t i = 0; i < input.size(); ++i) {
      logMergeRec(at(merged, i + offset), input[i], by, node[i + offset]);
    }
  } else if (input.IsMap()) {
    for (const auto& kv : input) {
      const std::string key = kv.first.Scalar();
      logMergeRec(at(merged, key), kv.second, by, node[key]);
    }
  }
}

void Introspection::logDiff(const YAML::Node& after, const By& by, const Event::Type log_diff_as) {
  auto& instance = Introspection::instance();
  instance.initLog();
  auto& node = instance.currentNode();
  instance.logDiffRec(after, by, node, log_diff_as);
  instance.logRemovesRec(after, node, by);
  instance.finishLog();
}

void Introspection::logDiffRec(const YAML::Node& after, const By& by, Node& node, const Event::Type log_diff_as) {
  // Only check for 'positive' changes here, i.e. set or modified values. Removed values will be handled in a separate
  // pass.
  if (after.IsScalar()) {
    // Compare actual values in the leaf nodes.
    const std::string after_value = yamlToString(after, true);
    const std::string& before_value = node.lastValue();
    if (after_value != before_value) {
      node.addEvent(Event(log_diff_as, by, after_value));
    }
    return;
  } else if (after.IsSequence()) {
    for (size_t i = 0; i < after.size(); ++i) {
      logDiffRec(after[i], by, node[i], log_diff_as);
    }
  } else if (after.IsMap()) {
    for (const auto& kv : after) {
      const std::string key = kv.first.Scalar();
      logDiffRec(kv.second, by, node[key], log_diff_as);
    }
  }
}

void Introspection::logSetValue(const MetaData& set, const MetaData& get_info) {
  auto& instance = Introspection::instance();
  instance.initLog();
  auto& node = instance.currentNode().atNamespace(get_info.ns);
  instance.logSetValueRec(set, get_info, node);
  instance.finishLog();
}

Node& subMetaDataNode(Node& node, const MetaData& meta, const std::string& parent_ns) {
  const std::string new_ns = meta.ns.length() > parent_ns.length()
                                 ? (parent_ns.empty() ? meta.ns : meta.ns.substr(parent_ns.length() + 1))
                                 : "";
  Node* current = &node.atNamespace(new_ns);
  if (meta.isArrayConfig()) {
    current = &current->at(meta.array_config_index);
  } else if (meta.isMapConfig()) {
    current = &current->at(*meta.map_config_key);
  }
  return *current;
}

void Introspection::logSetValueRec(const MetaData& set, const MetaData& get_info, Node& node) {
  // Register the config as a source.
  // TODO(lschmid): Consider tracking the invoking/parent configs for subconfigs as extra info?
  const std::string config_name =
      get_info.name.empty() ? (get_info.isVirtualConfig() ? kUninitializedVirtualConfigType : "Unnamed Config")
                            : get_info.name;
  const By by(By::config(config_name));

  // Parse all fields in the meta data.
  // NOTE(lschmid): Just associating fields by order should cover most cases, but technically funky things could happen
  // with conditional field reading etc. Same for config-vectors, these can technically mess up if get/set is different.
  size_t num_fields = set.field_infos.size();
  if (set.field_infos.size() != get_info.field_infos.size()) {
    Logger::logWarning("Different number of fields in config '" + config_name + "' for Set (" +
                       std::to_string(set.field_infos.size()) + ") and Get (" +
                       std::to_string(get_info.field_infos.size()) +
                       "). This might lead to incorrect introspection data.");
    num_fields = std::min(num_fields, get_info.field_infos.size());
  }
  for (size_t i = 0; i < num_fields; ++i) {
    const auto& set_field = set.field_infos[i];
    const auto& get_field = get_info.field_infos[i];
    if (get_field.ns != set_field.ns) {
      Logger::logWarning("Different namespaces for field '" + set_field.name + "' in config '" + config_name +
                         "' for Set ('" + set_field.ns + "') and Get ('" + get_field.ns +
                         "'). This might lead to incorrect introspection data.");
    }
    if (get_field.name != set_field.name) {
      Logger::logWarning("Different field names for index " + std::to_string(i) + " in config '" + config_name +
                         "' for Set ('" + set_field.name + "') and Get ('" + get_field.name +
                         "'). This might lead to incorrect introspection data.");
    }
    Node& field_node = node.atNamespace(get_field.ns).at(get_field.name);
    logSetRecurseLeaves(set_field.value, get_field.value, set_field.was_parsed, get_field.isDefault(), field_node, by);
  }

  // Handle sub-configs
  for (const auto& sub_get_info : get_info.sub_configs) {
    const auto match = set.findMatchingSubConfig(sub_get_info);
    Node& sub_node = subMetaDataNode(node, sub_get_info, get_info.ns);
    if (match) {
      logSetValueRec(*match, sub_get_info, sub_node);
    } else {
      // If the set meta-data is absent, a map or vector config was not present. Log their read (default) values.
      logSetValueRecAbsent(sub_get_info, sub_node);
    }
  }

  // TODO(lschmid): Can set.sub_configs be larger/different than get.sub_configs? Just warn for now.
  for (const auto& sub_set_info : set.sub_configs) {
    if (!get_info.findMatchingSubConfig(sub_set_info)) {
      Logger::logWarning("Could not find matching sub-config for introspection in config '" + config_name +
                         "' for subconfig: '" + sub_set_info.field_name + "', key: '" + sub_set_info.displayIndex() +
                         "', type: '" + sub_set_info.name + "'.");
    }
  }
}

void Introspection::logSetValueRecAbsent(const MetaData& get_info, Node& node) {
  // Register the config as a source.
  const std::string config_name = get_info.name.empty() ? "Unnamed Config" : get_info.name;
  const By by(By::config(config_name));

  // Parse all fields in the meta data.
  for (size_t i = 0; i < get_info.field_infos.size(); ++i) {
    const auto& get_field = get_info.field_infos[i];
    Node& field_node = node.atNamespace(get_field.ns).at(get_field.name);
    logSetRecurseLeaves(
        YAML::Node(YAML::NodeType::Undefined), get_field.value, false, get_field.isDefault(), field_node, by);
  }

  // Handle sub-configs
  for (const auto& sub_get_info : get_info.sub_configs) {
    logSetValueRecAbsent(sub_get_info, subMetaDataNode(node, sub_get_info, get_info.ns));
  }
}

void Introspection::logSetRecurseLeaves(const YAML::Node& set,
                                        const YAML::Node& get,
                                        bool was_parsed,
                                        bool is_default,
                                        Node& node,
                                        const By& by) {
  if (get.IsScalar()) {
    const std::string value = yamlToString(get, true);
    if (!set || !was_parsed) {
      node.addEvent(Event(Event::Type::GetAbsent, by, value));
      return;
    }
    if (is_default) {
      if (value == yamlToString(set, true)) {
        node.addEvent(Event(Event::Type::GetDefault, by, value));
      } else {
        node.addEvent(Event(Event::Type::GetError, by, value));
      }
      return;
    }
    node.addEvent(Event(Event::Type::Get, by, value));
    return;
  }
  if (get.IsSequence()) {
    for (size_t i = 0; i < get.size(); ++i) {
      logSetRecurseLeaves(at(set, i), get[i], was_parsed, is_default, node[i], by);
    }
  } else if (get.IsMap()) {
    for (const auto& kv : get) {
      const std::string key = kv.first.Scalar();
      logSetRecurseLeaves(at(set, key), kv.second, was_parsed, is_default, node[key], by);
    }
  }
}

void Introspection::logRemovesRec(const YAML::Node& present, Node& node, const By& by) {
  // Handle the scalar case.
  if (!node.lastValue().empty() && (!present || !present.IsScalar())) {
    node.addEvent(Event(Event::Type::Remove, by));
  }

  // Recurse through all children.
  for (size_t i = 0; i < node.list.size(); ++i) {
    logRemovesRec(at(present, i), node.list[i], by);
  }
  for (auto& [key, child] : node.map) {
    logRemovesRec(at(present, key), child, by);
  }
}

void Introspection::logClear(const By& by) {
  auto& instance = Introspection::instance();
  instance.initLog();
  instance.logRemovesRec(YAML::Node(YAML::NodeType::Undefined), instance.currentNode(), by);
  instance.finishLog();
}

void Introspection::logSingleEvent(const Event& event, const std::string& ns) {
  auto& instance = Introspection::instance();
  instance.initLog();
  instance.currentNode().atNamespace(ns).addEvent(event);
  instance.finishLog();
}

void Introspection::clear() {
  data_.clear();
  sources_.clear();
  sequence_id_ = 0;
}

void Introspection::enterNamespace(const std::string& name_space) {
  if (name_space.empty()) {
    return;
  }
  instance().current_namespace_.push_back(name_space);
}

void Introspection::exitNamespace() {
  if (!instance().current_namespace_.empty()) {
    instance().current_namespace_.pop_back();
  }
}

Node& Introspection::currentNode() {
  // Individual namespaces may be nested, so join first into a single string.
  const auto full_ns = joinNamespace(current_namespace_);
  return data_.atNamespace(full_ns);
}

void Introspection::initLog() {
  // Increment the sequence id for the next events.
  ++sequence_id_;
}

void Introspection::finishLog() {
  // Currently incrementally save all updates as processes can die (and introspection being slow if sort of fine)
  writeOutputData(Settings::instance().introspection.output);
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

nlohmann::json toJson(const Node& node, const std::string& key = "") {
  nlohmann::json j;
  if (!key.empty()) {
    j["key"] = key;
  }
  if (!node.history.empty()) {
    auto hist = nlohmann::json::array();
    for (const auto& event : node.history) {
      hist.push_back(toJson(event));
    }
    j["history"] = hist;
  }
  if (!node.list.empty()) {
    auto list = nlohmann::json::array();
    for (const auto& child : node.list) {
      list.push_back(toJson(child));
    }
    j["list"] = list;
  }
  if (!node.map.empty()) {
    // Use arrays to preserve order if desired.
    auto map = nlohmann::json::array();
    for (const auto& [key, child] : node.map) {
      map.push_back(toJson(child, key));
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
  try {
    std::filesystem::create_directories(output_dir);
  } catch (const std::filesystem::filesystem_error& e) {
    Logger::logError("Failed to create introspection output directory '" + output_dir.string() + "': '" +
                     std::string(e.what()) + "'. No introspection output will be written.");
    return;
  }

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
  Logger::logInfo("Wrote config introspection output to '" + json_path.string() + "'.");
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

Introspection& Introspection::instance() {
  static Introspection instance;
  return instance;
}

YAML::Node Introspection::at(const YAML::Node& node, const std::string& key) {
  return (node && node.IsMap()) ? node[key] : YAML::Node(YAML::NodeType::Undefined);
}

YAML::Node Introspection::at(const YAML::Node& node, size_t index) {
  return (node && node.IsSequence()) ? node[index] : YAML::Node(YAML::NodeType::Undefined);
}

}  // namespace config::internal
