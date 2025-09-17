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

namespace config::internal {

Introspection::Event::By::By(Type type, const std::string& value) : type(type) {
  auto& sources = Introspection::instance().sources_[type];
  const auto it = sources.find(value);
  if (it != sources.end()) {
    index = it->second;
    return;
  }
  index = sources.size();
  sources[value] = index;
}

Introspection::Event::By Introspection::Event::By::file(const std::string& filename) {
  return By(Type::File, filename);
}

Introspection::Event::By Introspection::Event::By::arg(const std::string& args) { return By(Type::Arg, args); }

Introspection::Event::By Introspection::Event::By::substitution(const std::string& substitution_details) {
  return By(Type::Substitution, substitution_details);
}

Introspection::Event::Event(Type type, const By& by, const std::string& info, const std::string& value)
    : type(type), by(by), info(info), value(value) {}

void Introspection::addEvent(const Key& key, const Event& event) {
  if (!Settings::instance().introspection.enabled()) {
    return;
  }
  instance().data_[key].emplace_back(event);
}

void Introspection::logMerge(const YAML::Node& merged, const YAML::Node& input, const Event::By& by) {
  // Log all top-level keys in the parsed node as 'Set' events.
  const auto merged_flat = flatten(merged);
  const auto input_flat = flatten(input);
  for (const auto& [key, value] : input_flat) {
    const auto it = merged_flat.find(key);
    if (it == merged_flat.end()) {
      continue;  // Should never happen after merging. This indicates that an upstream key was overwritten.
    }
    const auto& previous_value = instance().lastValue(key);
    if (it->second == previous_value) {
      addEvent(key, Event(Event::Type::SetNonModified, by));
      continue;
    }
    if (it->second == value) {
      addEvent(key, Event(Event::Type::Set, by, "", value));
      continue;
    }
    addEvent(key, Event(Event::Type::Update, by, "", value));
  }
}

void Introspection::logDiff(const YAML::Node& before,
                            const YAML::Node& after,
                            const Event::By& by,
                            const Event::Type log_diff_as) {
  // Log all top-level keys in the merged node as 'Read' events if they have not been set before.
  const auto before_flat = flatten(before);
  const auto after_flat = flatten(after);
  for (const auto& [key, value] : before_flat) {
    const auto it = after_flat.find(key);
    if (it == after_flat.end()) {
      addEvent(key, Event(Event::Type::Remove, by));
    }
  }
  for (const auto& [key, value] : after_flat) {
    const auto it = before_flat.find(key);
    if (it == before_flat.end()) {
      addEvent(key, Event(Event::Type::Set, by, "", value));
      continue;
    }
    if (it->second == value) {
      continue;  // No change.
    }
    addEvent(key, Event(log_diff_as, by, "", value));
  }
}

const std::string& Introspection::lastValue(const Key& key) const {
  static const std::string empty_string = "";
  const auto it = data_.find(key);
  if (it == data_.end() || it->second.empty()) {
    return empty_string;
  }
  // Find the last event with a value change.
  for (auto rit = it->second.rbegin(); rit != it->second.rend(); ++rit) {
    if (!rit->value.empty()) {
      return rit->value;
    }
  }
  return empty_string;
}

void Introspection::clear() {
  data_.clear();
  sources_.clear();
}

// Serialization: conditional compilation.
#ifdef CONFIG_UTILS_ENABLE_JSON

nlohmann::json toJson(const Introspection::Event& event) {
  nlohmann::json j;
  j["type"] = event.type == Introspection::Event::Type::Set ? "s" : "r";
  j["by"] = std::string(1, static_cast<char>(event.by.type)) + std::to_string(event.by.index);
  if (!event.info.empty()) {
    j["info"] = event.info;
  }
  if (!event.value.empty()) {
    j["val"] = event.value;
  }
  return j;
}

nlohmann::json toJson(const Introspection::Data& data) {
  nlohmann::json j;
  for (const auto& [key, events] : data) {
    nlohmann::json event_array = nlohmann::json::array();
    for (const auto& event : events) {
      event_array.push_back(toJson(event));
    }
    j[key] = event_array;
  }
  return j;
}

nlohmann::json toJson(const Introspection::Sources& sources) {
  nlohmann::json j;
  for (const auto& [type, entries] : sources) {
    nlohmann::json j_entry;
    for (const auto& [value, index] : entries) {
      j_entry[index] = value;
    }
    j[std::string(1, static_cast<char>(type))] = j_entry;
  }
  return j;
}

void writeOutputDataImpl(const Introspection::Data& data,
                         const Introspection::Sources& sources,
                         const std::string& output_directory) {
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