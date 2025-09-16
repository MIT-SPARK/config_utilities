#include "config_utilities/internal/introspection.h"

#include <filesystem>
#include <fstream>
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

Introspection::Event::Event(Type type, const By& by, const std::string& info, const std::string& value)
    : type(type), by(by), info(info), value(value) {}

void Introspection::addEvent(const Key& key, const Event& event) {
  if (!Settings::instance().introspection.enabled()) {
    return;
  }
  instance().data_[key].emplace_back(event);
}

void Introspection::logCliEntry(const YAML::Node& merged_node, const YAML::Node& parsed_node, const Event::By& by) {
  // Log all top-level keys in the parsed node as 'Set' events.
  const auto merged_flat = flattenNamespace(merged_node);
  const auto parsed_flat = flattenNamespace(parsed_node);
  for (const auto& [key, value] : parsed_flat) {
    const auto it = merged_flat.find(key);
    if (it == merged_flat.end()) {
      continue;  // Should never happen after merging.
    }

    const std::string merged_value = yamlToString(it->second, false);
    const auto& previous_value = instance().lastValue(key);
    if (merged_value == previous_value) {
      addEvent(key, Event(Event::Type::SetNonModified, by));
      continue;
    }
    const std::string parsed_value = yamlToString(value, false);
    if (merged_value == parsed_value) {
      addEvent(key, Event(Event::Type::Set, by, "", parsed_value));
      continue;
    }
    addEvent(key, Event(Event::Type::Update, by, "", parsed_value));
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

}  // namespace config::internal