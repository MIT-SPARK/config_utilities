#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "config_utilities/internal/checks.h"

namespace config::internal {

/**
 * @brief Struct that holds additional information about fields for printing.
 */
struct FieldInfo {
  // Name of the field. This is always given.
  std::string name;

  // Optional: Unit of the field.
  std::string unit;

  // The value of the field if the field is not a config.
  YAML::Node value;

  // Whether the field corresponds to its default value. Only queried if Settings().indicate_default_values is true.
  bool is_default = false;
};

// Struct to issue warnings. Currently used for parsing errors but can be extended to other warnings in the future.
struct Warning {
  virtual ~Warning() = default;
  Warning(const std::string& name, const std::string& message) : name_(name), message_(message) {}
  virtual std::string name() const { return name_; }
  virtual std::string message() const { return message_; }
  virtual std::unique_ptr<Warning> clone() const { return std::make_unique<Warning>(name_, message_); }

 private:
  std::string name_;
  std::string message_;
};

/**
 * @brief Meta-information struct that interfaces all the communication data when interacting with the configs. YAML is
 * used as internal data representation, so all conversions can be checked against yaml and all formatters, factories,
 * parsers can safely convert to yaml.
 */
struct MetaData {
  // Assignments and constructors.
  MetaData() = default;
  MetaData& operator=(const MetaData& other) {
    copyValues(other);
    return *this;
  }
  MetaData(const MetaData& other) { copyValues(other); }
  MetaData(MetaData&& other) = default;
  MetaData& operator=(MetaData&& other) = default;

  // Always get the name of the config if possible.
  std::string name;

  // Name of the field if the data is a sub-config.
  std::string field_name;

  // Whether the data stored belongs to a virtual config.
  bool is_virtual_config = false;

  // Yaml node used to get or set the data of a config.
  YAML::Node data;

  // All additional field information queried for printing.
  std::vector<FieldInfo> field_infos;

  // All checks that were performed on the config.
  std::vector<std::unique_ptr<CheckBase>> checks;

  // All error messages issued by the yaml parser (and potentially others in the future).
  std::vector<std::unique_ptr<Warning>> errors;

  // If a config has sub-configs, they are stored here.
  std::vector<MetaData> sub_configs;

  // Utility to look up if there's any error messages in the data or its sub-configs.
  bool hasErrors() const;

  // Utility function so not every class needs to write their own recursion.
  void performOnAll(const std::function<void(MetaData&)>& func);
  void performOnAll(const std::function<void(const MetaData&)>& func) const;

 private:
  void copyValues(const MetaData& other) {
    name = other.name;
    is_virtual_config = other.is_virtual_config;
    data = other.data;
    field_infos = other.field_infos;
    for (const auto& check : other.checks) {
      checks.emplace_back(check->clone());
    }
    for (const auto& error : other.errors) {
      errors.emplace_back(error->clone());
    }
    field_name = other.field_name;
    sub_configs = other.sub_configs;
  }
};

}  // namespace config::internal
