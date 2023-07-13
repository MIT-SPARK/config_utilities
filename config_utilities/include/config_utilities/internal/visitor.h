#pragma once

#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "config_utilities/internal/meta_data.h"
#include "config_utilities/internal/validity_checker.h"
#include "config_utilities/internal/yaml_parser.h"

namespace config::internal {

/**
 * @brief The visitor gets and sets information between the meta-data and configs. It is hidden via in-thread singleton
 * sequential access to make the public user interface cleaner. This is thread safe and creates one visitor instance per
 * thread, so it can be used as if single-threaded.
 */
struct Visitor {
  ~Visitor();

  // Interfaces for all internal tools interact with configs through the visitor.
  template <typename ConfigT>
  static MetaData setValues(ConfigT& config, const YAML::Node& node, bool print_warnings = true);

  template <typename ConfigT>
  static MetaData getValues(const ConfigT& config, bool print_warnings = true);

  template <typename ConfigT>
  static MetaData getChecks(const ConfigT& config);

  // Interfaces for the config declaration interfaces to to expose their info to the visitor.
  static void visitName(const std::string& name);

  template <typename T>
  static void visitField(T& field, const std::string& field_name, const std::string& unit);

  template <typename EnumT>
  static void visitEnumField(EnumT& field,
                             const std::string& field_name,
                             const std::map<EnumT, std::string>& enum_names);

  enum class CheckMode { kGT, kGE, kLT, kLE, kEQ, kNE };

  template <typename T>
  static void visitCheck(Visitor::CheckMode mode, const T& param, const T& value, const std::string& name);

  template <typename T>
  static void visitCheckInRange(const T& param, const T& lower, const T& upper, const std::string& name);

  static void visitCheckCondition(bool condition, const std::string& error_message);

  template <typename ConfigT>
  static void visitSubconfig(ConfigT& config, const std::string& field_name, const std::string& sub_namespace);

  template <typename ConfigT>
  static void visitBase(ConfigT& config, const std::string& sub_namespace);

  static std::optional<YAML::Node> visitVariableConfig(bool is_set, bool is_optional, const std::string& type);

 private:
  // Create and access the meta data for the current thread. Lifetime of the meta data is managed internally by the
  // objects. Note that meta data always needs to be created before it can be accessed. In short, 'instance()' is only
  // to be used within the 'declare_config()' function, whereas 'create()' is to be used to extract data from a struct
  // by calling 'declare_config()'.
  static Visitor create();

  static Visitor& instance();

  // Create one instance per thread and store the reference to it.
  explicit Visitor(std::thread::id _id);

  // Utility function to manipulate data.
  void extractErrors();

  // Which operations to perform on the data.
  enum class Mode { kUnspecified, kGet, kSet, kCheck } mode = Mode::kUnspecified;

  // Messenger data.
  MetaData data;

  // Internal data to handle visits.
  ValidityChecker checker;
  YamlParser parser;
  std::string name_prefix = "";
  std::string name_space = "";

  // Static registration to get access to the correct instance.
  inline static std::map<std::thread::id, Visitor*> instances;
  inline static std::mutex instance_mutex;

  // Member data.
  const std::thread::id id;
};

}  // namespace config::internal

#include "config_utilities/internal/visitor_impl.hpp"
