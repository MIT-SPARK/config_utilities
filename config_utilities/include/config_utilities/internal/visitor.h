#pragma once

#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "config_utilities/internal/meta_data.h"
#include "config_utilities/internal/namespacing.h"
#include "config_utilities/internal/yaml_parser.h"
#include "config_utilities/types/enum.h"

namespace config::internal {

/**
 * @brief The visitor gets and sets information between the meta-data and configs. It is hidden via in-thread singleton
 * sequential access to make the public user interface cleaner. This is thread safe and creates one visitor instance per
 * thread, so it can be used as if single-threaded.
 */
struct Visitor {
  ~Visitor();
  static bool hasInstance();

  // Interfaces for all internal tools interact with configs through the visitor.
  // Set the data in the config from the node.
  template <typename ConfigT>
  static MetaData setValues(ConfigT& config,
                            const YAML::Node& node,
                            const bool print_warnings = true,
                            const std::string& name_space = "",
                            const std::string& field_name = "");

  // Get the data stored in the config.
  template <typename ConfigT>
  static MetaData getValues(const ConfigT& config,
                            const bool print_warnings = true,
                            const std::string& name_space = "",
                            const std::string& field_name = "");

  // Execute all checks specified in the config.
  template <typename ConfigT>
  static MetaData getChecks(const ConfigT& config, const std::string& field_name = "");

  // Interfaces for the config declaration interfaces to to expose their info to the visitor.
  static void visitName(const std::string& name);

  template <typename T, typename std::enable_if<!isConfig<T>(), bool>::type = true>
  static void visitField(T& field, const std::string& field_name, const std::string& unit);

  template <typename Conversion, typename T, typename std::enable_if<!isConfig<T>(), bool>::type = true>
  static void visitField(T& field, const std::string& field_name, const std::string& unit);

  template <typename EnumT>
  static void visitEnumField(EnumT& field,
                             const std::string& field_name,
                             const std::map<EnumT, std::string>& enum_names);

  static void visitCheck(const CheckBase& check);

  template <typename ConfigT, typename std::enable_if<isConfig<ConfigT>(), bool>::type = true>
  static void visitField(ConfigT& config, const std::string& field_name, const std::string& /* unit */);

  template <typename ConfigT>
  static void visitBase(ConfigT& config);

  static std::optional<YAML::Node> visitVirtualConfig(bool is_set, bool is_optional, const std::string& type);

 private:
  friend class config::NameSpace;
  friend void config::enter_namespace(const std::string& name);
  friend void config::exit_namespace();
  friend void config::clear_namespaces();
  friend std::string config::current_namespace();

  // Which operations to perform on the data.
  enum class Mode { kGet, kGetDefaults, kSet, kCheck };
  const Mode mode;

  // Create and access the meta data for the current thread. Lifetime of the meta data is managed internally by the
  // objects. Note that meta data always needs to be created before it can be accessed. In short, 'instance()' is only
  // to be used within the 'declare_config()' function, whereas 'create()' is to be used to extract data from a struct
  // by calling 'declare_config()'.
  explicit Visitor(Mode _mode, const std::string& _name_space = "", const std::string& _field_name = "");

  static Visitor& instance();

  /* Utility function to manipulate data. */
  // Helper function to get the default values of a config.
  template <typename ConfigT, typename std::enable_if<!is_virtual_config<ConfigT>::value, bool>::type = true>
  static MetaData getDefaults(const ConfigT& config);
  template <typename ConfigT, typename std::enable_if<is_virtual_config<ConfigT>::value, bool>::type = true>
  static MetaData getDefaults(const ConfigT& config);

  // Labels all fields in the data as default if they match the default values of the config.
  template <typename ConfigT>
  static void flagDefaultValues(const ConfigT& config, MetaData& data);

  // Extend the current visitor with a sub-visitor, replicating the previous specification.
  template <typename ConfigT>
  static MetaData subVisit(ConfigT& config, const bool print_warnings, const std::string& field_name = "");

  /* Internal data to handle visits. */
  // The messenger data to read from and return eventually.
  MetaData data;

  // Storage for user specified namespaces. Managed by namespacing.h.
  OpenNameSpace::Stack open_namespaces;

  // The current namespace used to get or set values.
  std::string name_space;

  // Keep track of which base configs were already visited to avoid duplicates in diamond inheritance.
  std::set<std::string> visited_base_configs;

  /* Member variables. */
  // Static registration to get access to the correct instance. Instancs are managed per thread and as a stack, i.e.
  // nested calls are possible and will always use the latest visitor.
  static thread_local std::vector<Visitor*> instances;
};

}  // namespace config::internal

#include "config_utilities/internal/visitor_impl.hpp"
