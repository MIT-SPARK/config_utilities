#pragma once

#include <string>
#include <vector>

namespace config {

/**
 * @brief Enters a sub-namespace to get or set object fields. The namespace remains active until 'exit_namespace()' or
 * 'clear_namespaces()' is called.
 * @param name_space The name of the sub-namespace. This can be a nested namespace separated by '/', e.g. "foo/bar".
 */
void enter_namespace(const std::string& name_space);

/**
 * @brief Exits the last entered sub-namespace and re-enters the previous namespace, i.e. undoes the last call to
 * 'enter_namespace()'.
 */
void exit_namespace();

/**
 * @brief Exits all entered sub-namespaces and re-enters the root namespace.
 */
void clear_namespaces();

/**
 * @brief Switches the currently open sub-namespace to the specified namespace. This is equivalent to calling
 * 'exit_namespaces()' and then 'enter_namespace()'.
 */
void switch_namespace(const std::string& name_space);

/**
 * @brief Enters a sub-namespace to get or set object fields for the duration of the NameSpace's lifetime.
 */
class NameSpace {
 public:
  /**
   * @brief Enters a sub-namespace to get or set object fields for the duration of the NameSpace's lifetime.
   * @param name_space The name of the sub-namespace. This can be a nested namespace separated by '/', e.g. "foo/bar".
   */
  explicit NameSpace(const std::string& name_space);
  virtual ~NameSpace();

  // Additional methods to repeatedly enter and exit namespaces if requested.
  /**
   * @brief Exits the sub-namespace specified by this NameSpace and re-enters the previous namespace.
   */
  void exit();

  /**
   * @brief Re-enters the sub-namespace specified by this NameSpace. The namespace needs to be exited before it can be
   * re-entered.
   */
  void enter();

 private:
  friend void enter_namespace(const std::string& name_space);
  friend void exit_namespace();

  // Member data.
  const std::string sub_ns_;
  std::string previous_ns_;
  bool is_open_ = false;
};

}  // namespace config
