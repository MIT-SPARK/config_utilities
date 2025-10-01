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

#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "config_utilities/internal/meta_data.h"
#include "config_utilities/settings.h"

namespace config::internal {

/**
 * @brief Global registry as singleton to manage introspection state. The registry only checks for enabled state when
 * storing the final output, it is on the user side to check if introspection is enabled before adding events.
 */
class Introspection {
 public:
  // Data structures for introspection logging. The idea is to build a tree that mirrors the YAML structure of all keys
  // ever modified and stores a history of events for each key. Values are stored for scalars/leafs.

  /**
   *  @brief Source or owner of an event.
   */
  struct By {
    //! @brief Source type.
    enum Type : char { File = 'f', Arg = 'a', Substitution = 's', Programmatic = 'p', Config = 'c' } const type;

    //! @brief Index of the specific source in the sources lookup table.
    const size_t index;

    // Public facing constructors.
    static By file(const std::string& filename);
    static By arg(const std::string& args);
    static By substitution(const std::string& substitution_details = "");
    static By programmatic(const std::string& call);
    static By config(const std::string& config_name);

   private:
    By(Type type, const std::string& value);
    static size_t registerSource(Type type, const std::string& value);
  };

  // Registry of sources that have contributed to the current context. <SourceType, <SourceValue, SourceIndex>>
  using Sources = std::map<By::Type, std::map<std::string, size_t>>;

  /**
   * @brief Event data structure for introspection logging. The underlying information for each param is a history of
   * events.
   */
  struct Event {
    //! @brief Action type of events.
    enum Type : char {
      Set = 's',             // The value was set (new or overwritten).
      Update = 'u',          // The value was updated (typically retaining information from the previous state).
      SetNonModified = 'n',  // The value was set, but not modified (e.g. from a file, but same as before).
      SetFailed = 'f',       // An attempt was made to set the value, but it failed.
      Get = 'g',             // The value was read (successfully).
      GetDefault = 'd',      // The value was read but is the default value of the config.
      GetAbsent = 'a',       // An attempt was made to read the value, but it was not set.
      GetError = 'e',        // An attempt was made to read the value, but it was not successful.
      Remove = 'r'           // The value was removed (e.g. by a clear operation).
    } type;

    //! @brief Source or owner of the event.
    By by;

    //! @brief New value of the node if the value has changed. Values only indicate scalar nodes as YAML string.
    std::string value;

    //! @brief Sequence id of the event specifying the order of events in the global tree.
    const size_t sequence_id = 0;

    // Constructors.
    Event(Type type, const By& by, const std::string& value = "");

    /* Lookup functions */
    //! @brief Check if this event indicates unsetting/deletion of a value.
    bool isDeleteEvent() const;

    //! @brief Check if this event indicates a set (attempt) of a value.
    bool isSetEvent() const;

    //! @brief Check if this event indicates a get (attempt) of a value.
    bool isGetEvent() const;

    //! @brief Check if the value was modified in this event. This counts only setting the value to a new value, not
    //! removing it.
    bool valueModified() const;

    //! @brief Display the event as a string for debugging purposes.
    std::string display() const;
  };

  // History of events for a node.
  using History = std::vector<Event>;

  /**
   * @brief Tree data structure for storing introspection information.
   */
  struct Node {
    /* Data members */
    //! @brief The list of child nodes if the node is a sequence.
    std::vector<Node> list;

    //! @brief The map of child nodes if the node is a map. This is a vector to preserve insertion order.
    std::vector<std::pair<std::string, Node>> map;

    //! @brief The history of events for this node.
    History history;

    /* Interaction */
    //! @brief Get the current (most recent) value of the node.
    const std::string& lastValue() const;

    //! @brief Check if the node currently has a (scalar) value set.
    bool hasValue() const { return !last_value_.empty(); }

    //! @brief Add an event to the history of this node.
    void addEvent(const Event& event);

    //! @brief Get a child node by map-key. This will create the node if it does not exist.
    Node& at(const std::string& key);
    Node& operator[](const std::string& key) { return at(key); }

    //! @brief Get a child node by list-index. This will create the node if it does not exist.
    Node& at(size_t index);
    Node& operator[](size_t index) { return at(index); }

    //! @brief Get the child node at the specified namespace. This will create subsequent nodes if they do not exist.
    Node& atNamespace(const std::string& name_space);

    //! @brief Remove this node and all children from the tree.
    void clear();

    //! @brief Check if the node is empty (no history, no children).
    bool empty() const;

    //! @brief Serialize the node and all children to a YAML node at the specified sequence time step.
    YAML::Node toYaml(size_t at_sequence_id = std::numeric_limits<size_t>::max()) const;

    //! @brief Display the node and all children as a string for debugging purposes.
    std::string display(size_t indent = 0) const;

   private:
    // Caching of values for efficiency of tracking.
    std::string last_value_;

    // Recursive helper for toYaml.
    std::optional<YAML::Node> toYamlRec(size_t at_sequence_id) const;
  };

  // Singleton access.
  static Introspection& instance();
  Introspection(const Introspection&) = delete;
  Introspection& operator=(const Introspection&) = delete;

  // Logging interface.

  /**
   * @brief Log differences from merging a node into the current context. E.g. After a CLI entry is parsed and merged.
   * @todo (lschmid): Currently does not log nodes that have been removed by mergeMode::RESET.
   * @param merged The context node after merging the input node.
   * @param input The input node before it was merged into the context.
   * @param by The source of the entry (e.g. filename or arg).
   */
  static void logMerge(const YAML::Node& merged, const YAML::Node& input, const By& by);

  /**
   * @brief Log the difference between two nodes. E.g. before and after substitution resolution.
   * @note This does not log unsuccessful events (SetNonModified). All updates will be logged as Updated, even if the
   * value was completely overwritten.
   * @param after The current context node after all substitutions have been resolved.
   * @param by The source of the changes.
   * @param log_diff_as The event type to use for logging fields that have been changed. Choose Update or Set as
   * appropriate. Default is Update.
   */
  static void logDiff(const YAML::Node& after, const By& by, const Event::Type log_diff_as = Event::Type::Update);

  /**
   * @brief Log reading of values by the config system from the resulting meta data of the visitor.
   * @param set The meta data created by Visitor::setValues.
   * @param get_info The meta data created by Visitor::getInfo after the set call.
   */
  static void logSetValue(const MetaData& set, const MetaData& get_info);

  /**
   * @brief Log a clear event. This marks all current keys as removed.
   * @param by The source of the clear event.
   */
  static void logClear(const By& by);

  /**
   * @brief Log a single event. This is mostly useful for programmatic events that do not have a
   * specific key associated with them.
   * @param event The event to log.
   * @param ns Optionally specify a namespace at which to log the event. This is relatice to the already entered
   * namespaces.
   */
  static void logSingleEvent(const Event& event, const std::string& ns = "");

  /**
   * @brief Enter a namespace for all subsequent log Calls. This is useful for programmatic events that do not operate
   * at the root level but have the namespace obscured from the logging calls. Namespaces are all relative and will
   * stack if multiple are entered. Each enter namespace call must be matched by an exit namespace call to return to the
   * previous namespace.
   * @param name_space The namespace to enter.
   */
  static void enterNamespace(const std::string& name_space);

  /**
   * @brief Exit the current namespace and return to the previous namespace.
   */
  static void exitNamespace();

  /**
   * @brief Clear the introspection data.
   * @note This should not be invoked directly, mostly used for testing purposes.
   */
  void clear();

  /**
   * @brief Get the current introspection data tree.
   * @note This should probably not be invoked directly, mostly used for testing purposes.
   */
  const Node& data() const { return data_; }

  /**
   * @brief Write the introspection data to the specified output directory.
   * @note This should probably not be invoked directly, the output is written automatically at the end of processing if
   * enabled.
   */
  void writeOutputData(const std::string& output_dir);

 private:
  Introspection() = default;
  virtual ~Introspection() = default;

  // Root of the data tree tracked by this instance.
  Node data_;

  // Registry of sources that have contributed to the current context. <SourceType, <SourceValue, SourceIndex>>
  Sources sources_;

  // Counter for sequence ids. Events start at 1, leaving 0 for uninitialized events.
  size_t sequence_id_ = 0;

  std::vector<std::string> current_namespace_;

 private:
  // Setup a new logging event.
  void initLog();

  // Finish the current logging event.
  void finishLog();

  // Get the root node of the current namespace.
  Node& currentNode();

  // Recurse through the nodes and add events for merge logs.
  void logMergeRec(const YAML::Node& merged, const YAML::Node& input, const By& by, Node& node);

  // Recurse through the nodes and add events for difference logs.
  void logDiffRec(const YAML::Node& after, const By& by, Node& node, const Event::Type log_diff_as);

  // Recurse through the meta data and add get/set events.
  void logSetValueRec(const MetaData& set, const MetaData& get_info, Node& node);
  void logSetValueRecAbsent(const MetaData& get_info, Node& node);

  // Recurse through the meta data and log the events to the leaf nodes.
  void logSetRecurseLeaves(const YAML::Node& set,
                           const YAML::Node& get,
                           bool was_parsed,
                           bool is_default,
                           Node& node,
                           const By& by);

  // Find all values that were previously set but no longer present and log their removal.
  void logRemovesRec(const YAML::Node& present, Node& node, const By& by);

  // Lookup helpers to recurse through invalid nodes safely.
  static YAML::Node at(const YAML::Node& node, const std::string& key);
  static YAML::Node at(const YAML::Node& node, size_t index);
};

}  // namespace config::internal