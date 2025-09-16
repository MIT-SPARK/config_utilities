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
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "config_utilities/settings.h"

namespace config::internal {

/**
 * @brief Global registry as singleton to manage introspection state. The registry only checks for enabled state when
 * storing the final output, it is on the user side to check if introspection is enabled before adding events.
 */
class Introspection {
 public:
  // Singleton access.
  static Introspection& instance();
  Introspection(const Introspection&) = delete;
  Introspection& operator=(const Introspection&) = delete;

  /**
   * @brief Event data structure for introspection logging. The underlying information for each param is a history of
   * events.
   */
  struct Event {
    // Action type of events.
    enum Type : char { Set = 's', Read = 'r', Update = 'u', SetNonModified = 'n', ReadDefault = 'd' } type;

    // Source/owner of the event.
    struct By {
      enum Type : char { File = 'f', Arg = 'a' } type;

      static By file(const std::string& filename);
      static By arg(const std::string& args);

      size_t index;

     private:
      By(Type type, const std::string& value);

    } by;

    // TODO(lschmid): Revisit this for more clarity.
    std::string info;   // Additional info (e.g. value for Set events).
    std::string value;  // New value if the value has changed.

    Event(Type type, const By& by, const std::string& info = "", const std::string& value = "");
  };

  // Full namespace of each param.
  using Key = std::string;
  // The underlying data is a ordered history of events for each key that was interacted with.
  using Data = std::map<Key, std::vector<Event>>;
  // Registry of sources that have contributed to the current context.
  using Sources = std::map<Event::By::Type, std::map<std::string, size_t>>;

  // Logging interface.

  /**
   * @brief Manually add an event to the introspection log.
   * @param key Full namespace of the parameter.
   * @param event The event to log.
   */
  static void addEvent(const Key& key, const Event& event);

  /**
   * @brief Log global context events from CLI parsing for each entry.
   * @param merged_node The current context node after merging the parsed node.
   * @param parsed_node The newly parsed node from the CLI entry.
   * @param by The source of the entry (e.g. filename or 'arg').
   */
  static void logCliEntry(const YAML::Node& merged_node, const YAML::Node& parsed_node, const Event::By& by);

  /**
   * @brief Clear the introspection data.
   * @note This should not be invoked directly, mostly used for testing purposes.
   */
  void clear();

  /**
   * @brief Write the introspection data to the specified output directory.
   * @note This should not be invoked directly, the output is written automatically at the end of parsing if enabled.
   */
  void writeOutputData(const std::string& output_dir);

 private:
  Introspection() = default;
  ~Introspection();

  // Get the last value of a key.
  const std::string& lastValue(const Key& key) const;

  Data data_;
  Sources sources_;
};

}  // namespace config::internal