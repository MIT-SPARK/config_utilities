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

#include <string>

namespace config {

namespace internal {

/**
 * @brief Global settings for how config_utilities-based configs behave. These
 * can be dynamically set and changed throughout a program.
 */
struct Settings {
  // Singleton access to the global settings.
  static Settings& instance();

  /**
   * @brief Settings for how and what of configs is printed.
   */
  struct Printing {
    //! @brief Width of the 'toString()' output of configs.
    unsigned int width = 80u;

    //! @brief Indent after which values are printed aftert the field name.
    unsigned indent = 30u;

    //! @brief Indent for nested configs.
    unsigned int subconfig_indent = 3u;

    //! @brief If true, indicate which values are identical to the default.
    bool show_defaults = true;

    //! @brief If true, also display the unit of each parameter where provided.
    bool show_units = true;

    //! @brief If true integrate subconfig fields into the main config, if false print them as individual configs.
    bool inline_subconfigs = true;

    //! @brief If true, attempts to print floats and float-like fields with default stream precision.
    bool reformat_floats = true;

    //! @brief If true, prints fields that had no value present when being parsed.
    // TODO(lschmid): I think the implementation of was_parsed is actually also empty if parsing fails in some cases,
    // could double check if that's inportant.
    bool show_missing = false;

    //! @brief If true, print the type of subconfigs in the output.
    bool show_subconfig_types = true;

    //! @brief If true, indicate that a field is a virtual field in the output.
    bool show_virtual_configs = true;

    //! @brief If true show the enumeration info of failed checks in the output.
    bool show_num_checks = true;

    //! @brief If true print the meta fields of configs (e.g., type for virtual configs) in the config body. If false
    //! print only proper fields of the config.
    bool print_meta_fields = false;
  } printing;

  /**
   * @brief Settings for factory type registration and object creation
   */
  struct Factory {
    //! @brief The factory will look for this param to deduce the type of the object to be created.
    std::string type_param_name = "type";
  } factory;

  /**
   * @brief Settings to load external libraries and their modules into the factories.
   */
  struct ExternalLibraries {
    //! @brief Whether or not loading external libraries are enabled
    bool enabled = true;

    //! @brief Whether or not loading and unloading libraries should be verbose
    bool verbose_load = true;

    //! @brief Log any factory creation from an external library (for debugging purposes)
    bool log_allocation = false;
  } external_libraries;

  /**
   * @brief Settings for introspection (debug-tool). By default, no introspection is performed.
   */
  struct Introspection {
    //! @brief Directory where the output files are written to. If empty, no files are written.
    std::string output = "";

    bool enabled() const;
  } introspection;

  //! @brief Control whether config_utilities is initialized to log to stdout/stderr by default
  bool disable_default_stdout_logger = false;

  /* Options to specify the logger and formatter at run time. */
  // Specify the default logger to be used for printing. Loggers register themselves if included.
  void setLogger(const std::string& name);

  // Specify the formatter to be used for printing. Formatters register themselves if included.
  void setFormatter(const std::string& name);

  // Reset all settings to their default values.
  void restoreDefaults() { *this = Settings(); }

 private:
  friend struct Visitor;
  Settings() = default;
  Settings(const Settings& other) = default;
  Settings& operator=(const Settings& other) = default;
  static Settings instance_;
};

// Define global settings as configs so they can be set/get with any interface.
void declare_config(Settings& config);
void declare_config(Settings::Printing& config);
void declare_config(Settings::Factory& config);
void declare_config(Settings::ExternalLibraries& config);
void declare_config(Settings::Introspection& config);

}  // namespace internal

// Access function in regular namespace.

/**
 * @brief Global settings for how config_utilities-based configs behave. These
 * can be dynamically set and changed throughout a program.
 */
inline internal::Settings& Settings() { return internal::Settings::instance(); }

}  // namespace config
