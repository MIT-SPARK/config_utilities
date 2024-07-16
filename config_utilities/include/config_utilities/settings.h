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

  /* Printing Settings. */
  // TODO(lschmid): These should probably be moved into a config or so for different formatters.
  // Width of the 'toString()' output of configs.
  unsigned int print_width = 80u;

  // Indent after which values are printed.
  unsigned int print_indent = 30u;

  // Indent for nested configs.
  unsigned int subconfig_indent = 3u;

  // If true, indicate which values are identical to the default.
  bool indicate_default_values = true;

  // If true, also display the unit of each parameter where provided.
  bool indicate_units = true;

  // If true integrate subconfig fields into the main config, if false print them separately.
  bool inline_subconfig_field_names = true;

  // If true, store all validated configs for global printing.
  bool store_valid_configs = true;

  // If true, attempts to print floats and float-like fields with default stream precision
  bool reformat_floats = true;

  // If true, prints fields that had no value present when being parsed
  bool print_missing = false;

  /* Factory settings */
  // The factory will look for this param to deduce the type of the object to be created.
  std::string factory_type_param_name = "type";

  /* Options to specify the logger and formatter at run time. */
  // Specify the default logger to be used for printing. Loggers register themselves if included.
  void setLogger(const std::string& name);

  // Specify the formatter to be used for printing. Formatters register themselves if included.
  void setFormatter(const std::string& name);

  // Reset all settings to their default values.
  void restoreDefaults() { *this = Settings(); }

 private:
  Settings() = default;
  Settings(const Settings& other) = default;
  Settings& operator=(const Settings& other) = default;
  static Settings instance_;
};

}  // namespace internal

// Access function in regular namespace.

/**
 * @brief Global settings for how config_utilities-based configs behave. These
 * can be dynamically set and changed throughout a program.
 */
inline internal::Settings& Settings() { return internal::Settings::instance(); }

}  // namespace config
