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

#include <yaml-cpp/yaml.h>

namespace config {

/**
 * @brief Context for substitution parsing
 *
 * Together, the suffix, prefix and separator define the substitution grammar
 *
 *   TAG: LETTER (LETTER | DIGIT | "_" | "-")*
 *
 *   expression: .* | substitution | (\S* .* substitution .*)*
 *
 *   substitution: "prefix" TAG "separator" expression "suffix"
 */
struct ParserContext {
  //! Prefix for any substitution
  std::string prefix = R"""(\$<)""";
  //! Suffix for any substitution
  std::string suffix = R"""(>)""";
  //! Separator between substitution tag and substitution input
  std::string separator = R"""( *(\|| ) *)""";
  //! Whether or not substitutions are allowed
  bool allow_substitutions = true;
  //! Name-value pairs for use in substitution
  std::map<std::string, std::string> vars;

  //! @brief Flag that an error was encountered in parsing
  void error() const { error_ = true; }

  //! @brief Return if an error was encountered
  operator bool() const { return !error_; }

 private:
  mutable bool error_ = false;
};

struct Substitution {
  virtual ~Substitution() = default;

  /**
   * @brief Process arguments to substitution
   * @param[in] contents Arguments following {{
   */
  virtual std::string process(const ParserContext& context, const std::string& contents) const = 0;
};

class RegisteredSubstitutions {
 public:
  template <typename T>
  struct Registration {
    Registration();
  };

  ~RegisteredSubstitutions() = default;

  static const Substitution* getEntry(const std::string& tag);

 private:
  template <typename T>
  friend struct Registration;

  static RegisteredSubstitutions& instance();

  static void addEntry(const std::string& tag, std::unique_ptr<Substitution>&& proc);

  RegisteredSubstitutions();
  static std::unique_ptr<RegisteredSubstitutions> s_instance_;
  std::map<std::string, std::unique_ptr<Substitution>> entries_;
};

template <typename T>
RegisteredSubstitutions::Registration<T>::Registration() {
  RegisteredSubstitutions::addEntry(T::NAME, std::make_unique<T>());
}

/**
 * @brief Iterate through the node, resolving tags
 * @param[in] node Node to resolve tags for
 */
void resolveSubstitutions(YAML::Node node, const ParserContext& context = {}, bool strict = true);

}  // namespace config
