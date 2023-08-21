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

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace config {

/**
 * @brief Enters a sub-namespace to get or set object fields. The namespace remains active until 'exit_namespace()' or
 * 'clear_namespaces()' is called.
 * @param name_space The sub-namespace to enter. This can be a nested namespace separated by slashes, e.g. "foo/bar".
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
 * @brief Get the current sub-namespace used for getting or setting params.
 */
std::string current_namespace();

/**
 * @brief Enters a sub-namespace to get or set object fields for the duration of the NameSpace's lifetime.
 */
class NameSpace {
 public:
  /**
   * @brief Enters a sub-namespace to get or set object fields for the duration of the NameSpace's lifetime.
   * @param name_space The sub-namespace to enter. This can be a nested namespace separated by slashes, e.g. "foo/bar".
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

  //  private:
  const std::string sub_ns_;
  std::string previous_ns_;
  bool is_open_ = false;
};

namespace internal {

// Struct that keeps namespaces around in the visitor and manages keeping namespaces constant when visiting.
struct OpenNameSpace {
  using Stack = std::vector<std::unique_ptr<OpenNameSpace>>;

  explicit OpenNameSpace(const std::string& name_space);

  static void performOperationWithGuardedNs(Stack& stack, const std::function<void()>& operation);

  bool isLocked() const;

 private:
  // Interaction. Private so the lock count can not be messed with.
  void lock();
  void unlock();

  // Data.
  int locks = 0;
  const NameSpace ns;
};

}  // namespace internal

}  // namespace config
