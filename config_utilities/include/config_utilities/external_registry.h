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

#include <filesystem>
#include <map>
#include <vector>

namespace config {
namespace internal {

/**
 * @brief Base class required to hide shared library implementation details
 */
struct LibraryHolder {
  virtual ~LibraryHolder() = default;
};

/**
 * @brief Scope-based guard that unloads an external library once it leaves scope
 */
struct LibraryGuard {
  using List = std::vector<LibraryGuard>;
  LibraryGuard();
  explicit LibraryGuard(const std::filesystem::path library);
  ~LibraryGuard();
  LibraryGuard(const LibraryGuard& other) = delete;
  LibraryGuard& operator=(const LibraryGuard& other) = delete;
  LibraryGuard(LibraryGuard&& other);
  LibraryGuard& operator=(LibraryGuard&& other);

  operator bool() const;
  void unload();

 private:
  std::filesystem::path library_;
};

/**
 * @brief Tracker for all externally-registered factories
 */
struct ExternalRegistry {
  struct RegistryEntry;

  ~ExternalRegistry();

  void unload(const std::filesystem::path& library_path);
  [[nodiscard]] static LibraryGuard load(const std::filesystem::path& library_path);
  static void registerType(const std::string& current_library, const RegistryEntry& entry);

  static ExternalRegistry& instance();

 private:
  ExternalRegistry() = default;

  std::map<std::string, std::unique_ptr<LibraryHolder>> libraries_;
  std::map<std::string, std::vector<RegistryEntry>> entries_;
};

}  // namespace internal

/**
 * @brief Populate factory methods from a provided shared object library
 * @param library_path Relative or absolute path to library (without extension or lib prefix)
 *
 * Warning: loading modules or code from external libraries that are not linked into the current
 * executable has some inherent brittleness! If the external library is unlinked from the current
 * executable before every instance has been destructed, then any use of those instances will segfault,
 * including on destruction. As long as all allocated (i.e., created) instances are deallocated before
 * the library guard goes out of scope, everything will work as intended. However, any static instances (e.g.,
 * singletons) that were created from an external library or use other object instances loaded from a shared library
 * (including a populated virtual config with a config struct type from the external library) WILL NOT BE destructed
 * before main exits. Careful attention is required to avoid segfaults in these cases. Consider using a
 * `ManagedInstance` in these cases.
 *
 * @returns Guard that unloads library once out of scope
 */
[[nodiscard]] internal::LibraryGuard loadExternalFactories(const std::filesystem::path& library_path);

/**
 * @brief Populate factory methods from a provided shared object libraries
 * @param libraries Relative or absolute paths to libraries (without extension or lib prefix)
 *
 * See single library method for caveats
 *
 * @returns Guard that unloads library once out of scope
 */
[[nodiscard]] internal::LibraryGuard::List loadExternalFactories(const std::vector<std::filesystem::path>& libraries);

}  // namespace config
