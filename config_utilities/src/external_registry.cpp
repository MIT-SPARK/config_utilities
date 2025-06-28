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

#include "config_utilities/external_registry.h"

#include <iostream>

#include <boost/dll.hpp>
#include <config_utilities/factory.h>
#include <config_utilities/settings.h>

namespace config {
namespace internal {

std::unique_ptr<ExternalRegistry> ExternalRegistry::s_instance_ = nullptr;

struct ExternalRegistry::RegistryEntry {
  ModuleInfo key;
  std::string type;
};

bool operator<(const ExternalRegistry::RegistryEntry& lhs, const ExternalRegistry::RegistryEntry& rhs) {
  return lhs.type == rhs.type ? lhs.key < rhs.key : lhs.type < rhs.type;
}

struct LibraryHolderImpl : LibraryHolder {
  explicit LibraryHolderImpl(const std::filesystem::path& library_path) {
    // These modes are chosen to somewhat closely mimic dlopen. It will both append appropriate prefixes and suffixes
    // (lib and .so for linux) and allows loading from LD_LIBRARY_PATH by just specfiying the library name.
    // Your mileage may vary on windows, though it looks like boost::dll does a good job picking defaults that
    // are equivalent between the platforms.
    const auto mode = boost::dll::load_mode::append_decorations | boost::dll::load_mode::search_system_folders;
    library = boost::dll::shared_library(library_path.string(), mode);
  }

  boost::dll::shared_library library;
};

LibraryGuard::LibraryGuard() {}

LibraryGuard::LibraryGuard(const std::filesystem::path& library) { libraries_.push_back(library); }

LibraryGuard::~LibraryGuard() { unload(); }

LibraryGuard::LibraryGuard(LibraryGuard&& other) {
  libraries_.insert(libraries_.end(), other.libraries_.begin(), other.libraries_.end());
  other.libraries_.clear();
}

LibraryGuard& LibraryGuard::operator=(LibraryGuard&& other) {
  libraries_.insert(libraries_.end(), other.libraries_.begin(), other.libraries_.end());
  other.libraries_.clear();
  return *this;
}

void LibraryGuard::unload() {
  for (const auto& library : libraries_) {
    ExternalRegistry::instance().unload(library);
  }

  libraries_.clear();
}

LibraryGuard::operator bool() const { return !libraries_.empty(); }

ExternalRegistry::~ExternalRegistry() {
  std::vector<std::string> libraries;
  for (const auto& [path, lib] : libraries_) {
    // NOTE(nathan) technically unreachable (library guards call unload first)
    libraries.push_back(path);
  }

  for (const auto& path : libraries) {
    // NOTE(nathan) technically unreachable (library guards call unload first)
    unload(path);
  }
}

void ExternalRegistry::unload(const std::filesystem::path& library_path) {
  if (Settings::instance().external_libraries.verbose_load) {
    // NOTE(nathan) this is separate from the logger becuase there is no guarantee that it will be visible to the user
    // if it is through the logger
    std::cerr << "[WARNING] Unloading external library: " << library_path << std::endl;
  }

  auto iter = entries_.find(library_path);
  if (iter != entries_.end()) {
    // Remove any factories that use code from the external library in question. This will also delete any factory
    // module maps that become empty (as they rely on types only defined in the plugin). While plugin libraries
    // shouldn't do this, it's easy to happen in practice and will cause segfaults
    for (const auto& entry : iter->second) {
      internal::ModuleRegistry::removeModule(entry.key, entry.type);
      all_entries_.erase(entry);
    }
  }

  for (const auto& internal_ref : instances_) {
    const auto actual_ref = internal_ref.lock();
    if (actual_ref) {
      actual_ref->cleanup();
    }
  }

  entries_.erase(library_path);
  libraries_.erase(library_path);
  instances_.clear();
}

struct RegistryLock {
  RegistryLock(std::function<void(const ModuleInfo&, const std::string&, const std::string&)> callback) {
    ModuleRegistry::lock(std::move(callback));
  }

  ~RegistryLock() { ModuleRegistry::unlock(); }
};

LibraryGuard ExternalRegistry::load(const std::filesystem::path& library_path) {
  if (!Settings::instance().external_libraries.enabled) {
    Logger::logError("External library loading is disallowed! Not loading " + library_path.string());
    return {};
  }

  if (Settings::instance().external_libraries.verbose_load) {
    Logger::logInfo("Loading external library '" + library_path.string() + "'.");
  }

  // this sets the global registry to do two things:
  //   - it disallows registration of previously registered factories (silently) for registrations linked in the plugin
  //   - it provides a callback with information about everything that can be successfully registered (so that we can
  //   unload it when the library unloads)
  RegistryLock lock([library_path](const auto& key, const auto& type, const auto&) {
    ExternalRegistry::registerType(library_path, {key, type});
  });

  // load the library. This has the effect of calling all static initializers in the library (during the loading
  // process), which registers all external factories
  std::unique_ptr<LibraryHolder> library;
  try {
    library = std::make_unique<LibraryHolderImpl>(library_path);
  } catch (const std::exception& e) {
    const auto filename = boost::dll::shared_library::decorate(library_path.filename().string());
    Logger::logError("Unable to load library '" + library_path.string() + "': '" + e.what() + "'. Please check that '" +
                     filename.string() + "' exists " +
                     (library_path.has_parent_path() ? "at '" + library_path.parent_path().string() + "'"
                                                     : "on LD_LIBRARY_PATH or the platform equivalent"));
  }

  if (!library) {
    return {};
  }

  instance().libraries_.emplace(library_path, std::move(library));
  // return a scoped guard that controls when the library is unloaded
  return LibraryGuard(library_path);
}

void ExternalRegistry::registerType(const std::string& current_library, const RegistryEntry& entry) {
  auto& entries = instance().entries_;
  auto iter = entries.find(current_library);
  if (iter == entries.end()) {
    iter = entries.emplace(current_library, std::vector<RegistryEntry>()).first;
  }

  iter->second.push_back(entry);
  if (instance().all_entries_.count(entry)) {
    Logger::logWarning("Adding duplicate '" + entry.type + "' for " + entry.key.signature() + " in library '" +
                       current_library + "'");
  }

  instance().all_entries_.insert(entry);
}

void ExternalRegistry::logAllocation(const RegistryEntry& entry, void* pointer) {
  if (!instance().all_entries_.count(entry)) {
    return;
  }

  std::stringstream ss;
  ss << "Allocating '" << entry.type << "' @ " << pointer << " for signature " << entry.key.signature();
  Logger::logInfo(ss.str());
}

ExternalRegistry& ExternalRegistry::instance() {
  if (!s_instance_) {
    s_instance_.reset(new ExternalRegistry());
    if (Settings::instance().external_libraries.log_allocation) {
      ModuleRegistry::setCreationCallback([](const auto& info, const auto& type, void* pointer) {
        ExternalRegistry::logAllocation({info, type}, pointer);
      });
    }
  }

  return *s_instance_;
}

}  // namespace internal

internal::LibraryGuard loadExternalFactories(const std::filesystem::path& library_path) {
  return internal::ExternalRegistry::load(library_path);
}

internal::LibraryGuard loadExternalFactories(const std::vector<std::filesystem::path>& libraries) {
  internal::LibraryGuard guard;
  for (const auto& library_path : libraries) {
    guard = internal::ExternalRegistry::load(library_path);
  }

  return guard;
}

internal::LibraryGuard loadExternalFactories(const std::vector<std::string>& libraries) {
  internal::LibraryGuard guard;
  for (const auto& library_path : libraries) {
    guard = internal::ExternalRegistry::load(library_path);
  }

  return guard;
}

}  // namespace config
