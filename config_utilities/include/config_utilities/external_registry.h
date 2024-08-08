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
#include <functional>
#include <map>
#include <mutex>
#include <vector>

namespace config {
namespace internal {

struct InstanceInfoBase {
  virtual void cleanup() = 0;
};

//! @brief Instance holder for non-managed instances
template <typename T>
struct InstanceInfo : InstanceInfoBase {
  struct View {
    T* instance = nullptr;
    std::unique_lock<std::mutex> lock;

    operator bool() const { return instance != nullptr; }
    typename std::add_lvalue_reference<T>::type operator*() const { return *instance; }
    T* operator->() const { return instance; }
  };

  InstanceInfo() : instance(nullptr) {}
  explicit InstanceInfo(std::unique_ptr<T>&& instance) : instance(std::move(instance)) {}

  void cleanup() override {}
  virtual View view() const { return {instance.get()}; }
  virtual bool valid() const { return instance != nullptr; }
  operator bool() const { return valid(); }

  std::unique_ptr<T> instance;
};

//! @brief Instance holder for managed instances (thread-safety required)
template <typename T>
struct ManagedInstanceInfo : InstanceInfo<T> {
  ManagedInstanceInfo() : InstanceInfo<T>() {}
  explicit ManagedInstanceInfo(std::unique_ptr<T>&& instance) : InstanceInfo<T>(std::move(instance)) {}

  void cleanup() override {
    std::lock_guard<std::mutex> lock(mutex);
    InstanceInfo<T>::instance.reset();
  }

  typename InstanceInfo<T>::View view() const override {
    return {InstanceInfo<T>::instance.get(), std::unique_lock<std::mutex>(mutex)};
  }

  bool valid() const override {
    std::lock_guard<std::mutex> lock(mutex);
    return InstanceInfo<T>::valid();
  }

  mutable std::mutex mutex;
};

}  // namespace internal

template <typename T>
struct ManagedInstance {
 public:
  using InstancePtr = std::shared_ptr<internal::InstanceInfo<T>>;

  ManagedInstance() : info_(new internal::InstanceInfo<T>()) {}
  explicit ManagedInstance(std::unique_ptr<T>&& instance) : info_(new internal::InstanceInfo<T>(std::move(instance))) {}
  explicit ManagedInstance(InstancePtr instance) : info_(std::move(instance)) {}
  ~ManagedInstance() = default;

  ManagedInstance(const ManagedInstance& other) = delete;
  ManagedInstance& operator=(const ManagedInstance& other) = delete;

  ManagedInstance(ManagedInstance&& other) : info_(std::exchange(other.info, nullptr)) {}
  ManagedInstance& operator=(ManagedInstance&& other) {
    info_ = std::exchange(other.info, nullptr);
    return *this;
  }

  operator bool() const { return info_ != nullptr && info_->valid(); }

  typename internal::InstanceInfo<T>::View get() const {
    if (!info_) {
      return {};
    }

    return info_->view();
  }

 private:
  std::shared_ptr<internal::InstanceInfo<T>> info_;
};

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

  /**
   * @brief De-registers all external types and unloads external libraries
   */
  ~ExternalRegistry();

  /**
   * @brief load an external library
   * @param library_path Path to the external library
   * @returns Scoped guard object that unloads library automatically
   */
  [[nodiscard]] static LibraryGuard load(const std::filesystem::path& library_path);

  /**
   * @brief unload an external library
   * @param library_path Path that the external library was loaded from
   *
   * Note: all instances created from this library must be destructed before unloading
   */
  void unload(const std::filesystem::path& library_path);

  /**
   * @brief Get singleton instance of the registry
   */
  static ExternalRegistry& instance();

  template <typename T>
  static ManagedInstance<T> createManaged(std::unique_ptr<T>&& underlying) {
    if (!underlying) {
      // nullptr results in invalid managed instance
      return {};
    }

    const auto library = getLibraryForAllocation(underlying.get());
    if (!library) {
      // return unmanaged instance, instance not allocated externally
      return ManagedInstance<T>(std::move(underlying));
    }

    auto& instances = instance().instances_;
    auto iter = instances.find(*library);
    if (iter == instances.end()) {
      iter = instances.emplace(*library, std::vector<std::weak_ptr<InstanceInfoBase>>()).first;
    }

    auto managed_info = std::make_shared<ManagedInstanceInfo<T>>(std::move(underlying));
    iter->second.push_back(managed_info);
    return ManagedInstance<T>(managed_info);
  };

 private:
  ExternalRegistry() = default;
  static std::unique_ptr<ExternalRegistry> s_instance_;

  static std::optional<std::string> getLibraryForAllocation(void* pointer);
  static void registerType(const std::string& current_library, const RegistryEntry& entry);
  static void registerInstance(const RegistryEntry& entry, void* pointer);

  // maps from libraries to other information
  std::map<std::string, std::unique_ptr<LibraryHolder>> libraries_;
  std::map<std::string, std::vector<RegistryEntry>> entries_;
  std::map<std::string, std::vector<std::weak_ptr<InstanceInfoBase>>> instances_;
  // map from registered types to libraries
  std::map<RegistryEntry, std::string> library_lookup_;
  // current registered allocations
  std::map<void*, RegistryEntry> external_allocations_;
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
