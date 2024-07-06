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
#include <memory>
#include <string>

#include "config_utilities/internal/checks.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/string_utils.h"

namespace config {

namespace internal {
struct Visitor;
}

/**
 * @brief Contains checks to support common requests for verification of std::filesystem::path and path strings.
 */
struct Path {
 private:
  // Common interface for all path-checks.
  class Check : public internal::CheckBase {
   public:
    Check(const std::filesystem::path& param, const std::string& name) : path_(param), name_(name) {}

    std::string name() const override { return name_; }

   protected:
    const std::filesystem::path path_;
    const std::string name_;
  };

 public:
  // Path conversion converts a path or string into its lexically normal form.
  static std::string toIntermediate(const std::filesystem::path& value, std::string&);
  static void fromIntermediate(const std::string& intermediate, std::filesystem::path& value, std::string&);
  static std::string toIntermediate(std::string value, std::string&);
  static void fromIntermediate(const std::string& intermediate, std::string& value, std::string&);

  // Checks to run on paths.
  /**
   * @brief Check if a path is not empty.
   * @param param The path to check.
   * @param name The name of the parameter to print in warnings.
   */
  class IsSet : public Check {
   public:
    IsSet(const std::filesystem::path& param, const std::string& name) : Check(param, name) {}

    bool valid() const override;
    std::string message() const override;
    std::unique_ptr<CheckBase> clone() const override;
  };

  /**
   * @brief Check if a path or file exists.
   * @param param The path to check.
   * @param name The name of the parameter to print in warnings.
   */
  class Exists : public Check {
   public:
    Exists(const std::filesystem::path& param, const std::string& name) : Check(param, name) {}

    bool valid() const override;
    std::string message() const override;
    std::unique_ptr<CheckBase> clone() const override;
  };

  /**
   * @brief Check if a path or file does not yet exist.
   * @param param The path to check.
   * @param name The name of the parameter to print in warnings.
   */
  class DoesNotExist : public Check {
   public:
    DoesNotExist(const std::filesystem::path& param, const std::string& name) : Check(param, name) {}

    bool valid() const override;
    std::string message() const override;
    std::unique_ptr<CheckBase> clone() const override;
  };

  /**
   * @brief Check if a path exists and is a file.
   * @param param The path to check.
   * @param name The name of the parameter to print in warnings.
   */
  class IsFile : public Check {
   public:
    IsFile(const std::filesystem::path& param, const std::string& name) : Check(param, name) {}

    bool valid() const override;
    std::string message() const override;
    std::unique_ptr<CheckBase> clone() const override;
  };

  /**
   * @brief Check if a path exists and is a directory.
   * @param param The path to check.
   * @param name The name of the parameter to print in warnings.
   */
  class IsDirectory : public Check {
   public:
    IsDirectory(const std::filesystem::path& param, const std::string& name) : Check(param, name) {}

    bool valid() const override;
    std::string message() const override;
    std::unique_ptr<CheckBase> clone() const override;
  };

  /**
   * @brief Check if a directory exists and is empty.
   * @param param The path to check.
   * @param name The name of the parameter to print in warnings.
   */
  class IsEmptyDirectory : public Check {
   public:
    IsEmptyDirectory(const std::filesystem::path& param, const std::string& name) : Check(param, name) {}

    bool valid() const override;
    std::string message() const override;
    std::unique_ptr<CheckBase> clone() const override;
  };

  /**
   * @brief Check if a path has a specific extension. Supports extensions with and without leading dots. E.g. ".txt" and
   * "txt" are both valid.
   * @param param The path to check.
   * @param extension The extension to check for.
   * @param name The name of the parameter to print in warnings.
   */
  class HasExtension : public Check {
   public:
    HasExtension(const std::filesystem::path& param, const std::string& extension, const std::string& name)
        : Check(param, name),
          extension_(extension.empty() ? "" : (extension[0] == '.' ? extension.substr(1) : extension)) {}

    bool valid() const override;
    std::string message() const override;
    std::unique_ptr<CheckBase> clone() const override;

   private:
    const std::string extension_;
  };
};

}  // namespace config
