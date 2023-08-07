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
   * @param name The name of the parameter to print in warnings.
   * @param extension The extension to check for.
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
