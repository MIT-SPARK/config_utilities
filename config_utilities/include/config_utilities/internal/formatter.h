#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/meta_data.h"

namespace config::internal {

/**
 * @brief Abstract interface class for formatters. Formatters implement these methods to format the configs for
 * toString() calls and printing of validity checks.
 */
class Formatter {
 public:
  using Ptr = std::shared_ptr<Formatter>;

  // Constructor and destructor.
  Formatter() = default;
  virtual ~Formatter() = default;

  // Accessing the formatter.
  // Format all errors in the meta data into the display string.
  static std::string formatErrors(const MetaData& data,
                                  const std::string& what = "",
                                  const Severity severity = Severity::kWarning);

  // Format the content of a single config the display string.
  static std::string formatConfig(const MetaData& data);

  // Format the content of multiple configs the display string.
  static std::string formatConfigs(const std::vector<MetaData>& data);

  // Set the global formatter.
  static void setFormatter(Formatter::Ptr formatter);

 protected:
  virtual std::string formatErrorsImpl(const MetaData& data, const std::string& what, const Severity severity);
  virtual std::string formatConfigImpl(const MetaData& data);
  virtual std::string formatConfigsImpl(const std::vector<MetaData>& data);

 private:
  inline static Formatter::Ptr formatter_ = std::make_shared<Formatter>();
};

}  // namespace config::internal
