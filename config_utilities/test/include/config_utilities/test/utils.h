#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "config_utilities/internal/logger.h"

namespace config::test {

bool expectEqual(const YAML::Node& a, const YAML::Node& b);

class TestLogger : public internal::Logger {
 public:
  using Message = std::pair<internal::Severity, std::string>;
  using Messages = std::vector<Message>;

  TestLogger() = default;
  virtual ~TestLogger() = default;

  const Messages& messages() const { return messages_; }
  int numMessages() const { return messages_.size(); }
  void clear() { messages_.clear(); }
  void print() const;

  static std::shared_ptr<TestLogger> create();

 protected:
  void logImpl(const internal::Severity severity, const std::string& message) override;

 private:
  Messages messages_;
};

}  // namespace config::test
