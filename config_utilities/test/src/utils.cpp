#include "config_utilities/test/utils.h"

#include <gtest/gtest.h>

namespace config::test {

bool expectEqual(const YAML::Node& a, const YAML::Node& b) {
  EXPECT_EQ(a.Type(), b.Type());
  if (a.Type() != b.Type()) {
    return false;
  }
  switch (a.Type()) {
    case YAML::NodeType::Scalar:
      EXPECT_EQ(a.Scalar(), b.Scalar());
      return a.Scalar() == b.Scalar();
    case YAML::NodeType::Sequence:
      EXPECT_EQ(a.size(), b.size());
      if (a.size() != b.size()) {
        return false;
      }
      for (size_t i = 0; i < a.size(); ++i) {
        EXPECT_TRUE(expectEqual(a[i], b[i]));
        if (!expectEqual(a[i], b[i])) {
          return false;
        }
      }
      return true;
    case YAML::NodeType::Map:
      EXPECT_EQ(a.size(), b.size());
      if (a.size() != b.size()) {
        return false;
      }
      for (const auto& kv_pair : a) {
        const std::string key = kv_pair.first.Scalar();
        if (!b[key]) {
          ADD_FAILURE() << "Key " << key << " not found in b.";
          return false;
        }
        EXPECT_TRUE(expectEqual(kv_pair.second, b[key]));
        if (!expectEqual(kv_pair.second, b[key])) {
          return false;
        }
      }
      return true;
    case YAML::NodeType::Null:
      return true;
    case YAML::NodeType::Undefined:
      return true;
  }
  return false;
}

void TestLogger::logImpl(const internal::Severity severity, const std::string& message) {
  messages_.emplace_back(severity, message);
}

}  // namespace config::test
