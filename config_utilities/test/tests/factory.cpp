#include "config_utilities/factory.h"

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/test/utils.h"
#include "config_utilities/traits.h"

namespace config::test {

class Base {
 public:
  explicit Base(int i) : i_(i) {}
  virtual ~Base() = default;
  const int i_;
  virtual std::string name() const = 0;
};

class DerivedA : public Base {
 public:
  explicit DerivedA(int i) : Base(i) {}
  std::string name() const override { return "DerivedA"; }
  inline static const auto registration_ = config::Registration<Base, DerivedA, int>("DerivedA");
};

class DerivedB : public Base {
 public:
  explicit DerivedB(int i) : Base(i) {}
  std::string name() const override { return "DerivedB"; }
  inline static const auto registration_ = config::Registration<Base, DerivedB, int>("DerivedB");
};

class DerivedC : public Base {
 public:
  struct Config {
    float f = 0.f;
  };
  DerivedC(const Config& config, const int& i) : Base(i), config_(config) {}
  std::string name() const override { return "DerivedC"; }
  const Config config_;
  inline static const auto registration_ =
      config::RegistrationWithConfig<Base, DerivedC, DerivedC::Config, int>("DerivedC");
};

void declare_config(DerivedC::Config& config) {
  // Declare the config using the config utilities.
  config::name("DerivedC");
  config::field(config.f, "f");
}

class DerivedD : public Base {
 public:
  struct Config {
    int i = 0;
  };
  DerivedD(const Config& config, const int& i) : Base(i), config_(config) {}
  std::string name() const override { return "DerivedD"; }
  const Config config_;
  inline static const auto registration_ =
      config::RegistrationWithConfig<Base, DerivedD, DerivedD::Config, int>("DerivedD");
};

void declare_config(DerivedD::Config& config) {
  // Declare the config using the config utilities.
  config::name("DerivedD");
  config::field(config.i, "i");
}

TEST(Factory, create) {
  std::unique_ptr<Base> base = create<Base>("DerivedA", 1);
  EXPECT_TRUE(base);
  EXPECT_EQ(base->name(), "DerivedA");

  base = create<Base>("DerivedB", 1);
  EXPECT_TRUE(base);
  EXPECT_EQ(base->name(), "DerivedB");

  auto logger = TestLogger::create();
  base = create<Base>("NotRegistered", 1);
  EXPECT_FALSE(base);
  ASSERT_EQ(logger->numMessages(), 1);
  std::string msg = logger->messages().back().second;
  EXPECT_EQ(msg.find("No module of type 'NotRegistered' registered to the factory"), 0);
  EXPECT_NE(msg.find("Registered are: 'DerivedB', 'DerivedA'."), std::string::npos);

  base = create<Base>("DerivedA", 1, 2.f);
  EXPECT_FALSE(base);
  ASSERT_GE(logger->numMessages(), 2);
  msg = logger->messages().at(1).second;
  EXPECT_EQ(msg.find("Cannot create a module of type 'DerivedA': No modules registered to the factory"), 0);
  EXPECT_NE(
      msg.find(
          "Register modules using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct."),
      std::string::npos);
}

TEST(Factory, createWithConfig) {
  YAML::Node data;
  data["i"] = 3;
  data["f"] = 3.14f;
  data["type"] = "DerivedC";

  std::unique_ptr<Base> base = createFromYaml<Base>(data, 12);
  EXPECT_TRUE(base);
  EXPECT_EQ(base->name(), "DerivedC");
  EXPECT_EQ(dynamic_cast<DerivedC*>(base.get())->config_.f, 3.14f);
  EXPECT_EQ(base->i_, 12);

  auto logger = TestLogger::create();
  data["type"] = "NotRegistered";
  base = createFromYaml<Base>(data, 12);
  EXPECT_FALSE(base);
  EXPECT_EQ(logger->numMessages(), 1);
  std::string msg = logger->messages().back().second;
  EXPECT_EQ(msg.find("No module of type 'NotRegistered' registered to the factory"), 0);

  Settings().factory_type_param_name = "test_type";
  base = createFromYaml<Base>(data, 12);
  EXPECT_FALSE(base);
  EXPECT_EQ(logger->numMessages(), 2);
  msg = logger->messages().back().second;
  EXPECT_EQ(msg, "Could not read the param 'test_type' to deduce the type of the module to create.");

  data["test_type"] = "DerivedD";
  base = createFromYaml<Base>(data, 12);
  EXPECT_TRUE(base);
  EXPECT_EQ(base->name(), "DerivedD");
  EXPECT_EQ(dynamic_cast<DerivedD*>(base.get())->config_.i, 3);
}

}  // namespace config::test
