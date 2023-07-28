#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/test/utils.h"
#include "config_utilities/types/enum.h"

namespace config::test {

enum class MyEnum { A, B, C };
enum class OddEnum : int { X = -1, Y = 0, Z = 5 };

struct ConfigWithEnums {
  MyEnum my_enum;
  OddEnum odd_enum;
};

void declare_config(ConfigWithEnums& config) {
  name("ConfigWithEnums");
  field<Enum<MyEnum>>(config.my_enum, "my_enum");
  field<Enum<OddEnum>>(config.odd_enum, "odd_enum");
}

struct ConfigWithCustomEnumNames {
  MyEnum reggular_enum;
  MyEnum enum_with_custom_name;
  OddEnum odd_enum_with_custom_name;
};

void declare_config(ConfigWithCustomEnumNames& config) {
  name("ConfigWithCustomEnumNames");
  enum_field(config.reggular_enum, "reggular_enum");
  enum_field(config.enum_with_custom_name, "enum_with_custom_name", {"AAA", "BBB", "CCC"});
  enum_field(config.odd_enum_with_custom_name,
             "odd_enum_with_custom_name",
             {{OddEnum::X, "X-X"}, {OddEnum::Y, "Y-Y"}, {OddEnum::Z, "Z-Z"}});
}

auto init = Enum<OddEnum>::Initializer({{OddEnum::X, "X"}, {OddEnum::Y, "Y"}, {OddEnum::Z, "Z"}});

TEST(Enums, UserConversion) {
  Enum<MyEnum>::setNames({{MyEnum::A, "A"}, {MyEnum::B, "B"}, {MyEnum::C, "C"}});

  MyEnum e = MyEnum::A;
  std::string name = Enum<MyEnum>::toString(e);
  EXPECT_EQ(name, "A");

  e = Enum<MyEnum>::fromString("C");
  EXPECT_EQ(e, MyEnum::C);

  auto logger = TestLogger::create();
  e = Enum<MyEnum>::fromString("D");
  EXPECT_EQ(e, MyEnum::A);
  EXPECT_EQ(logger->numMessages(), 1);

  std::string msg = logger->messages().back().second;
  EXPECT_NE(msg.find("Enum conversion failed: Name 'D' is out of bounds for EnumT"), std::string::npos);
  EXPECT_NE(msg.find("with names ['A', 'B', 'C']"), std::string::npos);

  e = Enum<MyEnum>::fromString("D", false);
  EXPECT_EQ(e, MyEnum::A);
  EXPECT_EQ(logger->numMessages(), 1);

  e = static_cast<MyEnum>(5);
  name = Enum<MyEnum>::toString(e);
  EXPECT_EQ(name, "<Invalid Enum Value>");
  EXPECT_EQ(logger->numMessages(), 2);

  msg = logger->messages().back().second;
  EXPECT_NE(msg.find("Enum conversion failed: Value '5' is out of bounds for EnumT"), std::string::npos);
  EXPECT_NE(msg.find("with values ['0', '1', '2']"), std::string::npos);
}

TEST(Enums, OddEnum) {
  OddEnum e = OddEnum::X;
  std::string name = Enum<OddEnum>::toString(e);
  EXPECT_EQ(name, "X");

  e = Enum<OddEnum>::fromString("Z");
  EXPECT_EQ(e, OddEnum::Z);

  auto logger = TestLogger::create();
  e = Enum<OddEnum>::fromString("D");
  EXPECT_EQ(e, OddEnum::Y);
  EXPECT_EQ(logger->numMessages(), 1);

  std::string msg = logger->messages().back().second;
  EXPECT_NE(msg.find("Enum conversion failed: Name 'D' is out of bounds for EnumT"), std::string::npos);
  EXPECT_NE(msg.find("with names ['X', 'Y', 'Z']"), std::string::npos);

  e = Enum<OddEnum>::fromString("D", false);
  EXPECT_EQ(e, OddEnum::Y);
  EXPECT_EQ(logger->numMessages(), 1);

  e = static_cast<OddEnum>(1);
  name = Enum<OddEnum>::toString(e);
  EXPECT_EQ(name, "<Invalid Enum Value>");
  EXPECT_EQ(logger->numMessages(), 2);

  msg = logger->messages().back().second;
  EXPECT_NE(msg.find("Enum conversion failed: Value '1' is out of bounds for EnumT"), std::string::npos);
  EXPECT_NE(msg.find("with values ['-1', '0', '5']"), std::string::npos);
}

TEST(Enums, FieldConversion) {
  YAML::Node data;
  data["my_enum"] = "B";
  data["odd_enum"] = "Z";

  ConfigWithEnums config;
  internal::Visitor::setValues(config, data);
  EXPECT_EQ(config.my_enum, MyEnum::B);
  EXPECT_EQ(config.odd_enum, OddEnum::Z);

  data["my_enum"] = "D";
  data["odd_enum"] = "D";
  internal::MetaData meta_data = internal::Visitor::setValues(config, data);
  EXPECT_EQ(config.my_enum, MyEnum::B);
  EXPECT_EQ(config.odd_enum, OddEnum::Z);
  EXPECT_EQ(meta_data.errors.size(), 2);

  meta_data = internal::Visitor::getValues(config);
  EXPECT_EQ(meta_data.errors.size(), 0);
  EXPECT_EQ(meta_data.data["my_enum"].as<std::string>(), "B");
  EXPECT_EQ(meta_data.data["odd_enum"].as<std::string>(), "Z");

  config.my_enum = static_cast<MyEnum>(5);
  config.odd_enum = static_cast<OddEnum>(1);
  meta_data = internal::Visitor::getValues(config);
  EXPECT_EQ(meta_data.errors.size(), 2);
  EXPECT_EQ(meta_data.data["my_enum"].as<std::string>(), "<Invalid Enum Value>");
  EXPECT_EQ(meta_data.data["odd_enum"].as<std::string>(), "<Invalid Enum Value>");
}

TEST(Enums, enum_field) {
  YAML::Node data;
  data["reggular_enum"] = "B";
  data["enum_with_custom_name"] = "CCC";
  data["odd_enum_with_custom_name"] = "Z-Z";

  // Setting fields.
  ConfigWithCustomEnumNames config;
  internal::Visitor::setValues(config, data);
  EXPECT_EQ(config.reggular_enum, MyEnum::B);
  EXPECT_EQ(config.enum_with_custom_name, MyEnum::C);
  EXPECT_EQ(config.odd_enum_with_custom_name, OddEnum::Z);

  // Invalid names.
  data["reggular_enum"] = "D";
  data["enum_with_custom_name"] = "D";
  data["odd_enum_with_custom_name"] = "D";
  internal::MetaData meta_data = internal::Visitor::setValues(config, data);
  EXPECT_EQ(config.reggular_enum, MyEnum::B);
  EXPECT_EQ(config.enum_with_custom_name, MyEnum::C);
  EXPECT_EQ(config.odd_enum_with_custom_name, OddEnum::Z);
  EXPECT_EQ(meta_data.errors.size(), 3);

  // Getting fields.
  meta_data = internal::Visitor::getValues(config);
  EXPECT_EQ(meta_data.errors.size(), 0);
  EXPECT_EQ(meta_data.data["reggular_enum"].as<std::string>(), "B");
  EXPECT_EQ(meta_data.data["enum_with_custom_name"].as<std::string>(), "CCC");
  EXPECT_EQ(meta_data.data["odd_enum_with_custom_name"].as<std::string>(), "Z-Z");

  // Invalid values.
  config.reggular_enum = static_cast<MyEnum>(5);
  config.enum_with_custom_name = static_cast<MyEnum>(5);
  config.odd_enum_with_custom_name = static_cast<OddEnum>(1);
  meta_data = internal::Visitor::getValues(config);
  EXPECT_EQ(meta_data.errors.size(), 3);
  EXPECT_EQ(meta_data.data["reggular_enum"].as<std::string>(), "<Invalid Enum Value>");
  EXPECT_EQ(meta_data.data["enum_with_custom_name"].as<std::string>(), "<Invalid Enum Value>");
  EXPECT_EQ(meta_data.data["odd_enum_with_custom_name"].as<std::string>(), "<Invalid Enum Value>");

  // Check the global enum names are unchanged valid.
  MyEnum e = MyEnum::A;
  std::string name = Enum<MyEnum>::toString(e);
  EXPECT_EQ(name, "A");
  e = Enum<MyEnum>::fromString("C");
  EXPECT_EQ(e, MyEnum::C);

  OddEnum odd_e = OddEnum::X;
  name = Enum<OddEnum>::toString(odd_e);
  EXPECT_EQ(name, "X");
  odd_e = Enum<OddEnum>::fromString("Z");
  EXPECT_EQ(odd_e, OddEnum::Z);
}

}  // namespace config::test
