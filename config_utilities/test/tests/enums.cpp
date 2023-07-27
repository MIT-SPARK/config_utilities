#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/test/utils.h"
#include "config_utilities/types/enum.h"
namespace config::test {

enum class MyEnum { A, B, C };
enum class OddEnum : int { X = -1, Y = 3, Z = 5 };

inline static const auto enum_definition = Enum<OddEnum>({{OddEnum::X, "X"}, {OddEnum::Y, "Y"}, {OddEnum::Z, "Z"}});

struct ConfigWithEnums {
  MyEnum my_enum;
  OddEnum odd_enum;
};

void declare_config(ConfigWithEnums& config) {
  name("ConfigWithEnums");
  field<Enum<MyEnum>>(config.my_enum, "my_enum");
  field<Enum<OddEnum>>(config.odd_enum, "odd_enum");
}

TEST(Enums, UserConversion) {
  std::cout << "Runnin Enum Unit test" << std::endl;
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

  e = Enum<MyEnum>::fromString("D", false);
  EXPECT_EQ(e, MyEnum::A);
  EXPECT_EQ(logger->numMessages(), 1);

  e = static_cast<MyEnum>(5);
  name = Enum<MyEnum>::toString(e);
  EXPECT_EQ(name, "<Invalid Enum Value>");
  EXPECT_EQ(logger->numMessages(), 2);

  logger->print();
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

  e = Enum<OddEnum>::fromString("D", false);
  EXPECT_EQ(e, OddEnum::Y);
  EXPECT_EQ(logger->numMessages(), 1);

  e = static_cast<OddEnum>(1);
  name = Enum<OddEnum>::toString(e);
  EXPECT_EQ(name, "<Invalid Enum Value>");
  EXPECT_EQ(logger->numMessages(), 2);

  logger->print();
}

TEST(Enums, FieldConversion) {}

}  // namespace config::test
