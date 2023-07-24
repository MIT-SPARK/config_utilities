#include <cstdint>
#include <string>

namespace config {

struct CharConversion {
  static std::string toIntermediate(char value);
  static char fromIntermediate(const std::string& value);
};

struct ThreadNumConversion {
  static int toIntermediate(int value);
  static int fromIntermediate(int value);
};

}  // namespace config
