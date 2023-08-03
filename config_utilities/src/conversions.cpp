#include "config_utilities/types/conversions.h"

#include <thread>

#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/visitor.h"

namespace config {

std::string CharConversion::toIntermediate(char value, std::string&) { return {value}; }

void CharConversion::fromIntermediate(const std::string& intermediate, char& value, std::string& error) {
  if (intermediate.empty()) {
    error = "Unable to parse char from empty string";
    return;
  }

  if (intermediate.size() > 1) {
    error = "Multiple character string will result in the first character being used";
    return;
  }
  value = intermediate.at(0);
}

int ThreadNumConversion::toIntermediate(int value, std::string&) { return value; }

int ThreadNumConversion::getNumThreads(int intermediate) {
  return intermediate <= 0 ? std::thread::hardware_concurrency() : intermediate;
}

}  // namespace config
