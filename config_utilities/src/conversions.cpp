#include "config_utilities/types/conversions.h"

#include <thread>

#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/visitor.h"

namespace config {

std::string CharConversion::toIntermediate(char value, std::string&) { return {value}; }

char CharConversion::fromIntermediate(const std::string& value, std::string& error) {
  if (value.empty()) {
    error = "Unable to parse char from empty string";
    return '\0';
  }

  if (value.size() > 1) {
    error = "Multiple character string will result in the first character being used";
  }

  return value.at(0);
}

int ThreadNumConversion::toIntermediate(int value, std::string&) { return value; }

int ThreadNumConversion::fromIntermediate(int value, std::string&) {
  if (value <= 0) {
    return std::thread::hardware_concurrency();
  } else {
    return value;
  }
}

}  // namespace config
