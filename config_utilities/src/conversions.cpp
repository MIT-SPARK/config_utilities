#include "config_utilities/types/conversions.h"

#include <thread>

namespace config {

std::string CharConversion::toIntermediate(char value) { return {value}; }

char CharConversion::fromIntermediate(const std::string& value) {
  // TODO(nathan) logging
  if (value.empty()) {
    return '\0';
  }

  return value.at(0);
}

int ThreadNumConversion::toIntermediate(int value) { return value; }

int ThreadNumConversion::fromIntermediate(int value) {
  if (value <= 0) {
    return std::thread::hardware_concurrency();
  } else {
    return value;
  }
}

}  // namespace config
