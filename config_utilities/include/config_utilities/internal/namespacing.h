#pragma once

#include <string>

#include "config_utilities/internal/visitor.h"

namespace config {

class NameSpace {
 public:
  explicit NameSpace(const std::string& name_space);
  virtual ~NameSpace();

 private:
  std::string previous_ns_;
};

void enter_namespace(const std::string& name_space);

void exit_namespace();

}  // namespace config
