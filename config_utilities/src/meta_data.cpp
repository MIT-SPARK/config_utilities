#include "config_utilities/internal/meta_data.h"

namespace config::internal {

bool MetaData::hasErrors() const {
  if (!errors.empty()) {
    return true;
  }
  for (const MetaData& sub_config : sub_configs) {
    if (sub_config.hasErrors()) {
      return true;
    }
  }
  return false;
}

void MetaData::performOnAll(const std::function<void(MetaData&)>& func) {
  func(*this);
  for (MetaData& sub_config : sub_configs) {
    sub_config.performOnAll(func);
  }
}

void MetaData::performOnAll(const std::function<void(const MetaData&)>& func) const {
  func(*this);
  for (const MetaData& sub_config : sub_configs) {
    sub_config.performOnAll(func);
  }
}

}  // namespace config::internal
