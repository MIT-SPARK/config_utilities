#include "config_utilities/test/utils.h"

#include <ros/package.h>

namespace config::test {

std::string getResourcePath() { return ros::package::getPath("config_utilities") + "/test/resources/"; }

YAML::Node loadResource(const std::string& name) { return YAML::LoadFile(getResourcePath() + name + ".yaml"); }

}  // namespace config::test
