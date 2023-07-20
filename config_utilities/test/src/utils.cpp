#include "config_utilities/test/utils.h"

#include <ros/package.h>

namespace config::test {

std::string getResourcePath() { return ros::package::getPath("config_utilities") + "/test/resources/"; }

}  // namespace config::test
