/**
 * TODO: Shows how to use configs.
 */

#include "config_utilities/config.h"

#include <iostream>
#include <string>

#include <glog/logging.h>


namespace demo {



}  // namespace demo

int main(int argc, char** argv) {
  // Setup logging.
  FLAGS_alsologtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);

  return 0;
}
