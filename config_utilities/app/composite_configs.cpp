#include <iostream>

#include <config_utilities/parsing/commandline.h>

int main(int argc, char* argv[]) {
  auto result = config::internal::loadFromArguments(argc, argv, false);
  std::cout << result << std::endl;
  return 0;
}
