#include "LIVMapper.h"

#include <cstdio>
#include <cstdlib>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto mapper = std::make_shared<LIVMapper>();
  mapper->initializeSubscribersAndPublishers();
  mapper->run();
  std::fflush(stdout);
  std::fflush(stderr);
  std::_Exit(0);
  return 0;
}
