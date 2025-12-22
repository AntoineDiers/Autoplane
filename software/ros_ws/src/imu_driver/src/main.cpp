#include "ImuDriver.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuDriver>());
  rclcpp::shutdown();
  return 0;
}
