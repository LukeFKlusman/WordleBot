#include "interaction_execution/mission_coordinator.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MissionCoordinator>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
