#include "rclcpp/rclcpp.hpp"
#include "move_goal_test/move_goal_test.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<move_goal_test::MoveGoalTestNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}