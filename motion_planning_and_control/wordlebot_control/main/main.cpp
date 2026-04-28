#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "wordlebot_control/wordle_bot_control_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto wordlebot_control = std::make_shared<WordleBotControlNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  // Spin in a background thread so the state monitor can update
  auto spin_thread = std::make_unique<std::thread>(
    [&executor, &wordlebot_control]() {
      executor.add_node(wordlebot_control->getNodeBaseInterface());
      executor.spin();
      executor.remove_node(wordlebot_control->getNodeBaseInterface());
    });

  wordlebot_control->setupScene();
  wordlebot_control->run();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
