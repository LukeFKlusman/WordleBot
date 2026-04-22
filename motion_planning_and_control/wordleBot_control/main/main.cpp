#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "wordleBot_control/wordle_bot_control_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto wordleBot_control = std::make_shared<WordleBotControlNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  // Spin in a background thread so the state monitor can update
  auto spin_thread = std::make_unique<std::thread>(
    [&executor, &wordleBot_control]() {
      executor.add_node(wordleBot_control->getNodeBaseInterface());
      executor.spin();
      executor.remove_node(wordleBot_control->getNodeBaseInterface());
    });

  wordleBot_control->setupScene();
  wordleBot_control->run();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
