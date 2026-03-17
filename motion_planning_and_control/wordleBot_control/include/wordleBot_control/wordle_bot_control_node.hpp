#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "wordleBot_control/wordle_bot_controller.hpp"


class WordleBotControlNode
{
public:
  explicit WordleBotControlNode(const rclcpp::NodeOptions & options);

  // Used by the executor in main to spin this node
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  // Delegate scene setup to the controller (collision objects, etc.)
  void setupScene();

  // Run the main control loop 
  void run();

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<WordleBotController> controller_;

};
