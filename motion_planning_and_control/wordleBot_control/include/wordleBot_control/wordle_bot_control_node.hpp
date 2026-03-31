#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "wordleBot_control/wordle_bot_controller.hpp"


class WordleBotControlNode
{
public:
  explicit WordleBotControlNode(const rclcpp::NodeOptions & options);

  // Used by the executor in main to spin this node
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  // Delegate scene setup to the controller (collision objects, etc.)
  void setupScene();

  // Run the main control loop (hard-coded goal sequence)
  void run();

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<WordleBotController> controller_;

  // Topic-based goal interface
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_complete_pub_;

  // Called when a goal is received on /wordle_bot/goal_pose
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};
