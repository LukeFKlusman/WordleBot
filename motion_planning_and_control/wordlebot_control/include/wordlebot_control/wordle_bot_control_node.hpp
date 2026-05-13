#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "wordlebot_control/wordle_bot_controller.hpp"
#include "wordlebot_control/msg/pick_place_task.hpp"

class WordleBotControlNode
{
public:
  explicit WordleBotControlNode(const rclcpp::NodeOptions & options);
  ~WordleBotControlNode();

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupScene();
  void run();

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<WordleBotController> controller_;

  // Legacy single-goal interface (backward compat)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_complete_pub_;

  // Mission-level interface
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr set_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_mission_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_complete_pub_;

  // Stop / Resume / Abort control (resume and abort not yet implemented)
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr abort_mission_sub_;

  // Live state publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_state_pub_;

  // Collision object injection interface
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr add_collision_object_sub_;

  // Letter object interface — triggers pick-and-place mode
  rclcpp::Subscription<wordlebot_control::msg::PickPlaceTask>::SharedPtr letter_object_sub_;

  // Clears all letter collision objects from the planning scene
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_letter_objects_sub_;

  // PickPlaceEntry is defined on WordleBotController (the controller owns planning).
  std::vector<WordleBotController::PickPlaceEntry> pick_place_queue_;

  // IDs of every letter collision object added this session, used for targeted removal.
  std::vector<std::string> tracked_letter_ids_;

  int letter_object_counter_{0};

  std::atomic<bool> stop_requested_{false};
  bool mission_running_{false};  // guarded by queue_mutex_

  std::vector<geometry_msgs::msg::Pose> goal_queue_;
  bool mission_armed_{false};
  std::mutex queue_mutex_;
  std::condition_variable cv_;
  std::thread mission_thread_;

  // Subscriber callbacks
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void setMissionCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void startMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void stopMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void resumeMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void abortMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void collisionObjectCallback(const moveit_msgs::msg::CollisionObject::SharedPtr msg);
  void letterObjectCallback(const wordlebot_control::msg::PickPlaceTask::SharedPtr msg);
  void clearLetterObjectsCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Mission loop (runs on mission_thread_)
  void missionLoop();
};
