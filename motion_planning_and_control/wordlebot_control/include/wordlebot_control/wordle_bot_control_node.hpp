#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
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

  // ---------------------------------------------------------------------------
  // Public Interface
  // ---------------------------------------------------------------------------
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupScene();
  void run();

private:
  // ---------------------------------------------------------------------------
  // Core node handles
  // ---------------------------------------------------------------------------
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<WordleBotController> controller_;

  // ---------------------------------------------------------------------------
  // Publishers
  // ---------------------------------------------------------------------------
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   motion_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   goal_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   mission_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_state_pub_;

  // ---------------------------------------------------------------------------
  // Mission control subscriptions
  // ---------------------------------------------------------------------------
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr set_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            start_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            stop_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            resume_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            abort_mission_sub_;

  // ---------------------------------------------------------------------------
  // Object & scene management subscriptions
  // ---------------------------------------------------------------------------
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr         add_collision_object_sub_;
  rclcpp::Subscription<wordlebot_control::msg::PickPlaceTask>::SharedPtr     letter_object_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                       clear_letter_objects_sub_;

  // ---------------------------------------------------------------------------
  // Arm utility subscriptions
  // ---------------------------------------------------------------------------
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr open_gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr close_gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr return_home_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_and_sweep_sub_;

  // ---------------------------------------------------------------------------
  // Scan-and-sweep parameters (loaded from config/scan_sweep_poses.yaml)
  // ---------------------------------------------------------------------------
  std::array<geometry_msgs::msg::Pose, 6> scan_sweep_poses_;
  double scan_sweep_dwell_time_{1.5};

  // ---------------------------------------------------------------------------
  // Pick-and-place state
  // PickPlaceEntry is defined on WordleBotController (the controller owns planning).
  // ---------------------------------------------------------------------------
  std::vector<WordleBotController::PickPlaceEntry> pick_place_queue_;
  std::vector<std::string> tracked_letter_ids_;  // IDs of all letter objects added this session
  int letter_object_counter_{0};

  // ---------------------------------------------------------------------------
  // Mission loop state
  // ---------------------------------------------------------------------------
  std::atomic<bool>    stop_requested_{false};
  bool                 mission_running_{false};  // guarded by queue_mutex_
  std::vector<geometry_msgs::msg::Pose> goal_queue_;
  bool                 mission_armed_{false};
  std::mutex           queue_mutex_;
  std::condition_variable cv_;
  std::thread          mission_thread_;

  // STOPPED-state synchronisation (stopped_cv_ shares queue_mutex_)
  std::atomic<bool>       in_stopped_state_{false};
  std::atomic<bool>       resume_requested_{false};
  std::atomic<bool>       abort_requested_{false};
  std::condition_variable stopped_cv_;

  // Tasks saved across a STOPPED state so resume can re-execute them
  std::vector<WordleBotController::PickPlaceEntry> resume_pick_tasks_;
  std::vector<geometry_msgs::msg::Pose>            resume_goal_tasks_;

  // ---------------------------------------------------------------------------
  // Mission Control Callbacks
  // ---------------------------------------------------------------------------
  void setMissionCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void startMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void stopMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void resumeMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void abortMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // ---------------------------------------------------------------------------
  // Object & Scene Management Callbacks
  // ---------------------------------------------------------------------------
  void collisionObjectCallback(const moveit_msgs::msg::CollisionObject::SharedPtr msg);
  void letterObjectCallback(const wordlebot_control::msg::PickPlaceTask::SharedPtr msg);
  void clearLetterObjectsCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // ---------------------------------------------------------------------------
  // Arm Utility Callbacks
  // ---------------------------------------------------------------------------
  void returnHomeCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void openGripperCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void closeGripperCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void scanAndSweepCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // ---------------------------------------------------------------------------
  // Mission Execution Loop
  // ---------------------------------------------------------------------------
  void missionLoop();
};
