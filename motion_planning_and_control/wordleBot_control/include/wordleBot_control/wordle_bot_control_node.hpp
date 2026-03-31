#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include "wordleBot_control/wordle_bot_controller.hpp"


class WordleBotControlNode
{
public:
  explicit WordleBotControlNode(const rclcpp::NodeOptions & options);
  ~WordleBotControlNode();

  // Used by the executor in main to spin this node
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  // Delegate scene setup to the controller (collision objects, etc.)
  void setupScene();

  // Block main thread until shutdown; mission execution happens in missionThread_
  void run();

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<WordleBotController> controller_;

  // Topic-based goal interface (legacy single-goal; backward compat)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_complete_pub_;

  // Mission-level interface
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr set_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_mission_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_complete_pub_;

  // Collision object injection interface
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr add_collision_object_sub_;

  // Mission state (extensible for TC1.4 Stop/Resume/Abort)
  enum class MissionState { IDLE, RUNNING };

  // Thread-safe goal vector; protected by queue_mutex_
  // mission_armed_ must also be true before missionLoop will begin executing
  std::vector<geometry_msgs::msg::Pose> goal_queue_;
  bool mission_armed_{false};
  std::mutex queue_mutex_;
  std::condition_variable cv_;
  std::atomic<MissionState> mission_state_{MissionState::IDLE};
  std::thread mission_thread_;

  // Enqueue incoming goal and immediately arm (backward compat with /wordle_bot/goal_pose)
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Atomically replace goal queue with poses from msg; does NOT arm the mission
  void setMissionCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // Arm the mission so missionLoop can begin; no-op if queue is empty
  void startMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Runs on mission_thread_: waits until armed, then executes all goals sequentially
  void missionLoop();

  // Forward an incoming CollisionObject to the controller's planning scene
  void collisionObjectCallback(const moveit_msgs::msg::CollisionObject::SharedPtr msg);
};
