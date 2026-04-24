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
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

#include "wordleBot_control/wordle_bot_controller.hpp"

namespace mtc = moveit::task_constructor;

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
  mtc::Task task_;

  // Legacy single-goal interface (backward compat)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_complete_pub_;

  // Mission-level interface
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr set_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_mission_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_complete_pub_;

  // Collision object injection interface
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr add_collision_object_sub_;

  // Letter object interface — triggers pick-and-place mode
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr letter_object_sub_;
  geometry_msgs::msg::Pose letter_object_pose_;
  bool letter_object_received_{false};
  static constexpr const char * LETTER_OBJECT_ID = "letter_object";

  // Hardcoded place destination (world frame) — update once perception provides it
  static constexpr double PLACE_X = 0.0;
  static constexpr double PLACE_Y = 0.45;
  static constexpr double PLACE_Z = 0.02;

  // Mission state
  enum class MissionState { IDLE, RUNNING };

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

  // Arm the mission so missionLoop can begin
  void startMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Runs on mission_thread_: waits until armed, then dispatches to doPickAndPlace or goal loop
  void missionLoop();

  // Forward an incoming CollisionObject to the controller's planning scene
  void collisionObjectCallback(const moveit_msgs::msg::CollisionObject::SharedPtr msg);

  // Receive a letter object pose, add a 40 mm cube to the planning scene, arm pick-and-place
  void letterObjectCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Build the full MTC pick-and-place task for the given object pose
  mtc::Task createTask(const geometry_msgs::msg::Pose & object_pose);

  // Init, plan, and execute the MTC task; publishes mission_complete on success
  void doPickAndPlace();

  // Returns true if the trajectory controllers required for execution are active.
  // Logs a clear error and returns false if either is inactive or unreachable.
  bool controllersAreActive() const;
};
