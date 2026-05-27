#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
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

  // =========
  // NODE LIFECYCLE
  // __________
  // Public entry points used by the component wrapper and executable.
  // __________
  // getNodeBaseInterface - Returns the wrapped rclcpp node base interface.
  // setupScene - Clears and rebuilds the static collision scene.
  // run - Keeps the process alive until ROS shutdown.
  // =========
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupScene();
  void run();

private:
  // =========
  // CORE ROS OBJECTS
  // __________
  // Node and controller objects that own the ROS and MoveIt runtime interfaces.
  // __________
  // node_ - Underlying ROS node for publishers, subscriptions, and parameters.
  // controller_ - Motion controller used to plan, execute, and manage scenes.
  // =========
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<WordleBotController> controller_;

  // =========
  // STATUS PUBLISHERS
  // __________
  // Outbound mission status topics consumed by tests, UI, and other nodes.
  // __________
  // motion_complete_pub_ - Publishes when a mission completes.
  // goal_reached_pub_ - Publishes after each individual goal or pick task.
  // mission_complete_pub_ - Publishes when all queued mission work is complete.
  // robot_state_pub_ - Publishes IDLE, RUNNING, or STOPPED state strings.
  // =========
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   motion_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   goal_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   mission_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_state_pub_;

  // =========
  // MISSION COMMAND SUBSCRIPTIONS
  // __________
  // Inbound topics that queue, start, stop, resume, or abort mission execution.
  // __________
  // set_mission_sub_ - Receives queued pose missions.
  // start_mission_sub_ - Arms queued mission execution.
  // stop_mission_sub_ - Requests a safe stop.
  // resume_mission_sub_ - Requests resume from STOPPED state.
  // abort_mission_sub_ - Requests abort from STOPPED state.
  // =========
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr set_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            start_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            stop_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            resume_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            abort_mission_sub_;

  // =========
  // BOARD AND SCENE SUBSCRIPTIONS
  // __________
  // Inbound topics that add collision objects, queue letter tasks, or clear board state.
  // __________
  // add_collision_object_sub_ - Receives external collision object updates.
  // letter_object_sub_ - Receives perception pick-and-place tasks.
  // clear_letter_objects_sub_ - Clears tracked letter objects and queued tasks.
  // clear_board_objects_sub_ - Alias for clearing all tracked board objects.
  // =========
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr         add_collision_object_sub_;
  rclcpp::Subscription<wordlebot_control::msg::PickPlaceTask>::SharedPtr     letter_object_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                       clear_letter_objects_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                       clear_board_objects_sub_;

  // =========
  // MANUAL MOTION SUBSCRIPTIONS
  // __________
  // Inbound topics for standalone arm, gripper, and scan commands while idle.
  // __________
  // open_gripper_sub_ - Requests full gripper open.
  // close_gripper_sub_ - Requests full gripper close.
  // return_home_sub_ - Requests return to configured working joints.
  // scan_and_sweep_sub_ - Requests the scan-and-sweep sequence.
  // =========
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr open_gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr close_gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr return_home_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_and_sweep_sub_;

  // =========
  // SCAN CONFIGURATION
  // __________
  // Cached scan-and-sweep parameters loaded from config/wordle_bot_controller.yaml.
  // __________
  // scan_sweep_poses_ - Six scan poses executed by the scan workflow.
  // scan_sweep_dwell_time_ - Dwell duration at scan poses.
  // =========
  std::array<geometry_msgs::msg::Pose, 6> scan_sweep_poses_;
  double scan_sweep_dwell_time_{1.5};

  // =========
  // BOARD TASK STATE
  // __________
  // Queues and object tracking for perception-driven pick-and-place missions.
  // __________
  // pick_place_queue_ - Pending pick-and-place tasks.
  // tracked_letter_ids_ - Letter object IDs added during this session.
  // tracked_scene_objects_ - External board object messages keyed by ID.
  // letter_object_counter_ - Fallback ID counter for unnamed letter tasks.
  // =========
  std::vector<WordleBotController::PickPlaceEntry> pick_place_queue_;
  std::vector<std::string> tracked_letter_ids_;  // IDs of all letter objects added this session
  std::unordered_map<std::string, moveit_msgs::msg::CollisionObject> tracked_scene_objects_;
  int letter_object_counter_{0};

  // =========
  // MISSION WORKER STATE
  // __________
  // Thread, queues, flags, and condition variables used by the mission worker.
  // __________
  // stop_requested_ - Atomic stop request consumed by the worker.
  // mission_running_ - Whether mission execution is active.
  // goal_queue_ - Pending goal-pose mission.
  // mission_armed_ - Worker wake condition after start request.
  // queue_mutex_ - Protects mission queues and mission_running_.
  // cv_ - Wakes the worker when a mission is armed.
  // mission_thread_ - Background mission execution thread.
  // in_stopped_state_ - Enables resume/abort callbacks after a stop.
  // resume_requested_ - Resume request flag.
  // abort_requested_ - Abort request flag.
  // stopped_cv_ - Wakes the STOPPED-state wait.
  // resume_pick_tasks_ - Remaining pick tasks after a stop.
  // resume_goal_tasks_ - Remaining goal tasks after a stop.
  // =========
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

  // =========
  // MISSION COMMAND CALLBACKS
  // __________
  // Topic callbacks that arm, start, stop, resume, or abort mission work.
  // __________
  // setMissionCallback - Queues a pose mission.
  // startMissionCallback - Arms queued work for the mission worker.
  // stopMissionCallback - Requests the current mission stop safely.
  // resumeMissionCallback - Requests resume from STOPPED state.
  // abortMissionCallback - Requests abort from STOPPED state.
  // =========
  void setMissionCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void startMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void stopMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void resumeMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void abortMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // =========
  // BOARD AND SCENE CALLBACKS
  // __________
  // Topic callbacks that update collision objects and perception task queues.
  // __________
  // collisionObjectCallback - Applies and tracks external collision object updates.
  // letterObjectCallback - Normalizes, tracks, and queues one pick-and-place task.
  // clearLetterObjectsCallback - Clears tracked board objects and queued pick tasks.
  // =========
  void collisionObjectCallback(const moveit_msgs::msg::CollisionObject::SharedPtr msg);
  void letterObjectCallback(const wordlebot_control::msg::PickPlaceTask::SharedPtr msg);
  void clearLetterObjectsCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // =========
  // MANUAL MOTION CALLBACKS
  // __________
  // Topic callbacks that run standalone arm, gripper, and scan actions while idle.
  // __________
  // returnHomeCallback - Requests return to configured working joints.
  // openGripperCallback - Requests full gripper open.
  // closeGripperCallback - Requests full gripper close.
  // scanAndSweepCallback - Starts scan-and-sweep in a detached worker.
  // =========
  void returnHomeCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void openGripperCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void closeGripperCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void scanAndSweepCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // =========
  // MISSION WORKER
  // __________
  // Background worker that dispatches queued goal or pick-and-place missions and handles stop/resume.
  // __________
  // missionLoop - Waits for armed missions, executes them, publishes state, and handles STOPPED state.
  // =========
  void missionLoop();

  // =========
  // POSE AND LEGACY COMMAND HELPERS
  // __________
  // Static helpers for pose normalization, yaw logging, and legacy home-pose detection.
  // __________
  // normalizePoseOrientation - Normalizes a pose quaternion, using identity for invalid input.
  // yawFromPose - Computes yaw from a pose quaternion.
  // nearlyEqual - Compares two doubles with a tolerance.
  // isLegacyReturnHomePose - Detects the old pose command that maps to returnToHome.
  // =========
  static bool normalizePoseOrientation(geometry_msgs::msg::Pose & pose,
                                       const std::string & label);
  static double yawFromPose(const geometry_msgs::msg::Pose & pose);
  static bool nearlyEqual(double lhs, double rhs, double tolerance);
  static bool isLegacyReturnHomePose(const geometry_msgs::msg::Pose & pose);
};
