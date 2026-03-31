#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


class WordleBotController
{
public:
  explicit WordleBotController(rclcpp::Node::SharedPtr node);
  ~WordleBotController();

  // Move the end-effector to the specified target pose using free-space OMPL planning.
  // Applies floor collision, shoulder joint constraint, and syncs robot state before planning.
  bool moveToTarget(const geometry_msgs::msg::Pose & target);

  // Move to target via a Cartesian waypoint path: lift up → translate → descend.
  // More predictable path shape than free-space planning; returns false if < 90% of path planned.
  bool moveToTargetCartesian(const geometry_msgs::msg::Pose & target, double lift_height = 0.15);

  // Add floor collision and attach a sensor guard cylinder to the end effector.
  void setupCollisionScene();

  // Remove all collision objects and detach the sensor guard.
  void clearCollisionScene();

  // Attach a protective cylinder collision shape to the tool0 end-effector link.
  // The cylinder moves rigidly with the robot so the planner avoids it.
  void attachSensorCollisionObject();

  // Detach the sensor guard cylinder from tool0.
  void detachSensorCollisionObject();

  // Build a geometry_msgs::Pose from XYZ position and RPY orientation.
  static geometry_msgs::msg::Pose buildPose(double x, double y, double z,
                                            double roll, double pitch, double yaw);

  // Compute the total joint displacement of a plan: Σ|Δq| over all joints and trajectory steps.
  // This is the L1 path length in joint space — used to validate motion efficiency.
  static double computeTotalJointDisplacement(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan);

private:
  std::vector<double> computeBestIK(const moveit::core::RobotStatePtr & current_state,
                                    const geometry_msgs::msg::Pose & target_pose);

  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> generateCandidatePlans(int num_attempts);
  
  moveit::planning_interface::MoveGroupInterface::Plan selectBestPlan(const std::vector<moveit::planning_interface::MoveGroupInterface::Plan> & plans,
                                                                      const std::vector<double> & q_start,
                                                                      const std::vector<double> & q_goal);

  void visualisePlan(const moveit::planning_interface::MoveGroupInterface::Plan * plan,
                     const std::string & title);

  rclcpp::Node::SharedPtr node_;
  // move_group_ MUST be declared before visual_tools_ — initialisation order matters
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  moveit::core::RobotStatePtr current_state;
};
