#pragma once

#include <memory>

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

  // Move the end-effector to the specified target pose. Returns true if successful.
  bool moveToTarget(const geometry_msgs::msg::Pose & target);

  // Add a placeholder collision box to the planning scene.
  void setupCollisionScene();

  // Remove all collision objects added by this controller.
  void clearCollisionScene();

  // Build a geometry_msgs::Pose from XYZ position and RPY orientation.
  static geometry_msgs::msg::Pose buildPose(
    double x, double y, double z,
    double roll, double pitch, double yaw);

private:
  rclcpp::Node::SharedPtr node_;
  // move_group_ MUST be declared before visual_tools_ — initialisation order matters
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
};
