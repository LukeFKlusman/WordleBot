#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit/planning_pipeline/planning_pipeline.h>

class WordleMtcPlanner : public moveit::task_constructor::solvers::PlannerInterface
{
public:
  explicit WordleMtcPlanner(rclcpp::Node::SharedPtr node, std::string pipeline_name = "ompl");

  void init(const moveit::core::RobotModelConstPtr & robot_model) override;

  moveit::task_constructor::solvers::PlannerInterface::Result plan(
    const planning_scene::PlanningSceneConstPtr & from,
    const planning_scene::PlanningSceneConstPtr & to,
    const moveit::core::JointModelGroup * jmg,
    double timeout,
    robot_trajectory::RobotTrajectoryPtr & result,
    const moveit_msgs::msg::Constraints & path_constraints = moveit_msgs::msg::Constraints()) override;

  moveit::task_constructor::solvers::PlannerInterface::Result plan(
    const planning_scene::PlanningSceneConstPtr & from,
    const moveit::core::LinkModel & link,
    const Eigen::Isometry3d & offset,
    const Eigen::Isometry3d & target,
    const moveit::core::JointModelGroup * jmg,
    double timeout,
    robot_trajectory::RobotTrajectoryPtr & result,
    const moveit_msgs::msg::Constraints & path_constraints = moveit_msgs::msg::Constraints()) override;

  std::string getPlannerId() const override;

private:
  void initMotionPlanRequest(
    moveit_msgs::msg::MotionPlanRequest & req,
    const moveit::core::JointModelGroup * jmg,
    double timeout) const;

  moveit::task_constructor::solvers::PlannerInterface::Result planAndSelect(
    const planning_scene::PlanningSceneConstPtr & from,
    const moveit_msgs::msg::MotionPlanRequest & req,
    const moveit::core::JointModelGroup * jmg,
    robot_trajectory::RobotTrajectoryPtr & result);

  std::vector<double> computeBestIK(
    const planning_scene::PlanningSceneConstPtr & scene,
    const Eigen::Isometry3d & target_pose,
    const std::string & tip_link,
    const moveit::core::JointModelGroup * jmg) const;

  rclcpp::Node::SharedPtr node_;
  std::string pipeline_name_;
  planning_pipeline::PlanningPipelinePtr pipeline_;
};
