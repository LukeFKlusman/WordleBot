#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <optional>
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
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>


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

  // Apply a CollisionObject to the planning scene.
  // The obj.operation field (ADD / REMOVE / MOVE) is forwarded unchanged.
  // Sleeps 300 ms after applying to allow the planning scene to propagate.
  void addCollisionObject(const moveit_msgs::msg::CollisionObject & obj);

  // Attach a protective cylinder collision shape to the tool0 end-effector link.
  // The cylinder moves rigidly with the robot so the planner avoids it.
  void attachSensorCollisionObject();

  // Detach the sensor guard cylinder from tool0.
  void detachSensorCollisionObject();

  // Cancel the in-progress trajectory. Sets stop_requested_ and calls move_group_.stop().
  // Any blocking execute() or task.execute() call will return a non-SUCCESS error code.
  void stop();

  // Clear the stop flag before issuing a new motion so the motion is not immediately rejected.
  void clearStopFlag();

  // Move to the named "home" joint state. Does NOT apply path constraints so abort always succeeds.
  bool moveToHome();

  // Execute MTC stages 1-4: open gripper → move to pick → grasp → lift.
  // Returns true on success; false if stopped, planning failed, or execution failed.
  bool doPickPhase(const geometry_msgs::msg::Pose & object_pose);

  // Execute MTC stages 5-7: move to place → place → retreat → return home.
  // Must be called after a successful doPickPhase() so the object is attached in the planning scene.
  // Returns true on success; false if stopped, planning failed, or execution failed.
  bool doPlacePhase();

  // Build a geometry_msgs::Pose from XYZ position and RPY orientation.
  static geometry_msgs::msg::Pose buildPose(double x, double y, double z,
                                            double roll, double pitch, double yaw);

  // Return the path constraints used by moveToTarget: shoulder_lift_joint and wrist_3_joint bounds.
  // Pass the result to MTC Connect stages so pick-and-place planning is constrained identically.
  static moveit_msgs::msg::Constraints buildPathConstraints();

  // Compute the total joint displacement of a plan: Σ|Δq| over all joints and trajectory steps.
  // This is the L1 path length in joint space — used to validate motion efficiency.
  static double computeTotalJointDisplacement(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan);

  // Plan and execute a full MTC pick-and-place for the given object and place poses.
  bool doPickAndPlace(const geometry_msgs::msg::Pose & object_pose,
                      const geometry_msgs::msg::Pose & place_pose,
                      const std::string & object_id);

  static constexpr const char * LETTER_OBJECT_ID = "letter_object";

  // Five placement columns along the x-axis (P1=leftmost, P3=centre, P5=rightmost).
  // All slots share y=0.3 m and z=0.015 m; columns are 75 mm apart.
  struct PlaceSlot { double x, y, z; };
  static constexpr std::array<PlaceSlot, 5> PLACE_SLOTS = {{
    {-0.150, 0.3, 0.015},
    {-0.075, 0.3, 0.015},
    { 0.000, 0.3, 0.015},
    { 0.075, 0.3, 0.015},
    { 0.150, 0.3, 0.015},
  }};

private:
  std::vector<double> computeBestIK(const moveit::core::RobotStatePtr & current_state,
                                    const geometry_msgs::msg::Pose & target_pose);

  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> generateCandidatePlans(int num_attempts);

  moveit::planning_interface::MoveGroupInterface::Plan selectBestPlan(const std::vector<moveit::planning_interface::MoveGroupInterface::Plan> & plans,
                                                                      const std::vector<double> & q_start,
                                                                      const std::vector<double> & q_goal);

  void visualisePlan(const moveit::planning_interface::MoveGroupInterface::Plan * plan,
                     const std::string & title);

  // Monolithic MTC task used by doPickAndPlace.
  moveit::task_constructor::Task createTask(const geometry_msgs::msg::Pose & object_pose,
                                            const geometry_msgs::msg::Pose & place_pose,
                                            const std::string & object_id);

  // Phase-split task builders for stop/resume-aware pick-and-place.
  moveit::task_constructor::Task createPickTask(const geometry_msgs::msg::Pose & object_pose);
  moveit::task_constructor::Task createPlaceTask();

  rclcpp::Node::SharedPtr node_;
  // move_group_ MUST be declared before visual_tools_ — initialisation order matters
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  moveit::core::RobotStatePtr current_state;

  std::atomic<bool> stop_requested_{false};
};
