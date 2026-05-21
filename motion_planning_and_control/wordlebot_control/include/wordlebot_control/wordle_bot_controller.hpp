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
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


class WordleBotController
{
public:
  explicit WordleBotController(rclcpp::Node::SharedPtr node);
  ~WordleBotController();

  // ---------------------------------------------------------------------------
  // Data structures shared with the control node
  // ---------------------------------------------------------------------------

  // One entry per queued pick-and-place task. Moved here from the control node
  // so the controller's planning methods can accept it directly.
  struct PickPlaceEntry {
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
    moveit_msgs::msg::CollisionObject collision_object;
    std::string object_id;
  };

  // Holds a fully planned MTC move-to-goal task. Same chaining pattern as PlannedPickPlace.
  struct PlannedMoveToGoal {
    std::unique_ptr<moveit::task_constructor::Task> task;
    planning_scene::PlanningScenePtr end_scene;
    const moveit::task_constructor::SolutionBase* best_solution{nullptr};
  };

  // Holds a fully planned MTC task and metadata needed to execute it and chain
  // the next task. mtc::Task is non-copyable, so it is heap-allocated.
  struct PlannedPickPlace {
    std::unique_ptr<moveit::task_constructor::Task> task;
    planning_scene::PlanningScenePtr end_scene;   // cloned terminal scene for chaining
    const moveit::task_constructor::SolutionBase* best_solution{nullptr};
    std::string object_id;
  };

  // ---------------------------------------------------------------------------
  // Collision Scene Management
  // ---------------------------------------------------------------------------

  // Add floor collision and attach a sensor guard cylinder to the end effector.
  void setupCollisionScene();

  // Remove all collision objects and detach the sensor guard.
  void clearCollisionScene();

  // Apply a CollisionObject to the planning scene.
  // The obj.operation field (ADD / REMOVE / MOVE) is forwarded unchanged.
  // Sleeps 300 ms after applying to allow the planning scene to propagate.
  void addCollisionObject(const moveit_msgs::msg::CollisionObject & obj);

  // Remove collision objects with the given IDs from the planning scene.
  void clearLetterObjects(const std::vector<std::string> & ids);

  // Attach a protective cylinder collision shape to the tool0 end-effector link.
  // The cylinder moves rigidly with the robot so the planner avoids it.
  void attachSensorCollisionObject();

  // Detach the sensor guard cylinder from tool0.
  void detachSensorCollisionObject();

  // ---------------------------------------------------------------------------
  // Pick and Place
  // ---------------------------------------------------------------------------

  // Build an MTC task for one pick-and-place operation.
  // start_scene == nullptr → Stage 1 is CurrentState (reads live robot).
  // start_scene != nullptr → Stage 1 is FixedState seeded from start_scene.
  // include_return_home controls whether Stage 7 ("return home") is appended.
  // (Called internally by planPickAndPlace and doPickAndPlace.)

  // Plan one pick-and-place task without executing it.
  // start_scene: nullptr → use CurrentState (live robot, first task).
  //              non-null → use FixedState seeded from the previous task's terminal scene.
  // include_return_home: true only for the last task in a batch.
  // Returns a PlannedPickPlace; on failure, planned.task == nullptr.
  PlannedPickPlace planPickAndPlace(const PickPlaceEntry & entry,
                                    const planning_scene::PlanningScenePtr & start_scene,
                                    bool include_return_home);

  // Execute a previously planned task. Publishes the solution for visualisation
  // then calls task.execute(). Returns true on SUCCESS.
  bool executePlannedTask(PlannedPickPlace & planned);

  // Plan and execute a full MTC pick-and-place for the given object and place poses.
  bool doPickAndPlace(const geometry_msgs::msg::Pose & object_pose,
                      const geometry_msgs::msg::Pose & place_pose,
                      const std::string & object_id);

  // ---------------------------------------------------------------------------
  // Goal Navigation
  // ---------------------------------------------------------------------------

  // Plan a move-to-goal MTC task without executing it.
  // start_scene: nullptr → CurrentState (first goal); non-null → FixedState (chained).
  // include_return_home: true only for the last goal in the mission.
  PlannedMoveToGoal planMoveToGoal(const geometry_msgs::msg::Pose & goal_pose,
                                    const planning_scene::PlanningScenePtr & start_scene,
                                    bool include_return_home);

  // Execute a previously planned move-to-goal task. Returns true on SUCCESS.
  bool executePlannedMoveToGoal(PlannedMoveToGoal & planned);

  // Plan and execute one goal pose using MoveGroupInterface. Used when USE_MTC_FOR_GOALS == false.
  bool moveToGoal(const geometry_msgs::msg::Pose & goal_pose);

  // ---------------------------------------------------------------------------
  // Scan and Sweep
  // ---------------------------------------------------------------------------

  // Build a unified MTC task for the three Cartesian scan poses (poses[1..3]).
  // Uses alternating Connect(CartesianPath) + ComputeIK(GeneratePose) stages so
  // the planner can explore multiple IK configurations at each scan pose.
  // start_scene must be the terminal scene from executing pose 0.
  moveit::task_constructor::Task createScanAndSweepTask(
    const std::vector<geometry_msgs::msg::Pose> & scan_poses,
    const planning_scene::PlanningScenePtr & start_scene);

  // Execute the full scan-and-sweep sequence.
  // When USE_MTC_FOR_SCAN_SWEEP == false (default): poses[0] via moveToGoal, poses[1..3] via
  // computeCartesianPath with moveToGoal fallback, then returnToHome.
  // When USE_MTC_FOR_SCAN_SWEEP == true: existing MTC plan-all-then-execute path.
  // dwell_secs: how long to pause at each scan pose (0 = no dwell).
  bool runScanAndSweep(const std::vector<geometry_msgs::msg::Pose> & poses,
                       double dwell_secs = 0.0);

  // ---------------------------------------------------------------------------
  // Standalone Arm Motions
  // ---------------------------------------------------------------------------

  // Move the arm to the SRDF "home" named state using an MTC MoveTo stage.
  bool returnToHome();

  // Move the arm to the working pose defined in config/pose_working.yaml.
  bool returnToWorkingPose();

  // Open the gripper using an MTC MoveTo stage with the SRDF "open" named state.
  bool openGripper();

  // Close the gripper using an MTC MoveTo stage with the SRDF "closed" named state.
  bool closeGripper();

  // Returns true if the gripper joint positions are closer to the "closed" SRDF named state
  // than to the "open" state. Returns false on any state-monitor failure (safe default = open).
  bool isGripperClosed();

  // Move to safe recovery position (0.15, 0.15, 0.03), open gripper, return home.
  // held_object_id: if non-empty, the object is detached from gripper_tcp at the safe position
  // before the gripper opens. Pass "" when no object is known to be attached.
  // clearStopFlag() must be called before this.
  bool recoverObject(const std::string & held_object_id = "");

  // ---------------------------------------------------------------------------
  // Motion Control
  // ---------------------------------------------------------------------------

  // Cancel the in-progress trajectory. Sets stop_requested_ and calls move_group_.stop().
  // Any blocking execute() or task.execute() call will return a non-SUCCESS error code.
  void stop();

  // Clear the stop flag before issuing a new motion so the motion is not immediately rejected.
  void clearStopFlag();

  // ---------------------------------------------------------------------------
  // Helper Functions
  // ---------------------------------------------------------------------------

  // Build a geometry_msgs::Pose from XYZ position and RPY orientation.
  static geometry_msgs::msg::Pose buildPose(double x, double y, double z,
                                            double roll, double pitch, double yaw);

  // Return shoulder_lift_joint and wrist_3_joint path constraints used by MTC planning stages.
  static moveit_msgs::msg::Constraints buildPathConstraints();

  // Compute the total joint displacement of a plan: Σ|Δq| over all joints and trajectory steps.
  // This is the L1 path length in joint space — used to validate motion efficiency.
  static double computeTotalJointDisplacement(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan);

  // ---------------------------------------------------------------------------
  // Constants
  // ---------------------------------------------------------------------------

  // Change to false to use MoveGroupInterface sequential plan+execute per goal.
  // Change to true  to use MTC plan-all-then-execute-all (existing behaviour).
  static constexpr bool USE_MTC_FOR_GOALS = false;

  // Change to false to use MoveGroupInterface Cartesian path for scan-and-sweep.
  // Change to true  to use MTC plan-all-then-execute-all (existing behaviour).
  static constexpr bool USE_MTC_FOR_SCAN_SWEEP = false;

  static constexpr double kCartesianEefStep       = 0.01;  // 1 cm max step between waypoints
  static constexpr double kCartesianJumpThreshold = 0.0;   // disable joint-space jump check
  static constexpr double kCartesianMinFraction   = 0.95;  // fallback to moveToGoal below this

  static constexpr const char * LETTER_OBJECT_ID = "letter_object";

  // Five placement columns along the x-axis (P1=leftmost, P3=centre, P5=rightmost).
  // All slots share y=0.3 m and z=0.015 m; columns are 75 mm apart.
  struct PlaceSlot { double x, y, z; };
  static constexpr std::array<PlaceSlot, 5> PLACE_SLOTS = {{
    {-0.150, 0.35, 0.025},
    {-0.075, 0.35, 0.025},
    {-0.015, 0.35, 0.025},
    { 0.075, 0.35, 0.025},
    { 0.150, 0.35, 0.025},
  }};

private:
  // ---------------------------------------------------------------------------
  // Internal MTC task builders
  // ---------------------------------------------------------------------------

  // Build an MTC task for one pick-and-place operation.
  // start_scene == nullptr → Stage 1 is CurrentState (reads live robot).
  // start_scene != nullptr → Stage 1 is FixedState seeded from start_scene.
  // include_return_home controls whether Stage 7 ("return home") is appended.
  moveit::task_constructor::Task createTask(const geometry_msgs::msg::Pose & object_pose,
                                            const geometry_msgs::msg::Pose & place_pose,
                                            const std::string & object_id,
                                            const planning_scene::PlanningScenePtr & start_scene,
                                            bool include_return_home);

  // Phase-split task builders for stop/resume-aware pick-and-place.
  moveit::task_constructor::Task createPickTask(const geometry_msgs::msg::Pose & object_pose);
  moveit::task_constructor::Task createPlaceTask();

  // ---------------------------------------------------------------------------
  // MoveGroupInterface goal-navigation helpers (USE_MTC_FOR_GOALS == false)
  // ---------------------------------------------------------------------------

  // Solve IK for target_pose. First 5 attempts seed from warm-start config;
  // remaining 10 seed randomly. Applies 2π normalisation and wrist_3 [-π,π] clamp.
  // No shoulder rejection. Returns best joint vector by movement+functional cost, or empty.
  std::vector<double> computeBestIK(
    const moveit::core::RobotStatePtr & current_state,
    const geometry_msgs::msg::Pose & target_pose);

  // Call move_group_.plan() num_attempts times and return all successful plans.
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan>
  generateCandidatePlans(int num_attempts);

  // Return the plan with the lowest computeTotalJointDisplacement cost.
  // Returns a default-constructed (empty) plan if plans is empty.
  moveit::planning_interface::MoveGroupInterface::Plan
  selectBestPlan(
    const std::vector<moveit::planning_interface::MoveGroupInterface::Plan> & plans);

  // Move the end-effector to target_pose via computeCartesianPath.
  // Falls back to moveToGoal if the achieved fraction < kCartesianMinFraction.
  bool moveCartesianToWaypoint(const geometry_msgs::msg::Pose & target_pose);

  // ---------------------------------------------------------------------------
  // Member variables
  // move_group_ MUST be declared before visual_tools_ — initialisation order matters
  // ---------------------------------------------------------------------------
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  moveit::core::RobotStatePtr current_state;

  std::atomic<bool> stop_requested_{false};
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};
