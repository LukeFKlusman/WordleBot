#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <map>
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
#include <moveit/task_constructor/storage.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_common.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>


class WordleBotController
{
public:
  explicit WordleBotController(rclcpp::Node::SharedPtr node);
  ~WordleBotController();

  // =========
  // SHARED DATA STRUCTURES
  // __________
  // Small task/result structures passed between the control node and controller.
  // __________
  // PickPlaceEntry - Stores one queued pick pose, place pose, collision object, and ID.
  // PlannedMoveToGoal - Stores a planned MTC move-to-goal task and selected solution.
  // PlannedPickPlace - Stores a planned MTC pick-and-place task and selected solution.
  // =========

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

  // =========
  // PICK AND PLACE
  // __________
  // Public MTC and MoveGroupInterface entry points for moving letters between pick and place poses.
  // __________
  // planPickAndPlace - Plans one MTC pick-and-place task without executing it.
  // executePlannedTask - Executes a previously planned MTC pick-and-place task.
  // doPickAndPlace - Plans and executes one MTC pick-and-place task.
  // executePickAndPlaceMoveGroup - Executes pick-and-place with sequential MoveGroupInterface phases.
  // =========

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

  // Execute one pick-and-place operation with MoveGroupInterface as a sequence
  // of live plan/execute phases. The incoming pick/place poses are used as exact
  // gripper_tcp targets. include_return_working returns to the configured
  // working pose after the final task in a batch.
  bool executePickAndPlaceMoveGroup(const PickPlaceEntry & entry,
                                    bool include_return_working);

  // =========
  // MOVE TO GOAL
  // __________
  // Public MTC and MoveGroupInterface entry points for absolute end-effector goal poses.
  // __________
  // planMoveToGoal - Plans one MTC move-to-goal task without executing it.
  // executePlannedMoveToGoal - Executes a previously planned MTC move-to-goal task.
  // moveToGoal - Plans and executes one MoveGroupInterface goal pose.
  // =========

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

  // =========
  // SCAN AND SWEEP
  // __________
  // Public scan sequence entry points for camera sweep motion.
  // __________
  // createScanAndSweepTask - Builds the unified MTC scan-and-sweep task.
  // runScanAndSweep - Executes the full scan-and-sweep sequence.
  // =========

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

  // =========
  // OTHER MOTION
  // __________
  // Public standalone arm, gripper, recovery, and stop-control motions.
  // __________
  // returnToHome - Moves the arm to configured working joints.
  // returnToWorkingPose - Moves the arm to the configured working pose.
  // openGripperFull - Opens the gripper to the SRDF full-open named state.
  // closeGripperFull - Closes the gripper to the SRDF closed named state.
  // openGripperOperational - Opens the gripper to the operational width.
  // closeGripperOperational - Closes the gripper to the operational width.
  // isGripperClosed - Estimates whether the gripper is closer to closed than open.
  // recoverObject - Runs object recovery to a safe position.
  // stop - Cancels active motion and sets the stop flag.
  // clearStopFlag - Clears the stop flag before new motion.
  // =========

  // Move the arm to working_joints from config/wordle_bot_controller.yaml.
  bool returnToHome();

  // Move the arm to the working pose defined in config/wordle_bot_controller.yaml.
  bool returnToWorkingPose();

  // Open the gripper to the SRDF "open" named state (hardware limit, full travel).
  bool openGripperFull();

  // Close the gripper to the SRDF "closed" named state (hardware limit, full travel).
  bool closeGripperFull();

  // Open the gripper to the operational pick/place width (pick_place.gripper_open_operational_width).
  // Publishes directly to /onrobot/finger_width_controller/commands.
  bool openGripperOperational();

  // Close the gripper to the operational pick/place width (pick_place.gripper_closed_operational_width).
  // Publishes directly to /onrobot/finger_width_controller/commands.
  bool closeGripperOperational();

  // Returns true if the gripper joint positions are closer to the "closed" SRDF named state
  // than to the "open" state. Returns false on any state-monitor failure (safe default = open).
  bool isGripperClosed();

  // Move to safe recovery position (0.15, 0.15, 0.03), open gripper, return home.
  // held_object_id: if non-empty, the object is detached from gripper_tcp at the safe position
  // before the gripper opens. Pass "" when no object is known to be attached.
  // clearStopFlag() must be called before this.
  bool recoverObject(const std::string & held_object_id = "");

  // Cancel the in-progress trajectory. Sets stop_requested_ and calls move_group_.stop().
  // Any blocking execute() or task.execute() call will return a non-SUCCESS error code.
  void stop();

  // Clear the stop flag before issuing a new motion so the motion is not immediately rejected.
  void clearStopFlag();

  // =========
  // COLLISION AND PLANNING SCENE FUNCTIONS
  // __________
  // Public planning-scene setup and collision-object management entry points.
  // __________
  // setupCollisionScene - Adds the floor and gameboard, then attaches the sensor guard.
  // clearCollisionScene - Removes static collision objects and detaches the sensor guard.
  // addCollisionObject - Applies an ADD, REMOVE, or MOVE collision object.
  // clearLetterObjects - Removes letter collision objects by ID.
  // attachSensorCollisionObject - Attaches the sensor guard cylinder to tool0.
  // detachSensorCollisionObject - Detaches the sensor guard cylinder from tool0.
  // =========
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

  // =========
  // MATH AND OTHER FUNCTION HELPERS
  // __________
  // Public static helpers used by tests, launch/config readers, and planning code.
  // __________
  // buildPose - Builds full quaternion pose [(xyz),(xyzw)] from XYZ and RPY.
  // buildPathConstraints - Builds orientation path constraints for transit moves.
  // buildJointLimitConstraints - Builds wrist joint constraints for planning.
  // computeTotalJointDisplacement - Computes total absolute joint motion in a plan.
  // computeContinuousJointRevolutionOffset - Computes the nearest whole-turn continuous-joint offset.
  // =========

  // Build a geometry_msgs::Pose from XYZ position and RPY orientation.
  static geometry_msgs::msg::Pose buildPose(double x, double y, double z,
                                            double roll, double pitch, double yaw);

  // Return shoulder_lift_joint and wrist_3_joint path constraints used by MTC planning stages.
  static moveit_msgs::msg::Constraints buildPathConstraints();

  // Return joint constraints that clamp wrist_2 and wrist_3 to [-π, π], applied
  // to all MTC Connect stages and MGI Cartesian planning to prevent drift.
  static moveit_msgs::msg::Constraints buildJointLimitConstraints();

  // Compute the total joint displacement of a plan: Σ|Δq| over all joints and trajectory steps.
  // This is the L1 path length in joint space — used to validate motion efficiency.
  static double computeTotalJointDisplacement(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan);

  // Return the whole-2π offset that places a continuous joint waypoint on the
  // same revolution as a reference state while preserving the planned motion.
  static double computeContinuousJointRevolutionOffset(double reference_position,
                                                       double first_waypoint_position)
  {
    constexpr double two_pi = 6.28318530717958647692;
    return std::round((reference_position - first_waypoint_position) / two_pi) * two_pi;
  }

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
  // =========
  // PICK AND PLACE
  // __________
  // Private builders for MTC pick-and-place task construction.
  // __________
  // createTask - Builds the full MTC pick-and-place task with optional chained start scene.
  // createPickTask - Reserved phase-split MTC pick task builder.
  // createPlaceTask - Reserved phase-split MTC place task builder.
  // =========
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

  // =========
  // MOTION HELPERS
  // __________
  // Shared IK, planning, Cartesian, trajectory-alignment, and velocity helpers.
  // __________
  // computeBestIK - Finds the best collision-aware IK solution for a target pose.
  // generateCandidatePlans - Collects candidate MoveGroupInterface plans before timeout.
  // selectBestPlan - Selects the lowest joint-displacement candidate plan.
  // moveCartesianToWaypoint - Executes one scan-profile Cartesian segment with fallback.
  // moveCartesianToWaypointWithScaling - Executes one Cartesian segment using explicit scaling.
  // alignWrist3TrajectoryToCurrentState - Aligns wrist_3 commands to the live continuous joint.
  // executeAlignedTaskSolution - Executes an MTC solution after wrist_3 trajectory alignment.
  // loadVelocityScalingProfiles - Loads velocity profile parameters.
  // queryCurrentStateMinDistance - Measures current planning-scene clearance.
  // computeTransitScaling - Maps clearance to transit velocity scaling.
  // =========
  // Solve IK for target_pose. First 5 attempts seed from warm-start config;
  // remaining 10 seed randomly. Applies 2π normalisation and wrist_3 [-π,π] clamp.
  // No shoulder rejection. Returns best joint vector by movement+functional cost, or empty.
  std::vector<double> computeBestIK(
    const moveit::core::RobotStatePtr & current_state,
    const geometry_msgs::msg::Pose & target_pose);

  // Call move_group_.plan() until enough plans succeed or timeout expires.
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan>
  generateCandidatePlans(double timeout_seconds, int min_successes);

  // Return the plan with the lowest computeTotalJointDisplacement cost.
  // Returns a default-constructed (empty) plan if plans is empty.
  moveit::planning_interface::MoveGroupInterface::Plan
  selectBestPlan(
    const std::vector<moveit::planning_interface::MoveGroupInterface::Plan> & plans);

  // Move the end-effector to target_pose via computeCartesianPath.
  // Falls back to moveToGoal if the achieved fraction < kCartesianMinFraction.
  bool moveCartesianToWaypoint(const geometry_msgs::msg::Pose & target_pose);

  // Cartesian helper with explicit velocity profile for non-scan use cases.
  bool moveCartesianToWaypointWithScaling(const geometry_msgs::msg::Pose & target_pose,
                                          double velocity_scaling,
                                          double acceleration_scaling,
                                          double min_fraction,
                                          const std::string & context);

  // Shift wrist_3 trajectory positions by whole revolutions so the first
  // commanded point is numerically close to the live continuous-joint state.
  bool alignWrist3TrajectoryToCurrentState(moveit_msgs::msg::RobotTrajectory & trajectory,
                                           const std::string & context);

  // Execute an MTC solution after applying the same wrist_3 alignment to each
  // serialized sub-trajectory in the ExecuteTaskSolution goal.
  moveit::core::MoveItErrorCode executeAlignedTaskSolution(
    moveit::task_constructor::Task & task,
    const moveit::task_constructor::SolutionBase & solution,
    const std::string & context);

  struct VelocityScalingProfiles {
    double scan_vel, scan_acc;
    double precise_vel, precise_acc;
    double transit_vel, transit_acc;
    double near_threshold, far_threshold;
  };

  VelocityScalingProfiles vel_profiles_;

  void loadVelocityScalingProfiles();
  double queryCurrentStateMinDistance() const;
  double computeTransitScaling(double d) const;

  // =========
  // COLLISION AND PLANNING SCENE FUNCTIONS
  // __________
  // Private planning-scene synchronisation and collision preflight helpers.
  // __________
  // refreshPlanningScene - Requests the latest scene state from move_group.
  // waitForWorldObjectState - Waits until a world collision object reaches the expected state.
  // waitForAttachedObjectState - Waits until an attached object reaches the expected state/link.
  // logAttachedObjects - Logs currently attached collision objects.
  // validateStateCollisionFree - Checks a robot state against the monitored planning scene.
  // =========
  // Refresh and verify planning scene state after world/attached object changes.
  bool refreshPlanningScene(const std::string & context);
  bool waitForWorldObjectState(const std::string & object_id,
                               bool should_exist,
                               const std::string & context,
                               double timeout_seconds = 1.0);
  bool waitForAttachedObjectState(const std::string & object_id,
                                  bool should_exist,
                                  const std::string & expected_link,
                                  const std::string & context,
                                  double timeout_seconds = 1.0);
  void logAttachedObjects(const std::string & context);

  // Returns false when the state is in collision and logs contact diagnostics.
  bool validateStateCollisionFree(const moveit::core::RobotState & state,
                                  const std::string & context,
                                  const moveit::core::JointModelGroup * jmg = nullptr);

  // =========
  // MATH AND OTHER FUNCTION HELPERS
  // __________
  // Small stateless utilities for parameters, poses, yaw, task-solution scoring, and trajectories.
  // __________
  // gripperTouchLinks - Lists links allowed to touch attached gripper objects.
  // offsetWorldZ - Offsets a pose along world Z.
  // findJointIndex - Finds a joint name index in a joint-name vector.
  // shortestRevoluteDelta - Computes shortest signed revolute-joint delta.
  // yawPenalty - Scores yaw error outside tolerance.
  // isPlaceYawSolution - Identifies MTC place-yaw solution stages.
  // targetYawFromInterfaceState - Extracts target yaw from an MTC interface state.
  // placeYawFromSolution - Recursively finds place yaw in an MTC solution.
  // accumulateTrajectoryMotionScore - Adds joint-motion cost from one robot trajectory.
  // accumulateSolutionMotionScore - Recursively adds joint-motion cost from an MTC solution.
  // scoreTaskSolution - Scores a complete MTC solution for selection.
  // alignWrist3JointTrajectoryToReference - Aligns one joint trajectory to a wrist_3 reference.
  // getIntParam - Declares and reads an integer parameter.
  // getDoubleParam - Declares and reads a double parameter.
  // getWorkingJoints - Reads configured working joint targets.
  // normalizePoseOrientation - Normalizes a pose quaternion.
  // yawFromPose - Computes yaw from a pose quaternion.
  // rotatePoseYaw - Returns a pose rotated by a yaw delta.
  // =========
  struct SolutionMotionScore {
    double joint_motion{0.0};
    double wrist_spin{0.0};
    double place_yaw_error{0.0};
    double place_yaw_penalty{0.0};
    double total{0.0};
    bool rejected{false};
    bool has_place_yaw{false};
    std::size_t trajectory_count{0};
  };

  static std::vector<std::string> gripperTouchLinks();
  static geometry_msgs::msg::Pose offsetWorldZ(geometry_msgs::msg::Pose pose, double dz);
  static std::optional<std::size_t> findJointIndex(
    const std::vector<std::string> & joint_names,
    const std::string & joint_name);
  static double shortestRevoluteDelta(double from, double to);
  static double yawPenalty(double actual_yaw,
                           double desired_yaw,
                           double tolerance,
                           double weight,
                           double & yaw_error);
  static bool isPlaceYawSolution(const moveit::task_constructor::SolutionBase & solution);
  static std::optional<double> targetYawFromInterfaceState(
    const moveit::task_constructor::InterfaceState * state);
  static std::optional<double> placeYawFromSolution(
    const moveit::task_constructor::SolutionBase & solution);
  static void accumulateTrajectoryMotionScore(
    const robot_trajectory::RobotTrajectory & trajectory,
    const moveit::core::JointModelGroup * jmg,
    SolutionMotionScore & score);
  static void accumulateSolutionMotionScore(
    const moveit::task_constructor::SolutionBase & solution,
    const moveit::core::JointModelGroup * jmg,
    SolutionMotionScore & score);
  static SolutionMotionScore scoreTaskSolution(
    const moveit::task_constructor::SolutionBase & solution,
    const moveit::core::JointModelGroup * jmg,
    double wrist_spin_weight,
    double wrist_spin_reject_threshold,
    double desired_place_yaw,
    double place_yaw_tolerance,
    double place_yaw_penalty_weight);
  static bool alignWrist3JointTrajectoryToReference(
    trajectory_msgs::msg::JointTrajectory & joint_trajectory,
    double & reference_wrist3,
    const std::string & context);
  static int getIntParam(const rclcpp::Node::SharedPtr & node,
                         const std::string & name,
                         int default_value);
  static double getDoubleParam(const rclcpp::Node::SharedPtr & node,
                               const std::string & name,
                               double default_value);
  static std::map<std::string, double> getWorkingJoints(const rclcpp::Node::SharedPtr & node);
  static bool normalizePoseOrientation(geometry_msgs::msg::Pose & pose,
                                       const std::string & label,
                                       bool identity_if_invalid = true);
  static double yawFromPose(const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::Pose rotatePoseYaw(geometry_msgs::msg::Pose pose,
                                                double yaw_delta);

  // =========
  // MEMBER VARIABLES
  // __________
  // Runtime ROS, MoveIt, scene-monitor, and stop-state objects owned by the controller.
  // __________
  // node_ - ROS node used for parameters, logging, and MoveIt interfaces.
  // move_group_ - Primary MoveGroupInterface for arm and gripper motion.
  // planning_scene_ - Interface used to apply world and attached collision objects.
  // visual_tools_ - RViz visual tools instance.
  // current_state - Cached current robot state used during planning.
  // stop_requested_ - Atomic stop flag checked by motion routines.
  // psm_ - PlanningSceneMonitor used for collision-aware IK and scene checks.
  // =========
  // move_group_ MUST be declared before visual_tools_ — initialisation order matters.
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  moveit::core::RobotStatePtr current_state;

  std::atomic<bool> stop_requested_{false};
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};
