#include "wordleBot_control/wordle_bot_controller.hpp"

#include <chrono>
#include <cmath>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace mtc = moveit::task_constructor;

#include <moveit/kinematics_base/kinematics_base.h>

#include <algorithm>
#include <limits>
#include <vector>

#include <angles/angles.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("WordleBotController");

WordleBotController::WordleBotController(rclcpp::Node::SharedPtr node)
: node_(node),
  move_group_(node, "ur_onrobot_manipulator"),
  planning_scene_(),
  visual_tools_(node, "ur_base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_.getRobotModel())
{
  visual_tools_.deleteAllMarkers();
  visual_tools_.loadRemoteControl();

  // Give OMPL more time and attempts to find a better path
  move_group_.setPlanningTime(10.0);
  move_group_.setNumPlanningAttempts(5);

  // DEBUG: log all link names in the robot model so we can verify touch_links names
  RCLCPP_INFO(LOGGER, "Robot model link names:");
  for (const auto & link : move_group_.getRobotModel()->getLinkModelNames()) {
    RCLCPP_INFO(LOGGER, "  link: %s", link.c_str());
  }
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_.getPlanningFrame().c_str());
}

WordleBotController::~WordleBotController()
{
}

namespace 
{
constexpr char kPlanningGroup[] = "ur_onrobot_manipulator";

double jointDistance(const moveit::core::JointModel * joint_model, double from, double to)
{
  if (joint_model->getType() == moveit::core::JointModel::REVOLUTE) {
    return std::abs(angles::shortest_angular_distance(from, to));
  }

  return std::abs(to - from);
}

double configurationDistance(const moveit::core::JointModelGroup * joint_model_group,
                             const std::vector<double> & from,
                             const std::vector<double> & to)
{
  const auto & joint_models = joint_model_group->getActiveJointModels();
  double total_distance = 0.0;

  for (std::size_t i = 0; i < joint_models.size() && i < from.size() && i < to.size(); ++i) {
    total_distance += jointDistance(joint_models[i], from[i], to[i]);
  }

  return total_distance;
}

int jointNameIndex(const std::vector<std::string> & names, const std::string & joint_name)
{
  const auto it = std::find(names.begin(), names.end(), joint_name);
  if (it == names.end()) {
    return -1;
  }

  return static_cast<int>(std::distance(names.begin(), it));
}
}  // namespace

std::vector<double> WordleBotController::computeBestIK(const moveit::core::RobotStatePtr & current_state,
                                                       const geometry_msgs::msg::Pose & target_pose)
{
  const auto * joint_model_group = move_group_.getRobotModel()->getJointModelGroup(kPlanningGroup);
  if (joint_model_group == nullptr) {
    RCLCPP_ERROR(LOGGER, "computeBestIK: joint model group '%s' not found in robot model.", kPlanningGroup);
    return {};
  }
  if (current_state == nullptr) {
    RCLCPP_ERROR(LOGGER, "computeBestIK: current robot state is null.");
    return {};
  }

  // Verify kinematics solver is available before attempting IK
  const kinematics::KinematicsBaseConstPtr & solver = joint_model_group->getSolverInstance();
  if (!solver) {
    RCLCPP_ERROR(LOGGER,
      "computeBestIK: No kinematics solver loaded for group '%s'. "
      "Ensure kinematics.yaml is passed as a node parameter and the plugin is installed.",
      kPlanningGroup);
    return {};
  }
  RCLCPP_DEBUG(LOGGER, "computeBestIK: Using kinematics solver '%s'.", solver->getGroupName().c_str());

  std::vector<double> current_joint_values;
  current_state->copyJointGroupPositions(joint_model_group, current_joint_values);

  // Find shoulder_lift_joint index so we can normalise its angle to satisfy the path constraint.
  // The path constraint applied during planning is centred at -π/2 with ±110° tolerance.
  const double shoulder_center = -M_PI / 2.0;
  const double shoulder_tol    = M_PI / 180.0 * 110.0;
  const auto & joint_names = joint_model_group->getVariableNames();
  const int shoulder_idx = jointNameIndex(joint_names, "shoulder_lift_joint");

  std::vector<double> best_joint_values;
  double best_cost = std::numeric_limits<double>::infinity();
  int ik_successes = 0;
  int constraint_rejects = 0;

  // Functional position [0°, -135°, -90°, -45°, 0°, 0°] and per-joint weights.
  // The quadratic penalty steers IK selection toward a comfortable canonical posture.
  static const double q_func[6] = {0.0, -2.3562, -1.5708, -0.7854, 0.0, 0.0};
  static const double w_func[6] = {0.3,  0.5,     0.5,     0.5,    0.3,  0.3};

  for (int attempt = 0; attempt < 15; ++attempt) {
    moveit::core::RobotState ik_state(*current_state);
    ik_state.setToRandomPositions(joint_model_group);

    if (!ik_state.setFromIK(joint_model_group, target_pose, 0.1)) {
      // RCLCPP_INFO(LOGGER, "computeBestIK: attempt %d failed IK.", attempt);
      continue;
    }

    ++ik_successes;
    std::vector<double> candidate_joint_values;
    ik_state.copyJointGroupPositions(joint_model_group, candidate_joint_values);

    // Normalise every revolute joint to the 2π-equivalent numerically closest to the current
    // joint state. Without this, the planner receives a raw IK angle like 4.24 rad when the
    // robot is at -6.12 rad — they are equivalent mod 2π but 10.36 rad apart numerically,
    // causing the planner to generate a huge spinning trajectory.
    const auto & active_joints = joint_model_group->getActiveJointModels();
    for (std::size_t i = 0; i < candidate_joint_values.size() && i < active_joints.size(); ++i) {
      if (active_joints[i]->getType() != moveit::core::JointModel::REVOLUTE) {
        continue;
      }
      const double curr    = current_joint_values[i];
      const double raw     = candidate_joint_values[i];
      double best_norm     = raw;
      double best_dist     = std::abs(raw - curr);
      for (int k = -3; k <= 3; ++k) {
        const double offset = raw + k * 2.0 * M_PI;
        const double dist   = std::abs(offset - curr);
        if (dist < best_dist && active_joints[i]->satisfiesPositionBounds(&offset)) {
          best_dist = dist;
          best_norm = offset;
        }
      }
      if (std::abs(best_norm - raw) > 1e-6) {
        // RCLCPP_INFO(LOGGER,
        //   "computeBestIK: attempt %d normalised joint '%s': raw=%.4f -> norm=%.4f rad.",
        //   attempt, active_joints[i]->getName().c_str(), raw, best_norm);
      }
      candidate_joint_values[i] = best_norm;
    }

    // Clamp wrist_3_joint to [-π, π].
    // The UR RTDE hardware interface represents this continuous joint in [-π, π]. Allowing
    // values outside that range causes a 6.283 rad (2π) PATH_TOLERANCE_VIOLATED abort on
    // the scaled_joint_trajectory_controller when the controller's tracked state disagrees
    // with the planned trajectory start by exactly one full revolution.
    const int wrist3_idx = jointNameIndex(
      std::vector<std::string>(joint_names.begin(), joint_names.end()), "wrist_3_joint");
    if (wrist3_idx >= 0 && wrist3_idx < static_cast<int>(candidate_joint_values.size())) {
      double & w3 = candidate_joint_values[wrist3_idx];
      const double raw_w3 = w3;
      while (w3 > M_PI)  w3 -= 2.0 * M_PI;
      while (w3 < -M_PI) w3 += 2.0 * M_PI;
      if (std::abs(w3 - raw_w3) > 1e-6) {
        // RCLCPP_INFO(LOGGER,
        //   "computeBestIK: attempt %d clamped wrist_3 to [-pi,pi]: %.4f -> %.4f rad.",
        //   attempt, raw_w3, w3);
      }
    }

    // Reject if shoulder_lift_joint is outside the path constraint range after normalisation.
    if (shoulder_idx >= 0 && shoulder_idx < static_cast<int>(candidate_joint_values.size())) {
      const double sh = candidate_joint_values[shoulder_idx];
      if (std::abs(sh - shoulder_center) > shoulder_tol) {
        // RCLCPP_INFO(LOGGER,
        //   "computeBestIK: attempt %d rejected — shoulder %.4f rad outside constraint [%.4f, %.4f].",
        //   attempt, sh, shoulder_center - shoulder_tol, shoulder_center + shoulder_tol);
        ++constraint_rejects;
        continue;
      }
    }

    // Cost = actual joint movement (numeric distance after normalisation, not wrapped shortest
    // angle) + quadratic penalty for deviation from functional position.
    // Movement term (2×) dominates; functional penalty is a tiebreaker for canonical posture.
    double movement_cost = 0.0;
    double functional_penalty = 0.0;
    for (std::size_t i = 0; i < candidate_joint_values.size(); ++i) {
      movement_cost += std::abs(candidate_joint_values[i] - current_joint_values[i]);
      if (i < 6) {
        const double d = candidate_joint_values[i] - q_func[i];
        functional_penalty += w_func[i] * d * d;
      }
    }
    const double candidate_cost = (2.0 * movement_cost) + (0.3 * functional_penalty);

    // RCLCPP_INFO(LOGGER, "computeBestIK: attempt %d succeeded, cost=%.4f (best=%.4f).",
    //   attempt, candidate_cost, best_cost);

    if (candidate_cost < best_cost) {
      best_cost = candidate_cost;
      best_joint_values = candidate_joint_values;
    }
  }

  RCLCPP_INFO(LOGGER,
    "computeBestIK: %d/15 IK solutions found, %d rejected for shoulder constraint. Best cost: %.4f",
    ik_successes, constraint_rejects, best_cost);

  if (best_joint_values.empty()) {
    RCLCPP_ERROR(LOGGER,
      "computeBestIK: No valid IK solution found for target pose "
      "(x=%.3f y=%.3f z=%.3f). Target may be out of reach or in collision.",
      target_pose.position.x, target_pose.position.y, target_pose.position.z);
  }

  return best_joint_values;
}

std::vector<moveit::planning_interface::MoveGroupInterface::Plan>WordleBotController::generateCandidatePlans(int num_attempts)
{
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
  plans.reserve(static_cast<std::size_t>(std::max(num_attempts, 0)));

  RCLCPP_INFO(LOGGER, "generateCandidatePlans: generating %d plan attempts.", num_attempts);
  int successes = 0;
  for (int attempt = 0; attempt < num_attempts; ++attempt) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto result = move_group_.plan(plan);
    if (static_cast<bool>(result)) {
      ++successes;
      RCLCPP_DEBUG(LOGGER, "generateCandidatePlans: attempt %d succeeded (%zu waypoints).",
        attempt, plan.trajectory_.joint_trajectory.points.size());
      plans.push_back(plan);
    } else {
      RCLCPP_WARN(LOGGER, "generateCandidatePlans: attempt %d failed (error code %d).",
        attempt, result.val);
    }
  }
  RCLCPP_INFO(LOGGER, "generateCandidatePlans: %d/%d plans succeeded.", successes, num_attempts);

  return plans;
}

moveit::planning_interface::MoveGroupInterface::Plan WordleBotController::selectBestPlan(const std::vector<moveit::planning_interface::MoveGroupInterface::Plan> & plans,
                                                                                         const std::vector<double> & q_start,
                                                                                         const std::vector<double> & q_goal)
{
  moveit::planning_interface::MoveGroupInterface::Plan best_plan;
  double best_cost = std::numeric_limits<double>::infinity();

  const auto * joint_model_group = move_group_.getRobotModel()->getJointModelGroup(kPlanningGroup);
  if (joint_model_group == nullptr) {
    return best_plan;
  }

  const auto & variable_names = joint_model_group->getVariableNames();
  const auto & joint_models = joint_model_group->getActiveJointModels();
  const double direct_distance = configurationDistance(joint_model_group, q_start, q_goal);
  const int base_joint_index = jointNameIndex(variable_names, "shoulder_pan_joint");
  const int wrist_joint_index = jointNameIndex(variable_names, "wrist_3_joint");

  // Functional position penalty at the goal state: sum of |q_goal[i] - q_func[i]| per joint.
  // This steers plan selection toward paths that land in a comfortable canonical posture.
  static const double q_func[6] = {0.0, -2.3562, -1.5708, -0.7854, 0.0, 0.0};
  double functional_goal_penalty = 0.0;
  for (std::size_t i = 0; i < q_goal.size() && i < 6; ++i) {
    functional_goal_penalty += std::abs(q_goal[i] - q_func[i]);
  }

  std::size_t plan_index = 0;
  for (const auto & plan : plans) {
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    const auto & points = trajectory.points;
    if (points.empty()) {
      ++plan_index;
      continue;
    }

    std::vector<int> trajectory_indices;
    trajectory_indices.reserve(variable_names.size());
    bool missing_joint = false;

    for (const auto & variable_name : variable_names) {
      const int index = jointNameIndex(trajectory.joint_names, variable_name);
      if (index < 0) {
        missing_joint = true;
        break;
      }
      trajectory_indices.push_back(index);
    }

    if (missing_joint) {
      ++plan_index;
      continue;
    }

    double total_path_length = 0.0;
    double base_rotation = 0.0;
    double wrist_rotation = 0.0;

    for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
      if (point_index == 0) {
        continue;
      }

      const auto & previous_point = points[point_index - 1];
      const auto & current_point  = points[point_index];
      double segment_length = 0.0;

      for (std::size_t joint_index = 0; joint_index < joint_models.size(); ++joint_index) {
        const double prev_pos = previous_point.positions[trajectory_indices[joint_index]];
        const double curr_pos = current_point.positions[trajectory_indices[joint_index]];
        const double delta = jointDistance(joint_models[joint_index], prev_pos, curr_pos);
        segment_length += delta;

        if (static_cast<int>(joint_index) == base_joint_index) {
          base_rotation += delta;
        }
        if (static_cast<int>(joint_index) == wrist_joint_index) {
          wrist_rotation += delta;
        }
      }

      total_path_length += segment_length;
    }

    // Soft cost: path length (primary) + base rotation (secondary) + wrist rotation (tertiary)
    // + functional goal penalty (quaternary). No hard rejections — the lowest-cost plan always
    // wins regardless of ratio or rotation thresholds.
    const double plan_cost =
      (2.0 * total_path_length) +
      (1.5 * base_rotation) +
      (0.5 * wrist_rotation) +
      (0.25 * functional_goal_penalty);

    RCLCPP_INFO(LOGGER,
      "selectBestPlan: plan %zu — path=%.3f direct=%.3f ratio=%.2fx base_rot=%.3f rad cost=%.3f -> %s",
      plan_index,
      total_path_length, direct_distance,
      direct_distance > 1e-6 ? total_path_length / direct_distance : 0.0,
      base_rotation, plan_cost,
      plan_cost < best_cost ? "best so far" : "worse");

    if (plan_cost < best_cost) {
      best_cost = plan_cost;
      best_plan = plan;
    }
    ++plan_index;
  }

  if (best_cost > 15.0) {
    RCLCPP_WARN(LOGGER,
      "selectBestPlan: best plan cost=%.3f is high — motion may not be fully optimal.", best_cost);
  }

  return best_plan;
}

moveit_msgs::msg::Constraints WordleBotController::buildPathConstraints()
{
  moveit_msgs::msg::JointConstraint shoulder;
  shoulder.joint_name      = "shoulder_lift_joint";
  shoulder.position        = -M_PI / 2.0;
  shoulder.tolerance_above = M_PI / 180.0 * 110.0;
  shoulder.tolerance_below = M_PI / 180.0 * 110.0;
  shoulder.weight          = 1.0;

  // Keep wrist_3_joint within [-π, π] throughout the planned path.
  // The UR RTDE interface reports wrist_3 in this range; going outside causes
  // a PATH_TOLERANCE_VIOLATED (2π position error) on the trajectory controller.
  moveit_msgs::msg::JointConstraint wrist3;
  wrist3.joint_name      = "wrist_3_joint";
  wrist3.position        = 0.0;
  wrist3.tolerance_above = M_PI;
  wrist3.tolerance_below = M_PI;
  wrist3.weight          = 1.0;

  moveit_msgs::msg::Constraints constraints;
  constraints.joint_constraints.push_back(shoulder);
  constraints.joint_constraints.push_back(wrist3);
  return constraints;
}


bool WordleBotController::moveToTarget(const geometry_msgs::msg::Pose & target)
{
  RCLCPP_INFO(LOGGER, "Target pose:\n  pos  x=%.3f y=%.3f z=%.3f\n  quat x=%.3f y=%.3f z=%.3f w=%.3f",
    target.position.x, target.position.y, target.position.z,
    target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);

  move_group_.setStartStateToCurrentState();

  move_group_.setPathConstraints(buildPathConstraints());

  visualisePlan(nullptr, "Planning");

  move_group_.setStartStateToCurrentState();
  current_state = move_group_.getCurrentState(2.0);  // 2s timeout for fresh state
  const auto * joint_model_group = move_group_.getRobotModel()->getJointModelGroup(kPlanningGroup);

  std::vector<double> q_start;
  if (current_state != nullptr && joint_model_group != nullptr) {
    current_state->copyJointGroupPositions(joint_model_group, q_start);

    // DEBUG: log current joint angles so we can verify state monitor is reading correctly
    const auto & jnames = joint_model_group->getVariableNames();
    RCLCPP_INFO(LOGGER, "Current joint state (%zu joints):", q_start.size());
    for (std::size_t i = 0; i < q_start.size() && i < jnames.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
        jnames[i].c_str(), q_start[i], q_start[i] * 180.0 / M_PI);
    }

    // DEBUG: forward kinematics — does the computed EEF position match where the robot actually is?
    const Eigen::Isometry3d & eef_tf = current_state->getGlobalLinkTransform(
      move_group_.getEndEffectorLink());
    RCLCPP_INFO(LOGGER, "Current EEF pose (FK) [%s]: x=%.4f y=%.4f z=%.4f",
      move_group_.getEndEffectorLink().c_str(),
      eef_tf.translation().x(), eef_tf.translation().y(), eef_tf.translation().z());
  } else {
    RCLCPP_ERROR(LOGGER, "getCurrentState returned null — state monitor has no data yet!");
  }

  const std::vector<double> best_q = computeBestIK(current_state, target);
  if (best_q.empty()) {
    move_group_.clearPathConstraints();
    visualisePlan(nullptr, "Planning Failed!");
    RCLCPP_ERROR(LOGGER, "Planning failed!");
    return false;
  }

  // --- Diagnostic: log the chosen joint target values ---
  if (joint_model_group != nullptr) {
    const auto & joint_names = joint_model_group->getVariableNames();
    RCLCPP_INFO(LOGGER, "Target joint values (%zu joints):", best_q.size());
    for (std::size_t i = 0; i < best_q.size() && i < joint_names.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
        joint_names[i].c_str(), best_q[i], best_q[i] * 180.0 / M_PI);
    }

    // --- Diagnostic: check shoulder constraint satisfaction at goal ---
    const int shoulder_idx = jointNameIndex(
      std::vector<std::string>(joint_names.begin(), joint_names.end()),
      "shoulder_lift_joint");
    if (shoulder_idx >= 0 && shoulder_idx < static_cast<int>(best_q.size())) {
      const double shoulder_val = best_q[shoulder_idx];
      const double shoulder_center = -M_PI / 2.0;
      const double shoulder_tol = M_PI / 180.0 * 110.0;
      const bool within = std::abs(shoulder_val - shoulder_center) <= shoulder_tol;
      RCLCPP_INFO(LOGGER,
        "Shoulder constraint check: shoulder_lift_joint=%.4f rad (%.1f deg), "
        "allowed range [%.4f, %.4f] -> %s",
        shoulder_val, shoulder_val * 180.0 / M_PI,
        shoulder_center - shoulder_tol, shoulder_center + shoulder_tol,
        within ? "WITHIN constraint" : "VIOLATES constraint");
    }

    // --- Diagnostic: check if goal joint values violate joint bounds ---
    if (current_state != nullptr) {
      moveit::core::RobotState goal_state(*current_state);
      goal_state.setJointGroupPositions(joint_model_group, best_q);
      goal_state.update();

      bool bounds_ok = true;
      const auto & active_joints = joint_model_group->getActiveJointModels();
      for (std::size_t i = 0; i < active_joints.size() && i < best_q.size(); ++i) {
        if (!active_joints[i]->satisfiesPositionBounds(&best_q[i])) {
          RCLCPP_ERROR(LOGGER,
            "Goal joint '%s' value %.4f rad VIOLATES joint limits!",
            active_joints[i]->getName().c_str(), best_q[i]);
          bounds_ok = false;
        }
      }
      if (bounds_ok) {
        RCLCPP_INFO(LOGGER, "Goal state joint bounds check: ALL within limits");
      }
    }
  }

  // --- Diagnostic: log the path constraint being applied ---
  RCLCPP_INFO(LOGGER,
    "Path constraint: shoulder_lift_joint center=%.4f rad, tolerance=+/-%.4f rad (%.1f deg)",
    -M_PI / 2.0, M_PI / 180.0 * 110.0, 110.0);

  move_group_.clearPoseTargets();
  move_group_.setJointValueTarget(best_q);

  const auto plans = generateCandidatePlans(5);
  move_group_.clearPathConstraints();

  if (plans.empty()) {
    visualisePlan(nullptr, "Planning Failed!");
    RCLCPP_ERROR(LOGGER, "Planning failed!");
    return false;
  }

  auto plan = selectBestPlan(plans, q_start, best_q);
  const bool success = !plan.trajectory_.joint_trajectory.points.empty();

  if (success) {
    const double total_disp = computeTotalJointDisplacement(plan);
    const double direct_disp = configurationDistance(
      move_group_.getRobotModel()->getJointModelGroup(kPlanningGroup), q_start, best_q);
    RCLCPP_INFO(LOGGER,
      "Selected plan: total_joint_disp=%.4f rad  direct_joint_disp=%.4f rad  ratio=%.3fx",
      total_disp, direct_disp, direct_disp > 1e-6 ? total_disp / direct_disp : 0.0);

    visualisePlan(&plan, "Executing");
    if (stop_requested_.load()) {
      RCLCPP_INFO(LOGGER, "moveToTarget: stop requested before execute — aborting.");
      return false;
    }
    const auto exec_result = move_group_.execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Motion executed successfully.");
    } else {
      RCLCPP_ERROR(LOGGER, "Execution FAILED with error code: %d", exec_result.val);
      return false;
    }
  }
  else
  {
    visualisePlan(nullptr, "Planning Failed!");
    RCLCPP_ERROR(LOGGER, "Planning failed!");
  }

  return success;
}


bool WordleBotController::moveToTargetCartesian(const geometry_msgs::msg::Pose & target,
                                                double lift_height)
{
  RCLCPP_INFO(LOGGER, "Cartesian move: lift %.2fm then translate to target.", lift_height);

  move_group_.setStartStateToCurrentState();

  std::vector<geometry_msgs::msg::Pose> waypoints;

  geometry_msgs::msg::Pose lifted = move_group_.getCurrentPose().pose;
  lifted.position.z += lift_height;
  waypoints.push_back(lifted);
  waypoints.push_back(target);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = move_group_.computeCartesianPath(
    waypoints,
    0.01,   // eef_step: 1 cm resolution
    0.0,    // jump_threshold: 0 = disabled
    trajectory);

  RCLCPP_INFO(LOGGER, "Cartesian path coverage: %.0f%%", fraction * 100.0);

  if (fraction < 0.9) {
    RCLCPP_ERROR(LOGGER, "Cartesian path only %.0f%% complete — aborting.", fraction * 100.0);
    visualisePlan(nullptr, "Cartesian Planning Failed!");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
  cart_plan.trajectory_ = trajectory;

  visualisePlan(&cart_plan, "Executing Cartesian");
  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "moveToTargetCartesian: stop requested before execute — aborting.");
    return false;
  }
  move_group_.execute(cart_plan);
  RCLCPP_INFO(LOGGER, "Cartesian motion executed successfully.");
  return true;
}


void WordleBotController::setupCollisionScene()
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_.getPlanningFrame();
  collision_object.id = "box1";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 0.05;
  primitive.dimensions[primitive.BOX_Z] = 0.25;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.35;
  box_pose.position.y = 0.1;
  box_pose.position.z = 0.1;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // planning_scene_.applyCollisionObject(collision_object);

  moveit_msgs::msg::CollisionObject floor;
  floor.header.frame_id = move_group_.getPlanningFrame();
  floor.id = "floor";

  shape_msgs::msg::SolidPrimitive floor_shape;
  floor_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  floor_shape.dimensions = {2.0, 2.0, 0.01};  

  geometry_msgs::msg::Pose floor_pose;
  floor_pose.position.x = 0.0;
  floor_pose.position.y = 0.0;
  floor_pose.position.z = -0.015;
  floor_pose.orientation.w = 1.0;

  floor.primitives.push_back(floor_shape);
  floor.primitive_poses.push_back(floor_pose);
  floor.operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_.applyCollisionObject(floor);

  attachSensorCollisionObject();
  RCLCPP_INFO(LOGGER, "Collision scene set up: floor added, sensor guard attached.");
}


void WordleBotController::clearCollisionScene()
{
  detachSensorCollisionObject();
  planning_scene_.removeCollisionObjects({"box1", "floor"});
  RCLCPP_INFO(LOGGER, "Collision scene cleared.");
}


void WordleBotController::addCollisionObject(const moveit_msgs::msg::CollisionObject & obj)
{
  planning_scene_.applyCollisionObject(obj);
  rclcpp::sleep_for(std::chrono::milliseconds(300));
  RCLCPP_INFO(LOGGER, "addCollisionObject: applied object '%s' (operation=%d).",
    obj.id.c_str(), static_cast<int>(obj.operation));
}


void WordleBotController::attachSensorCollisionObject()
{
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "tool0";
  attached_object.object.id = "sensor_guard";
  attached_object.object.header.frame_id = "tool0";
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive cylinder;
  cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder.dimensions = {0.03, 0.06};  // [height, radius] in metres

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = -0.015;  // slight offset to avoid collision with the end-effector link itself
  pose.orientation.w = 1.0;

  attached_object.object.primitives.push_back(cylinder);
  attached_object.object.primitive_poses.push_back(pose);

  // Allow all links that physically overlap with the sensor guard cylinder.
  // Includes the full gripper chain beyond tool0 (onrobot RG2).
  attached_object.touch_links = {
    "wrist_3_link", "flange", "tool0", "ft_frame",
    "onrobot_base_link",
    "cable_connector_0", "cable_connector_1",
    "left_outer_knuckle", "left_inner_finger", "left_finger_tip",
    "finger_width_mock_link",
    "left_inner_knuckle", "right_inner_knuckle",
    "right_outer_knuckle", "right_inner_finger", "right_finger_tip",
    "gripper_tcp"
  };

  planning_scene_.applyAttachedCollisionObject(attached_object);

  // Give the move_group server time to propagate the scene update before planning begins.
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "Sensor guard attached to tool0");
}


void WordleBotController::detachSensorCollisionObject()
{
  moveit_msgs::msg::AttachedCollisionObject detach;
  detach.link_name = "tool0";
  detach.object.id = "sensor_guard";
  detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  planning_scene_.applyAttachedCollisionObject(detach);
  RCLCPP_INFO(LOGGER, "Sensor guard collision cylinder detached from tool0.");
}


geometry_msgs::msg::Pose WordleBotController::buildPose(
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  quat.normalize();

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();

  return pose;
}


double WordleBotController::computeTotalJointDisplacement(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  const auto & points = plan.trajectory_.joint_trajectory.points;
  double total = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto & prev = points[i - 1].positions;
    const auto & curr = points[i].positions;
    for (std::size_t j = 0; j < prev.size() && j < curr.size(); ++j) {
      total += std::abs(curr[j] - prev[j]);
    }
  }
  return total;
}


void WordleBotController::visualisePlan(const moveit::planning_interface::MoveGroupInterface::Plan * plan,
                                        const std::string & title)
{
  auto text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools_.publishText(text_pose, title, rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);

  if (plan != nullptr) {
    const auto * joint_model_group =
      move_group_.getRobotModel()->getJointModelGroup("ur_onrobot_manipulator");
    visual_tools_.publishTrajectoryLine(plan->trajectory_, joint_model_group);
  }

  visual_tools_.trigger();

  // Waits until Visualistion is complete before returning, ensuring the user sees the updated plan before the next step.
  rclcpp::sleep_for(std::chrono::milliseconds(500));
}

// ---------------------------------------------------------------------------
// Stop / home helpers
// ---------------------------------------------------------------------------

void WordleBotController::stop()
{
  stop_requested_.store(true);
  move_group_.stop();
  RCLCPP_INFO(LOGGER, "stop(): trajectory cancelled.");
}

void WordleBotController::clearStopFlag()
{
  stop_requested_.store(false);
}

bool WordleBotController::moveToHome()
{
  RCLCPP_INFO(LOGGER, "moveToHome: moving to named state 'home'.");
  move_group_.setStartStateToCurrentState();
  move_group_.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto result = move_group_.plan(plan);
  if (!static_cast<bool>(result)) {
    RCLCPP_ERROR(LOGGER, "moveToHome: planning failed (error %d).", result.val);
    return false;
  }

  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "moveToHome: stop requested before execute — aborting.");
    return false;
  }

  const auto exec_result = move_group_.execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "moveToHome: execution failed (error %d).", exec_result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "moveToHome: reached home.");
  return true;
}

// ---------------------------------------------------------------------------
// Pick-and-place: MTC task creation, planning, execution
// ---------------------------------------------------------------------------

mtc::Task WordleBotController::createTask(const geometry_msgs::msg::Pose & object_pose)
{
  RCLCPP_DEBUG(LOGGER, "createTask: initialising MTC task.");
  mtc::Task task;
  task.stages()->setName("pick and place letter");
  task.loadRobotModel(node_);

  const std::string arm_group  = "ur_onrobot_manipulator";
  const std::string hand_group = "ur_onrobot_gripper";
  const std::string hand_frame = "gripper_tcp";

  task.setProperty("group",    arm_group);
  task.setProperty("eef",      hand_group);
  task.setProperty("ik_frame", hand_frame);
  RCLCPP_DEBUG(LOGGER, "createTask: arm='%s', hand_group='%s', ik_frame='%s'.",
    arm_group.c_str(), hand_group.c_str(), hand_frame.c_str());

  // Explicitly name the OMPL pipeline so MTC never falls back to CHOMP.
  // ompl_planning.yaml must be loaded into the node's parameters (see wordle_bot.launch.py).
  auto sampling_planner      = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.001);
  RCLCPP_DEBUG(LOGGER, "createTask: CartesianPath planner configured (vel=0.5, acc=0.5, step=0.001).");

  // Dedicated planner for retreat: min_fraction=0 accepts whatever Cartesian
  // distance the arm can actually achieve at the (low, far-reach) place pose.
  auto retreat_planner = std::make_shared<mtc::solvers::CartesianPath>();
  retreat_planner->setMaxVelocityScalingFactor(0.5);
  retreat_planner->setMaxAccelerationScalingFactor(0.5);
  retreat_planner->setStepSize(0.001);
  retreat_planner->setMinFraction(0.0);

  // ── Stage 1: capture current state ────────────────────────────────────────
  mtc::Stage * current_state_ptr = nullptr;
  {
    RCLCPP_DEBUG(LOGGER, "createTask: adding stage 1 — CurrentState.");
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  // ── Stage 2: open gripper ─────────────────────────────────────────────────
  {
    RCLCPP_DEBUG(LOGGER, "\ncreateTask: adding stage 2 — open hand (goal='open').");
    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));
  }

  // ── Stage 3: free-space move to pick region ───────────────────────────────
  {
    RCLCPP_INFO(LOGGER, "createTask: adding stage 3 — Connect 'move to pick' (timeout=10 s).");
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
    stage_move_to_pick->setTimeout(10.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    stage_move_to_pick->setPathConstraints(WordleBotController::buildPathConstraints());
    task.add(std::move(stage_move_to_pick));
  }

  // ── Stage 4: pick container ───────────────────────────────────────────────
  mtc::Stage * attach_object_stage = nullptr;
  {
    RCLCPP_DEBUG(LOGGER, "\ncreateTask: building stage 4 — SerialContainer 'pick object'.");
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // 4a. Cartesian approach along gripper_tcp z-axis
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4a — MoveRelative 'approach object' "
        "(link='%s', dist=[0.05, 0.10], dir=+z in %s).",
        hand_frame.c_str(), hand_frame.c_str());

      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // 4b. Sample grasp pose around the object + solve IK
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4b — GenerateGraspPose for object '%s' "
        "(angle_delta=π/12, IK solutions=8, z_offset=0.08 m).",
        LETTER_OBJECT_ID);
      auto stage =
        std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(LETTER_OBJECT_ID);
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);

      // Transform from gripper_tcp to the object centre when grasping top-down.
      // z=0.08 means gripper_tcp sits 80 mm above the object centre at grasp time.
      constexpr double GRASP_Z_OFFSET = 0.01;
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = GRASP_Z_OFFSET;

      const double expected_grasp_z   = object_pose.position.z + GRASP_Z_OFFSET;
      const double expected_approach_z_min = expected_grasp_z + 0.10;
      const double expected_approach_z_max = expected_grasp_z + 0.15;
      RCLCPP_INFO(LOGGER,
        "createTask [grasp geometry]: object_z=%.4f m  grasp_z_offset=%.3f m  "
        "=> expected gripper_tcp z AT GRASP = %.4f m  "
        "| pre-approach z range = [%.4f, %.4f] m (world frame, top-down).",
        object_pose.position.z, GRASP_Z_OFFSET,
        expected_grasp_z, expected_approach_z_min, expected_approach_z_max);

      auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    // 4c. Allow collisions between gripper links and the object
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4c — ModifyPlanningScene allow collision "
        "('%s', hand links).", LETTER_OBJECT_ID);
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (hand,object)");
      stage->allowCollisions( LETTER_OBJECT_ID,
                              task.getRobotModel()
                                ->getJointModelGroup(hand_group)
                                ->getLinkModelNamesWithCollisionGeometry(),
                              true);
      grasp->insert(std::move(stage));
    }

    // 4d. Close gripper
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4d — MoveTo 'close hand' (goal='close').");
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("closed");
      grasp->insert(std::move(stage));
    }

    // 4e. Attach object to the gripper link — GeneratePlacePose monitors this stage
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4e — ModifyPlanningScene attach '%s' to '%s'.",
        LETTER_OBJECT_ID, hand_frame.c_str());
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(LETTER_OBJECT_ID, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // 4f. Lift object vertically
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4f — MoveRelative 'lift object' "
        "(dist=[0.05, 0.15], dir=+z world).");
      auto stage =
        std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    RCLCPP_DEBUG(LOGGER, "createTask: stage 4 'pick object' container complete.");
    task.add(std::move(grasp));
  }

  // ── Stage 5: free-space move to place region ──────────────────────────────
  {
    RCLCPP_INFO(LOGGER, "createTask: adding stage 5 — Connect 'move to place' (timeout=10 s).");
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{
        {arm_group, sampling_planner}});
    stage->setTimeout(10.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->setPathConstraints(WordleBotController::buildPathConstraints());
    task.add(std::move(stage));
  }

  // ── Stage 6: place container ──────────────────────────────────────────────
  {
    RCLCPP_DEBUG(LOGGER, "createTask: building stage 6 — SerialContainer 'place object'.");
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // 6a. Generate place pose + solve IK (object must reach PLACE_X/Y/Z in world)
    {
      RCLCPP_DEBUG(LOGGER, "\ncreateTask: 6a — GeneratePlacePose for '%s' "
        "target=(%.3f, %.3f, %.3f) world, IK solutions=4.",
        LETTER_OBJECT_ID, PLACE_X, PLACE_Y, PLACE_Z);
      auto stage =
        std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(LETTER_OBJECT_ID);

      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = "world";
      target_pose.pose.position.x = PLACE_X;
      target_pose.pose.position.y = PLACE_Y;
      target_pose.pose.position.z = 0.05;
      target_pose.pose.orientation.w = 1.0;
      stage->setPose(target_pose);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(4);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(LETTER_OBJECT_ID);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    // 6b. Open gripper to release the object
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 6b — MoveTo 'open hand' (goal='open').");
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    // 6c. Restore collision checking between gripper and object
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 6c — ModifyPlanningScene forbid collision "
        "('%s', hand links).", LETTER_OBJECT_ID);
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "forbid collision (hand,object)");
      stage->allowCollisions(
        LETTER_OBJECT_ID,
        task.getRobotModel()
          ->getJointModelGroup(hand_group)
          ->getLinkModelNamesWithCollisionGeometry(),
        false);
      place->insert(std::move(stage));
    }

    // 6d. Detach object from gripper
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 6d — ModifyPlanningScene detach '%s' from '%s'.",
        LETTER_OBJECT_ID, hand_frame.c_str());
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(LETTER_OBJECT_ID, hand_frame);
      place->insert(std::move(stage));
    }

    // 6e. Retreat vertically away from the placed object
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 6e — MoveRelative 'retreat' "
        "(dist=[0.05, 0.15], dir=+z world).");
      auto stage =
        std::make_unique<mtc::stages::MoveRelative>("retreat", retreat_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.03, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    RCLCPP_DEBUG(LOGGER, "createTask: stage 6 'place object' container complete.");
    task.add(std::move(place));
  }

  // ── Stage 7: return to named home state ───────────────────────────────────
  {
    RCLCPP_DEBUG(LOGGER, "createTask: adding stage 7 — MoveTo 'return home' (goal='home').");
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  RCLCPP_DEBUG(LOGGER, "createTask: task construction complete (7 top-level stages).");
  return task;
}

bool WordleBotController::doPickAndPlace(const geometry_msgs::msg::Pose & object_pose)
{
  RCLCPP_INFO(LOGGER, "doPickAndPlace: building MTC task.");

  RCLCPP_INFO(LOGGER,
    "doPickAndPlace: object pose = (%.4f, %.4f, %.4f)  "
    "orient=(%.4f, %.4f, %.4f, %.4f). "
    "MTC planning scene frame: 'world'.",
    object_pose.position.x, object_pose.position.y, object_pose.position.z,
    object_pose.orientation.x, object_pose.orientation.y,
    object_pose.orientation.z, object_pose.orientation.w);

  mtc::Task task = createTask(object_pose);

  try {
    task.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "MTC task init failed: " << e);
    return false;
  }

  RCLCPP_INFO(LOGGER, "doPickAndPlace: planning...");
  moveit::core::MoveItErrorCode plan_result;
  try {
    plan_result = task.plan(5);
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "MTC task planning threw InitStageException: " << e);
    return false;
  }
  if (!plan_result || task.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "MTC task planning failed — no solutions found.");
    return false;
  }

  RCLCPP_INFO(LOGGER,
    "doPickAndPlace: planning succeeded — %zu solution(s) found. "
    "Executing best solution (cost=%.3f).",
    task.solutions().size(), task.solutions().front()->cost());

  RCLCPP_DEBUG(LOGGER, "doPickAndPlace: publishing planned solution for visualization.");
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  task.introspection().publishSolution(*task.solutions().front());

  RCLCPP_INFO(LOGGER, "doPickAndPlace: executing...");
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "MTC task execution failed (error code %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "doPickAndPlace: pick-and-place succeeded.");
  return true;
}

// ---------------------------------------------------------------------------
// Phase-split MTC tasks for stop/resume-aware pick-and-place
// ---------------------------------------------------------------------------

mtc::Task WordleBotController::createPickTask(const geometry_msgs::msg::Pose & object_pose)
{
  // object_pose is available for future logging or geometric pre-checks;
  // MTC reads the object from the planning scene by LETTER_OBJECT_ID.
  (void)object_pose;

  mtc::Task task;
  task.stages()->setName("pick letter");
  task.loadRobotModel(node_);

  const std::string arm_group  = "ur_onrobot_manipulator";
  const std::string hand_group = "ur_onrobot_gripper";
  const std::string hand_frame = "gripper_tcp";

  task.setProperty("group",    arm_group);
  task.setProperty("eef",      hand_group);
  task.setProperty("ik_frame", hand_frame);

  auto sampling_planner      = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.001);

  // Stage 1: current state
  mtc::Stage * current_state_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  // Stage 2: open gripper
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  // Stage 3: move to pick region
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
    stage->setTimeout(10.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->setPathConstraints(WordleBotController::buildPathConstraints());
    task.add(std::move(stage));
  }

  // Stage 4: pick container (approach → grasp IK → allow collision → close → attach → lift)
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // 4a. Cartesian approach
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // 4b. Generate grasp pose + IK
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(LETTER_OBJECT_ID);
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);

      constexpr double GRASP_Z_OFFSET = 0.01;
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = GRASP_Z_OFFSET;

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    // 4c. Allow collision between hand and object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (hand,object)");
      stage->allowCollisions(LETTER_OBJECT_ID,
        task.getRobotModel()->getJointModelGroup(hand_group)->getLinkModelNamesWithCollisionGeometry(),
        true);
      grasp->insert(std::move(stage));
    }

    // 4d. Close gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("closed");
      grasp->insert(std::move(stage));
    }

    // 4e. Attach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(LETTER_OBJECT_ID, hand_frame);
      grasp->insert(std::move(stage));
    }

    // 4f. Lift object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  return task;
}

mtc::Task WordleBotController::createPlaceTask()
{
  mtc::Task task;
  task.stages()->setName("place letter");
  task.loadRobotModel(node_);

  const std::string arm_group  = "ur_onrobot_manipulator";
  const std::string hand_group = "ur_onrobot_gripper";
  const std::string hand_frame = "gripper_tcp";

  task.setProperty("group",    arm_group);
  task.setProperty("eef",      hand_group);
  task.setProperty("ik_frame", hand_frame);

  auto sampling_planner      = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto retreat_planner = std::make_shared<mtc::solvers::CartesianPath>();
  retreat_planner->setMaxVelocityScalingFactor(0.5);
  retreat_planner->setMaxAccelerationScalingFactor(0.5);
  retreat_planner->setStepSize(0.001);
  retreat_planner->setMinFraction(0.0);

  // Stage 1: current state — captures post-pick robot+scene including attached object.
  // GeneratePlacePose monitors this stage to reason about the object's attached state.
  mtc::Stage * current_state_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  // Stage 2: move to place region
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
    stage->setTimeout(10.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->setPathConstraints(WordleBotController::buildPathConstraints());
    task.add(std::move(stage));
  }

  // Stage 3: place container (place IK → open → forbid collision → detach → retreat)
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // 3a. Generate place pose + IK
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(LETTER_OBJECT_ID);

      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = "world";
      target_pose.pose.position.x = PLACE_X;
      target_pose.pose.position.y = PLACE_Y;
      target_pose.pose.position.z = 0.05;
      target_pose.pose.orientation.w = 1.0;
      stage->setPose(target_pose);
      stage->setMonitoredStage(current_state_ptr);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(4);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(LETTER_OBJECT_ID);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    // 3b. Open gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    // 3c. Forbid collision between hand and object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "forbid collision (hand,object)");
      stage->allowCollisions(LETTER_OBJECT_ID,
        task.getRobotModel()->getJointModelGroup(hand_group)->getLinkModelNamesWithCollisionGeometry(),
        false);
      place->insert(std::move(stage));
    }

    // 3d. Detach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(LETTER_OBJECT_ID, hand_frame);
      place->insert(std::move(stage));
    }

    // 3e. Retreat
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", retreat_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.03, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  // Stage 4: return home
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  return task;
}

bool WordleBotController::doPickPhase(const geometry_msgs::msg::Pose & object_pose)
{
  RCLCPP_INFO(LOGGER, "doPickPhase: building pick MTC task.");
  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "doPickPhase: stop requested before start — aborting.");
    return false;
  }

  mtc::Task task = createPickTask(object_pose);

  try {
    task.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "doPickPhase: init failed: " << e);
    return false;
  }

  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "doPickPhase: stop requested after init — aborting.");
    return false;
  }

  RCLCPP_INFO(LOGGER, "doPickPhase: planning...");
  moveit::core::MoveItErrorCode plan_result;
  try {
    plan_result = task.plan(5);
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "doPickPhase: planning threw: " << e);
    return false;
  }

  if (!plan_result || task.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "doPickPhase: planning failed — no solutions.");
    return false;
  }

  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "doPickPhase: stop requested after planning — aborting.");
    return false;
  }

  RCLCPP_INFO(LOGGER, "doPickPhase: executing best solution (cost=%.3f).",
    task.solutions().front()->cost());
  task.introspection().publishSolution(*task.solutions().front());
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  const auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "doPickPhase: execution failed (error %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "doPickPhase: pick complete — object attached.");
  return true;
}

bool WordleBotController::doPlacePhase()
{
  RCLCPP_INFO(LOGGER, "doPlacePhase: building place MTC task.");
  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "doPlacePhase: stop requested before start — aborting.");
    return false;
  }

  mtc::Task task = createPlaceTask();

  try {
    task.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "doPlacePhase: init failed: " << e);
    return false;
  }

  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "doPlacePhase: stop requested after init — aborting.");
    return false;
  }

  RCLCPP_INFO(LOGGER, "doPlacePhase: planning...");
  moveit::core::MoveItErrorCode plan_result;
  try {
    plan_result = task.plan(5);
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "doPlacePhase: planning threw: " << e);
    return false;
  }

  if (!plan_result || task.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "doPlacePhase: planning failed — no solutions.");
    return false;
  }

  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "doPlacePhase: stop requested after planning — aborting.");
    return false;
  }

  RCLCPP_INFO(LOGGER, "doPlacePhase: executing best solution (cost=%.3f).",
    task.solutions().front()->cost());
  task.introspection().publishSolution(*task.solutions().front());
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  const auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "doPlacePhase: execution failed (error %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "doPlacePhase: place complete.");
  return true;
}
