#include "wordlebot_control/wordle_mtc_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/utils/moveit_error_code.h>

namespace mtc = moveit::task_constructor;

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
const rclcpp::Logger LOGGER = rclcpp::get_logger("WordleMtcPlanner");
const std::vector<double> kDefaultIkJoints = {1.1345, -1.5708, 1.5708, -1.5708, -1.5708, 1.1345};
const std::vector<double> kDefaultIkWeights = {0.3, 0.5, 0.5, 0.5, 0.3, 0.3};

int getIntParam(const rclcpp::Node::SharedPtr & node, const std::string & name, int default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<int>(name, default_value);
  }
  return static_cast<int>(node->get_parameter(name).as_int());
}

double getDoubleParam(const rclcpp::Node::SharedPtr & node, const std::string & name, double default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<double>(name, default_value);
  }
  return node->get_parameter(name).as_double();
}

std::vector<double> getDoubleArrayParam(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const std::vector<double> & default_value,
  std::size_t expected_size)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<std::vector<double>>(name, default_value);
  }

  auto values = node->get_parameter(name).as_double_array();
  if (values.size() != expected_size) {
    RCLCPP_WARN(LOGGER,
      "Parameter '%s' has %zu values, expected %zu. Using defaults.",
      name.c_str(), values.size(), expected_size);
    return default_value;
  }
  return values;
}

double computeTotalJointDisplacement(
  const robot_trajectory::RobotTrajectory & trajectory,
  const moveit::core::JointModelGroup * jmg)
{
  if (jmg == nullptr || trajectory.getWayPointCount() < 2) {
    return 0.0;
  }

  double total = 0.0;
  std::vector<double> prev;
  std::vector<double> curr;
  for (std::size_t i = 1; i < trajectory.getWayPointCount(); ++i) {
    trajectory.getWayPoint(i - 1).copyJointGroupPositions(jmg, prev);
    trajectory.getWayPoint(i).copyJointGroupPositions(jmg, curr);
    for (std::size_t j = 0; j < prev.size() && j < curr.size(); ++j) {
      total += std::abs(curr[j] - prev[j]);
    }
  }
  return total;
}
}  // namespace

WordleMtcPlanner::WordleMtcPlanner(rclcpp::Node::SharedPtr node, std::string pipeline_name)
: node_(std::move(node)), pipeline_name_(std::move(pipeline_name))
{
  const uint candidate_plans = static_cast<uint>(
    std::max(1, getIntParam(node_, "wordle_mtc_planner.candidate_plans", 5)));
  const uint num_planning_attempts = static_cast<uint>(
    std::max(1, getIntParam(node_, "wordle_mtc_planner.num_planning_attempts", 1)));
  const double goal_joint_tolerance =
    getDoubleParam(node_, "wordle_mtc_planner.goal_joint_tolerance", 1e-4);
  const double max_velocity_scaling =
    getDoubleParam(node_, "wordle_mtc_planner.max_velocity_scaling_factor", 1.0);
  const double max_acceleration_scaling =
    getDoubleParam(node_, "wordle_mtc_planner.max_acceleration_scaling_factor", 1.0);

  ik_warm_attempts_ = std::max(0, getIntParam(node_, "wordle_mtc_planner.ik_warm_attempts", 5));
  ik_total_attempts_ = std::max(1, getIntParam(node_, "wordle_mtc_planner.ik_total_attempts", 10));
  ik_retry_multiplier_ = std::max(1, getIntParam(node_, "wordle_mtc_planner.ik_retry_multiplier", 2));
  ik_warm_start_ = getDoubleArrayParam(
    node_, "wordle_mtc_planner.ik_warm_start_joints", kDefaultIkJoints, 6);
  ik_functional_reference_ = getDoubleArrayParam(
    node_, "wordle_mtc_planner.ik_functional_reference_joints", kDefaultIkJoints, 6);
  ik_functional_weights_ = getDoubleArrayParam(
    node_, "wordle_mtc_planner.ik_functional_weights", kDefaultIkWeights, 6);
  ik_wrist_1_min_ = getDoubleParam(node_, "wordle_mtc_planner.ik_wrist_1_min", -M_PI);
  ik_wrist_1_max_ = getDoubleParam(node_, "wordle_mtc_planner.ik_wrist_1_max", M_PI / 12.0);
  ik_movement_cost_weight_ =
    getDoubleParam(node_, "wordle_mtc_planner.ik_movement_cost_weight", 2.0);
  ik_functional_cost_weight_ =
    getDoubleParam(node_, "wordle_mtc_planner.ik_functional_cost_weight", 0.3);

  auto & p = properties();
  p.declare<std::string>("planner", "", "planner id");
  p.declare<uint>("num_planning_attempts", num_planning_attempts, "number of planning attempts per candidate");
  p.declare<uint>("candidate_plans", candidate_plans, "number of candidate trajectories to generate and score");
  p.declare<moveit_msgs::msg::WorkspaceParameters>(
    "workspace_parameters", moveit_msgs::msg::WorkspaceParameters(),
    "allowed workspace of mobile base?");
  p.declare<double>("goal_joint_tolerance", goal_joint_tolerance, "tolerance for reaching joint goals");
  p.declare<double>("goal_position_tolerance", 1e-4, "tolerance for reaching position goals");
  p.declare<double>("goal_orientation_tolerance", 1e-4, "tolerance for reaching orientation goals");
  setMaxVelocityScalingFactor(max_velocity_scaling);
  setMaxAccelerationScalingFactor(max_acceleration_scaling);
}

void WordleMtcPlanner::init(const moveit::core::RobotModelConstPtr & robot_model)
{
  if (pipeline_ == nullptr) {
    mtc::solvers::PipelinePlanner::Specification spec;
    spec.model = robot_model;
    spec.pipeline = pipeline_name_;
    spec.ns = pipeline_name_;
    pipeline_ = mtc::solvers::PipelinePlanner::create(node_, spec);
  } else if (pipeline_->getRobotModel() != robot_model) {
    throw std::runtime_error(
      "WordleMtcPlanner planning pipeline robot model does not match the task robot model");
  }
}

mtc::solvers::PlannerInterface::Result WordleMtcPlanner::plan(
  const planning_scene::PlanningSceneConstPtr & from,
  const planning_scene::PlanningSceneConstPtr & to,
  const moveit::core::JointModelGroup * jmg,
  double timeout,
  robot_trajectory::RobotTrajectoryPtr & result,
  const moveit_msgs::msg::Constraints & path_constraints)
{
  if (from == nullptr || to == nullptr || jmg == nullptr) {
    return {false, "WordleMtcPlanner received a null planning input"};
  }

  moveit_msgs::msg::MotionPlanRequest req;
  initMotionPlanRequest(req, jmg, timeout);
  req.goal_constraints.resize(1);
  req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
    to->getCurrentState(), jmg, properties().get<double>("goal_joint_tolerance"));
  req.path_constraints = path_constraints;

  return planAndSelect(from, req, jmg, result);
}

mtc::solvers::PlannerInterface::Result WordleMtcPlanner::plan(
  const planning_scene::PlanningSceneConstPtr & from,
  const moveit::core::LinkModel & link,
  const Eigen::Isometry3d & offset,
  const Eigen::Isometry3d & target,
  const moveit::core::JointModelGroup * jmg,
  double timeout,
  robot_trajectory::RobotTrajectoryPtr & result,
  const moveit_msgs::msg::Constraints & path_constraints)
{
  if (from == nullptr || jmg == nullptr) {
    return {false, "WordleMtcPlanner received a null pose-goal planning input"};
  }

  const auto best_q = computeBestIK(from, target * offset.inverse(), link.getName(), jmg);
  if (best_q.empty()) {
    return {false, "WordleMtcPlanner could not find a valid IK solution"};
  }

  auto target_scene = from->diff();
  auto & target_state = target_scene->getCurrentStateNonConst();
  target_state.setJointGroupPositions(jmg, best_q);
  target_state.update();

  moveit_msgs::msg::MotionPlanRequest req;
  initMotionPlanRequest(req, jmg, timeout);
  req.goal_constraints.resize(1);
  req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
    target_state, jmg, properties().get<double>("goal_joint_tolerance"));
  req.path_constraints = path_constraints;

  return planAndSelect(from, req, jmg, result);
}

std::string WordleMtcPlanner::getPlannerId() const
{
  return "WordleMtcPlanner";
}

void WordleMtcPlanner::initMotionPlanRequest(
  moveit_msgs::msg::MotionPlanRequest & req,
  const moveit::core::JointModelGroup * jmg,
  double timeout) const
{
  const auto & props = properties();
  req.group_name = jmg->getName();
  req.planner_id = props.get<std::string>("planner");
  req.allowed_planning_time = std::min(timeout, props.get<double>("timeout"));
  req.start_state.is_diff = true;
  req.num_planning_attempts = props.get<uint>("num_planning_attempts");
  req.max_velocity_scaling_factor = props.get<double>("max_velocity_scaling_factor");
  req.max_acceleration_scaling_factor = props.get<double>("max_acceleration_scaling_factor");
  req.workspace_parameters = props.get<moveit_msgs::msg::WorkspaceParameters>("workspace_parameters");
}

mtc::solvers::PlannerInterface::Result WordleMtcPlanner::planAndSelect(
  const planning_scene::PlanningSceneConstPtr & from,
  const moveit_msgs::msg::MotionPlanRequest & req,
  const moveit::core::JointModelGroup * jmg,
  robot_trajectory::RobotTrajectoryPtr & result)
{
  if (pipeline_ == nullptr) {
    return {false, "WordleMtcPlanner planning pipeline was not initialised"};
  }

  const uint candidate_count = std::max(1u, properties().get<uint>("candidate_plans"));
  double best_cost = std::numeric_limits<double>::infinity();
  std::string last_error;
  robot_trajectory::RobotTrajectoryPtr best_trajectory;
  int successes = 0;

  for (uint attempt = 0; attempt < candidate_count; ++attempt) {
    planning_interface::MotionPlanResponse response;
    const bool success = pipeline_->generatePlan(from, req, response);
    if (!success || response.trajectory_ == nullptr || response.trajectory_->getWayPointCount() == 0) {
      last_error = moveit::core::error_code_to_string(response.error_code_.val);
      RCLCPP_DEBUG(LOGGER,
        "candidate %u/%u failed: %s",
        attempt + 1, candidate_count, last_error.c_str());
      continue;
    }

    ++successes;
    const double cost = computeTotalJointDisplacement(*response.trajectory_, jmg);
    RCLCPP_DEBUG(LOGGER,
      "candidate %u/%u cost=%.4f rad%s",
      attempt + 1, candidate_count, cost, cost < best_cost ? " (best)" : "");
    if (cost < best_cost) {
      best_cost = cost;
      best_trajectory = response.trajectory_;
    }
  }

  if (best_trajectory == nullptr) {
    return {false, last_error.empty() ? "all candidate plans failed" : last_error};
  }

  result = best_trajectory;
  RCLCPP_DEBUG(LOGGER,
    "selected best of %d/%u successful candidate(s), cost=%.4f rad.",
    successes, candidate_count, best_cost);
  return {true, ""};
}

std::vector<double> WordleMtcPlanner::computeBestIK(
  const planning_scene::PlanningSceneConstPtr & scene,
  const Eigen::Isometry3d & target_pose,
  const std::string & tip_link,
  const moveit::core::JointModelGroup * jmg) const
{
  if (scene == nullptr || jmg == nullptr || !jmg->getSolverInstance()) {
    return {};
  }

  const auto & start_state = scene->getCurrentState();
  const auto & joint_names = jmg->getVariableNames();
  const auto & active_joints = jmg->getActiveJointModels();

  std::vector<double> current_joint_values;
  start_state.copyJointGroupPositions(jmg, current_joint_values);

  std::vector<double> best_joint_values;
  double best_cost = std::numeric_limits<double>::infinity();

  auto get_joint_value = [&](const std::vector<double> & values, const char * joint_name) {
    for (std::size_t i = 0; i < joint_names.size() && i < values.size(); ++i) {
      if (joint_names[i] == joint_name) {
        return values[i];
      }
    }
    return std::numeric_limits<double>::quiet_NaN();
  };

  auto is_collision_free = [scene](
    moveit::core::RobotState * state,
    const moveit::core::JointModelGroup * group,
    const double * ik_solution)
  {
    state->setJointGroupPositions(group, ik_solution);
    state->update();
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.group_name = group->getName();
    scene->checkCollision(req, res, *state);
    return !res.collision;
  };

  auto run_attempt = [&](int attempt) {
    moveit::core::RobotState ik_state(start_state);
    if (attempt < ik_warm_attempts_ && ik_warm_start_.size() == jmg->getVariableCount()) {
      ik_state.setJointGroupPositions(jmg, ik_warm_start_);
    } else {
      ik_state.setToRandomPositions(jmg);
    }
    ik_state.update();

    if (!ik_state.setFromIK(jmg, target_pose, tip_link, jmg->getDefaultIKTimeout(), is_collision_free)) {
      return;
    }

    std::vector<double> candidate;
    ik_state.copyJointGroupPositions(jmg, candidate);

    for (std::size_t i = 0; i < candidate.size() && i < active_joints.size(); ++i) {
      if (active_joints[i]->getType() != moveit::core::JointModel::REVOLUTE) {
        continue;
      }
      const double curr = current_joint_values[i];
      const double raw = candidate[i];
      double best_norm = raw;
      double best_dist = std::abs(raw - curr);
      for (int k = -3; k <= 3; ++k) {
        const double offset = raw + kTwoPi * k;
        const double dist = std::abs(offset - curr);
        if (dist < best_dist && active_joints[i]->satisfiesPositionBounds(&offset)) {
          best_dist = dist;
          best_norm = offset;
        }
      }
      candidate[i] = best_norm;
    }

    const double wrist1 = get_joint_value(candidate, "wrist_1_joint");
    if (std::isfinite(wrist1) && (wrist1 < ik_wrist_1_min_ || wrist1 > ik_wrist_1_max_)) {
      return;
    }

    double movement_cost = 0.0;
    double functional_penalty = 0.0;
    for (std::size_t i = 0; i < candidate.size(); ++i) {
      movement_cost += std::abs(candidate[i] - current_joint_values[i]);
      if (i < ik_functional_reference_.size() && i < ik_functional_weights_.size()) {
        const double d = candidate[i] - ik_functional_reference_[i];
        functional_penalty += ik_functional_weights_[i] * d * d;
      }
    }
    const double cost =
      ik_movement_cost_weight_ * movement_cost +
      ik_functional_cost_weight_ * functional_penalty;
    if (cost < best_cost) {
      best_cost = cost;
      best_joint_values = candidate;
    }
  };

  for (int attempt = 0; attempt < ik_total_attempts_; ++attempt) {
    run_attempt(attempt);
  }
  if (best_joint_values.empty()) {
    for (int attempt = ik_total_attempts_; attempt < ik_total_attempts_ * ik_retry_multiplier_; ++attempt) {
      run_attempt(attempt);
    }
  }

  return best_joint_values;
}
