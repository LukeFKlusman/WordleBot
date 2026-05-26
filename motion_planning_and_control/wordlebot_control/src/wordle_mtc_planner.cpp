#include "wordlebot_control/wordle_mtc_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
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

struct TrajectoryScore {
  double joint_motion{0.0};
  double wrist_spin{0.0};
  double total{0.0};
  bool rejected{false};
};

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

std::optional<std::size_t> findJointIndex(
  const std::vector<std::string> & joint_names,
  const std::string & joint_name)
{
  const auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
  if (it == joint_names.end()) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(std::distance(joint_names.begin(), it));
}

double shortestRevoluteDelta(double from, double to)
{
  double delta = to - from;
  while (delta > M_PI) {
    delta -= kTwoPi;
  }
  while (delta < -M_PI) {
    delta += kTwoPi;
  }
  return delta;
}

TrajectoryScore scoreTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const moveit::core::JointModelGroup * jmg,
  double wrist_spin_weight,
  double wrist_spin_reject_threshold)
{
  TrajectoryScore score;
  if (jmg == nullptr || trajectory.getWayPointCount() < 2) {
    return score;
  }

  const auto & joint_names = jmg->getVariableNames();
  const auto wrist2_index = findJointIndex(joint_names, "wrist_2_joint");
  const auto wrist3_index = findJointIndex(joint_names, "wrist_3_joint");

  std::vector<double> prev;
  std::vector<double> curr;
  for (std::size_t i = 1; i < trajectory.getWayPointCount(); ++i) {
    trajectory.getWayPoint(i - 1).copyJointGroupPositions(jmg, prev);
    trajectory.getWayPoint(i).copyJointGroupPositions(jmg, curr);
    for (std::size_t j = 0; j < prev.size() && j < curr.size(); ++j) {
      const double delta = std::abs(shortestRevoluteDelta(prev[j], curr[j]));
      score.joint_motion += delta;
      if ((wrist2_index && j == *wrist2_index) || (wrist3_index && j == *wrist3_index)) {
        score.wrist_spin += delta;
      }
    }
  }

  score.total = score.joint_motion + wrist_spin_weight * score.wrist_spin;
  score.rejected =
    wrist_spin_reject_threshold > 0.0 && score.wrist_spin > wrist_spin_reject_threshold;
  return score;
}
}  // namespace

WordleMtcPlanner::WordleMtcPlanner(rclcpp::Node::SharedPtr node, std::string pipeline_name)
: node_(std::move(node)), pipeline_name_(std::move(pipeline_name))
{
  const uint candidate_plans = static_cast<uint>(
    std::max(1, getIntParam(node_, "wordle_mtc_planner.candidate_plans", 20)));
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
  trajectory_wrist_spin_weight_ =
    getDoubleParam(node_, "wordle_mtc_planner.trajectory_wrist_spin_weight", 4.0);
  trajectory_wrist_spin_reject_threshold_ =
    getDoubleParam(node_, "wordle_mtc_planner.trajectory_wrist_spin_reject_threshold", 7.0);

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
  int rejected = 0;

  RCLCPP_INFO(LOGGER,
    "planAndSelect: planning %u OMPL candidate(s), allowed_time=%.3f s, wrist_spin_weight=%.2f, "
    "wrist_reject_threshold=%.3f rad.",
    candidate_count, req.allowed_planning_time, trajectory_wrist_spin_weight_,
    trajectory_wrist_spin_reject_threshold_);

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
    const auto score = scoreTrajectory(
      *response.trajectory_, jmg, trajectory_wrist_spin_weight_,
      trajectory_wrist_spin_reject_threshold_);
    if (score.rejected) {
      ++rejected;
      RCLCPP_DEBUG(LOGGER,
        "candidate %u/%u rejected: joint_motion=%.4f wrist_spin=%.4f score=%.4f.",
        attempt + 1, candidate_count, score.joint_motion, score.wrist_spin, score.total);
      continue;
    }

    const double cost = score.total;
    RCLCPP_DEBUG(LOGGER,
      "candidate %u/%u accepted: joint_motion=%.4f wrist_spin=%.4f score=%.4f%s",
      attempt + 1, candidate_count, score.joint_motion, score.wrist_spin,
      score.total, cost < best_cost ? " (best)" : "");
    if (cost < best_cost) {
      best_cost = cost;
      best_trajectory = response.trajectory_;
    }
  }

  if (best_trajectory == nullptr) {
    return {false, rejected > 0 ? "all successful candidates were rejected by wrist-spin scoring" :
      (last_error.empty() ? "all candidate plans failed" : last_error)};
  }

  result = best_trajectory;
  RCLCPP_INFO(LOGGER,
    "planAndSelect: selected best of %d/%u successful candidate(s), rejected=%d, score=%.4f.",
    successes, candidate_count, rejected, best_cost);
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
