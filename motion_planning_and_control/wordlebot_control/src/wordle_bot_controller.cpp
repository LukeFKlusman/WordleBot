#include "wordlebot_control/wordle_bot_controller.hpp"
#include "wordlebot_control/wordle_mtc_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>

#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/task_constructor/storage.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace mtc = moveit::task_constructor;

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("WordleBotController");

namespace
{
constexpr const char * kWrist3JointName = "wrist_3_joint";
constexpr double kTwoPi = 2.0 * M_PI;

double yawFromPose(const geometry_msgs::msg::Pose & pose);

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

double yawPenalty(
  double actual_yaw,
  double desired_yaw,
  double tolerance,
  double weight,
  double & yaw_error)
{
  yaw_error = std::abs(shortestRevoluteDelta(desired_yaw, actual_yaw));
  const double excess = std::max(0.0, yaw_error - std::max(0.0, tolerance));
  return std::max(0.0, weight) * excess * excess;
}

bool isPlaceYawSolution(const mtc::SolutionBase & solution)
{
  const auto * creator = solution.creator();
  if (creator == nullptr) {
    return false;
  }

  const std::string & name = creator->name();
  return name == "generate place pose" ||
         name == "place pose IK";
}

std::optional<double> targetYawFromInterfaceState(const mtc::InterfaceState * state)
{
  if (state == nullptr || !state->properties().hasProperty("target_pose")) {
    return std::nullopt;
  }

  try {
    const auto & target_pose =
      state->properties().get<geometry_msgs::msg::PoseStamped>("target_pose");
    return yawFromPose(target_pose.pose);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<double> placeYawFromSolution(const mtc::SolutionBase & solution)
{
  if (isPlaceYawSolution(solution)) {
    if (auto yaw = targetYawFromInterfaceState(solution.end())) {
      return yaw;
    }
    if (auto yaw = targetYawFromInterfaceState(solution.start())) {
      return yaw;
    }
  }

  if (const auto * sequence = dynamic_cast<const mtc::SolutionSequence *>(&solution)) {
    for (const auto * child : sequence->solutions()) {
      if (child != nullptr) {
        if (auto yaw = placeYawFromSolution(*child)) {
          return yaw;
        }
      }
    }
    return std::nullopt;
  }

  if (const auto * wrapped = dynamic_cast<const mtc::WrappedSolution *>(&solution)) {
    if (wrapped->wrapped() != nullptr) {
      return placeYawFromSolution(*wrapped->wrapped());
    }
  }

  return std::nullopt;
}

void accumulateTrajectoryMotionScore(
  const robot_trajectory::RobotTrajectory & trajectory,
  const moveit::core::JointModelGroup * jmg,
  SolutionMotionScore & score)
{
  if (jmg == nullptr || trajectory.getWayPointCount() < 2) {
    return;
  }

  ++score.trajectory_count;
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
}

void accumulateSolutionMotionScore(
  const mtc::SolutionBase & solution,
  const moveit::core::JointModelGroup * jmg,
  SolutionMotionScore & score)
{
  if (const auto * sub = dynamic_cast<const mtc::SubTrajectory *>(&solution)) {
    if (sub->trajectory()) {
      accumulateTrajectoryMotionScore(*sub->trajectory(), jmg, score);
    }
    return;
  }

  if (const auto * sequence = dynamic_cast<const mtc::SolutionSequence *>(&solution)) {
    for (const auto * child : sequence->solutions()) {
      if (child != nullptr) {
        accumulateSolutionMotionScore(*child, jmg, score);
      }
    }
    return;
  }

  if (const auto * wrapped = dynamic_cast<const mtc::WrappedSolution *>(&solution)) {
    if (wrapped->wrapped() != nullptr) {
      accumulateSolutionMotionScore(*wrapped->wrapped(), jmg, score);
    }
  }
}

SolutionMotionScore scoreTaskSolution(
  const mtc::SolutionBase & solution,
  const moveit::core::JointModelGroup * jmg,
  double wrist_spin_weight,
  double wrist_spin_reject_threshold,
  double desired_place_yaw,
  double place_yaw_tolerance,
  double place_yaw_penalty_weight)
{
  SolutionMotionScore score;
  accumulateSolutionMotionScore(solution, jmg, score);
  if (const auto place_yaw = placeYawFromSolution(solution)) {
    score.has_place_yaw = true;
    score.place_yaw_penalty = yawPenalty(
      *place_yaw, desired_place_yaw, place_yaw_tolerance,
      place_yaw_penalty_weight, score.place_yaw_error);
  }
  score.total =
    score.joint_motion +
    wrist_spin_weight * score.wrist_spin +
    score.place_yaw_penalty +
    1e-3 * solution.cost();
  score.rejected =
    wrist_spin_reject_threshold > 0.0 && score.wrist_spin > wrist_spin_reject_threshold;
  return score;
}

bool alignWrist3JointTrajectoryToReference(
  trajectory_msgs::msg::JointTrajectory & joint_trajectory,
  double & reference_wrist3,
  const std::string & context)
{
  const auto wrist3_index = findJointIndex(joint_trajectory.joint_names, kWrist3JointName);
  if (!wrist3_index) {
    return false;
  }

  auto first_point = std::find_if(
    joint_trajectory.points.begin(), joint_trajectory.points.end(),
    [wrist3_index](const auto & point) {
      return point.positions.size() > *wrist3_index;
    });
  if (first_point == joint_trajectory.points.end()) {
    return false;
  }

  const double first_wrist3 = first_point->positions[*wrist3_index];
  const double reference_before = reference_wrist3;
  const double offset =
    WordleBotController::computeContinuousJointRevolutionOffset(reference_wrist3, first_wrist3);

  for (auto & point : joint_trajectory.points) {
    if (point.positions.size() > *wrist3_index) {
      point.positions[*wrist3_index] += offset;
      reference_wrist3 = point.positions[*wrist3_index];
    }
  }

  if (std::abs(offset) >= 1e-9) {
    RCLCPP_DEBUG(LOGGER,
      "%s: aligned wrist_3 trajectory by %.6f rad (%+.0f rev): reference=%.6f, first_raw=%.6f, first_aligned=%.6f.",
      context.c_str(), offset, offset / kTwoPi, reference_before, first_wrist3, first_wrist3 + offset);
  }
  return true;
}

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

bool normalizePoseOrientation(
  geometry_msgs::msg::Pose & pose,
  const std::string & label,
  bool identity_if_invalid = true)
{
  const double norm = std::sqrt(
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w);

  if (norm < 1e-9) {
    if (identity_if_invalid) {
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      RCLCPP_WARN(LOGGER,
        "%s orientation quaternion was invalid/near-zero; using identity orientation.",
        label.c_str());
    }
    return false;
  }

  pose.orientation.x /= norm;
  pose.orientation.y /= norm;
  pose.orientation.z /= norm;
  pose.orientation.w /= norm;
  return true;
}

double yawFromPose(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  q.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace

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
  move_group_.setPlanningTime(15.0);
  move_group_.setNumPlanningAttempts(5);

  // Planning scene monitor for collision-aware IK in computeBestIK.
  // Subscribes to the move_group's published scene so it stays in sync with
  // any collision objects added at runtime (floor, sensor guard, letter objects).
  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
  psm_->startSceneMonitor("/monitored_planning_scene");
  psm_->requestPlanningSceneState("/get_planning_scene");
  RCLCPP_INFO(LOGGER, "PlanningSceneMonitor started — collision-aware IK enabled.");

  // DEBUG: log all link names in the robot model so we can verify touch_links names
  RCLCPP_INFO(LOGGER, "Robot model link names:");
  for (const auto & link : move_group_.getRobotModel()->getLinkModelNames()) {
    RCLCPP_INFO(LOGGER, "  link: %s", link.c_str());
  }
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_.getPlanningFrame().c_str());

  loadVelocityScalingProfiles();
}

WordleBotController::~WordleBotController()
{
}

// ---------------------------------------------------------------------------
// Velocity Scaling
// ---------------------------------------------------------------------------

void WordleBotController::loadVelocityScalingProfiles()
{
  vel_profiles_.scan_vel  = getDoubleParam(node_, "velocity_scaling.scan.velocity",     0.15);
  vel_profiles_.scan_acc  = getDoubleParam(node_, "velocity_scaling.scan.acceleration", 0.15);
  vel_profiles_.precise_vel  = getDoubleParam(node_, "velocity_scaling.precise.velocity",     0.30);
  vel_profiles_.precise_acc  = getDoubleParam(node_, "velocity_scaling.precise.acceleration", 0.30);
  vel_profiles_.transit_vel  = getDoubleParam(node_, "velocity_scaling.transit.velocity",     0.75);
  vel_profiles_.transit_acc  = getDoubleParam(node_, "velocity_scaling.transit.acceleration", 0.75);
  vel_profiles_.near_threshold = getDoubleParam(node_, "velocity_scaling.proximity.near_threshold", 0.10);
  vel_profiles_.far_threshold  = getDoubleParam(node_, "velocity_scaling.proximity.far_threshold",  0.35);

  RCLCPP_INFO(LOGGER,
    "Velocity scaling profiles loaded — scan=%.2f, precise=%.2f, transit=%.2f, "
    "proximity=[%.2f, %.2f] m.",
    vel_profiles_.scan_vel, vel_profiles_.precise_vel, vel_profiles_.transit_vel,
    vel_profiles_.near_threshold, vel_profiles_.far_threshold);
}

double WordleBotController::queryCurrentStateMinDistance() const
{
  planning_scene_monitor::LockedPlanningSceneRO lps(psm_);
  moveit::core::RobotState state = lps->getCurrentState();
  state.update();

  collision_detection::CollisionRequest req;
  req.distance = true;
  req.contacts = false;
  collision_detection::CollisionResult res;
  lps->checkCollision(req, res, state);

  // res.distance is negative when in collision; clamp to 0.0 so the ramp always
  // returns precise_vel in that (shouldn't-happen-in-practice) case.
  return std::max(0.0, res.distance);
}

double WordleBotController::computeTransitScaling(double d) const
{
  if (d >= vel_profiles_.far_threshold) return vel_profiles_.transit_vel;
  if (d <= vel_profiles_.near_threshold) return vel_profiles_.precise_vel;
  const double t = (d - vel_profiles_.near_threshold) /
                   (vel_profiles_.far_threshold - vel_profiles_.near_threshold);
  return vel_profiles_.precise_vel + t * (vel_profiles_.transit_vel - vel_profiles_.precise_vel);
}

// ---------------------------------------------------------------------------
// Collision Scene Management
// Builds and tears down the static environment (floor, sensor guard) and
// manages dynamic collision objects added at runtime.
// ---------------------------------------------------------------------------
// setupCollisionScene       — add the floor plane and attach the sensor guard
//                             cylinder to the end effector
// clearCollisionScene       — remove all static objects and detach the sensor guard
// addCollisionObject        — apply an ADD / REMOVE / MOVE collision object
//                             to the live planning scene
// clearLetterObjects        — remove a list of letter objects by ID from the scene
// attachSensorCollisionObject — attach a protective cylinder to tool0 so the
//                               planner treats the sensor as part of the robot
// detachSensorCollisionObject — detach the sensor guard cylinder from tool0
// ---------------------------------------------------------------------------

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
  // floor_shape.dimensions = {0.975, 0.525, 0.01}; gamebaord
  floor_shape.dimensions = {2.0, 2.0, 0.01};  // large flat plane to catch dropped letters

  geometry_msgs::msg::Pose floor_pose;
  floor_pose.position.x = 0.0;
  floor_pose.position.y = 0.2275;
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


void WordleBotController::clearLetterObjects(const std::vector<std::string> & ids)
{
  if (ids.empty()) {
    RCLCPP_INFO(LOGGER, "clearLetterObjects: nothing to remove.");
    return;
  }
  planning_scene_.removeCollisionObjects(ids);
  RCLCPP_INFO(LOGGER, "clearLetterObjects: removed %zu letter object(s).", ids.size());
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

// ---------------------------------------------------------------------------
// Pick and Place
// Build, plan, and execute MTC pick-and-place tasks. The primary workflow is
// the two-phase plan-then-execute path (createTask → planPickAndPlace →
// executePlannedTask). doPickAndPlace is a single-call convenience wrapper
// that plans and executes in one shot.
// ---------------------------------------------------------------------------
// createTask          — build an MTC task with all pick-and-place stages;
//                       accepts an optional chained start scene for batching
// planPickAndPlace    — plan one pick-and-place task without executing it;
//                       returns a PlannedPickPlace with the terminal scene for
//                       chaining to the next task
// executePlannedTask  — execute a previously planned pick-and-place task
// doPickAndPlace      — convenience wrapper: plan and execute in one call
// ---------------------------------------------------------------------------

mtc::Task WordleBotController::createTask(const geometry_msgs::msg::Pose & object_pose,
                                          const geometry_msgs::msg::Pose & place_pose,
                                          const std::string & object_id,
                                          const planning_scene::PlanningScenePtr & start_scene,
                                          bool include_return_home)
{
  RCLCPP_DEBUG(LOGGER, "createTask: initialising MTC task.");
  auto normalized_object_pose = object_pose;
  auto normalized_place_pose = place_pose;
  normalizePoseOrientation(normalized_object_pose, "createTask pick");
  normalizePoseOrientation(normalized_place_pose, "createTask place");

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
  RCLCPP_INFO(LOGGER,
    "createTask [%s]: soft place-yaw mode enabled: pick_yaw=%.3f rad, desired_place_yaw=%.3f rad.",
    object_id.c_str(), yawFromPose(normalized_object_pose), yawFromPose(normalized_place_pose));

  // Explicitly name the OMPL pipeline so MTC never falls back to CHOMP.
  // ompl_planning.yaml must be loaded into the node's parameters (see wordle_bot.launch.py).
  // The custom planner keeps MTC's staged scene handling, but generates multiple
  // OMPL candidates and selects the one with the least joint motion.
  auto sampling_planner      = std::make_shared<WordleMtcPlanner>(node_, "ompl");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  const double cartesian_step_size =
    getDoubleParam(node_, "pick_place.cartesian_step_size", 0.001);
  const double retreat_min_fraction =
    getDoubleParam(node_, "pick_place.retreat_min_fraction", 0.0);
  const double move_to_pick_timeout =
    getDoubleParam(node_, "pick_place.move_to_pick_timeout", 0.50);
  const double move_to_place_timeout =
    getDoubleParam(node_, "pick_place.move_to_place_timeout", 0.50);
  const double approach_min_distance =
    getDoubleParam(node_, "pick_place.approach_min_distance", 0.05);
  const double approach_max_distance =
    getDoubleParam(node_, "pick_place.approach_max_distance", 0.15);
  const double lift_min_distance =
    getDoubleParam(node_, "pick_place.lift_min_distance", 0.05);
  const double lift_max_distance =
    getDoubleParam(node_, "pick_place.lift_max_distance", 0.15);
  const double retreat_min_distance =
    getDoubleParam(node_, "pick_place.retreat_min_distance", 0.03);
  const double retreat_max_distance =
    getDoubleParam(node_, "pick_place.retreat_max_distance", 0.15);
  const double grasp_z_offset =
    getDoubleParam(node_, "pick_place.grasp_z_offset", 0.01);
  const double grasp_angle_delta =
    getDoubleParam(node_, "pick_place.grasp_angle_delta", M_PI / 12.0);
  const int grasp_max_ik_solutions =
    std::max(1, getIntParam(node_, "pick_place.grasp_max_ik_solutions", 32));
  const int place_max_ik_solutions =
    std::max(1, getIntParam(node_, "pick_place.place_max_ik_solutions", 32));
  const double grasp_min_solution_distance =
    getDoubleParam(node_, "pick_place.grasp_min_solution_distance", 0.10);
  const double place_min_solution_distance =
    getDoubleParam(node_, "pick_place.place_min_solution_distance", 0.10);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(vel_profiles_.precise_vel);
  cartesian_planner->setMaxAccelerationScalingFactor(vel_profiles_.precise_acc);
  cartesian_planner->setStepSize(cartesian_step_size);
  RCLCPP_DEBUG(LOGGER,
    "createTask: CartesianPath planner configured (vel=%.3f, acc=%.3f, step=%.4f).",
    vel_profiles_.precise_vel, vel_profiles_.precise_acc, cartesian_step_size);

  // Dedicated planner for retreat: min_fraction=0 accepts whatever Cartesian
  // distance the arm can actually achieve at the (low, far-reach) place pose.
  auto retreat_planner = std::make_shared<mtc::solvers::CartesianPath>();
  retreat_planner->setMaxVelocityScalingFactor(vel_profiles_.precise_vel);
  retreat_planner->setMaxAccelerationScalingFactor(vel_profiles_.precise_acc);
  retreat_planner->setStepSize(cartesian_step_size);
  retreat_planner->setMinFraction(retreat_min_fraction);

  // ── Stage 1: start state — CurrentState (task 1) or FixedState (tasks 2..N) ──
  mtc::Stage * current_state_ptr = nullptr;
  {
    if (start_scene == nullptr) {
      RCLCPP_INFO(LOGGER, "createTask [%s]: Stage 1 = CurrentState (live robot state).",
                  object_id.c_str());
      auto stage = std::make_unique<mtc::stages::CurrentState>("current");
      current_state_ptr = stage.get();
      task.add(std::move(stage));
    } else {
      // Log the joint state embedded in the chained scene to verify correct threading.
      const moveit::core::RobotState & rs = start_scene->getCurrentState();
      const moveit::core::JointModelGroup * jmg =
          rs.getRobotModel()->getJointModelGroup("ur_onrobot_manipulator");
      if (jmg) {
        std::vector<double> jv;
        rs.copyJointGroupPositions(jmg, jv);
        const auto & jnames = jmg->getVariableNames();
        RCLCPP_INFO(LOGGER,
          "createTask [%s]: Stage 1 = FixedState (chained from previous solution). "
          "Start joints (%zu):", object_id.c_str(), jv.size());
        for (std::size_t ji = 0; ji < jv.size() && ji < jnames.size(); ++ji) {
          RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
            jnames[ji].c_str(), jv[ji], jv[ji] * 180.0 / M_PI);
        }
      }
      auto stage = std::make_unique<mtc::stages::FixedState>("fixed start");
      stage->setState(start_scene);
      current_state_ptr = stage.get();
      task.add(std::move(stage));
    }
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
    RCLCPP_INFO(LOGGER,
      "createTask: adding stage 3 — Connect 'move to pick' (timeout=%.2f s).",
      move_to_pick_timeout);
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
    stage_move_to_pick->setTimeout(move_to_pick_timeout);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    // stage_move_to_pick->setPathConstraints(buildJointLimitConstraints());
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
        "(link='%s', dist=[0.05, 0.15], dir=+z in %s).",
        hand_frame.c_str(), hand_frame.c_str());

      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(approach_min_distance, approach_max_distance);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // 4b. Sample object-frame grasp poses + solve IK.
    {
      RCLCPP_INFO(LOGGER,
        "createTask: 4b — GenerateGraspPose for object '%s' "
        "(angle_delta=%.6f rad, pick_yaw=%.3f rad, IK solutions=%d, z_offset=%.3f m).",
        object_id.c_str(), grasp_angle_delta, yawFromPose(normalized_object_pose),
        grasp_max_ik_solutions, grasp_z_offset);
      auto stage =
        std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(object_id);
      stage->setAngleDelta(grasp_angle_delta);
      stage->setMonitoredStage(current_state_ptr);

      // Transform from gripper_tcp to the object centre when grasping top-down.
      // z=0.08 means gripper_tcp sits 80 mm above the object centre at grasp time.
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = grasp_z_offset;

      const double expected_grasp_z   = normalized_object_pose.position.z + grasp_z_offset;
      const double expected_approach_z_min = expected_grasp_z + approach_min_distance;
      const double expected_approach_z_max = expected_grasp_z + approach_max_distance;
      RCLCPP_INFO(LOGGER,
        "createTask [grasp geometry]: object_z=%.4f m  grasp_z_offset=%.3f m  "
        "=> expected gripper_tcp z AT GRASP = %.4f m  "
        "| pre-approach z range = [%.4f, %.4f] m (world frame, top-down).",
        normalized_object_pose.position.z, grasp_z_offset,
        expected_grasp_z, expected_approach_z_min, expected_approach_z_max);

      auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(static_cast<uint32_t>(grasp_max_ik_solutions));
      wrapper->setMinSolutionDistance(grasp_min_solution_distance);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->setProperty("default_pose", std::string("test_configuration"));
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    // 4c. Allow collisions between gripper links and the object
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4c — ModifyPlanningScene allow collision "
        "('%s', hand links).", object_id.c_str());
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (hand,object)");
      stage->allowCollisions( object_id,
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

    // 4e. Attach object to the gripper link — the place pose generator monitors this stage.
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4e — ModifyPlanningScene attach '%s' to '%s'.",
        object_id.c_str(), hand_frame.c_str());
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_id, hand_frame);
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
      stage->setMinMaxDistance(lift_min_distance, lift_max_distance);
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
    RCLCPP_INFO(LOGGER,
      "createTask: adding stage 5 — Connect 'move to place' (timeout=%.2f s).",
      move_to_place_timeout);
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{
        {arm_group, sampling_planner}});
    stage->setTimeout(move_to_place_timeout);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    // stage->setPathConstraints(buildJointLimitConstraints());
    task.add(std::move(stage));
  }

  // ── Stage 6: place container ──────────────────────────────────────────────
  {
    RCLCPP_DEBUG(LOGGER, "createTask: building stage 6 — SerialContainer 'place object'.");
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // 6a. Generate place pose alternatives + solve IK (object must reach the requested slot in world)
    {
      RCLCPP_INFO(LOGGER,
        "createTask: 6a — GeneratePlacePose for '%s' in soft-yaw mode "
        "target=(%.3f, %.3f, %.3f) world, place_yaw=%.3f rad, IK solutions=%d.",
        object_id.c_str(),
        normalized_place_pose.position.x, normalized_place_pose.position.y,
        normalized_place_pose.position.z, yawFromPose(normalized_place_pose),
        place_max_ik_solutions);
      auto stage =
        std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object_id);

      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = "world";
      target_pose.pose = normalized_place_pose;
      stage->setPose(target_pose);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(static_cast<uint32_t>(place_max_ik_solutions));
      wrapper->setMinSolutionDistance(place_min_solution_distance);
      wrapper->setIKFrame(object_id);
      wrapper->setProperty("default_pose", std::string("test_configuration"));
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
        "('%s', hand links).", object_id.c_str());
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "forbid collision (hand,object)");
      stage->allowCollisions(
        object_id,
        task.getRobotModel()
          ->getJointModelGroup(hand_group)
          ->getLinkModelNamesWithCollisionGeometry(),
        false);
      place->insert(std::move(stage));
    }

    // 6d. Detach object from gripper
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 6d — ModifyPlanningScene detach '%s' from '%s'.",
        object_id.c_str(), hand_frame.c_str());
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_id, hand_frame);
      place->insert(std::move(stage));
    }

    // 6e. Retreat vertically away from the placed object
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 6e — MoveRelative 'retreat' "
        "(dist=[0.05, 0.15], dir=+z world).");
      auto stage =
        std::make_unique<mtc::stages::MoveRelative>("retreat", retreat_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(retreat_min_distance, retreat_max_distance);
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

  // ── Stage 7: return to working pose — only for the final task in a batch ──
  if (include_return_home) {
    RCLCPP_DEBUG(LOGGER, "createTask [%s]: adding stage 7 — 'return to working pose'.", object_id.c_str());
    auto stage = std::make_unique<mtc::stages::MoveTo>("return to working pose", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal(std::map<std::string, double>{
      {"shoulder_pan_joint",  node_->get_parameter("working_joints.shoulder_pan_joint").as_double()},
      {"shoulder_lift_joint", node_->get_parameter("working_joints.shoulder_lift_joint").as_double()},
      {"elbow_joint",         node_->get_parameter("working_joints.elbow_joint").as_double()},
      {"wrist_1_joint",       node_->get_parameter("working_joints.wrist_1_joint").as_double()},
      {"wrist_2_joint",       node_->get_parameter("working_joints.wrist_2_joint").as_double()},
      {"wrist_3_joint",       node_->get_parameter("working_joints.wrist_3_joint").as_double()},
    });
    task.add(std::move(stage));
  } else {
    RCLCPP_DEBUG(LOGGER,
      "createTask [%s]: skipping stage 7 — not the last task in the batch.", object_id.c_str());
  }

  RCLCPP_DEBUG(LOGGER, "createTask [%s]: task construction complete.", object_id.c_str());
  return task;
}

WordleBotController::PlannedPickPlace WordleBotController::planPickAndPlace(
  const PickPlaceEntry & entry,
  const planning_scene::PlanningScenePtr & start_scene,
  bool include_return_home)
{
  PlannedPickPlace result;
  result.object_id = entry.object_id;

  RCLCPP_INFO(LOGGER,
    "planPickAndPlace [%s]: building task (start=%s, return_home=%s).",
    entry.object_id.c_str(),
    start_scene ? "FixedState(chained)" : "CurrentState(live)",
    include_return_home ? "yes" : "no");

  // createTask() returns by value; Task has a move constructor so this move-constructs
  // into the heap-allocated object without any copy.
  result.task = std::make_unique<mtc::Task>(
    createTask(entry.pick_pose, entry.place_pose, entry.object_id,
               start_scene, include_return_home));

  try {
    result.task->init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER,
      "planPickAndPlace [" << entry.object_id << "]: task init failed: " << e);
    result.task.reset();
    return result;
  }

  const int task_solution_target_count =
    std::max(1, getIntParam(node_, "pick_place.task_solution_target_count", 15));
  const double accept_solution_score_threshold =
    getDoubleParam(node_, "pick_place.accept_solution_score_threshold", 25.0);
  RCLCPP_INFO(LOGGER,
    "planPickAndPlace [%s]: planning incrementally (target %d solution(s), accept score <= %.3f)...",
    entry.object_id.c_str(), task_solution_target_count, accept_solution_score_threshold);
  moveit::core::MoveItErrorCode plan_result;
  const auto * jmg = move_group_.getRobotModel()->getJointModelGroup("ur_onrobot_manipulator");
  const double solution_wrist_spin_weight =
    getDoubleParam(node_, "pick_place.solution_wrist_spin_weight", 4.0);
  const double solution_wrist_spin_reject_threshold =
    getDoubleParam(node_, "pick_place.solution_wrist_spin_reject_threshold", 7.0);
  auto normalized_place_pose = entry.place_pose;
  normalizePoseOrientation(normalized_place_pose, "planPickAndPlace place");
  const double desired_place_yaw = yawFromPose(normalized_place_pose);
  const double place_yaw_tolerance =
    getDoubleParam(node_, "pick_place.place_yaw_tolerance", 0.0872665);
  const double place_yaw_penalty_weight =
    getDoubleParam(node_, "pick_place.place_yaw_penalty_weight", 100.0);

  for (int target_count = 1; target_count <= task_solution_target_count; ++target_count) {
    try {
      plan_result = result.task->plan(static_cast<std::size_t>(target_count));
    } catch (const mtc::InitStageException & e) {
      RCLCPP_ERROR_STREAM(LOGGER,
        "planPickAndPlace [" << entry.object_id << "]: planning threw: " << e);
      result.task.reset();
      return result;
    }

    if (!result.task->solutions().empty()) {
      double current_best_score = std::numeric_limits<double>::infinity();
      for (const auto & solution : result.task->solutions()) {
        const auto score = scoreTaskSolution(
          *solution, jmg, solution_wrist_spin_weight, solution_wrist_spin_reject_threshold,
          desired_place_yaw, place_yaw_tolerance, place_yaw_penalty_weight);
        if (!score.rejected && score.total < current_best_score) {
          current_best_score = score.total;
        }
      }

      RCLCPP_INFO(LOGGER,
        "planPickAndPlace [%s]: incremental planning target=%d/%d, complete_solutions=%zu, "
        "best_non_rejected_score=%.4f.",
        entry.object_id.c_str(), target_count, task_solution_target_count,
        result.task->solutions().size(), current_best_score);

      if (current_best_score <= accept_solution_score_threshold) {
        RCLCPP_INFO(LOGGER,
          "planPickAndPlace [%s]: accepting early because best score %.4f <= threshold %.4f.",
          entry.object_id.c_str(), current_best_score, accept_solution_score_threshold);
        break;
      }
    }

    if (!plan_result) {
      break;
    }
  }

  if (!plan_result || result.task->solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "planPickAndPlace [%s]: no solutions found.", entry.object_id.c_str());
    result.task.reset();
    return result;
  }

  double best_score = std::numeric_limits<double>::infinity();
  const mtc::SolutionBase * best_solution = nullptr;
  int rejected_solutions = 0;
  std::size_t solution_index = 0;
  for (const auto & solution : result.task->solutions()) {
    ++solution_index;
    const auto score = scoreTaskSolution(
      *solution, jmg, solution_wrist_spin_weight, solution_wrist_spin_reject_threshold,
      desired_place_yaw, place_yaw_tolerance, place_yaw_penalty_weight);
    RCLCPP_INFO(LOGGER,
      "planPickAndPlace [%s]: complete solution %zu/%zu: mtc_cost=%.3f "
      "joint_motion=%.4f wrist_spin=%.4f place_yaw_error=%.4f place_yaw_penalty=%.4f "
      "trajectories=%zu score=%.4f%s%s.",
      entry.object_id.c_str(), solution_index, result.task->solutions().size(),
      solution->cost(), score.joint_motion, score.wrist_spin, score.place_yaw_error,
      score.place_yaw_penalty, score.trajectory_count, score.total,
      score.has_place_yaw ? "" : " (no place yaw)",
      score.rejected ? " (rejected wrist spin)" : "");
    if (score.rejected) {
      ++rejected_solutions;
      continue;
    }
    if (score.total < best_score) {
      best_score = score.total;
      best_solution = solution.get();
    }
  }

  if (best_solution == nullptr) {
    RCLCPP_WARN(LOGGER,
      "planPickAndPlace [%s]: all %zu complete solution(s) were rejected by wrist-spin scoring; "
      "falling back to lowest-scored rejected solution so planning can continue.",
      entry.object_id.c_str(), result.task->solutions().size());
    for (const auto & solution : result.task->solutions()) {
      const auto score = scoreTaskSolution(
        *solution, jmg, solution_wrist_spin_weight, 0.0,
        desired_place_yaw, place_yaw_tolerance, place_yaw_penalty_weight);
      if (score.total < best_score) {
        best_score = score.total;
        best_solution = solution.get();
      }
    }
  }

  // best_solution is a raw pointer into the task's internal storage.
  // It remains valid as long as result.task is alive — both live in PlannedPickPlace.
  result.best_solution = best_solution;

  RCLCPP_INFO(LOGGER,
    "planPickAndPlace [%s]: planning succeeded — %zu solution(s), rejected=%d, selected_mtc_cost=%.3f, "
    "selected_motion_score=%.4f.",
    entry.object_id.c_str(),
    result.task->solutions().size(),
    rejected_solutions,
    result.best_solution->cost(),
    best_score);

  const auto * grasp_ik_stage = result.task->findChild("pick object/grasp pose IK");
  const auto * place_ik_stage = result.task->findChild("place object/place pose IK");
  if (grasp_ik_stage != nullptr || place_ik_stage != nullptr) {
    RCLCPP_INFO(LOGGER,
      "planPickAndPlace [%s]: IK stage solutions — grasp=%zu, place=%zu.",
      entry.object_id.c_str(),
      grasp_ik_stage ? grasp_ik_stage->solutions().size() : 0,
      place_ik_stage ? place_ik_stage->solutions().size() : 0);
  }

  // Extract the terminal planning scene from the solution's end InterfaceState.
  // PlanningScene::clone() produces a fully independent non-const copy, which is
  // safe to pass to FixedState::setState() for the next task. Using const_pointer_cast
  // would alias the Task's internal scene — risky if MTC mutates it later.
  if (result.best_solution->end() == nullptr) {
    RCLCPP_ERROR(LOGGER,
      "planPickAndPlace [%s]: solution end state is null — cannot chain next task.",
      entry.object_id.c_str());
  } else {
    result.end_scene = planning_scene::PlanningScene::clone(
      result.best_solution->end()->scene());

    // Debug: log terminal joint state so we can verify chaining is correct.
    const moveit::core::RobotState & end_rs = result.end_scene->getCurrentState();
    const moveit::core::JointModelGroup * jmg =
        end_rs.getRobotModel()->getJointModelGroup("ur_onrobot_manipulator");
    if (jmg) {
      std::vector<double> end_jv;
      end_rs.copyJointGroupPositions(jmg, end_jv);
      const auto & jnames = jmg->getVariableNames();
      RCLCPP_INFO(LOGGER,
        "planPickAndPlace [%s]: terminal joint state (%zu joints) — "
        "will be FixedState start for next task:",
        entry.object_id.c_str(), end_jv.size());
      for (std::size_t ji = 0; ji < end_jv.size() && ji < jnames.size(); ++ji) {
        RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
          jnames[ji].c_str(), end_jv[ji], end_jv[ji] * 180.0 / M_PI);
      }
    }

    // Debug: confirm all expected collision objects are still present in the terminal scene.
    const std::vector<std::string> obj_ids = result.end_scene->getWorld()->getObjectIds();
    RCLCPP_INFO(LOGGER,
      "planPickAndPlace [%s]: terminal scene contains %zu world object(s):",
      entry.object_id.c_str(), obj_ids.size());
    for (const auto & oid : obj_ids) {
      RCLCPP_INFO(LOGGER, "  object: %s", oid.c_str());
    }
  }

  return result;
}


bool WordleBotController::executePlannedTask(PlannedPickPlace & planned)
{
  if (!planned.task || !planned.best_solution) {
    RCLCPP_ERROR(LOGGER,
      "executePlannedTask [%s]: task or solution is null — cannot execute.",
      planned.object_id.c_str());
    return false;
  }

  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER,
      "executePlannedTask [%s]: stop requested before execute — aborting.",
      planned.object_id.c_str());
    return false;
  }

  RCLCPP_INFO(LOGGER,
    "executePlannedTask [%s]: publishing solution for visualisation (cost=%.3f).",
    planned.object_id.c_str(), planned.best_solution->cost());
  planned.task->introspection().publishSolution(*planned.best_solution);
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "executePlannedTask [%s]: executing...", planned.object_id.c_str());
  const auto result = executeAlignedTaskSolution(
    *planned.task, *planned.best_solution,
    "executePlannedTask [" + planned.object_id + "]");

  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER,
      "executePlannedTask [%s]: execution failed (error code %d).",
      planned.object_id.c_str(), result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "executePlannedTask [%s]: execution succeeded.", planned.object_id.c_str());
  return true;
}

bool WordleBotController::doPickAndPlace(const geometry_msgs::msg::Pose & object_pose,
                                         const geometry_msgs::msg::Pose & place_pose,
                                         const std::string & object_id)
{
  RCLCPP_INFO(LOGGER, "doPickAndPlace: building MTC task for object '%s'.", object_id.c_str());

  RCLCPP_INFO(LOGGER,
    "doPickAndPlace: object pose = (%.4f, %.4f, %.4f)  "
    "orient=(%.4f, %.4f, %.4f, %.4f)  "
    "place target = (%.4f, %.4f, %.4f). "
    "MTC planning scene frame: 'world'.",
    object_pose.position.x, object_pose.position.y, object_pose.position.z,
    object_pose.orientation.x, object_pose.orientation.y,
    object_pose.orientation.z, object_pose.orientation.w,
    place_pose.position.x, place_pose.position.y, place_pose.position.z);

  mtc::Task task = createTask(object_pose, place_pose, object_id, nullptr, true);

  try {
    task.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "MTC task init failed: " << e);
    return false;
  }

  RCLCPP_INFO(LOGGER, "doPickAndPlace: planning...");
  const int task_solution_target_count =
    std::max(1, getIntParam(node_, "pick_place.task_solution_target_count", 15));
  const double accept_solution_score_threshold =
    getDoubleParam(node_, "pick_place.accept_solution_score_threshold", 25.0);
  const auto * jmg = move_group_.getRobotModel()->getJointModelGroup("ur_onrobot_manipulator");
  const double solution_wrist_spin_weight =
    getDoubleParam(node_, "pick_place.solution_wrist_spin_weight", 4.0);
  const double solution_wrist_spin_reject_threshold =
    getDoubleParam(node_, "pick_place.solution_wrist_spin_reject_threshold", 7.0);
  auto normalized_place_pose = place_pose;
  normalizePoseOrientation(normalized_place_pose, "doPickAndPlace place");
  const double desired_place_yaw = yawFromPose(normalized_place_pose);
  const double place_yaw_tolerance =
    getDoubleParam(node_, "pick_place.place_yaw_tolerance", 0.0872665);
  const double place_yaw_penalty_weight =
    getDoubleParam(node_, "pick_place.place_yaw_penalty_weight", 100.0);
  moveit::core::MoveItErrorCode plan_result;
  for (int target_count = 1; target_count <= task_solution_target_count; ++target_count) {
    try {
      plan_result = task.plan(static_cast<std::size_t>(target_count));
    } catch (const mtc::InitStageException & e) {
      RCLCPP_ERROR_STREAM(LOGGER, "MTC task planning threw InitStageException: " << e);
      return false;
    }

    if (!task.solutions().empty()) {
      double current_best_score = std::numeric_limits<double>::infinity();
      for (const auto & solution : task.solutions()) {
        const auto score = scoreTaskSolution(
          *solution, jmg, solution_wrist_spin_weight, solution_wrist_spin_reject_threshold,
          desired_place_yaw, place_yaw_tolerance, place_yaw_penalty_weight);
        if (!score.rejected && score.total < current_best_score) {
          current_best_score = score.total;
        }
      }

      RCLCPP_INFO(LOGGER,
        "doPickAndPlace: incremental planning target=%d/%d, complete_solutions=%zu, "
        "best_non_rejected_score=%.4f.",
        target_count, task_solution_target_count, task.solutions().size(), current_best_score);

      if (current_best_score <= accept_solution_score_threshold) {
        RCLCPP_INFO(LOGGER,
          "doPickAndPlace: accepting early because best score %.4f <= threshold %.4f.",
          current_best_score, accept_solution_score_threshold);
        break;
      }
    }

    if (!plan_result) {
      break;
    }
  }

  if (!plan_result || task.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "MTC task planning failed — no solutions found.");
    return false;
  }

  const mtc::SolutionBase * best_solution = nullptr;
  double best_score = std::numeric_limits<double>::infinity();
  std::size_t solution_index = 0;
  for (const auto & solution : task.solutions()) {
    ++solution_index;
    const auto score = scoreTaskSolution(
      *solution, jmg, solution_wrist_spin_weight, solution_wrist_spin_reject_threshold,
      desired_place_yaw, place_yaw_tolerance, place_yaw_penalty_weight);
    RCLCPP_INFO(LOGGER,
      "doPickAndPlace: complete solution %zu/%zu: mtc_cost=%.3f joint_motion=%.4f "
      "wrist_spin=%.4f place_yaw_error=%.4f place_yaw_penalty=%.4f "
      "trajectories=%zu score=%.4f%s%s.",
      solution_index, task.solutions().size(), solution->cost(), score.joint_motion,
      score.wrist_spin, score.place_yaw_error, score.place_yaw_penalty,
      score.trajectory_count, score.total, score.has_place_yaw ? "" : " (no place yaw)",
      score.rejected ? " (rejected wrist spin)" : "");
    if (!score.rejected && score.total < best_score) {
      best_score = score.total;
      best_solution = solution.get();
    }
  }

  if (best_solution == nullptr) {
    RCLCPP_WARN(LOGGER,
      "doPickAndPlace: all complete solutions were rejected by wrist-spin scoring; "
      "falling back to lowest-scored solution.");
    for (const auto & solution : task.solutions()) {
      const auto score = scoreTaskSolution(
        *solution, jmg, solution_wrist_spin_weight, 0.0,
        desired_place_yaw, place_yaw_tolerance, place_yaw_penalty_weight);
      if (score.total < best_score) {
        best_score = score.total;
        best_solution = solution.get();
      }
    }
  }

  RCLCPP_INFO(LOGGER,
    "doPickAndPlace: planning succeeded — %zu solution(s) found. "
    "Executing selected solution (mtc_cost=%.3f, motion_score=%.4f).",
    task.solutions().size(), best_solution->cost(), best_score);

  RCLCPP_DEBUG(LOGGER, "doPickAndPlace: publishing planned solution for visualization.");
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  task.introspection().publishSolution(*best_solution);

  RCLCPP_INFO(LOGGER, "doPickAndPlace: executing...");
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  auto result = executeAlignedTaskSolution(task, *best_solution, "doPickAndPlace");
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "MTC task execution failed (error code %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "doPickAndPlace: pick-and-place succeeded.");
  return true;
}

// ---------------------------------------------------------------------------
// Goal Navigation
// Plan and execute free-space goal-pose moves using the OMPL sampling planner.
// Supports scene chaining so multiple goals can be planned sequentially before
// any motion begins.
// ---------------------------------------------------------------------------
// planMoveToGoal         — plan a move to an absolute goal pose; accepts an
//                          optional chained start scene for multi-goal missions
// executePlannedMoveToGoal — execute a previously planned move-to-goal task
// ---------------------------------------------------------------------------

WordleBotController::PlannedMoveToGoal WordleBotController::planMoveToGoal(
  const geometry_msgs::msg::Pose & goal_pose,
  const planning_scene::PlanningScenePtr & start_scene,
  bool include_return_home)
{
  PlannedMoveToGoal result;

  RCLCPP_INFO(LOGGER,
    "planMoveToGoal: building task (start=%s, return_home=%s).",
    start_scene ? "FixedState(chained)" : "CurrentState(live)",
    include_return_home ? "yes" : "no");

  const std::string arm_group = "ur_onrobot_manipulator";

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  {
    const double s = computeTransitScaling(queryCurrentStateMinDistance());
    sampling_planner->setMaxVelocityScalingFactor(s);
    sampling_planner->setMaxAccelerationScalingFactor(s);
    RCLCPP_DEBUG(LOGGER, "planMoveToGoal: transit velocity scaling = %.2f.", s);
  }
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto task = std::make_unique<mtc::Task>();
  task->stages()->setName("move to goal");
  task->loadRobotModel(node_);
  task->setProperty("group", arm_group);

  if (start_scene == nullptr) {
    task->add(std::make_unique<mtc::stages::CurrentState>("current state"));
  } else {
    auto fixed = std::make_unique<mtc::stages::FixedState>("fixed state");
    fixed->setState(start_scene);
    task->add(std::move(fixed));
  }

  {
    geometry_msgs::msg::PoseStamped goal_stamped;
    goal_stamped.header.frame_id = "world";
    goal_stamped.pose = goal_pose;

    auto stage = std::make_unique<mtc::stages::MoveTo>("move to goal", sampling_planner);
    stage->setGroup(arm_group);
    stage->setGoal(goal_stamped);
    // stage->setPathConstraints(buildPathConstraints());
    task->add(std::move(stage));
  }

  if (include_return_home) {
    auto home = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    home->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    home->setGoal("home");
    task->add(std::move(home));
  }

  try {
    task->init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "planMoveToGoal: task init failed: " << e);
    return result;
  }

  moveit::core::MoveItErrorCode plan_result;
  try {
    plan_result = task->plan(100);
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "planMoveToGoal: planning threw: " << e);
    return result;
  }

  if (!plan_result || task->solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "planMoveToGoal: no solution found.");
    return result;
  }

  result.task = std::move(task);
  result.best_solution = result.task->solutions().front().get();

  RCLCPP_INFO(LOGGER, "planMoveToGoal: planning succeeded — cost=%.3f.",
    result.best_solution->cost());

  if (result.best_solution->end() != nullptr) {
    result.end_scene = planning_scene::PlanningScene::clone(
      result.best_solution->end()->scene());
  } else {
    RCLCPP_WARN(LOGGER, "planMoveToGoal: solution end state is null — chaining may fail.");
  }

  return result;
}

bool WordleBotController::executePlannedMoveToGoal(PlannedMoveToGoal & planned)
{
  if (!planned.task || !planned.best_solution) {
    RCLCPP_ERROR(LOGGER, "executePlannedMoveToGoal: task or solution is null.");
    return false;
  }

  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "executePlannedMoveToGoal: stop requested — aborting.");
    return false;
  }

  RCLCPP_INFO(LOGGER, "executePlannedMoveToGoal: publishing solution (cost=%.3f).",
    planned.best_solution->cost());
  planned.task->introspection().publishSolution(*planned.best_solution);
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "executePlannedMoveToGoal: executing...");
  const auto result = executeAlignedTaskSolution(
    *planned.task, *planned.best_solution, "executePlannedMoveToGoal");

  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "executePlannedMoveToGoal: execution failed (error code %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "executePlannedMoveToGoal: execution succeeded.");
  return true;
}

// ---------------------------------------------------------------------------
// MoveGroupInterface Goal Navigation (USE_MTC_FOR_GOALS == false)
// Sequential plan-then-execute per goal. Three private helpers feed into the
// public moveToGoal entry point.
// ---------------------------------------------------------------------------
// computeBestIK         — IK with warm-start seeding, 2π normalisation,
//                         wrist_3 clamping, no shoulder rejection
// generateCandidatePlans — call move_group_.plan() N times, collect successes
// selectBestPlan        — pick lowest-cost plan by computeTotalJointDisplacement
// moveToGoal            — orchestrator: IK → set target → plan × 5 → best → execute
// ---------------------------------------------------------------------------

std::vector<double> WordleBotController::computeBestIK(
  const moveit::core::RobotStatePtr & current_state,
  const geometry_msgs::msg::Pose & target_pose)
{
  const std::string arm_group = "ur_onrobot_manipulator";
  const auto * jmg = move_group_.getRobotModel()->getJointModelGroup(arm_group);
  if (jmg == nullptr) {
    RCLCPP_ERROR(LOGGER, "computeBestIK: joint model group '%s' not found.", arm_group.c_str());
    return {};
  }
  if (current_state == nullptr) {
    RCLCPP_ERROR(LOGGER, "computeBestIK: current robot state is null.");
    return {};
  }
  if (!jmg->getSolverInstance()) {
    RCLCPP_ERROR(LOGGER, "computeBestIK: no kinematics solver loaded for group '%s'.", arm_group.c_str());
    return {};
  }

  // Warm-start and functional-position config [shoulder_pan, shoulder_lift, elbow,
  // wrist_1, wrist_2, wrist_3] = [65°, -90°, 90°, -90°, -90°, 65°]
  static const std::vector<double> kWarmStart   = {1.1345, -1.5708, 1.5708, -1.5708, -1.5708, 1.1345};
  static const std::vector<double> kFuncPos     = {1.1345, -1.5708, 1.5708, -1.5708, -1.5708, 1.1345};
  static const std::vector<double> kFuncWeights = {0.3, 0.5, 0.5, 0.5, 0.3, 0.3};
  static constexpr int kWarmAttempts  = 5;
  static constexpr int kTotalAttempts = 15;

  std::vector<double> current_joint_values;
  current_state->copyJointGroupPositions(jmg, current_joint_values);

  const auto & joint_names   = jmg->getVariableNames();
  const auto & active_joints = jmg->getActiveJointModels();

  std::vector<double> best_joint_values;
  double best_cost  = std::numeric_limits<double>::infinity();
  int ik_successes  = 0;

  struct IkDiagnostics {
    int total_attempts = 0;
    int solver_failures = 0;
    int collision_callback_rejections = 0;
    int collision_rejected_attempts = 0;
    int wrist3_filter_rejections = 0;
    int wrist1_filter_rejections = 0;
    int accepted_candidates = 0;
  } ik_diag;

  auto get_joint_value = [&](const std::vector<double> & values,
                             const char * joint_name) -> double {
    for (std::size_t i = 0; i < joint_names.size() && i < values.size(); ++i) {
      if (joint_names[i] == joint_name) {
        return values[i];
      }
    }
    return std::numeric_limits<double>::quiet_NaN();
  };

  auto log_rejection = [&](int attempt, const char * reason,
                           double wrist1, double wrist2, double wrist3) {
    RCLCPP_DEBUG(LOGGER,
      "computeBestIK: attempt %d rejected by %s. "
      "wrist_1=%.4f wrist_2=%.4f wrist_3=%.4f. "
      "target xyz=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f).",
      attempt, reason, wrist1, wrist2, wrist3,
      target_pose.position.x, target_pose.position.y, target_pose.position.z,
      target_pose.orientation.x, target_pose.orientation.y,
      target_pose.orientation.z, target_pose.orientation.w);
  };

  // Collision-aware IK: reject any candidate that puts the robot in collision.
  // LockedPlanningSceneRO holds the read lock for the full IK loop so the scene
  // cannot change mid-loop and the lambda can safely query it.
  planning_scene_monitor::LockedPlanningSceneRO lps(psm_);
  int active_ik_attempt = -1;
  auto isCollisionFree = [&](moveit::core::RobotState* state,
                              const moveit::core::JointModelGroup* group,
                              const double* ik_solution) -> bool
  {
    state->setJointGroupPositions(group, ik_solution);
    state->update();
    const bool colliding = lps->isStateColliding(*state, group->getName());
    if (colliding) {
      ++ik_diag.collision_callback_rejections;
      std::vector<double> rejected_values;
      state->copyJointGroupPositions(group, rejected_values);
      log_rejection(active_ik_attempt, "collision callback",
        get_joint_value(rejected_values, "wrist_1_joint"),
        get_joint_value(rejected_values, "wrist_2_joint"),
        get_joint_value(rejected_values, "wrist_3_joint"));
    }
    return !colliding;
  };

  // One IK attempt: seed → solve → normalise → validate → cost. Uses return
  // in place of continue so the retry loop below can reuse the same logic.
  auto runIKAttempt = [&](int attempt) {
    ++ik_diag.total_attempts;
    active_ik_attempt = attempt;
    const int collision_rejections_before = ik_diag.collision_callback_rejections;

    moveit::core::RobotState ik_state(*current_state);
    if (attempt < kWarmAttempts) {
      ik_state.setJointGroupPositions(jmg, kWarmStart);
      ik_state.update();
    } else {
      ik_state.setToRandomPositions(jmg);
    }

    if (!ik_state.setFromIK(jmg, target_pose, "gripper_tcp", 0.1, isCollisionFree)) {
      if (ik_diag.collision_callback_rejections > collision_rejections_before) {
        ++ik_diag.collision_rejected_attempts;
      } else {
        ++ik_diag.solver_failures;
      }
      return;
    }
    ++ik_successes;

    std::vector<double> candidate;
    ik_state.copyJointGroupPositions(jmg, candidate);

    // 2π normalisation: map each revolute joint to the 2π-equivalent numerically
    // closest to the current state, preventing huge spinning trajectories.
    for (std::size_t i = 0; i < candidate.size() && i < active_joints.size(); ++i) {
      if (active_joints[i]->getType() != moveit::core::JointModel::REVOLUTE) {
        continue;
      }
      const double curr = current_joint_values[i];
      const double raw  = candidate[i];
      double best_norm  = raw;
      double best_dist  = std::abs(raw - curr);
      for (int k = -3; k <= 3; ++k) {
        const double offset = raw + k * 2.0 * M_PI;
        const double dist   = std::abs(offset - curr);
        if (dist < best_dist && active_joints[i]->satisfiesPositionBounds(&offset)) {
          best_dist = dist;
          best_norm = offset;
        }
      }
      candidate[i] = best_norm;
    }

    // Reject candidates where wrist_1_joint is outside [-180°, +15°].
    static constexpr double kWrist1Min = -M_PI;
    static constexpr double kWrist1Max =  M_PI / 12.0;
    bool wrist1_in_range = true;
    double wrist1_val = 0.0;
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      if (joint_names[i] == "wrist_1_joint" && i < candidate.size()) {
        wrist1_val = candidate[i];
        if (wrist1_val < kWrist1Min || wrist1_val > kWrist1Max) {
          wrist1_in_range = false;
        }
        break;
      }
    }
    if (!wrist1_in_range) {
      ++ik_diag.wrist1_filter_rejections;
      log_rejection(attempt, "wrist_1 range filter",
        wrist1_val,
        get_joint_value(candidate, "wrist_2_joint"),
        get_joint_value(candidate, "wrist_3_joint"));
      return;
    }

    ++ik_diag.accepted_candidates;

    double movement_cost      = 0.0;
    double functional_penalty = 0.0;
    for (std::size_t i = 0; i < candidate.size(); ++i) {
      movement_cost += std::abs(candidate[i] - current_joint_values[i]);
      if (i < kFuncPos.size()) {
        const double d = candidate[i] - kFuncPos[i];
        functional_penalty += kFuncWeights[i] * d * d;
      }
    }
    const double cost = 2.0 * movement_cost + 0.3 * functional_penalty;

    if (cost < best_cost) {
      best_cost         = cost;
      best_joint_values = candidate;
    }
  };

  for (int attempt = 0; attempt < kTotalAttempts; ++attempt) {
    runIKAttempt(attempt);
  }

  if (best_joint_values.empty()) {
    RCLCPP_WARN(LOGGER,
      "computeBestIK: no valid solution after %d attempts — retrying with %d more random seeds.",
      kTotalAttempts, kTotalAttempts);
    for (int attempt = kTotalAttempts; attempt < kTotalAttempts * 2; ++attempt) {
      runIKAttempt(attempt);
    }
  }

  RCLCPP_INFO(LOGGER, "computeBestIK: %d IK solutions found. Best cost: %.4f",
    ik_successes, best_cost);

  if (best_joint_values.empty()) {
    RCLCPP_WARN(LOGGER,
      "computeBestIK diagnostics: no accepted candidate after %d attempt(s). "
      "solver_failures=%d, solver_successes=%d, accepted=%d, "
      "collision_callback_rejections=%d, collision_rejected_attempts=%d, "
      "wrist_3_filter_rejections=%d, wrist_1_filter_rejections=%d.",
      ik_diag.total_attempts, ik_diag.solver_failures, ik_successes,
      ik_diag.accepted_candidates, ik_diag.collision_callback_rejections,
      ik_diag.collision_rejected_attempts, ik_diag.wrist3_filter_rejections,
      ik_diag.wrist1_filter_rejections);
    RCLCPP_INFO(LOGGER,
      "computeBestIK: IK FAILED for target pose (x=%.3f y=%.3f z=%.3f qx=%.3f qy=%.3f qz=%.3f qw=%.3f).",
      target_pose.position.x, target_pose.position.y, target_pose.position.z,
      target_pose.orientation.x, target_pose.orientation.y,
      target_pose.orientation.z, target_pose.orientation.w);
    RCLCPP_INFO(LOGGER, "computeBestIK: start joint state at time of failure:");
    for (std::size_t i = 0; i < current_joint_values.size() && i < joint_names.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
        joint_names[i].c_str(), current_joint_values[i],
        current_joint_values[i] * 180.0 / M_PI);
    }
    RCLCPP_ERROR(LOGGER,
      "computeBestIK: no valid IK solution found for pose (x=%.3f y=%.3f z=%.3f).",
      target_pose.position.x, target_pose.position.y, target_pose.position.z);
  } else {
    RCLCPP_INFO(LOGGER, "computeBestIK: goal joint state selected:");
    for (std::size_t i = 0; i < best_joint_values.size() && i < joint_names.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
        joint_names[i].c_str(), best_joint_values[i],
        best_joint_values[i] * 180.0 / M_PI);
    }
  }
  return best_joint_values;
}

std::vector<moveit::planning_interface::MoveGroupInterface::Plan>
WordleBotController::generateCandidatePlans(int num_attempts)
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
      plans.push_back(plan);
    } else {
      RCLCPP_WARN(LOGGER, "generateCandidatePlans: attempt %d failed (error code %d).",
        attempt, result.val);
    }
  }
  RCLCPP_INFO(LOGGER, "generateCandidatePlans: %d/%d plans succeeded.", successes, num_attempts);
  return plans;
}

moveit::planning_interface::MoveGroupInterface::Plan
WordleBotController::selectBestPlan(
  const std::vector<moveit::planning_interface::MoveGroupInterface::Plan> & plans)
{
  moveit::planning_interface::MoveGroupInterface::Plan best_plan;
  double best_cost = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < plans.size(); ++i) {
    const double cost = computeTotalJointDisplacement(plans[i]);
    RCLCPP_INFO(LOGGER, "selectBestPlan: plan %zu — cost=%.4f rad %s",
      i, cost, cost < best_cost ? "(best so far)" : "(worse)");
    if (cost < best_cost) {
      best_cost = cost;
      best_plan = plans[i];
    }
  }
  RCLCPP_INFO(LOGGER, "selectBestPlan: best cost=%.4f rad total joint displacement.", best_cost);
  return best_plan;
}

bool WordleBotController::moveToGoal(const geometry_msgs::msg::Pose & goal_pose)
{
  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "moveToGoal: stop requested — aborting before start.");
    return false;
  }

  {
    const double s = computeTransitScaling(queryCurrentStateMinDistance());
    move_group_.setMaxVelocityScalingFactor(s);
    move_group_.setMaxAccelerationScalingFactor(s);
    RCLCPP_DEBUG(LOGGER, "moveToGoal: transit velocity scaling = %.2f.", s);
  }

  RCLCPP_INFO(LOGGER,
    "moveToGoal: target pos (x=%.3f y=%.3f z=%.3f) quat (x=%.3f y=%.3f z=%.3f w=%.3f).",
    goal_pose.position.x, goal_pose.position.y, goal_pose.position.z,
    goal_pose.orientation.x, goal_pose.orientation.y,
    goal_pose.orientation.z, goal_pose.orientation.w);

  move_group_.setStartStateToCurrentState();
  current_state = move_group_.getCurrentState(2.0);
  if (!current_state) {
    RCLCPP_ERROR(LOGGER, "moveToGoal: getCurrentState returned null — state monitor not ready.");
    return false;
  }

  const std::string arm_group = "ur_onrobot_manipulator";
  const auto * jmg = move_group_.getRobotModel()->getJointModelGroup(arm_group);

  // Log current joint state for diagnostics.
  if (jmg) {
    std::vector<double> q_start;
    current_state->copyJointGroupPositions(jmg, q_start);
    const auto & jnames = jmg->getVariableNames();
    RCLCPP_INFO(LOGGER, "moveToGoal: current joint state (%zu joints):", q_start.size());
    for (std::size_t i = 0; i < q_start.size() && i < jnames.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
        jnames[i].c_str(), q_start[i], q_start[i] * 180.0 / M_PI);
    }
  }

  std::vector<double> best_q = computeBestIK(current_state, goal_pose);

  if (best_q.empty() && jmg) {
    // All IK attempts failed. Negate wrist_3_joint and move there, then retry.
    RCLCPP_WARN(LOGGER, "moveToGoal: IK failed — attempting wrist_3 flip recovery.");

    std::vector<double> flipped_q;
    current_state->copyJointGroupPositions(jmg, flipped_q);
    const auto & flip_jnames = jmg->getVariableNames();
    for (std::size_t i = 0; i < flipped_q.size() && i < flip_jnames.size(); ++i) {
      if (flip_jnames[i] == "wrist_3_joint") {
        double & v = flipped_q[i];
        const double before = v;
        while (v >  M_PI) v -= 2.0 * M_PI;
        while (v < -M_PI) v += 2.0 * M_PI;
        RCLCPP_INFO(LOGGER, "moveToGoal: normalising wrist_3: %.3f rad → %.3f rad.",
          before, v);
        break;
      }
    }

    move_group_.setStartStateToCurrentState();
    move_group_.setJointValueTarget(flipped_q);
    moveit::planning_interface::MoveGroupInterface::Plan flip_plan;
    if (static_cast<bool>(move_group_.plan(flip_plan)) &&
        alignWrist3TrajectoryToCurrentState(flip_plan.trajectory_, "moveToGoal flip recovery") &&
        move_group_.execute(flip_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      current_state = move_group_.getCurrentState(2.0);
      if (current_state) {
        best_q = computeBestIK(current_state, goal_pose);
      }
    } else {
      RCLCPP_ERROR(LOGGER, "moveToGoal: wrist_3 flip move failed.");
    }
  }

  if (best_q.empty()) {
    RCLCPP_ERROR(LOGGER, "moveToGoal: IK failed — no valid solution found.");
    return false;
  }

  // Log target joint values for diagnostics.
  if (jmg) {
    const auto & jnames = jmg->getVariableNames();
    RCLCPP_INFO(LOGGER, "moveToGoal: target joint values (%zu joints):", best_q.size());
    for (std::size_t i = 0; i < best_q.size() && i < jnames.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
        jnames[i].c_str(), best_q[i], best_q[i] * 180.0 / M_PI);
    }
  }

  // Seed the planner from the same state snapshot used for IK to avoid a race
  // where the robot drifts slightly between getCurrentState and plan().
  move_group_.setStartState(*current_state);
  move_group_.setJointValueTarget(best_q);

  const auto plans = generateCandidatePlans(5);
  if (plans.empty()) {
    RCLCPP_ERROR(LOGGER, "moveToGoal: all 5 planning attempts failed.");
    return false;
  }

  auto plan = selectBestPlan(plans);
  if (plan.trajectory_.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(LOGGER, "moveToGoal: selectBestPlan returned an empty plan.");
    return false;
  }

  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "moveToGoal: stop requested — aborting before execute.");
    return false;
  }

  RCLCPP_INFO(LOGGER, "moveToGoal: executing plan (total_joint_disp=%.4f rad).",
    computeTotalJointDisplacement(plan));

  if (!alignWrist3TrajectoryToCurrentState(plan.trajectory_, "moveToGoal")) {
    return false;
  }
  const auto exec_result = move_group_.execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "moveToGoal: execution failed (error code %d).", exec_result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "moveToGoal: goal reached.");
  return true;
}

// ---------------------------------------------------------------------------
// Scan and Sweep
// Four-pose camera scan sequence.
// MGI path (USE_MTC_FOR_SCAN_SWEEP == false):
//   Pose 0 via moveToGoal; poses 1-3 via computeCartesianPath with moveToGoal
//   fallback; then returnToHome.
// MTC path (USE_MTC_FOR_SCAN_SWEEP == true):
//   Pose 1 via free-space OMPL (Connect + ComputeIK); poses 2-4 via straight-
//   line Cartesian sweeps (MoveRelative).
// ---------------------------------------------------------------------------
// moveCartesianToWaypoint — single Cartesian segment with moveToGoal fallback
// createScanAndSweepTask  — build the unified MTC task
// runScanAndSweep         — execute the full four-pose scan sequence
// ---------------------------------------------------------------------------

bool WordleBotController::moveCartesianToWaypoint(
  const geometry_msgs::msg::Pose & target_pose)
{
  move_group_.setMaxVelocityScalingFactor(vel_profiles_.scan_vel);
  move_group_.setMaxAccelerationScalingFactor(vel_profiles_.scan_acc);

  static constexpr double kStepSizes[] = {kCartesianEefStep, 0.03, 0.05};

  for (const double step : kStepSizes) {
    move_group_.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group_.computeCartesianPath(
      {target_pose}, step, kCartesianJumpThreshold, trajectory);

    RCLCPP_INFO(LOGGER,
      "moveCartesianToWaypoint: step=%.3f  fraction=%.3f  target xyz=(%.3f, %.3f, %.3f)",
      step, fraction,
      target_pose.position.x, target_pose.position.y, target_pose.position.z);

    if (fraction >= kCartesianMinFraction) {
      if (!alignWrist3TrajectoryToCurrentState(trajectory, "moveCartesianToWaypoint")) {
        return false;
      }
      const auto result = move_group_.execute(trajectory);
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(LOGGER,
          "moveCartesianToWaypoint: execution failed (%d) — falling back to moveToGoal.",
          result.val);
        return moveToGoal(target_pose);
      }
      return true;
    }

    RCLCPP_WARN(LOGGER,
      "moveCartesianToWaypoint: step=%.3f fraction=%.3f < %.3f — trying next step size.",
      step, fraction, kCartesianMinFraction);
  }

  RCLCPP_WARN(LOGGER,
    "moveCartesianToWaypoint: all step sizes failed — falling back to moveToGoal.");
  return moveToGoal(target_pose);
}

mtc::Task WordleBotController::createScanAndSweepTask(
  const std::vector<geometry_msgs::msg::Pose> & scan_poses,
  const planning_scene::PlanningScenePtr & start_scene)
{
  const std::string arm_group = "ur_onrobot_manipulator";
  const std::string hand_group = "ur_onrobot_gripper";
  const std::string hand_frame = "gripper_tcp";

  RCLCPP_INFO(LOGGER, "createScanAndSweepTask: scan pose 1 target  xyz=(%.3f, %.3f, %.3f)",
    scan_poses[0].position.x, scan_poses[0].position.y, scan_poses[0].position.z);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(vel_profiles_.scan_vel);
  cartesian_planner->setMaxAccelerationScalingFactor(vel_profiles_.scan_acc);
  cartesian_planner->setStepSize(0.001);

  mtc::Task task;
  task.stages()->setName("scan and sweep");
  task.loadRobotModel(node_);
  task.setProperty("group",    arm_group);
  task.setProperty("eef",      hand_group);
  task.setProperty("ik_frame", hand_frame);

  // Stage 1: current robot state (or chained fixed state).
  mtc::Stage * current_state_ptr = nullptr;
  {
    if (start_scene == nullptr) {
      auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state_ptr = stage.get();
      task.add(std::move(stage));
    } else {
      auto stage = std::make_unique<mtc::stages::FixedState>("fixed start");
      stage->setState(start_scene);
      current_state_ptr = stage.get();
      task.add(std::move(stage));
    }
  }

  // Stage 3: free-space OMPL move toward scan pose 1 — mirrors "move to pick" in pick-and-place.
  {
    auto connect = std::make_unique<mtc::stages::Connect>(
      "move to scan 1",
      mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
    connect->setTimeout(0.2);
    connect->properties().configureInitFrom(mtc::Stage::PARENT);
    // connect->setPathConstraints(buildJointLimitConstraints());
    task.add(std::move(connect));
  }

  // Stage 3: IK resolution for scan pose 1.
  {
    auto gen_pose = std::make_unique<mtc::stages::GeneratePose>("generate scan pose 1");
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "world";
    target.pose = scan_poses[0];
    gen_pose->setPose(target);
    gen_pose->setMonitoredStage(current_state_ptr);

    auto ik = std::make_unique<mtc::stages::ComputeIK>("scan pose 1 IK", std::move(gen_pose));
    ik->setMaxIKSolutions(32);
    ik->setMinSolutionDistance(0.1);
    ik->setIKFrame(hand_frame);
    ik->setProperty("default_pose", std::string("test_configuration"));
    ik->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "eef"});
    ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    task.add(std::move(ik));
  }

  // Stages 4-6: straight-line Cartesian sweep between scan poses 1→2→3→4.
  // MoveRelative moves the IK frame along the world-frame delta between consecutive poses.
  for (size_t i = 1; i < scan_poses.size(); ++i) {
    const auto & from = scan_poses[i - 1];
    const auto & to   = scan_poses[i];

    const double dx   = to.position.x - from.position.x;
    const double dy   = to.position.y - from.position.y;
    const double dz   = to.position.z - from.position.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    RCLCPP_INFO(LOGGER,
      "createScanAndSweepTask: sweep %zu→%zu  delta=(%.3f, %.3f, %.3f)  dist=%.4f m",
      i, i + 1, dx, dy, dz, dist);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.x = dx / dist;
    vec.vector.y = dy / dist;
    vec.vector.z = dz / dist;

    auto stage = std::make_unique<mtc::stages::MoveRelative>(
      "move to scan " + std::to_string(i + 1), cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setIKFrame(hand_frame);
    stage->setDirection(vec);
    stage->setMinMaxDistance(dist * 0.95, dist * 1.05);
    task.add(std::move(stage));
  }

  return task;
}

bool WordleBotController::runScanAndSweep(
  const std::vector<geometry_msgs::msg::Pose> & poses, double dwell_secs)
{
  if (poses.size() != 6) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: expected 6 poses, got %zu.", poses.size());
    return false;
  }

  RCLCPP_INFO(LOGGER, "runScanAndSweep: starting.");
  for (size_t i = 0; i < poses.size(); ++i) {
    RCLCPP_INFO(LOGGER,
      "runScanAndSweep: pose %zu  xyz=(%.3f, %.3f, %.3f)  quat=(%.3f, %.3f, %.3f, %.3f)",
      i,
      poses[i].position.x,    poses[i].position.y,    poses[i].position.z,
      poses[i].orientation.x, poses[i].orientation.y,
      poses[i].orientation.z, poses[i].orientation.w);
  }

  if constexpr (USE_MTC_FOR_SCAN_SWEEP) {
    // ── MTC path — uses poses[0..3] only (original 4-pose behaviour) ──────
    if (stop_requested_.load()) { return false; }
    RCLCPP_INFO(LOGGER, "runScanAndSweep (MTC): building scan task (poses 0-3).");
    const std::vector<geometry_msgs::msg::Pose> mtc_poses(poses.begin(), poses.begin() + 4);
    auto sweep_task = createScanAndSweepTask(mtc_poses, nullptr);

    try {
      sweep_task.init();
    } catch (const mtc::InitStageException & e) {
      RCLCPP_ERROR_STREAM(LOGGER, "runScanAndSweep (MTC): sweep task init failed: " << e);
      return false;
    }

    RCLCPP_INFO(LOGGER, "runScanAndSweep (MTC): planning sweep.");
    const int mtc_solution_target_count =
      std::max(1, getIntParam(node_, "mtc_default_solution_target_count", 15));
    moveit::core::MoveItErrorCode plan_result;
    try {
      plan_result = sweep_task.plan(mtc_solution_target_count);
    } catch (const mtc::InitStageException & e) {
      RCLCPP_ERROR_STREAM(LOGGER, "runScanAndSweep (MTC): sweep task planning threw: " << e);
      return false;
    }

    if (!plan_result || sweep_task.solutions().empty()) {
      RCLCPP_ERROR(LOGGER, "runScanAndSweep (MTC): planning failed — no solutions found.");
      RCLCPP_ERROR(LOGGER, "runScanAndSweep (MTC): check above MTC stage tree for the failing stage.");
      return false;
    }

    RCLCPP_INFO(LOGGER,
      "runScanAndSweep (MTC): planned — %zu solution(s), best cost=%.3f. Executing.",
      sweep_task.solutions().size(), sweep_task.solutions().front()->cost());

    sweep_task.introspection().publishSolution(*sweep_task.solutions().front());
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    if (stop_requested_.load()) { return false; }
    const auto exec_result = executeAlignedTaskSolution(
      sweep_task, *sweep_task.solutions().front(), "runScanAndSweep (MTC)");
    if (exec_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "runScanAndSweep (MTC): sweep execution failed (error code %d).",
        exec_result.val);
      return false;
    }

    RCLCPP_INFO(LOGGER, "runScanAndSweep (MTC): returning to working pose.");
    returnToWorkingPose();
    RCLCPP_INFO(LOGGER, "runScanAndSweep (MTC): complete.");
    return true;

  } else {
    // ── MGI path ───────────────────────────────────────────────────────────
    // Sequence:
    //   moveToGoal(0)          — free-space move to start
    //   Cartesian → (1)        — first sweep
    //   moveToGoal(2)          — reorient in place (same XYZ as 1, yaw -90°)
    //   Cartesian → (3)        — second sweep
    //   moveToGoal(4)          — reorient in place (same XYZ as 3, yaw -90°)
    //   Cartesian → (5)        — third sweep
    //   returnToHome()
    // Dwell is applied after each Cartesian arrival; not at reorientation poses.

    auto dwell = [&](size_t idx) {
      if (dwell_secs > 0.0) {
        RCLCPP_INFO(LOGGER, "runScanAndSweep (MGI): dwelling %.2f s at pose %zu.", dwell_secs, idx);
        rclcpp::sleep_for(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(dwell_secs)));
      }
    };

    if (stop_requested_.load()) { return false; }

    RCLCPP_INFO(LOGGER, "runScanAndSweep (MGI): moveToGoal pose 0 (start).");
    if (!moveToGoal(poses[0])) {
      RCLCPP_ERROR(LOGGER, "runScanAndSweep (MGI): failed to reach pose 0.");
      return false;
    }
    dwell(0);

    if (stop_requested_.load()) { return false; }
    RCLCPP_INFO(LOGGER, "runScanAndSweep (MGI): Cartesian pose 0 → 1.");
    if (!moveCartesianToWaypoint(poses[1])) {
      RCLCPP_ERROR(LOGGER, "runScanAndSweep (MGI): failed to reach pose 1.");
      return false;
    }
    dwell(1);

    if (stop_requested_.load()) { return false; }
    RCLCPP_INFO(LOGGER, "runScanAndSweep (MGI): moveToGoal pose 2 (reorient -90° yaw).");
    if (!moveToGoal(poses[2])) {
      RCLCPP_ERROR(LOGGER, "runScanAndSweep (MGI): failed to reach pose 2.");
      return false;
    }

    if (stop_requested_.load()) { return false; }
    RCLCPP_INFO(LOGGER, "runScanAndSweep (MGI): Cartesian pose 2 → 3.");
    if (!moveCartesianToWaypoint(poses[3])) {
      RCLCPP_ERROR(LOGGER, "runScanAndSweep (MGI): failed to reach pose 3.");
      return false;
    }
    dwell(3);

    if (stop_requested_.load()) { return false; }
    RCLCPP_INFO(LOGGER, "runScanAndSweep (MGI): moveToGoal pose 4 (reorient -90° yaw).");
    if (!moveToGoal(poses[4])) {
      RCLCPP_ERROR(LOGGER, "runScanAndSweep (MGI): failed to reach pose 4.");
      return false;
    }

    if (stop_requested_.load()) { return false; }
    RCLCPP_INFO(LOGGER, "runScanAndSweep (MGI): Cartesian pose 4 → 5.");
    if (!moveCartesianToWaypoint(poses[5])) {
      RCLCPP_ERROR(LOGGER, "runScanAndSweep (MGI): failed to reach pose 5.");
      return false;
    }
    dwell(5);

    RCLCPP_INFO(LOGGER, "runScanAndSweep (MGI): returning to working pose.");
    returnToWorkingPose();
    RCLCPP_INFO(LOGGER, "runScanAndSweep (MGI): complete.");
    return true;
  }
}

// ---------------------------------------------------------------------------
// Standalone Arm Motions
// Simple one-shot MTC tasks for moving the arm to known named states.
// These are available while no mission is running (IDLE state).
// ---------------------------------------------------------------------------
// returnToHome  — move the arm to the SRDF "home" named state
// openGripper   — move the gripper to the SRDF "open" named state
// closeGripper  — move the gripper to the SRDF "closed" named state
// ---------------------------------------------------------------------------

bool WordleBotController::returnToHome()
{
  RCLCPP_INFO(LOGGER, "returnToHome: building MTC task.");

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(vel_profiles_.transit_vel);
  interpolation_planner->setMaxAccelerationScalingFactor(vel_profiles_.transit_acc);

  mtc::Task task;
  task.stages()->setName("return home");
  task.loadRobotModel(node_);
  task.setProperty("group", std::string("ur_onrobot_manipulator"));

  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
  stage->setGoal("home");
  task.add(std::move(stage));

  try {
    task.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "returnToHome: task init failed: " << e);
    return false;
  }

  const int mtc_solution_target_count =
    std::max(1, getIntParam(node_, "mtc_default_solution_target_count", 15));
  if (!task.plan(mtc_solution_target_count) || task.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "returnToHome: planning failed — no solutions found.");
    return false;
  }

  task.introspection().publishSolution(*task.solutions().front());
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  auto result = executeAlignedTaskSolution(task, *task.solutions().front(), "returnToHome");
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "returnToHome: execution failed (error code %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "returnToHome: succeeded.");
  return true;
}

bool WordleBotController::returnToWorkingPose()
{
  RCLCPP_INFO(LOGGER, "returnToWorkingPose: moving to working pose.");

  std::map<std::string, double> joints = {
    {"shoulder_pan_joint",  node_->get_parameter("working_joints.shoulder_pan_joint").as_double()},
    {"shoulder_lift_joint", node_->get_parameter("working_joints.shoulder_lift_joint").as_double()},
    {"elbow_joint",         node_->get_parameter("working_joints.elbow_joint").as_double()},
    {"wrist_1_joint",       node_->get_parameter("working_joints.wrist_1_joint").as_double()},
    {"wrist_2_joint",       node_->get_parameter("working_joints.wrist_2_joint").as_double()},
    {"wrist_3_joint",       node_->get_parameter("working_joints.wrist_3_joint").as_double()},
  };

  {
    const double s = computeTransitScaling(queryCurrentStateMinDistance());
    move_group_.setMaxVelocityScalingFactor(s);
    move_group_.setMaxAccelerationScalingFactor(s);
    RCLCPP_DEBUG(LOGGER, "returnToWorkingPose: transit velocity scaling = %.2f.", s);
  }
  move_group_.setStartStateToCurrentState();
  move_group_.setJointValueTarget(joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "returnToWorkingPose: planning failed.");
    return false;
  }

  if (!alignWrist3TrajectoryToCurrentState(plan.trajectory_, "returnToWorkingPose")) {
    return false;
  }
  auto result = move_group_.execute(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "returnToWorkingPose: execution failed (error code %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "returnToWorkingPose: succeeded.");
  return true;
}

bool WordleBotController::openGripper()
{
  RCLCPP_INFO(LOGGER, "openGripper: building MTC task.");

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  mtc::Task task;
  task.stages()->setName("open gripper");
  task.loadRobotModel(node_);

  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage->setGroup("ur_onrobot_gripper");
  stage->setGoal("open");
  task.add(std::move(stage));

  try {
    task.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "openGripper: task init failed: " << e);
    return false;
  }

  const int mtc_solution_target_count =
    std::max(1, getIntParam(node_, "mtc_default_solution_target_count", 15));
  if (!task.plan(mtc_solution_target_count) || task.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "openGripper: planning failed — no solutions found.");
    return false;
  }

  task.introspection().publishSolution(*task.solutions().front());
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "openGripper: execution failed (error code %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "openGripper: succeeded.");
  return true;
}

bool WordleBotController::closeGripper()
{
  RCLCPP_INFO(LOGGER, "closeGripper: building MTC task.");

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  mtc::Task task;
  task.stages()->setName("close gripper");
  task.loadRobotModel(node_);

  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage->setGroup("ur_onrobot_gripper");
  stage->setGoal("closed");
  task.add(std::move(stage));

  try {
    task.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "closeGripper: task init failed: " << e);
    return false;
  }

  const int mtc_solution_target_count =
    std::max(1, getIntParam(node_, "mtc_default_solution_target_count", 15));
  if (!task.plan(mtc_solution_target_count) || task.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "closeGripper: planning failed — no solutions found.");
    return false;
  }

  task.introspection().publishSolution(*task.solutions().front());
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "closeGripper: execution failed (error code %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "closeGripper: succeeded.");
  return true;
}

bool WordleBotController::isGripperClosed()
{
  auto state = move_group_.getCurrentState(2.0);
  if (!state) {
    RCLCPP_ERROR(LOGGER, "isGripperClosed: getCurrentState returned null — assuming open.");
    return false;
  }

  const moveit::core::JointModelGroup * jmg =
    state->getRobotModel()->getJointModelGroup("ur_onrobot_gripper");
  if (!jmg) {
    RCLCPP_ERROR(LOGGER, "isGripperClosed: joint group 'ur_onrobot_gripper' not found — assuming open.");
    return false;
  }

  std::vector<double> current_vals;
  state->copyJointGroupPositions(jmg, current_vals);

  moveit::core::RobotState open_state(*state);
  moveit::core::RobotState closed_state(*state);
  open_state.setToDefaultValues(jmg, "open");
  closed_state.setToDefaultValues(jmg, "closed");

  std::vector<double> open_vals, closed_vals;
  open_state.copyJointGroupPositions(jmg, open_vals);
  closed_state.copyJointGroupPositions(jmg, closed_vals);

  double dist_open = 0.0, dist_closed = 0.0;
  for (std::size_t i = 0; i < current_vals.size(); ++i) {
    double do_i = current_vals[i] - open_vals[i];
    double dc_i = current_vals[i] - closed_vals[i];
    dist_open   += do_i * do_i;
    dist_closed += dc_i * dc_i;
  }

  bool closed = (dist_closed < dist_open);
  RCLCPP_INFO(LOGGER, "isGripperClosed: dist_closed=%.4f dist_open=%.4f → %s.",
    dist_closed, dist_open, closed ? "CLOSED" : "OPEN");
  return closed;
}

bool WordleBotController::recoverObject(const std::string & held_object_id)
{
  RCLCPP_INFO(LOGGER, "recoverObject: beginning object recovery to safe position (0.15, 0.15, 0.03).");

  // Preserve current gripper orientation so the arm doesn't flip the wrist mid-recovery.
  auto eef_stamped = move_group_.getCurrentPose("gripper_tcp");
  // const auto & orient = eef_stamped.pose.orientation;

  geometry_msgs::msg::Pose approach_pose;
  approach_pose.position.x    = 0.15;
  approach_pose.position.y    = 0.15;
  approach_pose.position.z    = 0.15;
  approach_pose.orientation.x   = 1.0;
  approach_pose.orientation.y   = 0.0;
  approach_pose.orientation.z   = 0.0;
  approach_pose.orientation.w   = 0.0;

  geometry_msgs::msg::Pose safe_pose;
  safe_pose.position.x    = 0.15;
  safe_pose.position.y    = 0.15;
  safe_pose.position.z    = 0.03;
  safe_pose.orientation.x   = 1.0;
  safe_pose.orientation.y   = 0.0;
  safe_pose.orientation.z   = 0.0;
  safe_pose.orientation.w   = 0.0;

  bool ok = true;

  RCLCPP_INFO(LOGGER, "recoverObject: moving to approach pose above safe position.");
  if (!moveToGoal(approach_pose)) {
    RCLCPP_ERROR(LOGGER, "recoverObject: moveToGoal to approach failed — continuing.");
    ok = false;
  }

  if (ok) {
    RCLCPP_INFO(LOGGER, "recoverObject: Cartesian descent to safe position.");
    if (!moveCartesianToWaypoint(safe_pose)) {
      RCLCPP_ERROR(LOGGER, "recoverObject: Cartesian descent failed — continuing.");
      ok = false;
    }
  }

  // Detach the held object at the safe position before opening the gripper.
  // Mirrors detachSensorCollisionObject() — REMOVE moves the object back to the world scene
  // at its current (safe) position rather than leaving it fused to the end effector.
  if (!held_object_id.empty()) {
    moveit_msgs::msg::AttachedCollisionObject detach;
    detach.link_name = "gripper_tcp";
    detach.object.id = held_object_id;
    detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_.applyAttachedCollisionObject(detach);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO(LOGGER, "recoverObject: detached '%s' from gripper_tcp at safe position.",
      held_object_id.c_str());
  }

  RCLCPP_INFO(LOGGER, "recoverObject: opening gripper.");
  if (!openGripper()) {
    RCLCPP_ERROR(LOGGER, "recoverObject: openGripper failed — continuing.");
    ok = false;
  }

  if (ok) {
    RCLCPP_INFO(LOGGER, "recoverObject: Cartesian ascent from safe position.");
    if (!moveCartesianToWaypoint(approach_pose)) {
      RCLCPP_ERROR(LOGGER, "recoverObject: Cartesian ascent failed — continuing.");
    }
  }

  RCLCPP_INFO(LOGGER, "recoverObject: returning to working pose.");
  if (!returnToWorkingPose()) {
    RCLCPP_ERROR(LOGGER, "recoverObject: returnToWorkingPose failed.");
    return false;
  }

  RCLCPP_INFO(LOGGER, "recoverObject: complete.");
  return true;
}

// ---------------------------------------------------------------------------
// Motion Control
// Interrupt and reset helpers for the stop/resume lifecycle.
// ---------------------------------------------------------------------------
// stop          — cancel the in-progress trajectory and set the stop flag;
//                 any blocking execute() call will return a non-SUCCESS code
// clearStopFlag — clear the stop flag before issuing a new motion so the
//                 motion is not immediately rejected
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

// ---------------------------------------------------------------------------
// Helper Functions
// Stateless utility functions used across planning and scene management.
// ---------------------------------------------------------------------------
// buildPose                    — construct a geometry_msgs::Pose from XYZ
//                                position and RPY orientation
// computeTotalJointDisplacement — compute Σ|Δq| across all joints and
//                                 trajectory steps (L1 joint-space path length)
// buildPathConstraints         — return the MoveIt path constraints used by
//                                MTC planning stages
// ---------------------------------------------------------------------------

bool WordleBotController::alignWrist3TrajectoryToCurrentState(
  moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::string & context)
{
  if (trajectory.joint_trajectory.points.empty()) {
    return true;
  }

  const auto wrist3_index =
    findJointIndex(trajectory.joint_trajectory.joint_names, kWrist3JointName);
  if (!wrist3_index) {
    return true;
  }

  auto state = move_group_.getCurrentState(2.0);
  if (!state) {
    RCLCPP_ERROR(LOGGER,
      "%s: cannot align wrist_3 trajectory because getCurrentState() returned null.",
      context.c_str());
    return false;
  }

  const auto * jmg = move_group_.getRobotModel()->getJointModelGroup("ur_onrobot_manipulator");
  if (jmg == nullptr) {
    RCLCPP_ERROR(LOGGER,
      "%s: cannot align wrist_3 trajectory because group 'ur_onrobot_manipulator' was not found.",
      context.c_str());
    return false;
  }

  std::vector<double> joint_values;
  state->copyJointGroupPositions(jmg, joint_values);
  const auto current_index = findJointIndex(jmg->getVariableNames(), kWrist3JointName);
  if (!current_index || joint_values.size() <= *current_index) {
    RCLCPP_ERROR(LOGGER,
      "%s: cannot align wrist_3 trajectory because the live state has no wrist_3 value.",
      context.c_str());
    return false;
  }

  double reference_wrist3 = joint_values[*current_index];
  alignWrist3JointTrajectoryToReference(
    trajectory.joint_trajectory, reference_wrist3, context);
  return true;
}

moveit::core::MoveItErrorCode WordleBotController::executeAlignedTaskSolution(
  mtc::Task & task,
  const mtc::SolutionBase & solution,
  const std::string & context)
{
  using ExecuteTaskSolutionAction =
    moveit_task_constructor_msgs::action::ExecuteTaskSolution;

  moveit_msgs::msg::MoveItErrorCodes error_code;
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;

  auto state = move_group_.getCurrentState(2.0);
  if (!state) {
    RCLCPP_ERROR(LOGGER,
      "%s: cannot execute aligned MTC solution because getCurrentState() returned null.",
      context.c_str());
    return error_code;
  }

  const auto * jmg = move_group_.getRobotModel()->getJointModelGroup("ur_onrobot_manipulator");
  if (jmg == nullptr) {
    RCLCPP_ERROR(LOGGER,
      "%s: cannot execute aligned MTC solution because group 'ur_onrobot_manipulator' was not found.",
      context.c_str());
    return error_code;
  }

  std::vector<double> joint_values;
  state->copyJointGroupPositions(jmg, joint_values);
  const auto current_index = findJointIndex(jmg->getVariableNames(), kWrist3JointName);
  if (!current_index || joint_values.size() <= *current_index) {
    RCLCPP_ERROR(LOGGER,
      "%s: cannot execute aligned MTC solution because the live state has no wrist_3 value.",
      context.c_str());
    return error_code;
  }

  ExecuteTaskSolutionAction::Goal goal;
  solution.toMsg(goal.solution, &task.introspection());

  for (std::size_t i = 0; i < goal.solution.sub_trajectory.size(); ++i) {
    const auto & joint_trajectory = goal.solution.sub_trajectory[i].trajectory.joint_trajectory;
    if (joint_trajectory.points.empty()) {
      continue;
    }

    const auto & first = joint_trajectory.points.front().time_from_start;
    const auto & last = joint_trajectory.points.back().time_from_start;
    RCLCPP_DEBUG(LOGGER,
      "%s: subtrajectory %zu stage_id=%u joints=%zu points=%zu duration=%.6f s first_time=%.6f s.",
      context.c_str(), i + 1,
      goal.solution.sub_trajectory[i].info.stage_id,
      joint_trajectory.joint_names.size(),
      joint_trajectory.points.size(),
      rclcpp::Duration(last).seconds(),
      rclcpp::Duration(first).seconds());
  }

  double reference_wrist3 = joint_values[*current_index];
  std::size_t aligned_subtrajectories = 0;
  for (std::size_t i = 0; i < goal.solution.sub_trajectory.size(); ++i) {
    auto & sub_trajectory = goal.solution.sub_trajectory[i].trajectory;
    if (alignWrist3JointTrajectoryToReference(
        sub_trajectory.joint_trajectory, reference_wrist3,
        context + " subtrajectory " + std::to_string(i + 1))) {
      ++aligned_subtrajectories;
    }
  }
  RCLCPP_DEBUG(LOGGER, "%s: checked %zu MTC subtrajectory(s), aligned %zu containing wrist_3.",
    context.c_str(), goal.solution.sub_trajectory.size(), aligned_subtrajectories);

  auto execute_node = rclcpp::Node::make_shared("wordlebot_mtc_executor");
  auto execute_client =
    rclcpp_action::create_client<ExecuteTaskSolutionAction>(execute_node, "execute_task_solution");

  if (!execute_client->wait_for_action_server(std::chrono::milliseconds(500))) {
    RCLCPP_ERROR(LOGGER, "%s: failed to connect to the 'execute_task_solution' action server.",
      context.c_str());
    return error_code;
  }

  auto goal_handle_future = execute_client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(execute_node, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "%s: send goal call failed.", context.c_str());
    return error_code;
  }

  const auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(LOGGER, "%s: execute_task_solution goal was rejected.", context.c_str());
    return error_code;
  }

  auto result_future = execute_client->async_get_result(goal_handle);
  while (result_future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    if (stop_requested_.load()) {
      auto cancel_future = execute_client->async_cancel_goal(goal_handle);
      if (rclcpp::spin_until_future_complete(execute_node, cancel_future) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "%s: could not cancel MTC execution.", context.c_str());
      }
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
      return error_code;
    }
    rclcpp::spin_some(execute_node);
  }

  const auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    if (result.result) {
      error_code = result.result->error_code;
    }
    RCLCPP_ERROR(LOGGER,
      "%s: execute_task_solution goal was aborted or canceled (action_result=%d, moveit_error=%d).",
      context.c_str(), static_cast<int>(result.code), error_code.val);
    return error_code;
  }

  return result.result->error_code;
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

moveit_msgs::msg::Constraints WordleBotController::buildPathConstraints()
{
  moveit_msgs::msg::Constraints constraints;

  // Keep the gripper facing straight down throughout all transit moves.
  // This shrinks OMPL's search space significantly, preventing tumbling and
  // reducing joint excursions on free-space Connect/MoveTo stages.
  // Tolerances allow small tilts (±0.4 rad ≈ ±23°) but keep yaw free (±π).
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header.frame_id = "world";
  oc.link_name = "gripper_tcp";
  oc.orientation.x = 1.0;        // roll=π → gripper Z-axis points straight down
  oc.orientation.y = 0.0;
  oc.orientation.z = 0.0;
  oc.orientation.w = 0.0;
  oc.absolute_x_axis_tolerance = 0.4;
  oc.absolute_y_axis_tolerance = 0.4;
  oc.absolute_z_axis_tolerance = M_PI;
  oc.weight = 1.0;

  constraints.orientation_constraints.push_back(oc);
  return constraints;
}

moveit_msgs::msg::Constraints WordleBotController::buildJointLimitConstraints()
{
  moveit_msgs::msg::Constraints c;
  for (const char * name : {"wrist_2_joint", "wrist_3_joint"}) {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name      = name;
    jc.position        = 0.0;
    jc.tolerance_above = M_PI;
    jc.tolerance_below = M_PI;
    jc.weight          = 1.0;
    c.joint_constraints.push_back(jc);
  }
  return c;
}
