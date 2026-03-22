#include "wordleBot_control/wordle_bot_controller.hpp"

#include <chrono>
#include <cmath>

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
  move_group_(node, "ur_manipulator"),
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
constexpr char kPlanningGroup[] = "ur_manipulator";

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
  if (joint_model_group == nullptr || current_state == nullptr) {
    return {};
  }

  std::vector<double> current_joint_values;
  current_state->copyJointGroupPositions(joint_model_group, current_joint_values);

  std::vector<double> best_joint_values;
  double best_cost = std::numeric_limits<double>::infinity();

  for (int attempt = 0; attempt < 15; ++attempt) {
    moveit::core::RobotState ik_state(*current_state);
    ik_state.setToRandomPositions(joint_model_group);

    if (!ik_state.setFromIK(joint_model_group, target_pose, 0.1)) {
      continue;
    }

    std::vector<double> candidate_joint_values;
    ik_state.copyJointGroupPositions(joint_model_group, candidate_joint_values);

    const double candidate_cost = configurationDistance(
      joint_model_group, current_joint_values, candidate_joint_values);

    if (candidate_cost < best_cost) {
      best_cost = candidate_cost;
      best_joint_values = candidate_joint_values;
    }
  }

  return best_joint_values;
}

std::vector<moveit::planning_interface::MoveGroupInterface::Plan>WordleBotController::generateCandidatePlans(int num_attempts)
{
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
  plans.reserve(static_cast<std::size_t>(std::max(num_attempts, 0)));

  for (int attempt = 0; attempt < num_attempts; ++attempt) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (static_cast<bool>(move_group_.plan(plan))) {
      plans.push_back(plan);
    }
  }

  return plans;
}

moveit::planning_interface::MoveGroupInterface::Plan WordleBotController::selectBestPlan(const std::vector<moveit::planning_interface::MoveGroupInterface::Plan> & plans,
                                                                                         const std::vector<double> & q_start,
                                                                                         const std::vector<double> & q_goal)
{
  moveit::planning_interface::MoveGroupInterface::Plan best_plan;
  moveit::planning_interface::MoveGroupInterface::Plan fallback_plan;
  double best_cost = std::numeric_limits<double>::infinity();
  double fallback_cost = std::numeric_limits<double>::infinity();

  const auto * joint_model_group = move_group_.getRobotModel()->getJointModelGroup(kPlanningGroup);
  if (joint_model_group == nullptr) {
    return best_plan;
  }

  const auto & variable_names = joint_model_group->getVariableNames();
  const auto & joint_models = joint_model_group->getActiveJointModels();
  const double direct_distance = configurationDistance(joint_model_group, q_start, q_goal);
  const int base_joint_index = jointNameIndex(variable_names, "shoulder_pan_joint");
  const int wrist_joint_index = jointNameIndex(variable_names, "wrist_3_joint");

  for (const auto & plan : plans) {
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    const auto & points = trajectory.points;
    if (points.empty()) {
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
      continue;
    }

    double total_path_length = 0.0;
    double max_joint_movement = 0.0;
    double base_rotation = 0.0;
    double wrist_rotation = 0.0;

    for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
      const auto & point = points[point_index];

      for (std::size_t joint_index = 0; joint_index < joint_models.size(); ++joint_index) {
        const double point_position = point.positions[trajectory_indices[joint_index]];
        const double movement_from_start = jointDistance(
          joint_models[joint_index], q_start[joint_index], point_position);
        max_joint_movement = std::max(max_joint_movement, movement_from_start);
      }

      if (point_index == 0) {
        continue;
      }

      const auto & previous_point = points[point_index - 1];
      double segment_length = 0.0;

      for (std::size_t joint_index = 0; joint_index < joint_models.size(); ++joint_index) {
        const double previous_position = previous_point.positions[trajectory_indices[joint_index]];
        const double current_position = point.positions[trajectory_indices[joint_index]];
        const double delta = jointDistance(joint_models[joint_index], previous_position, current_position);
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

    const double plan_cost =
      (1.0 * total_path_length) +
      (0.5 * max_joint_movement) +
      (0.75 * base_rotation) +
      (0.25 * wrist_rotation);

    if (plan_cost < fallback_cost) {
      fallback_cost = plan_cost;
      fallback_plan = plan;
    }

    const bool exceeds_path_ratio = direct_distance > 1e-6 && total_path_length > (2.0 * direct_distance);
    const bool excessive_base_rotation = base_rotation > M_PI;
    if (exceeds_path_ratio || excessive_base_rotation) {
      continue;
    }

    if (plan_cost < best_cost) {
      best_cost = plan_cost;
      best_plan = plan;
    }
  }

  if (best_cost < std::numeric_limits<double>::infinity()) {
    return best_plan;
  }

  return fallback_plan;
}

bool WordleBotController::moveToTarget(const geometry_msgs::msg::Pose & target)
{
  RCLCPP_INFO(LOGGER, "Target pose:\n  pos  x=%.3f y=%.3f z=%.3f\n  quat x=%.3f y=%.3f z=%.3f w=%.3f",
    target.position.x, target.position.y, target.position.z,
    target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);

  move_group_.setStartStateToCurrentState();

  moveit_msgs::msg::JointConstraint shoulder;
  shoulder.joint_name        = "shoulder_lift_joint";
  shoulder.position          = -M_PI / 2.0;
  shoulder.tolerance_above   = M_PI / 180.0 * 110.0; 
  shoulder.tolerance_below   = M_PI / 180.0 * 110.0;
  shoulder.weight            = 1.0;

  moveit_msgs::msg::Constraints constraints;
  constraints.joint_constraints.push_back(shoulder);
  move_group_.setPathConstraints(constraints);

  visualisePlan(nullptr, "Planning", "Press 'Next' in the RvizVisualToolsGui window to plan");

  current_state = move_group_.getCurrentState();
  const auto * joint_model_group = move_group_.getRobotModel()->getJointModelGroup(kPlanningGroup);

  std::vector<double> q_start;
  if (current_state != nullptr && joint_model_group != nullptr) {
    current_state->copyJointGroupPositions(joint_model_group, q_start);
  }

  const std::vector<double> best_q = computeBestIK(current_state, target);
  if (best_q.empty()) {
    move_group_.clearPathConstraints();
    visualisePlan(nullptr, "Planning Failed!");
    RCLCPP_ERROR(LOGGER, "Planning failed!");
    return false;
  }

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
    visualisePlan(&plan, "Executing", "Press 'Next' in the RvizVisualToolsGui window to execute");
    const auto exec_result = move_group_.execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Motion executed successfully.");
    } else {
      RCLCPP_ERROR(LOGGER, "Execution FAILED with error code: %d", exec_result.val);
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

  visualisePlan(&cart_plan, "Executing Cartesian",
    "Press 'Next' in the RvizVisualToolsGui window to execute");
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


void WordleBotController::attachSensorCollisionObject()
{
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  // Attach to tool0 — the last active joint link on the UR3e.
  // No physical tool is mounted, so there is no tool0 geometry to clear.
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

  // Only allow the links that physically surround tool0 to touch the cylinder.
  // All other links (forearm, upper_arm, shoulder, etc.) remain real collision
  // targets — the planner will prevent them from entering the sensor guard zone.
  attached_object.touch_links = {
    "wrist_3_link", "flange", "tool0", "ft_frame"
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


void WordleBotController::visualisePlan(const moveit::planning_interface::MoveGroupInterface::Plan * plan,
                                        const std::string & title,
                                        const std::string & prompt)
{
  if (!prompt.empty()) {
    visual_tools_.prompt(prompt);
  }

  auto text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools_.publishText(text_pose, title, rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);

  if (plan != nullptr) {
    const auto * joint_model_group =
      move_group_.getRobotModel()->getJointModelGroup("ur_manipulator");
    visual_tools_.publishTrajectoryLine(plan->trajectory_, joint_model_group);
  }

  visual_tools_.trigger();
}
