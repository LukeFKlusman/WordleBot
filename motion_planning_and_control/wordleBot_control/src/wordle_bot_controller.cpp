#include "wordleBot_control/wordle_bot_controller.hpp"

#include <chrono>
#include <cmath>

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

bool WordleBotController::moveToTarget(const geometry_msgs::msg::Pose & target)
{
  move_group_.setPoseTarget(target);

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

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success = static_cast<bool>(move_group_.plan(plan));

  move_group_.clearPathConstraints();

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
