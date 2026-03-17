#include "wordleBot_control/wordle_bot_controller.hpp"

#include <cmath>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("WordleBotController");

// ---------------------------------------------------------------------------
// Constructor and destructor
// ---------------------------------------------------------------------------

WordleBotController::WordleBotController(rclcpp::Node::SharedPtr node)
: node_(node),
  move_group_(node, "ur_manipulator"),
  planning_scene_(),
  visual_tools_(node, "ur_base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_.getRobotModel())
{
  visual_tools_.deleteAllMarkers();
  visual_tools_.loadRemoteControl();
}

WordleBotController::~WordleBotController()
{
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------

bool WordleBotController::moveToTarget(const geometry_msgs::msg::Pose & target)
{
  move_group_.setPoseTarget(target);

  RCLCPP_INFO(LOGGER, "Target pose:\n  pos  x=%.3f y=%.3f z=%.3f\n  quat x=%.3f y=%.3f z=%.3f w=%.3f",
    target.position.x, target.position.y, target.position.z,
    target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);

  // Helper closures — mirrors hello_ur3eMoveit style
  auto const draw_title = [this](const std::string & text) {
    auto text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools_.publishText(text_pose, text, rviz_visual_tools::WHITE,
      rviz_visual_tools::XLARGE);
  };

  auto const draw_trajectory =
    [this, jmg = move_group_.getRobotModel()->getJointModelGroup("ur_manipulator")](
    const moveit::planning_interface::MoveGroupInterface::Plan & plan) {
      visual_tools_.publishTrajectoryLine(plan.trajectory_, jmg);
    };

  // Prompt operator before planning
  visual_tools_.prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  visual_tools_.trigger();

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success = static_cast<bool>(move_group_.plan(plan));

  if (success) {
    draw_trajectory(plan);
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    visual_tools_.trigger();
    move_group_.execute(plan);
    RCLCPP_INFO(LOGGER, "Motion executed successfully.");
  } 
  else 
  {
    draw_title("Planning Failed!");
    visual_tools_.trigger();
    RCLCPP_ERROR(LOGGER, "Planning failed!");
  }

  return success;
}

// ---------------------------------------------------------------------------
// Collision scene
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
  collision_object.operation = collision_object.ADD;

  planning_scene_.applyCollisionObject(collision_object);
  RCLCPP_INFO(LOGGER, "Collision scene set up: box1 added.");
}

void WordleBotController::clearCollisionScene()
{
  planning_scene_.removeCollisionObjects({"box1"});
  RCLCPP_INFO(LOGGER, "Collision scene cleared.");
}

// ---------------------------------------------------------------------------
// Pose builder — tf2::Quaternion RPY pattern from hello_ur3eMoveit
// ---------------------------------------------------------------------------

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
