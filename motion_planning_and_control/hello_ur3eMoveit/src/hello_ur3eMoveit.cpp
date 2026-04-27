#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_ur3eMoveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_ur3eMoveit");

  // Spin up a SingleThreadedExecutor for the current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Get the current end-effector pose
  auto const end_effector_pose = move_group_interface.getCurrentPose();

  RCLCPP_INFO(logger, "Current end-effector pose:\n x: %f, y: %f, z: %f\n r: %f, p: %f, y: %f, w: %f", 
                                              
    end_effector_pose.pose.position.x, 
                                              end_effector_pose.pose.position.y, 
                                              end_effector_pose.pose.position.z, 
                                              end_effector_pose.pose.orientation.x, 
                                              end_effector_pose.pose.orientation.y, 
                                              end_effector_pose.pose.orientation.z,
                                              end_effector_pose.pose.orientation.w
                                            );

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = 
    moveit_visual_tools::MoveItVisualTools{ node, "ur_base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                            move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
          "ur_manipulator")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

  // Define a target orientation in RPY and convert to quaternion
  tf2::Quaternion quat;
  quat.setRPY(M_PI, 0.0, 0.0); // Rotate 180 degrees around the X-axis to flip the end-effector downwards
  quat.normalize();

  // Set a target Pose
  auto const target_pose = [quat]{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = quat.x();
    msg.orientation.y = quat.y();
    msg.orientation.z = quat.z();
    msg.orientation.w = quat.w();
    msg.position.x = 0.335;
    msg.position.y = 0.3;
    msg.position.z = 0.15;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Log the target pose and orientation
  RCLCPP_INFO(logger, "Target orientation in RPY:\n roll: %f, pitch: %f, yaw: %f", M_PI, 0.0, 0.0);
  RCLCPP_INFO(logger, "Target orientation in quaternion:\n x: %f, y: %f, z: %f, w: %f", quat.x(), quat.y(), quat.z(), quat.w());
  RCLCPP_INFO(logger, "Target pose set to:\n x: %f, y: %f, z: %f\n r: %f, p: %f, y: %f, w: %f", target_pose.position.x, 
                                              target_pose.position.y, 
                                              target_pose.position.z, 
                                              target_pose.orientation.x, 
                                              target_pose.orientation.y, 
                                              target_pose.orientation.z,
                                              target_pose.orientation.w
                                            );

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id =
                                  move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.05;
    primitive.dimensions[primitive.BOX_Z] = 0.25;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.35;
    box_pose.position.y = 0.1;
    box_pose.position.z = 0.1;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } 
  else 
  {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join(); 
  return 0;
}
