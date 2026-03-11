#pragma once

#include <memory>
#include <array>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace move_goal_test
{

class MoveGoalTestNode : public rclcpp::Node
{
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJT = rclcpp_action::ClientGoalHandle<FollowJT>;

  explicit MoveGoalTestNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void start();                 // Wait for action server then send goal
  void send_goal();             // Build goal message and send
  FollowJT::Goal build_goal();  // Constructs the trajectory goal
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  std::array<double, 6> extract_current_positions(
    const std::array<std::string, 6> & joint_names) const;
  double compute_segment_duration(
    const std::array<double, 6> & from,
    const std::array<double, 6> & to) const;
  static builtin_interfaces::msg::Duration to_duration(double seconds);
  static double to_seconds(const builtin_interfaces::msg::Duration & duration);
  std::vector<std::array<double, 6>> densify_waypoints(
    const std::array<double, 6> & start,
    const std::vector<std::array<double, 6>> & targets) const;
  void append_points_with_timing(
    trajectory_msgs::msg::JointTrajectory & traj,
    const std::vector<std::array<double, 6>> & positions,
    double start_time_s) const;
  void fill_point_velocities(trajectory_msgs::msg::JointTrajectory & traj) const;
  void publish_trajectory_marker(const trajectory_msgs::msg::JointTrajectory & traj) const;

  // Callbacks for the action client
  void goal_response_callback(const GoalHandleFollowJT::SharedPtr & goal_handle);
  void feedback_callback(
    GoalHandleFollowJT::SharedPtr,
    const std::shared_ptr<const FollowJT::Feedback> feedback);
  void result_callback(const GoalHandleFollowJT::WrappedResult & result);

  rclcpp_action::Client<FollowJT>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  // Parameters
  std::string action_name_;
  std::string joint_states_topic_;
  double max_joint_speed_rad_s_;
  double speed_scaling_;
  double min_segment_duration_s_;
  double initial_point_delay_s_;
  double max_joint_step_rad_;
  std::string marker_topic_;
  std::string marker_frame_;

  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  bool goal_sent_{false};
};

}  // namespace move_goal_test
