#include "move_goal_test/move_goal_test.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

namespace move_goal_test
{

using namespace std::chrono_literals;
namespace
{
constexpr std::size_t kJointCount = 6;
}

MoveGoalTestNode::MoveGoalTestNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("move_goal_test_node", options)
{
  // Parameter so you can change controller name without recompiling
  action_name_ = this->declare_parameter<std::string>(
    "action_name",
    "/scaled_joint_trajectory_controller/follow_joint_trajectory");
  joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
  max_joint_speed_rad_s_ = this->declare_parameter<double>("max_joint_speed_rad_s", 0.5);
  speed_scaling_ = this->declare_parameter<double>("speed_scaling", 0.7);
  min_segment_duration_s_ = this->declare_parameter<double>("min_segment_duration_s", 0.2);
  initial_point_delay_s_ = this->declare_parameter<double>("initial_point_delay_s", 0.3);
  max_joint_step_rad_ = this->declare_parameter<double>("max_joint_step_rad", 0.03);
  marker_topic_ = this->declare_parameter<std::string>("trajectory_marker_topic", "/move_goal_test/trajectory_marker");
  marker_frame_ = this->declare_parameter<std::string>("trajectory_marker_frame", "base_link");

  action_client_ = rclcpp_action::create_client<FollowJT>(this, action_name_);
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, 10,
    std::bind(&MoveGoalTestNode::joint_state_callback, this, std::placeholders::_1));
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 1);

  // Kick off after node is fully up
  startup_timer_ = this->create_wall_timer(200ms, std::bind(&MoveGoalTestNode::start, this));
}

void MoveGoalTestNode::start()
{
  if (goal_sent_) {
    return;
  }

  if (!action_client_->wait_for_action_server(0s)) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "Waiting for action server: %s", action_name_.c_str());
    return;
  }

  if (!latest_joint_state_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "Waiting for joint states on topic: %s",
      joint_states_topic_.c_str());
    return;
  }

  startup_timer_->cancel();
  goal_sent_ = true;
  RCLCPP_INFO(get_logger(), "Action server is available. Sending goal.");
  send_goal();
}

void MoveGoalTestNode::send_goal()
{
  FollowJT::Goal goal_msg;
  try {
    goal_msg = build_goal();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Could not build trajectory goal: %s", ex.what());
    rclcpp::shutdown();
    return;
  }

  rclcpp_action::Client<FollowJT>::SendGoalOptions options;
  options.goal_response_callback =
    std::bind(&MoveGoalTestNode::goal_response_callback, this, std::placeholders::_1);
  options.feedback_callback =
    std::bind(&MoveGoalTestNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  options.result_callback =
    std::bind(&MoveGoalTestNode::result_callback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, options);
}

MoveGoalTestNode::FollowJT::Goal MoveGoalTestNode::build_goal()
{
  FollowJT::Goal goal;

  const std::array<std::string, kJointCount> joint_names = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
  };

  const std::array<std::array<double, kJointCount>, 5> targets = {{
    { 0.0,  -1.57,  1.57,  0.0,  1.57,  0.0 },
    { 0.8,  -1.20,  1.30, -0.60, 1.57,  0.50 },
    {-0.8,  -1.05,  1.90,  0.70, 1.57, -0.70 },
    { 1.2,  -1.45,  1.10, -1.00, 1.57,  1.00 },
    { 0.0,  -1.57,  1.57,  0.0,  1.57,  0.0 }
  }};

  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names.assign(joint_names.begin(), joint_names.end());

  auto previous = extract_current_positions(joint_names);
  double accumulated_time_s = std::max(initial_point_delay_s_, 0.0);

  trajectory_msgs::msg::JointTrajectoryPoint start_pt;
  start_pt.positions.assign(previous.begin(), previous.end());
  start_pt.time_from_start = to_duration(accumulated_time_s);
  traj.points.push_back(start_pt);

  const auto dense_positions = densify_waypoints(
    previous, std::vector<std::array<double, kJointCount>>(targets.begin(), targets.end()));
  append_points_with_timing(traj, dense_positions, accumulated_time_s);
  fill_point_velocities(traj);
  publish_trajectory_marker(traj);

  goal.trajectory = traj;
  return goal;
}

void MoveGoalTestNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  latest_joint_state_ = msg;
}

std::array<double, 6> MoveGoalTestNode::extract_current_positions(
  const std::array<std::string, 6> & joint_names) const
{
  if (!latest_joint_state_) {
    throw std::runtime_error("joint state not received yet");
  }

  std::array<double, 6> current{};
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    const auto & target_name = joint_names[i];
    auto it = std::find(latest_joint_state_->name.begin(), latest_joint_state_->name.end(), target_name);
    if (it == latest_joint_state_->name.end()) {
      throw std::runtime_error("joint '" + target_name + "' not found in /joint_states");
    }

    const auto idx = static_cast<std::size_t>(std::distance(latest_joint_state_->name.begin(), it));
    if (idx >= latest_joint_state_->position.size()) {
      throw std::runtime_error("joint state message has invalid position array length");
    }
    current[i] = latest_joint_state_->position[idx];
  }
  return current;
}

double MoveGoalTestNode::compute_segment_duration(
  const std::array<double, 6> & from,
  const std::array<double, 6> & to) const
{
  const double effective_speed = max_joint_speed_rad_s_ * speed_scaling_;
  if (effective_speed <= 0.0) {
    throw std::runtime_error("effective joint speed must be > 0 (check max_joint_speed_rad_s and speed_scaling)");
  }

  double max_joint_delta = 0.0;
  for (std::size_t i = 0; i < from.size(); ++i) {
    max_joint_delta = std::max(max_joint_delta, std::abs(to[i] - from[i]));
  }

  const double required_time_s = max_joint_delta / effective_speed;
  return std::max(required_time_s, min_segment_duration_s_);
}

builtin_interfaces::msg::Duration MoveGoalTestNode::to_duration(double seconds)
{
  if (seconds < 0.0) {
    seconds = 0.0;
  }

  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(std::floor(seconds));
  const double remainder = seconds - static_cast<double>(duration.sec);
  duration.nanosec = static_cast<uint32_t>(std::llround(remainder * 1e9));

  if (duration.nanosec >= 1000000000u) {
    duration.sec += 1;
    duration.nanosec -= 1000000000u;
  }

  return duration;
}

double MoveGoalTestNode::to_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

std::vector<std::array<double, 6>> MoveGoalTestNode::densify_waypoints(
  const std::array<double, 6> & start,
  const std::vector<std::array<double, 6>> & targets) const
{
  if (max_joint_step_rad_ <= 0.0) {
    throw std::runtime_error("max_joint_step_rad must be > 0");
  }

  std::vector<std::array<double, 6>> dense;
  auto from = start;

  for (const auto & to : targets) {
    double max_delta = 0.0;
    for (std::size_t i = 0; i < from.size(); ++i) {
      max_delta = std::max(max_delta, std::abs(to[i] - from[i]));
    }

    const std::size_t steps = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(max_delta / max_joint_step_rad_)));

    for (std::size_t step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      std::array<double, 6> q{};
      for (std::size_t j = 0; j < from.size(); ++j) {
        q[j] = from[j] + ratio * (to[j] - from[j]);
      }
      dense.push_back(q);
    }

    from = to;
  }

  return dense;
}

void MoveGoalTestNode::append_points_with_timing(
  trajectory_msgs::msg::JointTrajectory & traj,
  const std::vector<std::array<double, 6>> & positions,
  double start_time_s) const
{
  if (traj.points.empty()) {
    throw std::runtime_error("trajectory must have a start point before appending");
  }

  std::array<double, 6> previous{};
  for (std::size_t i = 0; i < previous.size(); ++i) {
    previous[i] = traj.points.back().positions[i];
  }

  double accumulated_time_s = start_time_s;
  for (const auto & q : positions) {
    accumulated_time_s += compute_segment_duration(previous, q);

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(q.begin(), q.end());
    pt.time_from_start = to_duration(accumulated_time_s);
    traj.points.push_back(pt);

    previous = q;
  }
}

void MoveGoalTestNode::fill_point_velocities(trajectory_msgs::msg::JointTrajectory & traj) const
{
  if (traj.points.size() < 2) {
    return;
  }

  const double max_velocity = max_joint_speed_rad_s_ * speed_scaling_;
  for (auto & point : traj.points) {
    point.velocities.assign(kJointCount, 0.0);
  }

  for (std::size_t i = 1; i + 1 < traj.points.size(); ++i) {
    const auto & prev = traj.points[i - 1];
    const auto & next = traj.points[i + 1];
    const double dt = to_seconds(next.time_from_start) - to_seconds(prev.time_from_start);
    if (dt <= 1e-6) {
      continue;
    }

    for (std::size_t j = 0; j < kJointCount; ++j) {
      const double raw_v = (next.positions[j] - prev.positions[j]) / dt;
      traj.points[i].velocities[j] = std::clamp(raw_v, -max_velocity, max_velocity);
    }
  }
}

void MoveGoalTestNode::publish_trajectory_marker(const trajectory_msgs::msg::JointTrajectory & traj) const
{
  if (!marker_pub_ || traj.points.empty()) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = this->now();
  marker.header.frame_id = marker_frame_;
  marker.ns = "trajectory_preview";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;
  marker.color.a = 1.0F;
  marker.color.r = 0.1F;
  marker.color.g = 0.9F;
  marker.color.b = 0.2F;

  marker.points.reserve(traj.points.size());
  for (const auto & pt : traj.points) {
    geometry_msgs::msg::Point p;
    p.x = to_seconds(pt.time_from_start);
    p.y = pt.positions.size() > 0 ? pt.positions[0] : 0.0;
    p.z = pt.positions.size() > 1 ? pt.positions[1] : 0.0;
    marker.points.push_back(p);
  }

  marker_pub_->publish(marker);
}

void MoveGoalTestNode::goal_response_callback(const GoalHandleFollowJT::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server.");
    return;
  }
  RCLCPP_INFO(get_logger(), "Goal accepted. Waiting for result...");
}

void MoveGoalTestNode::feedback_callback(
  GoalHandleFollowJT::SharedPtr,
  const std::shared_ptr<const FollowJT::Feedback> feedback)
{
  // Feedback is optional. This gives you visibility while testing.
  // Most controllers populate desired/actual/error.
  (void)feedback;
  // If you want, you can log at a low rate. Leaving quiet for now.
}

void MoveGoalTestNode::result_callback(const GoalHandleFollowJT::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Trajectory execution succeeded.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Trajectory execution was aborted.");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "Trajectory execution was canceled.");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code.");
      break;
  }

  // Optional: shut down after sending the one test trajectory
  rclcpp::shutdown();
}

}  // namespace move_goal_test
