#include "interaction_execution/mission_coordinator.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace
{
constexpr const char * kMissionCommandTopic = "/wordle_bot/mission_cmd";
constexpr const char * kMissionStateTopic = "/wordle_bot/mission_state";
constexpr const char * kPerceptionStateTopic = "/mission/state";
constexpr const char * kPerceptionStatusTopic = "/perception/status";
constexpr const char * kPerceptionDetectionsTopic = "/perception/detections";
constexpr const char * kHumanDetectedTopic = "/perception/human_detected";
constexpr const char * kGoalPoseTopic = "/wordle_bot/goal_pose";
constexpr const char * kMotionCompleteTopic = "/wordle_bot/motion_complete";
}  // namespace

MissionCoordinator::MissionCoordinator(const rclcpp::NodeOptions & options)
: rclcpp::Node("mission_coordinator", options)
{
  this->declare_parameter<bool>("auto_dispatch_motion", false);
  this->declare_parameter<int>("minimum_detected_blocks", 1);
  this->declare_parameter<std::string>("goal_frame_id", "ur_base_link");
  this->declare_parameter<std::string>("home_frame_id", "ur_base_link");

  this->declare_parameter<double>("goal.x", 0.30);
  this->declare_parameter<double>("goal.y", 0.25);
  this->declare_parameter<double>("goal.z", 0.25);
  this->declare_parameter<double>("goal.roll", M_PI);
  this->declare_parameter<double>("goal.pitch", 0.0);
  this->declare_parameter<double>("goal.yaw", 0.0);

  this->declare_parameter<double>("home.x", 0.30);
  this->declare_parameter<double>("home.y", 0.00);
  this->declare_parameter<double>("home.z", 0.30);
  this->declare_parameter<double>("home.roll", M_PI);
  this->declare_parameter<double>("home.pitch", 0.0);
  this->declare_parameter<double>("home.yaw", 0.0);

  perception_state_pub_ = this->create_publisher<std_msgs::msg::String>(
    kPerceptionStateTopic, rclcpp::QoS(1).reliable().transient_local());
  mission_state_pub_ = this->create_publisher<std_msgs::msg::String>(
    kMissionStateTopic, rclcpp::QoS(1).reliable().transient_local());
  goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(kGoalPoseTopic, 10);

  mission_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
    kMissionCommandTopic, 10,
    std::bind(&MissionCoordinator::handleMissionCommand, this, std::placeholders::_1));
  human_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    kHumanDetectedTopic, rclcpp::SensorDataQoS(),
    std::bind(&MissionCoordinator::handleHumanDetected, this, std::placeholders::_1));
  perception_status_sub_ = this->create_subscription<std_msgs::msg::String>(
    kPerceptionStatusTopic, 10,
    std::bind(&MissionCoordinator::handlePerceptionStatus, this, std::placeholders::_1));
  perception_detections_sub_ = this->create_subscription<std_msgs::msg::String>(
    kPerceptionDetectionsTopic, 10,
    std::bind(&MissionCoordinator::handlePerceptionDetections, this, std::placeholders::_1));
  motion_complete_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    kMotionCompleteTopic, 10,
    std::bind(&MissionCoordinator::handleMotionComplete, this, std::placeholders::_1));

  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MissionCoordinator::handleHeartbeat, this));

  publishPerceptionState("IDLE");
  publishMissionState();

  RCLCPP_INFO(
    this->get_logger(),
    "Mission coordinator ready. Subscribing to %s, %s, %s, %s, %s.",
    kMissionCommandTopic,
    kHumanDetectedTopic,
    kPerceptionStatusTopic,
    kPerceptionDetectionsTopic,
    kMotionCompleteTopic);
}

void MissionCoordinator::handleMissionCommand(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }

  pending_command_ = normaliseCommand(msg->data);
  RCLCPP_INFO(this->get_logger(), "Queued mission command: %s", pending_command_.c_str());
}

void MissionCoordinator::handleHumanDetected(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }

  const bool previous = human_detected_;
  human_detected_ = msg->data;

  if (human_detected_ && !previous) {
    RCLCPP_WARN(this->get_logger(), "Human detected. Safety guard will stop the mission.");
  }
}

void MissionCoordinator::handlePerceptionStatus(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }

  perception_status_ = msg->data;
}

void MissionCoordinator::handlePerceptionDetections(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }

  latest_detections_json_ = msg->data;
  latest_detection_count_ = countDetections(latest_detections_json_);
  last_detection_time_ = this->now();

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Perception detections updated. blocks=%d",
    latest_detection_count_);

  if (state_ != MissionState::SCANNING || human_detected_ || !hasEnoughDetections()) {
    return;
  }

  publishPerceptionState("IDLE");
  transitionTo(MissionState::READY_TO_MOVE, "Perception returned enough detections");

  if (shouldAutoDispatch()) {
    dispatchConfiguredGoal(false);
  }
}

void MissionCoordinator::handleMotionComplete(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg == nullptr || !msg->data) {
    return;
  }

  motion_complete_received_ = true;
}

void MissionCoordinator::handleHeartbeat()
{
  tickTree();
  publishMissionState();
}

void MissionCoordinator::tickTree()
{
  if (tickSafetyGuard() != NodeStatus::FAILURE) {
    return;
  }

  if (tickCommandBranch() != NodeStatus::FAILURE) {
    return;
  }

  if (tickMotionBranch() != NodeStatus::FAILURE) {
    return;
  }

  (void)tickScanBranch();
}

MissionCoordinator::NodeStatus MissionCoordinator::tickSafetyGuard()
{
  if (!human_detected_) {
    return NodeStatus::FAILURE;
  }

  pending_command_.clear();
  publishPerceptionState("IDLE");
  awaiting_motion_completion_ = false;
  pending_goal_request_ = GoalRequest::NONE;
  transitionTo(MissionState::STOPPED, "Human detected by perception");
  return NodeStatus::RUNNING;
}

MissionCoordinator::NodeStatus MissionCoordinator::tickCommandBranch()
{
  if (pending_command_.empty()) {
    return NodeStatus::FAILURE;
  }

  const std::string command = pending_command_;
  pending_command_.clear();

  if (command == "START") {
    motion_goal_sent_ = false;
    motion_complete_received_ = false;
    awaiting_motion_completion_ = false;
    pending_goal_request_ = GoalRequest::NONE;
    publishPerceptionState("SCANNING");
    transitionTo(MissionState::SCANNING, "Operator requested START");
    return NodeStatus::SUCCESS;
  }

  if (command == "STOP") {
    awaiting_motion_completion_ = false;
    pending_goal_request_ = GoalRequest::NONE;
    publishPerceptionState("IDLE");
    transitionTo(MissionState::STOPPED, "Operator requested STOP");
    return NodeStatus::SUCCESS;
  }

  if (command == "RESUME") {
    motion_complete_received_ = false;
    awaiting_motion_completion_ = false;
    if (hasEnoughDetections()) {
      pending_goal_request_ = GoalRequest::TASK_GOAL;
      publishPerceptionState("IDLE");
      transitionTo(MissionState::READY_TO_MOVE, "Operator resumed with detections already available");
    } else {
      motion_goal_sent_ = false;
      pending_goal_request_ = GoalRequest::NONE;
      publishPerceptionState("SCANNING");
      transitionTo(MissionState::SCANNING, "Operator requested RESUME");
    }
    return NodeStatus::SUCCESS;
  }

  if (command == "HOME" || command == "ABORT") {
    awaiting_motion_completion_ = false;
    pending_goal_request_ = GoalRequest::HOME_GOAL;
    publishPerceptionState("IDLE");
    transitionTo(MissionState::HOMING, "Operator requested HOME");
    return NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(this->get_logger(), "Ignoring unsupported mission command '%s'.", command.c_str());
  return NodeStatus::SUCCESS;
}

MissionCoordinator::NodeStatus MissionCoordinator::tickMotionBranch()
{
  if (motion_complete_received_) {
    motion_complete_received_ = false;

    if (!awaiting_motion_completion_) {
      RCLCPP_INFO(this->get_logger(), "Received motion complete while not awaiting motion completion.");
      return NodeStatus::SUCCESS;
    }

    awaiting_motion_completion_ = false;
    motion_goal_sent_ = false;
    pending_goal_request_ = GoalRequest::NONE;
    publishPerceptionState("IDLE");
    transitionTo(MissionState::IDLE, "Motion execution completed");
    return NodeStatus::SUCCESS;
  }

  if (pending_goal_request_ == GoalRequest::NONE) {
    return awaiting_motion_completion_ ? NodeStatus::RUNNING : NodeStatus::FAILURE;
  }

  if (!shouldAutoDispatch()) {
    return NodeStatus::RUNNING;
  }

  const bool home_goal = pending_goal_request_ == GoalRequest::HOME_GOAL;
  dispatchConfiguredGoal(home_goal);
  return NodeStatus::SUCCESS;
}

MissionCoordinator::NodeStatus MissionCoordinator::tickScanBranch()
{
  if (state_ != MissionState::SCANNING) {
    return NodeStatus::FAILURE;
  }

  if (!hasEnoughDetections()) {
    return NodeStatus::RUNNING;
  }

  pending_goal_request_ = GoalRequest::TASK_GOAL;
  publishPerceptionState("IDLE");
  transitionTo(MissionState::READY_TO_MOVE, "Perception returned enough detections");
  return NodeStatus::SUCCESS;
}

std::string MissionCoordinator::normaliseCommand(const std::string & command) const
{
  std::string upper = command;
  std::transform(upper.begin(), upper.end(), upper.begin(),
    [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return upper;
}

void MissionCoordinator::transitionTo(MissionState new_state, const std::string & reason)
{
  if (state_ == new_state) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Mission state remains %s (%s)",
      toString(new_state).c_str(),
      reason.c_str());
    publishMissionState();
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Mission state transition: %s -> %s (%s)",
    toString(state_).c_str(),
    toString(new_state).c_str(),
    reason.c_str());
  state_ = new_state;
  publishMissionState();
}

void MissionCoordinator::publishPerceptionState(const std::string & state)
{
  perception_state_ = state;

  std_msgs::msg::String msg;
  msg.data = state;
  perception_state_pub_->publish(msg);
}

void MissionCoordinator::publishMissionState()
{
  std_msgs::msg::String msg;
  msg.data = toString(state_);
  mission_state_pub_->publish(msg);
}

void MissionCoordinator::dispatchConfiguredGoal(bool home_goal)
{
  const std::string frame_id = this->get_parameter(
    home_goal ? "home_frame_id" : "goal_frame_id").as_string();

  const auto goal = buildPoseFromParameters(home_goal ? "home" : "goal", frame_id);
  goal_pose_pub_->publish(goal);
  motion_goal_sent_ = true;
  awaiting_motion_completion_ = true;
  pending_goal_request_ = GoalRequest::NONE;
  transitionTo(home_goal ? MissionState::HOMING : MissionState::MOVING,
    home_goal ? "Published configured home pose" : "Published configured motion goal");
}

geometry_msgs::msg::PoseStamped MissionCoordinator::buildPoseFromParameters(
  const std::string & prefix,
  const std::string & frame_id) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->now();
  pose.header.frame_id = frame_id;

  const double x = this->get_parameter(prefix + ".x").as_double();
  const double y = this->get_parameter(prefix + ".y").as_double();
  const double z = this->get_parameter(prefix + ".z").as_double();
  const double roll = this->get_parameter(prefix + ".roll").as_double();
  const double pitch = this->get_parameter(prefix + ".pitch").as_double();
  const double yaw = this->get_parameter(prefix + ".yaw").as_double();

  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
  pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
  pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
  pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
  return pose;
}

int MissionCoordinator::countDetections(const std::string & json_payload) const
{
  int count = 0;
  std::size_t cursor = 0;
  while ((cursor = json_payload.find("\"letter\"", cursor)) != std::string::npos) {
    ++count;
    cursor += 8;
  }
  return count;
}

bool MissionCoordinator::hasEnoughDetections() const
{
  const int minimum_detected_blocks = this->get_parameter("minimum_detected_blocks").as_int();
  return latest_detection_count_ >= minimum_detected_blocks;
}

bool MissionCoordinator::shouldAutoDispatch() const
{
  return this->get_parameter("auto_dispatch_motion").as_bool() && !motion_goal_sent_;
}

std::string MissionCoordinator::toString(MissionState state) const
{
  switch (state) {
    case MissionState::IDLE:
      return "IDLE";
    case MissionState::SCANNING:
      return "SCANNING";
    case MissionState::READY_TO_MOVE:
      return "READY_TO_MOVE";
    case MissionState::MOVING:
      return "MOVING";
    case MissionState::STOPPED:
      return "STOPPED";
    case MissionState::HOMING:
      return "HOMING";
    case MissionState::ERROR:
      return "ERROR";
  }

  return "ERROR";
}
