#include "interaction_execution/mission_coordinator.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

namespace
{
constexpr const char * kMissionCommandTopic = "/wordle_bot/mission_cmd";
constexpr const char * kMissionStateTopic = "/wordle_bot/mission_state";
constexpr const char * kMissionProgressTopic = "/wordle_bot/mission_progress";
constexpr const char * kPerceptionStateTopic = "/mission/state";
constexpr const char * kPerceptionStatusTopic = "/perception/status";
constexpr const char * kPerceptionDetectionsTopic = "/perception/detections";
constexpr const char * kHumanDetectedTopic = "/perception/human_detected";
constexpr const char * kSetMissionTopic = "/wordle_bot/set_mission";
constexpr const char * kStartMissionTopic = "/wordle_bot/start_mission";
constexpr const char * kMotionCompleteTopic = "/wordle_bot/motion_complete";

std::string jsonEscape(const std::string & value)
{
  std::ostringstream escaped;
  for (const char c : value) {
    switch (c) {
      case '\\':
        escaped << "\\\\";
        break;
      case '"':
        escaped << "\\\"";
        break;
      case '\n':
        escaped << "\\n";
        break;
      default:
        escaped << c;
        break;
    }
  }
  return escaped.str();
}
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
  mission_progress_pub_ = this->create_publisher<std_msgs::msg::String>(
    kMissionProgressTopic, rclcpp::QoS(1).reliable().transient_local());
  set_mission_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(kSetMissionTopic, 10);
  start_mission_pub_ = this->create_publisher<std_msgs::msg::Bool>(kStartMissionTopic, 10);

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

  publishPerceptionStateForMission(state_);
  publishMissionState();
  publishMissionProgress();

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
  publishMissionProgress();
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
    last_dispatched_goal_request_ = GoalRequest::NONE;
    last_completed_goal_request_ = GoalRequest::NONE;
    transitionTo(MissionState::SCANNING, "Operator requested START");
    return NodeStatus::SUCCESS;
  }

  if (command == "STOP") {
    awaiting_motion_completion_ = false;
    pending_goal_request_ = GoalRequest::NONE;
    last_dispatched_goal_request_ = GoalRequest::NONE;
    transitionTo(MissionState::STOPPED, "Operator requested STOP");
    return NodeStatus::SUCCESS;
  }

  if (command == "RESUME") {
    motion_complete_received_ = false;
    awaiting_motion_completion_ = false;
    last_completed_goal_request_ = GoalRequest::NONE;
    if (hasEnoughDetections()) {
      pending_goal_request_ = GoalRequest::TASK_GOAL;
      transitionTo(MissionState::READY_TO_MOVE, "Operator resumed with detections already available");
    } else {
      motion_goal_sent_ = false;
      pending_goal_request_ = GoalRequest::NONE;
      transitionTo(MissionState::SCANNING, "Operator requested RESUME");
    }
    return NodeStatus::SUCCESS;
  }

  if (command == "HOME" || command == "ABORT") {
    awaiting_motion_completion_ = false;
    pending_goal_request_ = GoalRequest::HOME_GOAL;
    last_completed_goal_request_ = GoalRequest::NONE;
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
    last_completed_goal_request_ = last_dispatched_goal_request_;
    last_dispatched_goal_request_ = GoalRequest::NONE;
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
  last_transition_reason_ = reason;

  if (state_ == new_state) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Mission state remains %s (%s)",
      toString(new_state).c_str(),
      reason.c_str());
    publishPerceptionStateForMission(state_);
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
  publishPerceptionStateForMission(state_);
  publishMissionState();
  publishMissionProgress();
}

void MissionCoordinator::publishPerceptionStateForMission(MissionState state)
{
  switch (state) {
    case MissionState::SCANNING:
    case MissionState::READY_TO_MOVE:
    case MissionState::MOVING:
    case MissionState::HOMING:
      publishPerceptionState("SCANNING");
      return;
    case MissionState::IDLE:
    case MissionState::STOPPED:
    case MissionState::ERROR:
      publishPerceptionState("IDLE");
      return;
  }
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

std::vector<MissionCoordinator::MissionStepView> MissionCoordinator::buildMissionSteps() const
{
  const int minimum_detected_blocks = this->get_parameter("minimum_detected_blocks").as_int();
  const std::string detections_detail =
    "Detected " + std::to_string(latest_detection_count_) + " block(s); minimum needed is " +
    std::to_string(minimum_detected_blocks) + ".";

  std::vector<MissionStepView> steps;
  steps.push_back({
    "operator",
    "Await operator command",
    state_ == MissionState::STOPPED ?
      (human_detected_ ?
        "Mission paused by safety. Clear the workspace, then choose RESUME or HOME." :
        "Mission paused. Choose RESUME to continue or HOME to return to the safe pose.") :
      (state_ == MissionState::IDLE ?
        "Ready for START to begin a new scan cycle." :
        "Operator command accepted. Mission is now running."),
    (state_ == MissionState::IDLE || state_ == MissionState::STOPPED) ? "active" : "done"});

  steps.push_back({
    "safety",
    "Monitor safety envelope",
    human_detected_ ?
      "Human detected. Robot motion is halted until the workspace is clear." :
      (awaiting_motion_completion_ ?
        "Safety monitoring remains active while the robot is moving." :
        "Human monitoring is active and the workspace is currently clear."),
    human_detected_ ? "blocked" : "active"});

  steps.push_back({
    "scan",
    "Scan puzzle and detect blocks",
    state_ == MissionState::SCANNING ?
      (detections_detail + " Waiting for enough detections to continue.") :
      (state_ == MissionState::READY_TO_MOVE || state_ == MissionState::MOVING ||
      state_ == MissionState::HOMING ?
        (detections_detail + " Continuous scanning remains active while the robot executes.") :
        (last_completed_goal_request_ == GoalRequest::TASK_GOAL ?
          "Previous task motion finished successfully. Perception is now idle until the next command." :
          "Perception is idle until the operator starts or resumes the mission.")),
    (state_ == MissionState::SCANNING || state_ == MissionState::READY_TO_MOVE ||
    state_ == MissionState::MOVING || state_ == MissionState::HOMING) ? "active" :
      (last_completed_goal_request_ == GoalRequest::TASK_GOAL ? "done" :
      (state_ == MissionState::STOPPED ? "blocked" : "pending"))});

  steps.push_back({
    "prepare",
    "Prepare robot motion",
    state_ == MissionState::READY_TO_MOVE ?
      "Target detections are ready. Waiting to dispatch the configured robot motion." :
      (state_ == MissionState::MOVING ?
        "Motion plan has been dispatched to the controller." :
        (state_ == MissionState::HOMING ?
          "Task motion is bypassed while the robot returns home." :
          "No motion plan is currently queued.")),
    state_ == MissionState::READY_TO_MOVE ? "active" :
      (state_ == MissionState::MOVING ? "done" :
      (state_ == MissionState::STOPPED ? "blocked" : "pending"))});

  steps.push_back({
    "execute",
    "Execute robot motion",
    state_ == MissionState::MOVING ?
      "Robot motion is in progress. Continue safety checks until motion_complete is received." :
      (last_completed_goal_request_ == GoalRequest::TASK_GOAL ?
        "The last task motion completed successfully and control returned to IDLE." :
        "Waiting for a task motion to begin."),
    state_ == MissionState::MOVING ? "active" :
      (last_completed_goal_request_ == GoalRequest::TASK_GOAL ? "done" :
      (state_ == MissionState::STOPPED ? "blocked" : "pending"))});

  steps.push_back({
    "home",
    "Return to safe home pose",
    state_ == MissionState::HOMING ?
      "Home motion is running. Perception continues scanning until the robot reaches home." :
      (last_completed_goal_request_ == GoalRequest::HOME_GOAL ?
        "Home motion completed. The system is back in IDLE and ready for the next command." :
        "Robot moves to a safe home pose upon full task completion or explicit HOME request."),
    state_ == MissionState::HOMING ? "active" :
      (last_completed_goal_request_ == GoalRequest::HOME_GOAL ? "done" :
      (state_ == MissionState::STOPPED ? "pending" : "pending"))});

  return steps;
}

std::string MissionCoordinator::buildMissionProgressJson() const
{
  const auto steps = buildMissionSteps();

  std::ostringstream payload;
  payload << '{'
          << "\"title\":\"" << jsonEscape("Wordle Game Pick and Place") << "\","
          << "\"summary\":\"" << jsonEscape(
              "State: " + toString(state_) + " | " + last_transition_reason_) << "\","
          << "\"state\":\"" << jsonEscape(toString(state_)) << "\","
          << "\"steps\":[";

  for (std::size_t index = 0; index < steps.size(); ++index) {
    const auto & step = steps.at(index);
    if (index > 0) {
      payload << ',';
    }

    payload << '{'
            << "\"id\":\"" << jsonEscape(step.id) << "\","
            << "\"title\":\"" << jsonEscape(step.title) << "\","
            << "\"detail\":\"" << jsonEscape(step.detail) << "\","
            << "\"status\":\"" << jsonEscape(step.status) << "\""
            << '}';
  }

  payload << "]}";
  return payload.str();
}

void MissionCoordinator::publishMissionProgress()
{
  std_msgs::msg::String msg;
  msg.data = buildMissionProgressJson();
  mission_progress_pub_->publish(msg);
}

void MissionCoordinator::dispatchConfiguredGoal(bool home_goal)
{
  const std::string frame_id = this->get_parameter(
    home_goal ? "home_frame_id" : "goal_frame_id").as_string();

  const auto goal = buildPoseFromParameters(home_goal ? "home" : "goal", frame_id);

  geometry_msgs::msg::PoseArray mission;
  mission.header = goal.header;
  mission.poses.push_back(goal.pose);
  set_mission_pub_->publish(mission);

  std_msgs::msg::Bool start_signal;
  start_signal.data = true;
  start_mission_pub_->publish(start_signal);

  motion_goal_sent_ = true;
  awaiting_motion_completion_ = true;
  pending_goal_request_ = GoalRequest::NONE;
  last_dispatched_goal_request_ = home_goal ? GoalRequest::HOME_GOAL : GoalRequest::TASK_GOAL;
  transitionTo(home_goal ? MissionState::HOMING : MissionState::MOVING,
    home_goal ? "Published configured home mission" : "Published configured motion mission");
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
