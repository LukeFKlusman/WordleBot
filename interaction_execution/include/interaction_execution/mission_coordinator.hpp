#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class MissionCoordinator : public rclcpp::Node
{
public:
  explicit MissionCoordinator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class NodeStatus
  {
    FAILURE,
    SUCCESS,
    RUNNING
  };

  enum class MissionState
  {
    IDLE,
    SCANNING,
    READY_TO_MOVE,
    MOVING,
    STOPPED,
    HOMING,
    ERROR
  };

  enum class GoalRequest
  {
    NONE,
    TASK_GOAL,
    HOME_GOAL
  };

  struct MissionStepView
  {
    std::string id;
    std::string title;
    std::string detail;
    std::string status;
  };

  void handleMissionCommand(const std_msgs::msg::String::SharedPtr msg);
  void handleHumanDetected(const std_msgs::msg::Bool::SharedPtr msg);
  void handlePerceptionStatus(const std_msgs::msg::String::SharedPtr msg);
  void handlePerceptionDetections(const std_msgs::msg::String::SharedPtr msg);
  void handleMotionComplete(const std_msgs::msg::Bool::SharedPtr msg);
  void handleHeartbeat();

  void tickTree();
  NodeStatus tickSafetyGuard();
  NodeStatus tickCommandBranch();
  NodeStatus tickMotionBranch();
  NodeStatus tickScanBranch();
  std::string normaliseCommand(const std::string & command) const;
  void transitionTo(MissionState new_state, const std::string & reason);
  std::vector<MissionStepView> buildMissionSteps() const;
  std::string buildMissionProgressJson() const;
  void publishPerceptionState(const std::string & state);
  void publishMissionState();
  void publishMissionProgress();
  void dispatchConfiguredGoal(bool home_goal);
  geometry_msgs::msg::PoseStamped buildPoseFromParameters(
    const std::string & prefix,
    const std::string & frame_id) const;
  int countDetections(const std::string & json_payload) const;
  bool hasEnoughDetections() const;
  bool shouldAutoDispatch() const;
  std::string toString(MissionState state) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_detections_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_complete_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr perception_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_progress_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr set_mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_mission_pub_;

  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  MissionState state_{MissionState::IDLE};
  std::string perception_state_{"IDLE"};
  std::string perception_status_{"UNKNOWN"};
  std::string latest_detections_json_{R"({"blocks":[]})"};
  std::string pending_command_;
  int latest_detection_count_{0};
  bool human_detected_{false};
  bool motion_goal_sent_{false};
  bool motion_complete_received_{false};
  bool awaiting_motion_completion_{false};
  GoalRequest pending_goal_request_{GoalRequest::NONE};
  GoalRequest last_dispatched_goal_request_{GoalRequest::NONE};
  GoalRequest last_completed_goal_request_{GoalRequest::NONE};
  std::string last_transition_reason_{"Awaiting operator start command"};
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
};
