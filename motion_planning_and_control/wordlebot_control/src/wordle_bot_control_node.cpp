#include "wordlebot_control/wordle_bot_control_node.hpp"

#include <chrono>
#include <cmath>
#include <thread>
#include <unordered_set>
#include <moveit/planning_scene/planning_scene.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("WordleBotControlNode");

namespace
{
bool normalizePoseOrientation(
  geometry_msgs::msg::Pose & pose,
  const std::string & label)
{
  const double norm = std::sqrt(
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w);

  if (norm < 1e-9) {
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    RCLCPP_WARN(LOGGER,
      "%s orientation quaternion was invalid/near-zero; using identity orientation.",
      label.c_str());
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

WordleBotControlNode::WordleBotControlNode(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("wordle_bot_control_node", options)),
  controller_(std::make_shared<WordleBotController>(node_))
{
  // ---------------------------------------------------------------------------
  // ROS2 topic subscriptions and publications
  // ---------------------------------------------------------------------------
  // Publishers
  // - /wordle_bot/motion_complete (std_msgs/Bool): Published after a successful mission execution.
  // - /wordle_bot/goal_reached (std_msgs/Bool): Published after reaching each individual goal in a mission.
  // - /wordle_bot/mission_complete (std_msgs/Bool): Published after completing all goals in a mission.
  // - /wordle_bot/robot_state (std_msgs/String): Published with "IDLE" or "RUNNING".

  // Subscriptions
  // - /wordle_bot/set_mission (geometry_msgs/PoseArray): Queue N goal poses; execution starts on /start_mission.
  // - /wordle_bot/start_mission (std_msgs/Bool): Plan and execute the queued mission (goal poses or pick-and-place).
  // - /wordle_bot/stop_mission (std_msgs/Bool): Stop the current mission safely.
  // - /wordle_bot/resume_mission (std_msgs/Bool): Resume a stopped mission (not yet implemented).
  // - /wordle_bot/abort_mission (std_msgs/Bool): Abort the current mission (not yet implemented).
  // - /wordle_bot/add_collision_object (moveit_msgs/CollisionObject): Add or remove a collision object.
  // - /perception/letter_objects (wordlebot_control/PickPlaceTask): Trigger pick-and-place mode.
  // - /wordle_bot/clear_letter_objects (std_msgs/Bool): Remove all board collision objects and reset queue.
  // - /wordle_bot/clear_board_objects (std_msgs/Bool): Alias for clearing letters and distractors.
  // - /wordle_bot/open_gripper (std_msgs/Bool): Open the gripper (IDLE only).
  // - /wordle_bot/close_gripper (std_msgs/Bool): Close the gripper (IDLE only).
  // - /wordle_bot/return_home (std_msgs/Bool): Return arm to home position (IDLE only).
  // ---------------------------------------------------------------------------
  motion_complete_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/wordle_bot/motion_complete", 10);

  set_mission_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    "/wordle_bot/set_mission", 10,
    std::bind(&WordleBotControlNode::setMissionCallback, this, std::placeholders::_1));

  goal_reached_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/wordle_bot/goal_reached", 10);

  mission_complete_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/wordle_bot/mission_complete", 10);

  start_mission_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/start_mission", 10,
    std::bind(&WordleBotControlNode::startMissionCallback, this, std::placeholders::_1));

  stop_mission_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/stop_mission", 10,
    std::bind(&WordleBotControlNode::stopMissionCallback, this, std::placeholders::_1));

  resume_mission_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/resume_mission", 10,
    std::bind(&WordleBotControlNode::resumeMissionCallback, this, std::placeholders::_1));

  abort_mission_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/abort_mission", 10,
    std::bind(&WordleBotControlNode::abortMissionCallback, this, std::placeholders::_1));

  robot_state_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/wordle_bot/robot_state", 10);

  add_collision_object_sub_ = node_->create_subscription<moveit_msgs::msg::CollisionObject>(
    "/wordle_bot/add_collision_object", 10,
    std::bind(&WordleBotControlNode::collisionObjectCallback, this, std::placeholders::_1));

  letter_object_sub_ = node_->create_subscription<wordlebot_control::msg::PickPlaceTask>(
    "perception/letter_objects", 10,
    std::bind(&WordleBotControlNode::letterObjectCallback, this, std::placeholders::_1));

  clear_letter_objects_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/clear_letter_objects", 10,
    std::bind(&WordleBotControlNode::clearLetterObjectsCallback, this, std::placeholders::_1));

  clear_board_objects_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/clear_board_objects", 10,
    std::bind(&WordleBotControlNode::clearLetterObjectsCallback, this, std::placeholders::_1));

  open_gripper_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/open_gripper", 10,
    std::bind(&WordleBotControlNode::openGripperCallback, this, std::placeholders::_1));

  close_gripper_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/close_gripper", 10,
    std::bind(&WordleBotControlNode::closeGripperCallback, this, std::placeholders::_1));

  return_home_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/return_home", 10,
    std::bind(&WordleBotControlNode::returnHomeCallback, this, std::placeholders::_1));

  scan_and_sweep_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/scan_and_sweep", 10,
    std::bind(&WordleBotControlNode::scanAndSweepCallback, this, std::placeholders::_1));

  // Load controller runtime parameters from config/wordle_bot_controller.yaml.
  if (!node_->has_parameter("working_joints.shoulder_pan_joint"))
    node_->declare_parameter<double>("working_joints.shoulder_pan_joint",  0.5236);
  if (!node_->has_parameter("working_joints.shoulder_lift_joint"))
    node_->declare_parameter<double>("working_joints.shoulder_lift_joint", -2.0071);
  if (!node_->has_parameter("working_joints.elbow_joint"))
    node_->declare_parameter<double>("working_joints.elbow_joint",          0.8901);
  if (!node_->has_parameter("working_joints.wrist_1_joint"))
    node_->declare_parameter<double>("working_joints.wrist_1_joint",       -0.4887);
  if (!node_->has_parameter("working_joints.wrist_2_joint"))
    node_->declare_parameter<double>("working_joints.wrist_2_joint",       -1.5184);
  if (!node_->has_parameter("working_joints.wrist_3_joint"))
    node_->declare_parameter<double>("working_joints.wrist_3_joint",       -1.0647);

  // Guard each declaration: if the YAML was loaded via --params-file the parameters
  // are already declared by the time the constructor runs.
  if (!node_->has_parameter("scan_sweep_dwell_time"))
    node_->declare_parameter<double>("scan_sweep_dwell_time", 1.5);
  if (!node_->has_parameter("scan_sweep_pose_0"))
    node_->declare_parameter<std::vector<double>>("scan_sweep_pose_0", {0.0, 0.0, 0.5, 0.0, 1.5708, 0.0});
  if (!node_->has_parameter("scan_sweep_pose_1"))
    node_->declare_parameter<std::vector<double>>("scan_sweep_pose_1", {0.0, 0.0, 0.5, 0.0, 1.5708, 0.0});
  if (!node_->has_parameter("scan_sweep_pose_2"))
    node_->declare_parameter<std::vector<double>>("scan_sweep_pose_2", {0.0, 0.0, 0.5, 0.0, 1.5708, 0.0});
  if (!node_->has_parameter("scan_sweep_pose_3"))
    node_->declare_parameter<std::vector<double>>("scan_sweep_pose_3", {0.0, 0.0, 0.5, 0.0, 1.5708, 0.0});
  if (!node_->has_parameter("scan_sweep_pose_4"))
    node_->declare_parameter<std::vector<double>>("scan_sweep_pose_4", {0.0, 0.0, 0.5, 0.0, 1.5708, 0.0});
  if (!node_->has_parameter("scan_sweep_pose_5"))
    node_->declare_parameter<std::vector<double>>("scan_sweep_pose_5", {0.0, 0.0, 0.5, 0.0, 1.5708, 0.0});

  if (!node_->has_parameter("mtc_default_solution_target_count"))
    node_->declare_parameter<int>("mtc_default_solution_target_count", 15);

  if (!node_->has_parameter("pick_place.task_solution_target_count"))
    node_->declare_parameter<int>("pick_place.task_solution_target_count", 15);
  if (!node_->has_parameter("pick_place.backend"))
    node_->declare_parameter<std::string>("pick_place.backend", "move_group");
  if (!node_->has_parameter("pick_place.grasp_max_ik_solutions"))
    node_->declare_parameter<int>("pick_place.grasp_max_ik_solutions", 32);
  if (!node_->has_parameter("pick_place.place_max_ik_solutions"))
    node_->declare_parameter<int>("pick_place.place_max_ik_solutions", 32);
  if (!node_->has_parameter("pick_place.grasp_min_solution_distance"))
    node_->declare_parameter<double>("pick_place.grasp_min_solution_distance", 0.10);
  if (!node_->has_parameter("pick_place.place_min_solution_distance"))
    node_->declare_parameter<double>("pick_place.place_min_solution_distance", 0.10);
  if (!node_->has_parameter("pick_place.grasp_angle_delta"))
    node_->declare_parameter<double>("pick_place.grasp_angle_delta", M_PI / 12.0);
  if (!node_->has_parameter("pick_place.cartesian_velocity_scaling"))
    node_->declare_parameter<double>("pick_place.cartesian_velocity_scaling", 0.5);
  if (!node_->has_parameter("pick_place.cartesian_acceleration_scaling"))
    node_->declare_parameter<double>("pick_place.cartesian_acceleration_scaling", 0.5);
  if (!node_->has_parameter("pick_place.cartesian_step_size"))
    node_->declare_parameter<double>("pick_place.cartesian_step_size", 0.001);
  if (!node_->has_parameter("pick_place.retreat_min_fraction"))
    node_->declare_parameter<double>("pick_place.retreat_min_fraction", 0.0);
  if (!node_->has_parameter("pick_place.move_to_pick_timeout"))
    node_->declare_parameter<double>("pick_place.move_to_pick_timeout", 0.50);
  if (!node_->has_parameter("pick_place.move_to_place_timeout"))
    node_->declare_parameter<double>("pick_place.move_to_place_timeout", 0.50);
  if (!node_->has_parameter("pick_place.solution_wrist_spin_weight"))
    node_->declare_parameter<double>("pick_place.solution_wrist_spin_weight", 4.0);
  if (!node_->has_parameter("pick_place.solution_wrist_spin_reject_threshold"))
    node_->declare_parameter<double>("pick_place.solution_wrist_spin_reject_threshold", 7.0);
  if (!node_->has_parameter("pick_place.accept_solution_score_threshold"))
    node_->declare_parameter<double>("pick_place.accept_solution_score_threshold", 25.0);
  if (!node_->has_parameter("pick_place.place_yaw_tolerance"))
    node_->declare_parameter<double>("pick_place.place_yaw_tolerance", 0.0872665);
  if (!node_->has_parameter("pick_place.place_yaw_penalty_weight"))
    node_->declare_parameter<double>("pick_place.place_yaw_penalty_weight", 100.0);
  if (!node_->has_parameter("pick_place.approach_min_distance"))
    node_->declare_parameter<double>("pick_place.approach_min_distance", 0.05);
  if (!node_->has_parameter("pick_place.approach_max_distance"))
    node_->declare_parameter<double>("pick_place.approach_max_distance", 0.15);
  if (!node_->has_parameter("pick_place.lift_min_distance"))
    node_->declare_parameter<double>("pick_place.lift_min_distance", 0.05);
  if (!node_->has_parameter("pick_place.lift_max_distance"))
    node_->declare_parameter<double>("pick_place.lift_max_distance", 0.15);
  if (!node_->has_parameter("pick_place.retreat_min_distance"))
    node_->declare_parameter<double>("pick_place.retreat_min_distance", 0.03);
  if (!node_->has_parameter("pick_place.retreat_max_distance"))
    node_->declare_parameter<double>("pick_place.retreat_max_distance", 0.15);
  if (!node_->has_parameter("pick_place.grasp_z_offset"))
    node_->declare_parameter<double>("pick_place.grasp_z_offset", 0.01);
  if (!node_->has_parameter("pick_place.mgi_planning_timeout"))
    node_->declare_parameter<double>("pick_place.mgi_planning_timeout", 10.0);
  if (!node_->has_parameter("pick_place.mgi_planning_min_successes"))
    node_->declare_parameter<int>("pick_place.mgi_planning_min_successes", 1);
  if (!node_->has_parameter("pick_place.mgi_place_open_recovery_yaw_delta"))
    node_->declare_parameter<double>("pick_place.mgi_place_open_recovery_yaw_delta", M_PI / 2.0);

  scan_sweep_dwell_time_ = node_->get_parameter("scan_sweep_dwell_time").as_double();

  auto load_pose = [&](const std::string & param_name) -> geometry_msgs::msg::Pose {
    auto v = node_->get_parameter(param_name).as_double_array();
    geometry_msgs::msg::Pose pose;
    pose.position.x = v[0];
    pose.position.y = v[1];
    pose.position.z = v[2];
    tf2::Quaternion q;
    q.setRPY(v[3], v[4], v[5]);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  };
  scan_sweep_poses_[0] = load_pose("scan_sweep_pose_0");
  scan_sweep_poses_[1] = load_pose("scan_sweep_pose_1");
  scan_sweep_poses_[2] = load_pose("scan_sweep_pose_2");
  scan_sweep_poses_[3] = load_pose("scan_sweep_pose_3");
  scan_sweep_poses_[4] = load_pose("scan_sweep_pose_4");
  scan_sweep_poses_[5] = load_pose("scan_sweep_pose_5");

  letter_object_counter_ = 0;

  mission_thread_ = std::thread(&WordleBotControlNode::missionLoop, this);

  rclcpp::on_shutdown([this]() { cv_.notify_all(); });

  RCLCPP_INFO(LOGGER, "WordleBotControlNode initialised.");
}

WordleBotControlNode::~WordleBotControlNode()
{
  cv_.notify_all();
  if (mission_thread_.joinable()) {
    mission_thread_.join();
  }
}

// ---------------------------------------------------------------------------
// Mission Control Callbacks
// Arms, starts, stops, resumes, and aborts missions in response to ROS2
// topic messages on the /wordle_bot/ namespace.
// ---------------------------------------------------------------------------
// setMissionCallback    — queue N goal poses; arms the mission for /start_mission
// startMissionCallback  — plan and execute the queued mission
// stopMissionCallback   — safely halt current motion
// resumeMissionCallback — resume a stopped mission (not yet implemented)
// abortMissionCallback  — abort the current mission (not yet implemented)
// ---------------------------------------------------------------------------

void WordleBotControlNode::setMissionCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    goal_queue_.clear();
    for (const auto & pose : msg->poses) {
      goal_queue_.push_back(pose);
    }
    mission_armed_ = false;
  }
  RCLCPP_INFO(LOGGER, "Mission set with %zu goals. Waiting for start_mission.", msg->poses.size());
}

void WordleBotControlNode::startMissionCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;

  bool do_scene_reset = false;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (mission_running_) {
      RCLCPP_WARN(LOGGER, "startMissionCallback: mission already running — ignoring start.");
      return;
    }
    if (in_stopped_state_.load()) {
      RCLCPP_WARN(LOGGER,
        "startMissionCallback: robot is in STOPPED state — send resume or abort first.");
      return;
    }
    if (goal_queue_.empty() && pick_place_queue_.empty()) {
      RCLCPP_WARN(LOGGER,
        "startMissionCallback: goal queue is empty and no pick-and-place tasks — ignoring.");
      return;
    }
    mission_armed_ = true;
    do_scene_reset = pick_place_queue_.empty();
  }
  cv_.notify_one();
  RCLCPP_INFO(LOGGER, "Mission armed — execution will begin.");

  if (do_scene_reset) {
    controller_->clearCollisionScene();
    controller_->setupCollisionScene();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void WordleBotControlNode::stopMissionCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  if (in_stopped_state_.load()) {
    RCLCPP_WARN(LOGGER, "stopMissionCallback: already in STOPPED state — ignoring.");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!mission_running_) {
      RCLCPP_WARN(LOGGER, "stopMissionCallback: no mission running — ignoring.");
      return;
    }
    stop_requested_.store(true);  // Inside lock — atomic with mission_running_ check
  }
  controller_->stop();
  RCLCPP_INFO(LOGGER, "stopMissionCallback: stop issued — robot will halt safely.");
}

void WordleBotControlNode::resumeMissionCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  if (!in_stopped_state_.load()) {
    RCLCPP_WARN(LOGGER, "resumeMissionCallback: not in STOPPED state — ignoring.");
    return;
  }
  resume_requested_.store(true);
  stopped_cv_.notify_one();
  RCLCPP_INFO(LOGGER, "resumeMissionCallback: resume signal sent.");
}

void WordleBotControlNode::abortMissionCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  if (!in_stopped_state_.load()) {
    RCLCPP_WARN(LOGGER, "abortMissionCallback: not in STOPPED state — ignoring.");
    return;
  }
  abort_requested_.store(true);
  stopped_cv_.notify_one();
  RCLCPP_INFO(LOGGER, "abortMissionCallback: abort signal sent.");
}

// ---------------------------------------------------------------------------
// Object & Scene Management Callbacks
// Manage collision objects in the MoveIt planning scene, including generic
// environment objects and the letter objects used in the pick-and-place workflow.
// ---------------------------------------------------------------------------
// collisionObjectCallback    — forward an ADD / REMOVE / MOVE operation directly
//                              to the controller's planning scene interface
// letterObjectCallback       — receive a pick/place task, register the letter
//                              collision object, and queue the task
// clearLetterObjectsCallback — remove all tracked board collision objects from
//                              the planning scene and reset the task queue
// ---------------------------------------------------------------------------

void WordleBotControlNode::collisionObjectCallback(const moveit_msgs::msg::CollisionObject::SharedPtr msg)
{
  RCLCPP_INFO(LOGGER, "collisionObjectCallback: received object '%s' (operation=%d).",
    msg->id.c_str(), static_cast<int>(msg->operation));
  controller_->addCollisionObject(*msg);

  if (msg->id.empty()) {
    RCLCPP_WARN(LOGGER,
      "collisionObjectCallback: received collision object with empty id; not tracking it.");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (msg->operation == moveit_msgs::msg::CollisionObject::REMOVE) {
      tracked_scene_objects_.erase(msg->id);
    } else {
      tracked_scene_objects_[msg->id] = *msg;
    }
  }
}

void WordleBotControlNode::letterObjectCallback(const wordlebot_control::msg::PickPlaceTask::SharedPtr msg)
{
  const std::string incoming_frame =
    msg->pick_pose.header.frame_id.empty() ? "world" : msg->pick_pose.header.frame_id;

  RCLCPP_INFO(LOGGER,
    "letterObjectCallback: received pick=(%.3f, %.3f, %.3f) place=(%.3f, %.3f, %.3f) "
    "object_id='%s' frame='%s'.",
    msg->pick_pose.pose.position.x,
    msg->pick_pose.pose.position.y,
    msg->pick_pose.pose.position.z,
    msg->place_pose.position.x,
    msg->place_pose.position.y,
    msg->place_pose.position.z,
    msg->object_id.c_str(),
    incoming_frame.c_str());

  if (incoming_frame != "world") {
    RCLCPP_WARN(LOGGER,
      "letterObjectCallback: incoming frame is '%s', not 'world'. "
      "MTC planning scene assumes 'world' — verify the publisher transforms poses into 'world'.",
      incoming_frame.c_str());
  }

  // Use the object_id from the message if provided, otherwise generate one
  const std::string object_id =
    msg->object_id.empty()
    ? "letter_" + std::to_string(++letter_object_counter_)
    : msg->object_id;

  auto normalized_pick_pose = msg->pick_pose.pose;
  auto normalized_place_pose = msg->place_pose;
  normalizePoseOrientation(normalized_pick_pose, "letterObjectCallback pick");
  normalizePoseOrientation(normalized_place_pose, "letterObjectCallback place");
  RCLCPP_INFO(LOGGER,
    "letterObjectCallback: normalized task orientation for '%s': pick_yaw=%.3f rad, "
    "place_yaw=%.3f rad.",
    object_id.c_str(), yawFromPose(normalized_pick_pose), yawFromPose(normalized_place_pose));

  moveit_msgs::msg::CollisionObject co;
  co.id = object_id;
  co.header.frame_id = incoming_frame;
  co.header.stamp = node_->get_clock()->now();
  co.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.05};
  co.primitives.push_back(box);
  co.primitive_poses.push_back(normalized_pick_pose);

  WordleBotController::PickPlaceEntry entry;
  entry.pick_pose = normalized_pick_pose;
  entry.place_pose = normalized_place_pose;
  entry.collision_object = co;
  entry.object_id = object_id;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    pick_place_queue_.push_back(entry);
    tracked_letter_ids_.push_back(object_id);
  }

  RCLCPP_INFO(LOGGER,
    "letterObjectCallback: task queued as '%s' (pick-and-place queue size=%zu). "
    "Waiting for start_mission.",
    object_id.c_str(), pick_place_queue_.size());
}

void WordleBotControlNode::clearLetterObjectsCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;

  std::vector<std::string> ids_to_remove;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (mission_running_) {
      RCLCPP_WARN(LOGGER,
        "clearLetterObjectsCallback: mission is running — cannot clear board objects now.");
      return;
    }
    ids_to_remove = tracked_letter_ids_;
    std::unordered_set<std::string> ids_seen(ids_to_remove.begin(), ids_to_remove.end());
    for (const auto & [id, object] : tracked_scene_objects_) {
      (void)object;
      if (ids_seen.insert(id).second) {
        ids_to_remove.push_back(id);
      }
    }
    tracked_letter_ids_.clear();
    tracked_scene_objects_.clear();
    pick_place_queue_.clear();
    letter_object_counter_ = 0;
  }

  controller_->clearLetterObjects(ids_to_remove);
  RCLCPP_INFO(LOGGER,
    "clearLetterObjectsCallback: cleared %zu tracked board object(s) and reset queue.",
    ids_to_remove.size());
}

// ---------------------------------------------------------------------------
// Arm Utility Callbacks
// Standalone arm motions available while no mission is running (IDLE state).
// ---------------------------------------------------------------------------
// returnHomeCallback   — return the arm to its SRDF "home" named state
// openGripperCallback  — fully open the gripper to the SRDF "open" named state
// closeGripperCallback — fully close the gripper to the SRDF "closed" named state
// scanAndSweepCallback — execute the four-pose camera scan sweep sequence
// ---------------------------------------------------------------------------

void WordleBotControlNode::returnHomeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (mission_running_) {
      RCLCPP_WARN(LOGGER, "returnHomeCallback: mission running — ignoring.");
      return;
    }
  }
  RCLCPP_INFO(LOGGER, "returnHomeCallback: returning to home.");
  controller_->returnToHome();
}

void WordleBotControlNode::openGripperCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (mission_running_) {
      RCLCPP_WARN(LOGGER, "openGripperCallback: mission running — ignoring.");
      return;
    }
  }
  RCLCPP_INFO(LOGGER, "openGripperCallback: opening gripper (full).");
  controller_->openGripperFull();
}

void WordleBotControlNode::closeGripperCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (mission_running_) {
      RCLCPP_WARN(LOGGER, "closeGripperCallback: mission running — ignoring.");
      return;
    }
  }
  RCLCPP_INFO(LOGGER, "closeGripperCallback: closing gripper (full).");
  controller_->closeGripperFull();
}

void WordleBotControlNode::scanAndSweepCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (mission_running_) {
      RCLCPP_WARN(LOGGER, "scanAndSweepCallback: mission running — ignoring.");
      return;
    }
    mission_running_ = true;
  }

  RCLCPP_INFO(LOGGER, "scanAndSweepCallback: starting scan-and-sweep sequence.");

  std_msgs::msg::String state_msg;
  state_msg.data = "RUNNING";
  robot_state_pub_->publish(state_msg);

  auto poses = scan_sweep_poses_;
  double dwell = scan_sweep_dwell_time_;

  std::thread([this, poses, dwell]() {
    controller_->runScanAndSweep(
      std::vector<geometry_msgs::msg::Pose>(poses.begin(), poses.end()), dwell);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      mission_running_ = false;
    }
    std_msgs::msg::String s;
    s.data = "IDLE";
    robot_state_pub_->publish(s);
    RCLCPP_INFO(rclcpp::get_logger("WordleBotControlNode"),
      "scanAndSweepCallback: sequence complete, returning to IDLE.");
  }).detach();
}

// ---------------------------------------------------------------------------
// Mission Execution Loop
// Background thread that wakes on mission_armed_, dispatches to either a
// goal-pose mission or a pick-and-place batch, then publishes completion
// signals. Uses a plan-then-execute strategy so all tasks are planned before
// any motion begins, enabling planning scene chaining across tasks.
// ---------------------------------------------------------------------------
// missionLoop — blocks on condition variable; on wake dispatches to
//               goal-pose execution or pick-and-place batch execution
// ---------------------------------------------------------------------------

void WordleBotControlNode::missionLoop()
{
  RCLCPP_INFO(LOGGER, "Mission thread started.");
  while (rclcpp::ok()) {
    bool do_pick_and_place = false;
    std::vector<geometry_msgs::msg::Pose> current_mission;
    std::vector<WordleBotController::PickPlaceEntry> current_tasks;
    std::vector<moveit_msgs::msg::CollisionObject> current_scene_objects;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      cv_.wait(lock, [this]() {
        return (mission_armed_ && (!goal_queue_.empty() || !pick_place_queue_.empty())) ||
               !rclcpp::ok();
      });
      if (!rclcpp::ok()) break;

      do_pick_and_place = !pick_place_queue_.empty();
      if (do_pick_and_place) {
        current_tasks = std::move(pick_place_queue_);
        pick_place_queue_.clear();
        current_scene_objects.reserve(tracked_scene_objects_.size());
        for (const auto & [id, object] : tracked_scene_objects_) {
          (void)id;
          current_scene_objects.push_back(object);
        }
      } else {
        current_mission = std::move(goal_queue_);
        goal_queue_.clear();
      }
      mission_armed_ = false;
      stop_requested_.store(false);
      controller_->clearStopFlag();
      mission_running_ = true;
    }

    {
      std_msgs::msg::String state_msg;
      state_msg.data = "RUNNING";
      robot_state_pub_->publish(state_msg);
    }

    std_msgs::msg::Bool signal;
    signal.data = true;

    // Index into current_tasks / current_mission where execution was interrupted.
    // Defaults to 0 so a planning-phase stop re-queues all tasks on resume.
    std::size_t pp_resume_from   = 0;
    std::size_t goal_resume_from = 0;

    if (do_pick_and_place) {
      RCLCPP_INFO(LOGGER, "Mission: %zu pick-and-place task(s) queued.", current_tasks.size());

      // ── Scene setup: add ALL collision objects before any planning ──────────
      // All objects must be present in the live scene so that CurrentState (task 1)
      // and the chained FixedState scenes (tasks 2..N) all see the full workspace.
      RCLCPP_INFO(LOGGER,
        "Mission: adding %zu externally supplied collision object(s) to scene before planning.",
        current_scene_objects.size());
      std::unordered_set<std::string> scene_object_ids;
      scene_object_ids.reserve(current_scene_objects.size() + current_tasks.size());
      for (const auto & object : current_scene_objects) {
        controller_->addCollisionObject(object);
        scene_object_ids.insert(object.id);
        RCLCPP_INFO(LOGGER, "  Added scene collision object '%s'.", object.id.c_str());
      }

      RCLCPP_INFO(LOGGER,
        "Mission: adding all %zu pick target collision object(s) to scene before planning.",
        current_tasks.size());
      for (const auto & entry : current_tasks) {
        if (scene_object_ids.find(entry.object_id) != scene_object_ids.end()) {
          RCLCPP_INFO(LOGGER,
            "  Pick target '%s' already supplied as a scene collision object.",
            entry.object_id.c_str());
          continue;
        }
        controller_->addCollisionObject(entry.collision_object);
        scene_object_ids.insert(entry.object_id);
        RCLCPP_INFO(LOGGER, "  Added collision object '%s'.", entry.object_id.c_str());
      }

      std::string pick_place_backend =
        node_->get_parameter("pick_place.backend").as_string();
      if (pick_place_backend != "mtc" && pick_place_backend != "move_group") {
        RCLCPP_WARN(LOGGER,
          "Mission: unsupported pick_place.backend='%s'; defaulting to 'move_group'.",
          pick_place_backend.c_str());
        pick_place_backend = "move_group";
      }

      if (pick_place_backend == "mtc") {
        RCLCPP_INFO(LOGGER,
          "Mission: pick-and-place backend is MTC — planning all tasks before execution.");

      // ── Phase 1: Plan all tasks sequentially, chaining via FixedState ───────
      std::vector<WordleBotController::PlannedPickPlace> planned_tasks;
      planned_tasks.reserve(current_tasks.size());
      bool planning_failed = false;

      for (std::size_t i = 0; i < current_tasks.size(); ++i) {
        if (stop_requested_.load()) {
          RCLCPP_INFO(LOGGER, "Planning phase: stop requested before task %zu — aborting.", i + 1);
          planning_failed = true;
          break;
        }

        const bool is_last = (i == current_tasks.size() - 1);

        // Task 0 starts from the live robot (nullptr = CurrentState).
        // Tasks 1..N start from the terminal scene of the previous planned task.
        // Type must be explicit — nullptr and PlanningScenePtr cannot unify in a ternary.
        planning_scene::PlanningScenePtr start_scene =
            (i == 0) ? nullptr : planned_tasks.back().end_scene;

        RCLCPP_INFO(LOGGER,
          "Planning task %zu of %zu: '%s' (return_home=%s).",
          i + 1, current_tasks.size(), current_tasks[i].object_id.c_str(),
          is_last ? "yes" : "no");

        WordleBotController::PlannedPickPlace planned =
            controller_->planPickAndPlace(current_tasks[i], start_scene, is_last);

        if (!planned.task) {
          RCLCPP_ERROR(LOGGER,
            "Planning FAILED for task %zu ('%s') — aborting remaining planning.",
            i + 1, current_tasks[i].object_id.c_str());
          planning_failed = true;
          break;
        }

        RCLCPP_INFO(LOGGER, "Planning succeeded for task %zu of %zu.", i + 1, current_tasks.size());
        planned_tasks.emplace_back(std::move(planned));
      }

      if (planned_tasks.empty()) {
        RCLCPP_ERROR(LOGGER, "Mission: no tasks were successfully planned — aborting.");
      } else {
        if (planning_failed) {
          RCLCPP_WARN(LOGGER,
            "Mission: %zu of %zu tasks planned — executing partial mission.",
            planned_tasks.size(), current_tasks.size());
        } else {
          RCLCPP_INFO(LOGGER,
            "Mission: all %zu tasks planned successfully — executing.", planned_tasks.size());
        }

        // ── Phase 2: Execute all pre-planned solutions in order ──────────────
        bool all_ok = true;
        for (std::size_t i = 0; i < planned_tasks.size(); ++i) {
          if (stop_requested_.load()) {
            RCLCPP_INFO(LOGGER,
              "Execution phase: stop requested before task %zu — aborting.", i + 1);
            pp_resume_from = i;
            all_ok = false;
            break;
          }

          RCLCPP_INFO(LOGGER, "Executing pre-planned task %zu of %zu: '%s'.",
            i + 1, planned_tasks.size(), planned_tasks[i].object_id.c_str());

          if (!controller_->executePlannedTask(planned_tasks[i]) || stop_requested_.load()) {
            RCLCPP_ERROR(LOGGER,
              "Execution FAILED at task %zu ('%s') — stopping.",
              i + 1, planned_tasks[i].object_id.c_str());
            pp_resume_from = i;
            all_ok = false;
            break;
          }

          goal_reached_pub_->publish(signal);
          RCLCPP_INFO(LOGGER, "Task %zu of %zu execution complete.", i + 1, planned_tasks.size());
        }

        if (all_ok && !planning_failed && !stop_requested_.load()) {
          mission_complete_pub_->publish(signal);
          motion_complete_pub_->publish(signal);
          RCLCPP_INFO(LOGGER, "Pick-and-place mission fully complete.");
        } else {
          RCLCPP_ERROR(LOGGER, "Pick-and-place mission did not complete fully.");
        }
      }
      } else {
        RCLCPP_INFO(LOGGER,
          "Mission: pick-and-place backend is MoveGroupInterface — planning and executing each task live.");

        bool all_ok = true;
        for (std::size_t i = 0; i < current_tasks.size(); ++i) {
          if (stop_requested_.load()) {
            RCLCPP_INFO(LOGGER,
              "Pick-and-place mission (MGI): stop requested before task %zu — aborting.",
              i + 1);
            pp_resume_from = i;
            all_ok = false;
            break;
          }

          const bool is_last = (i == current_tasks.size() - 1);
          RCLCPP_INFO(LOGGER,
            "Pick-and-place mission (MGI): executing task %zu of %zu: '%s' (return_working=%s).",
            i + 1, current_tasks.size(), current_tasks[i].object_id.c_str(),
            is_last ? "yes" : "no");

          if (!controller_->executePickAndPlaceMoveGroup(current_tasks[i], is_last) ||
              stop_requested_.load()) {
            RCLCPP_ERROR(LOGGER,
              "Pick-and-place mission (MGI): FAILED at task %zu ('%s').",
              i + 1, current_tasks[i].object_id.c_str());
            pp_resume_from = i;
            all_ok = false;
            break;
          }

          goal_reached_pub_->publish(signal);
          RCLCPP_INFO(LOGGER,
            "Pick-and-place mission (MGI): task %zu of %zu complete.",
            i + 1, current_tasks.size());
        }

        if (all_ok && !stop_requested_.load()) {
          mission_complete_pub_->publish(signal);
          motion_complete_pub_->publish(signal);
          RCLCPP_INFO(LOGGER, "Pick-and-place mission (MGI): fully complete.");
        } else {
          RCLCPP_ERROR(LOGGER, "Pick-and-place mission (MGI): did not complete fully.");
        }
      }

    } else {
      RCLCPP_INFO(LOGGER, "Goal mission: %zu goal(s) queued.", current_mission.size());

      if constexpr (WordleBotController::USE_MTC_FOR_GOALS) {
        // ── MTC path: plan-all-then-execute-all ──────────────────────────────

        // ── Phase 1: Plan all goals sequentially, chaining via FixedState ────
        std::vector<WordleBotController::PlannedMoveToGoal> planned_goals;
        planned_goals.reserve(current_mission.size());
        bool planning_failed = false;

        for (std::size_t i = 0; i < current_mission.size(); ++i) {
          if (stop_requested_.load()) {
            RCLCPP_INFO(LOGGER, "Goal planning: stop requested before goal %zu — aborting.", i + 1);
            planning_failed = true;
            break;
          }

          const bool is_last = (i == current_mission.size() - 1);
          planning_scene::PlanningScenePtr start_scene =
              (i == 0) ? nullptr : planned_goals.back().end_scene;

          RCLCPP_INFO(LOGGER, "Planning goal %zu of %zu (return_home=%s).",
            i + 1, current_mission.size(), is_last ? "yes" : "no");

          WordleBotController::PlannedMoveToGoal planned =
              controller_->planMoveToGoal(current_mission[i], start_scene, is_last);

          if (!planned.task) {
            RCLCPP_ERROR(LOGGER, "Planning FAILED for goal %zu — aborting.", i + 1);
            planning_failed = true;
            break;
          }

          RCLCPP_INFO(LOGGER, "Planning succeeded for goal %zu of %zu.", i + 1, current_mission.size());
          planned_goals.emplace_back(std::move(planned));
        }

        if (planned_goals.empty()) {
          RCLCPP_ERROR(LOGGER, "Goal mission: no goals were successfully planned — aborting.");
        } else {
          if (planning_failed) {
            RCLCPP_WARN(LOGGER, "Goal mission: %zu of %zu goals planned — executing partial mission.",
              planned_goals.size(), current_mission.size());
          } else {
            RCLCPP_INFO(LOGGER, "Goal mission: all %zu goals planned — executing.", planned_goals.size());
          }

          // ── Phase 2: Execute all pre-planned goals in order ────────────────
          bool all_ok = true;
          for (std::size_t i = 0; i < planned_goals.size(); ++i) {
            if (stop_requested_.load()) {
              RCLCPP_INFO(LOGGER, "Goal execution: stop requested before goal %zu — aborting.", i + 1);
              goal_resume_from = i;
              all_ok = false;
              break;
            }

            RCLCPP_INFO(LOGGER, "Executing goal %zu of %zu.", i + 1, planned_goals.size());

            if (!controller_->executePlannedMoveToGoal(planned_goals[i]) || stop_requested_.load()) {
              RCLCPP_ERROR(LOGGER, "Execution FAILED at goal %zu — stopping.", i + 1);
              goal_resume_from = i;
              all_ok = false;
              break;
            }

            goal_reached_pub_->publish(signal);
            RCLCPP_INFO(LOGGER, "Goal %zu of %zu reached.", i + 1, planned_goals.size());
          }

          if (all_ok && !planning_failed && !stop_requested_.load()) {
            mission_complete_pub_->publish(signal);
            motion_complete_pub_->publish(signal);
            RCLCPP_INFO(LOGGER, "Goal mission fully complete.");
          } else {
            RCLCPP_ERROR(LOGGER, "Goal mission did not complete fully.");
          }
        }

      } else {
        // ── MoveGroupInterface path: sequential plan+execute per goal ─────────

        bool all_ok = true;
        for (std::size_t i = 0; i < current_mission.size(); ++i) {
          if (stop_requested_.load()) {
            RCLCPP_INFO(LOGGER, "Goal mission (MGI): stop requested before goal %zu — aborting.", i + 1);
            goal_resume_from = i;
            all_ok = false;
            break;
          }

          RCLCPP_INFO(LOGGER, "Goal mission (MGI): executing goal %zu of %zu.",
            i + 1, current_mission.size());

          if (!controller_->moveToGoal(current_mission[i]) || stop_requested_.load()) {
            RCLCPP_ERROR(LOGGER, "Goal mission (MGI): FAILED at goal %zu.", i + 1);
            goal_resume_from = i;
            all_ok = false;
            break;
          }

          goal_reached_pub_->publish(signal);
          RCLCPP_INFO(LOGGER, "Goal mission (MGI): goal %zu of %zu reached.",
            i + 1, current_mission.size());
        }

        if (all_ok && !stop_requested_.load()) {
          RCLCPP_INFO(LOGGER, "Goal mission (MGI): returning to working pose.");
          controller_->returnToWorkingPose();
          mission_complete_pub_->publish(signal);
          motion_complete_pub_->publish(signal);
          RCLCPP_INFO(LOGGER, "Goal mission (MGI): fully complete.");
        } else {
          RCLCPP_ERROR(LOGGER, "Goal mission (MGI): did not complete fully.");
        }
      }
    }

    // ── Atomically decide: enter STOPPED state or complete normally ─────────────
    // stop_requested_ is set inside queue_mutex_ by stopMissionCallback, so checking
    // it here under the same lock is race-free with the mission_running_ cleanup.
    bool should_stop = false;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      should_stop = stop_requested_.load();
      if (!should_stop) {
        mission_running_ = false;  // Normal completion
      } else {
        // Signal callbacks immediately so resume/abort are accepted without delay.
        // in_stopped_state_ is set before isGripperClosed() (which can block ~2 s)
        // so the operator can send resume/abort the instant they see "STOPPED".
        in_stopped_state_.store(true);
      }
    }

    if (should_stop) {
      // ── STOPPED state handler ────────────────────────────────────────────────
      {
        std_msgs::msg::String s;
        s.data = "STOPPED";
        robot_state_pub_->publish(s);
      }

      // Snapshot gripper state (robot is halted at this point).
      bool gripper_closed = controller_->isGripperClosed();

      // Save remaining unexecuted tasks so resume can re-execute them.
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (do_pick_and_place) {
          resume_pick_tasks_.assign(
            current_tasks.begin() + pp_resume_from, current_tasks.end());
        } else {
          resume_goal_tasks_.assign(
            current_mission.begin() + goal_resume_from, current_mission.end());
        }
      }

      RCLCPP_INFO(LOGGER,
        "missionLoop: STOPPED (gripper_closed=%s, %zu pick tasks / %zu goal tasks remain).",
        gripper_closed ? "yes" : "no",
        resume_pick_tasks_.size(), resume_goal_tasks_.size());

      // Block until resume or abort (releases queue_mutex_ while waiting).
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        stopped_cv_.wait(lock, [this]() {
          return resume_requested_.load() || abort_requested_.load() || !rclcpp::ok();
        });
      }

      in_stopped_state_.store(false);
      if (!rclcpp::ok()) break;

      // ── Abort path ────────────────────────────────────────────────────────────
      if (abort_requested_.exchange(false)) {
        RCLCPP_INFO(LOGGER, "missionLoop: abort — clearing queues and returning to home.");
        controller_->clearStopFlag();
        stop_requested_.store(false);
        controller_->returnToHome();
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          pick_place_queue_.clear();
          goal_queue_.clear();
          resume_pick_tasks_.clear();
          resume_goal_tasks_.clear();
          std::vector<std::string> ids_to_remove = tracked_letter_ids_;
          std::unordered_set<std::string> ids_seen(ids_to_remove.begin(), ids_to_remove.end());
          for (const auto & [id, object] : tracked_scene_objects_) {
            (void)object;
            if (ids_seen.insert(id).second) {
              ids_to_remove.push_back(id);
            }
          }
          controller_->clearLetterObjects(ids_to_remove);
          tracked_letter_ids_.clear();
          tracked_scene_objects_.clear();
          letter_object_counter_ = 0;
          mission_running_ = false;
          mission_armed_   = false;
        }
        {
          std_msgs::msg::String s;
          s.data = "IDLE";
          robot_state_pub_->publish(s);
        }
        continue;
      }

      // ── Resume path ──────────────────────────────────────────────────────────
      resume_requested_.store(false);
      controller_->clearStopFlag();
      stop_requested_.store(false);

      if (gripper_closed) {
        RCLCPP_INFO(LOGGER, "missionLoop: gripper was closed — running recoverObject().");
        std::string held_id;
        if (do_pick_and_place && !resume_pick_tasks_.empty()) {
          held_id = resume_pick_tasks_.front().object_id;
        }
        controller_->recoverObject(held_id);
        // The object in the gripper belonged to resume_pick_tasks_[0]; skip it.
        if (do_pick_and_place && !resume_pick_tasks_.empty()) {
          resume_pick_tasks_.erase(resume_pick_tasks_.begin());
        }
      }

      bool has_remaining = false;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (do_pick_and_place && !resume_pick_tasks_.empty()) {
          // Rebuild collision scene for remaining letter objects only.
          controller_->clearLetterObjects(tracked_letter_ids_);
          tracked_letter_ids_.clear();
          letter_object_counter_ = 0;
          pick_place_queue_ = std::move(resume_pick_tasks_);
          for (const auto & t : pick_place_queue_) {
            if (tracked_scene_objects_.find(t.object_id) == tracked_scene_objects_.end()) {
              controller_->addCollisionObject(t.collision_object);
            }
            tracked_letter_ids_.push_back(t.object_id);
          }
          mission_armed_ = true;
          has_remaining  = true;
        } else if (!do_pick_and_place && !resume_goal_tasks_.empty()) {
          goal_queue_    = std::move(resume_goal_tasks_);
          mission_armed_ = true;
          has_remaining  = true;
        }

        if (!has_remaining) {
          mission_running_ = false;
          mission_armed_   = false;
        }
      }

      if (has_remaining) {
        RCLCPP_INFO(LOGGER, "missionLoop: re-arming for remaining tasks after resume.");
        // mission_running_ stays true; CV predicate satisfied immediately on continue.
        continue;
      }
      // No remaining tasks: fall through to IDLE publish below.
    }
    // ── End STOPPED state handler / normal completion ─────────────────────────

    {
      std_msgs::msg::String state_msg;
      state_msg.data = "IDLE";
      robot_state_pub_->publish(state_msg);
    }
  }
  RCLCPP_INFO(LOGGER, "Mission thread exiting.");
}

// ---------------------------------------------------------------------------
// Public Interface
// External entry points used by the ROS2 component wrapper and the
// executable main() to query node internals and drive the node.
// ---------------------------------------------------------------------------
// getNodeBaseInterface — return the underlying rclcpp::Node base interface
//                        required by the component manager
// setupScene           — clear and rebuild the static collision scene
// run                  — spin the node until ROS shutdown
// ---------------------------------------------------------------------------

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
WordleBotControlNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void WordleBotControlNode::setupScene()
{
  controller_->clearCollisionScene();
  controller_->setupCollisionScene();
}

void WordleBotControlNode::run()
{
  RCLCPP_INFO(LOGGER, "run(): mission interface active — waiting for goals.");
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(LOGGER, "run(): shutdown detected, joining mission thread.");
  if (mission_thread_.joinable()) {
    mission_thread_.join();
  }
  RCLCPP_INFO(LOGGER, "run(): mission thread joined.");
}
