#include "wordleBot_control/wordle_bot_control_node.hpp"

#include <chrono>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("WordleBotControlNode");

WordleBotControlNode::WordleBotControlNode(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("wordle_bot_control_node", options)),
  controller_(std::make_shared<WordleBotController>(node_))
{
  // Legacy single-goal interface (backward compat): goal_pose auto-arms the mission
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/wordle_bot/goal_pose", 10,
    std::bind(&WordleBotControlNode::goalCallback, this, std::placeholders::_1));

  motion_complete_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/wordle_bot/motion_complete", 10);

  // Mission-level interface
  set_mission_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    "/wordle_bot/set_mission", 10,
    std::bind(&WordleBotControlNode::setMissionCallback, this, std::placeholders::_1));

  start_mission_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/start_mission", 10,
    std::bind(&WordleBotControlNode::startMissionCallback, this, std::placeholders::_1));

  goal_reached_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/wordle_bot/goal_reached", 10);

  mission_complete_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/wordle_bot/mission_complete", 10);

  // Stop / Resume / Abort control
  stop_mission_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/stop_mission", 10,
    std::bind(&WordleBotControlNode::stopMissionCallback, this, std::placeholders::_1));

  resume_mission_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/resume_mission", 10,
    std::bind(&WordleBotControlNode::resumeMissionCallback, this, std::placeholders::_1));

  abort_mission_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/wordle_bot/abort_mission", 10,
    std::bind(&WordleBotControlNode::abortMissionCallback, this, std::placeholders::_1));

  // Live state
  robot_state_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/wordle_bot/robot_state", 10);

  add_collision_object_sub_ = node_->create_subscription<moveit_msgs::msg::CollisionObject>(
    "/wordle_bot/add_collision_object", 10,
    std::bind(&WordleBotControlNode::collisionObjectCallback, this, std::placeholders::_1));

  // Letter object interface: receives object pose and triggers pick-and-place mode
  letter_object_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "perception/letter_objects", 10,
    std::bind(&WordleBotControlNode::letterObjectCallback, this, std::placeholders::_1));

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
// Subscriber callbacks
// ---------------------------------------------------------------------------

void WordleBotControlNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    goal_queue_.push_back(msg->pose);
    mission_armed_ = true;
  }
  cv_.notify_one();
  RCLCPP_INFO(LOGGER, "Goal enqueued and armed. Queue size: %zu", goal_queue_.size());
}

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
    if (robot_state_ != RobotState::IDLE) {
      RCLCPP_WARN(LOGGER,
        "startMissionCallback: robot is %s — ignoring start.",
        robotStateToString(robot_state_).c_str());
      return;
    }
    if (goal_queue_.empty() && !letter_object_received_) {
      RCLCPP_WARN(LOGGER,
        "startMissionCallback: goal queue is empty and no letter object — ignoring.");
      return;
    }
    mission_armed_ = true;
    do_scene_reset = !letter_object_received_;
  }
  cv_.notify_one();
  RCLCPP_INFO(LOGGER, "Mission armed — execution will begin.");

  // Only reset the collision scene for waypoint missions; pick-and-place keeps the letter object
  if (do_scene_reset) {
    controller_->clearCollisionScene();
    controller_->setupCollisionScene();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void WordleBotControlNode::stopMissionCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (robot_state_ != RobotState::RUNNING) {
      RCLCPP_WARN(LOGGER, "stopMissionCallback: not RUNNING (state=%s) — ignoring.",
        robotStateToString(robot_state_).c_str());
      return;
    }
  }
  stop_requested_.store(true);
  controller_->stop();
  RCLCPP_INFO(LOGGER, "stopMissionCallback: stop issued — robot will halt safely.");
}

void WordleBotControlNode::resumeMissionCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (robot_state_ != RobotState::STOPPED) {
      RCLCPP_WARN(LOGGER, "resumeMissionCallback: not STOPPED (state=%s) — ignoring.",
        robotStateToString(robot_state_).c_str());
      return;
    }
    resume_requested_ = true;
  }
  cv_.notify_one();
  RCLCPP_INFO(LOGGER, "resumeMissionCallback: resume signalled.");
}

void WordleBotControlNode::abortMissionCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (robot_state_ == RobotState::IDLE || robot_state_ == RobotState::ABORTING) {
      RCLCPP_WARN(LOGGER, "abortMissionCallback: already %s — ignoring.",
        robotStateToString(robot_state_).c_str());
      return;
    }
    abort_requested_ = true;
  }
  stop_requested_.store(true);
  controller_->stop();
  cv_.notify_one();  // wake mission thread if it is already waiting in STOPPED state
  RCLCPP_INFO(LOGGER, "abortMissionCallback: abort issued — robot will return home.");
}

void WordleBotControlNode::collisionObjectCallback(
  const moveit_msgs::msg::CollisionObject::SharedPtr msg)
{
  RCLCPP_INFO(LOGGER, "collisionObjectCallback: received object '%s' (operation=%d).",
    msg->id.c_str(), static_cast<int>(msg->operation));
  controller_->addCollisionObject(*msg);
}

void WordleBotControlNode::letterObjectCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const std::string incoming_frame = msg->header.frame_id.empty() ? "world" : msg->header.frame_id;
  RCLCPP_INFO(LOGGER,
    "letterObjectCallback: received object at (%.3f, %.3f, %.3f) in frame '%s'.",
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
    incoming_frame.c_str());
  if (incoming_frame != "world") {
    RCLCPP_WARN(LOGGER,
      "letterObjectCallback: incoming frame is '%s', not 'world'. "
      "MTC planning scene assumes 'world' — this may cause a frame mismatch and "
      "incorrect grasp height. Verify that the publisher transforms poses into 'world'.",
      incoming_frame.c_str());
  }

  // Build a 40 mm cube collision object at the received pose
  moveit_msgs::msg::CollisionObject co;
  co.id = WordleBotController::LETTER_OBJECT_ID;
  co.header.frame_id = incoming_frame;
  co.header.stamp = node_->get_clock()->now();
  co.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.04, 0.04, 0.04};
  co.primitives.push_back(box);
  co.primitive_poses.push_back(msg->pose);

  controller_->addCollisionObject(co);

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    letter_object_pose_ = msg->pose;
    letter_object_received_ = true;
  }

  RCLCPP_INFO(LOGGER,
    "letterObjectCallback: '%s' added to planning scene — ready for pick-and-place.",
    WordleBotController::LETTER_OBJECT_ID);
}

// ---------------------------------------------------------------------------
// State helpers
// ---------------------------------------------------------------------------

std::string WordleBotControlNode::robotStateToString(RobotState s)
{
  switch (s) {
    case RobotState::IDLE:     return "IDLE";
    case RobotState::RUNNING:  return "RUNNING";
    case RobotState::STOPPED:  return "STOPPED";
    case RobotState::ABORTING: return "ABORTING";
  }
  return "UNKNOWN";
}

void WordleBotControlNode::publishRobotState()
{
  std_msgs::msg::String msg;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    msg.data = robotStateToString(robot_state_);
  }
  robot_state_pub_->publish(msg);
  RCLCPP_INFO(LOGGER, "Robot state: %s", msg.data.c_str());
}

void WordleBotControlNode::resetMissionState()
{
  // Must be called under queue_mutex_
  execution_mode_       = ExecutionMode::NONE;
  pick_place_phase_     = PickPlacePhase::NONE;
  current_waypoint_idx_ = 0;
  current_mission_.clear();
  letter_object_received_ = false;
  resume_requested_     = false;
  abort_requested_      = false;
  mission_armed_        = false;
  stop_requested_.store(false);
}

// ---------------------------------------------------------------------------
// Mission loop
// ---------------------------------------------------------------------------

void WordleBotControlNode::missionLoop()
{
  RCLCPP_INFO(LOGGER, "Mission thread started.");

  while (rclcpp::ok()) {

    // ── Wait for a fresh armed mission from IDLE ─────────────────────────────
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      cv_.wait(lock, [this]() {
        return (robot_state_ == RobotState::IDLE && mission_armed_ &&
                (!goal_queue_.empty() || letter_object_received_)) ||
               !rclcpp::ok();
      });
      if (!rclcpp::ok()) break;

      execution_mode_       = letter_object_received_ ? ExecutionMode::PICK_PLACE
                                                      : ExecutionMode::WAYPOINTS;
      pick_place_phase_     = PickPlacePhase::NONE;
      current_waypoint_idx_ = 0;

      if (execution_mode_ == ExecutionMode::WAYPOINTS) {
        current_mission_ = std::move(goal_queue_);
        goal_queue_.clear();
      }

      mission_armed_    = false;
      resume_requested_ = false;
      abort_requested_  = false;
      stop_requested_.store(false);
      controller_->clearStopFlag();

      robot_state_ = RobotState::RUNNING;
    }
    publishRobotState();

    // ── Execute (label re-entered on resume) ─────────────────────────────────
    RESUME_POINT:

    if (execution_mode_ == ExecutionMode::WAYPOINTS) {
      RCLCPP_INFO(LOGGER, "Mission thread: executing %zu waypoint(s) from index %zu.",
        current_mission_.size(), current_waypoint_idx_);
      runWaypointMission();
    } else {
      RCLCPP_INFO(LOGGER, "Mission thread: pick-and-place (phase=%d).",
        static_cast<int>(pick_place_phase_));
      runPickAndPlaceMission();
    }

    // ── Outcome: stopped or completed ────────────────────────────────────────
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);

      if (robot_state_ == RobotState::STOPPED) {
        lock.unlock();
        publishRobotState();
        lock.lock();

        // Wait for resume or abort
        cv_.wait(lock, [this]() {
          return resume_requested_ || abort_requested_ || !rclcpp::ok();
        });
        if (!rclcpp::ok()) break;

        if (abort_requested_) {
          abort_requested_ = false;
          robot_state_ = RobotState::ABORTING;
          lock.unlock();
          publishRobotState();
          doAbort();
          lock.lock();
          resetMissionState();
          robot_state_ = RobotState::IDLE;
          lock.unlock();
          publishRobotState();
          std_msgs::msg::Bool sig;
          sig.data = false;
          mission_complete_pub_->publish(sig);
          continue;
        }

        if (resume_requested_) {
          resume_requested_ = false;
          stop_requested_.store(false);
          controller_->clearStopFlag();
          robot_state_ = RobotState::RUNNING;
          lock.unlock();
          publishRobotState();
          goto RESUME_POINT;
        }
      }

      // Normal completion
      resetMissionState();
      robot_state_ = RobotState::IDLE;
    }
    publishRobotState();
  }

  RCLCPP_INFO(LOGGER, "Mission thread exiting.");
}

// ---------------------------------------------------------------------------
// Execution helpers (run on mission_thread_)
// ---------------------------------------------------------------------------

void WordleBotControlNode::runWaypointMission()
{
  const std::size_t n = current_mission_.size();
  std_msgs::msg::Bool sig;
  sig.data = true;

  while (current_waypoint_idx_ < n) {
    if (stop_requested_.load()) {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      robot_state_ = RobotState::STOPPED;
      RCLCPP_INFO(LOGGER, "runWaypointMission: STOPPED before waypoint %zu.", current_waypoint_idx_);
      return;
    }

    RCLCPP_INFO(LOGGER, "Executing waypoint %zu/%zu.", current_waypoint_idx_ + 1, n);
    const bool ok = controller_->moveToTarget(current_mission_[current_waypoint_idx_]);

    if (!ok || stop_requested_.load()) {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      robot_state_ = RobotState::STOPPED;
      RCLCPP_INFO(LOGGER, "runWaypointMission: STOPPED at waypoint %zu.", current_waypoint_idx_);
      return;
    }

    goal_reached_pub_->publish(sig);
    motion_complete_pub_->publish(sig);
    RCLCPP_INFO(LOGGER, "Waypoint %zu reached.", current_waypoint_idx_ + 1);
    ++current_waypoint_idx_;
  }

  mission_complete_pub_->publish(sig);
  RCLCPP_INFO(LOGGER, "Waypoint mission complete.");
}

void WordleBotControlNode::runPickAndPlaceMission()
{
  geometry_msgs::msg::Pose object_pose;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    object_pose = letter_object_pose_;
  }

  // PRE_PICK phase — skipped when resuming after a successful pick (POST_PICK)
  if (pick_place_phase_ != PickPlacePhase::POST_PICK) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pick_place_phase_ = PickPlacePhase::PRE_PICK;
    }

    if (stop_requested_.load()) {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      robot_state_ = RobotState::STOPPED;
      RCLCPP_INFO(LOGGER, "runPickAndPlaceMission: STOPPED before pick phase.");
      return;
    }

    RCLCPP_INFO(LOGGER, "runPickAndPlaceMission: executing pick phase.");
    const bool pick_ok = controller_->doPickPhase(object_pose);

    if (!pick_ok || stop_requested_.load()) {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pick_place_phase_ = PickPlacePhase::PRE_PICK;  // resume retries pick from current position
      robot_state_ = RobotState::STOPPED;
      RCLCPP_INFO(LOGGER, "runPickAndPlaceMission: STOPPED during pick — will retry on resume.");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pick_place_phase_ = PickPlacePhase::POST_PICK;
    }
    RCLCPP_INFO(LOGGER, "runPickAndPlaceMission: pick complete — object attached.");
  } else {
    RCLCPP_INFO(LOGGER, "runPickAndPlaceMission: resuming into place phase (object already in hand).");
  }

  // Check stop between pick and place
  if (stop_requested_.load()) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    robot_state_ = RobotState::STOPPED;
    RCLCPP_INFO(LOGGER, "runPickAndPlaceMission: STOPPED between pick and place.");
    return;
  }

  RCLCPP_INFO(LOGGER, "runPickAndPlaceMission: executing place phase.");
  const bool place_ok = controller_->doPlacePhase();

  if (!place_ok || stop_requested_.load()) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    // Phase stays POST_PICK — resume will skip pick and retry place
    robot_state_ = RobotState::STOPPED;
    RCLCPP_INFO(LOGGER, "runPickAndPlaceMission: STOPPED during place — will retry place on resume.");
    return;
  }

  std_msgs::msg::Bool sig;
  sig.data = true;
  mission_complete_pub_->publish(sig);
  motion_complete_pub_->publish(sig);
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    letter_object_received_ = false;
    pick_place_phase_ = PickPlacePhase::NONE;
  }
  RCLCPP_INFO(LOGGER, "runPickAndPlaceMission: pick-and-place complete.");
}

void WordleBotControlNode::doAbort()
{
  RCLCPP_INFO(LOGGER, "doAbort: returning to home position.");
  // Clear stop flag so the home motion is not immediately rejected by the controller
  stop_requested_.store(false);
  controller_->clearStopFlag();

  if (!controller_->moveToHome()) {
    RCLCPP_ERROR(LOGGER, "doAbort: moveToHome failed — robot may not be at home.");
  } else {
    RCLCPP_INFO(LOGGER, "doAbort: arm returned to home.");
  }
}

// ---------------------------------------------------------------------------
// Public interface
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
