#include "wordleBot_control/wordle_bot_control_node.hpp"

#include <chrono>
#include <thread>

// TODO: Functionalities to add:
// - Scan area for obstacles and add to planning scene (TC1.3)
// - Stop/Resume/Abort mission (TC1.4) — requires mission state management

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
    if (goal_queue_.empty() && !letter_object_received_) {
      RCLCPP_WARN(LOGGER,
        "start_mission received but goal queue is empty and no letter object — ignoring.");
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
// Mission loop
// ---------------------------------------------------------------------------

void WordleBotControlNode::missionLoop()
{
  RCLCPP_INFO(LOGGER, "Mission thread started.");
  while (rclcpp::ok()) {
    bool do_pick_and_place = false;
    std::vector<geometry_msgs::msg::Pose> current_mission;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      cv_.wait(lock, [this]() {
        return (mission_armed_ && (!goal_queue_.empty() || letter_object_received_)) ||
               !rclcpp::ok();
      });
      if (!rclcpp::ok()) break;

      do_pick_and_place = letter_object_received_;
      if (!do_pick_and_place) {
        current_mission = std::move(goal_queue_);
        goal_queue_.clear();
      }
      mission_armed_ = false;
    }

    mission_state_ = MissionState::RUNNING;
    std_msgs::msg::Bool signal;
    signal.data = true;

    if (do_pick_and_place) {
      RCLCPP_INFO(LOGGER, "Mission thread: dispatching pick-and-place task.");
      doPickAndPlace();
    } else {
      RCLCPP_INFO(LOGGER, "Mission thread: executing %zu waypoint goal(s).",
        current_mission.size());
      for (std::size_t i = 0; i < current_mission.size(); ++i) {
        RCLCPP_INFO(LOGGER, "Executing goal %zu of %zu.", i + 1, current_mission.size());
        controller_->moveToTarget(current_mission[i]);
        goal_reached_pub_->publish(signal);
        motion_complete_pub_->publish(signal);
        RCLCPP_INFO(LOGGER, "Goal %zu reached.", i + 1);
      }
      mission_complete_pub_->publish(signal);
      RCLCPP_INFO(LOGGER, "Mission complete published.");
    }

    mission_state_ = MissionState::IDLE;
  }
  RCLCPP_INFO(LOGGER, "Mission thread exiting.");
}

// ---------------------------------------------------------------------------
// Pick-and-place
// ---------------------------------------------------------------------------

void WordleBotControlNode::doPickAndPlace()
{
  geometry_msgs::msg::Pose object_pose;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    object_pose = letter_object_pose_;
  }

  const bool success = controller_->doPickAndPlace(object_pose);

  if (success) {
    std_msgs::msg::Bool signal;
    signal.data = true;
    mission_complete_pub_->publish(signal);
    motion_complete_pub_->publish(signal);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      letter_object_received_ = false;
    }
  } else {
    RCLCPP_ERROR(LOGGER, "doPickAndPlace: pick-and-place failed.");
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
