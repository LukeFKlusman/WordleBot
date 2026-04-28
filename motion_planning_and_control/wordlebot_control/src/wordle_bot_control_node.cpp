#include "wordlebot_control/wordle_bot_control_node.hpp"

#include <chrono>
#include <thread>

// TODO: Functionalities to add:
// - Scan area for obstacles and add to planning scene (TC1.3)
// - Resume / Abort mission (TC1.4) — stop is functional; resume/abort require mission state management

static const rclcpp::Logger LOGGER = rclcpp::get_logger("WordleBotControlNode");

WordleBotControlNode::WordleBotControlNode(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("wordle_bot_control_node", options)),
  controller_(std::make_shared<WordleBotController>(node_))
{
  // ---------------------------------------------------------------------------
  // Legacy single-goal interface (backward compat): goal_pose auto-arms the mission
  // ---------------------------------------------------------------------------
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/wordle_bot/goal_pose", 10,
    std::bind(&WordleBotControlNode::goalCallback, this, std::placeholders::_1));

  motion_complete_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/wordle_bot/motion_complete", 10);

  // ---------------------------------------------------------------------------
  // Mission-level interface
  // ---------------------------------------------------------------------------
  set_mission_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    "/wordle_bot/set_mission", 10,
    std::bind(&WordleBotControlNode::setMissionCallback, this, std::placeholders::_1));

  goal_reached_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/wordle_bot/goal_reached", 10);

  mission_complete_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/wordle_bot/mission_complete", 10);

  // Stop / Resume / Abort control (resume and abort not yet implemented)
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

  // Collision object interface: receives collision objects to add to the planning scene
  add_collision_object_sub_ = node_->create_subscription<moveit_msgs::msg::CollisionObject>(
    "/wordle_bot/add_collision_object", 10,
    std::bind(&WordleBotControlNode::collisionObjectCallback, this, std::placeholders::_1));

  // Letter object interface: receives pick pose + place slot, queues pick-and-place tasks
  letter_object_sub_ = node_->create_subscription<wordlebot_control::msg::PickPlaceTask>(
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
    if (mission_running_) {
      RCLCPP_WARN(LOGGER, "startMissionCallback: mission already running — ignoring start.");
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

  // Only reset the collision scene for waypoint missions; pick-and-place keeps letter objects
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
    if (!mission_running_) {
      RCLCPP_WARN(LOGGER, "stopMissionCallback: no mission running — ignoring.");
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
  RCLCPP_WARN(LOGGER, "resumeMissionCallback: resume not yet implemented.");
}

void WordleBotControlNode::abortMissionCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;
  RCLCPP_WARN(LOGGER, "abortMissionCallback: abort not yet implemented.");
}

void WordleBotControlNode::collisionObjectCallback(
  const moveit_msgs::msg::CollisionObject::SharedPtr msg)
{
  RCLCPP_INFO(LOGGER, "collisionObjectCallback: received object '%s' (operation=%d).",
    msg->id.c_str(), static_cast<int>(msg->operation));
  controller_->addCollisionObject(*msg);
}

void WordleBotControlNode::letterObjectCallback(
  const wordlebot_control::msg::PickPlaceTask::SharedPtr msg)
{
  const int slot = msg->place_slot;
  if (slot < 1 || slot > 5) {
    RCLCPP_WARN(LOGGER,
      "letterObjectCallback: place_slot=%d is out of range [1,5] — discarding task.", slot);
    return;
  }

  const std::string incoming_frame =
    msg->pick_pose.header.frame_id.empty() ? "world" : msg->pick_pose.header.frame_id;

  RCLCPP_INFO(LOGGER,
    "letterObjectCallback: received pick=(%.3f, %.3f, %.3f) frame='%s'  place_slot=P%d.",
    msg->pick_pose.pose.position.x,
    msg->pick_pose.pose.position.y,
    msg->pick_pose.pose.position.z,
    incoming_frame.c_str(), slot);

  if (incoming_frame != "world") {
    RCLCPP_WARN(LOGGER,
      "letterObjectCallback: incoming frame is '%s', not 'world'. "
      "MTC planning scene assumes 'world' — verify the publisher transforms poses into 'world'.",
      incoming_frame.c_str());
  }

  // Resolve the place pose from the slot index
  const auto & s = WordleBotController::PLACE_SLOTS[static_cast<std::size_t>(slot - 1)];
  geometry_msgs::msg::Pose place_pose;
  place_pose.position.x = s.x;
  place_pose.position.y = s.y;
  place_pose.position.z = s.z;
  place_pose.orientation.w = 1.0;

  // Build 40 mm cube collision object at the pick pose
  moveit_msgs::msg::CollisionObject co;
  co.id = WordleBotController::LETTER_OBJECT_ID;
  co.header.frame_id = incoming_frame;
  co.header.stamp = node_->get_clock()->now();
  co.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.05};
  co.primitives.push_back(box);
  co.primitive_poses.push_back(msg->pick_pose.pose);

  PickPlaceEntry entry;
  entry.pick_pose = msg->pick_pose.pose;
  entry.place_pose = place_pose;
  entry.collision_object = co;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    pick_place_queue_.push_back(entry);
  }

  RCLCPP_INFO(LOGGER,
    "letterObjectCallback: task queued (pick-and-place queue size=%zu). "
    "Waiting for start_mission.",
    pick_place_queue_.size());
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
    std::vector<PickPlaceEntry> current_tasks;
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

    if (do_pick_and_place) {
      RCLCPP_INFO(LOGGER, "Mission thread: executing %zu pick-and-place task(s).",
        current_tasks.size());

      bool all_ok = true;
      for (std::size_t i = 0; i < current_tasks.size(); ++i) {
        if (stop_requested_.load()) {
          RCLCPP_INFO(LOGGER, "Pick-and-place queue stopped before task %zu.", i + 1);
          all_ok = false;
          break;
        }
        RCLCPP_INFO(LOGGER, "Executing pick-and-place task %zu of %zu.", i + 1, current_tasks.size());

        // Re-add this task's collision object so MTC can find it in the planning scene
        controller_->addCollisionObject(current_tasks[i].collision_object);

        const bool ok = controller_->doPickAndPlace(
          current_tasks[i].pick_pose, current_tasks[i].place_pose);

        if (!ok || stop_requested_.load()) {
          RCLCPP_ERROR(LOGGER, "Pick-and-place task %zu failed or was stopped.", i + 1);
          all_ok = false;
          break;
        }

        goal_reached_pub_->publish(signal);
        RCLCPP_INFO(LOGGER, "Pick-and-place task %zu complete.", i + 1);
      }

      if (all_ok && !stop_requested_.load()) {
        mission_complete_pub_->publish(signal);
        motion_complete_pub_->publish(signal);
        RCLCPP_INFO(LOGGER, "Pick-and-place mission complete.");
      } else {
        RCLCPP_ERROR(LOGGER, "Pick-and-place mission did not complete successfully.");
      }

    } else {
      RCLCPP_INFO(LOGGER, "Mission thread: executing %zu waypoint goal(s).",
        current_mission.size());
      for (std::size_t i = 0; i < current_mission.size(); ++i) {
        if (stop_requested_.load()) {
          RCLCPP_INFO(LOGGER, "Waypoint mission stopped before goal %zu.", i + 1);
          break;
        }
        RCLCPP_INFO(LOGGER, "Executing goal %zu of %zu.", i + 1, current_mission.size());
        const bool ok = controller_->moveToTarget(current_mission[i]);
        if (!ok || stop_requested_.load()) {
          RCLCPP_INFO(LOGGER, "Waypoint mission stopped at goal %zu.", i + 1);
          break;
        }
        goal_reached_pub_->publish(signal);
        motion_complete_pub_->publish(signal);
        RCLCPP_INFO(LOGGER, "Goal %zu reached.", i + 1);
      }
      if (!stop_requested_.load()) {
        mission_complete_pub_->publish(signal);
        RCLCPP_INFO(LOGGER, "Mission complete published.");
      }
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      mission_running_ = false;
    }
    {
      std_msgs::msg::String state_msg;
      state_msg.data = "IDLE";
      robot_state_pub_->publish(state_msg);
    }
  }
  RCLCPP_INFO(LOGGER, "Mission thread exiting.");
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
