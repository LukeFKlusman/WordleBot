#include "wordlebot_control/wordle_bot_control_node.hpp"

#include <chrono>
#include <thread>
#include <moveit/planning_scene/planning_scene.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("WordleBotControlNode");

WordleBotControlNode::WordleBotControlNode(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("wordle_bot_control_node", options)),
  controller_(std::make_shared<WordleBotController>(node_))
{
  // ---------------------------------------------------------------------------
  // ROS2 topic subscriptions and publications
  // ---------------------------------------------------------------------------
  // Publishers
  // - /wordle_bot/motion_complete (std_msgs/Bool): Published after each successful motion execution.
  // - /wordle_bot/goal_reached (std_msgs/Bool): Published after reaching each individual goal in a mission.
  // - /wordle_bot/mission_complete (std_msgs/Bool): Published after completing all goals in a mission.
  // - /wordle_bot/robot_state (std_msgs/String): Published with "IDLE" or "RUNNING" to indicate current state.

  // Subscriptions
  // - /wordle_bot/goal_pose (geometry_msgs/PoseStamped): Legacy single-goal interface for free-space motion.
  // - /wordle_bot/set_mission (geometry_msgs/PoseArray): Set a multi-goal mission; execution starts on /start_mission.
  // - /wordle_bot/start_mission (std_msgs/Bool): Start executing the currently set mission.
  // - /wordle_bot/stop_mission (std_msgs/Bool): Request the robot to stop the current mission safely.
  // - /wordle_bot/resume_mission (std_msgs/Bool): Request the robot to resume a stopped mission. (not yet implemented)
  // - /wordle_bot/abort_mission (std_msgs/Bool): Request the robot to abort the current mission. (not yet implemented)
  // - /wordle_bot/add_collision_object (moveit_msgs/CollisionObject): Add or remove a collision object in the planning scene.
  // - /perception/letter_objects (wordlebot_control/PickPlaceTask): Trigger a pick-and-place task for a detected letter object, with specified pick pose and place slot.
  // - /wordle_bot/clear_letter_objects (std_msgs/Bool): Remove all tracked letter collision objects from the planning scene and reset the queue.
  // ---------------------------------------------------------------------------
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/wordle_bot/goal_pose", 10,
    std::bind(&WordleBotControlNode::goalCallback, this, std::placeholders::_1));

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
// Subscriber callbacks
// ---------------------------------------------------------------------------
// goalCallback
// 
// setMissionCallback
// 
// startMissionCallback
// 
// stopMissionCallback
// 
// resumeMissionCallback
// 
// abortMissionCallback
// 
// collisionObjectCallback
//
// letterObjectCallback
//
// clearLetterObjectsCallback
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

void WordleBotControlNode::collisionObjectCallback(const moveit_msgs::msg::CollisionObject::SharedPtr msg)
{
  RCLCPP_INFO(LOGGER, "collisionObjectCallback: received object '%s' (operation=%d).",
    msg->id.c_str(), static_cast<int>(msg->operation));
  controller_->addCollisionObject(*msg);
}

void WordleBotControlNode::letterObjectCallback(const wordlebot_control::msg::PickPlaceTask::SharedPtr msg)
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

  // Build 50 mm cube collision object at the pick pose
  const std::string object_id = "letter_" + std::to_string(++letter_object_counter_);

  moveit_msgs::msg::CollisionObject co;
  co.id = object_id;
  co.header.frame_id = incoming_frame;
  co.header.stamp = node_->get_clock()->now();
  co.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.05};
  co.primitives.push_back(box);
  co.primitive_poses.push_back(msg->pick_pose.pose);

  WordleBotController::PickPlaceEntry entry;
  entry.pick_pose = msg->pick_pose.pose;
  entry.place_pose = place_pose;
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
        "clearLetterObjectsCallback: mission is running — cannot clear letter objects now.");
      return;
    }
    ids_to_remove = tracked_letter_ids_;
    tracked_letter_ids_.clear();
    pick_place_queue_.clear();
    letter_object_counter_ = 0;
  }

  controller_->clearLetterObjects(ids_to_remove);
  RCLCPP_INFO(LOGGER,
    "clearLetterObjectsCallback: cleared %zu letter object(s) and reset queue.",
    ids_to_remove.size());
}

// ---------------------------------------------------------------------------
// Mission loop
// ---------------------------------------------------------------------------
// 
// 
// ---------------------------------------------------------------------------

void WordleBotControlNode::missionLoop()
{
  RCLCPP_INFO(LOGGER, "Mission thread started.");
  while (rclcpp::ok()) {
    bool do_pick_and_place = false;
    std::vector<geometry_msgs::msg::Pose> current_mission;
    std::vector<WordleBotController::PickPlaceEntry> current_tasks;
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
      RCLCPP_INFO(LOGGER, "Mission: %zu pick-and-place task(s) queued.", current_tasks.size());

      // ── Scene setup: add ALL collision objects before any planning ──────────
      // All objects must be present in the live scene so that CurrentState (task 1)
      // and the chained FixedState scenes (tasks 2..N) all see the full workspace.
      RCLCPP_INFO(LOGGER,
        "Mission: adding all %zu collision objects to scene before planning.",
        current_tasks.size());
      for (const auto & entry : current_tasks) {
        controller_->addCollisionObject(entry.collision_object);
        RCLCPP_INFO(LOGGER, "  Added collision object '%s'.", entry.object_id.c_str());
      }

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
            all_ok = false;
            break;
          }

          RCLCPP_INFO(LOGGER, "Executing pre-planned task %zu of %zu: '%s'.",
            i + 1, planned_tasks.size(), planned_tasks[i].object_id.c_str());

          if (!controller_->executePlannedTask(planned_tasks[i]) || stop_requested_.load()) {
            RCLCPP_ERROR(LOGGER,
              "Execution FAILED at task %zu ('%s') — stopping.",
              i + 1, planned_tasks[i].object_id.c_str());
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
