#include "wordlebot_control/wordle_bot_control_node.hpp"

#include <chrono>
#include <thread>
#include <moveit/planning_scene/planning_scene.h>
#include <tf2/LinearMath/Quaternion.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("WordleBotControlNode");

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
  // - /wordle_bot/clear_letter_objects (std_msgs/Bool): Remove all letter collision objects and reset queue.
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

  // Load scan-and-sweep parameters from config/scan_sweep_poses.yaml.
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
  entry.place_pose = msg->place_pose;
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
  RCLCPP_INFO(LOGGER, "openGripperCallback: opening gripper.");
  controller_->openGripper();
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
  RCLCPP_INFO(LOGGER, "closeGripperCallback: closing gripper.");
  controller_->closeGripper();
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
      RCLCPP_INFO(LOGGER, "Goal mission: %zu goal(s) queued.", current_mission.size());

      // ── Phase 1: Plan all goals sequentially, chaining via FixedState ────────
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

        // ── Phase 2: Execute all pre-planned goals in order ──────────────────
        bool all_ok = true;
        for (std::size_t i = 0; i < planned_goals.size(); ++i) {
          if (stop_requested_.load()) {
            RCLCPP_INFO(LOGGER, "Goal execution: stop requested before goal %zu — aborting.", i + 1);
            all_ok = false;
            break;
          }

          RCLCPP_INFO(LOGGER, "Executing goal %zu of %zu.", i + 1, planned_goals.size());

          if (!controller_->executePlannedMoveToGoal(planned_goals[i]) || stop_requested_.load()) {
            RCLCPP_ERROR(LOGGER, "Execution FAILED at goal %zu — stopping.", i + 1);
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
