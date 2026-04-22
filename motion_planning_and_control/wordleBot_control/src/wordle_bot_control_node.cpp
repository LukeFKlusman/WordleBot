#include "wordleBot_control/wordle_bot_control_node.hpp"

#include <chrono>
#include <thread>

#include <Eigen/Geometry>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

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
  co.id = LETTER_OBJECT_ID;
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
    LETTER_OBJECT_ID);
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
// Pick-and-place: MTC task creation, planning, execution
// ---------------------------------------------------------------------------

mtc::Task WordleBotControlNode::createTask(const geometry_msgs::msg::Pose & object_pose)
{
  RCLCPP_DEBUG(LOGGER, "createTask: initialising MTC task.");
  mtc::Task task;
  task.stages()->setName("pick and place letter");
  task.loadRobotModel(node_);

  const std::string arm_group  = "ur_onrobot_manipulator";
  const std::string hand_group = "ur_onrobot_gripper";
  const std::string hand_frame = "gripper_tcp";

  task.setProperty("group",    arm_group);
  task.setProperty("eef",      hand_group);
  task.setProperty("ik_frame", hand_frame);
  RCLCPP_DEBUG(LOGGER, "createTask: arm='%s', hand_group='%s', ik_frame='%s'.",
    arm_group.c_str(), hand_group.c_str(), hand_frame.c_str());

  // Explicitly name the OMPL pipeline so MTC never falls back to CHOMP.
  // ompl_planning.yaml must be loaded into the node's parameters (see wordle_bot.launch.py).
  auto sampling_planner      = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.001);
  RCLCPP_DEBUG(LOGGER, "createTask: CartesianPath planner configured (vel=0.5, acc=0.5, step=0.001).");

  // Dedicated planner for retreat: min_fraction=0 accepts whatever Cartesian
  // distance the arm can actually achieve at the (low, far-reach) place pose.
  auto retreat_planner = std::make_shared<mtc::solvers::CartesianPath>();
  retreat_planner->setMaxVelocityScalingFactor(0.5);
  retreat_planner->setMaxAccelerationScalingFactor(0.5);
  retreat_planner->setStepSize(0.001);
  retreat_planner->setMinFraction(0.0);

  // ── Stage 1: capture current state ────────────────────────────────────────
  mtc::Stage * current_state_ptr = nullptr;
  {
    RCLCPP_DEBUG(LOGGER, "createTask: adding stage 1 — CurrentState.");
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  // ── Stage 2: open gripper ─────────────────────────────────────────────────
  {
    RCLCPP_DEBUG(LOGGER, "\ncreateTask: adding stage 2 — open hand (goal='open').");
    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));
  }

  // ── Stage 3: free-space move to pick region ───────────────────────────────
  {
    RCLCPP_INFO(LOGGER, "createTask: adding stage 3 — Connect 'move to pick' (timeout=10 s, no path constraints).");
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
    stage_move_to_pick->setTimeout(10.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    // Path constraints are intentionally NOT applied here. buildPathConstraints() uses raw
    // joint values that MoveIt doesn't normalise (±2π), causing the start state to appear
    // invalid and triggering 10-second timeouts before falling back to unconstrained planning.
    task.add(std::move(stage_move_to_pick));
  }

  // ── Stage 4: pick container ───────────────────────────────────────────────
  mtc::Stage * attach_object_stage = nullptr;
  {
    RCLCPP_DEBUG(LOGGER, "\ncreateTask: building stage 4 — SerialContainer 'pick object'.");
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // 4a. Cartesian approach along gripper_tcp z-axis
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4a — MoveRelative 'approach object' "
        "(link='%s', dist=[0.05, 0.10], dir=+z in %s).",
        hand_frame.c_str(), hand_frame.c_str());
      
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // 4b. Sample grasp pose around the object + solve IK
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4b — GenerateGraspPose for object '%s' "
        "(angle_delta=π/12, IK solutions=8, z_offset=0.08 m).",
        LETTER_OBJECT_ID);
      auto stage =
        std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(LETTER_OBJECT_ID);
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);

      // Transform from gripper_tcp to the object centre when grasping top-down.
      // z=0.08 means gripper_tcp sits 80 mm above the object centre at grasp time.
      constexpr double GRASP_Z_OFFSET = 0.01;
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = GRASP_Z_OFFSET;

      const double expected_grasp_z   = object_pose.position.z + GRASP_Z_OFFSET;
      const double expected_approach_z_min = expected_grasp_z + 0.10;
      const double expected_approach_z_max = expected_grasp_z + 0.15;
      RCLCPP_INFO(LOGGER,
        "createTask [grasp geometry]: object_z=%.4f m  grasp_z_offset=%.3f m  "
        "=> expected gripper_tcp z AT GRASP = %.4f m  "
        "| pre-approach z range = [%.4f, %.4f] m (world frame, top-down).",
        object_pose.position.z, GRASP_Z_OFFSET,
        expected_grasp_z, expected_approach_z_min, expected_approach_z_max);

      auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    // 4c. Allow collisions between gripper links and the object
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4c — ModifyPlanningScene allow collision "
        "('%s', hand links).", LETTER_OBJECT_ID);
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (hand,object)");
      stage->allowCollisions( LETTER_OBJECT_ID,
                              task.getRobotModel()
                                ->getJointModelGroup(hand_group)
                                ->getLinkModelNamesWithCollisionGeometry(),
                              true);
      grasp->insert(std::move(stage));
    }

    // 4d. Close gripper
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4d — MoveTo 'close hand' (goal='close').");
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("closed");
      grasp->insert(std::move(stage));
    }

    // 4e. Attach object to the gripper link — GeneratePlacePose monitors this stage
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4e — ModifyPlanningScene attach '%s' to '%s'.",
        LETTER_OBJECT_ID, hand_frame.c_str());
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(LETTER_OBJECT_ID, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // 4f. Lift object vertically
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4f — MoveRelative 'lift object' "
        "(dist=[0.05, 0.15], dir=+z world).");
      auto stage =
        std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    RCLCPP_DEBUG(LOGGER, "createTask: stage 4 'pick object' container complete.");
    task.add(std::move(grasp));
  }

  // ── Stage 5: free-space move to place region ──────────────────────────────
  {
    RCLCPP_INFO(LOGGER, "createTask: adding stage 5 — Connect 'move to place' (timeout=10 s, no path constraints).");
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{
        {arm_group, sampling_planner}});
    stage->setTimeout(10.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    // Path constraints intentionally omitted — see stage 3 comment.
    task.add(std::move(stage));
  }

  // ── Stage 6: place container ──────────────────────────────────────────────
  {
    RCLCPP_DEBUG(LOGGER, "createTask: building stage 6 — SerialContainer 'place object'.");
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // 6a. Generate place pose + solve IK (object must reach PLACE_X/Y/Z in world)
    {
      RCLCPP_DEBUG(LOGGER, "\ncreateTask: 6a — GeneratePlacePose for '%s' "
        "target=(%.3f, %.3f, %.3f) world, IK solutions=4.",
        LETTER_OBJECT_ID, PLACE_X, PLACE_Y, PLACE_Z);
      auto stage =
        std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(LETTER_OBJECT_ID);

      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = "world";
      target_pose.pose.position.x = PLACE_X;
      target_pose.pose.position.y = PLACE_Y;
      target_pose.pose.position.z = PLACE_Z;
      target_pose.pose.orientation.w = 1.0;
      stage->setPose(target_pose);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(4);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(LETTER_OBJECT_ID);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    // 6b. Open gripper to release the object
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 6b — MoveTo 'open hand' (goal='open').");
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    // 6c. Restore collision checking between gripper and object
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 6c — ModifyPlanningScene forbid collision "
        "('%s', hand links).", LETTER_OBJECT_ID);
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "forbid collision (hand,object)");
      stage->allowCollisions(
        LETTER_OBJECT_ID,
        task.getRobotModel()
          ->getJointModelGroup(hand_group)
          ->getLinkModelNamesWithCollisionGeometry(),
        false);
      place->insert(std::move(stage));
    }

    // 6d. Detach object from gripper
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 6d — ModifyPlanningScene detach '%s' from '%s'.",
        LETTER_OBJECT_ID, hand_frame.c_str());
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(LETTER_OBJECT_ID, hand_frame);
      place->insert(std::move(stage));
    }

    // 6e. Retreat vertically away from the placed object
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 6e — MoveRelative 'retreat' "
        "(dist=[0.05, 0.15], dir=+z world).");
      auto stage =
        std::make_unique<mtc::stages::MoveRelative>("retreat", retreat_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.03, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    RCLCPP_DEBUG(LOGGER, "createTask: stage 6 'place object' container complete.");
    task.add(std::move(place));
  }

  // ── Stage 7: return to named home state ───────────────────────────────────
  {
    RCLCPP_DEBUG(LOGGER, "createTask: adding stage 7 — MoveTo 'return home' (goal='home').");
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  RCLCPP_DEBUG(LOGGER, "createTask: task construction complete (7 top-level stages).");
  return task;
}

void WordleBotControlNode::doPickAndPlace()
{
  RCLCPP_INFO(LOGGER, "doPickAndPlace: building MTC task.");

  geometry_msgs::msg::Pose object_pose;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    object_pose = letter_object_pose_;
  }

  RCLCPP_INFO(LOGGER,
    "doPickAndPlace: object pose = (%.4f, %.4f, %.4f)  "
    "orient=(%.4f, %.4f, %.4f, %.4f). "
    "MTC planning scene frame: 'world'.",
    object_pose.position.x, object_pose.position.y, object_pose.position.z,
    object_pose.orientation.x, object_pose.orientation.y,
    object_pose.orientation.z, object_pose.orientation.w);

  task_ = createTask(object_pose);

  try {
    task_.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "MTC task init failed: " << e);
    return;
  }

  RCLCPP_INFO(LOGGER, "doPickAndPlace: planning...");
  moveit::core::MoveItErrorCode plan_result;
  try {
    plan_result = task_.plan(5);
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "MTC task planning threw InitStageException: " << e);
    return;
  }
  if (!plan_result || task_.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "MTC task planning failed — no solutions found.");
    return;
  }

  RCLCPP_INFO(LOGGER,
    "doPickAndPlace: planning succeeded — %zu solution(s) found. "
    "Executing best solution (cost=%.3f).",
    task_.solutions().size(),
    task_.solutions().front()->cost());

  task_.introspection().publishSolution(*task_.solutions().front());

  RCLCPP_INFO(LOGGER, "doPickAndPlace: executing...");
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "MTC task execution failed (error code %d).", result.val);
    return;
  }

  RCLCPP_INFO(LOGGER, "doPickAndPlace: pick-and-place succeeded.");

  std_msgs::msg::Bool signal;
  signal.data = true;
  mission_complete_pub_->publish(signal);
  motion_complete_pub_->publish(signal);

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    letter_object_received_ = false;
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
