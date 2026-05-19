#include "wordlebot_control/wordle_bot_controller.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace mtc = moveit::task_constructor;

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("WordleBotController");

WordleBotController::WordleBotController(rclcpp::Node::SharedPtr node)
: node_(node),
  move_group_(node, "ur_onrobot_manipulator"),
  planning_scene_(),
  visual_tools_(node, "ur_base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_.getRobotModel())
{
  visual_tools_.deleteAllMarkers();
  visual_tools_.loadRemoteControl();

  // Give OMPL more time and attempts to find a better path
  move_group_.setPlanningTime(15.0);
  move_group_.setNumPlanningAttempts(5);

  // DEBUG: log all link names in the robot model so we can verify touch_links names
  RCLCPP_INFO(LOGGER, "Robot model link names:");
  for (const auto & link : move_group_.getRobotModel()->getLinkModelNames()) {
    RCLCPP_INFO(LOGGER, "  link: %s", link.c_str());
  }
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_.getPlanningFrame().c_str());
}

WordleBotController::~WordleBotController()
{
}

// ---------------------------------------------------------------------------
// Collision Scene Management
// Builds and tears down the static environment (floor, sensor guard) and
// manages dynamic collision objects added at runtime.
// ---------------------------------------------------------------------------
// setupCollisionScene       — add the floor plane and attach the sensor guard
//                             cylinder to the end effector
// clearCollisionScene       — remove all static objects and detach the sensor guard
// addCollisionObject        — apply an ADD / REMOVE / MOVE collision object
//                             to the live planning scene
// clearLetterObjects        — remove a list of letter objects by ID from the scene
// attachSensorCollisionObject — attach a protective cylinder to tool0 so the
//                               planner treats the sensor as part of the robot
// detachSensorCollisionObject — detach the sensor guard cylinder from tool0
// ---------------------------------------------------------------------------

void WordleBotController::setupCollisionScene()
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_.getPlanningFrame();
  collision_object.id = "box1";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 0.05;
  primitive.dimensions[primitive.BOX_Z] = 0.25;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.35;
  box_pose.position.y = 0.1;
  box_pose.position.z = 0.1;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // planning_scene_.applyCollisionObject(collision_object);

  moveit_msgs::msg::CollisionObject floor;
  floor.header.frame_id = move_group_.getPlanningFrame();
  floor.id = "floor";

  shape_msgs::msg::SolidPrimitive floor_shape;
  floor_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  floor_shape.dimensions = {2.0, 2.0, 0.01};

  geometry_msgs::msg::Pose floor_pose;
  floor_pose.position.x = 0.0;
  floor_pose.position.y = 0.0;
  floor_pose.position.z = -0.015;
  floor_pose.orientation.w = 1.0;

  floor.primitives.push_back(floor_shape);
  floor.primitive_poses.push_back(floor_pose);
  floor.operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_.applyCollisionObject(floor);

  attachSensorCollisionObject();
  RCLCPP_INFO(LOGGER, "Collision scene set up: floor added, sensor guard attached.");
}


void WordleBotController::clearCollisionScene()
{
  detachSensorCollisionObject();
  planning_scene_.removeCollisionObjects({"box1", "floor"});
  RCLCPP_INFO(LOGGER, "Collision scene cleared.");
}


void WordleBotController::addCollisionObject(const moveit_msgs::msg::CollisionObject & obj)
{
  planning_scene_.applyCollisionObject(obj);
  rclcpp::sleep_for(std::chrono::milliseconds(300));
  RCLCPP_INFO(LOGGER, "addCollisionObject: applied object '%s' (operation=%d).",
    obj.id.c_str(), static_cast<int>(obj.operation));
}


void WordleBotController::clearLetterObjects(const std::vector<std::string> & ids)
{
  if (ids.empty()) {
    RCLCPP_INFO(LOGGER, "clearLetterObjects: nothing to remove.");
    return;
  }
  planning_scene_.removeCollisionObjects(ids);
  RCLCPP_INFO(LOGGER, "clearLetterObjects: removed %zu letter object(s).", ids.size());
}


void WordleBotController::attachSensorCollisionObject()
{
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "tool0";
  attached_object.object.id = "sensor_guard";
  attached_object.object.header.frame_id = "tool0";
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive cylinder;
  cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder.dimensions = {0.03, 0.06};  // [height, radius] in metres

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = -0.015;  // slight offset to avoid collision with the end-effector link itself
  pose.orientation.w = 1.0;

  attached_object.object.primitives.push_back(cylinder);
  attached_object.object.primitive_poses.push_back(pose);

  // Allow all links that physically overlap with the sensor guard cylinder.
  // Includes the full gripper chain beyond tool0 (onrobot RG2).
  attached_object.touch_links = {
    "wrist_3_link", "flange", "tool0", "ft_frame",
    "onrobot_base_link",
    "cable_connector_0", "cable_connector_1",
    "left_outer_knuckle", "left_inner_finger", "left_finger_tip",
    "finger_width_mock_link",
    "left_inner_knuckle", "right_inner_knuckle",
    "right_outer_knuckle", "right_inner_finger", "right_finger_tip",
    "gripper_tcp"
  };

  planning_scene_.applyAttachedCollisionObject(attached_object);

  // Give the move_group server time to propagate the scene update before planning begins.
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "Sensor guard attached to tool0");
}


void WordleBotController::detachSensorCollisionObject()
{
  moveit_msgs::msg::AttachedCollisionObject detach;
  detach.link_name = "tool0";
  detach.object.id = "sensor_guard";
  detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  planning_scene_.applyAttachedCollisionObject(detach);
  RCLCPP_INFO(LOGGER, "Sensor guard collision cylinder detached from tool0.");
}

// ---------------------------------------------------------------------------
// Pick and Place
// Build, plan, and execute MTC pick-and-place tasks. The primary workflow is
// the two-phase plan-then-execute path (createTask → planPickAndPlace →
// executePlannedTask). doPickAndPlace is a single-call convenience wrapper
// that plans and executes in one shot.
// ---------------------------------------------------------------------------
// createTask          — build an MTC task with all pick-and-place stages;
//                       accepts an optional chained start scene for batching
// planPickAndPlace    — plan one pick-and-place task without executing it;
//                       returns a PlannedPickPlace with the terminal scene for
//                       chaining to the next task
// executePlannedTask  — execute a previously planned pick-and-place task
// doPickAndPlace      — convenience wrapper: plan and execute in one call
// ---------------------------------------------------------------------------

mtc::Task WordleBotController::createTask(const geometry_msgs::msg::Pose & object_pose,
                                          const geometry_msgs::msg::Pose & place_pose,
                                          const std::string & object_id,
                                          const planning_scene::PlanningScenePtr & start_scene,
                                          bool include_return_home)
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

  // ── Stage 1: start state — CurrentState (task 1) or FixedState (tasks 2..N) ──
  mtc::Stage * current_state_ptr = nullptr;
  {
    if (start_scene == nullptr) {
      RCLCPP_INFO(LOGGER, "createTask [%s]: Stage 1 = CurrentState (live robot state).",
                  object_id.c_str());
      auto stage = std::make_unique<mtc::stages::CurrentState>("current");
      current_state_ptr = stage.get();
      task.add(std::move(stage));
    } else {
      // Log the joint state embedded in the chained scene to verify correct threading.
      const moveit::core::RobotState & rs = start_scene->getCurrentState();
      const moveit::core::JointModelGroup * jmg =
          rs.getRobotModel()->getJointModelGroup("ur_onrobot_manipulator");
      if (jmg) {
        std::vector<double> jv;
        rs.copyJointGroupPositions(jmg, jv);
        const auto & jnames = jmg->getVariableNames();
        RCLCPP_INFO(LOGGER,
          "createTask [%s]: Stage 1 = FixedState (chained from previous solution). "
          "Start joints (%zu):", object_id.c_str(), jv.size());
        for (std::size_t ji = 0; ji < jv.size() && ji < jnames.size(); ++ji) {
          RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
            jnames[ji].c_str(), jv[ji], jv[ji] * 180.0 / M_PI);
        }
      }
      auto stage = std::make_unique<mtc::stages::FixedState>("fixed start");
      stage->setState(start_scene);
      current_state_ptr = stage.get();
      task.add(std::move(stage));
    }
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
    RCLCPP_INFO(LOGGER, "createTask: adding stage 3 — Connect 'move to pick' (timeout=10 s).");
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
    stage_move_to_pick->setTimeout(0.20);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    // stage_move_to_pick->setPathConstraints(WordleBotController::buildPathConstraints());
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
        "(link='%s', dist=[0.05, 0.15], dir=+z in %s).",
        hand_frame.c_str(), hand_frame.c_str());

      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, 0.15);

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
        object_id.c_str());
      auto stage =
        std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(object_id);
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
      wrapper->setMaxIKSolutions(32);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->setProperty("default_pose", std::string("test_configuration"));
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    // 4c. Allow collisions between gripper links and the object
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 4c — ModifyPlanningScene allow collision "
        "('%s', hand links).", object_id.c_str());
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (hand,object)");
      stage->allowCollisions( object_id,
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
        object_id.c_str(), hand_frame.c_str());
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_id, hand_frame);
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
      stage->setMinMaxDistance(0.05, 0.15);
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
    RCLCPP_INFO(LOGGER, "createTask: adding stage 5 — Connect 'move to place' (timeout=10 s).");
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{
        {arm_group, sampling_planner}});
    stage->setTimeout(0.20);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    // stage->setPathConstraints(WordleBotController::buildPathConstraints());
    task.add(std::move(stage));
  }

  // ── Stage 6: place container ──────────────────────────────────────────────
  {
    RCLCPP_DEBUG(LOGGER, "createTask: building stage 6 — SerialContainer 'place object'.");
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // 6a. Generate place pose + solve IK (object must reach the requested slot in world)
    {
      RCLCPP_DEBUG(LOGGER, "\ncreateTask: 6a — GeneratePlacePose for '%s' "
        "target=(%.3f, %.3f, %.3f) world, IK solutions=4.",
        object_id.c_str(),
        place_pose.position.x, place_pose.position.y, place_pose.position.z);
      auto stage =
        std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object_id);

      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = "world";
      target_pose.pose.position.x = place_pose.position.x;
      target_pose.pose.position.y = place_pose.position.y;
      target_pose.pose.position.z = place_pose.position.z;
      target_pose.pose.orientation.w = 1.0;
      stage->setPose(target_pose);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(32);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(object_id);
      wrapper->setProperty("default_pose", std::string("test_configuration"));
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
        "('%s', hand links).", object_id.c_str());
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "forbid collision (hand,object)");
      stage->allowCollisions(
        object_id,
        task.getRobotModel()
          ->getJointModelGroup(hand_group)
          ->getLinkModelNamesWithCollisionGeometry(),
        false);
      place->insert(std::move(stage));
    }

    // 6d. Detach object from gripper
    {
      RCLCPP_DEBUG(LOGGER, "createTask: 6d — ModifyPlanningScene detach '%s' from '%s'.",
        object_id.c_str(), hand_frame.c_str());
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_id, hand_frame);
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

  // ── Stage 7: return home — only for the final task in a batch ────────────
  if (include_return_home) {
    RCLCPP_DEBUG(LOGGER, "createTask [%s]: adding stage 7 — 'return home'.", object_id.c_str());
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal("home");
    task.add(std::move(stage));
  } else {
    RCLCPP_DEBUG(LOGGER,
      "createTask [%s]: skipping stage 7 — not the last task in the batch.", object_id.c_str());
  }

  RCLCPP_DEBUG(LOGGER, "createTask [%s]: task construction complete.", object_id.c_str());
  return task;
}

WordleBotController::PlannedPickPlace WordleBotController::planPickAndPlace(
  const PickPlaceEntry & entry,
  const planning_scene::PlanningScenePtr & start_scene,
  bool include_return_home)
{
  PlannedPickPlace result;
  result.object_id = entry.object_id;

  RCLCPP_INFO(LOGGER,
    "planPickAndPlace [%s]: building task (start=%s, return_home=%s).",
    entry.object_id.c_str(),
    start_scene ? "FixedState(chained)" : "CurrentState(live)",
    include_return_home ? "yes" : "no");

  // createTask() returns by value; Task has a move constructor so this move-constructs
  // into the heap-allocated object without any copy.
  result.task = std::make_unique<mtc::Task>(
    createTask(entry.pick_pose, entry.place_pose, entry.object_id,
               start_scene, include_return_home));

  try {
    result.task->init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER,
      "planPickAndPlace [" << entry.object_id << "]: task init failed: " << e);
    result.task.reset();
    return result;
  }

  RCLCPP_INFO(LOGGER, "planPickAndPlace [%s]: planning (max 5 solutions)...",
              entry.object_id.c_str());
  moveit::core::MoveItErrorCode plan_result;
  try {
    plan_result = result.task->plan(15);
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER,
      "planPickAndPlace [" << entry.object_id << "]: planning threw: " << e);
    result.task.reset();
    return result;
  }

  if (!plan_result || result.task->solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "planPickAndPlace [%s]: no solutions found.", entry.object_id.c_str());
    result.task.reset();
    return result;
  }

  // best_solution is a raw pointer into the task's internal storage.
  // It remains valid as long as result.task is alive — both live in PlannedPickPlace.
  result.best_solution = result.task->solutions().front().get();

  RCLCPP_INFO(LOGGER,
    "planPickAndPlace [%s]: planning succeeded — %zu solution(s), best cost=%.3f.",
    entry.object_id.c_str(),
    result.task->solutions().size(),
    result.best_solution->cost());

  // Extract the terminal planning scene from the solution's end InterfaceState.
  // PlanningScene::clone() produces a fully independent non-const copy, which is
  // safe to pass to FixedState::setState() for the next task. Using const_pointer_cast
  // would alias the Task's internal scene — risky if MTC mutates it later.
  if (result.best_solution->end() == nullptr) {
    RCLCPP_ERROR(LOGGER,
      "planPickAndPlace [%s]: solution end state is null — cannot chain next task.",
      entry.object_id.c_str());
  } else {
    result.end_scene = planning_scene::PlanningScene::clone(
      result.best_solution->end()->scene());

    // Debug: log terminal joint state so we can verify chaining is correct.
    const moveit::core::RobotState & end_rs = result.end_scene->getCurrentState();
    const moveit::core::JointModelGroup * jmg =
        end_rs.getRobotModel()->getJointModelGroup("ur_onrobot_manipulator");
    if (jmg) {
      std::vector<double> end_jv;
      end_rs.copyJointGroupPositions(jmg, end_jv);
      const auto & jnames = jmg->getVariableNames();
      RCLCPP_INFO(LOGGER,
        "planPickAndPlace [%s]: terminal joint state (%zu joints) — "
        "will be FixedState start for next task:",
        entry.object_id.c_str(), end_jv.size());
      for (std::size_t ji = 0; ji < end_jv.size() && ji < jnames.size(); ++ji) {
        RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
          jnames[ji].c_str(), end_jv[ji], end_jv[ji] * 180.0 / M_PI);
      }
    }

    // Debug: confirm all expected collision objects are still present in the terminal scene.
    const std::vector<std::string> obj_ids = result.end_scene->getWorld()->getObjectIds();
    RCLCPP_INFO(LOGGER,
      "planPickAndPlace [%s]: terminal scene contains %zu world object(s):",
      entry.object_id.c_str(), obj_ids.size());
    for (const auto & oid : obj_ids) {
      RCLCPP_INFO(LOGGER, "  object: %s", oid.c_str());
    }
  }

  return result;
}


bool WordleBotController::executePlannedTask(PlannedPickPlace & planned)
{
  if (!planned.task || !planned.best_solution) {
    RCLCPP_ERROR(LOGGER,
      "executePlannedTask [%s]: task or solution is null — cannot execute.",
      planned.object_id.c_str());
    return false;
  }

  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER,
      "executePlannedTask [%s]: stop requested before execute — aborting.",
      planned.object_id.c_str());
    return false;
  }

  RCLCPP_INFO(LOGGER,
    "executePlannedTask [%s]: publishing solution for visualisation (cost=%.3f).",
    planned.object_id.c_str(), planned.best_solution->cost());
  planned.task->introspection().publishSolution(*planned.best_solution);
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "executePlannedTask [%s]: executing...", planned.object_id.c_str());
  const auto result = planned.task->execute(*planned.best_solution);

  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER,
      "executePlannedTask [%s]: execution failed (error code %d).",
      planned.object_id.c_str(), result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "executePlannedTask [%s]: execution succeeded.", planned.object_id.c_str());
  return true;
}

bool WordleBotController::doPickAndPlace(const geometry_msgs::msg::Pose & object_pose,
                                         const geometry_msgs::msg::Pose & place_pose,
                                         const std::string & object_id)
{
  RCLCPP_INFO(LOGGER, "doPickAndPlace: building MTC task for object '%s'.", object_id.c_str());

  RCLCPP_INFO(LOGGER,
    "doPickAndPlace: object pose = (%.4f, %.4f, %.4f)  "
    "orient=(%.4f, %.4f, %.4f, %.4f)  "
    "place target = (%.4f, %.4f, %.4f). "
    "MTC planning scene frame: 'world'.",
    object_pose.position.x, object_pose.position.y, object_pose.position.z,
    object_pose.orientation.x, object_pose.orientation.y,
    object_pose.orientation.z, object_pose.orientation.w,
    place_pose.position.x, place_pose.position.y, place_pose.position.z);

  mtc::Task task = createTask(object_pose, place_pose, object_id, nullptr, true);

  try {
    task.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "MTC task init failed: " << e);
    return false;
  }

  RCLCPP_INFO(LOGGER, "doPickAndPlace: planning...");
  moveit::core::MoveItErrorCode plan_result;
  try {
    plan_result = task.plan(15);
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "MTC task planning threw InitStageException: " << e);
    return false;
  }
  if (!plan_result || task.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "MTC task planning failed — no solutions found.");
    return false;
  }

  RCLCPP_INFO(LOGGER,
    "doPickAndPlace: planning succeeded — %zu solution(s) found. "
    "Executing best solution (cost=%.3f).",
    task.solutions().size(), task.solutions().front()->cost());

  RCLCPP_DEBUG(LOGGER, "doPickAndPlace: publishing planned solution for visualization.");
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  task.introspection().publishSolution(*task.solutions().front());

  RCLCPP_INFO(LOGGER, "doPickAndPlace: executing...");
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "MTC task execution failed (error code %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "doPickAndPlace: pick-and-place succeeded.");
  return true;
}

// ---------------------------------------------------------------------------
// Goal Navigation
// Plan and execute free-space goal-pose moves using the OMPL sampling planner.
// Supports scene chaining so multiple goals can be planned sequentially before
// any motion begins.
// ---------------------------------------------------------------------------
// planMoveToGoal         — plan a move to an absolute goal pose; accepts an
//                          optional chained start scene for multi-goal missions
// executePlannedMoveToGoal — execute a previously planned move-to-goal task
// ---------------------------------------------------------------------------

WordleBotController::PlannedMoveToGoal WordleBotController::planMoveToGoal(
  const geometry_msgs::msg::Pose & goal_pose,
  const planning_scene::PlanningScenePtr & start_scene,
  bool include_return_home)
{
  PlannedMoveToGoal result;

  RCLCPP_INFO(LOGGER,
    "planMoveToGoal: building task (start=%s, return_home=%s).",
    start_scene ? "FixedState(chained)" : "CurrentState(live)",
    include_return_home ? "yes" : "no");

  const std::string arm_group = "ur_onrobot_manipulator";

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto task = std::make_unique<mtc::Task>();
  task->stages()->setName("move to goal");
  task->loadRobotModel(node_);
  task->setProperty("group", arm_group);

  if (start_scene == nullptr) {
    task->add(std::make_unique<mtc::stages::CurrentState>("current state"));
  } else {
    auto fixed = std::make_unique<mtc::stages::FixedState>("fixed state");
    fixed->setState(start_scene);
    task->add(std::move(fixed));
  }

  {
    geometry_msgs::msg::PoseStamped goal_stamped;
    goal_stamped.header.frame_id = "world";
    goal_stamped.pose = goal_pose;

    auto stage = std::make_unique<mtc::stages::MoveTo>("move to goal", sampling_planner);
    stage->setGroup(arm_group);
    stage->setGoal(goal_stamped);
    // stage->setPathConstraints(buildPathConstraints());
    task->add(std::move(stage));
  }

  if (include_return_home) {
    auto home = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    home->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    home->setGoal("home");
    task->add(std::move(home));
  }

  try {
    task->init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "planMoveToGoal: task init failed: " << e);
    return result;
  }

  moveit::core::MoveItErrorCode plan_result;
  try {
    plan_result = task->plan(100);
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "planMoveToGoal: planning threw: " << e);
    return result;
  }

  if (!plan_result || task->solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "planMoveToGoal: no solution found.");
    return result;
  }

  result.task = std::move(task);
  result.best_solution = result.task->solutions().front().get();

  RCLCPP_INFO(LOGGER, "planMoveToGoal: planning succeeded — cost=%.3f.",
    result.best_solution->cost());

  if (result.best_solution->end() != nullptr) {
    result.end_scene = planning_scene::PlanningScene::clone(
      result.best_solution->end()->scene());
  } else {
    RCLCPP_WARN(LOGGER, "planMoveToGoal: solution end state is null — chaining may fail.");
  }

  return result;
}

bool WordleBotController::executePlannedMoveToGoal(PlannedMoveToGoal & planned)
{
  if (!planned.task || !planned.best_solution) {
    RCLCPP_ERROR(LOGGER, "executePlannedMoveToGoal: task or solution is null.");
    return false;
  }

  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "executePlannedMoveToGoal: stop requested — aborting.");
    return false;
  }

  RCLCPP_INFO(LOGGER, "executePlannedMoveToGoal: publishing solution (cost=%.3f).",
    planned.best_solution->cost());
  planned.task->introspection().publishSolution(*planned.best_solution);
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "executePlannedMoveToGoal: executing...");
  const auto result = planned.task->execute(*planned.best_solution);

  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "executePlannedMoveToGoal: execution failed (error code %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "executePlannedMoveToGoal: execution succeeded.");
  return true;
}

// ---------------------------------------------------------------------------
// MoveGroupInterface Goal Navigation (USE_MTC_FOR_GOALS == false)
// Sequential plan-then-execute per goal. Three private helpers feed into the
// public moveToGoal entry point.
// ---------------------------------------------------------------------------
// computeBestIK         — IK with warm-start seeding, 2π normalisation,
//                         wrist_3 clamping, no shoulder rejection
// generateCandidatePlans — call move_group_.plan() N times, collect successes
// selectBestPlan        — pick lowest-cost plan by computeTotalJointDisplacement
// moveToGoal            — orchestrator: IK → set target → plan × 5 → best → execute
// ---------------------------------------------------------------------------

std::vector<double> WordleBotController::computeBestIK(
  const moveit::core::RobotStatePtr & current_state,
  const geometry_msgs::msg::Pose & target_pose)
{
  const std::string arm_group = "ur_onrobot_manipulator";
  const auto * jmg = move_group_.getRobotModel()->getJointModelGroup(arm_group);
  if (jmg == nullptr) {
    RCLCPP_ERROR(LOGGER, "computeBestIK: joint model group '%s' not found.", arm_group.c_str());
    return {};
  }
  if (current_state == nullptr) {
    RCLCPP_ERROR(LOGGER, "computeBestIK: current robot state is null.");
    return {};
  }
  if (!jmg->getSolverInstance()) {
    RCLCPP_ERROR(LOGGER, "computeBestIK: no kinematics solver loaded for group '%s'.", arm_group.c_str());
    return {};
  }

  // Warm-start and functional-position config [shoulder_pan, shoulder_lift, elbow,
  // wrist_1, wrist_2, wrist_3] = [65°, -90°, 90°, -90°, -90°, 65°]
  static const std::vector<double> kWarmStart   = {1.1345, -1.5708, 1.5708, -1.5708, -1.5708, 1.1345};
  static const std::vector<double> kFuncPos     = {1.1345, -1.5708, 1.5708, -1.5708, -1.5708, 1.1345};
  static const std::vector<double> kFuncWeights = {0.3, 0.5, 0.5, 0.5, 0.3, 0.3};
  static constexpr int kWarmAttempts  = 5;
  static constexpr int kTotalAttempts = 15;

  std::vector<double> current_joint_values;
  current_state->copyJointGroupPositions(jmg, current_joint_values);

  const auto & joint_names   = jmg->getVariableNames();
  const auto & active_joints = jmg->getActiveJointModels();

  std::vector<double> best_joint_values;
  double best_cost  = std::numeric_limits<double>::infinity();
  int ik_successes  = 0;

  for (int attempt = 0; attempt < kTotalAttempts; ++attempt) {
    moveit::core::RobotState ik_state(*current_state);

    if (attempt < kWarmAttempts) {
      ik_state.setJointGroupPositions(jmg, kWarmStart);
      ik_state.update();
    } else {
      ik_state.setToRandomPositions(jmg);
    }

    if (!ik_state.setFromIK(jmg, target_pose, "gripper_tcp", 0.1)) {
      continue;
    }
    ++ik_successes;

    std::vector<double> candidate;
    ik_state.copyJointGroupPositions(jmg, candidate);

    // 2π normalisation: map each revolute joint to the 2π-equivalent numerically
    // closest to the current state, preventing huge spinning trajectories.
    for (std::size_t i = 0; i < candidate.size() && i < active_joints.size(); ++i) {
      if (active_joints[i]->getType() != moveit::core::JointModel::REVOLUTE) {
        continue;
      }
      const double curr = current_joint_values[i];
      const double raw  = candidate[i];
      double best_norm  = raw;
      double best_dist  = std::abs(raw - curr);
      for (int k = -3; k <= 3; ++k) {
        const double offset = raw + k * 2.0 * M_PI;
        const double dist   = std::abs(offset - curr);
        if (dist < best_dist && active_joints[i]->satisfiesPositionBounds(&offset)) {
          best_dist = dist;
          best_norm = offset;
        }
      }
      candidate[i] = best_norm;
    }

    // Clamp wrist_3 to [-π, π]: UR RTDE reports wrist_3 in this range; values outside
    // it cause a PATH_TOLERANCE_VIOLATED abort (2π position error) on the controller.
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      if (joint_names[i] == "wrist_3_joint" && i < candidate.size()) {
        while (candidate[i] > M_PI)  candidate[i] -= 2.0 * M_PI;
        while (candidate[i] < -M_PI) candidate[i] += 2.0 * M_PI;
      }
    }

    double movement_cost      = 0.0;
    double functional_penalty = 0.0;
    for (std::size_t i = 0; i < candidate.size(); ++i) {
      movement_cost += std::abs(candidate[i] - current_joint_values[i]);
      if (i < kFuncPos.size()) {
        const double d = candidate[i] - kFuncPos[i];
        functional_penalty += kFuncWeights[i] * d * d;
      }
    }
    const double cost = 2.0 * movement_cost + 0.3 * functional_penalty;

    if (cost < best_cost) {
      best_cost         = cost;
      best_joint_values = candidate;
    }
  }

  RCLCPP_INFO(LOGGER, "computeBestIK: %d/%d solutions found. Best cost: %.4f",
    ik_successes, kTotalAttempts, best_cost);

  if (best_joint_values.empty()) {
    RCLCPP_ERROR(LOGGER,
      "computeBestIK: no valid IK solution found for pose (x=%.3f y=%.3f z=%.3f).",
      target_pose.position.x, target_pose.position.y, target_pose.position.z);
  }
  return best_joint_values;
}

std::vector<moveit::planning_interface::MoveGroupInterface::Plan>
WordleBotController::generateCandidatePlans(int num_attempts)
{
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
  plans.reserve(static_cast<std::size_t>(std::max(num_attempts, 0)));

  RCLCPP_INFO(LOGGER, "generateCandidatePlans: generating %d plan attempts.", num_attempts);
  int successes = 0;
  for (int attempt = 0; attempt < num_attempts; ++attempt) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto result = move_group_.plan(plan);
    if (static_cast<bool>(result)) {
      ++successes;
      plans.push_back(plan);
    } else {
      RCLCPP_WARN(LOGGER, "generateCandidatePlans: attempt %d failed (error code %d).",
        attempt, result.val);
    }
  }
  RCLCPP_INFO(LOGGER, "generateCandidatePlans: %d/%d plans succeeded.", successes, num_attempts);
  return plans;
}

moveit::planning_interface::MoveGroupInterface::Plan
WordleBotController::selectBestPlan(
  const std::vector<moveit::planning_interface::MoveGroupInterface::Plan> & plans)
{
  moveit::planning_interface::MoveGroupInterface::Plan best_plan;
  double best_cost = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < plans.size(); ++i) {
    const double cost = computeTotalJointDisplacement(plans[i]);
    RCLCPP_INFO(LOGGER, "selectBestPlan: plan %zu — cost=%.4f rad %s",
      i, cost, cost < best_cost ? "(best so far)" : "(worse)");
    if (cost < best_cost) {
      best_cost = cost;
      best_plan = plans[i];
    }
  }
  RCLCPP_INFO(LOGGER, "selectBestPlan: best cost=%.4f rad total joint displacement.", best_cost);
  return best_plan;
}

bool WordleBotController::moveToGoal(const geometry_msgs::msg::Pose & goal_pose)
{
  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "moveToGoal: stop requested — aborting before start.");
    return false;
  }

  RCLCPP_INFO(LOGGER,
    "moveToGoal: target pos (x=%.3f y=%.3f z=%.3f) quat (x=%.3f y=%.3f z=%.3f w=%.3f).",
    goal_pose.position.x, goal_pose.position.y, goal_pose.position.z,
    goal_pose.orientation.x, goal_pose.orientation.y,
    goal_pose.orientation.z, goal_pose.orientation.w);

  move_group_.setStartStateToCurrentState();
  current_state = move_group_.getCurrentState(2.0);
  if (!current_state) {
    RCLCPP_ERROR(LOGGER, "moveToGoal: getCurrentState returned null — state monitor not ready.");
    return false;
  }

  const std::string arm_group = "ur_onrobot_manipulator";
  const auto * jmg = move_group_.getRobotModel()->getJointModelGroup(arm_group);

  // Log current joint state for diagnostics.
  if (jmg) {
    std::vector<double> q_start;
    current_state->copyJointGroupPositions(jmg, q_start);
    const auto & jnames = jmg->getVariableNames();
    RCLCPP_INFO(LOGGER, "moveToGoal: current joint state (%zu joints):", q_start.size());
    for (std::size_t i = 0; i < q_start.size() && i < jnames.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
        jnames[i].c_str(), q_start[i], q_start[i] * 180.0 / M_PI);
    }
  }

  const std::vector<double> best_q = computeBestIK(current_state, goal_pose);
  if (best_q.empty()) {
    RCLCPP_ERROR(LOGGER, "moveToGoal: IK failed — no valid solution found.");
    return false;
  }

  // Log target joint values for diagnostics.
  if (jmg) {
    const auto & jnames = jmg->getVariableNames();
    RCLCPP_INFO(LOGGER, "moveToGoal: target joint values (%zu joints):", best_q.size());
    for (std::size_t i = 0; i < best_q.size() && i < jnames.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  %s = %.4f rad (%.1f deg)",
        jnames[i].c_str(), best_q[i], best_q[i] * 180.0 / M_PI);
    }
  }

  // Seed the planner from the same state snapshot used for IK to avoid a race
  // where the robot drifts slightly between getCurrentState and plan().
  move_group_.setStartState(*current_state);
  move_group_.setJointValueTarget(best_q);

  const auto plans = generateCandidatePlans(5);
  if (plans.empty()) {
    RCLCPP_ERROR(LOGGER, "moveToGoal: all 5 planning attempts failed.");
    return false;
  }

  const auto plan = selectBestPlan(plans);
  if (plan.trajectory_.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(LOGGER, "moveToGoal: selectBestPlan returned an empty plan.");
    return false;
  }

  if (stop_requested_.load()) {
    RCLCPP_INFO(LOGGER, "moveToGoal: stop requested — aborting before execute.");
    return false;
  }

  RCLCPP_INFO(LOGGER, "moveToGoal: executing plan (total_joint_disp=%.4f rad).",
    computeTotalJointDisplacement(plan));

  const auto exec_result = move_group_.execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "moveToGoal: execution failed (error code %d).", exec_result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "moveToGoal: goal reached.");
  return true;
}

// ---------------------------------------------------------------------------
// Scan and Sweep
// Four-pose camera scan sequence. Pose 1 is reached via free-space OMPL
// (Connect + ComputeIK). Poses 2-4 are straight-line Cartesian sweeps
// (MoveRelative) along the world-frame delta between consecutive scan poses.
// ---------------------------------------------------------------------------
// createScanAndSweepTask — build the unified MTC task
// runScanAndSweep        — execute the full four-pose scan sequence
// ---------------------------------------------------------------------------

mtc::Task WordleBotController::createScanAndSweepTask(
  const std::vector<geometry_msgs::msg::Pose> & scan_poses,
  const planning_scene::PlanningScenePtr & start_scene)
{
  const std::string arm_group = "ur_onrobot_manipulator";
  const std::string hand_group = "ur_onrobot_gripper";
  const std::string hand_frame = "gripper_tcp";

  RCLCPP_INFO(LOGGER, "createScanAndSweepTask: scan pose 1 target  xyz=(%.3f, %.3f, %.3f)",
    scan_poses[0].position.x, scan_poses[0].position.y, scan_poses[0].position.z);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.001);

  mtc::Task task;
  task.stages()->setName("scan and sweep");
  task.loadRobotModel(node_);
  task.setProperty("group",    arm_group);
  task.setProperty("eef",      hand_group);
  task.setProperty("ik_frame", hand_frame);

  // Stage 1: current robot state (or chained fixed state).
  mtc::Stage * current_state_ptr = nullptr;
  {
    if (start_scene == nullptr) {
      auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state_ptr = stage.get();
      task.add(std::move(stage));
    } else {
      auto stage = std::make_unique<mtc::stages::FixedState>("fixed start");
      stage->setState(start_scene);
      current_state_ptr = stage.get();
      task.add(std::move(stage));
    }
  }

  // Stage 3: free-space OMPL move toward scan pose 1 — mirrors "move to pick" in pick-and-place.
  {
    auto connect = std::make_unique<mtc::stages::Connect>(
      "move to scan 1",
      mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
    connect->setTimeout(0.2);
    connect->properties().configureInitFrom(mtc::Stage::PARENT);
    // connect->setPathConstraints(WordleBotController::buildPathConstraints());
    task.add(std::move(connect));
  }

  // Stage 3: IK resolution for scan pose 1.
  {
    auto gen_pose = std::make_unique<mtc::stages::GeneratePose>("generate scan pose 1");
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "world";
    target.pose = scan_poses[0];
    gen_pose->setPose(target);
    gen_pose->setMonitoredStage(current_state_ptr);

    auto ik = std::make_unique<mtc::stages::ComputeIK>("scan pose 1 IK", std::move(gen_pose));
    ik->setMaxIKSolutions(32);
    ik->setMinSolutionDistance(0.1);
    ik->setIKFrame(hand_frame);
    ik->setProperty("default_pose", std::string("test_configuration"));
    ik->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "eef"});
    ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    task.add(std::move(ik));
  }

  // Stages 4-6: straight-line Cartesian sweep between scan poses 1→2→3→4.
  // MoveRelative moves the IK frame along the world-frame delta between consecutive poses.
  for (size_t i = 1; i < scan_poses.size(); ++i) {
    const auto & from = scan_poses[i - 1];
    const auto & to   = scan_poses[i];

    const double dx   = to.position.x - from.position.x;
    const double dy   = to.position.y - from.position.y;
    const double dz   = to.position.z - from.position.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    RCLCPP_INFO(LOGGER,
      "createScanAndSweepTask: sweep %zu→%zu  delta=(%.3f, %.3f, %.3f)  dist=%.4f m",
      i, i + 1, dx, dy, dz, dist);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.x = dx / dist;
    vec.vector.y = dy / dist;
    vec.vector.z = dz / dist;

    auto stage = std::make_unique<mtc::stages::MoveRelative>(
      "move to scan " + std::to_string(i + 1), cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setIKFrame(hand_frame);
    stage->setDirection(vec);
    stage->setMinMaxDistance(dist * 0.95, dist * 1.05);
    task.add(std::move(stage));
  }

  return task;
}

bool WordleBotController::runScanAndSweep(
  const std::vector<geometry_msgs::msg::Pose> & poses)
{
  if (poses.size() != 4) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: expected 4 poses, got %zu.", poses.size());
    return false;
  }

  RCLCPP_INFO(LOGGER, "runScanAndSweep: starting.");
  for (size_t i = 0; i < poses.size(); ++i) {
    RCLCPP_INFO(LOGGER,
      "runScanAndSweep: pose %zu  xyz=(%.3f, %.3f, %.3f)  quat=(%.3f, %.3f, %.3f, %.3f)",
      i + 1,
      poses[i].position.x,    poses[i].position.y,    poses[i].position.z,
      poses[i].orientation.x, poses[i].orientation.y,
      poses[i].orientation.z, poses[i].orientation.w);
  }

  // ── All 4 poses: unified MTC scan-and-sweep task from current robot state ─
  if (stop_requested_.load()) { return false; }
  RCLCPP_INFO(LOGGER, "runScanAndSweep: building scan task.");
  auto sweep_task = createScanAndSweepTask(poses, nullptr);

  try {
    sweep_task.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "runScanAndSweep: sweep task init failed: " << e);
    return false;
  }

  RCLCPP_INFO(LOGGER, "runScanAndSweep: planning sweep (all 4 poses).");
  moveit::core::MoveItErrorCode plan_result;
  try {
    plan_result = sweep_task.plan(15);
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "runScanAndSweep: sweep task planning threw: " << e);
    return false;
  }

  if (!plan_result || sweep_task.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: planning failed — no solutions found.");
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: check above MTC stage tree for the failing stage.");
    return false;
  }

  RCLCPP_INFO(LOGGER,
    "runScanAndSweep: sweep task planned — %zu solution(s), best cost=%.3f. Executing.",
    sweep_task.solutions().size(), sweep_task.solutions().front()->cost());

  sweep_task.introspection().publishSolution(*sweep_task.solutions().front());
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  if (stop_requested_.load()) { return false; }
  const auto exec_result = sweep_task.execute(*sweep_task.solutions().front());
  if (exec_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: sweep execution failed (error code %d).",
      exec_result.val);
    return false;
  }

  // ── Return home ───────────────────────────────────────────────────────────
  RCLCPP_INFO(LOGGER, "runScanAndSweep: returning to home.");
  returnToHome();

  RCLCPP_INFO(LOGGER, "runScanAndSweep: complete.");
  return true;
}

// ---------------------------------------------------------------------------
// Standalone Arm Motions
// Simple one-shot MTC tasks for moving the arm to known named states.
// These are available while no mission is running (IDLE state).
// ---------------------------------------------------------------------------
// returnToHome  — move the arm to the SRDF "home" named state
// openGripper   — move the gripper to the SRDF "open" named state
// closeGripper  — move the gripper to the SRDF "closed" named state
// ---------------------------------------------------------------------------

bool WordleBotController::returnToHome()
{
  RCLCPP_INFO(LOGGER, "returnToHome: building MTC task.");

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  mtc::Task task;
  task.stages()->setName("return home");
  task.loadRobotModel(node_);
  task.setProperty("group", std::string("ur_onrobot_manipulator"));

  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
  stage->setGoal("home");
  task.add(std::move(stage));

  try {
    task.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "returnToHome: task init failed: " << e);
    return false;
  }

  if (!task.plan(15) || task.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "returnToHome: planning failed — no solutions found.");
    return false;
  }

  task.introspection().publishSolution(*task.solutions().front());
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "returnToHome: execution failed (error code %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "returnToHome: succeeded.");
  return true;
}

bool WordleBotController::openGripper()
{
  RCLCPP_INFO(LOGGER, "openGripper: building MTC task.");

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  mtc::Task task;
  task.stages()->setName("open gripper");
  task.loadRobotModel(node_);

  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage->setGroup("ur_onrobot_gripper");
  stage->setGoal("open");
  task.add(std::move(stage));

  try {
    task.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "openGripper: task init failed: " << e);
    return false;
  }

  if (!task.plan(15) || task.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "openGripper: planning failed — no solutions found.");
    return false;
  }

  task.introspection().publishSolution(*task.solutions().front());
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "openGripper: execution failed (error code %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "openGripper: succeeded.");
  return true;
}

bool WordleBotController::closeGripper()
{
  RCLCPP_INFO(LOGGER, "closeGripper: building MTC task.");

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  mtc::Task task;
  task.stages()->setName("close gripper");
  task.loadRobotModel(node_);

  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage->setGroup("ur_onrobot_gripper");
  stage->setGoal("closed");
  task.add(std::move(stage));

  try {
    task.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "closeGripper: task init failed: " << e);
    return false;
  }

  if (!task.plan(15) || task.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "closeGripper: planning failed — no solutions found.");
    return false;
  }

  task.introspection().publishSolution(*task.solutions().front());
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "closeGripper: execution failed (error code %d).", result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "closeGripper: succeeded.");
  return true;
}

// ---------------------------------------------------------------------------
// Motion Control
// Interrupt and reset helpers for the stop/resume lifecycle.
// ---------------------------------------------------------------------------
// stop          — cancel the in-progress trajectory and set the stop flag;
//                 any blocking execute() call will return a non-SUCCESS code
// clearStopFlag — clear the stop flag before issuing a new motion so the
//                 motion is not immediately rejected
// ---------------------------------------------------------------------------

void WordleBotController::stop()
{
  stop_requested_.store(true);
  move_group_.stop();
  RCLCPP_INFO(LOGGER, "stop(): trajectory cancelled.");
}

void WordleBotController::clearStopFlag()
{
  stop_requested_.store(false);
}

// ---------------------------------------------------------------------------
// Helper Functions
// Stateless utility functions used across planning and scene management.
// ---------------------------------------------------------------------------
// buildPose                    — construct a geometry_msgs::Pose from XYZ
//                                position and RPY orientation
// computeTotalJointDisplacement — compute Σ|Δq| across all joints and
//                                 trajectory steps (L1 joint-space path length)
// buildPathConstraints         — return the MoveIt path constraints used by
//                                MTC planning stages
// ---------------------------------------------------------------------------

geometry_msgs::msg::Pose WordleBotController::buildPose(
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  quat.normalize();

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();

  return pose;
}


double WordleBotController::computeTotalJointDisplacement(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  const auto & points = plan.trajectory_.joint_trajectory.points;
  double total = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto & prev = points[i - 1].positions;
    const auto & curr = points[i].positions;
    for (std::size_t j = 0; j < prev.size() && j < curr.size(); ++j) {
      total += std::abs(curr[j] - prev[j]);
    }
  }
  return total;
}

moveit_msgs::msg::Constraints WordleBotController::buildPathConstraints()
{
  moveit_msgs::msg::Constraints constraints;

  // Keep the gripper facing straight down throughout all transit moves.
  // This shrinks OMPL's search space significantly, preventing tumbling and
  // reducing joint excursions on free-space Connect/MoveTo stages.
  // Tolerances allow small tilts (±0.4 rad ≈ ±23°) but keep yaw free (±π).
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header.frame_id = "world";
  oc.link_name = "gripper_tcp";
  oc.orientation.x = 1.0;        // roll=π → gripper Z-axis points straight down
  oc.orientation.y = 0.0;
  oc.orientation.z = 0.0;
  oc.orientation.w = 0.0;
  oc.absolute_x_axis_tolerance = 0.4;
  oc.absolute_y_axis_tolerance = 0.4;
  oc.absolute_z_axis_tolerance = M_PI;
  oc.weight = 1.0;

  constraints.orientation_constraints.push_back(oc);
  return constraints;
}
