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
    stage_move_to_pick->setPathConstraints(WordleBotController::buildPathConstraints());
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
    stage->setPathConstraints(WordleBotController::buildPathConstraints());
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
      wrapper->setMaxIKSolutions(4);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(object_id);
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
    plan_result = result.task->plan(5);
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
    plan_result = task.plan(5);
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

  constexpr int NUM_PLANNING_ATTEMPTS = 5;
  std::unique_ptr<mtc::Task> best_task;
  const mtc::SolutionBase * best_solution_ptr = nullptr;
  double best_cost = std::numeric_limits<double>::infinity();

  for (int attempt = 1; attempt <= NUM_PLANNING_ATTEMPTS; ++attempt) {
    auto sampling_planner      = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
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

    auto stage = std::make_unique<mtc::stages::MoveTo>("move to goal", sampling_planner);
    stage->setGroup(arm_group);
    geometry_msgs::msg::PoseStamped goal_stamped;
    goal_stamped.header.frame_id = "world";
    goal_stamped.pose = goal_pose;
    stage->setGoal(goal_stamped);
    stage->setPathConstraints(buildPathConstraints());
    task->add(std::move(stage));

    if (include_return_home) {
      auto home = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
      home->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      home->setGoal("home");
      task->add(std::move(home));
    }

    try {
      task->init();
    } catch (const mtc::InitStageException & e) {
      RCLCPP_ERROR_STREAM(LOGGER,
        "planMoveToGoal attempt " << attempt << "/" << NUM_PLANNING_ATTEMPTS
        << ": task init failed: " << e);
      continue;
    }

    moveit::core::MoveItErrorCode plan_result;
    try {
      plan_result = task->plan(1);
    } catch (const mtc::InitStageException & e) {
      RCLCPP_ERROR_STREAM(LOGGER,
        "planMoveToGoal attempt " << attempt << "/" << NUM_PLANNING_ATTEMPTS
        << ": planning threw: " << e);
      continue;
    }

    if (!plan_result || task->solutions().empty()) {
      RCLCPP_WARN(LOGGER, "planMoveToGoal attempt %d/%d: no solution found.",
        attempt, NUM_PLANNING_ATTEMPTS);
      continue;
    }

    double cost = task->solutions().front()->cost();
    RCLCPP_INFO(LOGGER, "planMoveToGoal attempt %d/%d: solution found, cost=%.3f.",
      attempt, NUM_PLANNING_ATTEMPTS, cost);

    if (cost < best_cost) {
      best_cost = cost;
      best_task = std::move(task);
      best_solution_ptr = best_task->solutions().front().get();
    }
  }

  if (!best_task) {
    RCLCPP_ERROR(LOGGER, "planMoveToGoal: no solutions found across %d attempts.",
      NUM_PLANNING_ATTEMPTS);
    return result;
  }

  result.task = std::move(best_task);
  result.best_solution = best_solution_ptr;

  RCLCPP_INFO(LOGGER,
    "planMoveToGoal: planning succeeded — best cost=%.3f across %d attempts.",
    result.best_solution->cost(), NUM_PLANNING_ATTEMPTS);

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
// Scan and Sweep
// Cartesian move-to-goal planning and the four-pose camera scan sequence.
// Pose 0 is reached via free-space OMPL planning; poses 1-3 are Cartesian
// straight-line moves chained from the previous pose's terminal scene.
// ---------------------------------------------------------------------------
// planMoveToGoalCartesian — plan a Cartesian (straight-line) move to a goal
//                           pose using the CartesianPath solver
// runScanAndSweep         — execute the full four-pose scan sequence with a
//                           configurable dwell time at each pose
// ---------------------------------------------------------------------------

WordleBotController::PlannedMoveToGoal WordleBotController::planMoveToGoalCartesian(
  const geometry_msgs::msg::Pose & goal_pose,
  const planning_scene::PlanningScenePtr & start_scene)
{
  PlannedMoveToGoal result;

  RCLCPP_INFO(LOGGER,
    "planMoveToGoalCartesian: building task (start=%s).",
    start_scene ? "FixedState(chained)" : "CurrentState(live)");

  const std::string arm_group = "ur_onrobot_manipulator";

  constexpr int NUM_PLANNING_ATTEMPTS = 5;
  std::unique_ptr<mtc::Task> best_task;
  const mtc::SolutionBase * best_solution_ptr = nullptr;
  double best_cost = std::numeric_limits<double>::infinity();

  for (int attempt = 1; attempt <= NUM_PLANNING_ATTEMPTS; ++attempt) {
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.5);
    cartesian_planner->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner->setStepSize(0.001);

    auto task = std::make_unique<mtc::Task>();
    task->stages()->setName("move to goal cartesian");
    task->loadRobotModel(node_);
    task->setProperty("group", arm_group);

    if (start_scene == nullptr) {
      task->add(std::make_unique<mtc::stages::CurrentState>("current state"));
    } else {
      auto fixed = std::make_unique<mtc::stages::FixedState>("fixed state");
      fixed->setState(start_scene);
      task->add(std::move(fixed));
    }

    auto stage = std::make_unique<mtc::stages::MoveTo>("move to goal cartesian", cartesian_planner);
    stage->setGroup(arm_group);
    geometry_msgs::msg::PoseStamped goal_stamped;
    goal_stamped.header.frame_id = "world";
    goal_stamped.pose = goal_pose;
    stage->setGoal(goal_stamped);
    task->add(std::move(stage));

    try {
      task->init();
    } catch (const mtc::InitStageException & e) {
      RCLCPP_ERROR_STREAM(LOGGER,
        "planMoveToGoalCartesian attempt " << attempt << "/" << NUM_PLANNING_ATTEMPTS
        << ": task init failed: " << e);
      continue;
    }

    moveit::core::MoveItErrorCode plan_result;
    try {
      plan_result = task->plan(1);
    } catch (const mtc::InitStageException & e) {
      RCLCPP_ERROR_STREAM(LOGGER,
        "planMoveToGoalCartesian attempt " << attempt << "/" << NUM_PLANNING_ATTEMPTS
        << ": planning threw: " << e);
      continue;
    }

    if (!plan_result || task->solutions().empty()) {
      RCLCPP_WARN(LOGGER, "planMoveToGoalCartesian attempt %d/%d: no solution found.",
        attempt, NUM_PLANNING_ATTEMPTS);
      continue;
    }

    double cost = task->solutions().front()->cost();
    RCLCPP_INFO(LOGGER, "planMoveToGoalCartesian attempt %d/%d: solution found, cost=%.3f.",
      attempt, NUM_PLANNING_ATTEMPTS, cost);

    if (cost < best_cost) {
      best_cost = cost;
      best_task = std::move(task);
      best_solution_ptr = best_task->solutions().front().get();
    }
  }

  if (!best_task) {
    RCLCPP_ERROR(LOGGER, "planMoveToGoalCartesian: no solutions found across %d attempts.",
      NUM_PLANNING_ATTEMPTS);
    return result;
  }

  result.task = std::move(best_task);
  result.best_solution = best_solution_ptr;

  RCLCPP_INFO(LOGGER,
    "planMoveToGoalCartesian: planning succeeded — best cost=%.3f across %d attempts.",
    result.best_solution->cost(), NUM_PLANNING_ATTEMPTS);

  if (result.best_solution->end() != nullptr) {
    result.end_scene = planning_scene::PlanningScene::clone(
      result.best_solution->end()->scene());
  } else {
    RCLCPP_WARN(LOGGER, "planMoveToGoalCartesian: solution end state is null — chaining may fail.");
  }

  return result;
}

bool WordleBotController::runScanAndSweep(
  const std::vector<geometry_msgs::msg::Pose> & poses,
  double dwell_time_seconds)
{
  if (poses.size() != 4) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: expected 4 poses, got %zu.", poses.size());
    return false;
  }

  const auto dwell = std::chrono::duration<double>(dwell_time_seconds);
  RCLCPP_INFO(LOGGER, "runScanAndSweep: starting (dwell=%.2f s).", dwell_time_seconds);

  // ── Pose 0: free-space move to starting scan position ────────────────────
  if (stop_requested_.load()) { return false; }
  RCLCPP_INFO(LOGGER, "runScanAndSweep: planning pose 0 (OMPL free-space).");
  auto planned0 = planMoveToGoal(poses[0], nullptr, false);
  if (!planned0.task) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: planning pose 0 failed.");
    return false;
  }
  if (!executePlannedMoveToGoal(planned0)) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: execution to pose 0 failed.");
    return false;
  }
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(dwell));

  // ── Pose 1: Cartesian sweep ───────────────────────────────────────────────
  if (stop_requested_.load()) { return false; }
  RCLCPP_INFO(LOGGER, "runScanAndSweep: planning pose 1 (Cartesian).");
  auto planned1 = planMoveToGoalCartesian(poses[1], planned0.end_scene);
  if (!planned1.task) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: planning pose 1 failed.");
    return false;
  }
  if (!executePlannedMoveToGoal(planned1)) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: execution to pose 1 failed.");
    return false;
  }
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(dwell));

  // ── Pose 2: Cartesian sweep ───────────────────────────────────────────────
  if (stop_requested_.load()) { return false; }
  RCLCPP_INFO(LOGGER, "runScanAndSweep: planning pose 2 (Cartesian).");
  auto planned2 = planMoveToGoalCartesian(poses[2], planned1.end_scene);
  if (!planned2.task) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: planning pose 2 failed.");
    return false;
  }
  if (!executePlannedMoveToGoal(planned2)) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: execution to pose 2 failed.");
    return false;
  }
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(dwell));

  // ── Pose 3: Cartesian sweep ───────────────────────────────────────────────
  if (stop_requested_.load()) { return false; }
  RCLCPP_INFO(LOGGER, "runScanAndSweep: planning pose 3 (Cartesian).");
  auto planned3 = planMoveToGoalCartesian(poses[3], planned2.end_scene);
  if (!planned3.task) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: planning pose 3 failed.");
    return false;
  }
  if (!executePlannedMoveToGoal(planned3)) {
    RCLCPP_ERROR(LOGGER, "runScanAndSweep: execution to pose 3 failed.");
    return false;
  }
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(dwell));

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

  if (!task.plan(5) || task.solutions().empty()) {
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

  if (!task.plan(5) || task.solutions().empty()) {
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

  if (!task.plan(5) || task.solutions().empty()) {
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
  return constraints;
}
