# WordleBot Control Technical Documentation

## Project overview

`wordlebot_control` is the ROS 2 control package that turns high-level WordleBot commands into UR3e arm and OnRobot RG2 gripper motion. It owns the MoveIt planning scene, mission queue, pick-and-place execution, scan sweep motion, gripper commands, and recovery behaviour.

This page explains the control package internals. Full-stack installation, launch ordering, terminal setup, and system-level screenshots belong in the wider control-system overview page.

## Key features

- ROS topic interface for goals, pick-and-place tasks, collision objects, scan sweep, gripper commands, stop, resume, and abort.
- Mission worker thread that serialises robot actions so only one mission runs at a time.
- Default exact-pose `MoveGroupInterface` pick-and-place backend.
- Optional MoveIt Task Constructor backend for long-horizon plan-all-then-execute pick-and-place.
- Collision-aware IK and planning using MoveIt planning scene monitoring.
- Static scene setup for floor, gameboard, and an attached sensor guard.
- Six-pose scan-and-sweep routine for camera/perception coverage.
- Motion scoring to reduce unnecessary joint travel and wrist spinning.
- Stop/resume/abort recovery logic for interrupted missions.

## Dependencies

The package is written for ROS 2 Humble with MoveIt 2 and the UR + OnRobot ROS 2 stack used by the project. The package-level dependencies are declared in `package.xml` and `CMakeLists.txt`.

Important runtime dependencies:

| Dependency | Purpose |
| --- | --- |
| `rclcpp` | ROS 2 node, publishers, subscribers, parameters, threading. |
| `geometry_msgs`, `std_msgs` | Mission pose messages, command booleans, state strings. |
| `moveit_ros_planning_interface` | `MoveGroupInterface` and `PlanningSceneInterface`. |
| `moveit_ros_planning` | Planning scene monitor and collision checking. |
| `moveit_task_constructor_core` | Optional MTC pick-and-place and scan task construction. |
| `moveit_task_constructor_msgs` | Executes selected MTC solutions through the `execute_task_solution` action. |
| `moveit_visual_tools` | RViz marker support for planned solutions. |
| `shape_msgs`, `moveit_msgs` | Collision object geometry and planning scene updates. |
| `tf2` | Quaternion and roll/pitch/yaw conversion. |
| `controller_manager_msgs` | Controller integration dependency for the robot stack. |

Hardware assumptions:

- Universal Robots UR3e arm.
- OnRobot RG2 gripper.
- `ur_onrobot_manipulator` MoveIt planning group.
- `ur_onrobot_gripper` MoveIt gripper group.
- `gripper_tcp` as the pick/place TCP frame.
- `tool0` as the sensor guard attachment link.
- World-frame board area with a gameboard collision object centred around `x=0.0`, `y=0.225`.

## Package structure

| File | Role |
| --- | --- |
| `main/main.cpp` | Creates `WordleBotControlNode`, sets up the scene, and spins the process. |
| `include/wordlebot_control/wordle_bot_control_node.hpp` | ROS-facing mission node interface and mission state. |
| `src/wordle_bot_control_node.cpp` | Topic callbacks, mission queueing, stop/resume/abort, scan command dispatch. |
| `include/wordlebot_control/wordle_bot_controller.hpp` | Motion controller API and planning helper declarations. |
| `src/wordle_bot_controller.cpp` | MoveIt scene management, MGI motion, MTC task building, gripper control, recovery. |
| `include/wordlebot_control/wordle_mtc_planner.hpp` | Custom MTC planner interface. |
| `src/wordle_mtc_planner.cpp` | MTC OMPL candidate planning, IK selection, and trajectory scoring. |
| `msg/PickPlaceTask.msg` | Perception-to-control pick/place command. |
| `config/wordle_bot_controller.yaml` | Main runtime parameters. |
| `config/wordle_mtc_planner.yaml` | Custom MTC planner tuning. |
| `config/scan_sweep_poses.yaml` | Scan pose reference configuration. |
| `config/kinematics.yaml` | MoveIt IK solver configuration. |
| `launch/wordle_bot_mtc.launch.py` | Starts the control node with controller/planner parameters. |
| `rviz/wordlebot_control.rviz` | RViz layout for observing robot state, scene, and MTC markers. |
| `test/` | Pytest and C++ tests for control concepts, collision avoidance, and wrist alignment. |

## Runtime interface

The node name is `wordle_bot_control_node`.

### Command topics

| Topic | Type | Meaning |
| --- | --- | --- |
| `/wordle_bot/set_mission` | `geometry_msgs/msg/PoseArray` | Replaces the queued direct goal mission. |
| `/wordle_bot/start_mission` | `std_msgs/msg/Bool` | Starts queued goals or queued pick-and-place tasks when `data=true`. |
| `/wordle_bot/stop_mission` | `std_msgs/msg/Bool` | Requests a safe stop of the active mission. |
| `/wordle_bot/resume_mission` | `std_msgs/msg/Bool` | Resumes after the node has entered `STOPPED`. |
| `/wordle_bot/abort_mission` | `std_msgs/msg/Bool` | Aborts after `STOPPED`, clears queues/objects, and returns to working joints. |
| `/wordle_bot/add_collision_object` | `moveit_msgs/msg/CollisionObject` | Adds, removes, or moves an external scene object. |
| `/perception/letter_objects` | `wordlebot_control/msg/PickPlaceTask` | Queues one detected letter pick-and-place task. |
| `/wordle_bot/clear_letter_objects` | `std_msgs/msg/Bool` | Clears tracked letter objects and pending pick/place tasks. |
| `/wordle_bot/clear_board_objects` | `std_msgs/msg/Bool` | Alias for clearing tracked board objects. |
| `/wordle_bot/open_gripper` | `std_msgs/msg/Bool` | Opens the gripper while idle. |
| `/wordle_bot/close_gripper` | `std_msgs/msg/Bool` | Closes the gripper while idle. |
| `/wordle_bot/return_home` | `std_msgs/msg/Bool` | Moves to configured `working_joints` while idle. |
| `/wordle_bot/scan_and_sweep` | `std_msgs/msg/Bool` | Runs the scan-and-sweep sequence while idle. |

### Status topics

| Topic | Type | Meaning |
| --- | --- | --- |
| `/wordle_bot/robot_state` | `std_msgs/msg/String` | Publishes `IDLE`, `RUNNING`, or `STOPPED`. |
| `/wordle_bot/goal_reached` | `std_msgs/msg/Bool` | Publishes `true` after each completed goal or pick/place task. |
| `/wordle_bot/mission_complete` | `std_msgs/msg/Bool` | Publishes `true` after all queued mission work succeeds. |
| `/wordle_bot/motion_complete` | `std_msgs/msg/Bool` | Publishes `true` at the end of a successful mission. |

## Control architecture

The package is split into two layers:

1. `WordleBotControlNode` is the ROS mission layer. It creates publishers/subscribers, stores queued work, manages state transitions, and runs the mission worker thread.
2. `WordleBotController` is the robot-motion layer. It owns `MoveGroupInterface`, `PlanningSceneInterface`, `PlanningSceneMonitor`, visual tools, scene objects, IK helpers, MGI execution, MTC task creation, and recovery motions.

This split keeps ROS command handling separate from motion-planning details. The mission node decides what should happen next. The controller decides how to make the robot do it.

## Mission lifecycle

The mission worker runs in `WordleBotControlNode::missionLoop()`.

Normal mission flow:

1. A user or high-level node publishes goal poses or pick/place tasks.
2. `/wordle_bot/start_mission` sets `mission_armed_=true` and wakes the worker.
3. The worker snapshots the current queue under `queue_mutex_`.
4. `robot_state` is published as `RUNNING`.
5. The worker chooses pick-and-place if `pick_place_queue_` is non-empty; otherwise it executes direct goal poses.
6. Each completed goal/task publishes `goal_reached=true`.
7. A fully successful mission publishes `mission_complete=true` and `motion_complete=true`.
8. `robot_state` returns to `IDLE`.

Only one active mission is allowed. Manual gripper, return-home, and scan commands check `mission_running_` and are ignored while a mission is active.

### Stop, resume, and abort

`/wordle_bot/stop_mission` sets an atomic stop flag and calls `WordleBotController::stop()`, which forwards `move_group_.stop()` to the active MoveIt execution. The worker then enters `STOPPED`, saves the unexecuted tasks, and waits for resume or abort.

Resume behaviour:

- The controller checks whether the gripper appears closed.
- If the gripper is closed, `recoverObject()` moves to a safe recovery pose, detaches the held object if known, opens the gripper, and returns to the working pose.
- Remaining tasks are requeued and the worker starts another mission cycle.

Abort behaviour:

- Clears active queues.
- Clears tracked letter and board objects from the planning scene.
- Returns the arm to `working_joints`.
- Publishes `IDLE`.

## Planning scene design

`WordleBotController::setupCollisionScene()` builds the static collision context:

- `floor`: a large `2.0 x 2.0 x 0.01 m` plane below the workspace.
- `gameboard`: a `0.957 x 0.525 x 0.0149 m` board object.
- `sensor_guard`: a cylinder attached to `tool0`, with gripper touch links allowed.

The planning scene monitor subscribes to `/monitored_planning_scene` and requests scene state from `/get_planning_scene`. This matters because IK and collision checks should use the same scene that MoveIt is planning against. The code waits for world or attached objects to appear/disappear after scene updates, which reduces race conditions where planning begins before MoveIt has received the object update.

During pick-and-place, all known collision objects are inserted before planning/execution. In the MGI backend, the selected target object is removed just before the final exact Cartesian approach so the robot can move into the grasp pose without colliding with the object it is about to pick. After grasping, an attached collision object is applied to `gripper_tcp`. After release, the object is detached and re-added to the world at the final place pose.

## MGI and MTC design decision

The package contains two planning styles because they solve different problems.

| Aspect | MoveGroupInterface (MGI) | MoveIt Task Constructor (MTC) |
| --- | --- | --- |
| Current default for pick/place | Yes, `pick_place.backend: "move_group"` | Optional, `pick_place.backend: "mtc"` |
| Planning horizon | Short horizon: plan/execute one phase at a time | Long horizon: build a full task and solve the sequence before execution |
| Control over exact poses | Strong. The code directly targets exact `gripper_tcp` pick/place poses. | Weaker for this project. Stages can sample IK and place alternatives, but exact repeatable orientation control has been harder to enforce. |
| Best for | Repeatable movements that need specific positions, orientations, and recovery behaviour. | Variable tasks where a full staged plan can reason about pick, carry, place, collision changes, and terminal scene chaining. |
| Limitations | Myopic. Each phase only knows the immediate target, so it cannot optimise the whole mission at once. | More fragile to build, tune, and debug. Specific movements can fail at a stage or produce unexpected IK/orientation choices. |
| Code path | `executePickAndPlaceMoveGroup()`, `moveToGoal()`, `moveCartesianToWaypointWithScaling()` | `createTask()`, `planPickAndPlace()`, `executePlannedTask()`, `WordleMtcPlanner` |

The design choice in the current code is practical: MGI is used by default because WordleBot needs repeatable, exact letter movements. MTC remains available because it is the more complete task-planning model and is useful when the planner should account for the entire pick/place operation before the robot moves.

### Why MGI is the default

The high-level system provides concrete pick and place poses. For letter placement, small orientation and position errors matter. The MGI backend treats those incoming poses as exact `gripper_tcp` targets and executes a known sequence:

1. Open gripper.
2. Move to a pre-pick pose above the pick pose.
3. Remove the target object from the world collision scene.
4. Cartesian approach to the exact pick pose.
5. Close gripper.
6. Attach the object to `gripper_tcp`.
7. Cartesian lift.
8. Move to a pre-place pose.
9. Cartesian descent to exact place pose.
10. Open gripper, with yaw recovery if the gripper fails to open.
11. Detach object and re-add it to the world at the place pose.
12. Cartesian retreat.
13. Return to the working pose if this is the last task.

This gives the package direct control over each physical phase. It also makes failures easier to identify because the logs name the failed phase.

### Why MTC is still included

MTC describes a task as a set of connected stages. For a pick/place operation, it can reason about opening the gripper, moving to pick, generating grasp poses, allowing collisions, attaching the object, lifting, moving to place, releasing, detaching, retreating, and returning home as one connected plan. In a multi-object batch, the code also chains the terminal planning scene from one planned task into the next task's `FixedState`, so later plans account for earlier placed objects.

The tradeoff is that MTC adds more stage-level constraints and more ways to fail. It is powerful for variable long-horizon planning, but less convenient when the robot must perform the same exact TCP movement repeatedly.

## MGI internals

The main MGI primitive is `moveToGoal()`.

`moveToGoal()` does more than call MoveIt once:

1. Chooses velocity/acceleration scaling from current scene clearance.
2. Reads the current robot state.
3. Refreshes the planning scene and combines the live joint state with the monitored scene state.
4. Runs collision-aware IK with `computeBestIK()`.
5. If IK fails, attempts a wrist-3 normalisation recovery and retries IK.
6. Validates both start and goal robot states for collision.
7. Sets a joint-value target from the selected IK result.
8. Generates one or more OMPL candidate plans until the configured success count or timeout.
9. Selects the plan with the lowest total joint displacement.
10. Aligns continuous wrist-3 trajectory values to the current revolution.
11. Executes the selected plan.

### MGI IK generation

`WordleBotController::computeBestIK()` searches for a usable joint solution for a requested `gripper_tcp` pose.

The search process:

- Uses the MoveIt kinematics solver for `ur_onrobot_manipulator`.
- Starts with five warm-start attempts near a known functional posture.
- Continues with random joint seeds up to the total attempt count.
- Uses a planning-scene collision callback inside `setFromIK()` so IK candidates that collide are rejected immediately.
- Normalises revolute joints by adding or subtracting whole `2*pi` rotations to keep each candidate close to the current joint state.
- Rejects candidates where `wrist_1_joint` is outside the configured functional range.
- Scores the accepted candidates and returns the lowest-cost one.

The cost function is:

```text
cost = 2.0 * movement_cost + 0.3 * functional_penalty
```

Where:

- `movement_cost` is the sum of absolute joint differences from the current state.
- `functional_penalty` is a weighted squared distance from a preferred functional posture.

This cost function is a design decision: it avoids solutions that technically reach the pose but require large joint motion or awkward wrist/arm configurations. It favours smooth, nearby, repeatable configurations.

### MGI path optimisation

After IK selects a joint target, `generateCandidatePlans()` repeatedly asks OMPL for candidate trajectories until it reaches `pick_place.mgi_planning_min_successes` or `pick_place.mgi_planning_timeout`.

`selectBestPlan()` scores each trajectory with `computeTotalJointDisplacement()`:

```text
path_cost = sum(abs(delta_joint_position)) over all joints and all trajectory waypoints
```

The lowest path cost is selected. This is simple but useful for the UR3e because it discourages large unnecessary joint excursions.

### Cartesian segments

`moveCartesianToWaypointWithScaling()` is used for precise approach, lift, descent, retreat, and scan sweeps. It calls `computeCartesianPath()` with several step sizes. If the Cartesian fraction is below the required threshold, or execution fails, it falls back to `moveToGoal()`.

This gives exact straight-line behaviour when MoveIt can solve it, while still allowing the robot to recover with a regular joint-space plan when a Cartesian segment is infeasible.

## MTC internals

The optional MTC pick/place backend is built by `WordleBotController::createTask()`.

MTC task stages:

| Stage | Name | Purpose |
| --- | --- | --- |
| 1 | `current` or `fixed start` | Uses live state for the first task or a chained terminal scene for later tasks. |
| 2 | `open hand` | Opens the RG2 to the operational width. |
| 3 | `move to pick` | Free-space OMPL connection to the pick region. |
| 4a | `approach object` | Cartesian approach along the TCP z-axis. |
| 4b | `generate grasp pose` + `grasp pose IK` | Samples grasp yaw alternatives and solves IK. |
| 4c | `allow collision (hand,object)` | Allows gripper/object contact for grasping. |
| 4d | `close hand` | Closes the RG2 to the operational width. |
| 4e | `attach object` | Attaches the letter object to `gripper_tcp`. |
| 4f | `lift object` | Cartesian lift in world z. |
| 5 | `move to place` | Free-space OMPL connection to the place region. |
| 6a | `generate place pose` + `place pose IK` | Solves IK for the requested world place pose. |
| 6b | `open hand` | Releases the object. |
| 6c | `forbid collision (hand,object)` | Restores normal collision checking. |
| 6d | `detach object` | Detaches the object from the gripper. |
| 6e | `retreat` | Cartesian retreat in world z. |
| 7 | `return to working pose` | Added only to the final task in a batch. |

`planPickAndPlace()` plans incrementally. It requests one complete solution, scores it, then requests more up to `pick_place.task_solution_target_count`. If a good enough solution is found early, based on `pick_place.accept_solution_score_threshold`, planning stops early.

The complete MTC solution score is:

```text
score =
  joint_motion
  + wrist_spin_weight * wrist_spin
  + place_yaw_penalty
  + 0.001 * mtc_solution_cost
```

Where:

- `joint_motion` is total joint movement across all sub-trajectories.
- `wrist_spin` is movement from `wrist_2_joint` and `wrist_3_joint`.
- `place_yaw_penalty` increases when the selected place yaw exceeds the configured yaw tolerance.
- `mtc_solution_cost` is the internal MTC cost, included with a small weight.

Solutions above `solution_wrist_spin_reject_threshold` are rejected unless all solutions are rejected, in which case the code falls back to the lowest-scored rejected solution so the mission can still continue if needed.

## Custom MTC planner

`WordleMtcPlanner` is a custom MTC `PlannerInterface`. It exists because the default MTC pipeline planner generally returns the first valid trajectory, while this project needs to reduce excessive wrist spinning and unnecessary joint movement.

For pose goals inside MTC, `WordleMtcPlanner`:

1. Computes a collision-aware IK solution for the target link pose.
2. Builds a joint-goal `MotionPlanRequest`.
3. Generates several OMPL candidate trajectories.
4. Scores each trajectory using joint motion plus weighted wrist spin.
5. Rejects candidates above the wrist-spin threshold.
6. Returns the lowest-cost accepted trajectory.

The MTC IK cost function mirrors the MGI approach but is parameterised from `config/wordle_mtc_planner.yaml`:

```text
ik_cost =
  ik_movement_cost_weight * movement_cost
  + ik_functional_cost_weight * functional_penalty
```

Key parameters:

- `wordle_mtc_planner.candidate_plans`
- `wordle_mtc_planner.ik_warm_attempts`
- `wordle_mtc_planner.ik_total_attempts`
- `wordle_mtc_planner.ik_retry_multiplier`
- `wordle_mtc_planner.ik_warm_start_joints`
- `wordle_mtc_planner.ik_functional_reference_joints`
- `wordle_mtc_planner.ik_functional_weights`
- `wordle_mtc_planner.ik_wrist_1_min`
- `wordle_mtc_planner.ik_wrist_1_max`
- `wordle_mtc_planner.trajectory_wrist_spin_weight`
- `wordle_mtc_planner.trajectory_wrist_spin_reject_threshold`

## Scan-and-sweep subsystem

The scan sweep uses six configured poses:

1. Move to the scan start pose.
2. Cartesian sweep to the first scan endpoint.
3. Reorient at the same point.
4. Cartesian sweep across the board.
5. Reorient at the far side.
6. Cartesian sweep back/down to the final pose.

By default, `USE_MTC_FOR_SCAN_SWEEP` is `false`, so the scan path uses MGI:

- `moveToGoal()` for reorientation and non-Cartesian moves.
- `moveCartesianToWaypoint()` for straight-line camera sweeps.
- Dwell time after scan poses using `scan_sweep_dwell_time`.
- `returnToWorkingPose()` at the end.

An MTC scan path still exists behind `USE_MTC_FOR_SCAN_SWEEP`, but the current default uses MGI because the scan route is a fixed repeatable path and benefits from direct control.

## Configurable settings

Main configuration file: `config/wordle_bot_controller.yaml`.

| Parameter | Meaning |
| --- | --- |
| `working_joints.*` | Joint target used by return-home/working-pose behaviours. |
| `scan_sweep_pose_0` to `scan_sweep_pose_5` | Scan poses as `[x, y, z, roll, pitch, yaw]`. |
| `scan_sweep_dwell_time` | Seconds to pause at scan poses. |
| `pick_place.backend` | `"move_group"` for MGI default, `"mtc"` for MTC backend. |
| `pick_place.task_solution_target_count` | MTC solution target count. |
| `pick_place.grasp_max_ik_solutions` | Max MTC grasp IK alternatives. |
| `pick_place.place_max_ik_solutions` | Max MTC place IK alternatives. |
| `pick_place.grasp_angle_delta` | Angular spacing for MTC grasp sampling. |
| `pick_place.cartesian_step_size` | MTC Cartesian planner step size. |
| `pick_place.move_to_pick_timeout` | MTC free-space pick connection timeout. |
| `pick_place.move_to_place_timeout` | MTC free-space place connection timeout. |
| `pick_place.solution_wrist_spin_weight` | MTC full-solution wrist spin score weight. |
| `pick_place.solution_wrist_spin_reject_threshold` | MTC full-solution wrist spin rejection threshold. |
| `pick_place.accept_solution_score_threshold` | Early-accept threshold for incremental MTC planning. |
| `pick_place.place_yaw_tolerance` | Allowed place yaw error before penalty. |
| `pick_place.place_yaw_penalty_weight` | Weight for place yaw error beyond tolerance. |
| `pick_place.approach_min_distance`, `approach_max_distance` | MTC approach distance bounds. |
| `pick_place.lift_min_distance`, `lift_max_distance` | MTC lift distance bounds. |
| `pick_place.retreat_min_distance`, `retreat_max_distance` | MTC retreat distance bounds. |
| `pick_place.grasp_z_offset` | TCP/object z offset used by MTC grasp frame. |
| `pick_place.mgi_pre_pick_distance` | MGI pre-pick vertical offset. |
| `pick_place.mgi_lift_distance` | MGI lift distance after grasp. |
| `pick_place.mgi_pre_place_distance` | MGI pre-place vertical offset. |
| `pick_place.mgi_retreat_distance` | MGI retreat distance after release. |
| `pick_place.mgi_cartesian_min_fraction` | Required Cartesian path fraction before fallback. |
| `pick_place.mgi_planning_timeout` | MGI candidate planning timeout. |
| `pick_place.mgi_planning_min_successes` | Number of successful MGI plans to collect before selection. |
| `pick_place.mgi_place_open_recovery_yaw_delta` | Yaw adjustment if opening at place pose fails. |
| `pick_place.gripper_open_operational_width` | RG2 width used for operational open. |
| `pick_place.gripper_closed_operational_width` | RG2 width used for operational close. |
| `velocity_scaling.scan.*` | Slow scan motion scaling. |
| `velocity_scaling.precise.*` | Approach/lift/descent/retreat scaling. |
| `velocity_scaling.transit.*` | Free-space transit scaling. |
| `velocity_scaling.proximity.*` | Scene-clearance thresholds for transit speed selection. |

## Running and testing this package

Full launch instructions should be kept in the full-stack overview page. For package-level validation after the workspace is built and sourced:

```bash
colcon test --packages-select wordlebot_control
colcon test-result --verbose --test-result-base build/wordlebot_control
```

Useful direct checks while the full stack is running:

```bash
ros2 topic echo /wordle_bot/robot_state
ros2 topic echo /wordle_bot/goal_reached
ros2 topic echo /wordle_bot/mission_complete
ros2 topic echo /wordle_bot/motion_complete
```

Direct pick/place message shape:

```text
wordlebot_control/msg/PickPlaceTask
  geometry_msgs/PoseStamped pick_pose
  geometry_msgs/Pose place_pose
  string object_id
```

The normal source of this message is the high-level control/perception layer. Direct terminal publishing is useful only for isolated package testing.

## Known limitations and assumptions

- The default MGI backend is short-horizon. It gives better exact pose control but does not optimise the complete mission as one plan.
- The optional MTC backend is long-horizon, but has been less reliable for exact repeatable place orientation in this application.
- `USE_MTC_FOR_GOALS` and `USE_MTC_FOR_SCAN_SWEEP` are compile-time constants in `WordleBotController`, not runtime parameters.
- The MGI IK warm start and functional posture are hard-coded inside `computeBestIK()`. The MTC planner equivalent is configurable in `wordle_mtc_planner.yaml`.
- The code assumes the MoveIt group/link names from the UR + OnRobot configuration: `ur_onrobot_manipulator`, `ur_onrobot_gripper`, `gripper_tcp`, and `tool0`.
- The sensor guard geometry is a simple cylinder approximation, not a detailed camera/end-effector mesh.
- Scene propagation is handled with state waits, but MoveIt scene timing can still be a source of intermittent planning failures if the wider stack is not fully ready.
- Resume after a stop re-executes remaining tasks; it does not continue from a partially executed trajectory waypoint.

## Troubleshooting

### Node stays idle after publishing goals

Check that `/wordle_bot/start_mission` was published after `/wordle_bot/set_mission`, and check `/wordle_bot/robot_state`. The worker only starts when the mission is armed and at least one queue is non-empty.

### Pick/place task does not run

Check that the task was published on `/perception/letter_objects`, not `/wordle_bot/perception/letter_objects`. The code subscribes to `perception/letter_objects`, which resolves to `/perception/letter_objects` for the node.

### Planning fails before motion starts

Look for these log messages:

- `computeBestIK: no valid IK solution`
- `collision preflight failed`
- `sensor_guard ... timed out`
- `world object ... timed out`

These usually mean the target pose is unreachable, the target collides with the scene, the scene has not synchronised, or the attached sensor guard is missing.

### Robot makes excessive wrist motion

For MGI, inspect `selectBestPlan` logs and consider increasing `pick_place.mgi_planning_min_successes` or `pick_place.mgi_planning_timeout` so more candidates are available. For MTC, tune `trajectory_wrist_spin_weight` and `trajectory_wrist_spin_reject_threshold` in `wordle_mtc_planner.yaml`, plus `pick_place.solution_wrist_spin_weight` for full-task solution selection.

### Cartesian approach or retreat fails

`moveCartesianToWaypointWithScaling()` logs the Cartesian fraction for each step size. If all fractions are below `pick_place.mgi_cartesian_min_fraction`, the code falls back to `moveToGoal()`. Repeated failures usually mean the requested straight line is blocked by collision geometry or requires an infeasible wrist orientation.

### Gripper fails to release at the place pose

The MGI backend closes the gripper again, rotates the place yaw by `pick_place.mgi_place_open_recovery_yaw_delta`, moves to the recovered pose, and retries opening. If this still fails, check the place pose clearance and the operational gripper widths.

### MTC stage fails

Use RViz MTC introspection and the stage name in the log. Common failing stages are:

- `move to pick`: free-space connection cannot find a path.
- `grasp pose IK`: no grasp candidate can satisfy IK/collision constraints.
- `move to place`: carrying the object blocks the transit path.
- `place pose IK`: requested place pose/yaw cannot be reached.
- `retreat`: Cartesian retreat is blocked or too constrained.

### Scene object appears in the wrong place

Check the collision object's `header.frame_id`, `primitive_poses`, and `operation`. This package expects world-frame pick/place commands for normal operation.

