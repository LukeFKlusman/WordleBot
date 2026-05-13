# WordleBot Control Technical Documentation

## Project Overview

`wordlebot_control` is the ROS 2 control package for the WordleBot UR3e robotic arm and OnRobot RG2 gripper. It provides MoveIt 2 based motion planning, mission execution, collision-scene management, scan sweeps, and pick-and-place control for letter blocks.

### Key Features

- Topic-driven mission control for one or more target poses.
- MoveIt Task Constructor (MTC) pick-and-place pipeline.
- Static and dynamic MoveIt planning-scene collision objects.
- On-demand gripper open/close and return-home commands.
- Four-pose scan-and-sweep sequence for camera/perception workflows.
- RViz configuration and pytest-based subsystem tests.

## Dependencies

### Hardware

| Item | Purpose | Notes |
| --- | --- | --- |
| Universal Robots UR3e arm | Manipulator | Tested target robot for this package. |
| OnRobot RG2 gripper | End effector | Used for letter block grasping. |
| Control PC | ROS 2 and MoveIt host | See computing specs below. |
| Network connection to UR controller | Robot communication | Real robot examples in this document use `192.168.0.191`; update for your robot. |
| WordleBot work area | Physical task board / workspace | TODO: add final dimensions and safe operating boundary. |
| Letter blocks | Pick-and-place objects | Current collision model assumes `0.05 m x 0.05 m x 0.05 m` blocks. |
| Camera / perception sensor | Letter detection and scan sweep | TODO: add exact sensor model and mounting details. |
| Custom sensor mount / guard | Collision-aware tooling | The controller attaches a `sensor_guard` cylinder to `tool0`. |

### Computing Specs

TODO: add tested workstation specs.

Recommended fields:

- CPU:
- RAM:
- GPU, if used:
- Operating system:
- ROS 2 distribution:
- Network adapter / robot network setup:

### Software

Install these before building `wordlebot_control`:

- Ubuntu 22.04.
- ROS 2 Humble.
- MoveIt 2 for Humble.
- Universal Robots ROS 2 driver and UR description packages.
- UR + OnRobot ROS 2 packages from `UR_OnRobot_ROS2`.
- Python 3 and `pytest` for the test scripts.

External installation guides:

- MoveIt 2 Humble getting started: <https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html>
- OnRobot gripper / UR OnRobot ROS 2 installation: <https://github.com/tonydle/UR_OnRobot_ROS2>

Package-level ROS dependencies are declared in `package.xml` and `CMakeLists.txt`, including:

- `rclcpp`
- `geometry_msgs`
- `std_msgs`
- `moveit_msgs`
- `shape_msgs`
- `tf2`
- `moveit_ros_planning_interface`
- `moveit_visual_tools`
- `moveit_task_constructor_core`
- `controller_manager_msgs`

## Installation

### Hardware Setup

1. Mount the OnRobot RG2 gripper to the UR3e flange.
2. Mount the camera or perception sensor to the end effector.
3. Check that the mounted sensor is represented by the controller's attached `sensor_guard` collision cylinder.
4. Place the robot, task board, and letter blocks inside the agreed work area.
5. Confirm the robot controller IP address.
6. Confirm that the emergency stop and teach pendant are accessible before running motion.

TODO: add photos of:

- Robot and gripper assembly.
- Sensor mount.
- Work area layout.
- Letter block start and goal positions.

### Software Setup

Follow the MoveIt 2 and OnRobot setup guides first:

1. Install ROS 2 Humble.
2. Complete the MoveIt 2 Humble setup: <https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html>
3. Install and build the UR OnRobot ROS 2 packages: <https://github.com/tonydle/UR_OnRobot_ROS2>

Then install this package in a ROS 2 workspace:

```bash
mkdir -p ~/wordlebot_ws/src
cd ~/wordlebot_ws/src
git clone <WORDLEBOT_REPOSITORY_URL>
cd ~/wordlebot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select wordlebot_control
source install/setup.bash
```

If the package already exists in your workspace, rebuild it from the workspace root:

```bash
cd ~/wordlebot_ws
colcon build --packages-select wordlebot_control
source install/setup.bash
```

## Running the System

Run each command in a separate sourced terminal.

### Terminal 1: Source the Workspace

```bash
cd ~/wordlebot_ws
source install/setup.bash
```

### Terminal 2: Start the UR + OnRobot Driver

Simulation / fake hardware:

```bash
ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  use_fake_hardware:=true \
  launch_rviz:=false
```

Real robot:

```bash
ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  robot_ip:=192.168.0.191 \
  launch_rviz:=false
```

### Terminal 3: Start MoveIt

Simulation / fake hardware:

```bash
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  launch_rviz:=false
```

Real robot:

```bash
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  robot_ip:=192.168.0.191 \
  launch_rviz:=false
```

### Terminal 4: Start RViz

Recommended WordleBot RViz configuration:

```bash
ros2 launch wordlebot_control wordlebot_rviz.launch.py
```

Alternative UR OnRobot tutorial RViz:

```bash
ros2 launch ur_onrobot_hello_moveit tutorials_rviz.launch.py
```

### Terminal 5: Start the WordleBot Control Node

```bash
ros2 launch wordlebot_control wordle_bot_mtc.launch.py
```

Expected outcome:

- The node logs `WordleBotControlNode initialised`.
- The mission thread starts and waits for goals.
- The planning scene contains a floor collision object.
- A `sensor_guard` collision cylinder is attached to `tool0`.
- RViz shows the UR3e, RG2 gripper, planning scene, and MTC visualisation markers.

Add expected screenshots here:

![RViz initial state](images/wordlebot-rviz-initial-state.png)
![Successful pick and place](images/wordlebot-pick-place-success.png)

## Basic Operation

### Monitor Robot State

```bash
ros2 topic echo /wordle_bot/robot_state
```

Expected values:

- `IDLE`: no mission is running.
- `RUNNING`: a goal mission, pick-and-place mission, or scan sweep is active.

### Run a Goal Mission

Load one or more poses, then start the mission:

```bash
ros2 topic pub --once /wordle_bot/set_mission geometry_msgs/msg/PoseArray \
  "{header: {frame_id: 'world'}, poses: [
    {position: {x: 0.4, y: 0.0, z: 0.3}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}
  ]}"

ros2 topic pub --once /wordle_bot/start_mission std_msgs/msg/Bool "{data: true}"
```

Completion signals:

```bash
ros2 topic echo /wordle_bot/goal_reached
ros2 topic echo /wordle_bot/mission_complete
ros2 topic echo /wordle_bot/motion_complete
```

### Stop a Running Mission

```bash
ros2 topic pub --once /wordle_bot/stop_mission std_msgs/msg/Bool "{data: true}"
```

### Resume or Abort

These topics exist, but the callbacks currently log `not yet implemented`:

```bash
ros2 topic pub --once /wordle_bot/resume_mission std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /wordle_bot/abort_mission std_msgs/msg/Bool "{data: true}"
```

### Gripper Commands

These commands are accepted only while the node is `IDLE`:

```bash
ros2 topic pub --once /wordle_bot/open_gripper std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /wordle_bot/close_gripper std_msgs/msg/Bool "{data: true}"
```

### Return Home

```bash
ros2 topic pub --once /wordle_bot/return_home std_msgs/msg/Bool "{data: true}"
```

### Run Scan and Sweep

```bash
ros2 topic pub --once /wordle_bot/scan_and_sweep std_msgs/msg/Bool "{data: true}"
```

The four scan poses and dwell time are configured in `config/scan_sweep_poses.yaml`.

## Subsystem Specifics

### Control Node

Purpose: `WordleBotControlNode` exposes the ROS 2 topic interface, queues missions, and dispatches work to the controller.

Key files:

- `src/wordle_bot_control_node.cpp`
- `include/wordlebot_control/wordle_bot_control_node.hpp`
- `main/main.cpp`
- `launch/wordle_bot_mtc.launch.py`

Publishers:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/wordle_bot/robot_state` | `std_msgs/msg/String` | Publishes `IDLE` or `RUNNING`. |
| `/wordle_bot/goal_reached` | `std_msgs/msg/Bool` | Published after each goal or pick-place task succeeds. |
| `/wordle_bot/mission_complete` | `std_msgs/msg/Bool` | Published after all mission items complete. |
| `/wordle_bot/motion_complete` | `std_msgs/msg/Bool` | Published after a full successful mission. |

Subscriptions:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/wordle_bot/set_mission` | `geometry_msgs/msg/PoseArray` | Queue target poses. |
| `/wordle_bot/start_mission` | `std_msgs/msg/Bool` | Start the queued mission. |
| `/wordle_bot/stop_mission` | `std_msgs/msg/Bool` | Stop current execution. |
| `/wordle_bot/resume_mission` | `std_msgs/msg/Bool` | Exists but not implemented. |
| `/wordle_bot/abort_mission` | `std_msgs/msg/Bool` | Exists but not implemented. |
| `/wordle_bot/add_collision_object` | `moveit_msgs/msg/CollisionObject` | Add, remove, or move planning-scene objects. |
| `/perception/letter_objects` | `wordlebot_control/msg/PickPlaceTask` | Queue a pick-and-place task. |
| `/wordle_bot/clear_letter_objects` | `std_msgs/msg/Bool` | Remove queued/tracked letter objects. |
| `/wordle_bot/open_gripper` | `std_msgs/msg/Bool` | Open gripper while idle. |
| `/wordle_bot/close_gripper` | `std_msgs/msg/Bool` | Close gripper while idle. |
| `/wordle_bot/return_home` | `std_msgs/msg/Bool` | Move to SRDF `home` while idle. |
| `/wordle_bot/scan_and_sweep` | `std_msgs/msg/Bool` | Execute the configured scan sweep. |

Run independently:

```bash
ros2 launch wordlebot_control wordle_bot_mtc.launch.py
```

### Motion Planning Controller

Purpose: `WordleBotController` owns MoveIt 2 planning, MTC task construction, collision-scene updates, gripper moves, and scan-sweep motion.

Key files:

- `src/wordle_bot_controller.cpp`
- `include/wordlebot_control/wordle_bot_controller.hpp`

Important MoveIt groups and frames:

| Name | Use |
| --- | --- |
| `ur_onrobot_manipulator` | Arm planning group. |
| `ur_onrobot_gripper` | Gripper planning group. |
| `gripper_tcp` | End-effector frame used for grasping. |
| `tool0` | Link used for the attached sensor collision object. |
| `world` | Expected frame for pick and place poses. |

Planning behaviour:

- OMPL is the default planning pipeline.
- General motion uses MTC `Connect` stages with path constraints.
- Pick-and-place uses a plan-then-execute workflow.
- Multiple pick-and-place tasks are planned sequentially by chaining terminal planning scenes into the next task.
- The final task in a batch returns the arm to the SRDF `home` named state.

Known assumptions:

- The UR OnRobot MoveIt configuration provides named states `home`, `open`, and `closed`.
- Incoming perception pick poses should be transformed into `world`.
- Letter object collision geometry is currently a fixed `0.05 m` cube.

### Pick-and-Place Interface

Purpose: Receive detected letter objects and move them from a pick pose to a place pose.

Message:

```text
geometry_msgs/PoseStamped pick_pose
geometry_msgs/Pose place_pose
string object_id
```

Example:

```bash
ros2 topic pub --once /perception/letter_objects wordlebot_control/msg/PickPlaceTask \
  "{pick_pose: {header: {frame_id: 'world'}, pose: {position: {x: -0.35, y: 0.1, z: 0.025}, orientation: {w: 1.0}}},
    place_pose: {position: {x: -0.15, y: 0.35, z: 0.025}, orientation: {w: 1.0}},
    object_id: 'C_object_1'}"

ros2 topic pub --once /wordle_bot/start_mission std_msgs/msg/Bool "{data: true}"
```

Clear tracked letter objects:

```bash
ros2 topic pub --once /wordle_bot/clear_letter_objects std_msgs/msg/Bool "{data: true}"
```

### Collision Scene

Purpose: Keep MoveIt aware of the table/floor, attached sensor geometry, letter blocks, and any external obstacles.

Default scene setup:

- Adds `floor`, a `2.0 m x 2.0 m x 0.01 m` box at `z = -0.015`.
- Attaches `sensor_guard`, a cylinder on `tool0`.

Add a collision object:

```bash
ros2 topic pub --once /wordle_bot/add_collision_object moveit_msgs/msg/CollisionObject \
  "{id: 'my_box', header: {frame_id: 'world'},
    primitives: [{type: 1, dimensions: [0.05, 0.05, 0.05]}],
    primitive_poses: [{position: {x: 0.3, y: 0.2, z: 0.1}, orientation: {w: 1.0}}],
    operation: 0}"
```

Use `operation: 0` to add and `operation: 2` to remove.

### Scan and Sweep

Purpose: Move the end effector through four configured scan poses for perception.

Key file:

- `config/scan_sweep_poses.yaml`

Parameters:

| Parameter | Meaning |
| --- | --- |
| `scan_sweep_dwell_time` | Seconds to wait at each pose. |
| `scan_sweep_pose_0` | First scan pose, reached by free-space planning. |
| `scan_sweep_pose_1` | Cartesian sweep pose. |
| `scan_sweep_pose_2` | Cartesian sweep pose. |
| `scan_sweep_pose_3` | Cartesian sweep pose. |

Pose format:

```text
[x, y, z, roll, pitch, yaw]
```

Run:

```bash
ros2 topic pub --once /wordle_bot/scan_and_sweep std_msgs/msg/Bool "{data: true}"
```

## Configuration

### Launch Arguments

`launch/wordle_bot_mtc.launch.py` supports:

| Argument | Default | Purpose |
| --- | --- | --- |
| `ur_type` | `ur3e` | UR robot model. |
| `onrobot_type` | `rg2` | OnRobot gripper model. |
| `safety_limits` | `true` | Enable safety limit controller settings. |
| `safety_pos_margin` | `0.15` | Joint safety position margin. |
| `safety_k_position` | `20` | Safety controller k-position factor. |
| `prefix` | `""` | Joint-name prefix. |

Example:

```bash
ros2 launch wordlebot_control wordle_bot_mtc.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2
```

### Important Config Files

| File | Purpose |
| --- | --- |
| `config/scan_sweep_poses.yaml` | Scan sweep pose and dwell-time parameters. |
| `config/kinematics.yaml` | Package-local kinematics parameters, kept for launch/reference compatibility. |
| `rviz/wordlebot_control.rviz` | RViz display configuration. |
| `msg/PickPlaceTask.msg` | Pick-and-place task interface. |

## Testing

Build before running tests:

```bash
cd ~/wordlebot_ws
colcon build --packages-select wordlebot_control
source install/setup.bash
```

Run package tests:

```bash
colcon test --packages-select wordlebot_control
colcon test-result --verbose
```

Run specific pytest files:

```bash
python3 -m pytest src/wordlebot_control/test/tc1_key_control_concepts.py -s -v
python3 -m pytest src/wordlebot_control/test/tc2_advanced_motion_control.py -s -v
python3 -m pytest src/wordlebot_control/test/tc3_advanced_collision_avoidance.py -s -v
```

Run selected TC1 tests:

```bash
python3 -m pytest src/wordlebot_control/test/tc1_key_control_concepts.py -k tc1_1 -s -v
python3 -m pytest src/wordlebot_control/test/tc1_key_control_concepts.py -k tc1_2 -s -v
python3 -m pytest src/wordlebot_control/test/tc1_key_control_concepts.py -k tc1_5 -s -v
python3 -m pytest src/wordlebot_control/test/tc1_key_control_concepts.py -k tc1_6 -s -v
```

## Troubleshooting and FAQs

### The control node starts but no motion happens

Check that:

- The UR + OnRobot driver is running.
- MoveIt is running.
- The workspace was sourced in every terminal.
- `/wordle_bot/set_mission` or `/perception/letter_objects` was published before `/wordle_bot/start_mission`.
- `/wordle_bot/robot_state` changes from `IDLE` to `RUNNING`.

### MTC planning fails with no solution

Check that:

- The target pose is reachable by the UR3e.
- The pose is in the `world` frame.
- Collision objects are not blocking all valid paths.
- The object ID in `PickPlaceTask.object_id` matches the collision object being picked.
- The robot is not already in collision in RViz.

### Pick-and-place receives a non-`world` frame

The node will warn if `pick_pose.header.frame_id` is not `world`. Transform perception outputs into `world` before publishing to `/perception/letter_objects`.

### The gripper does not open or close

Check that:

- The OnRobot driver is installed and running.
- The MoveIt SRDF provides `open` and `closed` named states for `ur_onrobot_gripper`.
- No mission is currently running. Gripper utility commands are ignored while `mission_running_` is true.

### `resume_mission` or `abort_mission` does not work

The topics exist, but the callbacks currently log `resume not yet implemented` and `abort not yet implemented`. Use `/wordle_bot/stop_mission` for the currently implemented stop pathway.

### RViz does not show the planned task

Check that:

- `wordlebot_rviz.launch.py` or the UR OnRobot tutorial RViz launch is running.
- The fixed frame is compatible with the robot planning frame.
- MTC introspection markers are enabled in RViz.
- The control node logs a successful plan before execution.

### Collision objects remain in the scene

Clear letter objects:

```bash
ros2 topic pub --once /wordle_bot/clear_letter_objects std_msgs/msg/Bool "{data: true}"
```

For custom collision objects, republish the same object ID with `operation: 2`.

## Known Limitations

- Resume and abort topics are declared but not implemented.
- Hardware BOM, workstation specs, and final physical workspace documentation still need project-specific values.
- Scan-sweep poses in `config/scan_sweep_poses.yaml` are marked as placeholders and should be validated on the physical robot.
- Letter collision geometry is fixed to a `0.05 m` cube.
- The pick-and-place pipeline assumes perception publishes poses in or transformable to the `world` frame.
- Real robot commands must be checked against the current robot IP address and safety setup before use.
