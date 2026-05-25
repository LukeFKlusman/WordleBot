# wordlebot_control

Main control package for the WordleBot UR3e robotic arm.

Once the program is running it will sit in idle until an object is requested to be picked up, or a goal is requested.

## Prerequisites

- ROS 2 (Humble or later)
- MoveIt 2
- UR Driver
- UR ROS 2 driver MoveIt config (`ur_moveit_config`)
- On Robot Driver

Follow canvas steps on what packages to install

---

## How to Run

### Terminal 1. Build the package

From the workspace root:

```bash
colcon build --packages-select wordlebot_control
source install/setup.bash
```

### Terminal 2. Launch Robot driver

```bash
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 use_fake_hardware:=true launch_rviz:=false
# or use real hardware
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=192.168.0.194 launch_rviz:=false 
# using the id of the ur3e
```

### Terminal 3. Launch MoveIt 

This starts the UR3e MoveIt stack. Keep this running throughout your session.

```bash
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2 launch_rviz:=false
# or use real hardware
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2 launch_rviz:=false robot_ip:=192.168.0.194
# using the id of the ur3e
```

### Terminal 4. Launch Rviz

This starts Rviz

```bash
ros2 launch ur_onrobot_hello_moveit tutorials_rviz.launch.py
```

Alternatively, launch RViz with the wordlebot-specific config:

```bash
ros2 launch wordlebot_control wordlebot_rviz.launch.py
```


### Terminal 5. Launch the control node (Terminal 2)

Only ran after the previous terminals are running

```bash
ros2 launch wordlebot_control wordle_bot_mtc.launch.py
```

### Other Terminal.

Only ran after the previous terminals are running

```bash

ros2 launch ur_onrobot_hello_moveit tutorials_rviz.launch.py

ros2 launch wordlebot_control wordle_bot.launch.py
```

## Manual Terminal Control

Use these commands to control the robot from the terminal without an integration layer. Run each in a separate terminal after the control node is up.

### Start the mission

First load a goal (or set of goals), then send the start signal.

```bash
# Send a single goal pose (legacy interface — auto-starts the mission)
ros2 topic pub --once /wordle_bot/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.4, y: 0.0, z: 0.3}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"

# Or: load a multi-goal mission then start it in two steps
ros2 topic pub --once /wordle_bot/set_mission geometry_msgs/msg/PoseArray \
  "{header: {frame_id: 'world'}, poses: [{position: {x: 0.3, y: 0.15, z: 0.175}, orientation: {x: 1.0, y: 0.0, z: -0.6820, w: 0.7320}}]}"

ros2 topic pub --once /wordle_bot/start_mission std_msgs/msg/Bool "{data: true}"
```

### Stop (pause) the mission

```bash
ros2 topic pub --once /wordle_bot/stop_mission std_msgs/msg/Bool "{data: true}"
```

### Resume after a stop

```bash
ros2 topic pub --once /wordle_bot/resume_mission std_msgs/msg/Bool "{data: true}"
```

### Abort the mission

Cancels execution and sends the robot back to home.

```bash
ros2 topic pub --once /wordle_bot/abort_mission std_msgs/msg/Bool "{data: true}"
```

### Add a collision object to the planning scene

```bash
ros2 topic pub --once /wordle_bot/add_collision_object moveit_msgs/msg/CollisionObject \
  "{id: 'my_box', header: {frame_id: 'world'}, \
    primitives: [{type: 1, dimensions: [0.05, 0.05, 0.05]}], \
    primitive_poses: [{position: {x: 0.3, y: 0.2, z: 0.1}, orientation: {w: 1.0}}], \
    operation: 0}"
```

`operation: 0` = ADD, `operation: 2` = REMOVE.

### Send a letter/wordle object for pick-and-place

This triggers the pick-and-place sequence using the custom `PickPlaceTask` message.
`pick_pose` and `place_pose` are provided in the `world` frame; `object_id` is the MoveIt collision object ID (e.g. `C_object_1`).
With the default `move_group` backend these poses are treated as exact `gripper_tcp` targets. With the `mtc` backend they retain the existing MTC object-pose semantics.

> In normal operation this is published automatically by `hl_control_node`. Use the command below only for direct testing without the HL control layer.

Pick-and-place has two selectable backends in `config/wordle_bot_controller.yaml`:

```yaml
pick_place:
  backend: "move_group"  # default exact-pose sequential backend
  # backend: "mtc"       # existing MoveIt Task Constructor plan-all-then-execute backend
```

`move_group` plans and executes each phase live, so it can use the requested pick/place orientations directly. `mtc` keeps the staged MTC task visualization and whole-task planning behavior, but requested pick/place orientation control has proven unreliable for this application.

```bash
ros2 topic pub --once /perception/letter_objects wordlebot_control/msg/PickPlaceTask \
  "{pick_pose: {header: {frame_id: 'world'}, pose: {position: {x: -0.35, y: 0.1, z: 0.025}, orientation: {w: 1.0}}},
    place_pose: {position: {x: -0.15, y: 0.225, z: 0.025}, orientation: {w: 1.0}},
    object_id: 'C_object_1'}"

ros2 topic pub --once /wordle_bot/start_mission std_msgs/msg/Bool "{data: true}"
```

### Clear all letter collision objects

Removes every letter object added this session from the planning scene and resets the pick-and-place queue. Only works when the robot is idle (not mid-mission).

```bash
ros2 topic pub --once /wordle_bot/clear_letter_objects std_msgs/msg/Bool "{data: true}"
```

### Monitor robot state

```bash
ros2 topic echo /wordle_bot/robot_state
```

---

## How to Test (without intergration)

```bash
# All Tests 
python3 -m pytest src/wordlebot_control/test/tc1_key_control_concepts.py -s -v

# Run TC1.1 in isolation: Move to 1 Goal
python3 -m pytest src/wordlebot_control/test/tc1_key_control_concepts.py -k tc1_1 -s -v

# Run TC1.2 in isolation: Movge to 3 Goal
python3 -m pytest src/wordlebot_control/test/tc1_key_control_concepts.py -k tc1_2 -s -v

# Run TC1.5 in isolation: Collision Avoidance
python3 -m pytest src/wordlebot_control/test/tc1_key_control_concepts.py -k tc1_5 -s -v

# Run TC1.6 in isolation: Pick and Place 
python3 -m pytest src/wordlebot_control/test/tc1_key_control_concepts.py -k tc1_6 -s -v
```
