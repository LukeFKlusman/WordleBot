# WordleBot Full Stack Overview

## Project overview

WordleBot is a ROS 2 robotic pick-and-place system that uses a UR3e arm and OnRobot RG2 gripper to arrange physical letter tiles into a requested five-letter word. The full stack receives a board state and word request, generates an ordered task sequence with `hl_control`, then executes those tasks with `wordlebot_control` through MoveIt.

This README is the full-stack install, hardware setup, and running guide. Package internals are documented separately in:

- `wordlebot_control/Technical-Documentation.md`
- `hl_control/TECHNICAL_DOCUMENTATION.md`

## Key features and subsystems

| Subsystem | Package / source | Purpose |
| --- | --- | --- |
| Robot driver | `ur_onrobot_control` | Starts the UR3e and OnRobot RG2 hardware or fake hardware. |
| MoveIt stack | `ur_onrobot_moveit_config` | Provides robot model, planning scene, controllers, and MoveIt services/actions. |
| Low-level control | `wordlebot_control` | Queues missions, manages MoveIt scene objects, plans/executed pick-place, scan, gripper, stop/resume/abort. |
| High-level sequencing | `hl_control` | Converts board state and target word into a sequential list of pick-and-place tasks using MaskablePPO. |
| RViz | `wordlebot_control/rviz` | Visualises the robot, planning scene, collision objects, and planned motion. |
| Test publisher | `hl_control/test/test_sim.py` | Publishes a test board state and word request without perception or solver. |

Typical data flow:

```text
/perception/gameboard_state + /hl_control/word_request
        |
        v
hl_control_node
        |
        | publishes collision objects + PickPlaceTask sequence
        v
wordle_bot_control_node
        |
        | waits for /wordle_bot/start_mission
        v
MoveIt + UR/OnRobot drivers
        |
        v
UR3e + RG2 executes pick and place
```

## Dependencies

### Hardware bill of materials

| Item | Purpose | Notes |
| --- | --- | --- |
| Universal Robots UR3e | Main manipulator | Supported robot type in launch files is `ur3e` by default. |
| OnRobot RG2 gripper | Letter tile grasping | Launch default is `onrobot_type:=rg2`. |
| UR control box and teach pendant | Robot controller and safety interface | Must be reachable from the control PC network. |
| Control PC | Runs ROS 2, MoveIt, RViz, and RL inference | See computing specs below. |
| Ethernet connection | PC to UR controller | Default example IP is `192.168.0.194`; confirm your robot IP. |
| Wordle board / work surface | Physical workspace | 13 x 7 logical grid, 75 mm spacing. |
| Letter tiles | Pick/place objects | Collision model uses 50 mm cube objects. |
| End-effector camera or perception sensor | Detects letter poses | Perception must publish `hl_control/GameboardState` in `world`. |
| Sensor mount / guard | Physical camera mounting | Low-level scene approximates this as a `sensor_guard` cylinder attached to `tool0`. |
| Emergency stop access | Safety | E-stop and pendant must be reachable before enabling real robot motion. |

### Computing specs

Recommended tested-class machine:

- Ubuntu 22.04 LTS.
- ROS 2 Humble.
- Python 3.10.
- 16 GB RAM or more.
- Wired Ethernet adapter for the UR controller network.
- GPU is not required for inference; the bundled MaskablePPO model can run on CPU.

### Software

Required software stack:

- ROS 2 Humble.
- MoveIt 2 for Humble.
- Universal Robots ROS 2 driver stack.
- UR + OnRobot ROS 2 packages that provide:
  - `ur_onrobot_control`
  - `ur_onrobot_moveit_config`
  - `ur_onrobot_description`
- This repository containing:
  - `wordlebot_control`
  - `hl_control`
- Python RL dependencies:
  - `stable-baselines3`
  - `sb3-contrib`
  - `gymnasium`
  - `numpy`
  - `matplotlib`
  - `PyYAML`

## Installation

### Hardware setup

1. Place the UR3e on a stable surface and confirm it is bolted or fixed according to the lab setup.
2. Attach the OnRobot RG2 gripper to the UR3e flange.
3. Mount the perception sensor/camera on the end effector if used.
4. Confirm the attached sensor does not collide with the gripper, tiles, or board during expected motion.
5. Place the board in the robot work area with the logical `world` frame matching the software coordinate convention.
6. Lay out the board as a 13 x 7 grid with 75 mm cell spacing.
7. Confirm the five Wordle target slots are centred at:

   | Slot | X (m) | Y (m) |
   | --- | --- | --- |
   | 0 | `-0.150` | `0.225` |
   | 1 | `-0.075` | `0.225` |
   | 2 | `0.000` | `0.225` |
   | 3 | `0.075` | `0.225` |
   | 4 | `0.150` | `0.225` |

8. Confirm letter tiles are approximately 50 mm cubes and can be gripped by the RG2 at the configured operational widths.
9. Connect the control PC to the UR controller network.
10. Confirm the robot IP address from the teach pendant or network settings.
11. Verify E-stop, teach pendant, and workspace are clear before running real hardware.

### UR robot setup before real hardware runs

Before launching real hardware:

1. Power on the UR control box.
2. Release the emergency stop if safe.
3. Boot the robot and confirm all joints are ready.
4. Start or load the required external-control program for the UR ROS 2 driver if your driver setup requires it.
5. Set the robot mode to remote/external control as required by the UR driver installation.
6. Confirm the PC can reach the robot:

```bash
ping 192.168.0.194
```

Replace `192.168.0.194` with the robot's actual IP address.

### Software setup

From a clean workspace:

```bash
mkdir -p ~/wordlebot_ws/src
cd ~/wordlebot_ws/src
git clone <REPOSITORY_URL>
cd ~/wordlebot_ws
```

Install ROS dependencies:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Install Python RL dependencies:

```bash
python3 -m pip install stable-baselines3 sb3-contrib gymnasium numpy matplotlib PyYAML
```

Build the two WordleBot packages:

```bash
cd ~/wordlebot_ws
colcon build --packages-select wordlebot_control hl_control
source install/setup.bash
```

For repeated terminal use, source the workspace in each new terminal:

```bash
cd ~/wordlebot_ws
source install/setup.bash
```

## Running the system

Use fake hardware first. Only switch to real hardware after RViz and the topic pipeline behave as expected.

### Option A: recommended fake-hardware full stack

This uses separate terminals so failures are easier to identify.

#### Terminal 1: robot driver, MoveIt, and RViz

```bash
cd ~/wordlebot_ws
source install/setup.bash
ros2 launch wordlebot_control wordlebot_bringup.launch.py
```

This launches:

- UR + OnRobot fake hardware driver.
- MoveIt after a startup delay.
- WordleBot RViz after MoveIt starts.

Expected outcome:

- Driver starts without requiring a real robot.
- MoveIt starts and publishes the robot model.
- RViz opens with the WordleBot config.
- Robot model is visible in RViz.

#### Terminal 2: low-level + high-level WordleBot nodes

```bash
cd ~/wordlebot_ws
source install/setup.bash
ros2 launch wordlebot_control wordlecontrolfullstack.launch.py
```

Expected outcome:

- `wordle_bot_control_node` logs `WordleBotControlNode initialised`.
- The mission thread starts.
- The collision scene contains `floor`, `gameboard`, and attached `sensor_guard`.
- `hl_control_node` logs that it is waiting for word request and board state.

#### Terminal 3: publish a test board and word

```bash
cd ~/wordlebot_ws/src/hl_control
source ~/wordlebot_ws/install/setup.bash
python3 test/test_sim.py --ros-args \
  -p config_path:=$(pwd)/config/tc2_1_board.yaml \
  -p word:=CRANE
```

Expected outcome:

- `hl_control_node` receives the board state and word request.
- It publishes letter collision objects.
- It logs the ordered task sequence.
- It publishes one `/perception/letter_objects` message per task.
- The robot does not move yet.

#### Terminal 4: start execution

```bash
cd ~/wordlebot_ws
source install/setup.bash
ros2 topic pub --once /wordle_bot/start_mission std_msgs/msg/Bool "{data: true}"
```

Expected outcome:

- `/wordle_bot/robot_state` changes from `IDLE` to `RUNNING`.
- The robot executes each queued pick-and-place task.
- `/wordle_bot/goal_reached` publishes after each task.
- `/wordle_bot/mission_complete` and `/wordle_bot/motion_complete` publish when all tasks complete.
- `/wordle_bot/robot_state` returns to `IDLE`.

### Option B: real hardware full stack

Use this only after the fake-hardware path works.

#### Terminal 1: real robot driver, MoveIt, and RViz

```bash
cd ~/wordlebot_ws
source install/setup.bash
ros2 launch wordlebot_control wordlebot_bringup.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.0.194 \
  launch_rviz:=true
```

Replace `192.168.0.194` with the real UR controller IP.

#### Terminal 2: low-level + high-level WordleBot nodes

```bash
cd ~/wordlebot_ws
source install/setup.bash
ros2 launch wordlebot_control wordlecontrolfullstack.launch.py
```

#### Terminal 3: provide real or test inputs

For the real full system, perception publishes `/perception/gameboard_state` and the word/solver layer publishes `/hl_control/word_request`.

For a controlled hardware test without perception:

```bash
cd ~/wordlebot_ws/src/hl_control
source ~/wordlebot_ws/install/setup.bash
python3 test/test_sim.py --ros-args \
  -p config_path:=$(pwd)/config/tc2_1_board.yaml \
  -p word:=CRANE
```

#### Terminal 4: start execution

Before pressing enter, check RViz, clear the workspace, and keep the E-stop reachable.

```bash
ros2 topic pub --once /wordle_bot/start_mission std_msgs/msg/Bool "{data: true}"
```

### Option C: manual launch terminals

Use this when you want every process isolated.

#### Terminal 1: UR + OnRobot driver

Fake hardware:

```bash
ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  use_fake_hardware:=true \
  launch_rviz:=false
```

Real hardware:

```bash
ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  robot_ip:=192.168.0.194 \
  launch_rviz:=false
```

#### Terminal 2: MoveIt

```bash
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  launch_rviz:=false
```

#### Terminal 3: RViz

```bash
ros2 launch wordlebot_control wordlebot_rviz.launch.py
```

#### Terminal 4: low-level control node

```bash
ros2 launch wordlebot_control wordle_bot_mtc.launch.py
```

#### Terminal 5: high-level control node

```bash
ros2 launch hl_control hl_control.launch.py
```

#### Terminal 6: publish board/word inputs

```bash
cd ~/wordlebot_ws/src/hl_control
python3 test/test_sim.py --ros-args \
  -p config_path:=$(pwd)/config/tc2_1_board.yaml \
  -p word:=CRANE
```

#### Terminal 7: start mission

```bash
ros2 topic pub --once /wordle_bot/start_mission std_msgs/msg/Bool "{data: true}"
```

## Expected visual outcome

In RViz you should see:

- UR3e arm and RG2 gripper.
- Planning scene objects:
  - `floor`
  - `gameboard`
  - `sensor_guard` attached to `tool0`
  - one collision cube for each detected letter
- Planned and executed robot motion when a mission starts.

Suggested wiki images:

```text
images/full-stack-rviz-initial.png
images/full-stack-letter-collision-objects.png
images/full-stack-pick-place-running.png
images/hardware-board-layout.jpg
images/ur3e-rg2-camera-mount.jpg
```

## Subsystem specifics

### Robot driver and MoveIt

Purpose: Connect ROS 2 to the UR3e/RG2 hardware or fake hardware and expose MoveIt planning/execution.

Key launch files:

- `ur_onrobot_control/start_robot.launch.py`
- `ur_onrobot_moveit_config/ur_onrobot_moveit.launch.py`
- `wordlebot_control/launch/wordlebot_bringup.launch.py`

Inputs:

- `robot_ip`
- `use_fake_hardware`
- `ur_type`
- `onrobot_type`

Outputs:

- Robot state.
- Controller interfaces.
- MoveIt planning scene and planning/execution actions.

Independent test:

```bash
ros2 launch wordlebot_control wordlebot_bringup.launch.py
```

### Low-level control: `wordlebot_control`

Purpose: Queue and execute arm/gripper missions.

Main node:

- `wordle_bot_control_node`

Key files:

- `src/wordle_bot_control_node.cpp`
- `src/wordle_bot_controller.cpp`
- `config/wordle_bot_controller.yaml`
- `config/wordle_mtc_planner.yaml`

Main inputs:

- `/perception/letter_objects`
- `/wordle_bot/start_mission`
- `/wordle_bot/stop_mission`
- `/wordle_bot/add_collision_object`

Main outputs:

- `/wordle_bot/robot_state`
- `/wordle_bot/goal_reached`
- `/wordle_bot/mission_complete`
- `/wordle_bot/motion_complete`

Independent test:

```bash
ros2 launch wordlebot_control wordle_bot_mtc.launch.py
ros2 topic pub --once /wordle_bot/open_gripper std_msgs/msg/Bool "{data: true}"
```

Important parameters:

- `pick_place.backend`
- `working_joints.*`
- `scan_sweep_pose_0` to `scan_sweep_pose_5`
- `velocity_scaling.*`
- `pick_place.mgi_*`

### High-level control: `hl_control`

Purpose: Convert a word request and perceived board state into an ordered task sequence.

Main node:

- `hl_control_node`

Key files:

- `hl_control/hl_control_node.py`
- `hl_control/rl_task_optimiser.py`
- `rl_task_optimiser/task_sequencer.py`
- `rl_task_optimiser/training_env/wordle_env.py`
- `models/wordle_ppo_latest.zip`

Main inputs:

- `/hl_control/word_request`
- `/perception/gameboard_state`

Main outputs:

- `/wordle_bot/add_collision_object`
- `/perception/letter_objects`

Independent test:

```bash
ros2 launch hl_control hl_control.launch.py
python3 test/test_sim.py --ros-args \
  -p config_path:=$(pwd)/config/tc2_1_board.yaml \
  -p word:=CRANE
```

Important parameter:

- `model_path`: absolute MaskablePPO checkpoint path without `.zip`.

## Publish/toggle command reference

Run these only after the relevant nodes are up and the workspace has been sourced.

### Mission control toggles

Start queued goals or pick/place tasks:

```bash
ros2 topic pub --once /wordle_bot/start_mission std_msgs/msg/Bool "{data: true}"
```

Stop/pause active mission:

```bash
ros2 topic pub --once /wordle_bot/stop_mission std_msgs/msg/Bool "{data: true}"
```

Resume after `STOPPED`:

```bash
ros2 topic pub --once /wordle_bot/resume_mission std_msgs/msg/Bool "{data: true}"
```

Abort after `STOPPED`, clear queues, and return home:

```bash
ros2 topic pub --once /wordle_bot/abort_mission std_msgs/msg/Bool "{data: true}"
```

### Manual arm and gripper toggles

Open gripper:

```bash
ros2 topic pub --once /wordle_bot/open_gripper std_msgs/msg/Bool "{data: true}"
```

Close gripper:

```bash
ros2 topic pub --once /wordle_bot/close_gripper std_msgs/msg/Bool "{data: true}"
```

Return to configured working joints:

```bash
ros2 topic pub --once /wordle_bot/return_home std_msgs/msg/Bool "{data: true}"
```

Run scan-and-sweep:

```bash
ros2 topic pub --once /wordle_bot/scan_and_sweep std_msgs/msg/Bool "{data: true}"
```

### Direct goal mission

Queue one or more end-effector goals:

```bash
ros2 topic pub --once /wordle_bot/set_mission geometry_msgs/msg/PoseArray \
  "{header: {frame_id: 'world'}, poses: [
    {position: {x: 0.30, y: 0.15, z: 0.175},
     orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}
  ]}"
```

Then start:

```bash
ros2 topic pub --once /wordle_bot/start_mission std_msgs/msg/Bool "{data: true}"
```

### High-level word and board inputs

Publish a target word:

```bash
ros2 topic pub --once /hl_control/word_request std_msgs/msg/String "{data: 'CRANE'}"
```

Publish a simple board state:

```bash
ros2 topic pub --once /perception/gameboard_state hl_control/msg/GameboardState \
  "{letters: [
    {letter: 'C', object_id: 'C_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.15, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.247, w: 0.969}}}},
    {letter: 'R', object_id: 'R_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.30, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.565, w: 0.825}}}},
    {letter: 'A', object_id: 'A_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.075, y: 0.375, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: -0.389, w: 0.921}}}},
    {letter: 'N', object_id: 'N_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.375, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.867, w: 0.498}}}},
    {letter: 'E', object_id: 'E_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.150, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: -0.682, w: 0.732}}}}
  ]}
```

Recommended isolated publisher:

```bash
cd ~/wordlebot_ws/src/hl_control
python3 test/test_sim.py --ros-args \
  -p config_path:=$(pwd)/config/tc2_1_board.yaml \
  -p word:=CRANE
```

### Direct pick-and-place task

Use this only to bypass `hl_control` for low-level testing:

```bash
ros2 topic pub --once /perception/letter_objects wordlebot_control/msg/PickPlaceTask \
  "{pick_pose: {header: {frame_id: 'world'},
                pose: {position: {x: -0.35, y: 0.10, z: 0.025},
                       orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}},
    place_pose: {position: {x: -0.15, y: 0.225, z: 0.025},
                 orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}},
    object_id: 'C_object_1'}"
```

Then start:

```bash
ros2 topic pub --once /wordle_bot/start_mission std_msgs/msg/Bool "{data: true}"
```

### Collision object toggles

Add a box:

```bash
ros2 topic pub --once /wordle_bot/add_collision_object moveit_msgs/msg/CollisionObject \
  "{id: 'test_box',
    header: {frame_id: 'world'},
    primitives: [{type: 1, dimensions: [0.05, 0.05, 0.05]}],
    primitive_poses: [{position: {x: 0.30, y: 0.20, z: 0.025},
                       orientation: {w: 1.0}}],
    operation: 0}"
```

Remove the same box:

```bash
ros2 topic pub --once /wordle_bot/add_collision_object moveit_msgs/msg/CollisionObject \
  "{id: 'test_box', header: {frame_id: 'world'}, operation: 2}"
```

Clear tracked letter/board objects and queued pick tasks:

```bash
ros2 topic pub --once /wordle_bot/clear_letter_objects std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /wordle_bot/clear_board_objects std_msgs/msg/Bool "{data: true}"
```

### Monitor topics

Robot state:

```bash
ros2 topic echo /wordle_bot/robot_state
```

Task progress:

```bash
ros2 topic echo /wordle_bot/goal_reached
ros2 topic echo /wordle_bot/mission_complete
ros2 topic echo /wordle_bot/motion_complete
```

High-level generated tasks:

```bash
ros2 topic echo /perception/letter_objects
```

MoveIt collision object requests:

```bash
ros2 topic echo /wordle_bot/add_collision_object
```

## Testing

Build and run package tests:

```bash
cd ~/wordlebot_ws
source install/setup.bash
colcon test --packages-select wordlebot_control hl_control
colcon test-result --verbose
```

Low-level control tests:

```bash
colcon test --packages-select wordlebot_control --pytest-args -k tc1
colcon test --packages-select wordlebot_control --pytest-args -k tc2
colcon test --packages-select wordlebot_control --pytest-args -k tc3
```

HL model demonstration without ROS:

```bash
cd ~/wordlebot_ws/src/hl_control
python3 test/demonstration_test.py
```

## Known limitations and assumptions

- The full stack assumes ROS 2 Humble and the UR + OnRobot package names used above.
- The real robot IP must match your network; `192.168.0.194` is only the default example.
- The physical board frame must match the software `world` frame.
- `hl_control` produces task sequences but does not start robot execution.
- `wordlebot_control` defaults to the MGI pick/place backend for exact repeatable poses.
- Place/task generation assumes five-letter words.
- Letter collision objects are simple 50 mm boxes.
- The attached camera/sensor is approximated by a cylinder, not an exact mesh.
- If physical execution fails, `hl_control` does not automatically replan from the new board state.

## Troubleshooting and FAQs

### RViz opens but no robot appears

Check that the robot driver and MoveIt are running, and that every terminal sourced the workspace.

```bash
ros2 topic list | grep robot_description
```

### `hl_control_node` waits forever

It needs both inputs:

```bash
ros2 topic echo /hl_control/word_request
ros2 topic echo /perception/gameboard_state
```

If you publish a new word, publish a fresh board state afterwards.

### Tasks are published but the robot does not move

This is normal until you arm execution:

```bash
ros2 topic pub --once /wordle_bot/start_mission std_msgs/msg/Bool "{data: true}"
```

### Real robot driver cannot connect

Check:

- Robot IP address.
- PC and robot are on the same network.
- UR external-control program and remote-control mode are configured as required.
- Firewall/VPN is not blocking the connection.

### Motion planning fails

Check RViz for collisions, then inspect logs for:

- Missing `sensor_guard`.
- Collision preflight failure.
- IK failure.
- Target pose outside the UR3e workspace.
- Letter object placed at an unexpected `world` pose.

### Gripper does not move

Check that the OnRobot driver/controllers are running and that no mission is already active. Manual gripper toggles are ignored while a mission is running.

### Model file is missing

The default HL launch expects an installed model checkpoint. Verify:

```bash
ls ~/wordlebot_ws/install/hl_control/lib/hl_control/models/wordle_ppo_latest.zip
```

Or pass a model path without `.zip`:

```bash
ros2 launch hl_control hl_control.launch.py model_path:=/absolute/path/to/wordle_ppo_latest
```

# Full Stack Wordle Bot Execution 

```bash
colcon build --packages-select wordlebot_control hl_control interaction_execution
source ~/wordlebot_ws/install/setup.bash 

ros2 launch wordlebot_control wordlebot_bringup.launch.py

ros2 launch interaction_execution gui.launch.py

ros2 launch wordlebot_control wordlecontrolfullstack.launch.py

python3 ~/git/RS2/gamification/gamification_node.py 
```