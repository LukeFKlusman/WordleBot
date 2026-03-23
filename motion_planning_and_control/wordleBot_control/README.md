# wordleBot_control

Main control package for the WordleBot UR3e robotic arm.

## Prerequisites

- ROS 2 (Humble or later)
- MoveIt 2
- UR ROS 2 driver and MoveIt config (`ur_moveit_config`)

---

## How to Run

### 1. Build the package

From the workspace root:

```bash
colcon build --packages-select wordleBot_control
source install/setup.bash
```

### 2. Start URSim (UR3e)

In a new terminal:

```bash
ros2 run ur_client_library start_ursim.sh -m ur3e
```

### 3. Launch Robot driver

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true
# Or if using real Hardware
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:={ENTER IP} launch_rviz:=true
```

### 4. Launch MoveIt + RViz (Terminal 1)

This starts the UR3e MoveIt stack and RViz. Keep this running throughout your session.

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true
# Or if using real Hardware
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e robot_ip:={ENTER IP} launch_rviz:=true
```

### 5. Launch the control node (Terminal 2)

```bash
ros2 launch wordleBot_control wordle_bot.launch.py
```

The node will:
1. Add a collision box to the planning scene
2. Prompt you to press **Next** in the `RvizVisualToolsGui` panel to plan
3. Visualise the planned trajectory in RViz
4. Prompt you to press **Next** again to execute
5. Move the arm to the target pose

> Make sure the **RvizVisualToolsGui** panel is open in RViz, otherwise the prompts will block indefinitely.

---

## Package Structure

```
wordleBot_control/
├── include/wordleBot_control/
│   ├── wordle_bot_controller.hpp   # Motion planning wrapper (MoveIt, collision, tf2 quaternions)
│   └── wordle_bot_control_node.hpp # ROS 2 node class
├── launch/
│   └── wordle_bot.launch.py        # Node launch (MoveIt runs separately)
├── main/
│   └── main.cpp                    # Entry point — executor + spin thread
├── src/
│   ├── wordle_bot_controller.cpp   # MoveGroupInterface, collision scene, buildPose()
│   └── wordle_bot_control_node.cpp # Goal sequence, future perception hook
├── CMakeLists.txt
└── package.xml
```

---

## Development Notes

- Goals are set in `WordleBotControlNode::run()` inside [src/wordle_bot_control_node.cpp](src/wordle_bot_control_node.cpp)
- To add more goals, call `controller_->moveToTarget(pose)` with a pose built via `WordleBotController::buildPose(x, y, z, roll, pitch, yaw)`
- The perception subscription stub is in [include/wordleBot_control/wordle_bot_control_node.hpp](include/wordleBot_control/wordle_bot_control_node.hpp) — this is where future perception integration will be wired in
