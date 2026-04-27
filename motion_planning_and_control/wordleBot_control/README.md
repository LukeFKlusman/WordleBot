# wordleBot_control

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
colcon build --packages-select wordleBot_control
source install/setup.bash
```

### Terminal 2. Launch Robot driver

```bash
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 use_fake_hardware:=true launch_rviz:=false
# or use real hardware
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=192.168.0.197 launch_rviz:=false 
# using the id of the ur3e
```

### Terminal 3. Launch MoveIt 

This starts the UR3e MoveIt stack. Keep this running throughout your session.

```bash
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2 launch_rviz:=false
# or use real hardware
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2 launch_rviz:=false robot_ip:=192.168.0.197
# using the id of the ur3e
```

### Terminal 4. Launch Rviz

This starts Rviz

```bash
ros2 launch ur_onrobot_hello_moveit tutorials_rviz.launch.py
```


### Terminal 5. Launch the control node (Terminal 2)

Only ran after the previous terminals are running

```bash
ros2 launch wordleBot_control wordle_bot_mtc.launch.py
```

### Other Terminal.

Only ran after the previous terminals are running

```bash

ros2 launch ur_onrobot_hello_moveit tutorials_rviz.launch.py

ros2 launch wordleBot_control wordle_bot.launch.py
```

## How to Test (without intergration)

```bash
# All Tests 
python3 -m pytest src/wordleBot_control/test/tc1_key_control_concepts.py -s -v

# Run TC1.1 in isolation: Move to 1 Goal
python3 -m pytest src/wordleBot_control/test/tc1_key_control_concepts.py -k tc1_1 -s -v

# Run TC1.2 in isolation: Movge to 3 Goal
python3 -m pytest src/wordleBot_control/test/tc1_key_control_concepts.py -k tc1_2 -s -v

# Run TC1.5 in isolation: Collision Avoidance
python3 -m pytest src/wordleBot_control/test/tc1_key_control_concepts.py -k tc1_5 -s -v

# Run TC1.6 in isolation: Pick and Place 
python3 -m pytest src/wordleBot_control/test/tc1_key_control_concepts.py -k tc1_6 -s -v
```