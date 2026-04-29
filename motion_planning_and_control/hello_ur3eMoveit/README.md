Terminal

bash
```
cd ~/ws_moveit2
colcon build --mixin debug
source install/setup.bash
```

bash s
```
ros2 run ur_client_library start_ursim.sh -m ur3e
```

bash 
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true
```

bash 
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true
```

bash 
```
ros2 run hello_ur3eMoveit hello_ur3eMoveit
```

