cmake_minimum_required(VERSION 3.8)
project(hl_control)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LetterObject.msg"
  "msg/GameboardState.msg"
  DEPENDENCIES geometry_msgs
)

# Install the two ROS2 node scripts as executables.
install(PROGRAMS
  hl_control/hl_control_node.py
  hl_control/rl_task_optimiser.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the RL engine source so it is importable at runtime via sys.path.
install(DIRECTORY rl_task_optimiser/
  DESTINATION lib/${PROJECT_NAME}/rl_task_optimiser
)

# Install the trained model checkpoint.
install(DIRECTORY models/
  DESTINATION lib/${PROJECT_NAME}/models
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
