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

# Install all Python scripts as executables.
# rl_task_optimiser.py is co-installed so hl_control_node.py can import it
# from the same directory via sys.path at runtime.
install(PROGRAMS
  hl_control/hl_control_node.py
  hl_control/rl_task_optimiser.py
  hl_control/test_env_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
