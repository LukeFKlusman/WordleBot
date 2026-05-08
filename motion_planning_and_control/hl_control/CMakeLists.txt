cmake_minimum_required(VERSION 3.8)
project(hl_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LetterObject.msg"
  "msg/GameboardState.msg"
  DEPENDENCIES geometry_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  hl_control/hl_control_node.py
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
