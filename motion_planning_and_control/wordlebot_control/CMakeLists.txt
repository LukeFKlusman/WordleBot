cmake_minimum_required(VERSION 3.8)
project(wordlebot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PickPlaceTask.msg"
  DEPENDENCIES geometry_msgs
)

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_visual_tools
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  tf2
  geometry_msgs
  std_msgs
  moveit_msgs
  shape_msgs
  controller_manager_msgs
)

# --- Library: controller + node sources ---
add_library(wordle_bot_lib
  src/wordle_bot_controller.cpp
  src/wordle_bot_control_node.cpp
  src/wordle_mtc_planner.cpp
)
target_include_directories(wordle_bot_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(wordle_bot_lib PUBLIC cxx_std_17)
ament_target_dependencies(wordle_bot_lib ${DEPENDENCIES})
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(wordle_bot_lib "${cpp_typesupport_target}")

# --- Executable ---
add_executable(wordlebot_control_node main/main.cpp)
target_include_directories(wordlebot_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(wordlebot_control_node PUBLIC cxx_std_17)
target_link_libraries(wordlebot_control_node wordle_bot_lib)

# --- Install ---
install(TARGETS wordle_bot_lib
  DESTINATION lib
)
install(TARGETS wordlebot_control_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  add_executable(test_wrist3_alignment
    test/test_wrist3_alignment.cpp
  )
  target_link_libraries(test_wrist3_alignment wordle_bot_lib)
  add_test(NAME test_wrist3_alignment COMMAND test_wrist3_alignment)

  # TC1 — Key Control Concepts (validates P, C, D)
  ament_add_pytest_test(tc1_key_control_concepts
    test/tc1_key_control_concepts.py
    TIMEOUT 600
  )

  # TC2 — Advanced Motion Control (validates HD)
  ament_add_pytest_test(tc2_advanced_motion_control
    test/tc2_advanced_motion_control.py
    TIMEOUT 600
  )

  # TC3 — Advanced Collision Avoidance (validates Perfect)
  ament_add_pytest_test(tc3_advanced_collision_avoidance
    test/tc3_advanced_collision_avoidance.py
    TIMEOUT 600
  )
endif()

ament_package()
