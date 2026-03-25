cmake_minimum_required(VERSION 3.8)
project(wordleBot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  tf2
  geometry_msgs
  std_msgs
)

# --- Library: controller + node sources ---
add_library(wordle_bot_lib
  src/wordle_bot_controller.cpp
  src/wordle_bot_control_node.cpp
)
target_include_directories(wordle_bot_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(wordle_bot_lib PUBLIC cxx_std_17)
ament_target_dependencies(wordle_bot_lib ${DEPENDENCIES})

# --- Executable ---
add_executable(wordleBot_control main/main.cpp)
target_include_directories(wordleBot_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(wordleBot_control PUBLIC cxx_std_17)
target_link_libraries(wordleBot_control wordle_bot_lib)

# --- Install ---
install(TARGETS wordle_bot_lib wordleBot_control
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(goal_motion_test
    test/goal_motion_test.py
    TIMEOUT 300
  )
endif()

ament_package()
