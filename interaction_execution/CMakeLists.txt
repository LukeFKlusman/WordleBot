cmake_minimum_required(VERSION 3.8)
project(interaction_execution)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt5WebEngineWidgets QUIET)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(interaction_execution_node
  include/interaction_execution/main_window.hpp
  include/interaction_execution/camera_view.hpp
  include/interaction_execution/rviz_sim_view.hpp
  include/interaction_execution/rviz_moveit_view.hpp
  include/interaction_execution/wordle_view.hpp
  src/main.cpp
  src/main_window.cpp
  src/camera_view.cpp
  src/rviz_sim_view.cpp
  src/rviz_moveit_view.cpp
  src/wordle_view.cpp
  src/rs2_concept.ui
)
target_include_directories(interaction_execution_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(interaction_execution_node
  ament_index_cpp
  cv_bridge
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  sensor_msgs
  std_msgs
)

target_link_libraries(interaction_execution_node
  Qt5::Widgets
  ${OpenCV_LIBRARIES}
)

if(Qt5WebEngineWidgets_FOUND)
  target_compile_definitions(interaction_execution_node PRIVATE HAS_QT_WEBENGINE)
  target_link_libraries(interaction_execution_node Qt5::WebEngineWidgets)
endif()

add_executable(mission_coordinator_node
  include/interaction_execution/mission_coordinator.hpp
  src/mission_coordinator.cpp
  src/mission_coordinator_main.cpp
)
target_include_directories(mission_coordinator_node PRIVATE include)

ament_target_dependencies(mission_coordinator_node
  geometry_msgs
  rclcpp
  std_msgs
)

install(TARGETS interaction_execution_node mission_coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY src/wordle-clone
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../voice_control/gui_bridge.py
  DESTINATION share/${PROJECT_NAME}/voice_control
)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../gamification/dictionary.txt
  DESTINATION share/${PROJECT_NAME}/gamification
)

ament_package()
