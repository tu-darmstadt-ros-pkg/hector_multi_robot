cmake_minimum_required(VERSION 3.8)
project(hector_multi_robot_announcement)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hector_multi_robot_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)


add_library(simple_multi_robot_announcer_component SHARED src/simple_multi_robot_announcer.cpp)

rclcpp_components_register_node(simple_multi_robot_announcer_component
  PLUGIN "hector_multi_robot_announcement::SimpleMultiRobotAnnouncer"
  EXECUTABLE simple_multi_robot_announcer
)

target_include_directories(simple_multi_robot_announcer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(simple_multi_robot_announcer_component
  rclcpp
  hector_multi_robot_msgs
  rclcpp_components
)

install(TARGETS simple_multi_robot_announcer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
