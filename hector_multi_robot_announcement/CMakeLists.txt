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
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hector_multi_robot_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


add_library(simple_multi_robot_announcer_component SHARED
  src/simple_multi_robot_announcer.cpp
  src/status_reporter.cpp
  src/tf_forwarding.cpp
  src/visualization_config.cpp
)

rclcpp_components_register_node(simple_multi_robot_announcer_component
  PLUGIN "hector_multi_robot_announcement::SimpleMultiRobotAnnouncer"
  EXECUTABLE simple_multi_robot_announcer
)

target_include_directories(simple_multi_robot_announcer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(simple_multi_robot_announcer_component
  geometry_msgs
  rclcpp
  hector_multi_robot_msgs
  rclcpp_components
  sensor_msgs
  tf2_msgs
  tf2_ros
)

install(TARGETS simple_multi_robot_announcer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_tf_forwarding test/test_tf_forwarding.cpp)
  target_link_libraries(test_tf_forwarding simple_multi_robot_announcer_component)
  ament_target_dependencies(test_tf_forwarding geometry_msgs rclcpp tf2_msgs)

  ament_add_gtest(test_utils test/test_utils.cpp)
  target_link_libraries(test_utils simple_multi_robot_announcer_component)
  ament_target_dependencies(test_utils rclcpp)

  ament_add_gtest(test_status_reporter test/test_status_reporter.cpp)
  target_link_libraries(test_status_reporter simple_multi_robot_announcer_component)
  ament_target_dependencies(test_status_reporter rclcpp sensor_msgs hector_multi_robot_msgs)

  ament_add_gtest(test_visualization_config test/test_visualization_config.cpp)
  target_link_libraries(test_visualization_config simple_multi_robot_announcer_component)
  ament_target_dependencies(test_visualization_config rclcpp hector_multi_robot_msgs)
endif()

ament_package()
