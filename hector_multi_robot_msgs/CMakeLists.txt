cmake_minimum_required(VERSION 3.8)
project(hector_multi_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Heartbeat.msg
  msg/RobotAnnouncement.msg
  msg/RobotStatus.msg
  msg/Sensor.msg
  msg/Visualization.msg
  srv/AddSensor.srv
  srv/AddVisualization.srv
  srv/RemoveSensor.srv
  srv/RemoveVisualization.srv
  srv/SetStatus.srv
  DEPENDENCIES sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
