#include "hector_multi_robot_announcement/simple_multi_robot_announcer.hpp"
#include <chrono>
#include <functional>

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE( hector_multi_robot_announcement::SimpleMultiRobotAnnouncer )

using hector_multi_robot_msgs::msg::RobotAnnouncement;
using rcl_interfaces::msg::ParameterDescriptor;

namespace hector_multi_robot_announcement
{

SimpleMultiRobotAnnouncer::SimpleMultiRobotAnnouncer( const rclcpp::NodeOptions &options )
    : Node( "simple_announcer", options )
{
  declare_parameter<std::string>( "robot_id",
                                  ParameterDescriptor().set__read_only( true ).set__description(
                                      "The unique ID of the robot." ) );
  declare_parameter<std::string>( "robot_name",
                                  ParameterDescriptor().set__read_only( true ).set__description(
                                      "The human-readable name of the robot." ) );
  declare_parameter<std::string>( "robot_namespace",
                                  ParameterDescriptor().set__read_only( true ).set__description(
                                      "The ROS namespace of the robot." ) );

  robot_id_ = get_parameter( "robot_id" ).as_string();
  robot_name_ = get_parameter( "robot_name" ).as_string();
  robot_namespace_ = get_parameter( "robot_namespace" ).as_string();
  if ( robot_namespace_.empty() || robot_namespace_[0] != '/' ) {
    robot_namespace_ = "/" + robot_namespace_;
  }

  setup();
}

void SimpleMultiRobotAnnouncer::setup()
{
  const rclcpp::QoS announcement_qos = rclcpp::QoS( 1 ).reliable().transient_local();
  announcement_publisher_ =
      create_publisher<RobotAnnouncement>( "robot_announcement", announcement_qos );
  if ( get_effective_namespace() != "/" ) {
    global_announcement_publisher_ =
        create_publisher<RobotAnnouncement>( "/robot_announcement", announcement_qos );
  }

  RCLCPP_INFO_STREAM( get_logger(), "Announcing robot named '"
                                        << robot_name_ << "' with id '" << robot_id_
                                        << "' in namespace: " << robot_namespace_ );
  RobotAnnouncement announcement;
  announcement.header.stamp = now();
  announcement.id = robot_id_;
  announcement.name = robot_name_;
  announcement.ros_namespace = robot_namespace_;
  announcement_publisher_->publish( announcement );
  if ( global_announcement_publisher_ )
    global_announcement_publisher_->publish( announcement );
}
} // namespace hector_multi_robot_announcement
