#include "hector_multi_robot_announcement/simple_multi_robot_announcer.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>

#include "hector_multi_robot_announcement/utils.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE( hector_multi_robot_announcement::SimpleMultiRobotAnnouncer )

using hector_multi_robot_msgs::msg::RobotAnnouncement;
using rcl_interfaces::msg::ParameterDescriptor;

namespace hector_multi_robot_announcement
{

namespace
{
// Prepends identity remaps for /tf and /tf_static to the node's arguments so the global TF
// publisher topics out-rank the common multi-robot `/tf:=tf` remap and stay un-remapped. rcl
// applies the first matching node-local remap rule, so the identity rules must be inserted
// *before* any caller-provided remap: right after the first `--ros-args` token (a new block is
// appended only if none exists).
rclcpp::NodeOptions bypass_global_tf_remap( const rclcpp::NodeOptions &options )
{
  const std::vector<std::string> remaps = { "--remap", "/tf:=/tf", "--remap",
                                            "/tf_static:=/tf_static" };
  std::vector<std::string> arguments = options.arguments();
  const auto ros_args = std::find( arguments.begin(), arguments.end(), "--ros-args" );
  if ( ros_args == arguments.end() ) {
    arguments.push_back( "--ros-args" );
    arguments.insert( arguments.end(), remaps.begin(), remaps.end() );
  } else {
    arguments.insert( ros_args + 1, remaps.begin(), remaps.end() );
  }
  rclcpp::NodeOptions opts = options;
  opts.arguments( std::move( arguments ) );
  return opts;
}
} // namespace

SimpleMultiRobotAnnouncer::SimpleMultiRobotAnnouncer( const rclcpp::NodeOptions &options )
    : Node( "simple_announcer", bypass_global_tf_remap( options ) )
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
  // Optional (default ""): a robot may announce itself without declaring a type.
  declare_parameter<std::string>(
      "type", "",
      ParameterDescriptor().set__read_only( true ).set__description(
          "The type of robot (e.g. wheeled, tracked, legged, quadcopter, humanoid; custom allowed)." ) );

  robot_id_ = get_parameter( "robot_id" ).as_string();
  robot_name_ = get_parameter( "robot_name" ).as_string();
  robot_namespace_ = normalize_namespace( get_parameter( "robot_namespace" ).as_string() );
  robot_type_ = get_parameter( "type" ).as_string();
  configuration_ = parse_configuration( get_node_parameters_interface()->get_parameter_overrides() );

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
  announcement.type = robot_type_;
  for ( const auto &[key, value] : configuration_ ) {
    announcement.keys.push_back( key );
    announcement.values.push_back( value );
  }
  announcement_publisher_->publish( announcement );
  if ( global_announcement_publisher_ )
    global_announcement_publisher_->publish( announcement );

  tf_forwarder_ = std::make_unique<TfForwarder>( *this, robot_namespace_ );
}
} // namespace hector_multi_robot_announcement
