#include "hector_multi_robot_announcement/simple_multi_robot_announcer.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>

#include "hector_multi_robot_announcement/sensor_config.hpp"
#include "hector_multi_robot_announcement/utils.hpp"
#include "hector_multi_robot_announcement/visualization_config.hpp"

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

//! @brief Maximum number of list entries logged before truncating list.
constexpr size_t kMaxLoggedEntries = 10;
//! @brief Maximum length of a configuration value in the startup log; longer values are
//!        truncated with a trailing ellipsis so the total stays within the limit.
constexpr size_t kMaxLoggedValueLength = 40;

std::string truncate_for_log( const std::string &value )
{
  if ( value.size() <= kMaxLoggedValueLength )
    return value;
  return value.substr( 0, kMaxLoggedValueLength - 3 ) + "...";
}

//! @brief Logs a one-time startup summary of the built announcement (name/id/namespace/type,
//!        configuration keys, visualizations and sensors), truncating long lists and values. Called
//!        before the announcement is handed to the Announcer, so the formatting helpers stay in the
//!        node rather than in the Announcer.
void log_announcement_summary( const rclcpp::Logger &logger, const RobotAnnouncement &announcement )
{
  RCLCPP_INFO_STREAM( logger, "Announcing robot named '"
                                  << announcement.name << "' with id '" << announcement.id
                                  << "' in namespace: " << announcement.ros_namespace );
  if ( !announcement.type.empty() )
    RCLCPP_INFO_STREAM( logger, "Robot type: " << announcement.type );

  if ( announcement.keys.empty() ) {
    RCLCPP_INFO( logger, "No configuration keys." );
  } else {
    std::ostringstream config_log;
    config_log << "Configuration (" << announcement.keys.size() << " keys):";
    for ( size_t i = 0; i < announcement.keys.size() && i < kMaxLoggedEntries; ++i )
      config_log << "\n  " << announcement.keys[i] << ": " << truncate_for_log( announcement.values[i] );
    if ( announcement.keys.size() > kMaxLoggedEntries )
      config_log << "\n  ... and " << ( announcement.keys.size() - kMaxLoggedEntries ) << " more";
    RCLCPP_INFO_STREAM( logger, config_log.str() );
  }

  if ( announcement.visualizations.empty() ) {
    RCLCPP_INFO( logger, "No visualizations." );
  } else {
    std::ostringstream vis_log;
    vis_log << "Visualizations (" << announcement.visualizations.size() << "):";
    for ( size_t i = 0; i < announcement.visualizations.size() && i < kMaxLoggedEntries; ++i )
      vis_log << "\n  " << announcement.visualizations[i].name << " ("
              << announcement.visualizations[i].topic << ")";
    if ( announcement.visualizations.size() > kMaxLoggedEntries )
      vis_log << "\n  ... and " << ( announcement.visualizations.size() - kMaxLoggedEntries )
              << " more";
    RCLCPP_INFO_STREAM( logger, vis_log.str() );
  }

  if ( announcement.sensors.empty() ) {
    RCLCPP_INFO( logger, "No sensors." );
  } else {
    std::ostringstream sensor_log;
    sensor_log << "Sensors (" << announcement.sensors.size() << "):";
    for ( size_t i = 0; i < announcement.sensors.size() && i < kMaxLoggedEntries; ++i )
      sensor_log << "\n  " << announcement.sensors[i].name << " ("
                 << announcement.sensors[i].topic << ")";
    if ( announcement.sensors.size() > kMaxLoggedEntries )
      sensor_log << "\n  ... and " << ( announcement.sensors.size() - kMaxLoggedEntries ) << " more";
    RCLCPP_INFO_STREAM( logger, sensor_log.str() );
  }
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
  robot_namespace_ = normalize_namespace( get_parameter( "robot_namespace" ).as_string() );
  const auto &overrides = get_node_parameters_interface()->get_parameter_overrides();

  // Build the initial announcement from the parameters and overrides. Announcer sets the header
  // stamp on every publish, so it is left unset here.
  RobotAnnouncement announcement;
  announcement.id = robot_id_;
  announcement.name = get_parameter( "robot_name" ).as_string();
  announcement.ros_namespace = robot_namespace_;
  announcement.type = get_parameter( "type" ).as_string();
  for ( const auto &[key, value] : parse_configuration( overrides ) ) {
    announcement.keys.push_back( key );
    announcement.values.push_back( value );
  }
  announcement.visualizations = parse_visualizations( overrides, get_logger() );
  announcement.sensors = parse_sensors( overrides, get_logger() );

  log_announcement_summary( get_logger(), announcement );

  setup( std::move( announcement ) );
}

void SimpleMultiRobotAnnouncer::setup( RobotAnnouncement announcement )
{
  announcer_ = std::make_unique<Announcer>( *this, std::move( announcement ) );

  tf_forwarder_ = std::make_unique<TfForwarder>( *this, robot_namespace_ );
  topic_forwarder_ = std::make_unique<TopicForwarder>( *this, robot_namespace_ );
  status_reporter_ = std::make_unique<StatusReporter>( *this, robot_id_ );
}
} // namespace hector_multi_robot_announcement
