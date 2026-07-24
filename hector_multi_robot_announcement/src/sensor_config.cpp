#include "hector_multi_robot_announcement/sensor_config.hpp"

#include <map>
#include <string>

#include <rclcpp/logging.hpp>

#include "hector_multi_robot_announcement/utils.hpp"

namespace hector_multi_robot_announcement
{

using hector_multi_robot_msgs::msg::Sensor;

namespace
{
//! @brief Assigns `value` to the field of `sensor` named by `field`, appends a `hints.<key>` entry
//!        to its parallel keys/values, or ignores an unrecognized field.
void apply_field( Sensor &sensor, const std::string &field, const rclcpp::ParameterValue &value )
{
  if ( field == "name" ) {
    sensor.name = rclcpp::to_string( value );
  } else if ( field == "topic" ) {
    sensor.topic = rclcpp::to_string( value );
  } else if ( field == "message_type" ) {
    sensor.message_type = rclcpp::to_string( value );
  } else if ( field == "field" ) {
    sensor.field = rclcpp::to_string( value );
  } else if ( field == "unit" ) {
    sensor.unit = rclcpp::to_string( value );
  } else if ( field == "icon" ) {
    sensor.icon = rclcpp::to_string( value );
  } else if ( field.rfind( kSensorHintsField, 0 ) == 0 ) {
    const std::string key = field.substr( std::char_traits<char>::length( kSensorHintsField ) );
    if ( !key.empty() ) {
      sensor.keys.push_back( key );
      sensor.values.push_back( rclcpp::to_string( value ) );
    }
  }
  // Unrecognized fields are ignored so consumers may add their own without breaking parsing.
}
} // namespace

std::vector<Sensor> parse_sensors( const std::map<std::string, rclcpp::ParameterValue> &overrides,
                                   const rclcpp::Logger &logger )
{
  // Accumulate by id. The overrides map is sorted, so all keys of one id are contiguous, ids come
  // out sorted, and hint keys accumulate in sorted order, keeping keys/values aligned.
  std::map<std::string, Sensor> by_id;
  for_each_override_id_field(
      overrides, kSensorPrefix,
      [&]( const std::string &id, const std::string &field, const rclcpp::ParameterValue &value ) {
        apply_field( by_id[id], field, value );
      } );

  std::vector<Sensor> result;
  for ( auto &[id, sensor] : by_id ) {
    sensor.id = id; // The config map key is the stable sensor identifier.
    if ( sensor.name.empty() )
      sensor.name = id; // Fall back to the map key as the human-readable name.
    if ( sensor.topic.empty() ) {
      RCLCPP_WARN( logger, "Skipping sensor '%s': no 'topic' configured.", id.c_str() );
      continue;
    }
    result.push_back( std::move( sensor ) );
  }
  return result;
}

} // namespace hector_multi_robot_announcement
