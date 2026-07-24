#include "hector_multi_robot_announcement/sensor_config.hpp"

#include <map>
#include <optional>
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
  } else {
    // A "hints.<key>" field is collected as a key/value hint; any other field is ignored so
    // consumers may add their own without breaking parsing.
    try_apply_hint( field, value, sensor.keys, sensor.values );
  }
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
    if ( const std::optional<std::string> reason = normalize_sensor( sensor ) ) {
      RCLCPP_WARN( logger, "Skipping sensor '%s': %s", id.c_str(), reason->c_str() );
      continue;
    }
    result.push_back( std::move( sensor ) );
  }
  return result;
}

std::optional<std::string> normalize_sensor( Sensor &sensor )
{
  if ( sensor.id.empty() )
    return "empty id.";
  if ( sensor.name.empty() )
    sensor.name = sensor.id; // Fall back to the id as the human-readable name.
  if ( sensor.topic.empty() )
    return "no 'topic' configured.";
  return std::nullopt;
}

} // namespace hector_multi_robot_announcement
