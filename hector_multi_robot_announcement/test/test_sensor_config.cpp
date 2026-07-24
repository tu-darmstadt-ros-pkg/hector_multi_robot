#include <gtest/gtest.h>

#include <map>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/parameter_value.hpp>

#include "hector_multi_robot_announcement/sensor_config.hpp"

using hector_multi_robot_announcement::parse_sensors;
using hector_multi_robot_msgs::msg::Sensor;

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger( "test_sensor_config" ); }

rclcpp::ParameterValue str( const std::string &value ) { return rclcpp::ParameterValue( value ); }
} // namespace

// --- parse_sensors -----------------------------------------------------------

// A fully specified entry maps every field, sets id from the map key, and nested `hints.<key>`
// become parallel keys/values.
TEST( ParseSensors, MapsAllFieldsAndHints )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["sensors.co2.name"] = str( "CO2" );
  overrides["sensors.co2.topic"] = str( "co2" );
  overrides["sensors.co2.message_type"] = str( "std_msgs/msg/Float64" );
  overrides["sensors.co2.field"] = str( "data" );
  overrides["sensors.co2.unit"] = str( "ppm" );
  overrides["sensors.co2.icon"] = str( "co2" );
  overrides["sensors.co2.hints.warn_above"] = str( "1000" );

  const auto result = parse_sensors( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  const auto &sensor = result.front();
  EXPECT_EQ( sensor.id, "co2" );
  EXPECT_EQ( sensor.name, "CO2" );
  EXPECT_EQ( sensor.topic, "co2" );
  EXPECT_EQ( sensor.message_type, "std_msgs/msg/Float64" );
  EXPECT_EQ( sensor.field, "data" );
  EXPECT_EQ( sensor.unit, "ppm" );
  EXPECT_EQ( sensor.icon, "co2" );
  ASSERT_EQ( sensor.keys.size(), 1u );
  ASSERT_EQ( sensor.values.size(), 1u );
  EXPECT_EQ( sensor.keys.front(), "warn_above" );
  EXPECT_EQ( sensor.values.front(), "1000" );
}

// Optional structural fields default to empty; id is still set from the map key; no hints.
TEST( ParseSensors, AppliesDefaultsForOptionalFields )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["sensors.battery.topic"] = str( "battery" );

  const auto result = parse_sensors( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  const auto &sensor = result.front();
  EXPECT_EQ( sensor.id, "battery" );
  EXPECT_EQ( sensor.topic, "battery" );
  EXPECT_EQ( sensor.message_type, "" );
  EXPECT_EQ( sensor.field, "" );
  EXPECT_EQ( sensor.unit, "" );
  EXPECT_EQ( sensor.icon, "" );
  EXPECT_TRUE( sensor.keys.empty() );
  EXPECT_TRUE( sensor.values.empty() );
}

// `name` falls back to the map key when absent or explicitly empty; `id` always equals the map key.
TEST( ParseSensors, NameFallsBackToMapKey )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["sensors.temperature.topic"] = str( "temp" );
  overrides["sensors.blank_name.topic"] = str( "other" );
  overrides["sensors.blank_name.name"] = str( "" );

  const auto result = parse_sensors( overrides, logger() );

  ASSERT_EQ( result.size(), 2u );
  // Sorted by id: "blank_name" before "temperature".
  EXPECT_EQ( result[0].id, "blank_name" );
  EXPECT_EQ( result[0].name, "blank_name" );
  EXPECT_EQ( result[1].id, "temperature" );
  EXPECT_EQ( result[1].name, "temperature" );
}

// An entry without a non-empty topic is skipped; others survive.
TEST( ParseSensors, SkipsEntryWithoutTopic )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["sensors.no_topic.name"] = str( "No Topic" );
  overrides["sensors.no_topic.unit"] = str( "ppm" );
  overrides["sensors.empty_topic.topic"] = str( "" );
  overrides["sensors.valid.topic"] = str( "co2" );

  const auto result = parse_sensors( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  EXPECT_EQ( result.front().id, "valid" );
}

// Hint values are stringified; unrelated overrides are not treated as sensors.
TEST( ParseSensors, StringifiesScalarsAndIgnoresUnrelated )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["robot_id"] = str( "robot1" );
  overrides["configuration.main_track"] = str( "true" );
  overrides["visualizations.cloud.topic"] = str( "points" );
  overrides["sensors.co2.topic"] = str( "co2" );
  overrides["sensors.co2.hints.decimals"] = rclcpp::ParameterValue( 4 ); // unquoted int

  const auto result = parse_sensors( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  const auto &sensor = result.front();
  ASSERT_EQ( sensor.keys.size(), 1u );
  EXPECT_EQ( sensor.keys.front(), "decimals" );
  EXPECT_EQ( sensor.values.front(), "4" ); // stringified via rclcpp::to_string
}

// Multiple hints stay in sorted key order with keys/values aligned regardless of insertion order.
TEST( ParseSensors, MultipleHintsStayAlignedInSortedOrder )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["sensors.co2.topic"] = str( "co2" );
  overrides["sensors.co2.hints.warn_above"] = str( "1000" );
  overrides["sensors.co2.hints.decimals"] = str( "1" );
  overrides["sensors.co2.hints.timeout"] = str( "5.0" );

  const auto result = parse_sensors( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  const auto &sensor = result.front();
  ASSERT_EQ( sensor.keys.size(), 3u );
  ASSERT_EQ( sensor.values.size(), 3u );
  // Sorted by key: decimals, timeout, warn_above.
  EXPECT_EQ( sensor.keys[0], "decimals" );
  EXPECT_EQ( sensor.values[0], "1" );
  EXPECT_EQ( sensor.keys[1], "timeout" );
  EXPECT_EQ( sensor.values[1], "5.0" );
  EXPECT_EQ( sensor.keys[2], "warn_above" );
  EXPECT_EQ( sensor.values[2], "1000" );
}
