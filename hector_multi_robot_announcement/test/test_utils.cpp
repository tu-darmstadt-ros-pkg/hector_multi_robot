#include <gtest/gtest.h>

#include <map>
#include <string>

#include <rclcpp/parameter_value.hpp>

#include "hector_multi_robot_announcement/utils.hpp"

using hector_multi_robot_announcement::normalize_namespace;
using hector_multi_robot_announcement::parse_configuration;

// --- normalize_namespace -----------------------------------------------------

// Regression: a trailing or repeated slash in robot_namespace used to flow straight into the
// subscription topic ("/robot1//tf"), which rmw rejects, throwing out of node construction.
// normalize_namespace must canonicalize to a single leading slash, no repeats, no trailing slash.
TEST( NormalizeNamespace, Canonicalizes )
{
  EXPECT_EQ( normalize_namespace( "/robot1" ), "/robot1" );             // already canonical
  EXPECT_EQ( normalize_namespace( "robot1" ), "/robot1" );              // missing leading slash
  EXPECT_EQ( normalize_namespace( "/robot1/" ), "/robot1" );            // trailing slash stripped
  EXPECT_EQ( normalize_namespace( "/team//robot1/" ), "/team/robot1" ); // repeats collapsed
  EXPECT_EQ( normalize_namespace( "" ), "/" );                          // empty -> root
  EXPECT_EQ( normalize_namespace( "/" ), "/" );                         // root stays root
  EXPECT_EQ( normalize_namespace( "//" ), "/" );
}

// --- parse_configuration -----------------------------------------------------

// Only "configuration.<key>" overrides become pairs, with the prefix stripped; unrelated params
// (robot_id, tf_config.*) are ignored. The overrides map is sorted, so pairs come out sorted too,
// keeping the announcement's parallel keys/values arrays aligned.
TEST( ParseConfiguration, ExtractsPrefixedKeysInSortedOrder )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["robot_id"] = rclcpp::ParameterValue( std::string( "robot1" ) );
  overrides["tf_config.max_rate"] = rclcpp::ParameterValue( 30.0 );
  overrides["configuration.main_track"] = rclcpp::ParameterValue( std::string( "true" ) );
  overrides["configuration.flipper_links"] =
      rclcpp::ParameterValue( std::string( "[flipper_fr_link]" ) );

  const auto config = parse_configuration( overrides );

  ASSERT_EQ( config.size(), 2u );
  EXPECT_EQ( config[0].first, "flipper_links" ); // sorted before "main_track"
  EXPECT_EQ( config[0].second, "[flipper_fr_link]" );
  EXPECT_EQ( config[1].first, "main_track" );
  EXPECT_EQ( config[1].second, "true" );
}

// A nested map (configuration.a.b) keeps its dotted suffix as the key.
TEST( ParseConfiguration, KeepsNestedDottedKey )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["configuration.sensors.lidar"] = rclcpp::ParameterValue( std::string( "velodyne" ) );

  const auto config = parse_configuration( overrides );

  ASSERT_EQ( config.size(), 1u );
  EXPECT_EQ( config[0].first, "sensors.lidar" );
  EXPECT_EQ( config[0].second, "velodyne" );
}

// Unquoted scalars (bool/int) are accepted and stringified rather than dropped.
TEST( ParseConfiguration, StringifiesNonStringScalars )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["configuration.has_arm"] = rclcpp::ParameterValue( true );
  overrides["configuration.wheel_count"] = rclcpp::ParameterValue( 4 );

  const auto config = parse_configuration( overrides );

  ASSERT_EQ( config.size(), 2u );
  EXPECT_EQ( config[0].first, "has_arm" );
  EXPECT_EQ( config[0].second, "true" );
  EXPECT_EQ( config[1].first, "wheel_count" );
  EXPECT_EQ( config[1].second, "4" );
}

// No "configuration.*" override -> empty result (the field stays unset on the announcement).
TEST( ParseConfiguration, EmptyWhenNoConfigurationOverride )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["robot_id"] = rclcpp::ParameterValue( std::string( "robot1" ) );
  overrides["configuration"] = rclcpp::ParameterValue( std::string( "ignored" ) ); // no trailing dot

  EXPECT_TRUE( parse_configuration( overrides ).empty() );
}
