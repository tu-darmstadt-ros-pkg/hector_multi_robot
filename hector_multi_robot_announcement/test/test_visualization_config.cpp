#include <gtest/gtest.h>

#include <map>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/parameter_value.hpp>

#include "hector_multi_robot_announcement/visualization_config.hpp"

using hector_multi_robot_announcement::parse_visualizations;
using hector_multi_robot_msgs::msg::Visualization;

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger( "test_visualization_config" ); }

rclcpp::ParameterValue str( const std::string &value ) { return rclcpp::ParameterValue( value ); }
} // namespace

// --- parse_visualizations ----------------------------------------------------

// A fully specified entry maps every field, and nested `hints.<key>` become parallel keys/values.
TEST( ParseVisualizations, MapsAllFieldsAndHints )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["visualizations.elevation_map.name"] = str( "Elevation Map" );
  overrides["visualizations.elevation_map.topic"] = str( "elevation_map" );
  overrides["visualizations.elevation_map.message_type"] = str( "grid_map_msgs/msg/GridMap" );
  overrides["visualizations.elevation_map.kind"] = str( "elevation_map" );
  overrides["visualizations.elevation_map.group"] = str( "Mapping" );
  overrides["visualizations.elevation_map.default_visibility"] = str( "when_active" );
  overrides["visualizations.elevation_map.hints.colormap"] = str( "turbo" );

  const auto result = parse_visualizations( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  const auto &vis = result.front();
  EXPECT_EQ( vis.name, "Elevation Map" );
  EXPECT_EQ( vis.topic, "elevation_map" );
  EXPECT_EQ( vis.message_type, "grid_map_msgs/msg/GridMap" );
  EXPECT_EQ( vis.kind, "elevation_map" );
  EXPECT_EQ( vis.group, "Mapping" );
  EXPECT_EQ( vis.default_visibility, Visualization::DEFAULT_VISIBILITY_WHEN_ACTIVE );
  ASSERT_EQ( vis.keys.size(), 1u );
  ASSERT_EQ( vis.values.size(), 1u );
  EXPECT_EQ( vis.keys.front(), "colormap" );
  EXPECT_EQ( vis.values.front(), "turbo" );
}

// Optional fields default: empty strings, default_visibility hidden, no hints.
TEST( ParseVisualizations, AppliesDefaultsForOptionalFields )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["visualizations.cloud.topic"] = str( "points" );

  const auto result = parse_visualizations( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  const auto &vis = result.front();
  EXPECT_EQ( vis.topic, "points" );
  EXPECT_EQ( vis.message_type, "" );
  EXPECT_EQ( vis.kind, "" );
  EXPECT_EQ( vis.group, "" );
  EXPECT_EQ( vis.default_visibility, Visualization::DEFAULT_VISIBILITY_HIDDEN );
  EXPECT_TRUE( vis.keys.empty() );
  EXPECT_TRUE( vis.values.empty() );
}

// `name` falls back to the map key when absent or explicitly empty.
TEST( ParseVisualizations, NameFallsBackToMapKey )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["visualizations.planned_path.topic"] = str( "plan" );
  overrides["visualizations.blank_name.topic"] = str( "other" );
  overrides["visualizations.blank_name.name"] = str( "" );

  const auto result = parse_visualizations( overrides, logger() );

  ASSERT_EQ( result.size(), 2u );
  // Sorted by id: "blank_name" before "planned_path".
  EXPECT_EQ( result[0].name, "blank_name" );
  EXPECT_EQ( result[1].name, "planned_path" );
}

// An entry without a non-empty topic is skipped; others survive.
TEST( ParseVisualizations, SkipsEntryWithoutTopic )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["visualizations.no_topic.name"] = str( "No Topic" );
  overrides["visualizations.no_topic.kind"] = str( "path" );
  overrides["visualizations.empty_topic.topic"] = str( "" );
  overrides["visualizations.valid.topic"] = str( "points" );

  const auto result = parse_visualizations( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  EXPECT_EQ( result.front().name, "valid" );
}

// Two entries resolving to the same name keep only the first in sorted id order; the duplicate is
// dropped.
TEST( ParseVisualizations, DropsDuplicateNames )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["visualizations.a_map.name"] = str( "Map" );
  overrides["visualizations.a_map.topic"] = str( "map_a" );
  overrides["visualizations.b_map.name"] = str( "Map" );
  overrides["visualizations.b_map.topic"] = str( "map_b" );
  overrides["visualizations.c_other.topic"] = str( "other" );

  const auto result = parse_visualizations( overrides, logger() );

  ASSERT_EQ( result.size(), 2u );
  // "a_map" claims the name "Map"; "b_map" is dropped. "c_other" keeps its id as name.
  EXPECT_EQ( result[0].name, "Map" );
  EXPECT_EQ( result[0].topic, "map_a" );
  EXPECT_EQ( result[1].name, "c_other" );
}

// A duplicate collides on the resolved name even when it comes from the id fallback.
TEST( ParseVisualizations, DuplicateNameCollidesWithIdFallback )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["visualizations.cloud.topic"] = str( "points_a" );
  overrides["visualizations.zebra.name"] = str( "cloud" ); // explicit name equals the other's id
  overrides["visualizations.zebra.topic"] = str( "points_b" );

  const auto result = parse_visualizations( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  // Sorted by id: "cloud" (name falls back to "cloud") wins; "zebra" reusing "cloud" is dropped.
  EXPECT_EQ( result.front().name, "cloud" );
  EXPECT_EQ( result.front().topic, "points_a" );
}

// Hint values are stringified; unrelated overrides are not treated as visualizations.
TEST( ParseVisualizations, StringifiesScalarsAndIgnoresUnrelated )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["robot_id"] = str( "robot1" );
  overrides["configuration.main_track"] = str( "true" );
  overrides["visualizations.cloud.topic"] = str( "points" );
  overrides["visualizations.cloud.hints.line_width"] = rclcpp::ParameterValue( 4 ); // unquoted int

  const auto result = parse_visualizations( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  const auto &vis = result.front();
  ASSERT_EQ( vis.keys.size(), 1u );
  EXPECT_EQ( vis.keys.front(), "line_width" );
  EXPECT_EQ( vis.values.front(), "4" ); // stringified via rclcpp::to_string
}

// Multiple hints stay in sorted key order with keys/values aligned regardless of insertion order.
TEST( ParseVisualizations, MultipleHintsStayAlignedInSortedOrder )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["visualizations.cloud.topic"] = str( "points" );
  overrides["visualizations.cloud.hints.line_width"] = str( "0.05" );
  overrides["visualizations.cloud.hints.alpha"] = str( "0.6" );
  overrides["visualizations.cloud.hints.color"] = str( "#e91e63" );

  const auto result = parse_visualizations( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  const auto &vis = result.front();
  ASSERT_EQ( vis.keys.size(), 3u );
  ASSERT_EQ( vis.values.size(), 3u );
  // Sorted by key: alpha, color, line_width.
  EXPECT_EQ( vis.keys[0], "alpha" );
  EXPECT_EQ( vis.values[0], "0.6" );
  EXPECT_EQ( vis.keys[1], "color" );
  EXPECT_EQ( vis.values[1], "#e91e63" );
  EXPECT_EQ( vis.keys[2], "line_width" );
  EXPECT_EQ( vis.values[2], "0.05" );
}

// Each visibility string maps to its message constant.
TEST( ParseVisualizations, MapsDefaultVisibilityStrings )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["visualizations.a_hidden.topic"] = str( "points" );
  overrides["visualizations.a_hidden.default_visibility"] = str( "hidden" );
  overrides["visualizations.b_active.topic"] = str( "elevation_map" );
  overrides["visualizations.b_active.default_visibility"] = str( "when_active" );
  overrides["visualizations.c_always.topic"] = str( "plan" );
  overrides["visualizations.c_always.default_visibility"] = str( "always" );

  const auto result = parse_visualizations( overrides, logger() );

  ASSERT_EQ( result.size(), 3u );
  // Sorted by id: a_hidden, b_active, c_always.
  EXPECT_EQ( result[0].default_visibility, Visualization::DEFAULT_VISIBILITY_HIDDEN );
  EXPECT_EQ( result[1].default_visibility, Visualization::DEFAULT_VISIBILITY_WHEN_ACTIVE );
  EXPECT_EQ( result[2].default_visibility, Visualization::DEFAULT_VISIBILITY_ALWAYS );
}

// An unknown visibility string warns and falls back to hidden.
TEST( ParseVisualizations, UnknownDefaultVisibilityFallsBackToHidden )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["visualizations.cloud.topic"] = str( "points" );
  overrides["visualizations.cloud.default_visibility"] = str( "sometimes" );

  const auto result = parse_visualizations( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  EXPECT_EQ( result.front().default_visibility, Visualization::DEFAULT_VISIBILITY_HIDDEN );
}

// A non-string visibility value warns and falls back to hidden.
TEST( ParseVisualizations, NonStringDefaultVisibilityFallsBackToHidden )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["visualizations.cloud.topic"] = str( "points" );
  overrides["visualizations.cloud.default_visibility"] = rclcpp::ParameterValue( true );

  const auto result = parse_visualizations( overrides, logger() );

  ASSERT_EQ( result.size(), 1u );
  EXPECT_EQ( result.front().default_visibility, Visualization::DEFAULT_VISIBILITY_HIDDEN );
}
