#include "hector_multi_robot_announcement/visualization_config.hpp"

#include <map>
#include <string>

#include <rclcpp/logging.hpp>

namespace hector_multi_robot_announcement
{

using hector_multi_robot_msgs::msg::Visualization;

namespace
{
//! @brief Assigns `value` to the field of `vis` named by `field`, appends a `hints.<key>` entry to
//!        its parallel keys/values, or ignores an unrecognized field. `id` and `logger` are used to
//!        warn on a `default_visibility` that is not one of the known string values.
void apply_field( Visualization &vis, const std::string &id, const std::string &field,
                  const rclcpp::ParameterValue &value, const rclcpp::Logger &logger )
{
  if ( field == "name" ) {
    vis.name = rclcpp::to_string( value );
  } else if ( field == "topic" ) {
    vis.topic = rclcpp::to_string( value );
  } else if ( field == "message_type" ) {
    vis.message_type = rclcpp::to_string( value );
  } else if ( field == "kind" ) {
    vis.kind = rclcpp::to_string( value );
  } else if ( field == "group" ) {
    vis.group = rclcpp::to_string( value );
  } else if ( field == "default_visibility" ) {
    if ( value.get_type() == rclcpp::ParameterType::PARAMETER_STRING ) {
      const std::string &visibility = value.get<std::string>();
      if ( visibility == "hidden" ) {
        vis.default_visibility = Visualization::DEFAULT_VISIBILITY_HIDDEN;
      } else if ( visibility == "when_active" ) {
        vis.default_visibility = Visualization::DEFAULT_VISIBILITY_WHEN_ACTIVE;
      } else if ( visibility == "always" ) {
        vis.default_visibility = Visualization::DEFAULT_VISIBILITY_ALWAYS;
      } else {
        RCLCPP_WARN( logger,
                     "Ignoring unknown 'default_visibility' value '%s' for visualization '%s'; "
                     "expected 'hidden', 'when_active' or 'always'. Defaulting to hidden.",
                     visibility.c_str(), id.c_str() );
      }
    } else {
      RCLCPP_WARN( logger,
                   "Ignoring non-string 'default_visibility' for visualization '%s'; defaulting to "
                   "hidden.",
                   id.c_str() );
    }
  } else if ( field.rfind( kVisualizationHintsField, 0 ) == 0 ) {
    const std::string key = field.substr( std::char_traits<char>::length( kVisualizationHintsField ) );
    if ( !key.empty() ) {
      vis.keys.push_back( key );
      vis.values.push_back( rclcpp::to_string( value ) );
    }
  }
  // Unrecognized fields are ignored so consumers may add their own without breaking parsing.
}
} // namespace

std::vector<Visualization>
parse_visualizations( const std::map<std::string, rclcpp::ParameterValue> &overrides,
                      const rclcpp::Logger &logger )
{
  const std::size_t prefix_length = std::char_traits<char>::length( kVisualizationPrefix );

  // Accumulate by id. The overrides map is sorted, so all keys of one id are contiguous, ids come
  // out sorted, and hint keys accumulate in sorted order, keeping keys/values aligned.
  std::map<std::string, Visualization> by_id;
  for ( const auto &[name, value] : overrides ) {
    if ( name.rfind( kVisualizationPrefix, 0 ) != 0 )
      continue;
    const std::string rest = name.substr( prefix_length );
    const std::size_t dot = rest.find( '.' );
    if ( dot == std::string::npos )
      continue; // "visualizations.<id>" scalar has no field; nothing to populate.
    const std::string id = rest.substr( 0, dot );
    const std::string field = rest.substr( dot + 1 );
    if ( id.empty() || field.empty() )
      continue;
    apply_field( by_id[id], id, field, value, logger );
  }

  std::vector<Visualization> result;
  for ( auto &[id, vis] : by_id ) {
    if ( vis.name.empty() )
      vis.name = id; // Fall back to the map key as the human-readable name.
    if ( vis.topic.empty() ) {
      RCLCPP_WARN( logger, "Skipping visualization '%s': no 'topic' configured.", id.c_str() );
      continue;
    }
    result.push_back( std::move( vis ) );
  }
  return result;
}

} // namespace hector_multi_robot_announcement
