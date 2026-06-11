#ifndef HECTOR_MULTI_ROBOT_ANNOUNCEMENT_UTILS_HPP
#define HECTOR_MULTI_ROBOT_ANNOUNCEMENT_UTILS_HPP

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/parameter_value.hpp>

namespace hector_multi_robot_announcement
{

//! @brief Normalizes a ROS namespace to its canonical absolute form: a single leading slash,
//!        no repeated slashes, and no trailing slash (root stays "/"). Empty input maps to "/".
//!
//! A trailing or repeated slash would otherwise survive into a composed topic name (e.g.
//! "<ns>/tf" becoming "/robot1//tf"), which rmw rejects, throwing out of node construction.
inline std::string normalize_namespace( const std::string &ros_namespace )
{
  std::string result = "/";
  for ( const char c : ros_namespace ) {
    if ( c == '/' && result.back() == '/' )
      continue; // collapse leading and repeated slashes into the single leading "/"
    result.push_back( c );
  }
  if ( result.size() > 1 && result.back() == '/' )
    result.pop_back(); // drop trailing slash (root stays "/")
  return result;
}

//! @brief Extracts string key/value pairs from parameter overrides of the form
//!        "configuration.<key>".
//!
//! `configuration` is an open-ended YAML map whose keys are unknown at compile time, so (like the
//! per-frame tf rates) it is read straight from the overrides rather than declared as parameters.
//! Values are stringified via rclcpp::to_string so unquoted scalars (e.g. `main_track: true`) are
//! accepted; a nested key keeps its dotted suffix (`configuration.a.b` -> `a.b`). Pairs are returned
//! in the overrides map's sorted key order, keeping the announcement's parallel keys/values aligned.
inline std::vector<std::pair<std::string, std::string>>
parse_configuration( const std::map<std::string, rclcpp::ParameterValue> &overrides )
{
  constexpr const char *prefix = "configuration.";
  const std::size_t prefix_length = std::char_traits<char>::length( prefix );
  std::vector<std::pair<std::string, std::string>> result;
  for ( const auto &[name, value] : overrides ) {
    if ( name.rfind( prefix, 0 ) != 0 )
      continue;
    result.emplace_back( name.substr( prefix_length ), rclcpp::to_string( value ) );
  }
  return result;
}

} // namespace hector_multi_robot_announcement

#endif // HECTOR_MULTI_ROBOT_ANNOUNCEMENT_UTILS_HPP
