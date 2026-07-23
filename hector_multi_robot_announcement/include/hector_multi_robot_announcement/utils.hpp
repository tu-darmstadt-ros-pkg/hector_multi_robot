#ifndef HECTOR_MULTI_ROBOT_ANNOUNCEMENT_UTILS_HPP
#define HECTOR_MULTI_ROBOT_ANNOUNCEMENT_UTILS_HPP

#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>

namespace hector_multi_robot_announcement
{

//! @brief QoS for latched state topics: keep-last depth 1, reliable, transient_local, so a
//!        late-joining subscriber immediately receives the most recent sample.
inline rclcpp::QoS latched_qos() { return rclcpp::QoS( 1 ).reliable().transient_local(); }

//! @brief Reads a numeric parameter value as a double, accepting both integer- and
//!        double-typed values. Returns std::nullopt for any other type.
//!
//! Parameter overrides loaded from YAML are typed by their literal form, so `rate: 5`
//! becomes an integer and `rate: 5.0` a double. Treating both as a rate avoids crashing
//! on a perfectly reasonable config.
inline std::optional<double> numeric_value_as_double( const rclcpp::ParameterValue &value )
{
  switch ( value.get_type() ) {
  case rclcpp::ParameterType::PARAMETER_DOUBLE:
    return value.get<double>();
  case rclcpp::ParameterType::PARAMETER_INTEGER:
    return static_cast<double>( value.get<int64_t>() );
  default:
    return std::nullopt;
  }
}

//! @brief Declares parameter `name` with `descriptor`, preserving an override's literal type so an
//!        integer YAML literal (e.g. `30`) does not fail a double declaration; falls back to
//!        `default_value` (a double) when there is no override. Resolve the declared value through
//!        numeric_value_as_double, which accepts both integer- and double-typed parameters.
inline void declare_param_preserving_override_type(
    rclcpp::Node &node, const std::string &name, double default_value,
    const rcl_interfaces::msg::ParameterDescriptor &descriptor )
{
  const auto &overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  const auto override_it = overrides.find( name );
  if ( override_it != overrides.end() )
    node.declare_parameter( name, override_it->second, descriptor );
  else
    node.declare_parameter<double>( name, default_value, descriptor );
}

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

//! @brief Prepends `prefix` to `frame` unless `frame` is empty or listed in `global_frames`.
//!
//! Shared by tf forwarding (prefix_transform) and topic forwarding (prefix_frame_ids); it is a
//! pure string helper with no ROS dependencies, so it lives here next to normalize_namespace.
inline std::string prefix_frame_id( const std::string &frame, const std::string &prefix,
                                    const std::unordered_set<std::string> &global_frames )
{
  if ( frame.empty() || global_frames.count( frame ) > 0 )
    return frame;
  return prefix + frame;
}

//! @brief Iterates parameter overrides of the form "<prefix><id>.<field>", invoking
//!        `apply(id, field, value)` for each match.
//!
//! Shared scaffold for the open-ended `<id>`-keyed override maps (visualizations, forwarded
//! topics). Keys outside `prefix`, the bare "<prefix><id>" scalar (no field), and empty id/field
//! segments are skipped. The overrides map is sorted, so a given id's fields arrive contiguously
//! and ids are visited in sorted order.
template<typename Fn>
void for_each_override_id_field( const std::map<std::string, rclcpp::ParameterValue> &overrides,
                                 const char *prefix, Fn &&apply )
{
  const std::size_t prefix_length = std::char_traits<char>::length( prefix );
  for ( const auto &[name, value] : overrides ) {
    if ( name.rfind( prefix, 0 ) != 0 )
      continue;
    const std::string rest = name.substr( prefix_length );
    const std::size_t dot = rest.find( '.' );
    if ( dot == std::string::npos )
      continue; // "<prefix><id>" scalar has no field; nothing to populate.
    const std::string id = rest.substr( 0, dot );
    const std::string field = rest.substr( dot + 1 );
    if ( id.empty() || field.empty() )
      continue;
    apply( id, field, value );
  }
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
