#ifndef HECTOR_MULTI_ROBOT_ANNOUNCEMENT_VISUALIZATION_CONFIG_HPP
#define HECTOR_MULTI_ROBOT_ANNOUNCEMENT_VISUALIZATION_CONFIG_HPP

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <hector_multi_robot_msgs/msg/visualization.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/parameter_value.hpp>

namespace hector_multi_robot_announcement
{

//! @brief Parameter name prefix that identifies a visualization configuration block.
inline constexpr const char *kVisualizationPrefix = "visualizations.";

//! @brief Builds the announced Visualization list from parameter overrides of the form
//!        "visualizations.<id>.<field>".
//!
//! Like `configuration` and the per-frame tf rates, the visualization block is an open-ended YAML
//! map whose keys are unknown at compile time, so it is read straight from the overrides rather
//! than declared as parameters. Each `<id>` becomes one Visualization; recognized fields are
//! `name`, `topic`, `message_type`, `kind`, `group`, `default_visibility` and the nested
//! `hints.<key>` map (mapped to parallel `keys`/`values`). `name` defaults to `<id>` when absent or
//! empty and must be unique across entries: a later entry reusing an earlier one's name is skipped
//! with a warning (the first id in sorted order keeps the name). An entry without a non-empty
//! `topic` is skipped with a warning. Scalar values are
//! stringified via rclcpp::to_string so unquoted YAML scalars are accepted; `default_visibility`
//! reads one of the strings `hidden`, `when_active` or `always` (mapped to the message's
//! DEFAULT_VISIBILITY_* constants) and warns (defaulting to hidden) on any other value or type.
//! Entries and their hints are returned in the overrides map's sorted key order.
std::vector<hector_multi_robot_msgs::msg::Visualization>
parse_visualizations( const std::map<std::string, rclcpp::ParameterValue> &overrides,
                      const rclcpp::Logger &logger );

//! @brief Validates a Visualization in place for the runtime `add_visualization` service. Requires a
//!        non-empty `name` and `topic`. Returns a human-readable reason phrase when invalid, or
//!        std::nullopt when valid.
//!
//! Unlike parse_visualizations this is not used by the config path: there is no map key to fall back
//! to for an empty `name`, and the service's upsert-by-name keeps names unique without the parse
//! path's cross-entry dedupe.
std::optional<std::string>
normalize_visualization( hector_multi_robot_msgs::msg::Visualization &visualization );

} // namespace hector_multi_robot_announcement

#endif // HECTOR_MULTI_ROBOT_ANNOUNCEMENT_VISUALIZATION_CONFIG_HPP
