#ifndef HECTOR_MULTI_ROBOT_ANNOUNCEMENT_SENSOR_CONFIG_HPP
#define HECTOR_MULTI_ROBOT_ANNOUNCEMENT_SENSOR_CONFIG_HPP

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <hector_multi_robot_msgs/msg/sensor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/parameter_value.hpp>

namespace hector_multi_robot_announcement
{

//! @brief Parameter name prefix that identifies a sensor configuration block.
inline constexpr const char *kSensorPrefix = "sensors.";

//! @brief Builds the announced Sensor list from parameter overrides of the form
//!        "sensors.<id>.<field>".
//!
//! Like `configuration` and `visualizations`, the sensor block is an open-ended YAML map whose keys
//! are unknown at compile time, so it is read straight from the overrides rather than declared as
//! parameters. Each `<id>` becomes one Sensor; its `id` is set to `<id>` verbatim. Recognized fields
//! are `name`, `topic`, `message_type`, `field`, `unit`, `icon` and the nested `hints.<key>` map
//! (mapped to parallel `keys`/`values`). `name` defaults to `<id>` when absent or empty. An entry
//! without a non-empty `topic` is skipped with a warning. Scalar values are stringified via
//! rclcpp::to_string so unquoted YAML scalars are accepted. Entries and their hints are returned in
//! the overrides map's sorted key order.
std::vector<hector_multi_robot_msgs::msg::Sensor>
parse_sensors( const std::map<std::string, rclcpp::ParameterValue> &overrides,
               const rclcpp::Logger &logger );

//! @brief Validates and normalizes a Sensor in place, shared by the config parse path and the runtime
//!        `add_sensor` service. Requires a non-empty `id` and `topic`, and defaults an empty `name`
//!        to the `id`. Returns a human-readable reason phrase when invalid (leaving the message
//!        partially normalized), or std::nullopt when valid.
std::optional<std::string> normalize_sensor( hector_multi_robot_msgs::msg::Sensor &sensor );

} // namespace hector_multi_robot_announcement

#endif // HECTOR_MULTI_ROBOT_ANNOUNCEMENT_SENSOR_CONFIG_HPP
