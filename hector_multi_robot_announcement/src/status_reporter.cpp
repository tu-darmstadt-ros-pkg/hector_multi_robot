#include "hector_multi_robot_announcement/status_reporter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <sstream>
#include <string_view>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "hector_multi_robot_announcement/utils.hpp"

using hector_multi_robot_msgs::msg::Heartbeat;
using hector_multi_robot_msgs::msg::RobotStatus;
using hector_multi_robot_msgs::srv::SetStatus;
using rcl_interfaces::msg::ParameterDescriptor;

namespace hector_multi_robot_announcement
{

namespace
{
//! @brief If `name` is a "status.battery.state_topics.<label>" key, returns the (non-empty) <label>;
//!        otherwise std::nullopt. Single source of the key pattern, shared by parse_battery_topics()
//!        and StatusReporter::load_config() so the two cannot drift.
std::optional<std::string> label_from_battery_key( const std::string &name )
{
  constexpr std::string_view prefix = kBatteryStatePrefix;
  if ( name.size() <= prefix.size() )
    return std::nullopt;
  if ( name.compare( 0, prefix.size(), prefix ) != 0 )
    return std::nullopt;
  return name.substr( prefix.size() );
}

//! @brief Whether `p` is a usable battery percentage: in [0, 1] and not NaN.
bool is_valid_percentage( float p ) { return !std::isnan( p ) && p >= 0.0f && p <= 1.0f; }

//! @brief Capacity in Ah to weight a pack by: its measured last-full `capacity` if known, else the
//!        `design_capacity` as a fallback. nullopt when neither is finite and > 0.
std::optional<double> effective_capacity( const sensor_msgs::msg::BatteryState &b )
{
  if ( std::isfinite( b.capacity ) && b.capacity > 0.0f )
    return b.capacity;
  if ( std::isfinite( b.design_capacity ) && b.design_capacity > 0.0f )
    return b.design_capacity;
  return std::nullopt;
}

//! @brief Remaining charge in Ah: the measured `charge` if known, else derived from
//!        percentage * capacity. nullopt when neither is available.
std::optional<double> remaining_charge( const sensor_msgs::msg::BatteryState &b, double capacity )
{
  if ( std::isfinite( b.charge ) && b.charge > 0.0f )
    return b.charge;
  if ( is_valid_percentage( b.percentage ) )
    return static_cast<double>( b.percentage ) * capacity;
  return std::nullopt;
}

//! @brief A pack's remaining fraction in [0, 1] from the best data it exposes: charge over effective
//!        capacity when a capacity is known, else the raw percentage. nullopt when the pack exposes
//!        nothing usable.
std::optional<double> battery_fraction( const sensor_msgs::msg::BatteryState &b )
{
  if ( const auto capacity = effective_capacity( b ) ) {
    if ( const auto charge = remaining_charge( b, *capacity ) )
      return std::clamp( *charge / *capacity, 0.0, 1.0 );
  }
  if ( is_valid_percentage( b.percentage ) )
    return static_cast<double>( b.percentage );
  return std::nullopt;
}

//! @brief Maps a 0-1 fraction to a rounded, clamped int8 percentage in [0, 100].
int8_t fraction_to_level( double fraction )
{
  const long level = std::lround( fraction * 100.0 );
  return static_cast<int8_t>( std::clamp<long>( level, 0, 100 ) );
}

std::chrono::nanoseconds frequency_to_period( double hz )
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>( 1.0 / hz ) );
}

//! @brief Placeholder BatteryState for a configured battery that has not reported yet: present = false
//!        and every measurement NaN (unknown). It makes a RobotStatus list all configured batteries
//!        from the first publish, and aggregation ignores it until a real message replaces it.
//!        `location` carries the configured label so a consumer can tell the packs apart.
sensor_msgs::msg::BatteryState unknown_battery_state( const std::string &label )
{
  constexpr float nan = std::numeric_limits<float>::quiet_NaN();
  sensor_msgs::msg::BatteryState battery;
  battery.voltage = nan;
  battery.temperature = nan;
  battery.current = nan;
  battery.charge = nan;
  battery.capacity = nan;
  battery.design_capacity = nan;
  battery.percentage = nan;
  // power_supply_status/health/technology default to their UNKNOWN (0) constants.
  battery.present = false;
  battery.location = label;
  return battery;
}
} // namespace

std::map<std::string, std::string>
parse_battery_topics( const std::map<std::string, rclcpp::ParameterValue> &overrides )
{
  std::map<std::string, std::string> result;
  for ( const auto &[name, value] : overrides ) {
    const std::optional<std::string> label = label_from_battery_key( name );
    if ( !label )
      continue;
    if ( value.get_type() != rclcpp::ParameterType::PARAMETER_STRING )
      continue; // the topic must be a string
    const std::string topic = value.get<std::string>();
    if ( topic.empty() )
      continue;
    result.insert_or_assign( *label, topic );
  }
  return result;
}

BatteryAggregate aggregate_battery_level( const std::vector<sensor_msgs::msg::BatteryState> &batteries,
                                          BatteryAggregation strategy )
{
  BatteryAggregate result;

  if ( strategy == BatteryAggregation::Combined ) {
    double charge_sum = 0.0;
    double capacity_sum = 0.0;
    size_t with_level = 0;   // packs exposing a usable level (a fraction)
    size_t contributing = 0; // packs in the capacity-weighted energy pool
    for ( const auto &battery : batteries ) {
      if ( battery_fraction( battery ) )
        ++with_level;
      const auto capacity = effective_capacity( battery );
      if ( !capacity )
        continue;
      const auto charge = remaining_charge( battery, *capacity );
      if ( !charge )
        continue;
      ++contributing;
      charge_sum += *charge;
      capacity_sum += *capacity;
    }
    if ( capacity_sum > 0.0 ) {
      result.level = fraction_to_level( charge_sum / capacity_sum );
      result.combined_dropped = with_level - contributing; // had a level but no capacity weight
      return result;
    }
    // No pack exposed a capacity: fall back to the mean of per-pack fractions below.
    result.combined_used_mean_fallback = with_level > 0;
  }

  double min_fraction = std::numeric_limits<double>::infinity();
  double fraction_sum = 0.0;
  size_t valid = 0;
  for ( const auto &battery : batteries ) {
    const auto fraction = battery_fraction( battery );
    if ( !fraction )
      continue;
    ++valid;
    fraction_sum += *fraction;
    min_fraction = std::min( min_fraction, *fraction );
  }
  if ( valid == 0 )
    return result; // level stays -1: empty set or every pack unknown

  const double fraction =
      ( strategy == BatteryAggregation::Minimum ) ? min_fraction : ( fraction_sum / valid );
  result.level = fraction_to_level( fraction );
  return result;
}

StatusReporter::StatusReporter( rclcpp::Node &node, const std::string &robot_id )
    : node_( node ), robot_id_( robot_id )
{
  load_config();
}

void StatusReporter::load_config()
{
  const auto &overrides = node_.get_node_parameters_interface()->get_parameter_overrides();

  // Declares a read-only frequency parameter and resolves it as a double. <= 0 / absent /
  // non-numeric / non-finite all mean "disabled". A non-finite value (e.g. .inf) would otherwise
  // pass the > 0 check and yield a zero-period timer that spins the executor.
  const auto declare_frequency = [&]( const std::string &name, const char *description ) -> double {
    declare_param_preserving_override_type(
        node_, name, 0.0,
        ParameterDescriptor().set__read_only( true ).set__description( description ) );
    const std::optional<double> hz =
        numeric_value_as_double( node_.get_parameter( name ).get_parameter_value() );
    if ( !hz || !std::isfinite( *hz ) ) {
      RCLCPP_WARN( node_.get_logger(),
                   "Ignoring non-numeric or non-finite value for parameter '%s'; the corresponding "
                   "publishing is disabled.",
                   name.c_str() );
      return 0.0;
    }
    return *hz;
  };

  const double status_frequency =
      declare_frequency( "status.status_frequency",
                         "Frequency in Hz at which RobotStatus is published on 'robot_status'. "
                         "0 disables status publishing." );
  const double heartbeat_frequency =
      declare_frequency( "status.heartbeat_frequency",
                         "Frequency in Hz at which Heartbeat is published on 'robot_heartbeat'. "
                         "0 disables heartbeat publishing." );

  const std::string aggregation = node_.declare_parameter<std::string>(
      "status.battery.aggregation", "minimum",
      ParameterDescriptor().set__read_only( true ).set__description(
          "How RobotStatus.battery_level aggregates multiple batteries: 'minimum' (worst pack, the "
          "robot is limited by it) or 'combined' (capacity-weighted pooled level, "
          "sum(charge)/sum(capacity); charge is derived from percentage and capacity falls back to "
          "design capacity when not measured directly)." ) );
  if ( aggregation == "combined" ) {
    aggregation_ = BatteryAggregation::Combined;
  } else {
    aggregation_ = BatteryAggregation::Minimum;
    if ( aggregation != "minimum" )
      RCLCPP_WARN( node_.get_logger(), "Unknown battery aggregation '%s'; using 'minimum'.",
                   aggregation.c_str() );
  }

  // Heartbeat is independent of status: best-effort, volatile QoS since a stale latched heartbeat
  // would be misleading.
  if ( heartbeat_frequency > 0.0 ) {
    heartbeat_pub_ = node_.create_publisher<Heartbeat>( "robot_heartbeat", rclcpp::SensorDataQoS() );
    heartbeat_timer_ = node_.create_wall_timer( frequency_to_period( heartbeat_frequency ),
                                                std::bind( &StatusReporter::publish_heartbeat, this ) );
    RCLCPP_INFO( node_.get_logger(), "Publishing heartbeat at %.1f Hz on 'robot_heartbeat'.",
                 heartbeat_frequency );
  } else {
    RCLCPP_INFO( node_.get_logger(), "Heartbeat publishing is disabled." );
  }

  if ( status_frequency <= 0.0 ) {
    RCLCPP_INFO( node_.get_logger(),
                 "Status publishing is disabled; no battery monitoring or set_status service." );
    return;
  }

  // Latched so a late-joining UI immediately receives the last status.
  status_pub_ = node_.create_publisher<RobotStatus>( "robot_status", latched_qos() );

  // The battery topic set is open-ended, so it is read straight from the overrides. Each recognized
  // key is also declared read-only so it is introspectable via `ros2 param list`.
  for ( const auto &[name, value] : overrides ) {
    if ( !label_from_battery_key( name ) || node_.has_parameter( name ) )
      continue;
    node_.declare_parameter( name, value,
                             ParameterDescriptor().set__read_only( true ).set__description(
                                 "Topic publishing sensor_msgs/BatteryState for this battery." ) );
  }

  const auto battery_topics = parse_battery_topics( overrides );
  std::stringstream battery_log;
  for ( const auto &[label, topic] : battery_topics ) {
    // Seed an unknown/absent placeholder before subscribing so RobotStatus reports every configured
    // battery from the first publish, not only those that have already produced a message.
    latest_batteries_[label] = unknown_battery_state( label );
    battery_subs_.push_back( node_.create_subscription<sensor_msgs::msg::BatteryState>(
        topic, rclcpp::SensorDataQoS(),
        [this, label = label]( sensor_msgs::msg::BatteryState::ConstSharedPtr msg ) {
          battery_callback( label, msg );
        } ) );
    battery_log << " " << label << " (" << topic << ")";
  }
  if ( battery_topics.empty() )
    RCLCPP_INFO( node_.get_logger(), "No battery topics configured." );
  else
    RCLCPP_INFO( node_.get_logger(), "Monitoring %zu battery topic(s):%s", battery_topics.size(),
                 battery_log.str().c_str() );

  set_status_srv_ = node_.create_service<SetStatus>(
      "set_status", std::bind( &StatusReporter::handle_set_status, this, std::placeholders::_1,
                               std::placeholders::_2 ) );

  status_timer_ = node_.create_wall_timer( frequency_to_period( status_frequency ),
                                           std::bind( &StatusReporter::publish_status, this ) );
  RCLCPP_INFO( node_.get_logger(), "Publishing status at %.1f Hz on 'robot_status'.",
               status_frequency );
}

void StatusReporter::battery_callback( const std::string &label,
                                       sensor_msgs::msg::BatteryState::ConstSharedPtr msg )
{
  std::lock_guard<std::mutex> lock( mutex_ );
  latest_batteries_[label] = *msg;
}

void StatusReporter::publish_status()
{
  RobotStatus status;
  status.stamp = node_.now();
  status.robot_id = robot_id_;
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    status.batteries.reserve( latest_batteries_.size() );
    for ( const auto &entry : latest_batteries_ )
      status.batteries.push_back( entry.second );
    status.status_code = status_code_;
    status.status_message = status_message_;
  }
  const BatteryAggregate battery = aggregate_battery_level( status.batteries, aggregation_ );
  status.battery_level = battery.level;

  if ( battery.combined_used_mean_fallback ) {
    RCLCPP_WARN_THROTTLE( node_.get_logger(), *node_.get_clock(), 10000,
                          "Combined battery aggregation selected but no battery exposes a capacity "
                          "(or design capacity) with a charge or percentage; using the mean of "
                          "per-battery levels instead." );
  } else if ( battery.combined_dropped > 0 ) {
    RCLCPP_WARN_THROTTLE( node_.get_logger(), *node_.get_clock(), 10000,
                          "Combined battery aggregation is omitting one or more batteries that "
                          "report a level but no capacity (nor design capacity); the pooled "
                          "battery_level does not account for them." );
  }

  status_pub_->publish( status );
}

void StatusReporter::publish_heartbeat()
{
  Heartbeat heartbeat;
  heartbeat.stamp = node_.now();
  heartbeat.seq = heartbeat_seq_++;
  heartbeat_pub_->publish( heartbeat );
}

void StatusReporter::handle_set_status( const std::shared_ptr<SetStatus::Request> request,
                                        std::shared_ptr<SetStatus::Response> /*response*/ )
{
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    status_code_ = request->status_code;
    status_message_ = request->status_message;
  }
  // Reflect the new status immediately instead of waiting for the next timer tick.
  publish_status();
}

} // namespace hector_multi_robot_announcement
