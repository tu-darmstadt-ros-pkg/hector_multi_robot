#include "hector_multi_robot_announcement/tf_forwarding.hpp"

#include <functional>
#include <sstream>
#include <utility>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2_ros/qos.hpp>

#include "hector_multi_robot_announcement/utils.hpp"

using rcl_interfaces::msg::ParameterDescriptor;
using tf2_msgs::msg::TFMessage;

namespace hector_multi_robot_announcement
{

namespace
{
//! @brief If `name` is a "tf_config.frame_configs.<frame>.rate" key, returns the (non-empty)
//!        <frame>; otherwise std::nullopt. Single source of the key pattern, shared by
//!        parse_frame_intervals() and TfForwarder::load_config() so the two cannot drift.
std::optional<std::string> frame_from_rate_key( const std::string &name )
{
  const std::string prefix = kFrameConfigPrefix;
  const std::string suffix = kRateSuffix;
  if ( name.size() < prefix.size() + suffix.size() )
    return std::nullopt;
  if ( name.compare( 0, prefix.size(), prefix ) != 0 )
    return std::nullopt;
  if ( name.compare( name.size() - suffix.size(), suffix.size(), suffix ) != 0 )
    return std::nullopt;
  std::string frame = name.substr( prefix.size(), name.size() - prefix.size() - suffix.size() );
  if ( frame.empty() )
    return std::nullopt;
  return frame;
}
} // namespace

std::optional<double> numeric_value_as_double( const rclcpp::ParameterValue &value )
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

std::unordered_map<std::string, rclcpp::Duration>
parse_frame_intervals( const std::map<std::string, rclcpp::ParameterValue> &overrides )
{
  std::unordered_map<std::string, rclcpp::Duration> result;
  for ( const auto &[name, value] : overrides ) {
    const std::optional<std::string> frame = frame_from_rate_key( name );
    if ( !frame )
      continue;

    const std::optional<double> rate_hz = numeric_value_as_double( value );
    rclcpp::Duration interval = rclcpp::Duration::from_nanoseconds( 0 );
    if ( rate_hz && *rate_hz > 0.0 )
      interval = rclcpp::Duration::from_seconds( 1.0 / *rate_hz );
    result.insert_or_assign( *frame, interval );
  }
  return result;
}

std::string prefix_frame_id( const std::string &frame, const std::string &prefix,
                             const std::unordered_set<std::string> &global_frames )
{
  if ( frame.empty() || global_frames.count( frame ) > 0 )
    return frame;
  return prefix + frame;
}

geometry_msgs::msg::TransformStamped
prefix_transform( geometry_msgs::msg::TransformStamped transform, const std::string &prefix,
                  const std::unordered_set<std::string> &global_frames )
{
  transform.header.frame_id = prefix_frame_id( transform.header.frame_id, prefix, global_frames );
  transform.child_frame_id = prefix_frame_id( transform.child_frame_id, prefix, global_frames );
  return transform;
}

FrameRateLimiter::FrameRateLimiter(
    rclcpp::Duration default_interval,
    std::unordered_map<std::string, rclcpp::Duration> per_frame_interval,
    size_t max_tracked_frames )
    : default_interval_( default_interval )
    , per_frame_interval_( std::move( per_frame_interval ) )
    , max_tracked_frames_( max_tracked_frames )
{
}

const rclcpp::Duration &FrameRateLimiter::resolve_interval( const std::string &frame ) const
{
  const auto it = per_frame_interval_.find( frame );
  return it != per_frame_interval_.end() ? it->second : default_interval_;
}

bool FrameRateLimiter::allow( const std::string &frame, const rclcpp::Time &now )
{
  const rclcpp::Duration &interval = resolve_interval( frame );
  if ( interval.nanoseconds() <= 0 )
    return true; // Always forward; skip bookkeeping so these frames never accumulate state.

  const auto last_it = last_publish_time_.find( frame );
  if ( last_it != last_publish_time_.end() ) {
    // A negative elapsed time means the clock jumped backwards (sim reset / looping bag);
    // forward and re-anchor rather than blocking the frame until time re-passes the old stamp.
    const int64_t elapsed_ns = ( now - last_it->second ).nanoseconds();
    if ( elapsed_ns >= 0 && elapsed_ns < interval.nanoseconds() )
      return false;
    last_it->second = now;
    return true;
  }

  prune( now );
  last_publish_time_.emplace( frame, now );
  return true;
}

void FrameRateLimiter::prune( const rclcpp::Time &now )
{
  if ( last_publish_time_.size() < max_tracked_frames_ )
    return;

  // Drop frames whose throttle window has already elapsed (or whose stamp is in the future after
  // a backwards time jump): a later message for them would be allowed and re-tracked anyway, so
  // removing the entry now changes nothing.
  for ( auto it = last_publish_time_.begin(); it != last_publish_time_.end(); ) {
    const rclcpp::Duration &interval = resolve_interval( it->first );
    const int64_t elapsed_ns = ( now - it->second ).nanoseconds();
    if ( elapsed_ns < 0 || elapsed_ns >= interval.nanoseconds() ) {
      it = last_publish_time_.erase( it );
    } else {
      ++it;
    }
  }

  // Every remaining frame is still inside its window, so nothing more can be freed without
  // affecting throttling. Raise the limit (to twice the active set) so we sit well below it
  // again instead of attempting a prune on every subsequent new frame.
  if ( last_publish_time_.size() >= max_tracked_frames_ )
    max_tracked_frames_ = last_publish_time_.size() * 2;
}

TfForwarder::TfForwarder( rclcpp::Node &node, const std::string &robot_namespace ) : node_( node )
{
  node_.declare_parameter<bool>( "enable_tf_forwarding", false,
                                 ParameterDescriptor().set__read_only( true ).set__description(
                                     "Whether to forward the robot's tf tree to the global tf "
                                     "tree, prefixing frame ids with the robot namespace." ) );
  node_.declare_parameter<std::vector<std::string>>(
      "tf_config.global_frames", std::vector<std::string>{},
      ParameterDescriptor().set__read_only( true ).set__description(
          "Frame ids that are shared across robots and must not be prefixed when forwarded "
          "to the global tf tree (e.g. 'map', 'world')." ) );
  const auto max_rate_descriptor =
      ParameterDescriptor().set__read_only( true ).set__description(
          "Maximum rate in Hz at which transforms without an explicit "
          "tf_config.frame_configs.<frame>.rate parameter are forwarded to the global tf tree." );
  const auto &overrides = node_.get_node_parameters_interface()->get_parameter_overrides();
  const auto max_rate_override = overrides.find( "tf_config.max_rate" );
  if ( max_rate_override != overrides.end() ) {
    // Preserve the override's original type so integer YAML literals (`max_rate: 30`) do not
    // fail declaration; load_config() resolves integers and doubles through numeric_value_as_double.
    node_.declare_parameter( "tf_config.max_rate", max_rate_override->second, max_rate_descriptor );
  } else {
    node_.declare_parameter<double>( "tf_config.max_rate", 30.0, max_rate_descriptor );
  }

  // Canonicalize before building topic names: a trailing or repeated slash (e.g. "/robot1/")
  // would otherwise yield an invalid "/robot1//tf" and make create_subscription throw.
  const std::string ns = normalize_namespace( robot_namespace );
  if ( ns == "/" || !node_.get_parameter( "enable_tf_forwarding" ).as_bool() ) {
    return;
  }
  frame_prefix_ = ns.substr( 1 ) + "/";

  const auto global_frames_param =
      node_.get_parameter( "tf_config.global_frames" ).as_string_array();
  global_frames_.insert( global_frames_param.begin(), global_frames_param.end() );

  load_config();

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>( node_ );
  tf_sub_ = node_.create_subscription<TFMessage>(
      ns + "/tf", tf2_ros::DynamicListenerQoS(),
      std::bind( &TfForwarder::tf_callback, this, std::placeholders::_1 ) );

  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>( node_ );
  tf_static_sub_ = node_.create_subscription<TFMessage>(
      ns + "/tf_static", tf2_ros::StaticListenerQoS(),
      std::bind( &TfForwarder::tf_static_callback, this, std::placeholders::_1 ) );
}

void TfForwarder::load_config()
{
  rclcpp::Duration default_interval = rclcpp::Duration::from_nanoseconds( 0 );
  const auto max_rate_value = node_.get_parameter( "tf_config.max_rate" ).get_parameter_value();
  const std::optional<double> max_tf_rate = numeric_value_as_double( max_rate_value );
  if ( max_tf_rate && *max_tf_rate > 0.0 ) {
    default_interval = rclcpp::Duration::from_seconds( 1.0 / *max_tf_rate );
  } else if ( !max_tf_rate ) {
    RCLCPP_WARN( node_.get_logger(),
                 "Ignoring non-numeric rate for parameter 'tf_config.max_rate'; frames without "
                 "an explicit rate will be forwarded without rate limiting." );
  }

  // Per-frame rates come straight from the parameter overrides via parse_frame_intervals.
  // We additionally declare each recognized `.rate` key so it is introspectable via
  // `ros2 param list` (automatically_declare_parameters_from_overrides would conflict with the
  // read-only robot_id/robot_name/robot_namespace declarations). They are declared read-only with
  // their original override value: the value is consumed once below to build rate_limiter_, so a
  // runtime change has no effect, and re-declaring as a normalized double would clash with an
  // integer/string override's type. A single pass over the overrides declares, warns on a
  // non-numeric rate (which silently degrades to always-forward), and collects names for the log.
  const auto &overrides = node_.get_node_parameters_interface()->get_parameter_overrides();
  std::stringstream rate_limited_frames;
  for ( const auto &[name, value] : overrides ) {
    const std::optional<std::string> frame = frame_from_rate_key( name );
    if ( !frame )
      continue;
    if ( !numeric_value_as_double( value ) )
      RCLCPP_WARN( node_.get_logger(),
                   "Ignoring non-numeric rate for parameter '%s'; frame '%s' will be forwarded "
                   "without rate limiting.",
                   name.c_str(), frame->c_str() );
    if ( !node_.has_parameter( name ) )
      node_.declare_parameter(
          name, value,
          ParameterDescriptor().set__read_only( true ).set__description(
              "Per-frame maximum forwarding rate in Hz for this child frame. 0 forwards every "
              "message." ) );
    rate_limited_frames << " " << *frame;
  }

  auto frame_intervals = parse_frame_intervals( overrides );
  if ( !frame_intervals.empty() )
    RCLCPP_INFO( node_.get_logger(), "TF forwarding rate-limited for %zu frame(s):%s",
                 frame_intervals.size(), rate_limited_frames.str().c_str() );
  rate_limiter_ = FrameRateLimiter( default_interval, std::move( frame_intervals ) );
}

void TfForwarder::tf_callback( TFMessage::ConstSharedPtr msg )
{
  std::vector<geometry_msgs::msg::TransformStamped> out;
  out.reserve( msg->transforms.size() );
  const rclcpp::Time now_time = node_.now();
  {
    std::lock_guard<std::mutex> lock( rate_limiter_mutex_ );
    for ( const auto &t : msg->transforms ) {
      if ( !rate_limiter_.allow( t.child_frame_id, now_time ) )
        continue;
      out.push_back( prefix_transform( t, frame_prefix_, global_frames_ ) );
    }
  }
  if ( !out.empty() ) {
    tf_broadcaster_->sendTransform( out ); // TransformBroadcaster::sendTransform only publishes.
  }
}

void TfForwarder::tf_static_callback( TFMessage::ConstSharedPtr msg )
{
  std::vector<geometry_msgs::msg::TransformStamped> out;
  out.reserve( msg->transforms.size() );
  for ( const auto &t : msg->transforms ) {
    out.push_back( prefix_transform( t, frame_prefix_, global_frames_ ) );
  }
  if ( !out.empty() ) {
    std::lock_guard<std::mutex> lock( static_broadcast_mutex_ );
    tf_static_broadcaster_->sendTransform( out );
  }
}

} // namespace hector_multi_robot_announcement
