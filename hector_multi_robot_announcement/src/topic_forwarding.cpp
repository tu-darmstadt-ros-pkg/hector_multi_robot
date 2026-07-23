#include "hector_multi_robot_announcement/topic_forwarding.hpp"

#include <algorithm>
#include <map>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <ros_babel_fish/messages/array_message.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>
#include <ros_babel_fish/messages/value_message.hpp>
#include <ros_babel_fish/method_invoke_helpers.hpp>

#include "hector_multi_robot_announcement/utils.hpp"

using rcl_interfaces::msg::ParameterDescriptor;
using ros_babel_fish::ArrayMessage_;
using ros_babel_fish::ArrayMessageBase;
using ros_babel_fish::CompoundArrayMessage_;
using ros_babel_fish::CompoundMessage;
using ros_babel_fish::Message;
namespace MessageTypes = ros_babel_fish::MessageTypes;

namespace hector_multi_robot_announcement
{

namespace
{
//! @brief Normalizes a subnamespace to a bare relative path: leading/trailing/duplicate slashes are
//!        collapsed (multi-segment allowed, e.g. "a//b/" -> "a/b"). "/" or empty maps to "" (topic
//!        forwarding is then disabled). normalize_namespace() yields "/a/b"; drop the leading slash.
std::string normalize_subnamespace( const std::string &subnamespace )
{
  const std::string normalized = normalize_namespace( subnamespace );
  return normalized == "/" ? "" : normalized.substr( 1 );
}

//! @brief Functor dispatched over an array message: recurses into compound arrays (each element may
//!        hold a frame id, e.g. Path.poses[]), ignores primitive/string arrays (a frame id is
//!        always a scalar string field, never an array).
struct FrameArrayRecursor {
  const std::string &prefix;
  const std::unordered_set<std::string> &globals;

  template<bool BOUNDED, bool FIXED_LENGTH>
  void operator()( CompoundArrayMessage_<BOUNDED, FIXED_LENGTH> &array ) const
  {
    for ( size_t i = 0; i < array.size(); ++i ) prefix_frame_ids( array[i], prefix, globals );
  }

  template<typename T, bool B, bool FL>
  void operator()( ArrayMessage_<T, B, FL> & ) const
  {
  }
};

//! @brief Normalizes a discovered source QoS into one safe to (re)use for a subscription and a
//!        publisher. Preserves reliability, durability and liveliness so latched/best-effort
//!        streams round-trip, but forces KeepLast(max(depth, 10)) when the reported history is
//!        KEEP_ALL/UNKNOWN or the depth is 0 (a raw copy would yield a KEEP_LAST(0) endpoint or an
//!        rmw warning).
rclcpp::QoS normalize_qos( const rclcpp::QoS &source )
{
  const rmw_qos_profile_t &profile = source.get_rmw_qos_profile();
  const bool history_unusable = profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL ||
                                profile.history == RMW_QOS_POLICY_HISTORY_UNKNOWN ||
                                profile.depth == 0;
  if ( !history_unusable )
    return source;

  rclcpp::QoS qos( rclcpp::KeepLast( std::max<size_t>( profile.depth, 10 ) ) );
  qos.reliability( profile.reliability );
  qos.durability( profile.durability );
  qos.liveliness( profile.liveliness );
  return qos;
}
} // namespace

std::vector<ForwardedTopicConfig>
parse_forwarded_topics( const std::map<std::string, rclcpp::ParameterValue> &overrides,
                        const rclcpp::Logger &logger )
{
  // Accumulate by id. The overrides map is sorted, so all keys of one id are contiguous and ids
  // come out sorted.
  std::map<std::string, ForwardedTopicConfig> by_id;
  for_each_override_id_field(
      overrides, kForwardedTopicPrefix,
      [&]( const std::string &id, const std::string &field, const rclcpp::ParameterValue &value ) {
        ForwardedTopicConfig &config = by_id[id];
        config.id = id;
        if ( field == "topic" )
          config.topic = rclcpp::to_string( value );
        else if ( field == "message_type" )
          config.message_type = rclcpp::to_string( value );
        // Unrecognized fields are ignored so consumers may add their own without breaking parsing.
      } );

  std::vector<ForwardedTopicConfig> result;
  for ( auto &[id, config] : by_id ) {
    if ( config.topic.empty() ) {
      RCLCPP_WARN( logger, "Skipping forwarded topic '%s': no 'topic' configured.", id.c_str() );
      continue;
    }
    result.push_back( std::move( config ) );
  }
  return result;
}

void prefix_frame_ids( Message &message, const std::string &prefix,
                       const std::unordered_set<std::string> &global_frames )
{
  if ( message.type() == MessageTypes::Compound ) {
    auto &compound = message.as<CompoundMessage>();
    const auto keys = compound.keys();
    auto values = compound.values();
    for ( size_t i = 0; i < keys.size(); ++i ) {
      Message &child = *values[i];
      if ( child.type() == MessageTypes::String &&
           ( keys[i] == "frame_id" || keys[i] == "child_frame_id" ) )
        child = prefix_frame_id( child.value<std::string>(), prefix, global_frames );
      else
        prefix_frame_ids( child, prefix, global_frames );
    }
  } else if ( message.type() == MessageTypes::Array ) {
    invoke_for_array_message( message.as<ArrayMessageBase>(),
                              FrameArrayRecursor{ prefix, global_frames } );
  }
}

TopicForwarder::TopicForwarder( rclcpp::Node &node, const std::string &robot_namespace,
                                std::chrono::nanoseconds resolve_period )
    : node_( node )
{
  const auto &overrides = node_.get_node_parameters_interface()->get_parameter_overrides();
  const std::vector<ForwardedTopicConfig> topics =
      parse_forwarded_topics( overrides, node_.get_logger() );

  node_.declare_parameter<std::string>(
      "topic_forwarding.subnamespace", "world",
      ParameterDescriptor().set__read_only( true ).set__description(
          "Subnamespace under which forwarded topics are republished, e.g. '<ns>/<subnamespace>/"
          "<topic>'." ) );
  const std::string subnamespace =
      normalize_subnamespace( node_.get_parameter( "topic_forwarding.subnamespace" ).as_string() );

  // Read global_frames directly from the overrides so we do not depend on TfForwarder having
  // declared it; both default to empty ("prefix everything").
  const auto global_frames_it = overrides.find( "tf_config.global_frames" );
  if ( global_frames_it != overrides.end() &&
       global_frames_it->second.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY ) {
    const auto frames = global_frames_it->second.get<std::vector<std::string>>();
    global_frames_.insert( frames.begin(), frames.end() );
  }

  const std::string ns = normalize_namespace( robot_namespace );
  if ( ns == "/" || topics.empty() || subnamespace.empty() ) {
    if ( ns == "/" )
      RCLCPP_INFO( node_.get_logger(), "Topic forwarding inactive: the node is not namespaced." );
    else if ( topics.empty() )
      RCLCPP_INFO( node_.get_logger(), "Topic forwarding inactive: no topics configured." );
    else
      RCLCPP_WARN( node_.get_logger(),
                   "Topic forwarding inactive: 'topic_forwarding.subnamespace' is empty (would "
                   "republish onto the source topic)." );
    return;
  }
  frame_prefix_ = ns.substr( 1 ) + "/";

  for ( const ForwardedTopicConfig &config : topics ) {
    // normalize_namespace collapses leading/trailing/duplicate slashes, so a topic with a leading
    // '/' or stray slashes composes into a valid rmw topic name instead of throwing on creation.
    const std::string input_topic = normalize_namespace( ns + "/" + config.topic );
    const std::string output_topic =
        normalize_namespace( ns + "/" + subnamespace + "/" + config.topic );
    pending_.push_back( { input_topic, output_topic, config.message_type } );
  }

  RCLCPP_INFO( node_.get_logger(),
               "Topic forwarding active: forwarding %zu topic(s) into subnamespace '%s' with frame "
               "prefix '%s'.",
               pending_.size(), subnamespace.c_str(), frame_prefix_.c_str() );

  active_ = true;
  resolve_timer_ = node_.create_wall_timer( resolve_period, [this] { resolve_pending(); } );
}

void TopicForwarder::resolve_pending()
{
  for ( auto it = pending_.begin(); it != pending_.end(); ) {
    const std::vector<rclcpp::TopicEndpointInfo> endpoints =
        node_.get_publishers_info_by_topic( it->input_topic );
    if ( endpoints.empty() ) {
      ++it; // No publisher yet; keep polling for both the type and the QoS.
      continue;
    }

    std::string type = endpoints.front().topic_type();
    const rclcpp::QoS qos = normalize_qos( endpoints.front().qos_profile() );
    const bool inconsistent_graph = std::any_of( endpoints.begin() + 1, endpoints.end(),
                                                 [&]( const rclcpp::TopicEndpointInfo &endpoint ) {
                                                   return endpoint.topic_type() != type;
                                                 } );
    if ( inconsistent_graph )
      RCLCPP_WARN( node_.get_logger(), "Topic '%s' has publishers with differing types; using '%s'.",
                   it->input_topic.c_str(), type.c_str() );

    // The subscription uses the first publisher's QoS. A publisher whose reliability or durability
    // differs is QoS-incompatible and silently delivers nothing, so warn rather than drop unnoticed.
    const rmw_qos_profile_t &ref_qos = endpoints.front().qos_profile().get_rmw_qos_profile();
    const bool inconsistent_qos = std::any_of(
        endpoints.begin() + 1, endpoints.end(), [&]( const rclcpp::TopicEndpointInfo &endpoint ) {
          const rmw_qos_profile_t &profile = endpoint.qos_profile().get_rmw_qos_profile();
          return profile.reliability != ref_qos.reliability ||
                 profile.durability != ref_qos.durability;
        } );
    if ( inconsistent_qos )
      RCLCPP_WARN( node_.get_logger(),
                   "Topic '%s' has publishers with differing QoS; using the first publisher's QoS, "
                   "messages from QoS-incompatible publishers will not be forwarded.",
                   it->input_topic.c_str() );

    if ( !it->configured_type.empty() ) {
      if ( it->configured_type != type )
        RCLCPP_WARN( node_.get_logger(),
                     "Topic '%s' is published as '%s' but pinned to '%s' in the config; using the "
                     "configured type.",
                     it->input_topic.c_str(), type.c_str(), it->configured_type.c_str() );
      type = it->configured_type;
    }

    try {
      auto pub = fish_.create_publisher( node_, it->output_topic, type, qos );
      auto sub = fish_.create_subscription(
          node_, it->input_topic, type, qos, [this, pub]( CompoundMessage::SharedPtr msg ) {
            prefix_frame_ids( *msg, frame_prefix_, global_frames_ );
            pub->publish( *msg );
          } );
      entries_.push_back( { sub, pub } );
      RCLCPP_INFO( node_.get_logger(), "Forwarding '%s' -> '%s' (type '%s').",
                   it->input_topic.c_str(), it->output_topic.c_str(), type.c_str() );
    } catch ( const ros_babel_fish::BabelFishException &e ) {
      RCLCPP_ERROR( node_.get_logger(), "Failed to forward '%s' -> '%s' (type '%s'): %s. Dropping from forward list.",
                    it->input_topic.c_str(), it->output_topic.c_str(), type.c_str(), e.what() );
    }

    it = pending_.erase( it );
  }

  if ( pending_.empty() )
    resolve_timer_->cancel();
}

} // namespace hector_multi_robot_announcement
