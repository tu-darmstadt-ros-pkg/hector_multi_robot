#ifndef HECTOR_MULTI_ROBOT_ANNOUNCEMENT_TOPIC_FORWARDING_HPP
#define HECTOR_MULTI_ROBOT_ANNOUNCEMENT_TOPIC_FORWARDING_HPP

#include <chrono>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/messages/message.hpp>

namespace hector_multi_robot_announcement
{

//! @brief Parameter name prefix that identifies a forwarded-topic configuration block. Includes the
//!        `topics.` segment so the first-dot split of the remainder yields `<id>` and `<field>`; a
//!        bare `topic_forwarding.` prefix would mis-split `<id>.<field>` into id="topics".
inline constexpr const char *kForwardedTopicPrefix = "topic_forwarding.topics.";

//! @brief One forwarded topic parsed from the `topic_forwarding.topics.<id>` config map.
struct ForwardedTopicConfig {
  std::string id;
  std::string topic;         //!< Source topic relative to the robot namespace (leading '/' stripped).
  std::string message_type;  //!< Empty = auto-discover the type from the ROS graph.
};

//! @brief Builds the forwarded-topic list from parameter overrides of the form
//!        "topic_forwarding.topics.<id>.<field>".
//!
//! Like `visualizations`, this is an open-ended YAML map read straight from the overrides. Each
//! `<id>` yields one ForwardedTopicConfig with the recognized fields `topic` (required) and
//! `message_type` (optional). An entry without a non-empty `topic` is skipped; unknown fields are
//! ignored. Entries are returned in the overrides map's sorted key (id) order.
std::vector<ForwardedTopicConfig>
parse_forwarded_topics( const std::map<std::string, rclcpp::ParameterValue> &overrides,
                        const rclcpp::Logger &logger );

//! @brief Recursively rewrites every `frame_id`/`child_frame_id` string field of `message` via
//!        prefix_frame_id(), descending into nested compounds and compound arrays.
//!
//! A frame id is always a scalar string in ROS message conventions, so string arrays are left
//! untouched. Empty and `global_frames` members are preserved (see prefix_frame_id).
void prefix_frame_ids( ros_babel_fish::Message &message, const std::string &prefix,
                       const std::unordered_set<std::string> &global_frames );

//! @brief Forwards a configured set of a robot's namespaced topics into a subnamespace, rewriting
//!        every frame id to match the prefixed global tf tree.
//!
//! For each configured topic it resolves the message type and QoS from the ROS graph (an explicit
//! `message_type` pins the type), generically subscribes via ros_babel_fish, rewrites frame ids
//! with the robot namespace prefix (leaving `tf_config.global_frames` untouched), and republishes
//! under `<ns>/<subnamespace>/<topic>`. Topics whose publisher has not appeared yet stay pending
//! and are retried at `resolve_period`.
//!
//! Construction declares the `topic_forwarding.subnamespace` parameter and, when the node is
//! namespaced and at least one topic and a non-empty subnamespace are configured, starts the
//! resolve timer. Otherwise the forwarder stays inert (see active()). Keep the instance alive for
//! as long as forwarding should run.
//!
//! @note Thread-safety under a multi-threaded executor rests on the resolve timer being the sole
//!       mutator of the forwarder state and living in a mutually-exclusive callback group; it must
//!       not be placed in a reentrant group. Subscription callbacks share no mutable state.
class TopicForwarder
{
public:
  //! @param node             Node that owns the parameter, subscriptions and publishers. Must
  //!                         outlive this forwarder.
  //! @param robot_namespace  Absolute robot namespace (e.g. "/robot1"); used as frame prefix.
  //! @param resolve_period   Poll interval for resolving pending topics against the graph.
  TopicForwarder( rclcpp::Node &node, const std::string &robot_namespace,
                  std::chrono::nanoseconds resolve_period = std::chrono::seconds( 1 ) );

  //! @brief Whether forwarding is wired up (namespaced, at least one topic, non-empty subnamespace).
  bool active() const { return active_; }

  //! @brief Number of topics whose subscription has been created so far (for tests).
  size_t resolved_count() const { return entries_.size(); }

private:
  //! @brief Timer callback: resolves each still-pending topic against the graph and, once its
  //!        publisher appears, creates the subscription/publisher pair. Cancels itself when the
  //!        pending set is empty.
  void resolve_pending();

  rclcpp::Node &node_;
  ros_babel_fish::BabelFish fish_;

  //! @brief Prefix prepended to forwarded frame ids, e.g. "robot1/".
  std::string frame_prefix_;
  //! @brief Frame ids shared across robots that must not be prefixed (e.g. "map", "world").
  std::unordered_set<std::string> global_frames_;
  bool active_ = false;

  //! @brief A topic awaiting graph resolution. `configured_type` empty ⇒ auto-discover (same
  //!        empty-string sentinel as the config).
  struct Pending {
    std::string input_topic, output_topic, configured_type;
  };
  std::vector<Pending> pending_;

  //! @brief A resolved topic; both the subscription and publisher are kept alive here.
  struct Entry {
    ros_babel_fish::BabelFishSubscription::SharedPtr sub;
    ros_babel_fish::BabelFishPublisher::SharedPtr pub;
  };
  std::vector<Entry> entries_;

  rclcpp::TimerBase::SharedPtr resolve_timer_;
};

} // namespace hector_multi_robot_announcement

#endif // HECTOR_MULTI_ROBOT_ANNOUNCEMENT_TOPIC_FORWARDING_HPP
