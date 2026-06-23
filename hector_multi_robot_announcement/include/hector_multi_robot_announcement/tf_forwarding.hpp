#ifndef HECTOR_MULTI_ROBOT_ANNOUNCEMENT_TF_FORWARDING_HPP
#define HECTOR_MULTI_ROBOT_ANNOUNCEMENT_TF_FORWARDING_HPP

#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace hector_multi_robot_announcement
{

//! @brief Parameter name prefix that identifies a per-frame configuration block.
inline constexpr const char *kFrameConfigPrefix = "tf_config.frame_configs.";
//! @brief Suffix of the per-frame rate parameter, e.g. "tf_config.frame_configs.<frame>.rate".
inline constexpr const char *kRateSuffix = ".rate";

//! @brief Extracts per-frame minimum publish intervals from parameter overrides.
//!
//! Scans for keys of the form "tf_config.frame_configs.<frame>.rate" and maps <frame> to a
//! minimum publish interval of 1/rate. A rate <= 0 (or a non-numeric value) maps to a zero
//! duration, meaning "always forward". Keys that do not match the pattern are ignored.
std::unordered_map<std::string, rclcpp::Duration>
parse_frame_intervals( const std::map<std::string, rclcpp::ParameterValue> &overrides );

//! @brief Prepends `prefix` to `frame` unless `frame` is empty or listed in `global_frames`.
std::string prefix_frame_id( const std::string &frame, const std::string &prefix,
                             const std::unordered_set<std::string> &global_frames );

//! @brief Returns a copy of `transform` with both its parent and child frame ids prefixed
//!        via prefix_frame_id().
geometry_msgs::msg::TransformStamped
prefix_transform( geometry_msgs::msg::TransformStamped transform, const std::string &prefix,
                  const std::unordered_set<std::string> &global_frames );

//! @brief Default upper bound on the number of frames tracked for rate limiting before stale
//!        entries are evicted (see FrameRateLimiter::prune).
inline constexpr size_t kDefaultMaxTrackedFrames = 1024;

//! @brief Tracks per-frame publish times and decides whether a transform may be forwarded,
//!        enforcing a per-frame (or default) minimum publish interval.
//!
//! Frames whose resolved interval is zero are always forwarded and incur no bookkeeping, so
//! they never accumulate state. For rate-limited frames a last-publish time is tracked; the
//! tracked set is bounded (see prune) so a robot that emits transient, uniquely named child
//! frames cannot grow it without limit. A backwards time jump (sim reset, looping bag) is
//! treated as "interval elapsed" so forwarding re-anchors instead of stalling.
//!
//! @note Not internally synchronized; callers must serialize access (TfForwarder guards it
//!       with a mutex so it is safe under a multi-threaded executor).
class FrameRateLimiter
{
public:
  FrameRateLimiter() = default;
  FrameRateLimiter( rclcpp::Duration default_interval,
                    std::unordered_map<std::string, rclcpp::Duration> per_frame_interval,
                    size_t max_tracked_frames = kDefaultMaxTrackedFrames );

  //! @brief Returns true if a transform for `frame` may be forwarded at `now`, recording the
  //!        time when it returns true (always-forward frames are not recorded).
  bool allow( const std::string &frame, const rclcpp::Time &now );

  //! @brief Number of frames for which a last-publish time is currently tracked.
  size_t tracked_frame_count() const { return last_publish_time_.size(); }

  //! @brief Current upper bound on tracked frames; grows when the set cannot be pruned below it.
  size_t tracked_frame_limit() const { return max_tracked_frames_; }

private:
  //! @brief Bounds last_publish_time_. Called before tracking a new frame: evicts entries whose
  //!        throttle window has already elapsed (re-forwarding them later is free anyway), and
  //!        if that cannot bring the set below the limit raises the limit so it sits well under
  //!        it again rather than pruning on every message.
  void prune( const rclcpp::Time &now );

  //! @brief Resolved minimum publish interval for `frame`: its per-frame value if configured,
  //!        otherwise the default. Used by both allow() and prune() so they stay in sync.
  const rclcpp::Duration &resolve_interval( const std::string &frame ) const;

  rclcpp::Duration default_interval_ = rclcpp::Duration::from_nanoseconds( 0 );
  std::unordered_map<std::string, rclcpp::Duration> per_frame_interval_;
  std::unordered_map<std::string, rclcpp::Time> last_publish_time_;
  size_t max_tracked_frames_ = kDefaultMaxTrackedFrames;
};

//! @brief Forwards a robot's namespaced tf tree to the global tf tree.
//!
//! Owns the complete forwarding behaviour: it declares the `enable_tf_forwarding` and
//! `tf_config.*` parameters, subscribes to `<robot_namespace>/tf` and `/tf_static`, prefixes
//! frame ids with the robot namespace (leaving `tf_config.global_frames` untouched),
//! rate-limits dynamic transforms per child frame, and re-broadcasts them on the global tf
//! topics.
//!
//! Construction declares the parameters and, when `enable_tf_forwarding` is true and the node
//! is namespaced, wires up the subscriptions and broadcasters. Otherwise the forwarder stays
//! inert (see active()). Keep the instance alive for as long as forwarding should run; its
//! subscriptions stop when it is destroyed.
//!
//! @note Before constructing the node, ensure its global /tf and /tf_static topics are shielded
//!       from the common `/tf:=tf` multi-robot remap (the announcer does this via
//!       bypass_global_tf_remap) so the global tf publisher topics survive it.
class TfForwarder
{
public:
  //! @param node              Node that owns the parameters, subscriptions and broadcasters.
  //!                          Must outlive this forwarder.
  //! @param robot_namespace   Absolute robot namespace (e.g. "/robot1"); used as frame prefix.
  TfForwarder( rclcpp::Node &node, const std::string &robot_namespace );

  //! @brief Whether forwarding is active (enabled, namespaced, and subscriptions set up).
  bool active() const { return tf_sub_ != nullptr; }

private:
  //! @brief Discovers tf_config.frame_configs.<frame>.rate parameters and initializes
  //!        rate_limiter_.
  void load_config();

  void tf_callback( tf2_msgs::msg::TFMessage::ConstSharedPtr msg );
  void tf_static_callback( tf2_msgs::msg::TFMessage::ConstSharedPtr msg );

  rclcpp::Node &node_;

  //! @brief Prefix prepended to forwarded frame ids, e.g. "robot1/".
  std::string frame_prefix_;

  //! @brief Frame ids that are shared across robots (e.g. "map", "world") and must not be
  //!        prefixed when forwarded to the global tf tree.
  std::unordered_set<std::string> global_frames_;

  //! @brief Rate-limits forwarded dynamic transforms per child frame.
  FrameRateLimiter rate_limiter_;

  //! @brief Serializes rate_limiter_ access; the node may run under a multi-threaded executor
  //!        in a shared component container, where its callbacks can fire concurrently.
  std::mutex rate_limiter_mutex_;

  //! @brief Serializes StaticTransformBroadcaster::sendTransform, which mutates an internal
  //!        accumulated message and is not itself thread-safe.
  std::mutex static_broadcast_mutex_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  //! @brief Subscriptions are declared last so they are destroyed first: their callbacks
  //!        dereference the broadcasters, rate_limiter_ and mutexes above, which must therefore
  //!        outlive them while the executor may still be dispatching a callback during teardown.
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
};

} // namespace hector_multi_robot_announcement

#endif // HECTOR_MULTI_ROBOT_ANNOUNCEMENT_TF_FORWARDING_HPP
