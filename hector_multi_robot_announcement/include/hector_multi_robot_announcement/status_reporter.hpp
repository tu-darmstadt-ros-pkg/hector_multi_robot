#ifndef HECTOR_MULTI_ROBOT_ANNOUNCEMENT_STATUS_REPORTER_HPP
#define HECTOR_MULTI_ROBOT_ANNOUNCEMENT_STATUS_REPORTER_HPP

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <sensor_msgs/msg/battery_state.hpp>

#include <hector_multi_robot_msgs/msg/heartbeat.hpp>
#include <hector_multi_robot_msgs/msg/robot_status.hpp>
#include <hector_multi_robot_msgs/srv/set_status.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hector_multi_robot_announcement
{

//! @brief Parameter name prefix that identifies a battery-state topic, e.g.
//!        "status.battery.state_topics.<label>" whose value is the topic to subscribe to.
inline constexpr const char *kBatteryStatePrefix = "status.battery.state_topics.";

//! @brief Strategy for collapsing several batteries into RobotStatus.battery_level.
enum class BatteryAggregation {
  //! Separate packs: the robot is limited by its weakest pack, so report the worst pack's level.
  Minimum,
  //! Parallel/pooled packs: report the pooled remaining capacity, sum(charge)/sum(capacity).
  Combined
};

//! @brief Extracts the open-ended "status.battery.state_topics.<label>" -> "<topic>" map from parameter
//!        overrides.
std::map<std::string, std::string>
parse_battery_topics( const std::map<std::string, rclcpp::ParameterValue> &overrides );

//! @brief Result of aggregate_battery_level: the level plus why the Combined strategy may have
//!        degraded, so a caller can warn without re-deriving per-battery capacity/charge.
struct BatteryAggregate {
  //! Battery level in [0, 100], or -1 when the set is empty or every battery's level is unknown.
  int8_t level = -1;
  //! Combined was requested but no pack exposed a capacity, so the mean of per-pack levels was
  //! used instead. Always false for the Minimum strategy.
  bool combined_used_mean_fallback = false;
  //! Number of packs with a usable level that were left out of the Combined energy pool because
  //! they expose no capacity. Always 0 for the Minimum strategy and when the mean fallback is used.
  size_t combined_dropped = 0;
};

//! @brief Aggregates a set of BatteryState messages into a battery level in [0, 100] (or -1 when
//!        the set is empty or every battery's level is unknown), reporting how the Combined
//!        strategy degraded via the returned BatteryAggregate.
BatteryAggregate aggregate_battery_level( const std::vector<sensor_msgs::msg::BatteryState> &batteries,
                                          BatteryAggregation strategy );

//! @brief Publishes a robot's live status and heartbeat and aggregates its battery state.
//!
//! Owns the complete status behaviour: it declares the `status.*` parameters, subscribes to the
//! configured `status.battery.state_topics.<label>` BatteryState topics, and on two independent timers
//! publishes a RobotStatus (status code/message + aggregated battery) on `robot_status` and a
//! Heartbeat on `robot_heartbeat`. A `set_status` service lets the robot's own software push a
//! status code/message that the next RobotStatus carries.
//!
//! Status publishing (with its battery subscriptions and the service) is only wired up when
//! `status.status_frequency` is > 0; heartbeat publishing only when `status.heartbeat_frequency` is
//! > 0. The two are independent. With neither configured the reporter stays inert (see
//! status_active()/heartbeat_active()). Keep the instance alive for as long as reporting should run;
//! its timers, subscriptions and service stop when it is destroyed.
class StatusReporter
{
public:
  //! @param node      Node that owns the parameters, publishers, subscriptions, timers and service.
  //!                  Must outlive this reporter.
  //! @param robot_id  Robot id copied into every published RobotStatus.
  StatusReporter( rclcpp::Node &node, const std::string &robot_id );

  //! @brief Whether periodic RobotStatus publishing is active (status_frequency > 0).
  bool status_active() const { return status_timer_ != nullptr; }

  //! @brief Whether periodic Heartbeat publishing is active (heartbeat_frequency > 0).
  bool heartbeat_active() const { return heartbeat_timer_ != nullptr; }

private:
  //! @brief Declares the `status.*` parameters, resolves the frequencies and aggregation strategy,
  //!        and discovers the `status.battery.state_topics.<label>` topic map.
  void load_config();

  void battery_callback( const std::string &label,
                         sensor_msgs::msg::BatteryState::ConstSharedPtr msg );
  void publish_status();    //!< status timer; also called from handle_set_status for an immediate update.
  void publish_heartbeat(); //!< heartbeat timer.
  void handle_set_status(
      const std::shared_ptr<hector_multi_robot_msgs::srv::SetStatus::Request> request,
      std::shared_ptr<hector_multi_robot_msgs::srv::SetStatus::Response> response );

  rclcpp::Node &node_;
  std::string robot_id_;

  //! @brief How RobotStatus.battery_level is aggregated; resolved once from a parameter.
  BatteryAggregation aggregation_ = BatteryAggregation::Minimum;

  //! @brief Serializes the shared status state below; the node may run under a multi-threaded
  //!        executor in a shared component container, where its callbacks can fire concurrently.
  std::mutex mutex_;
  //! @brief Latest BatteryState per configured label (guarded by mutex_).
  std::map<std::string, sensor_msgs::msg::BatteryState> latest_batteries_;
  //! @brief Current status code/message reported in RobotStatus (guarded by mutex_).
  uint8_t status_code_ = hector_multi_robot_msgs::msg::RobotStatus::STATUS_OK;
  std::string status_message_;

  rclcpp::Publisher<hector_multi_robot_msgs::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<hector_multi_robot_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;

  //! @brief Monotonic Heartbeat sequence number; only ever touched by the heartbeat timer callback
  //!        (mutually exclusive with itself), so it needs no lock.
  uint32_t heartbeat_seq_ = 0;

  //! @brief Subscriptions, service and timers are declared last so they are destroyed first: their
  //!        callbacks dereference mutex_ and the state above, which must outlive them while the
  //!        executor may still be dispatching a callback during teardown.
  std::vector<rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> battery_subs_;
  rclcpp::Service<hector_multi_robot_msgs::srv::SetStatus>::SharedPtr set_status_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

} // namespace hector_multi_robot_announcement

#endif // HECTOR_MULTI_ROBOT_ANNOUNCEMENT_STATUS_REPORTER_HPP
