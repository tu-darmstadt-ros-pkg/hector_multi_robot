#ifndef HECTOR_MULTI_ROBOT_ANNOUNCEMENT_ANNOUNCER_HPP
#define HECTOR_MULTI_ROBOT_ANNOUNCEMENT_ANNOUNCER_HPP

#include <memory>
#include <mutex>

#include <hector_multi_robot_msgs/msg/robot_announcement.hpp>
#include <hector_multi_robot_msgs/srv/add_sensor.hpp>
#include <hector_multi_robot_msgs/srv/add_visualization.hpp>
#include <hector_multi_robot_msgs/srv/remove_sensor.hpp>
#include <hector_multi_robot_msgs/srv/remove_visualization.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hector_multi_robot_announcement
{

//! @brief Owns a robot's RobotAnnouncement: its latched publisher(s), the one-shot startup publish,
//!        and the runtime services that add/remove sensors and visualizations.
//!
//! The node builds the initial announcement (id/name/namespace/type, configuration, the parsed
//! sensor/visualization lists) and hands the finished message to this class, which becomes its sole
//! owner. On construction it creates the namespaced `robot_announcement` publisher (and, iff the node
//! is namespaced, the global `/robot_announcement` publisher), publishes once, and creates the four
//! services `add_sensor`, `remove_sensor`, `add_visualization` and `remove_visualization`. Each
//! successful add/remove upserts/erases the entry and republishes the (latched) announcement so late
//! joiners and existing consumers see the newest full announcement.
//!
//! The node may run under a multi-threaded executor in a shared component container, so the
//! announcement is guarded by a mutex; handlers copy a stamped announcement out under the lock and
//! publish it after releasing it. Keep the instance alive for as long as the announcement should be
//! served; it must be destroyed before the node that owns its parameters and interfaces.
class Announcer
{
public:
  //! @param node     Node that owns the publishers and services. Must outlive this Announcer.
  //! @param initial  The fully built initial announcement, minus the header stamp (set on every
  //!                 publish). Moved in and held as the single source of truth.
  Announcer( rclcpp::Node &node, hector_multi_robot_msgs::msg::RobotAnnouncement initial );

  Announcer( const Announcer & ) = delete;
  Announcer &operator=( const Announcer & ) = delete;

private:
  //! @brief Returns a copy of announcement_ stamped with the current time. Caller must hold mutex_.
  hector_multi_robot_msgs::msg::RobotAnnouncement stamped_announcement();

  //! @brief Publishes the announcement on the namespaced publisher and, when it exists, the global
  //!        one. Publishers are thread-safe, so this is called after releasing mutex_.
  void publish( const hector_multi_robot_msgs::msg::RobotAnnouncement &announcement );

  //! @brief Runs `mutate` under mutex_ and, iff it returns true, stamps the announcement under the
  //!        lock and republishes it after releasing the lock. Returns mutate's result so a handler
  //!        can report a no-op (e.g. removing an absent entry). `mutate` reports whether it changed
  //!        the announcement, i.e. whether a republish is warranted.
  template<typename Mutate>
  bool mutate_and_republish( Mutate &&mutate );

  void handle_add_sensor(
      const std::shared_ptr<hector_multi_robot_msgs::srv::AddSensor::Request> request,
      std::shared_ptr<hector_multi_robot_msgs::srv::AddSensor::Response> response );
  void handle_remove_sensor(
      const std::shared_ptr<hector_multi_robot_msgs::srv::RemoveSensor::Request> request,
      std::shared_ptr<hector_multi_robot_msgs::srv::RemoveSensor::Response> response );
  void handle_add_visualization(
      const std::shared_ptr<hector_multi_robot_msgs::srv::AddVisualization::Request> request,
      std::shared_ptr<hector_multi_robot_msgs::srv::AddVisualization::Response> response );
  void handle_remove_visualization(
      const std::shared_ptr<hector_multi_robot_msgs::srv::RemoveVisualization::Request> request,
      std::shared_ptr<hector_multi_robot_msgs::srv::RemoveVisualization::Response> response );

  rclcpp::Node &node_;

  //! @brief The single source of truth for the announcement (guarded by mutex_).
  hector_multi_robot_msgs::msg::RobotAnnouncement announcement_;

  //! @brief Serializes access to announcement_; the node may run under a multi-threaded executor.
  std::mutex mutex_;

  rclcpp::Publisher<hector_multi_robot_msgs::msg::RobotAnnouncement>::SharedPtr announcement_publisher_;
  //! @brief Global `/robot_announcement` publisher; only created when the node is namespaced (null
  //!        otherwise).
  rclcpp::Publisher<hector_multi_robot_msgs::msg::RobotAnnouncement>::SharedPtr global_announcement_publisher_;

  //! @brief The services are declared last so they are destroyed first: their handlers publish on the
  //!        publishers and read/mutate announcement_ under mutex_, which must outlive them while the
  //!        executor may still be dispatching a callback during teardown.
  rclcpp::Service<hector_multi_robot_msgs::srv::AddSensor>::SharedPtr add_sensor_srv_;
  rclcpp::Service<hector_multi_robot_msgs::srv::RemoveSensor>::SharedPtr remove_sensor_srv_;
  rclcpp::Service<hector_multi_robot_msgs::srv::AddVisualization>::SharedPtr add_visualization_srv_;
  rclcpp::Service<hector_multi_robot_msgs::srv::RemoveVisualization>::SharedPtr remove_visualization_srv_;
};

} // namespace hector_multi_robot_announcement

#endif // HECTOR_MULTI_ROBOT_ANNOUNCEMENT_ANNOUNCER_HPP
