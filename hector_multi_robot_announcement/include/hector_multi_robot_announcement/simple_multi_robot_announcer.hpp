#ifndef HECTOR_MULTI_ROBOT_ANNOUNCEMENT_ANNOUNCER_HPP
#define HECTOR_MULTI_ROBOT_ANNOUNCEMENT_ANNOUNCER_HPP

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <hector_multi_robot_msgs/msg/robot_announcement.hpp>
#include <hector_multi_robot_msgs/msg/robot_status.hpp>
#include <hector_multi_robot_msgs/msg/sensor.hpp>
#include <hector_multi_robot_msgs/msg/visualization.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hector_multi_robot_announcement/status_reporter.hpp"
#include "hector_multi_robot_announcement/tf_forwarding.hpp"
#include "hector_multi_robot_announcement/topic_forwarding.hpp"

namespace hector_multi_robot_announcement
{

class SimpleMultiRobotAnnouncer : public rclcpp::Node
{
public:
  explicit SimpleMultiRobotAnnouncer( const rclcpp::NodeOptions &options );

private:
  //! @brief Sets up subscribers, publishers, etc. to configure the node
  void setup();

  //! @brief Builds a RobotAnnouncement from the current members and publishes it on the namespaced
  //!        publisher and, when it exists, the global one.
  void publish_announcement();

  rclcpp::Publisher<hector_multi_robot_msgs::msg::RobotAnnouncement>::SharedPtr announcement_publisher_;
  rclcpp::Publisher<hector_multi_robot_msgs::msg::RobotAnnouncement>::SharedPtr global_announcement_publisher_;

  std::string robot_id_;
  std::string robot_name_;
  std::string robot_namespace_;
  std::string robot_type_;

  //! @brief Robot configuration as ordered string key/value pairs, copied into the announcement.
  std::vector<std::pair<std::string, std::string>> configuration_;

  //! @brief Visualization topic descriptions copied into the announcement (for UI/rviz consumers).
  std::vector<hector_multi_robot_msgs::msg::Visualization> visualizations_;

  //! @brief Sensor value descriptions copied into the announcement (for UI consumers).
  std::vector<hector_multi_robot_msgs::msg::Sensor> sensors_;

  //! @brief Forwards the robot's namespaced tf tree to the global tf tree (inert unless enabled).
  std::unique_ptr<TfForwarder> tf_forwarder_;

  //! @brief Forwards configured topics into a subnamespace, rewriting frame ids (inert unless
  //!        topics are configured).
  std::unique_ptr<TopicForwarder> topic_forwarder_;

  //! @brief Publishes RobotStatus + Heartbeat and aggregates battery state (inert unless enabled).
  std::unique_ptr<StatusReporter> status_reporter_;
};

} // namespace hector_multi_robot_announcement

#endif // HECTOR_MULTI_ROBOT_ANNOUNCEMENT_ANNOUNCER_HPP
