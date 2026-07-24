#ifndef HECTOR_MULTI_ROBOT_ANNOUNCEMENT_SIMPLE_MULTI_ROBOT_ANNOUNCER_HPP
#define HECTOR_MULTI_ROBOT_ANNOUNCEMENT_SIMPLE_MULTI_ROBOT_ANNOUNCER_HPP

#include <memory>
#include <string>

#include <hector_multi_robot_msgs/msg/robot_announcement.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hector_multi_robot_announcement/announcer.hpp"
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
  //! @brief Hands the built announcement to the Announcer and wires up the tf/topic forwarders and
  //!        status reporter. Takes the announcement by value so it can be moved into the Announcer.
  void setup( hector_multi_robot_msgs::msg::RobotAnnouncement announcement );

  std::string robot_id_;
  std::string robot_namespace_;

  //! @brief Owns the RobotAnnouncement, its latched publisher(s) and the runtime add/remove services.
  std::unique_ptr<Announcer> announcer_;

  //! @brief Forwards the robot's namespaced tf tree to the global tf tree (inert unless enabled).
  std::unique_ptr<TfForwarder> tf_forwarder_;

  //! @brief Forwards configured topics into a subnamespace, rewriting frame ids (inert unless
  //!        topics are configured).
  std::unique_ptr<TopicForwarder> topic_forwarder_;

  //! @brief Publishes RobotStatus + Heartbeat and aggregates battery state (inert unless enabled).
  std::unique_ptr<StatusReporter> status_reporter_;
};

} // namespace hector_multi_robot_announcement

#endif // HECTOR_MULTI_ROBOT_ANNOUNCEMENT_SIMPLE_MULTI_ROBOT_ANNOUNCER_HPP
