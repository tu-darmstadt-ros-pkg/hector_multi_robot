#ifndef HECTOR_MULTI_ROBOT_ANNOUNCEMENT_ANNOUNCER_HPP
#define HECTOR_MULTI_ROBOT_ANNOUNCEMENT_ANNOUNCER_HPP

#include <memory>
#include <string>
#include <vector>

#include <hector_multi_robot_msgs/msg/robot_announcement.hpp>
#include <hector_multi_robot_msgs/msg/robot_status.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hector_multi_robot_announcement
{

class SimpleMultiRobotAnnouncer : public rclcpp::Node
{
public:
  explicit SimpleMultiRobotAnnouncer( const rclcpp::NodeOptions &options );

private:
  //! @brief Sets up subscribers, publishers, etc. to configure the node
  void setup();

private:
  rclcpp::Publisher<hector_multi_robot_msgs::msg::RobotAnnouncement>::SharedPtr announcement_publisher_;
  rclcpp::Publisher<hector_multi_robot_msgs::msg::RobotAnnouncement>::SharedPtr global_announcement_publisher_;

  std::string robot_id_;
  std::string robot_name_;
  std::string robot_namespace_;
};

} // namespace hector_multi_robot_announcement

#endif // HECTOR_MULTI_ROBOT_ANNOUNCEMENT_ANNOUNCER_HPP
