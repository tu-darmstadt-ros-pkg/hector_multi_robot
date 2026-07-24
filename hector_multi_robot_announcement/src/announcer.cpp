#include "hector_multi_robot_announcement/announcer.hpp"

#include <algorithm>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "hector_multi_robot_announcement/sensor_config.hpp"
#include "hector_multi_robot_announcement/utils.hpp"
#include "hector_multi_robot_announcement/visualization_config.hpp"

using hector_multi_robot_msgs::msg::RobotAnnouncement;
using hector_multi_robot_msgs::msg::Sensor;
using hector_multi_robot_msgs::msg::Visualization;
using hector_multi_robot_msgs::srv::AddSensor;
using hector_multi_robot_msgs::srv::AddVisualization;
using hector_multi_robot_msgs::srv::RemoveSensor;
using hector_multi_robot_msgs::srv::RemoveVisualization;

namespace hector_multi_robot_announcement
{

namespace
{
//! @brief Replaces the first element whose key (via `key_fn`) matches `entry`'s, else appends
//!        `entry`. Returns whether an existing entry was replaced (false when newly appended).
template<typename T, typename KeyFn>
bool upsert_by_key( std::vector<T> &elements, const T &entry, KeyFn key_fn )
{
  const auto it = std::find_if( elements.begin(), elements.end(), [&]( const T &element ) {
    return key_fn( element ) == key_fn( entry );
  } );
  if ( it != elements.end() ) {
    *it = entry;
    return true;
  }
  elements.push_back( entry );
  return false;
}

//! @brief Erases the first element matching `key(element) == key`; returns whether one was removed.
template<typename T, typename KeyFn>
bool erase_by_key( std::vector<T> &elements, const std::string &key, KeyFn key_fn )
{
  const auto it = std::find_if( elements.begin(), elements.end(),
                                [&]( const T &element ) { return key_fn( element ) == key; } );
  if ( it == elements.end() )
    return false;
  elements.erase( it );
  return true;
}
} // namespace

Announcer::Announcer( rclcpp::Node &node, RobotAnnouncement initial )
    : node_( node ), announcement_( std::move( initial ) )
{
  const rclcpp::QoS announcement_qos = latched_qos();
  announcement_publisher_ =
      node_.create_publisher<RobotAnnouncement>( "robot_announcement", announcement_qos );
  // Gate on the node's effective namespace, not announcement_.ros_namespace: the robot_namespace
  // parameter that fills the message field is independent of where the node is launched.
  if ( node_.get_effective_namespace() != "/" ) {
    global_announcement_publisher_ =
        node_.create_publisher<RobotAnnouncement>( "/robot_announcement", announcement_qos );
  }

  publish( stamped_announcement() );

  using std::placeholders::_1;
  using std::placeholders::_2;
  add_sensor_srv_ = node_.create_service<AddSensor>(
      "add_sensor", std::bind( &Announcer::handle_add_sensor, this, _1, _2 ) );
  remove_sensor_srv_ = node_.create_service<RemoveSensor>(
      "remove_sensor", std::bind( &Announcer::handle_remove_sensor, this, _1, _2 ) );
  add_visualization_srv_ = node_.create_service<AddVisualization>(
      "add_visualization", std::bind( &Announcer::handle_add_visualization, this, _1, _2 ) );
  remove_visualization_srv_ = node_.create_service<RemoveVisualization>(
      "remove_visualization", std::bind( &Announcer::handle_remove_visualization, this, _1, _2 ) );
}

RobotAnnouncement Announcer::stamped_announcement()
{
  RobotAnnouncement copy = announcement_;
  copy.header.stamp = node_.now();
  return copy;
}

void Announcer::publish( const RobotAnnouncement &announcement )
{
  announcement_publisher_->publish( announcement );
  if ( global_announcement_publisher_ )
    global_announcement_publisher_->publish( announcement );
}

template<typename Mutate>
bool Announcer::mutate_and_republish( Mutate &&mutate )
{
  RobotAnnouncement to_publish;
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    if ( !mutate() )
      return false;
    to_publish = stamped_announcement();
  }
  publish( to_publish );
  return true;
}

void Announcer::handle_add_sensor( const std::shared_ptr<AddSensor::Request> request,
                                   std::shared_ptr<AddSensor::Response> response )
{
  Sensor sensor = request->sensor;
  if ( const std::optional<std::string> reason = normalize_sensor( sensor ) ) {
    response->success = false;
    response->message = "Rejected sensor: " + *reason;
    return;
  }

  bool replaced = false;
  mutate_and_republish( [&] {
    replaced = upsert_by_key( announcement_.sensors, sensor,
                              []( const Sensor &s ) { return s.id; } );
    return true;
  } );

  response->success = true;
  response->message = ( replaced ? "Replaced sensor '" : "Added sensor '" ) + sensor.id + "'.";
}

void Announcer::handle_remove_sensor( const std::shared_ptr<RemoveSensor::Request> request,
                                      std::shared_ptr<RemoveSensor::Response> response )
{
  if ( !mutate_and_republish( [&] {
         return erase_by_key( announcement_.sensors, request->id,
                              []( const Sensor &s ) { return s.id; } );
       } ) ) {
    response->success = false;
    response->message = "No sensor with id '" + request->id + "'.";
    return;
  }

  response->success = true;
  response->message = "Removed sensor '" + request->id + "'.";
}

void Announcer::handle_add_visualization( const std::shared_ptr<AddVisualization::Request> request,
                                          std::shared_ptr<AddVisualization::Response> response )
{
  Visualization visualization = request->visualization;
  if ( const std::optional<std::string> reason = normalize_visualization( visualization ) ) {
    response->success = false;
    response->message = "Rejected visualization: " + *reason;
    return;
  }

  bool replaced = false;
  mutate_and_republish( [&] {
    replaced = upsert_by_key( announcement_.visualizations, visualization,
                              []( const Visualization &v ) { return v.name; } );
    return true;
  } );

  response->success = true;
  response->message =
      ( replaced ? "Replaced visualization '" : "Added visualization '" ) + visualization.name + "'.";
}

void Announcer::handle_remove_visualization(
    const std::shared_ptr<RemoveVisualization::Request> request,
    std::shared_ptr<RemoveVisualization::Response> response )
{
  if ( !mutate_and_republish( [&] {
         return erase_by_key( announcement_.visualizations, request->name,
                              []( const Visualization &v ) { return v.name; } );
       } ) ) {
    response->success = false;
    response->message = "No visualization with name '" + request->name + "'.";
    return;
  }

  response->success = true;
  response->message = "Removed visualization '" + request->name + "'.";
}

} // namespace hector_multi_robot_announcement
