#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "hector_multi_robot_announcement/announcer.hpp"
#include "hector_multi_robot_announcement/sensor_config.hpp"
#include "hector_multi_robot_announcement/simple_multi_robot_announcer.hpp"
#include "hector_multi_robot_announcement/utils.hpp"
#include "hector_multi_robot_announcement/visualization_config.hpp"

using hector_multi_robot_announcement::Announcer;
using hector_multi_robot_announcement::latched_qos;
using hector_multi_robot_announcement::normalize_sensor;
using hector_multi_robot_announcement::normalize_visualization;
using hector_multi_robot_announcement::SimpleMultiRobotAnnouncer;
using hector_multi_robot_msgs::msg::RobotAnnouncement;
using hector_multi_robot_msgs::msg::Sensor;
using hector_multi_robot_msgs::msg::Visualization;
using hector_multi_robot_msgs::srv::AddSensor;
using hector_multi_robot_msgs::srv::AddVisualization;
using hector_multi_robot_msgs::srv::RemoveSensor;
using hector_multi_robot_msgs::srv::RemoveVisualization;

namespace
{
Sensor make_sensor( const std::string &id, const std::string &topic, const std::string &name = "" )
{
  Sensor sensor;
  sensor.id = id;
  sensor.topic = topic;
  sensor.name = name;
  return sensor;
}

Visualization make_visualization( const std::string &name, const std::string &topic )
{
  Visualization visualization;
  visualization.name = name;
  visualization.topic = topic;
  return visualization;
}
} // namespace

// --- normalize_sensor --------------------------------------------------------

TEST( NormalizeSensor, RejectsEmptyId )
{
  Sensor sensor = make_sensor( "", "co2" );
  const auto reason = normalize_sensor( sensor );
  ASSERT_TRUE( reason.has_value() );
  EXPECT_EQ( *reason, "empty id." );
}

TEST( NormalizeSensor, RejectsEmptyTopic )
{
  Sensor sensor = make_sensor( "co2", "" );
  const auto reason = normalize_sensor( sensor );
  ASSERT_TRUE( reason.has_value() );
  EXPECT_EQ( *reason, "no 'topic' configured." );
}

TEST( NormalizeSensor, FallsBackNameToId )
{
  Sensor sensor = make_sensor( "co2", "co2" ); // empty name
  EXPECT_FALSE( normalize_sensor( sensor ).has_value() );
  EXPECT_EQ( sensor.name, "co2" );
}

TEST( NormalizeSensor, ValidKeepsExplicitName )
{
  Sensor sensor = make_sensor( "co2", "co2", "CO2" );
  EXPECT_FALSE( normalize_sensor( sensor ).has_value() );
  EXPECT_EQ( sensor.name, "CO2" );
}

// --- normalize_visualization -------------------------------------------------

TEST( NormalizeVisualization, RejectsEmptyName )
{
  Visualization visualization = make_visualization( "", "points" );
  const auto reason = normalize_visualization( visualization );
  ASSERT_TRUE( reason.has_value() );
  EXPECT_EQ( *reason, "empty name." );
}

TEST( NormalizeVisualization, RejectsEmptyTopic )
{
  Visualization visualization = make_visualization( "Cloud", "" );
  const auto reason = normalize_visualization( visualization );
  ASSERT_TRUE( reason.has_value() );
  EXPECT_EQ( *reason, "no 'topic' configured." );
}

TEST( NormalizeVisualization, ValidPasses )
{
  Visualization visualization = make_visualization( "Cloud", "points" );
  EXPECT_FALSE( normalize_visualization( visualization ).has_value() );
}

// --- Announcer (requires a live node) ----------------------------------------

namespace
{
std::shared_ptr<rclcpp::Node> make_node( const std::string &name, const std::string &ns = "/" )
{
  return std::make_shared<rclcpp::Node>( name, ns );
}

RobotAnnouncement make_announcement( const std::string &ros_namespace = "/robot1" )
{
  RobotAnnouncement announcement;
  announcement.id = "robot1";
  announcement.name = "Robot One";
  announcement.ros_namespace = ros_namespace;
  announcement.type = "tracked";
  return announcement;
}

//! @brief Spins until `done()` holds or ~10s elapses; returns whether `done()` held.
bool spin_until( rclcpp::executors::SingleThreadedExecutor &exec, const std::function<bool()> &done )
{
  for ( int i = 0; i < 1000 && !done(); ++i ) {
    exec.spin_some();
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }
  return done();
}

//! @brief Creates a client for `service_name`, waits for it to become ready, sends `request` and
//!        spins until the response arrives. Returns the response, or nullptr on timeout.
template<typename ServiceT>
typename ServiceT::Response::SharedPtr
call_service( rclcpp::executors::SingleThreadedExecutor &exec, rclcpp::Node &node,
              const std::string &service_name,
              const std::shared_ptr<typename ServiceT::Request> &request )
{
  auto client = node.create_client<ServiceT>( service_name );
  if ( !spin_until( exec, [&] { return client->service_is_ready(); } ) )
    return nullptr;
  auto future = client->async_send_request( request );
  if ( exec.spin_until_future_complete( future ) != rclcpp::FutureReturnCode::SUCCESS )
    return nullptr;
  return future.get();
}
} // namespace

TEST( AnnouncerService, AddSensorAddsNewSensor )
{
  auto node = make_node( "announcer_add_sensor", "/robot1" );
  Announcer announcer( *node, make_announcement() );

  std::optional<RobotAnnouncement> got;
  auto sub = node->create_subscription<RobotAnnouncement>(
      "robot_announcement", latched_qos(),
      [&got]( RobotAnnouncement::ConstSharedPtr msg ) { got = *msg; } );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  ASSERT_TRUE( spin_until( exec, [&] { return got.has_value(); } ) ) << "initial announcement not received";
  ASSERT_TRUE( got->sensors.empty() );

  auto request = std::make_shared<AddSensor::Request>();
  request->sensor = make_sensor( "co2", "co2", "CO2" );
  auto response = call_service<AddSensor>( exec, *node, "add_sensor", request );
  ASSERT_NE( response, nullptr );
  EXPECT_TRUE( response->success );

  ASSERT_TRUE( spin_until( exec, [&] { return got->sensors.size() == 1; } ) )
      << "republished announcement with the sensor not received";
  EXPECT_EQ( got->sensors.front().id, "co2" );
  EXPECT_EQ( got->sensors.front().name, "CO2" );
}

TEST( AnnouncerService, AddSensorReplacesExistingById )
{
  auto node = make_node( "announcer_replace_sensor", "/robot1" );
  auto initial = make_announcement();
  initial.sensors.push_back( make_sensor( "co2", "co2", "CO2" ) );
  Announcer announcer( *node, initial );

  std::optional<RobotAnnouncement> got;
  auto sub = node->create_subscription<RobotAnnouncement>(
      "robot_announcement", latched_qos(),
      [&got]( RobotAnnouncement::ConstSharedPtr msg ) { got = *msg; } );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  ASSERT_TRUE( spin_until( exec, [&] { return got && got->sensors.size() == 1; } ) );

  auto request = std::make_shared<AddSensor::Request>();
  request->sensor = make_sensor( "co2", "co2_v2", "CO2 v2" ); // same id, changed fields
  auto response = call_service<AddSensor>( exec, *node, "add_sensor", request );
  ASSERT_NE( response, nullptr );
  EXPECT_TRUE( response->success );

  ASSERT_TRUE( spin_until( exec, [&] {
    return got->sensors.size() == 1 && got->sensors.front().topic == "co2_v2";
  } ) ) << "sensor was not replaced in place";
  EXPECT_EQ( got->sensors.front().name, "CO2 v2" );
}

TEST( AnnouncerService, AddSensorRejectsInvalidSensor )
{
  auto node = make_node( "announcer_reject_sensor", "/robot1" );
  Announcer announcer( *node, make_announcement() );

  std::optional<RobotAnnouncement> got;
  auto sub = node->create_subscription<RobotAnnouncement>(
      "robot_announcement", latched_qos(),
      [&got]( RobotAnnouncement::ConstSharedPtr msg ) { got = *msg; } );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  ASSERT_TRUE( spin_until( exec, [&] { return got.has_value(); } ) );

  auto request = std::make_shared<AddSensor::Request>();
  request->sensor = make_sensor( "co2", "" ); // empty topic
  auto response = call_service<AddSensor>( exec, *node, "add_sensor", request );
  ASSERT_NE( response, nullptr );
  EXPECT_FALSE( response->success );

  // No republish is expected; the latched announcement stays empty.
  for ( int i = 0; i < 10; ++i ) {
    exec.spin_some();
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }
  EXPECT_TRUE( got->sensors.empty() );
}

TEST( AnnouncerService, RemoveSensorErasesById )
{
  auto node = make_node( "announcer_remove_sensor", "/robot1" );
  auto initial = make_announcement();
  initial.sensors.push_back( make_sensor( "co2", "co2", "CO2" ) );
  Announcer announcer( *node, initial );

  std::optional<RobotAnnouncement> got;
  auto sub = node->create_subscription<RobotAnnouncement>(
      "robot_announcement", latched_qos(),
      [&got]( RobotAnnouncement::ConstSharedPtr msg ) { got = *msg; } );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  ASSERT_TRUE( spin_until( exec, [&] { return got && got->sensors.size() == 1; } ) );

  auto request = std::make_shared<RemoveSensor::Request>();
  request->id = "co2";
  auto response = call_service<RemoveSensor>( exec, *node, "remove_sensor", request );
  ASSERT_NE( response, nullptr );
  EXPECT_TRUE( response->success );

  ASSERT_TRUE( spin_until( exec, [&] { return got->sensors.empty(); } ) )
      << "republished announcement without the sensor not received";
}

TEST( AnnouncerService, RemoveMissingSensorFails )
{
  auto node = make_node( "announcer_remove_missing_sensor", "/robot1" );
  Announcer announcer( *node, make_announcement() );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );

  auto request = std::make_shared<RemoveSensor::Request>();
  request->id = "does_not_exist";
  auto response = call_service<RemoveSensor>( exec, *node, "remove_sensor", request );
  ASSERT_NE( response, nullptr );
  EXPECT_FALSE( response->success );
}

TEST( AnnouncerService, AddReplaceRemoveVisualization )
{
  auto node = make_node( "announcer_visualization", "/robot1" );
  Announcer announcer( *node, make_announcement() );

  std::optional<RobotAnnouncement> got;
  auto sub = node->create_subscription<RobotAnnouncement>(
      "robot_announcement", latched_qos(),
      [&got]( RobotAnnouncement::ConstSharedPtr msg ) { got = *msg; } );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  ASSERT_TRUE( spin_until( exec, [&] { return got.has_value(); } ) );
  ASSERT_TRUE( got->visualizations.empty() );

  // Add.
  auto add = std::make_shared<AddVisualization::Request>();
  add->visualization = make_visualization( "Cloud", "points" );
  auto add_response = call_service<AddVisualization>( exec, *node, "add_visualization", add );
  ASSERT_NE( add_response, nullptr );
  EXPECT_TRUE( add_response->success );
  ASSERT_TRUE( spin_until( exec, [&] { return got->visualizations.size() == 1; } ) );
  EXPECT_EQ( got->visualizations.front().name, "Cloud" );

  // Replace by name.
  auto replace = std::make_shared<AddVisualization::Request>();
  replace->visualization = make_visualization( "Cloud", "points_v2" );
  auto replace_response = call_service<AddVisualization>( exec, *node, "add_visualization", replace );
  ASSERT_NE( replace_response, nullptr );
  EXPECT_TRUE( replace_response->success );
  ASSERT_TRUE( spin_until( exec, [&] {
    return got->visualizations.size() == 1 && got->visualizations.front().topic == "points_v2";
  } ) ) << "visualization was not replaced in place";

  // Remove.
  auto remove = std::make_shared<RemoveVisualization::Request>();
  remove->name = "Cloud";
  auto remove_response = call_service<RemoveVisualization>( exec, *node, "remove_visualization", remove );
  ASSERT_NE( remove_response, nullptr );
  EXPECT_TRUE( remove_response->success );
  ASSERT_TRUE( spin_until( exec, [&] { return got->visualizations.empty(); } ) );

  // Removing again fails.
  auto remove_again = std::make_shared<RemoveVisualization::Request>();
  remove_again->name = "Cloud";
  auto remove_again_response =
      call_service<RemoveVisualization>( exec, *node, "remove_visualization", remove_again );
  ASSERT_NE( remove_again_response, nullptr );
  EXPECT_FALSE( remove_again_response->success );
}

TEST( AnnouncerService, AddVisualizationRejectsEmptyName )
{
  auto node = make_node( "announcer_reject_visualization", "/robot1" );
  Announcer announcer( *node, make_announcement() );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );

  auto request = std::make_shared<AddVisualization::Request>();
  request->visualization = make_visualization( "", "points" ); // empty name
  auto response = call_service<AddVisualization>( exec, *node, "add_visualization", request );
  ASSERT_NE( response, nullptr );
  EXPECT_FALSE( response->success );
}

// A namespaced Announcer republishes on both the namespaced and the global topic; the gate keys off
// the node's effective namespace.
TEST( AnnouncerGlobalPublisher, NamespacedRepublishesOnBoth )
{
  auto node = make_node( "announcer_ns_global", "/robot1" );
  Announcer announcer( *node, make_announcement( "/robot1" ) );

  std::optional<RobotAnnouncement> local;
  std::optional<RobotAnnouncement> global;
  auto local_sub = node->create_subscription<RobotAnnouncement>(
      "robot_announcement", latched_qos(),
      [&local]( RobotAnnouncement::ConstSharedPtr msg ) { local = *msg; } );
  auto global_sub = node->create_subscription<RobotAnnouncement>(
      "/robot_announcement", latched_qos(),
      [&global]( RobotAnnouncement::ConstSharedPtr msg ) { global = *msg; } );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  ASSERT_TRUE( spin_until( exec, [&] { return local.has_value() && global.has_value(); } ) )
      << "initial announcement not received on both topics";

  auto request = std::make_shared<AddSensor::Request>();
  request->sensor = make_sensor( "co2", "co2", "CO2" );
  auto response = call_service<AddSensor>( exec, *node, "add_sensor", request );
  ASSERT_NE( response, nullptr );
  EXPECT_TRUE( response->success );

  ASSERT_TRUE( spin_until( exec, [&] {
    return local->sensors.size() == 1 && global->sensors.size() == 1;
  } ) ) << "republish not observed on both topics";
}

// A root-namespace Announcer must not create the global publisher: "robot_announcement" already
// resolves to "/robot_announcement", so this node owns exactly one publisher on it (a second would be
// the wrongly-created global one). Count only this node's own endpoints; other tests in the same
// process leave global publishers lingering in the graph.
TEST( AnnouncerGlobalPublisher, RootNamespaceHasNoGlobalPublisher )
{
  auto node = make_node( "announcer_root_only" );
  Announcer announcer( *node, make_announcement( "/" ) );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  const auto own_publishers = [&] {
    size_t count = 0;
    for ( const auto &info : node->get_publishers_info_by_topic( "/robot_announcement" ) )
      if ( info.node_name() == node->get_name() )
        ++count;
    return count;
  };
  ASSERT_TRUE( spin_until( exec, [&] { return own_publishers() >= 1; } ) )
      << "the node's own announcement publisher never appeared in the graph";
  EXPECT_EQ( own_publishers(), 1u );
}

// End-to-end through the node: parse overrides -> build message -> hand to Announcer -> publish on
// both the namespaced and global topics. This is the only test that constructs the node.
TEST( SimpleAnnouncerNode, PublishesInitialAnnouncementOnBoth )
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      { rclcpp::Parameter( "robot_id", std::string( "robot1" ) ),
        rclcpp::Parameter( "robot_name", std::string( "Robot One" ) ),
        rclcpp::Parameter( "robot_namespace", std::string( "/robot1" ) ),
        rclcpp::Parameter( "type", std::string( "tracked" ) ),
        rclcpp::Parameter( "sensors.co2.topic", std::string( "co2" ) ),
        rclcpp::Parameter( "sensors.co2.name", std::string( "CO2" ) ),
        rclcpp::Parameter( "visualizations.cloud.topic", std::string( "points" ) ),
        rclcpp::Parameter( "visualizations.cloud.name", std::string( "Cloud" ) ) } );
  options.arguments( { "--ros-args", "-r", "__ns:=/robot1" } );
  auto announcer_node = std::make_shared<SimpleMultiRobotAnnouncer>( options );

  auto observer = make_node( "observer" );
  std::optional<RobotAnnouncement> local;
  std::optional<RobotAnnouncement> global;
  auto local_sub = observer->create_subscription<RobotAnnouncement>(
      "/robot1/robot_announcement", latched_qos(),
      [&local]( RobotAnnouncement::ConstSharedPtr msg ) { local = *msg; } );
  auto global_sub = observer->create_subscription<RobotAnnouncement>(
      "/robot_announcement", latched_qos(),
      [&global]( RobotAnnouncement::ConstSharedPtr msg ) { global = *msg; } );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( announcer_node );
  exec.add_node( observer );
  ASSERT_TRUE( spin_until( exec, [&] { return local.has_value() && global.has_value(); } ) )
      << "initial announcement not received on both topics";

  for ( const auto *received : { &local, &global } ) {
    const RobotAnnouncement &announcement = received->value();
    EXPECT_EQ( announcement.id, "robot1" );
    EXPECT_EQ( announcement.ros_namespace, "/robot1" );
    ASSERT_EQ( announcement.sensors.size(), 1u );
    EXPECT_EQ( announcement.sensors.front().id, "co2" );
    ASSERT_EQ( announcement.visualizations.size(), 1u );
    EXPECT_EQ( announcement.visualizations.front().name, "Cloud" );
  }
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
