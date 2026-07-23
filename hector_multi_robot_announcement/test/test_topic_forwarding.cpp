#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/messages/array_message.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>

#include "hector_multi_robot_announcement/topic_forwarding.hpp"

using hector_multi_robot_announcement::ForwardedTopicConfig;
using hector_multi_robot_announcement::parse_forwarded_topics;
using hector_multi_robot_announcement::prefix_frame_ids;
using hector_multi_robot_announcement::TopicForwarder;
using ros_babel_fish::BabelFish;
using ros_babel_fish::CompoundArrayMessage;

// --- parse_forwarded_topics --------------------------------------------------

TEST( ParseForwardedTopics, ExtractsTopicAndOptionalType )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["topic_forwarding.topics.grid_map.topic"] = rclcpp::ParameterValue( std::string( "map" ) );
  overrides["topic_forwarding.topics.planned_path.topic"] =
      rclcpp::ParameterValue( std::string( "move_base/path" ) );
  overrides["topic_forwarding.topics.planned_path.message_type"] =
      rclcpp::ParameterValue( std::string( "nav_msgs/msg/Path" ) );

  const auto topics = parse_forwarded_topics( overrides, rclcpp::get_logger( "test" ) );

  ASSERT_EQ( topics.size(), 2u );
  // Ids come out sorted: grid_map before planned_path.
  EXPECT_EQ( topics[0].id, "grid_map" );
  EXPECT_EQ( topics[0].topic, "map" );
  EXPECT_TRUE( topics[0].message_type.empty() ); // auto-discover
  EXPECT_EQ( topics[1].id, "planned_path" );
  EXPECT_EQ( topics[1].topic, "move_base/path" );
  EXPECT_EQ( topics[1].message_type, "nav_msgs/msg/Path" );
}

TEST( ParseForwardedTopics, SkipsEntryWithoutTopic )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  // Only a message_type, no topic -> skipped.
  overrides["topic_forwarding.topics.typed_only.message_type"] =
      rclcpp::ParameterValue( std::string( "nav_msgs/msg/Path" ) );
  overrides["topic_forwarding.topics.valid.topic"] =
      rclcpp::ParameterValue( std::string( "scan" ) );

  const auto topics = parse_forwarded_topics( overrides, rclcpp::get_logger( "test" ) );

  ASSERT_EQ( topics.size(), 1u );
  EXPECT_EQ( topics[0].id, "valid" );
}

TEST( ParseForwardedTopics, IgnoresUnknownFieldsAndUnrelatedKeys )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["robot_id"] = rclcpp::ParameterValue( std::string( "robot1" ) );
  overrides["topic_forwarding.subnamespace"] = rclcpp::ParameterValue( std::string( "world" ) );
  overrides["topic_forwarding.topics.foo.topic"] = rclcpp::ParameterValue( std::string( "bar" ) );
  overrides["topic_forwarding.topics.foo.unknown"] = rclcpp::ParameterValue( true );

  const auto topics = parse_forwarded_topics( overrides, rclcpp::get_logger( "test" ) );

  ASSERT_EQ( topics.size(), 1u );
  EXPECT_EQ( topics[0].id, "foo" );
  EXPECT_EQ( topics[0].topic, "bar" );
}

// --- prefix_frame_ids --------------------------------------------------------

TEST( PrefixFrameIds, RewritesNestedPathFrames )
{
  BabelFish fish;
  auto path = fish.create_message( "nav_msgs/msg/Path" );
  path["header"]["frame_id"] = "odom";
  auto &poses = path["poses"].as<CompoundArrayMessage>();
  poses.appendEmpty()["header"]["frame_id"] = "odom";  // poses[0]
  poses.appendEmpty()["header"]["frame_id"] = "world"; // poses[1], a global frame
  poses.appendEmpty()["header"]["frame_id"] = "";      // poses[2], empty stays empty

  prefix_frame_ids( path, "robot1/", { "world" } );

  EXPECT_EQ( path["header"]["frame_id"].value<std::string>(), "robot1/odom" );
  EXPECT_EQ( poses[0]["header"]["frame_id"].value<std::string>(), "robot1/odom" );
  EXPECT_EQ( poses[1]["header"]["frame_id"].value<std::string>(), "world" );
  EXPECT_EQ( poses[2]["header"]["frame_id"].value<std::string>(), "" );
}

TEST( PrefixFrameIds, RewritesChildFrameId )
{
  BabelFish fish;
  auto odom = fish.create_message( "nav_msgs/msg/Odometry" );
  odom["header"]["frame_id"] = "odom";
  odom["child_frame_id"] = "base_link";

  prefix_frame_ids( odom, "robot1/", {} );

  EXPECT_EQ( odom["header"]["frame_id"].value<std::string>(), "robot1/odom" );
  EXPECT_EQ( odom["child_frame_id"].value<std::string>(), "robot1/base_link" );
}

// --- TopicForwarder activation -----------------------------------------------

namespace
{
std::shared_ptr<rclcpp::Node> make_node( const std::string &name,
                                         std::vector<rclcpp::Parameter> overrides )
{
  return std::make_shared<rclcpp::Node>(
      name, rclcpp::NodeOptions().parameter_overrides( std::move( overrides ) ) );
}
} // namespace

// active() reflects whether forwarding was wired up: namespaced AND at least one topic AND a
// non-empty subnamespace. Each case gets its own node since the subnamespace param is declared once.
TEST( TopicForwarderActivation, ActiveOnlyWhenNamespacedTopicsAndSubnamespace )
{
  auto active = make_node(
      "fwd_active",
      { rclcpp::Parameter( "topic_forwarding.topics.foo.topic", "move_base/pose" ) } );
  EXPECT_TRUE( TopicForwarder( *active, "/robot1" ).active() );

  auto root = make_node(
      "fwd_root", { rclcpp::Parameter( "topic_forwarding.topics.foo.topic", "move_base/pose" ) } );
  EXPECT_FALSE( TopicForwarder( *root, "/" ).active() ); // root namespace -> nothing to prefix

  auto no_topics = make_node( "fwd_no_topics", {} );
  EXPECT_FALSE( TopicForwarder( *no_topics, "/robot1" ).active() );

  auto empty_sub = make_node(
      "fwd_empty_sub", { rclcpp::Parameter( "topic_forwarding.topics.foo.topic", "move_base/pose" ),
                         rclcpp::Parameter( "topic_forwarding.subnamespace", "" ) } );
  EXPECT_FALSE( TopicForwarder( *empty_sub, "/robot1" ).active() );
}

// --- TopicForwarder integration ----------------------------------------------

// End-to-end: a PoseStamped published on <ns>/move_base/pose (type auto-discovered from the graph)
// is republished on <ns>/<subnamespace>/move_base/pose with its frame id prefixed. A short
// resolve_period removes the 1 Hz production floor so the test is not gated on the timer.
TEST( TopicForwarderIntegration, ForwardsWithFramePrefix )
{
  auto node = std::make_shared<rclcpp::Node>(
      "fwd_itest", "/robot1",
      rclcpp::NodeOptions()
          .use_global_arguments( false )
          .parameter_overrides(
              { rclcpp::Parameter( "topic_forwarding.topics.pose.topic", "move_base/pose" ) } ) );
  TopicForwarder forwarder( *node, "/robot1", std::chrono::milliseconds( 50 ) );
  ASSERT_TRUE( forwarder.active() );

  geometry_msgs::msg::PoseStamped received;
  bool got = false;
  auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robot1/world/move_base/pose", rclcpp::QoS( 10 ),
      [&]( geometry_msgs::msg::PoseStamped::ConstSharedPtr msg ) {
        received = *msg;
        got = true;
      } );
  auto pub =
      node->create_publisher<geometry_msgs::msg::PoseStamped>( "/robot1/move_base/pose", rclcpp::QoS( 10 ) );

  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "odom";

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  // Bound generously: covers resolve_period plus two-stage discovery (publisher -> forwarder,
  // forwarder -> our subscriber). Exits as soon as the forwarded message arrives.
  for ( int i = 0; i < 1000 && !got; ++i ) {
    pub->publish( msg );
    exec.spin_some();
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }

  ASSERT_TRUE( got ) << "pose was not forwarded";
  EXPECT_EQ( forwarder.resolved_count(), 1u );
  EXPECT_EQ( received.header.frame_id, "robot1/odom" );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
