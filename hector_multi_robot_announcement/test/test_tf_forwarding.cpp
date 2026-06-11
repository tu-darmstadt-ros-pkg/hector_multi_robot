#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "hector_multi_robot_announcement/tf_forwarding.hpp"

using hector_multi_robot_announcement::FrameRateLimiter;
using hector_multi_robot_announcement::numeric_value_as_double;
using hector_multi_robot_announcement::parse_frame_intervals;
using hector_multi_robot_announcement::prefix_frame_id;
using hector_multi_robot_announcement::prefix_transform;
using hector_multi_robot_announcement::TfForwarder;

namespace
{
rclcpp::Time t( double seconds ) { return rclcpp::Time( static_cast<int64_t>( seconds * 1e9 ) ); }
} // namespace

// --- parse_frame_intervals ---------------------------------------------------

// Regression: the prefix used to enumerate frame configs ended in '.', so rclcpp's
// list_parameters never matched anything and per-frame rates were silently ignored.
// parse_frame_intervals must recover the configured frames from the overrides map.
TEST( ParseFrameIntervals, ExtractsConfiguredFrames )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["tf_config.frame_configs.base_link.rate"] = rclcpp::ParameterValue( 10.0 );
  overrides["tf_config.frame_configs.odom.rate"] = rclcpp::ParameterValue( 5.0 );

  const auto intervals = parse_frame_intervals( overrides );

  ASSERT_EQ( intervals.size(), 2u );
  EXPECT_DOUBLE_EQ( intervals.at( "base_link" ).seconds(), 0.1 );
  EXPECT_DOUBLE_EQ( intervals.at( "odom" ).seconds(), 0.2 );
}

// Regression: a rate written as an integer (`rate: 5`) is typed PARAMETER_INTEGER and used to
// crash get_parameter(...).as_double(). It must now be accepted.
TEST( ParseFrameIntervals, AcceptsIntegerRate )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["tf_config.frame_configs.base_link.rate"] = rclcpp::ParameterValue( 5 );

  const auto intervals = parse_frame_intervals( overrides );

  ASSERT_EQ( intervals.size(), 1u );
  EXPECT_DOUBLE_EQ( intervals.at( "base_link" ).seconds(), 0.2 );
}

TEST( ParseFrameIntervals, ZeroRateMeansAlwaysForward )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["tf_config.frame_configs.map.rate"] = rclcpp::ParameterValue( 0.0 );

  const auto intervals = parse_frame_intervals( overrides );

  ASSERT_EQ( intervals.size(), 1u );
  EXPECT_EQ( intervals.at( "map" ).nanoseconds(), 0 );
}

TEST( ParseFrameIntervals, IgnoresUnrelatedAndMalformedKeys )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["robot_id"] = rclcpp::ParameterValue( std::string( "robot1" ) );
  overrides["tf_config.max_rate"] = rclcpp::ParameterValue( 30.0 );
  overrides["tf_config.frame_configs.base_link.enabled"] = rclcpp::ParameterValue( true );
  overrides["tf_config.frame_configs..rate"] =
      rclcpp::ParameterValue( 5.0 ); // empty frame name -> rejected by the empty-frame guard
  overrides["tf_config.frame_configs.lidar.rate"] =
      rclcpp::ParameterValue( std::string( "fast" ) ); // non-numeric -> always forward

  const auto intervals = parse_frame_intervals( overrides );

  ASSERT_EQ( intervals.size(), 1u );
  ASSERT_TRUE( intervals.count( "lidar" ) );
  EXPECT_EQ( intervals.at( "lidar" ).nanoseconds(), 0 );
}

// --- numeric_value_as_double -------------------------------------------------

TEST( NumericValueAsDouble, HandlesDoubleIntAndRejectsOthers )
{
  EXPECT_EQ( numeric_value_as_double( rclcpp::ParameterValue( 2.5 ) ), 2.5 );
  EXPECT_EQ( numeric_value_as_double( rclcpp::ParameterValue( 7 ) ), 7.0 );
  EXPECT_FALSE( numeric_value_as_double( rclcpp::ParameterValue( std::string( "x" ) ) ) );
  EXPECT_FALSE( numeric_value_as_double( rclcpp::ParameterValue( true ) ) );
}

// --- prefix_frame_id / prefix_transform --------------------------------------

TEST( PrefixFrameId, PrefixesNormalFramesOnly )
{
  const std::unordered_set<std::string> global{ "world", "map" };
  EXPECT_EQ( prefix_frame_id( "base_link", "robot1/", global ), "robot1/base_link" );
  EXPECT_EQ( prefix_frame_id( "world", "robot1/", global ), "world" );  // global, untouched
  EXPECT_EQ( prefix_frame_id( "", "robot1/", global ), "" );            // empty, untouched
}

TEST( PrefixTransform, PrefixesParentAndChild )
{
  const std::unordered_set<std::string> global{ "world" };
  geometry_msgs::msg::TransformStamped in;
  in.header.frame_id = "odom";
  in.child_frame_id = "base_link";

  const auto out = prefix_transform( in, "robot1/", global );
  EXPECT_EQ( out.header.frame_id, "robot1/odom" );
  EXPECT_EQ( out.child_frame_id, "robot1/base_link" );

  in.header.frame_id = "world"; // global parent stays, child still prefixed
  const auto out2 = prefix_transform( in, "robot1/", global );
  EXPECT_EQ( out2.header.frame_id, "world" );
  EXPECT_EQ( out2.child_frame_id, "robot1/base_link" );
}

// --- FrameRateLimiter --------------------------------------------------------

TEST( FrameRateLimiterTest, ThrottlesAtDefaultInterval )
{
  FrameRateLimiter limiter( rclcpp::Duration::from_seconds( 1.0 ), {} ); // 1 Hz default

  EXPECT_TRUE( limiter.allow( "base_link", t( 0.0 ) ) );  // first always passes
  EXPECT_FALSE( limiter.allow( "base_link", t( 0.5 ) ) ); // within interval
  EXPECT_TRUE( limiter.allow( "base_link", t( 1.0 ) ) );  // interval elapsed
}

TEST( FrameRateLimiterTest, PerFrameIntervalOverridesDefault )
{
  std::unordered_map<std::string, rclcpp::Duration> per_frame;
  per_frame.emplace( "fast", rclcpp::Duration::from_seconds( 0.1 ) ); // 10 Hz
  FrameRateLimiter limiter( rclcpp::Duration::from_seconds( 1.0 ), std::move( per_frame ) );

  EXPECT_TRUE( limiter.allow( "fast", t( 0.0 ) ) );
  EXPECT_TRUE( limiter.allow( "fast", t( 0.2 ) ) ); // its own 10 Hz allows it

  // An unconfigured frame falls back to the 1 Hz default and is blocked at 0.2 s.
  EXPECT_TRUE( limiter.allow( "slow", t( 0.0 ) ) );
  EXPECT_FALSE( limiter.allow( "slow", t( 0.2 ) ) );
}

TEST( FrameRateLimiterTest, SeparateFramesTrackedIndependently )
{
  FrameRateLimiter limiter( rclcpp::Duration::from_seconds( 1.0 ), {} );
  EXPECT_TRUE( limiter.allow( "a", t( 0.0 ) ) );
  EXPECT_TRUE( limiter.allow( "b", t( 0.0 ) ) ); // different frame, own bucket
  EXPECT_FALSE( limiter.allow( "a", t( 0.5 ) ) );
}

// Regression: always-forward frames (zero interval) must not accumulate bookkeeping state,
// otherwise the last-publish-time map grows unbounded for dynamically-named frames.
TEST( FrameRateLimiterTest, ZeroIntervalAlwaysForwardsWithoutTracking )
{
  std::unordered_map<std::string, rclcpp::Duration> per_frame;
  per_frame.emplace( "always", rclcpp::Duration::from_nanoseconds( 0 ) );
  FrameRateLimiter limiter( rclcpp::Duration::from_nanoseconds( 0 ), std::move( per_frame ) );

  for ( int i = 0; i < 5; ++i ) {
    EXPECT_TRUE( limiter.allow( "always", t( i ) ) );
    EXPECT_TRUE( limiter.allow( "unconfigured", t( i ) ) ); // falls back to zero default
  }
  EXPECT_EQ( limiter.tracked_frame_count(), 0u );
}

// Regression: a backwards time jump (sim reset / looping bag) must not block a frame until time
// re-passes the stale timestamp; the next message re-anchors and forwards.
TEST( FrameRateLimiterTest, BackwardTimeJumpReanchors )
{
  FrameRateLimiter limiter( rclcpp::Duration::from_seconds( 1.0 ), {} );

  EXPECT_TRUE( limiter.allow( "base_link", t( 10.0 ) ) );
  EXPECT_FALSE( limiter.allow( "base_link", t( 10.5 ) ) ); // within interval
  EXPECT_TRUE( limiter.allow( "base_link", t( 0.0 ) ) );   // clock jumped back -> forward
  EXPECT_FALSE( limiter.allow( "base_link", t( 0.5 ) ) );  // throttles against the new anchor
}

// Once the tracked set hits the limit, entries whose window has elapsed are evicted (dropping
// them is behaviourally free) so transient, uniquely named frames cannot grow the map forever.
TEST( FrameRateLimiterTest, EvictsElapsedEntriesAtLimit )
{
  FrameRateLimiter limiter( rclcpp::Duration::from_seconds( 1.0 ), {}, /*max_tracked_frames=*/4 );
  for ( int i = 0; i < 4; ++i )
    EXPECT_TRUE( limiter.allow( "frame" + std::to_string( i ), t( 0.0 ) ) );
  ASSERT_EQ( limiter.tracked_frame_count(), 4u );

  // A new frame at t=2s triggers a prune; all four prior entries are >1s stale and get dropped.
  EXPECT_TRUE( limiter.allow( "frame_new", t( 2.0 ) ) );
  EXPECT_EQ( limiter.tracked_frame_count(), 1u );
  EXPECT_EQ( limiter.tracked_frame_limit(), 4u ); // limit unchanged, prune did the work
}

// If nothing can be evicted (every tracked frame is still within its window) the limit is raised
// instead, so the set sits well below it again rather than pruning on every new frame.
TEST( FrameRateLimiterTest, RaisesLimitWhenNothingEvictable )
{
  FrameRateLimiter limiter( rclcpp::Duration::from_seconds( 1.0 ), {}, /*max_tracked_frames=*/4 );
  for ( int i = 0; i < 4; ++i )
    EXPECT_TRUE( limiter.allow( "frame" + std::to_string( i ), t( 0.0 ) ) );

  // New frame at t=0.1s: all prior entries are still inside their 1s window, so none are evicted.
  EXPECT_TRUE( limiter.allow( "frame_new", t( 0.1 ) ) );
  EXPECT_EQ( limiter.tracked_frame_count(), 5u );
  EXPECT_EQ( limiter.tracked_frame_limit(), 8u ); // raised to twice the active set
}

// --- TfForwarder (requires a live node) --------------------------------------

namespace
{
// Builds a node whose parameter overrides are applied before TfForwarder declares them.
std::shared_ptr<rclcpp::Node> make_node( const std::string &name,
                                         std::vector<rclcpp::Parameter> overrides )
{
  return std::make_shared<rclcpp::Node>(
      name, rclcpp::NodeOptions().parameter_overrides( std::move( overrides ) ) );
}
} // namespace

// active() must reflect whether forwarding was actually wired up: only when enabled AND the
// robot namespace is non-root. Each case gets its own node since TfForwarder declares the
// tf_config parameters and a second instance on the same node would fail to re-declare them.
TEST( TfForwarderActivation, ActiveOnlyWhenEnabledAndNamespaced )
{
  auto enabled = make_node( "fwd_enabled", { rclcpp::Parameter( "enable_tf_forwarding", true ) } );
  EXPECT_TRUE( TfForwarder( *enabled, "/robot1" ).active() );

  auto disabled = make_node( "fwd_disabled", {} ); // enable_tf_forwarding defaults to false
  EXPECT_FALSE( TfForwarder( *disabled, "/robot1" ).active() );

  auto root = make_node( "fwd_root", { rclcpp::Parameter( "enable_tf_forwarding", true ) } );
  EXPECT_FALSE( TfForwarder( *root, "/" ).active() ); // root namespace -> nothing to prefix
}

// Regression: a trailing slash in robot_namespace produced the invalid topic "/robot1//tf", whose
// repeated slash made create_subscription throw and aborted node construction. It must normalize.
TEST( TfForwarderActivation, ToleratesTrailingSlashNamespace )
{
  auto node = make_node( "fwd_trailing", { rclcpp::Parameter( "enable_tf_forwarding", true ) } );
  EXPECT_NO_THROW( { EXPECT_TRUE( TfForwarder( *node, "/robot1/" ).active() ); } );
}

// Regression: YAML `max_rate: 30` is an integer parameter, while the default is a double.
// The forwarder must accept it the same way per-frame rates accept integer literals.
TEST( TfForwarderActivation, AcceptsIntegerMaxRateOverride )
{
  auto node = make_node( "fwd_integer_max_rate",
                         { rclcpp::Parameter( "enable_tf_forwarding", true ),
                           rclcpp::Parameter( "tf_config.max_rate", 30 ) } );
  EXPECT_NO_THROW( { EXPECT_TRUE( TfForwarder( *node, "/robot1" ).active() ); } );
}

// End-to-end: a robot-local transform on <ns>/tf is re-broadcast on the global /tf with both
// frame ids prefixed by the namespace. Exercises the callback -> rate-limit -> prefix -> publish
// wiring AND the remap bypass: the node runs in namespace /robot1 with a hostile multi-robot
// `/tf:=tf` remap (which, in a namespace, would divert the broadcaster's global /tf to /robot1/tf)
// preceded by the identity remaps that must out-rank it (mirroring bypass_global_tf_remap in the
// announcer). If the bypass regresses, the forwarded transforms land on /robot1/tf instead and the
// /tf subscriber below receives nothing.
TEST( TfForwarderIntegration, ForwardsDynamicTransformsWithPrefix )
{
  auto options =
      rclcpp::NodeOptions()
          .use_global_arguments( false ) // isolate from the test runner's own command line
          .parameter_overrides( { rclcpp::Parameter( "enable_tf_forwarding", true ) } )
          // Identity remaps for /tf and /tf_static precede the hostile `/tf:=tf` remap so the
          // global TF topics survive it (mirrors bypass_global_tf_remap in the announcer).
          .arguments( { "--ros-args", "--remap", "/tf:=/tf", "--remap", "/tf_static:=/tf_static",
                        "--remap", "/tf:=tf", "--remap", "/tf_static:=tf_static" } );
  auto node = std::make_shared<rclcpp::Node>( "fwd_itest", "/robot1", options );
  TfForwarder forwarder( *node, "/robot1" );
  ASSERT_TRUE( forwarder.active() );

  std::vector<geometry_msgs::msg::TransformStamped> received;
  auto sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS( 10 ), [&received]( tf2_msgs::msg::TFMessage::ConstSharedPtr msg ) {
        received.insert( received.end(), msg->transforms.begin(), msg->transforms.end() );
      } );
  auto pub = node->create_publisher<tf2_msgs::msg::TFMessage>( "/robot1/tf", rclcpp::QoS( 10 ) );

  tf2_msgs::msg::TFMessage msg;
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_link";
  msg.transforms.push_back( tf );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  // Wait for discovery before relying on delivery rather than guessing a fixed wall-clock budget:
  // the forwarder's /robot1/tf subscription must match our publisher and our /tf subscription must
  // match the forwarder's broadcaster. The bound is a generous safety cap; the loop exits as soon
  // as both ends are connected.
  auto connected = [&] {
    return pub->get_subscription_count() > 0 && sub->get_publisher_count() > 0;
  };
  for ( int i = 0; i < 1000 && !connected(); ++i ) {
    exec.spin_some();
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }
  // Discovery done; publish until a transform is forwarded (usually the first iteration).
  for ( int i = 0; i < 1000 && received.empty(); ++i ) {
    pub->publish( msg );
    exec.spin_some();
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }

  ASSERT_FALSE( received.empty() ) << "transform was not forwarded to /tf";
  EXPECT_EQ( received.front().header.frame_id, "robot1/odom" );
  EXPECT_EQ( received.front().child_frame_id, "robot1/base_link" );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
