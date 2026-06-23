#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <thread>

#include "hector_multi_robot_announcement/status_reporter.hpp"
#include "hector_multi_robot_announcement/utils.hpp"

using hector_multi_robot_announcement::aggregate_battery_level;
using hector_multi_robot_announcement::BatteryAggregation;
using hector_multi_robot_announcement::latched_qos;
using hector_multi_robot_announcement::parse_battery_topics;
using hector_multi_robot_announcement::StatusReporter;
using hector_multi_robot_msgs::msg::Heartbeat;
using hector_multi_robot_msgs::msg::RobotStatus;
using hector_multi_robot_msgs::srv::SetStatus;
using sensor_msgs::msg::BatteryState;

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

//! Builds a BatteryState; charge/capacity/design_capacity default to NaN (unknown) for
//! percentage-only tests.
BatteryState battery( float percentage, float charge = kNaN, float capacity = kNaN,
                      float design_capacity = kNaN )
{
  BatteryState b;
  b.percentage = percentage;
  b.charge = charge;
  b.capacity = capacity;
  b.design_capacity = design_capacity;
  return b;
}

int level( const std::vector<BatteryState> &batteries, BatteryAggregation strategy )
{
  return static_cast<int>( aggregate_battery_level( batteries, strategy ).level );
}
} // namespace

// --- parse_battery_topics ----------------------------------------------------

TEST( ParseBatteryTopics, ExtractsLabelTopicPairs )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["status.battery.state_topics.left"] = rclcpp::ParameterValue( std::string( "battery_left" ) );
  overrides["status.battery.state_topics.right"] = rclcpp::ParameterValue( std::string( "battery_right" ) );

  const auto topics = parse_battery_topics( overrides );

  ASSERT_EQ( topics.size(), 2u );
  EXPECT_EQ( topics.at( "left" ), "battery_left" );
  EXPECT_EQ( topics.at( "right" ), "battery_right" );
}

TEST( ParseBatteryTopics, IgnoresUnrelatedNonStringAndEmpty )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["robot_id"] = rclcpp::ParameterValue( std::string( "robot1" ) );
  overrides["status.status_frequency"] = rclcpp::ParameterValue( 1.0 );
  overrides["status.battery.state_topics."] =
      rclcpp::ParameterValue( std::string( "x" ) ); // empty label -> rejected
  overrides["status.battery.state_topics.bad"] = rclcpp::ParameterValue( 5 ); // non-string -> rejected
  overrides["status.battery.state_topics.empty"] =
      rclcpp::ParameterValue( std::string( "" ) ); // empty topic -> rejected
  overrides["status.battery.state_topics.left"] = rclcpp::ParameterValue( std::string( "battery_left" ) );

  const auto topics = parse_battery_topics( overrides );

  ASSERT_EQ( topics.size(), 1u );
  EXPECT_EQ( topics.at( "left" ), "battery_left" );
}

TEST( ParseBatteryTopics, KeepsNestedDottedLabel )
{
  std::map<std::string, rclcpp::ParameterValue> overrides;
  overrides["status.battery.state_topics.pack.a"] = rclcpp::ParameterValue( std::string( "topic_a" ) );

  const auto topics = parse_battery_topics( overrides );

  ASSERT_EQ( topics.size(), 1u );
  EXPECT_EQ( topics.at( "pack.a" ), "topic_a" );
}

// --- aggregate_battery_level -------------------------------------------------

TEST( AggregateBatteryLevel, EmptyIsUnknown )
{
  EXPECT_EQ( level( {}, BatteryAggregation::Minimum ), -1 );
  EXPECT_EQ( level( {}, BatteryAggregation::Combined ), -1 );
}

TEST( AggregateBatteryLevel, AllUnknownIsUnknown )
{
  EXPECT_EQ( level( { battery( kNaN ), battery( kNaN ) }, BatteryAggregation::Minimum ), -1 );
  EXPECT_EQ( level( { battery( kNaN ), battery( kNaN ) }, BatteryAggregation::Combined ), -1 );
}

TEST( AggregateBatteryLevel, MinimumPicksWorst )
{
  EXPECT_EQ( level( { battery( 0.8f ), battery( 0.3f ), battery( 0.5f ) },
                    BatteryAggregation::Minimum ),
             30 );
}

TEST( AggregateBatteryLevel, CombinedIsCapacityWeighted )
{
  // (1 + 3) Ah charge over (2 + 4) Ah capacity -> 4/6 -> 67. Percentages are ignored when
  // charge/capacity are available.
  EXPECT_EQ( level( { battery( kNaN, 1.0f, 2.0f ), battery( kNaN, 3.0f, 4.0f ) },
                    BatteryAggregation::Combined ),
             67 );
}

TEST( AggregateBatteryLevel, CombinedFallsBackToMeanWithoutCapacity )
{
  // No pack exposes charge/capacity -> mean of percentages: (0.8 + 0.4) / 2 -> 60.
  EXPECT_EQ( level( { battery( 0.8f ), battery( 0.4f ) }, BatteryAggregation::Combined ), 60 );
}

TEST( AggregateBatteryLevel, IgnoresNaNAndOutOfRangePercentages )
{
  EXPECT_EQ( level( { battery( kNaN ), battery( 0.5f ) }, BatteryAggregation::Minimum ), 50 );
  EXPECT_EQ( level( { battery( -0.2f ), battery( 1.5f ), battery( 0.5f ) },
                    BatteryAggregation::Minimum ),
             50 );
}

TEST( AggregateBatteryLevel, RoundsAndClamps )
{
  EXPECT_EQ( level( { battery( 0.126f ) }, BatteryAggregation::Minimum ), 13 ); // rounds 12.6 -> 13
  // Combined with charge exceeding capacity would exceed 100% and must clamp.
  EXPECT_EQ( level( { battery( kNaN, 5.0f, 4.0f ) }, BatteryAggregation::Combined ), 100 );
}

TEST( AggregateBatteryLevel, CombinedDerivesChargeFromPercentage )
{
  // First pack reports charge/capacity (1 Ah of 2 Ah); the second reports only percentage and
  // capacity, so its charge is derived as 0.5 * 4 = 2 Ah. Pooled: (1 + 2) / (2 + 4) -> 50. The
  // second pack is no longer ignored.
  EXPECT_EQ( level( { battery( kNaN, 1.0f, 2.0f ), battery( 0.5f, kNaN, 4.0f ) },
                    BatteryAggregation::Combined ),
             50 );
}

TEST( AggregateBatteryLevel, CombinedUsesDesignCapacityFallback )
{
  // capacity is unmeasured but design_capacity (4 Ah) stands in: 2 Ah charge over 4 Ah -> 50.
  EXPECT_EQ( level( { battery( kNaN, 2.0f, kNaN, 4.0f ) }, BatteryAggregation::Combined ), 50 );
}

TEST( AggregateBatteryLevel, CombinedExcludesCapacitylessPack )
{
  // The percentage-only pack has no capacity to weight it, so it stays out of the energy pool; the
  // result is the lone charge/capacity pack: 1 / 2 -> 50.
  EXPECT_EQ( level( { battery( kNaN, 1.0f, 2.0f ), battery( 0.2f ) }, BatteryAggregation::Combined ),
             50 );
}

TEST( AggregateBatteryLevel, MinimumUsesChargeCapacityWhenPercentageMissing )
{
  // The first pack reports no percentage; its level comes from charge/capacity (1/4 -> 0.25) and is
  // the worst pack. Without the derivation it would have been ignored and the result 80.
  EXPECT_EQ( level( { battery( kNaN, 1.0f, 4.0f ), battery( 0.8f ) }, BatteryAggregation::Minimum ),
             25 );
}

TEST( AggregateBatteryLevel, TreatsDefaultZeroChargeAsUnset )
{
  // Real wire messages default charge to 0.0 (not NaN). A pack that sets capacity and percentage
  // but leaves charge at the default 0 must derive charge from percentage (0.9 * 4 -> 3.6 Ah over
  // 4 Ah -> 90), not read the default 0 as a fully discharged pack.
  BatteryState b;
  b.percentage = 0.9f;
  b.capacity = 4.0f;
  b.charge = 0.0f; // default-constructed value, not a measurement
  EXPECT_EQ( level( { b }, BatteryAggregation::Combined ), 90 );
  EXPECT_EQ( level( { b }, BatteryAggregation::Minimum ), 90 );
}

TEST( AggregateBatteryLevel, CombinedReportsDroppedCapacitylessPack )
{
  // One pack contributes (charge/capacity); the percentage-only pack has a level but no capacity to
  // weight it, so it is left out of the energy pool and reported via combined_dropped.
  const auto agg = aggregate_battery_level( { battery( kNaN, 1.0f, 2.0f ), battery( 0.2f ) },
                                            BatteryAggregation::Combined );
  EXPECT_EQ( static_cast<int>( agg.level ), 50 );
  EXPECT_EQ( agg.combined_dropped, 1u );
  EXPECT_FALSE( agg.combined_used_mean_fallback );
}

TEST( AggregateBatteryLevel, CombinedFlagsMeanFallback )
{
  // No pack exposes a capacity, so Combined falls back to the mean of percentages and flags it.
  const auto agg = aggregate_battery_level( { battery( 0.8f ), battery( 0.4f ) },
                                            BatteryAggregation::Combined );
  EXPECT_EQ( static_cast<int>( agg.level ), 60 );
  EXPECT_TRUE( agg.combined_used_mean_fallback );
  EXPECT_EQ( agg.combined_dropped, 0u );
}

// --- StatusReporter (requires a live node) -----------------------------------

namespace
{
std::shared_ptr<rclcpp::Node> make_node( const std::string &name,
                                         std::vector<rclcpp::Parameter> overrides )
{
  return std::make_shared<rclcpp::Node>(
      name, rclcpp::NodeOptions().parameter_overrides( std::move( overrides ) ) );
}

//! @brief Spins until `done()` holds or ~10s elapses; returns whether `done()` held, so callers can
//!        fail with a timeout message instead of on a misleading downstream assertion.
bool spin_until( rclcpp::executors::SingleThreadedExecutor &exec, const std::function<bool()> &done )
{
  for ( int i = 0; i < 1000 && !done(); ++i ) {
    exec.spin_some();
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }
  return done();
}
} // namespace

// status/heartbeat publishing is wired up only when the respective frequency is > 0.
TEST( StatusReporterActivation, InertWhenFrequenciesAbsent )
{
  auto node = make_node( "status_inert", {} );
  StatusReporter reporter( *node, "robot1" );
  EXPECT_FALSE( reporter.status_active() );
  EXPECT_FALSE( reporter.heartbeat_active() );
}

TEST( StatusReporterActivation, ActiveWhenFrequenciesPositive )
{
  auto node = make_node( "status_pos", { rclcpp::Parameter( "status.status_frequency", 1.0 ),
                                         rclcpp::Parameter( "status.heartbeat_frequency", 5.0 ) } );
  StatusReporter reporter( *node, "robot1" );
  EXPECT_TRUE( reporter.status_active() );
  EXPECT_TRUE( reporter.heartbeat_active() );
}

// A non-finite frequency (e.g. `.inf` in YAML) passes the > 0 check but would yield a zero-period
// timer spinning the executor; it must be treated as disabled instead.
TEST( StatusReporterActivation, NonFiniteFrequencyDisablesPublishing )
{
  const double inf = std::numeric_limits<double>::infinity();
  auto node = make_node( "status_inf", { rclcpp::Parameter( "status.status_frequency", inf ),
                                         rclcpp::Parameter( "status.heartbeat_frequency", inf ) } );
  StatusReporter reporter( *node, "robot1" );
  EXPECT_FALSE( reporter.status_active() );
  EXPECT_FALSE( reporter.heartbeat_active() );
}

// Regression mirror of the tf max_rate case: YAML `status_frequency: 1` is an integer parameter
// while the default is a double; declaring it must not throw on the type mismatch.
TEST( StatusReporterActivation, AcceptsIntegerFrequencyOverride )
{
  auto node = make_node( "status_int", { rclcpp::Parameter( "status.status_frequency", 1 ),
                                         rclcpp::Parameter( "status.heartbeat_frequency", 5 ) } );
  EXPECT_NO_THROW( {
    StatusReporter reporter( *node, "robot1" );
    EXPECT_TRUE( reporter.status_active() );
    EXPECT_TRUE( reporter.heartbeat_active() );
  } );
}

// End-to-end: two configured BatteryState topics are aggregated into a published RobotStatus.
TEST( StatusReporterIntegration, PublishesStatusWithAggregatedBattery )
{
  auto node =
      make_node( "status_battery_itest",
                 { rclcpp::Parameter( "status.status_frequency", 50.0 ),
                   rclcpp::Parameter( "status.battery.state_topics.left", std::string( "battery_left" ) ),
                   rclcpp::Parameter( "status.battery.state_topics.right",
                                      std::string( "battery_right" ) ) } );
  StatusReporter reporter( *node, "robot_xyz" );
  ASSERT_TRUE( reporter.status_active() );

  std::optional<RobotStatus> got;
  auto sub = node->create_subscription<RobotStatus>(
      "robot_status", latched_qos(),
      [&got]( RobotStatus::ConstSharedPtr msg ) {
        // Skip the placeholder-only status (NaN percentages) published before any battery message.
        if ( msg->batteries.size() == 2 && !std::isnan( msg->batteries[0].percentage ) &&
             !std::isnan( msg->batteries[1].percentage ) )
          got = *msg;
      } );
  auto pub_left = node->create_publisher<BatteryState>( "battery_left", rclcpp::SensorDataQoS() );
  auto pub_right = node->create_publisher<BatteryState>( "battery_right", rclcpp::SensorDataQoS() );

  const BatteryState left = battery( 0.5f );
  const BatteryState right = battery( 0.9f );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  spin_until( exec, [&] {
    return pub_left->get_subscription_count() > 0 && pub_right->get_subscription_count() > 0 &&
           sub->get_publisher_count() > 0;
  } );
  for ( int i = 0; i < 1000 && !got; ++i ) {
    pub_left->publish( left );
    pub_right->publish( right );
    exec.spin_some();
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }

  ASSERT_TRUE( got.has_value() ) << "no RobotStatus with both batteries was received";
  EXPECT_EQ( got->robot_id, "robot_xyz" );
  ASSERT_EQ( got->batteries.size(), 2u );
  EXPECT_EQ( static_cast<int>( got->battery_level ), 50 ); // minimum of 50% / 90%
}

// Configured batteries are reported as present=false/unknown before any BatteryState arrives, so the
// status reflects the configured battery count rather than only the packs that have published.
TEST( StatusReporterIntegration, ReportsConfiguredBatteriesBeforeData )
{
  auto node = make_node(
      "status_battery_placeholder_itest",
      { rclcpp::Parameter( "status.status_frequency", 50.0 ),
        rclcpp::Parameter( "status.battery.state_topics.left", std::string( "battery_left" ) ),
        rclcpp::Parameter( "status.battery.state_topics.right", std::string( "battery_right" ) ) } );
  StatusReporter reporter( *node, "robot1" );
  ASSERT_TRUE( reporter.status_active() );

  std::optional<RobotStatus> got;
  auto sub = node->create_subscription<RobotStatus>(
      "robot_status", latched_qos(), [&got]( RobotStatus::ConstSharedPtr msg ) { got = *msg; } );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  ASSERT_TRUE( spin_until( exec, [&] { return got.has_value(); } ) ) << "no RobotStatus published";

  ASSERT_EQ( got->batteries.size(), 2u ); // both configured batteries reported, none published
  EXPECT_EQ( got->batteries[0].location, "left" );
  EXPECT_EQ( got->batteries[1].location, "right" );
  for ( const auto &b : got->batteries ) {
    EXPECT_FALSE( b.present );
    EXPECT_TRUE( std::isnan( b.percentage ) );
  }
  EXPECT_EQ( static_cast<int>( got->battery_level ), -1 ); // all unknown
}

// The set_status service updates the code/message carried by the next RobotStatus.
TEST( StatusReporterIntegration, SetStatusUpdatesPublishedStatus )
{
  auto node =
      make_node( "status_setstatus_itest", { rclcpp::Parameter( "status.status_frequency", 50.0 ) } );
  StatusReporter reporter( *node, "robot1" );
  ASSERT_TRUE( reporter.status_active() );

  std::optional<RobotStatus> got;
  auto sub = node->create_subscription<RobotStatus>(
      "robot_status", latched_qos(),
      [&got]( RobotStatus::ConstSharedPtr msg ) { got = *msg; } );
  auto client = node->create_client<SetStatus>( "set_status" );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  ASSERT_TRUE( spin_until( exec, [&] { return client->service_is_ready(); } ) )
      << "set_status service never became ready";

  auto request = std::make_shared<SetStatus::Request>();
  request->status_code = RobotStatus::STATUS_WARNING;
  request->status_message = "low battery";
  auto future = client->async_send_request( request );
  ASSERT_EQ( exec.spin_until_future_complete( future ), rclcpp::FutureReturnCode::SUCCESS );

  ASSERT_TRUE(
      spin_until( exec, [&] { return got && got->status_code == RobotStatus::STATUS_WARNING; } ) )
      << "RobotStatus carrying the new status_code was not received";
  ASSERT_TRUE( got.has_value() );
  EXPECT_EQ( static_cast<int>( got->status_code ),
             static_cast<int>( RobotStatus::STATUS_WARNING ) );
  EXPECT_EQ( got->status_message, "low battery" );
}

TEST( StatusReporterIntegration, HeartbeatIsPublished )
{
  auto node = make_node( "status_heartbeat_itest",
                         { rclcpp::Parameter( "status.heartbeat_frequency", 50.0 ) } );
  StatusReporter reporter( *node, "robot1" );
  ASSERT_TRUE( reporter.heartbeat_active() );

  std::vector<uint32_t> seqs;
  auto sub = node->create_subscription<Heartbeat>(
      "robot_heartbeat", rclcpp::SensorDataQoS(),
      [&seqs]( Heartbeat::ConstSharedPtr msg ) { seqs.push_back( msg->seq ); } );

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );
  ASSERT_TRUE( spin_until( exec, [&] { return seqs.size() >= 2; } ) )
      << "fewer than two heartbeats received";
  EXPECT_GT( seqs.back(), seqs.front() ); // sequence advances across heartbeats
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
