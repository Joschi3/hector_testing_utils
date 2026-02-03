/// @file test_qos_helpers.cpp
/// @brief Tests for QoS validation and diagnostics functionality.
///
/// This file tests:
/// - QoS compatibility checking between publishers and subscribers
/// - QoS information formatting and conversion
/// - Topic QoS introspection
/// - Compatibility hint generation

#include <chrono>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>

#include <hector_testing_utils/hector_testing_utils.hpp>

using namespace std::chrono_literals;
using hector_testing_utils::HectorTestFixture;

// =============================================================================
// QoSInfo Tests
// =============================================================================

TEST( QoSInfo, ReliabilityToString )
{
  using hector_testing_utils::QoSInfo;

  EXPECT_EQ( QoSInfo::reliability_to_string( RMW_QOS_POLICY_RELIABILITY_RELIABLE ), "RELIABLE" );
  EXPECT_EQ( QoSInfo::reliability_to_string( RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT ),
             "BEST_EFFORT" );
  EXPECT_EQ( QoSInfo::reliability_to_string( RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT ),
             "SYSTEM_DEFAULT" );
  EXPECT_EQ( QoSInfo::reliability_to_string( RMW_QOS_POLICY_RELIABILITY_UNKNOWN ), "UNKNOWN" );
}

TEST( QoSInfo, DurabilityToString )
{
  using hector_testing_utils::QoSInfo;

  EXPECT_EQ( QoSInfo::durability_to_string( RMW_QOS_POLICY_DURABILITY_VOLATILE ), "VOLATILE" );
  EXPECT_EQ( QoSInfo::durability_to_string( RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL ),
             "TRANSIENT_LOCAL" );
  EXPECT_EQ( QoSInfo::durability_to_string( RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT ),
             "SYSTEM_DEFAULT" );
  EXPECT_EQ( QoSInfo::durability_to_string( RMW_QOS_POLICY_DURABILITY_UNKNOWN ), "UNKNOWN" );
}

TEST( QoSInfo, HistoryToString )
{
  using hector_testing_utils::QoSInfo;

  EXPECT_EQ( QoSInfo::history_to_string( RMW_QOS_POLICY_HISTORY_KEEP_LAST ), "KEEP_LAST" );
  EXPECT_EQ( QoSInfo::history_to_string( RMW_QOS_POLICY_HISTORY_KEEP_ALL ), "KEEP_ALL" );
  EXPECT_EQ( QoSInfo::history_to_string( RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT ),
             "SYSTEM_DEFAULT" );
  EXPECT_EQ( QoSInfo::history_to_string( RMW_QOS_POLICY_HISTORY_UNKNOWN ), "UNKNOWN" );
}

TEST( QoSInfo, FromRclcppQoS )
{
  using hector_testing_utils::QoSInfo;

  rclcpp::QoS qos( 10 );
  qos.reliable();
  qos.durability_volatile();

  auto info = QoSInfo::from_rclcpp_qos( qos );

  EXPECT_EQ( info.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE );
  EXPECT_EQ( info.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE );
  EXPECT_EQ( info.depth, 10u );
}

TEST( QoSInfo, Format )
{
  using hector_testing_utils::QoSInfo;

  QoSInfo info;
  info.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  info.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  info.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  info.depth = 10;

  std::string formatted = info.format();

  EXPECT_NE( formatted.find( "RELIABLE" ), std::string::npos );
  EXPECT_NE( formatted.find( "VOLATILE" ), std::string::npos );
  EXPECT_NE( formatted.find( "KEEP_LAST" ), std::string::npos );
  EXPECT_NE( formatted.find( "10" ), std::string::npos );
}

// =============================================================================
// QoS Compatibility Tests
// =============================================================================

TEST( QoSCompatibility, ReliableSubscriberWithBestEffortPublisher )
{
  using hector_testing_utils::check_reliability_compatibility;

  auto result = check_reliability_compatibility( RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                                 RMW_QOS_POLICY_RELIABILITY_RELIABLE );

  EXPECT_FALSE( result.compatible );
  EXPECT_FALSE( result.errors.empty() );
}

TEST( QoSCompatibility, BestEffortSubscriberWithReliablePublisher )
{
  using hector_testing_utils::check_reliability_compatibility;

  auto result = check_reliability_compatibility( RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                                 RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT );

  EXPECT_TRUE( result.compatible );
  EXPECT_TRUE( result.errors.empty() );
}

TEST( QoSCompatibility, ReliableSubscriberWithReliablePublisher )
{
  using hector_testing_utils::check_reliability_compatibility;

  auto result = check_reliability_compatibility( RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                                 RMW_QOS_POLICY_RELIABILITY_RELIABLE );

  EXPECT_TRUE( result.compatible );
  EXPECT_TRUE( result.errors.empty() );
}

TEST( QoSCompatibility, TransientLocalSubscriberWithVolatilePublisher )
{
  using hector_testing_utils::check_durability_compatibility;

  auto result = check_durability_compatibility( RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                                RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL );

  EXPECT_FALSE( result.compatible );
  EXPECT_FALSE( result.errors.empty() );
}

TEST( QoSCompatibility, VolatileSubscriberWithTransientLocalPublisher )
{
  using hector_testing_utils::check_durability_compatibility;

  auto result = check_durability_compatibility( RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                                                RMW_QOS_POLICY_DURABILITY_VOLATILE );

  EXPECT_TRUE( result.compatible );
  EXPECT_TRUE( result.errors.empty() );
}

TEST( QoSCompatibility, FullQoSCheck )
{
  using hector_testing_utils::check_qos_compatibility;
  using hector_testing_utils::QoSInfo;

  QoSInfo pub_qos;
  pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  pub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  QoSInfo sub_qos;
  sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  sub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  auto result = check_qos_compatibility( pub_qos, sub_qos );

  EXPECT_FALSE( result.compatible );
  // Should have errors for both reliability and durability
  EXPECT_GE( result.errors.size(), 2u );
}

TEST( QoSCompatibility, CompatibleQoS )
{
  using hector_testing_utils::check_qos_compatibility;

  rclcpp::QoS pub_qos( 10 );
  pub_qos.reliable();
  pub_qos.transient_local();

  rclcpp::QoS sub_qos( 10 );
  sub_qos.reliable();
  sub_qos.transient_local();

  auto result = check_qos_compatibility( pub_qos, sub_qos );

  EXPECT_TRUE( result.compatible );
  EXPECT_TRUE( result.errors.empty() );
}

// =============================================================================
// QoSCompatibilityResult Tests
// =============================================================================

TEST( QoSCompatibilityResult, Format )
{
  using hector_testing_utils::QoSCompatibilityResult;

  QoSCompatibilityResult result;
  result.compatible = false;
  result.errors.push_back( "Reliability mismatch" );
  result.warnings.push_back( "Consider adjusting depth" );

  std::string formatted = result.format();

  EXPECT_NE( formatted.find( "INCOMPATIBLE" ), std::string::npos );
  EXPECT_NE( formatted.find( "ERROR" ), std::string::npos );
  EXPECT_NE( formatted.find( "Reliability mismatch" ), std::string::npos );
  EXPECT_NE( formatted.find( "WARN" ), std::string::npos );
}

TEST( QoSCompatibilityResult, FormatCompatible )
{
  using hector_testing_utils::QoSCompatibilityResult;

  QoSCompatibilityResult result;
  result.compatible = true;

  std::string formatted = result.format();

  EXPECT_NE( formatted.find( "compatible" ), std::string::npos );
}

// =============================================================================
// EndpointInfo Tests
// =============================================================================

TEST( EndpointInfo, FullNodeName )
{
  using hector_testing_utils::EndpointInfo;

  EndpointInfo info;
  info.node_name = "my_node";
  info.node_namespace = "/my_namespace";

  EXPECT_EQ( info.full_node_name(), "/my_namespace/my_node" );
}

TEST( EndpointInfo, FullNodeNameRootNamespace )
{
  using hector_testing_utils::EndpointInfo;

  EndpointInfo info;
  info.node_name = "my_node";
  info.node_namespace = "/";

  EXPECT_EQ( info.full_node_name(), "/my_node" );
}

TEST( EndpointInfo, Format )
{
  using hector_testing_utils::EndpointInfo;
  using hector_testing_utils::QoSInfo;

  EndpointInfo info;
  info.node_name = "test_node";
  info.node_namespace = "/";
  info.topic_type = "std_msgs/msg/Int32";
  info.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  info.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  info.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  info.qos.depth = 10;

  std::string formatted = info.format();

  EXPECT_NE( formatted.find( "test_node" ), std::string::npos );
  EXPECT_NE( formatted.find( "std_msgs/msg/Int32" ), std::string::npos );
  EXPECT_NE( formatted.find( "RELIABLE" ), std::string::npos );
}

// =============================================================================
// Topic QoS Introspection Tests (with actual ROS nodes)
// =============================================================================

TEST_F( HectorTestFixture, GetPublishersInfo )
{
  // Create a publisher with specific QoS
  rclcpp::QoS qos( 10 );
  qos.reliable();

  auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/qos_topic", qos );

  // Give the graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  auto publishers = hector_testing_utils::get_publishers_info( tester_node_, "/test/qos_topic" );

  // Should find our publisher
  ASSERT_FALSE( publishers.empty() );

  bool found = false;
  for ( const auto &info : publishers ) {
    if ( info.topic_type.find( "Int32" ) != std::string::npos ) {
      found = true;
      EXPECT_EQ( info.qos.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE );
    }
  }

  EXPECT_TRUE( found ) << "Expected to find our publisher with Int32 type";
}

TEST_F( HectorTestFixture, GetSubscribersInfo )
{
  // Create a subscriber with specific QoS
  rclcpp::QoS qos( 10 );
  qos.best_effort();

  auto sub = tester_node_->create_subscription<std_msgs::msg::Int32>(
      "/test/qos_sub_topic", qos, []( std_msgs::msg::Int32::SharedPtr ) { } );

  // Give the graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  auto subscribers =
      hector_testing_utils::get_subscribers_info( tester_node_, "/test/qos_sub_topic" );

  // Should find our subscriber
  ASSERT_FALSE( subscribers.empty() );

  bool found = false;
  for ( const auto &info : subscribers ) {
    if ( info.topic_type.find( "Int32" ) != std::string::npos ) {
      found = true;
      EXPECT_EQ( info.qos.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT );
    }
  }

  EXPECT_TRUE( found ) << "Expected to find our subscriber with Int32 type";
}

TEST_F( HectorTestFixture, DiagnoseTopicQoS )
{
  // Create incompatible pub/sub pair
  rclcpp::QoS pub_qos( 10 );
  pub_qos.best_effort();

  rclcpp::QoS sub_qos( 10 );
  sub_qos.reliable();

  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/incompatible_topic", pub_qos );
  auto sub = tester_node_->create_subscription<std_msgs::msg::Int32>(
      "/test/incompatible_topic", sub_qos, []( std_msgs::msg::Int32::SharedPtr ) { } );

  // Give the graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  std::string diagnosis =
      hector_testing_utils::diagnose_topic_qos( tester_node_, "/test/incompatible_topic" );

  // Should report the incompatibility
  EXPECT_NE( diagnosis.find( "Publisher" ), std::string::npos );
  EXPECT_NE( diagnosis.find( "Subscriber" ), std::string::npos );
}

TEST_F( HectorTestFixture, GetTopicQoSSummary )
{
  // Create a publisher
  rclcpp::QoS qos( 10 );
  qos.reliable();

  auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/summary_topic", qos );

  // Give the graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  std::string summary =
      hector_testing_utils::get_topic_qos_summary( tester_node_, "/test/summary_topic" );

  // Should contain publisher info
  EXPECT_NE( summary.find( "pubs:" ), std::string::npos );
  EXPECT_NE( summary.find( "RELIABLE" ), std::string::npos );
}

TEST_F( HectorTestFixture, CheckTopicQoSCompatibility )
{
  // Create a subscriber with RELIABLE QoS
  rclcpp::QoS sub_qos( 10 );
  sub_qos.reliable();

  auto sub = tester_node_->create_subscription<std_msgs::msg::Int32>(
      "/test/compat_check_topic", sub_qos, []( std_msgs::msg::Int32::SharedPtr ) { } );

  // Give the graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  // Check if a BEST_EFFORT publisher would be compatible
  rclcpp::QoS pub_qos( 10 );
  pub_qos.best_effort();

  auto result = hector_testing_utils::check_topic_qos_compatibility(
      tester_node_, "/test/compat_check_topic", pub_qos, true );

  // Should be incompatible
  EXPECT_FALSE( result.compatible );
}

TEST_F( HectorTestFixture, GetQoSCompatibilityHint )
{
  // Create a subscriber with RELIABLE QoS
  rclcpp::QoS sub_qos( 10 );
  sub_qos.reliable();

  auto sub = tester_node_->create_subscription<std_msgs::msg::Int32>(
      "/test/hint_topic", sub_qos, []( std_msgs::msg::Int32::SharedPtr ) { } );

  // Give the graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  // Check hint for incompatible publisher
  rclcpp::QoS pub_qos( 10 );
  pub_qos.best_effort();

  std::string hint = hector_testing_utils::get_qos_compatibility_hint(
      tester_node_, "/test/hint_topic", pub_qos, true );

  // Should indicate incompatibility
  EXPECT_NE( hint.find( "INCOMPATIBLE" ), std::string::npos );
}

TEST_F( HectorTestFixture, GetQoSCompatibilityHintCompatible )
{
  // Create a subscriber with BEST_EFFORT QoS
  rclcpp::QoS sub_qos( 10 );
  sub_qos.best_effort();

  auto sub = tester_node_->create_subscription<std_msgs::msg::Int32>(
      "/test/hint_compat_topic", sub_qos, []( std_msgs::msg::Int32::SharedPtr ) { } );

  // Give the graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  // Check hint for compatible publisher
  rclcpp::QoS pub_qos( 10 );
  pub_qos.reliable();

  std::string hint = hector_testing_utils::get_qos_compatibility_hint(
      tester_node_, "/test/hint_compat_topic", pub_qos, true );

  // Should be empty (compatible)
  EXPECT_TRUE( hint.empty() );
}

// =============================================================================
// Integration with Suggestions
// =============================================================================

TEST_F( HectorTestFixture, SuggestionsIncludeQoSInfo )
{
  // Create a publisher with specific QoS
  rclcpp::QoS pub_qos( 10 );
  pub_qos.best_effort();

  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/qos_suggest_topic", pub_qos );

  // Give the graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  // Create a subscriber that will wait for a non-existent topic
  rclcpp::QoS sub_qos( 10 );
  sub_qos.reliable();

  auto test_sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>(
      "/test/qos_suggestXX_topic", sub_qos ); // Misspelled topic

  // Wait (should timeout) and check failure info
  hector_testing_utils::WaitFailureInfo failure_info;
  bool success = test_sub->wait_for_publishers( *executor_, 1, 200ms, failure_info );

  EXPECT_FALSE( success );

  // Check that suggestions include QoS info
  bool found_topic = false;
  for ( const auto &suggestion : failure_info.suggestions ) {
    if ( suggestion.name == "/test/qos_suggest_topic" ) {
      found_topic = true;
      // Should have QoS info
      EXPECT_FALSE( suggestion.qos_info.empty() );

      // Should have compatibility hint (since we're using incompatible QoS)
      EXPECT_FALSE( suggestion.qos_compatibility_hint.empty() );
      break;
    }
  }

  // The topic should be found in suggestions since we created a publisher
  EXPECT_TRUE( found_topic ) << "Expected to find /test/qos_suggest_topic in suggestions";
}
