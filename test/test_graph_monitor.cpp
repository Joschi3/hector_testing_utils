/// @file test_graph_monitor.cpp
/// @brief Tests for graph change monitoring functionality.
///
/// This file tests:
/// - Node, topic, and service existence checking
/// - Waiting for nodes/topics/services to appear or disappear
/// - Graph change detection and tracking
/// - Change callbacks

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>

#include <hector_testing_utils/hector_testing_utils.hpp>

using namespace std::chrono_literals;
using hector_testing_utils::GraphChange;
using hector_testing_utils::GraphMonitor;
using hector_testing_utils::HectorTestFixture;

// =============================================================================
// GraphChange Tests
// =============================================================================

TEST( GraphChange, TypeToString )
{
  EXPECT_EQ( GraphChange::type_to_string( GraphChange::Type::NODE_ADDED ), "NODE_ADDED" );
  EXPECT_EQ( GraphChange::type_to_string( GraphChange::Type::NODE_REMOVED ), "NODE_REMOVED" );
  EXPECT_EQ( GraphChange::type_to_string( GraphChange::Type::TOPIC_ADDED ), "TOPIC_ADDED" );
  EXPECT_EQ( GraphChange::type_to_string( GraphChange::Type::TOPIC_REMOVED ), "TOPIC_REMOVED" );
  EXPECT_EQ( GraphChange::type_to_string( GraphChange::Type::SERVICE_ADDED ), "SERVICE_ADDED" );
  EXPECT_EQ( GraphChange::type_to_string( GraphChange::Type::SERVICE_REMOVED ), "SERVICE_REMOVED" );
}

TEST( GraphChange, Format )
{
  GraphChange change;
  change.type = GraphChange::Type::TOPIC_ADDED;
  change.name = "/my_topic";
  change.types = { "std_msgs/msg/Int32" };
  change.timestamp = std::chrono::steady_clock::now();

  std::string formatted = change.format();

  EXPECT_NE( formatted.find( "TOPIC_ADDED" ), std::string::npos );
  EXPECT_NE( formatted.find( "/my_topic" ), std::string::npos );
  EXPECT_NE( formatted.find( "std_msgs/msg/Int32" ), std::string::npos );
}

TEST( GraphChange, FormatWithoutTypes )
{
  GraphChange change;
  change.type = GraphChange::Type::NODE_ADDED;
  change.name = "/my_node";
  change.timestamp = std::chrono::steady_clock::now();

  std::string formatted = change.format();

  EXPECT_NE( formatted.find( "NODE_ADDED" ), std::string::npos );
  EXPECT_NE( formatted.find( "/my_node" ), std::string::npos );
}

// =============================================================================
// GraphMonitor Basic Tests
// =============================================================================

TEST_F( HectorTestFixture, GraphMonitorConstruction )
{
  GraphMonitor monitor( tester_node_ );

  // Should not be monitoring yet
  EXPECT_FALSE( monitor.is_monitoring() );
}

TEST_F( HectorTestFixture, GraphMonitorStartStop )
{
  GraphMonitor monitor( tester_node_ );

  monitor.start();
  EXPECT_TRUE( monitor.is_monitoring() );

  monitor.stop();
  EXPECT_FALSE( monitor.is_monitoring() );
}

TEST_F( HectorTestFixture, GraphMonitorDoubleStart )
{
  GraphMonitor monitor( tester_node_ );

  monitor.start();
  monitor.start(); // Should be a no-op

  EXPECT_TRUE( monitor.is_monitoring() );

  monitor.stop();
  EXPECT_FALSE( monitor.is_monitoring() );
}

// =============================================================================
// Query Function Tests
// =============================================================================

TEST_F( HectorTestFixture, HasNode )
{
  GraphMonitor monitor( tester_node_ );

  // Our own node should exist
  auto node_names = tester_node_->get_node_names();
  ASSERT_FALSE( node_names.empty() );

  // The tester node should find itself
  bool found_self = false;
  for ( const auto &name : node_names ) {
    if ( monitor.has_node( name ) ) {
      found_self = true;
      break;
    }
  }
  EXPECT_TRUE( found_self );

  // Non-existent node should not be found
  EXPECT_FALSE( monitor.has_node( "/definitely_nonexistent_node_12345" ) );
}

TEST_F( HectorTestFixture, HasTopic )
{
  GraphMonitor monitor( tester_node_ );

  // Create a publisher to create a topic
  auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/monitor_topic", 10 );

  // Give graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  EXPECT_TRUE( monitor.has_topic( "/test/monitor_topic" ) );
  EXPECT_FALSE( monitor.has_topic( "/definitely_nonexistent_topic_12345" ) );
}

TEST_F( HectorTestFixture, HasService )
{
  using Service = example_interfaces::srv::AddTwoInts;

  GraphMonitor monitor( tester_node_ );

  // Create a service
  auto service = tester_node_->create_service<Service>(
      "/test/monitor_service",
      []( const std::shared_ptr<Service::Request>, std::shared_ptr<Service::Response> ) { } );

  // Give graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  EXPECT_TRUE( monitor.has_service( "/test/monitor_service" ) );
  EXPECT_FALSE( monitor.has_service( "/definitely_nonexistent_service_12345" ) );
}

TEST_F( HectorTestFixture, GetNodes )
{
  GraphMonitor monitor( tester_node_ );

  auto nodes = monitor.get_nodes();

  // Should have at least our own node
  EXPECT_FALSE( nodes.empty() );
}

TEST_F( HectorTestFixture, GetTopics )
{
  GraphMonitor monitor( tester_node_ );

  // Create a publisher
  auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/get_topics_test", 10 );

  // Give graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  auto topics = monitor.get_topics();

  // Should find our topic
  EXPECT_TRUE( topics.count( "/test/get_topics_test" ) > 0 );
}

TEST_F( HectorTestFixture, GetServices )
{
  using Service = example_interfaces::srv::AddTwoInts;

  GraphMonitor monitor( tester_node_ );

  // Create a service
  auto service = tester_node_->create_service<Service>(
      "/test/get_services_test",
      []( const std::shared_ptr<Service::Request>, std::shared_ptr<Service::Response> ) { } );

  // Give graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  auto services = monitor.get_services();

  // Should find our service
  EXPECT_TRUE( services.count( "/test/get_services_test" ) > 0 );
}

// =============================================================================
// Wait Function Tests
// =============================================================================

TEST_F( HectorTestFixture, WaitForTopic )
{
  GraphMonitor monitor( tester_node_ );
  monitor.start();

  // Create a publisher after a short delay
  std::thread creator( [this]() {
    std::this_thread::sleep_for( 100ms );
    auto pub =
        tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/wait_for_topic_test", 10 );
    std::this_thread::sleep_for( 1s ); // Keep publisher alive
  } );

  // Wait for the topic
  bool found = monitor.wait_for_topic( *executor_, "/test/wait_for_topic_test", 2s );

  creator.join();

  EXPECT_TRUE( found );
  monitor.stop();
}

TEST_F( HectorTestFixture, WaitForTopicTimeout )
{
  GraphMonitor monitor( tester_node_ );
  monitor.start();

  // Wait for a topic that will never appear
  bool found = monitor.wait_for_topic( *executor_, "/definitely_nonexistent_topic_xyz", 200ms );

  EXPECT_FALSE( found );
  monitor.stop();
}

TEST_F( HectorTestFixture, WaitForService )
{
  using Service = example_interfaces::srv::AddTwoInts;

  GraphMonitor monitor( tester_node_ );
  monitor.start();

  // Create a service after a short delay
  rclcpp::Service<Service>::SharedPtr service;
  std::thread creator( [this, &service]() {
    std::this_thread::sleep_for( 100ms );
    service = tester_node_->create_service<Service>(
        "/test/wait_for_service_test",
        []( const std::shared_ptr<Service::Request>, std::shared_ptr<Service::Response> ) { } );
    std::this_thread::sleep_for( 1s ); // Keep service alive
  } );

  // Wait for the service
  bool found = monitor.wait_for_service( *executor_, "/test/wait_for_service_test", 2s );

  creator.join();

  EXPECT_TRUE( found );
  monitor.stop();
}

TEST_F( HectorTestFixture, WaitForTopicRemoved )
{
  GraphMonitor monitor( tester_node_ );
  monitor.start();

  // Create a publisher that we'll destroy
  auto pub = std::make_shared<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>(
      tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/wait_for_removed_test", 10 ) );

  // Give graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  EXPECT_TRUE( monitor.has_topic( "/test/wait_for_removed_test" ) );

  // Destroy the publisher in a separate thread
  std::thread destroyer( [&pub]() {
    std::this_thread::sleep_for( 100ms );
    pub->reset();
  } );

  // Wait for topic to disappear
  bool removed = monitor.wait_for_topic_removed( *executor_, "/test/wait_for_removed_test", 2s );

  destroyer.join();

  EXPECT_TRUE( removed );
  monitor.stop();
}

// =============================================================================
// Change Detection Tests
// =============================================================================

TEST_F( HectorTestFixture, DetectTopicAdded )
{
  GraphMonitor monitor( tester_node_ );
  monitor.start();

  // Clear any initial changes
  monitor.clear_changes();

  // Create a publisher
  auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/detect_added_topic", 10 );

  // Wait for any change
  bool changed = monitor.wait_for_any_change( *executor_, 2s );
  EXPECT_TRUE( changed );

  auto changes = monitor.get_changes();

  // Should detect the topic being added
  bool found_add = false;
  for ( const auto &change : changes ) {
    if ( change.type == GraphChange::Type::TOPIC_ADDED &&
         change.name == "/test/detect_added_topic" ) {
      found_add = true;
      break;
    }
  }

  EXPECT_TRUE( found_add ) << "Expected to detect topic addition";
  monitor.stop();
}

TEST_F( HectorTestFixture, GetChangesSince )
{
  GraphMonitor monitor( tester_node_ );
  monitor.start();

  auto before_change = std::chrono::steady_clock::now();

  // Create a publisher
  auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/changes_since_topic", 10 );

  // Wait for any change
  monitor.wait_for_any_change( *executor_, 2s );

  auto changes = monitor.get_changes_since( before_change );

  // Should find changes after the time point
  EXPECT_FALSE( changes.empty() );

  monitor.stop();
}

TEST_F( HectorTestFixture, ChangeCallback )
{
  GraphMonitor monitor( tester_node_ );

  std::vector<GraphChange> callback_changes;
  std::mutex callback_mutex;

  monitor.set_change_callback( [&callback_changes, &callback_mutex]( const GraphChange &change ) {
    std::lock_guard<std::mutex> lock( callback_mutex );
    callback_changes.push_back( change );
  } );

  monitor.start();

  // Create a publisher to trigger a change
  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/callback_trigger_topic", 10 );

  // Wait for change
  monitor.wait_for_any_change( *executor_, 2s );

  // Check callback was called
  {
    std::lock_guard<std::mutex> lock( callback_mutex );
    EXPECT_FALSE( callback_changes.empty() );
  }

  monitor.stop();
}

TEST_F( HectorTestFixture, ClearChanges )
{
  GraphMonitor monitor( tester_node_ );
  monitor.start();

  // Create a publisher
  auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/clear_changes_topic", 10 );

  // Wait for change
  monitor.wait_for_any_change( *executor_, 2s );

  auto changes = monitor.get_changes();
  EXPECT_FALSE( changes.empty() );

  // Clear changes
  monitor.clear_changes();

  changes = monitor.get_changes();
  EXPECT_TRUE( changes.empty() );

  monitor.stop();
}

// =============================================================================
// Edge Cases
// =============================================================================

TEST_F( HectorTestFixture, PollWithoutMonitoring )
{
  GraphMonitor monitor( tester_node_ );

  // Polling without starting should be a no-op
  monitor.poll();

  // Should not have any changes
  auto changes = monitor.get_changes();
  EXPECT_TRUE( changes.empty() );
}

TEST_F( HectorTestFixture, MultipleStartStopCycles )
{
  GraphMonitor monitor( tester_node_ );

  for ( int i = 0; i < 3; ++i ) {
    monitor.start();
    EXPECT_TRUE( monitor.is_monitoring() );

    // Create a publisher
    auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>(
        "/test/multi_cycle_topic_" + std::to_string( i ), 10 );

    executor_->spin_until( []() { return true; }, 200ms );

    monitor.stop();
    EXPECT_FALSE( monitor.is_monitoring() );

    monitor.clear_changes();
  }
}
