#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include <std_msgs/msg/int32.hpp>

#include <rclcpp/rclcpp.hpp>

#include <hector_testing_utils/hector_testing_utils.hpp>

using namespace std::chrono_literals;
using hector_testing_utils::HectorTestFixture;

// Test that timeouts work correctly when condition is not met
TEST_F( HectorTestFixture, TimeoutWhenConditionNotMet )
{
  const std::string topic = "/timeout_test";

  // Create a subscriber but no publisher
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>( topic );

  auto start = std::chrono::steady_clock::now();
  bool result = sub->wait_for_message( *executor_, 1s );
  auto duration = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE( result );        // Should timeout
  EXPECT_GE( duration, 1s );     // Should wait at least 1 second
  EXPECT_LT( duration, 1500ms ); // But not too much longer
}

// Test that successful conditions return quickly
TEST_F( HectorTestFixture, QuickReturnOnSuccess )
{
  const std::string topic = "/quick_return_test";

  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>( topic );
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>( topic );

  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 5s ) );

  auto start = std::chrono::steady_clock::now();

  // Publish immediately
  std_msgs::msg::Int32 msg;
  msg.data = 123;
  pub->publish( msg );

  bool result = sub->wait_for_message( *executor_, 5s );
  auto duration = std::chrono::steady_clock::now() - start;

  EXPECT_TRUE( result );
  // Should return quickly, well before the timeout
  EXPECT_LT( duration, 1s );
}

// Test executor spin_until behavior
TEST_F( HectorTestFixture, ExecutorSpinUntilBehavior )
{
  int counter = 0;

  // Increment counter in a background thread
  std::thread increment_thread( [&counter]() {
    std::this_thread::sleep_for( 200ms );
    counter = 1;
    std::this_thread::sleep_for( 200ms );
    counter = 2;
    std::this_thread::sleep_for( 200ms );
    counter = 3;
  } );

  // Wait until counter reaches 3
  auto start = std::chrono::steady_clock::now();
  bool result = executor_->spin_until( [&counter]() { return counter >= 3; }, 2s );
  auto duration = std::chrono::steady_clock::now() - start;

  increment_thread.join();

  EXPECT_TRUE( result );
  EXPECT_GE( counter, 3 );
  EXPECT_GE( duration, 600ms ); // Should take at least 600ms
  EXPECT_LT( duration, 1s );    // But return quickly after condition met
}

// Test wait_for_publishers with actual timing
TEST_F( HectorTestFixture, WaitForPublishersTiming )
{
  const std::string topic = "/publisher_timing_test";

  auto sub_node = std::make_shared<rclcpp::Node>( "subscriber_timing_node" );
  executor_->add_node( sub_node );

  hector_testing_utils::CachedSubscriber<std_msgs::msg::Int32> sub( sub_node, topic );

  // Start thread that creates publisher after delay
  std::thread delayed_pub( [this, topic]() {
    std::this_thread::sleep_for( 300ms );
    auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>( topic, 10 );
    std::this_thread::sleep_for( 1s ); // Keep it alive
  } );

  auto start = std::chrono::steady_clock::now();
  bool result = sub.wait_for_publishers( *executor_, 1, 2s );
  auto duration = std::chrono::steady_clock::now() - start;

  delayed_pub.join();

  EXPECT_TRUE( result );
  EXPECT_GE( duration, 300ms ); // Should wait for publisher
  EXPECT_LT( duration, 1s );    // But return soon after
}

// Test multiple sequential wait operations
TEST_F( HectorTestFixture, SequentialWaitOperations )
{
  const std::string topic = "/sequential_wait_test";

  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>( topic );
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>( topic );

  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 5s ) );

  // Publish multiple messages with delays
  for ( int i = 0; i < 3; ++i ) {
    sub->reset(); // Clear previous messages

    std_msgs::msg::Int32 msg;
    msg.data = i * 10;
    pub->publish( msg );

    auto start = std::chrono::steady_clock::now();
    ASSERT_TRUE( sub->wait_for_message( *executor_, 1s ) );
    auto duration = std::chrono::steady_clock::now() - start;

    auto received = sub->last_message();
    ASSERT_TRUE( received.has_value() );
    EXPECT_EQ( received->data, i * 10 );
    EXPECT_LT( duration, 500ms ); // Each should be quick
  }
}

// Test that wait_for_all_connections handles timeout correctly
TEST_F( HectorTestFixture, WaitForAllConnectionsTimeout )
{
  const std::string topic = "/no_connection_test";

  // Create a publisher but no subscriber - it won't connect
  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>( topic );

  std::string diagnostic;
  auto start = std::chrono::steady_clock::now();
  bool result = tester_node_->wait_for_all_connections( *executor_, 1s, &diagnostic );
  auto duration = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE( result );
  EXPECT_GE( duration, 1s );
  EXPECT_FALSE( diagnostic.empty() ); // Should contain diagnostic info
  EXPECT_NE( diagnostic.find( "Publisher" ), std::string::npos );
}

// Test spin_some doesn't block indefinitely
TEST_F( HectorTestFixture, SpinSomeNonBlocking )
{
  auto start = std::chrono::steady_clock::now();

  // Call spin_some multiple times - should be fast
  for ( int i = 0; i < 100; ++i ) { executor_->spin_some(); }

  auto duration = std::chrono::steady_clock::now() - start;

  // 100 spin_some calls should be very quick
  EXPECT_LT( duration, 100ms );
}

// Test that predicates are evaluated correctly during wait
TEST_F( HectorTestFixture, PredicateEvaluationDuringWait )
{
  const std::string topic = "/predicate_timing_test";

  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>( topic );
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>( topic );

  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 5s ) );

  // Publish messages in background thread
  std::thread publisher_thread( [&pub]() {
    for ( int i = 1; i <= 20; ++i ) {
      std_msgs::msg::Int32 msg;
      msg.data = i;
      pub->publish( msg );
      std::this_thread::sleep_for( 50ms );
    }
  } );

  // Wait for specific value (15)
  auto start = std::chrono::steady_clock::now();
  bool result = sub->wait_for_message(
      *executor_, 5s, []( const std_msgs::msg::Int32 &msg ) { return msg.data == 15; } );
  auto duration = std::chrono::steady_clock::now() - start;

  publisher_thread.join();

  EXPECT_TRUE( result );
  auto received = sub->last_message();
  ASSERT_TRUE( received.has_value() );
  EXPECT_EQ( received->data, 15 );
  // Should take roughly 15 * 50ms = 750ms
  EXPECT_GE( duration, 700ms );
  EXPECT_LT( duration, 1500ms );
}

// Test CachedSubscriber message count increments correctly
TEST_F( HectorTestFixture, MessageCountIncrement )
{
  const std::string topic = "/message_count_test";

  auto sub_node = std::make_shared<rclcpp::Node>( "message_count_node" );
  executor_->add_node( sub_node );

  auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>( topic, 10 );
  hector_testing_utils::CachedSubscriber<std_msgs::msg::Int32> sub( sub_node, topic );

  ASSERT_TRUE( sub.wait_for_publishers( *executor_, 1, 5s ) );

  EXPECT_EQ( sub.message_count(), 0u );

  // Publish 5 messages
  for ( int i = 0; i < 5; ++i ) {
    std_msgs::msg::Int32 msg;
    msg.data = i;
    pub->publish( msg );
    std::this_thread::sleep_for( 20ms );
    executor_->spin_some();
  }

  // Give time for all messages to arrive
  std::this_thread::sleep_for( 200ms );
  executor_->spin_some();

  EXPECT_GE( sub.message_count(), 5u );
}

// Test reset functionality
TEST_F( HectorTestFixture, ResetFunctionality )
{
  const std::string topic = "/reset_test";

  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>( topic );
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>( topic );

  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 5s ) );

  // Send first message
  std_msgs::msg::Int32 msg1;
  msg1.data = 100;
  pub->publish( msg1 );

  ASSERT_TRUE( sub->wait_for_message( *executor_, 1s ) );
  EXPECT_TRUE( sub->last_message().has_value() );

  // Reset
  sub->reset();
  EXPECT_FALSE( sub->last_message().has_value() );

  // Send second message
  std_msgs::msg::Int32 msg2;
  msg2.data = 200;
  pub->publish( msg2 );

  ASSERT_TRUE( sub->wait_for_message( *executor_, 1s ) );
  auto received = sub->last_message();
  ASSERT_TRUE( received.has_value() );
  EXPECT_EQ( received->data, 200 );
}
