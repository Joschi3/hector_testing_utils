#include <chrono>
#include <gtest/gtest.h>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using hector_testing_utils::LogCapture;

class LogCaptureTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init( 0, nullptr ); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F( LogCaptureTest, CapturesLogs )
{
  LogCapture capture;
  auto node = std::make_shared<rclcpp::Node>( "log_test_node" );

  RCLCPP_INFO( node->get_logger(), "Test message 123" );

  hector_testing_utils::TestExecutor executor;
  ASSERT_TRUE( capture.wait_for_log( executor, "Test message 123" ) );
  ASSERT_TRUE( capture.has_log( "123" ) );
  ASSERT_FALSE( capture.has_log( "non_existent" ) );
}

TEST_F( LogCaptureTest, WaitForLogTimeout )
{
  LogCapture capture;
  hector_testing_utils::TestExecutor executor;

  auto start = std::chrono::steady_clock::now();
  // Wait for a log message that is never published to verify timeout behavior
  bool result = capture.wait_for_log( executor, "This will never appear", 200ms );
  auto duration = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE( result );
  EXPECT_GE( duration, 200ms );
}

TEST_F( LogCaptureTest, ClearsLogs )
{
  LogCapture capture;
  auto node = std::make_shared<rclcpp::Node>( "log_test_node" );

  RCLCPP_WARN( node->get_logger(), "Warning to clear" );
  ASSERT_TRUE( capture.has_log( "Warning" ) );

  capture.clear();
  ASSERT_FALSE( capture.has_log( "Warning" ) );
}

TEST_F( LogCaptureTest, SingletonEnforcement )
{
  LogCapture capture1;
  EXPECT_THROW( { LogCapture capture2; }, std::runtime_error );
}

#include <std_msgs/msg/string.hpp>

// Verify wait_for_all_connections logs warning/error on timeout
class ConnectWarningTest : public hector_testing_utils::HectorTestFixture
{
};

TEST_F( ConnectWarningTest, WaitForAllConnectionsWarnings )
{
  LogCapture capture;
  const std::string topic = "/logging_test_topic";

  // Create publisher but no subscriber
  auto pub = tester_node_->create_test_publisher<std_msgs::msg::String>( topic );

  // Capture logs while waiting - should timeout and log ERROR
  // Using 1s timeout
  bool result = tester_node_->wait_for_all_connections( *executor_, std::chrono::seconds( 3 ) );

  EXPECT_FALSE( result );
  // It should have logged "Timeout waiting for connections"
  EXPECT_TRUE( capture.has_log( "Timeout waiting for connections" ) );
  EXPECT_TRUE( capture.has_log( "Publisher: /logging_test_topic" ) );
  EXPECT_FALSE( capture.has_log( "Subscriber: /does_not_exist" ) );
}
