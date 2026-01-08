#include <gtest/gtest.h>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>

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

TEST_F( LogCaptureTest, ClearsLogs )
{
  LogCapture capture;
  auto node = std::make_shared<rclcpp::Node>( "log_test_node" );

  RCLCPP_WARN( node->get_logger(), "Warning to clear" );
  ASSERT_TRUE( capture.has_log( "Warning" ) );

  capture.clear();
  ASSERT_FALSE( capture.has_log( "Warning" ) );
}
