#include <example_interfaces/srv/add_two_ints.hpp>
#include <gtest/gtest.h>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>

using hector_testing_utils::HectorTestFixture;
using namespace std::chrono_literals;

class BackgroundSpinnerTest : public HectorTestFixture
{
};

TEST_F( BackgroundSpinnerTest, SpinsInBackground )
{
  auto node = std::make_shared<rclcpp::Node>( "bg_test_node" );
  executor_->add_node( node );

  int callback_count = 0;
  auto timer = node->create_wall_timer( std::chrono::milliseconds( 10 ),
                                        [&callback_count]() { callback_count++; } );

  // Verify that calling start multiple times is safe (idempotent)
  executor_->start_background_spinner();
  executor_->start_background_spinner();

  // Wait for some callbacks to accumulate
  std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

  // Verify that calling stop multiple times is safe
  executor_->stop_background_spinner();
  executor_->stop_background_spinner();

  ASSERT_GT( callback_count, 0 );
}

TEST_F( BackgroundSpinnerTest, SpinSomeWhileBackgroundSpinning )
{
  executor_->start_background_spinner();
  EXPECT_THROW( executor_->spin_some(), std::runtime_error );
  executor_->stop_background_spinner();
}

TEST_F( BackgroundSpinnerTest, SpinUntilFutureCompleteWorks )
{
  auto node = std::make_shared<rclcpp::Node>( "spin_until_future_node" );
  executor_->add_node( node );

  auto client = node->create_client<example_interfaces::srv::AddTwoInts>( "add_two_ints" );
  auto service = node->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints", []( const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> req,
                          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> res ) {
        res->sum = req->a + req->b;
      } );

  executor_->start_background_spinner();

  auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  req->a = 5;
  req->b = 7;

  auto future = client->async_send_request( req );

  // This should work even with background spinner (passive wait)
  ASSERT_TRUE( executor_->spin_until_future_complete( future, 5s ) );
  EXPECT_EQ( future.get()->sum, 12 );

  executor_->stop_background_spinner();
}

TEST_F( BackgroundSpinnerTest, ScopedSpinnerWorks )
{
  auto node = std::make_shared<rclcpp::Node>( "scoped_test_node" );
  executor_->add_node( node );

  int callback_count = 0;
  auto timer = node->create_wall_timer( std::chrono::milliseconds( 10 ),
                                        [&callback_count]() { callback_count++; } );

  {
    hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
    std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
  }

  // Should stop spinning now
  int count_after = callback_count;
  std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
  ASSERT_EQ( callback_count, count_after );
  ASSERT_GT( callback_count, 0 );
}

TEST_F( BackgroundSpinnerTest, WaitHelpersWorkWithBackgroundSpinner )
{
  // Verify that wait_for_log works even when background spinner is active
  // This tests the "waiter" mode of spin_until

  hector_testing_utils::LogCapture capture;
  executor_->start_background_spinner();

  RCLCPP_INFO( tester_node_->get_logger(), "Async Log" );

  ASSERT_TRUE( capture.wait_for_log( *executor_, "Async Log" ) );

  executor_->stop_background_spinner();
}

#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_msgs/msg/int32.hpp>

using namespace std::chrono_literals;

TEST_F( BackgroundSpinnerTest, WaitForMessageWorks )
{
  auto node = std::make_shared<rclcpp::Node>( "wait_for_message_test_node" );
  executor_->add_node( node );

  const std::string topic = "/test_topic";
  auto pub = node->create_publisher<std_msgs::msg::Int32>( topic, 10 );
  hector_testing_utils::TestSubscription<std_msgs::msg::Int32> sub( node, topic );

  // Start background spinning
  executor_->start_background_spinner();
  ASSERT_TRUE( sub.wait_for_publishers( *executor_, 1, 5s ) );

  std_msgs::msg::Int32 msg;
  msg.data = 42;
  pub->publish( msg );

  ASSERT_TRUE( sub.wait_for_message( *executor_, 5s ) );
  auto last = sub.last_message();
  ASSERT_TRUE( last.has_value() );
  EXPECT_EQ( last->data, 42 );

  executor_->stop_background_spinner();
}

TEST_F( BackgroundSpinnerTest, WaitForServiceWorks )
{
  auto node = std::make_shared<rclcpp::Node>( "wait_for_service_test_node" );
  executor_->add_node( node );

  auto service = node->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints", []( const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response ) {
        response->sum = request->a + request->b;
      } );

  auto client = node->create_client<example_interfaces::srv::AddTwoInts>( "add_two_ints" );

  // Start background spinning
  executor_->start_background_spinner();

  // Wait for service to be available
  ASSERT_TRUE( client->wait_for_service( std::chrono::seconds( 5 ) ) );

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 10;
  request->b = 20;

  auto future = client->async_send_request( request );
  // Wait for the future to retrieve result
  // Note: we can't use spin_until_future_complete easily with background spinner if the future
  // depends on the same executor spinning But here the client is spinning in background, so the
  // future will become ready automatically. We just wait for it.

  ASSERT_EQ( future.wait_for( std::chrono::seconds( 5 ) ), std::future_status::ready );
  EXPECT_EQ( future.get()->sum, 30 );

  executor_->stop_background_spinner();
}
