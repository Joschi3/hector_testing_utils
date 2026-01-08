#include <gtest/gtest.h>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>

using hector_testing_utils::HectorTestFixture; // Not needing to override it directly since we use the fixture helper

class MultiThreadedJwt : public HectorTestFixture
{
protected:
  std::shared_ptr<rclcpp::Executor> create_test_executor() override
  {
    return std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  }
};

TEST_F( MultiThreadedJwt, RunsMultiThreaded )
{
  // Just verify it doesn't crash and we can call things
  auto node = std::make_shared<rclcpp::Node>( "mt_test_node" );
  executor_->add_node( node );

  std::atomic<bool> timer_called{ false };
  auto timer = node->create_wall_timer( std::chrono::milliseconds( 10 ),
                                        [&timer_called]() { timer_called = true; } );

  ASSERT_TRUE( executor_->spin_until( [&timer_called]() { return timer_called.load(); },
                                      std::chrono::seconds( 1 ) ) );
}
