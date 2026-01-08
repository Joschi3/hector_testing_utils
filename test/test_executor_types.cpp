#include <chrono>
#include <gtest/gtest.h>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
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
  // Verify that the multi-threaded executor runs without crashing and can process callbacks
  auto node = std::make_shared<rclcpp::Node>( "mt_test_node" );
  executor_->add_node( node );

  std::atomic<bool> timer_called{ false };
  auto timer = node->create_wall_timer( std::chrono::milliseconds( 10 ),
                                        [&timer_called]() { timer_called = true; } );

  ASSERT_TRUE( executor_->spin_until( [&timer_called]() { return timer_called.load(); },
                                      std::chrono::seconds( 1 ) ) );
}

class NullExecutorFixture : public HectorTestFixture
{
protected:
  // Expose SetUp for testing
  void SetUp() override { HectorTestFixture::SetUp(); }
  std::shared_ptr<rclcpp::Executor> create_test_executor() override { return nullptr; }

public:
  void runSetUp() { SetUp(); }
  void TestBody() override { }
};

TEST( FixtureTests, FixtureThrowsOnNullExecutor )
{
  NullExecutorFixture fixture;
  EXPECT_THROW( fixture.runSetUp(), std::runtime_error );
}

TEST_F( HectorTestFixture, SpinUntilReturnsFalseOnContextShutdown )
{
  std::thread shutdown_thread( [this]() {
    std::this_thread::sleep_for( 200ms );
    // Shutdown the global context used by the executor in HectorTestFixture
    if ( rclcpp::ok() ) {
      rclcpp::shutdown();
    }
  } );

  // spin_until should return false when rclcpp::ok() becomes false.
  // We pass a predicate that is never true to force reliance on the loop condition checking rclcpp::ok().
  bool result = executor_->spin_until( []() { return false; }, 2s );

  shutdown_thread.join();
  EXPECT_FALSE( result );

  // Note: Since we called rclcpp::shutdown() globally, subsequent tests in this process
  // might be affected if they expect a valid context. However, gtest fixtures typically
  // handle re-initialization if set up correctly. HectorTestFixture checks rclcpp::ok() in SetUp.
}
