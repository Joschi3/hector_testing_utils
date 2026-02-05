#ifndef HECTOR_TESTING_UTILS_TEST_EXECUTOR_HPP
#define HECTOR_TESTING_UTILS_TEST_EXECUTOR_HPP

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <hector_testing_utils/constants.hpp>

namespace hector_testing_utils
{

/// @brief Helper class to manage a SingleThreadedExecutor for testing.
///
/// It supports both active spinning (driving the executor via `spin_until`) and
/// passive waiting (if a background spinner is active).
class TestExecutor
{
public:
  /// @brief Construct a new Test Executor with a new context.
  TestExecutor() : context_( std::make_shared<rclcpp::Context>() )
  {
    rclcpp::InitOptions init_options;
    init_options.auto_initialize_logging( false );
    context_->init( 0, nullptr, init_options );
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(
        make_executor_options( context_ ) );
  }

  /// @brief Construct a new Test Executor with an existing context.
  /// @param context The context to use.
  explicit TestExecutor( const rclcpp::Context::SharedPtr &context ) : context_( context )
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(
        make_executor_options( context_ ) );
  }

  /// @brief Construct a new Test Executor with an existing executor.
  /// @param executor The executor to wrap.
  explicit TestExecutor( const std::shared_ptr<rclcpp::Executor> &executor )
      : context_( rclcpp::contexts::get_global_default_context() ), executor_( executor )
  {
  }

  ~TestExecutor() { stop_background_spinner(); }

  // Non-copyable
  TestExecutor( const TestExecutor & ) = delete;
  TestExecutor &operator=( const TestExecutor & ) = delete;

  /// @brief Add a node to the executor.
  /// @param node The node to add.
  void add_node( const rclcpp::Node::SharedPtr &node ) { executor_->add_node( node ); }

  /// @brief Spin until the predicate returns true or timeout is reached.
  ///
  /// If a background spinner is active, this method simply waits (sleeps) until the predicate is true.
  /// If no background spinner is active, this method calls `spin_some()` on the executor in a loop.
  ///
  /// @param predicate The condition to wait for.
  /// @param timeout Maximum duration to wait.
  /// @param spin_period Sleep duration between checks.
  /// @return true if predicate became true, false on timeout or context shutdown.
  bool spin_until( const std::function<bool()> &predicate,
                   std::chrono::nanoseconds timeout = kDefaultTimeout,
                   std::chrono::nanoseconds spin_period = kDefaultSpinPeriod )
  {
    const auto start = std::chrono::steady_clock::now();
    while ( rclcpp::ok( context_ ) ) {
      if ( predicate() )
        return true;
      if ( std::chrono::steady_clock::now() - start > timeout )
        return false;

      // If background spinner is active, we just wait (passive mode).
      // Otherwise, we drive the executor (active mode).
      if ( !background_spinner_thread_.joinable() ) {
        executor_->spin_some();
      }
      std::this_thread::sleep_for( spin_period );
    }
    return false;
  }

  /// @brief Wait until a future is complete or timeout is reached.
  ///
  /// Handles background spinning correctly by falling back to a predicate wait if needed.
  ///
  /// @tparam FutureT Type of the future.
  /// @param future The future to wait for.
  /// @param timeout Maximum duration to wait.
  /// @return true if future completed successfully, false otherwise.
  template<typename FutureT>
  bool spin_until_future_complete( FutureT &future,
                                   std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    // If background spinning is active, we can't use spin_until_future_complete because
    // it tries to spin the executor which is already spinning.
    // So we fall back to our predicate-based wait.
    if ( background_spinner_thread_.joinable() ) {
      return spin_until(
          [&future]() {
            return future.wait_for( std::chrono::seconds( 0 ) ) == std::future_status::ready;
          },
          timeout );
    }
    auto result = executor_->spin_until_future_complete( future, timeout );
    return result == rclcpp::FutureReturnCode::SUCCESS;
  }

  /// @brief Execute any available work.
  /// @throws std::runtime_error if background spinner is active.
  void spin_some()
  {
    if ( background_spinner_thread_.joinable() ) {
      throw std::runtime_error( "Cannot call spin_some() while background spinner is active!" );
    }
    executor_->spin_some();
  }

  /// @brief Start a background thread that continuously calls spin_some().
  ///
  /// This is useful for tests that need to wait for async events without manually calling spin().
  void start_background_spinner()
  {
    if ( background_spinner_thread_.joinable() ) {
      return; // Already spinning
    }
    stop_signal_ = false;
    background_spinner_thread_ = std::thread( [this]() {
      while ( rclcpp::ok( context_ ) && !stop_signal_ ) {
        executor_->spin_some();
        // Small sleep to yield if spin_some returns immediately (no work)
        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
      }
    } );
  }

  /// @brief Stop the background spinner thread.
  void stop_background_spinner()
  {
    if ( background_spinner_thread_.joinable() ) {
      stop_signal_ = true;
      executor_->cancel();
      background_spinner_thread_.join();
    }
  }

  /// @brief RAII Helper to start the background spinner on construction and stop it on destruction.
  struct ScopedSpinner {
    explicit ScopedSpinner( TestExecutor &exec ) : exec_( exec )
    {
      exec_.start_background_spinner();
    }
    ~ScopedSpinner() { exec_.stop_background_spinner(); }

    // Non-copyable, non-movable
    ScopedSpinner( const ScopedSpinner & ) = delete;
    ScopedSpinner &operator=( const ScopedSpinner & ) = delete;

  private:
    TestExecutor &exec_;
  };

private:
  static rclcpp::ExecutorOptions make_executor_options( const rclcpp::Context::SharedPtr &context )
  {
    rclcpp::ExecutorOptions options;
    options.context = context;
    return options;
  }

  rclcpp::Context::SharedPtr context_;
  std::shared_ptr<rclcpp::Executor> executor_;
  std::thread background_spinner_thread_;
  std::atomic<bool> stop_signal_{ false };
};

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_TEST_EXECUTOR_HPP
