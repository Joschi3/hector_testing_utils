#ifndef HECTOR_TESTING_UTILS_TEST_CONTEXT_HPP
#define HECTOR_TESTING_UTILS_TEST_CONTEXT_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace hector_testing_utils
{

/// @brief Helper class to manage a dedicated ROS 2 context for testing.
///
/// This class handles the initialization and shutdown of a `rclcpp::Context`.
class TestContext
{
public:
  /// @brief Construct a new Test Context object
  ///
  /// @param argc Argument count for init
  /// @param argv Argument vector for init
  explicit TestContext( int argc = 0, char **argv = nullptr )
  {
    context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    init_options.auto_initialize_logging( false );
    context_->init( argc, argv, init_options );
  }

  /// @brief Destroy the Test Context object and shutdown the context.
  ~TestContext()
  {
    if ( context_ && context_->is_valid() ) {
      rclcpp::shutdown( context_ );
    }
  }

  // Non-copyable
  TestContext( const TestContext & ) = delete;
  TestContext &operator=( const TestContext & ) = delete;

  // Movable
  TestContext( TestContext && ) = default;
  TestContext &operator=( TestContext && ) = default;

  /// @brief Get the shared pointer to the underlying rclcpp::Context.
  rclcpp::Context::SharedPtr context() const { return context_; }

  /// @brief Get node options configured with this context.
  rclcpp::NodeOptions node_options() const
  {
    rclcpp::NodeOptions options;
    options.context( context_ );
    return options;
  }

private:
  rclcpp::Context::SharedPtr context_;
};

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_TEST_CONTEXT_HPP
