#ifndef HECTOR_TESTING_UTILS_ASSERTIONS_HPP
#define HECTOR_TESTING_UTILS_ASSERTIONS_HPP

#include <chrono>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <hector_testing_utils/test_executor.hpp>

namespace hector_testing_utils
{

// =============================================================================
// Assertion Functions
// =============================================================================

/// @brief Assert that a service exists on the ROS graph within a timeout.
/// @param executor The executor to spin.
/// @param node The node to use for graph queries.
/// @param service_name The service name to look for.
/// @param timeout Maximum wait time.
/// @return AssertionSuccess if found, AssertionFailure otherwise.
inline ::testing::AssertionResult assert_service_exists( TestExecutor &executor,
                                                         rclcpp::Node::SharedPtr node,
                                                         const std::string &service_name,
                                                         std::chrono::nanoseconds timeout )
{
  bool found = executor.spin_until(
      [&]() {
        auto services = node->get_service_names_and_types();
        return services.find( service_name ) != services.end();
      },
      timeout );

  if ( found )
    return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "Service '" << service_name << "' failed to appear.";
}

/// @brief Assert that an action server exists on the ROS graph within a timeout.
/// @param executor The executor to spin.
/// @param node The node to use for graph queries.
/// @param action_name The action name to look for.
/// @param timeout Maximum wait time.
/// @return AssertionSuccess if found, AssertionFailure otherwise.
inline ::testing::AssertionResult assert_action_server_exists( TestExecutor &executor,
                                                               rclcpp::Node::SharedPtr node,
                                                               const std::string &action_name,
                                                               std::chrono::nanoseconds timeout )
{
  // Note: checking graph for actions is tricky because actions expand to multiple topics/services.
  // Reliable way: create a temp action client and check readiness, or check for the 'action_name/_action/status' topic.
  // Here we check for the status topic which every action server publishes.
  std::string status_topic = action_name + "/_action/status";

  bool found = executor.spin_until(
      [&]() {
        auto topics = node->get_topic_names_and_types();
        return topics.find( status_topic ) != topics.end();
      },
      timeout );

  if ( found )
    return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "Action Server '" << action_name << "' failed to appear.";
}

// =============================================================================
// Macros
// =============================================================================

// clang-format off

/// @brief Assert that a service exists (uses executor_ from test fixture).
#define ASSERT_SERVICE_EXISTS( node, service, timeout )                                            \
  ASSERT_TRUE( hector_testing_utils::assert_service_exists(                                        \
      *executor_, node, service, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

/// @brief Expect that a service exists (uses executor_ from test fixture).
#define EXPECT_SERVICE_EXISTS( node, service, timeout )                                            \
  EXPECT_TRUE( hector_testing_utils::assert_service_exists(                                        \
      *executor_, node, service, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

/// @brief Assert that an action server exists (uses executor_ from test fixture).
#define ASSERT_ACTION_EXISTS( node, action, timeout )                                              \
  ASSERT_TRUE( hector_testing_utils::assert_action_server_exists(                                  \
      *executor_, node, action, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

/// @brief Expect that an action server exists (uses executor_ from test fixture).
#define EXPECT_ACTION_EXISTS( node, action, timeout )                                              \
  EXPECT_TRUE( hector_testing_utils::assert_action_server_exists(                                  \
      *executor_, node, action, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

/// @brief Assert that a service exists (with explicit executor).
#define ASSERT_SERVICE_EXISTS_WITH_EXECUTOR( executor, node, service, timeout )                    \
  ASSERT_TRUE( hector_testing_utils::assert_service_exists(                                        \
      executor, node, service, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

/// @brief Expect that a service exists (with explicit executor).
#define EXPECT_SERVICE_EXISTS_WITH_EXECUTOR( executor, node, service, timeout )                    \
  EXPECT_TRUE( hector_testing_utils::assert_service_exists(                                        \
      executor, node, service, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

/// @brief Assert that an action server exists (with explicit executor).
#define ASSERT_ACTION_EXISTS_WITH_EXECUTOR( executor, node, action, timeout )                      \
  ASSERT_TRUE( hector_testing_utils::assert_action_server_exists(                                  \
      executor, node, action, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

/// @brief Expect that an action server exists (with explicit executor).
#define EXPECT_ACTION_EXISTS_WITH_EXECUTOR( executor, node, action, timeout )                      \
  EXPECT_TRUE( hector_testing_utils::assert_action_server_exists(                                  \
      executor, node, action, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

// clang-format on

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_ASSERTIONS_HPP
