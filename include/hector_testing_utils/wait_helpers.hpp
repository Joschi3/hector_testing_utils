#ifndef HECTOR_TESTING_UTILS_WAIT_HELPERS_HPP
#define HECTOR_TESTING_UTILS_WAIT_HELPERS_HPP

#include <chrono>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <hector_testing_utils/constants.hpp>
#include <hector_testing_utils/test_executor.hpp>

namespace hector_testing_utils
{

// =============================================================================
// Topic and Service/Action Helpers
// =============================================================================

/// @brief Wait until the topic has at least min_publishers publishers.
/// @param executor The executor to spin.
/// @param node The node to use for counting.
/// @param topic The topic name.
/// @param min_publishers Minimum required publishers.
/// @param timeout Maximum wait time.
/// @return true if condition met, false on timeout.
inline bool wait_for_publishers( TestExecutor &executor, const rclcpp::Node::SharedPtr &node,
                                 const std::string &topic, size_t min_publishers,
                                 std::chrono::nanoseconds timeout )
{
  return executor.spin_until(
      [&node, &topic, min_publishers]() { return node->count_publishers( topic ) >= min_publishers; },
      timeout );
}

/// @brief Wait until the topic has at least min_subscribers subscribers.
/// @param executor The executor to spin.
/// @param node The node to use for counting.
/// @param topic The topic name.
/// @param min_subscribers Minimum required subscribers.
/// @param timeout Maximum wait time.
/// @return true if condition met, false on timeout.
inline bool wait_for_subscribers( TestExecutor &executor, const rclcpp::Node::SharedPtr &node,
                                  const std::string &topic, size_t min_subscribers,
                                  std::chrono::nanoseconds timeout )
{
  return executor.spin_until(
      [&node, &topic, min_subscribers]() {
        return node->count_subscribers( topic ) >= min_subscribers;
      },
      timeout );
}

/// @brief Wait for a service to appear on the ROS graph.
/// @tparam ServiceT The service type.
/// @param client The service client.
/// @param executor The executor to spin.
/// @param timeout Maximum wait time.
/// @return true if service became available, false on timeout.
template<typename ServiceT>
bool wait_for_service( const typename rclcpp::Client<ServiceT>::SharedPtr &client,
                       TestExecutor &executor, std::chrono::nanoseconds timeout )
{
  return executor.spin_until(
      [&client]() { return client->wait_for_service( std::chrono::nanoseconds( 0 ) ); }, timeout );
}

/// @brief Wait for an action server to appear on the ROS graph.
/// @tparam ActionT The action type.
/// @param client The action client.
/// @param executor The executor to spin.
/// @param timeout Maximum wait time.
/// @return true if action server became available, false on timeout.
template<typename ActionT>
bool wait_for_action_server( const typename rclcpp_action::Client<ActionT>::SharedPtr &client,
                             TestExecutor &executor, std::chrono::nanoseconds timeout )
{
  return executor.spin_until(
      [&client]() { return client->wait_for_action_server( std::chrono::nanoseconds( 0 ) ); },
      timeout );
}

// =============================================================================
// Service Call Helper
// =============================================================================

/// @brief Options for service calls.
struct ServiceCallOptions {
  std::chrono::nanoseconds service_timeout{ kDefaultTimeout };
  std::chrono::nanoseconds response_timeout{ kDefaultTimeout };
};

/// @brief Call a service and wait for the response.
/// @tparam ServiceT The service type.
/// @param client The service client.
/// @param request The request to send.
/// @param executor The executor to spin.
/// @param options Call options (timeouts).
/// @return The response, or nullptr on failure.
template<typename ServiceT>
typename ServiceT::Response::SharedPtr
call_service( const typename rclcpp::Client<ServiceT>::SharedPtr &client,
              const typename ServiceT::Request::SharedPtr &request, TestExecutor &executor,
              const ServiceCallOptions &options = ServiceCallOptions() )
{
  if ( !wait_for_service<ServiceT>( client, executor, options.service_timeout ) ) {
    return nullptr;
  }
  auto future = client->async_send_request( request );
  if ( !executor.spin_until_future_complete( future, options.response_timeout ) ) {
    return nullptr;
  }
  return future.get();
}

// =============================================================================
// Action Call Helper
// =============================================================================

/// @brief Options for action calls.
struct ActionCallOptions {
  std::chrono::nanoseconds server_timeout{ kDefaultTimeout };
  std::chrono::nanoseconds goal_timeout{ kDefaultTimeout };
  std::chrono::nanoseconds result_timeout{ std::chrono::seconds( 30 ) };
};

/// @brief Send an action goal and wait for the result.
/// @tparam ActionT The action type.
/// @param client The action client.
/// @param goal The goal to send.
/// @param executor The executor to spin.
/// @param options Call options (timeouts).
/// @return The wrapped result, or std::nullopt on failure.
template<typename ActionT>
std::optional<typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult>
call_action( const typename rclcpp_action::Client<ActionT>::SharedPtr &client,
             const typename ActionT::Goal &goal, TestExecutor &executor,
             const ActionCallOptions &options = ActionCallOptions() )
{
  if ( !wait_for_action_server<ActionT>( client, executor, options.server_timeout ) ) {
    return std::nullopt;
  }

  auto goal_future = client->async_send_goal( goal );
  if ( !executor.spin_until_future_complete( goal_future, options.goal_timeout ) ) {
    return std::nullopt;
  }
  auto goal_handle = goal_future.get();
  if ( !goal_handle ) {
    return std::nullopt;
  }

  auto result_future = client->async_get_result( goal_handle );
  if ( !executor.spin_until_future_complete( result_future, options.result_timeout ) ) {
    return std::nullopt;
  }
  return result_future.get();
}

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_WAIT_HELPERS_HPP
