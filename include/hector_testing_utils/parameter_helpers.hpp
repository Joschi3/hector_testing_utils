#ifndef HECTOR_TESTING_UTILS_PARAMETER_HELPERS_HPP
#define HECTOR_TESTING_UTILS_PARAMETER_HELPERS_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <hector_testing_utils/constants.hpp>
#include <hector_testing_utils/test_executor.hpp>

namespace hector_testing_utils
{

// =============================================================================
// Parameter Value Conversion Helpers
// =============================================================================

namespace detail
{

/// @brief Convert a value to rcl_interfaces::msg::ParameterValue.
template<typename T>
rcl_interfaces::msg::ParameterValue to_parameter_value( const T &value );

template<>
inline rcl_interfaces::msg::ParameterValue to_parameter_value( const bool &value )
{
  rcl_interfaces::msg::ParameterValue pv;
  pv.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  pv.bool_value = value;
  return pv;
}

template<>
inline rcl_interfaces::msg::ParameterValue to_parameter_value( const int &value )
{
  rcl_interfaces::msg::ParameterValue pv;
  pv.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  pv.integer_value = value;
  return pv;
}

template<>
inline rcl_interfaces::msg::ParameterValue to_parameter_value( const int64_t &value )
{
  rcl_interfaces::msg::ParameterValue pv;
  pv.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  pv.integer_value = value;
  return pv;
}

template<>
inline rcl_interfaces::msg::ParameterValue to_parameter_value( const double &value )
{
  rcl_interfaces::msg::ParameterValue pv;
  pv.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  pv.double_value = value;
  return pv;
}

template<>
inline rcl_interfaces::msg::ParameterValue to_parameter_value( const std::string &value )
{
  rcl_interfaces::msg::ParameterValue pv;
  pv.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  pv.string_value = value;
  return pv;
}

template<>
inline rcl_interfaces::msg::ParameterValue to_parameter_value( const char *const &value )
{
  rcl_interfaces::msg::ParameterValue pv;
  pv.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  pv.string_value = value;
  return pv;
}

template<>
inline rcl_interfaces::msg::ParameterValue to_parameter_value( const std::vector<bool> &value )
{
  rcl_interfaces::msg::ParameterValue pv;
  pv.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
  pv.bool_array_value = value;
  return pv;
}

template<>
inline rcl_interfaces::msg::ParameterValue to_parameter_value( const std::vector<int64_t> &value )
{
  rcl_interfaces::msg::ParameterValue pv;
  pv.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
  pv.integer_array_value = value;
  return pv;
}

template<>
inline rcl_interfaces::msg::ParameterValue to_parameter_value( const std::vector<double> &value )
{
  rcl_interfaces::msg::ParameterValue pv;
  pv.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
  pv.double_array_value = value;
  return pv;
}

template<>
inline rcl_interfaces::msg::ParameterValue to_parameter_value( const std::vector<std::string> &value )
{
  rcl_interfaces::msg::ParameterValue pv;
  pv.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  pv.string_array_value = value;
  return pv;
}

/// @brief Convert rcl_interfaces::msg::ParameterValue to a typed value.
template<typename T>
T from_parameter_value( const rcl_interfaces::msg::ParameterValue &pv );

template<>
inline bool from_parameter_value( const rcl_interfaces::msg::ParameterValue &pv )
{
  return pv.bool_value;
}

template<>
inline int64_t from_parameter_value( const rcl_interfaces::msg::ParameterValue &pv )
{
  return pv.integer_value;
}

template<>
inline int from_parameter_value( const rcl_interfaces::msg::ParameterValue &pv )
{
  return static_cast<int>( pv.integer_value );
}

template<>
inline double from_parameter_value( const rcl_interfaces::msg::ParameterValue &pv )
{
  return pv.double_value;
}

template<>
inline std::string from_parameter_value( const rcl_interfaces::msg::ParameterValue &pv )
{
  return pv.string_value;
}

template<>
inline std::vector<bool> from_parameter_value( const rcl_interfaces::msg::ParameterValue &pv )
{
  return pv.bool_array_value;
}

template<>
inline std::vector<int64_t> from_parameter_value( const rcl_interfaces::msg::ParameterValue &pv )
{
  return pv.integer_array_value;
}

template<>
inline std::vector<double> from_parameter_value( const rcl_interfaces::msg::ParameterValue &pv )
{
  return pv.double_array_value;
}

template<>
inline std::vector<std::string> from_parameter_value( const rcl_interfaces::msg::ParameterValue &pv )
{
  return pv.string_array_value;
}

} // namespace detail

// =============================================================================
// Remote Parameter Client
// =============================================================================

/// @brief Result of a parameter operation.
struct ParameterResult {
  bool success{ false }; ///< Whether the operation succeeded
  std::string reason;    ///< Reason for failure (if any)

  /// @brief Implicit conversion to bool.
  operator bool() const { return success; }
};

/// @brief Client for setting and getting parameters on remote nodes.
///
/// This class provides a convenient interface for manipulating parameters
/// on other nodes during testing. It handles the service client creation
/// and waiting internally.
///
/// Example usage:
/// @code
///   RemoteParameterClient params(my_node, "/target_node");
///
///   // Set a parameter
///   auto result = params.set_parameter(executor, "my_param", 42);
///   EXPECT_TRUE(result.success);
///
///   // Get a parameter
///   auto value = params.get_parameter<int>(executor, "my_param");
///   EXPECT_EQ(value, 42);
/// @endcode
class RemoteParameterClient
{
public:
  /// @brief Construct a new Remote Parameter Client.
  /// @param node The local node to use for creating service clients
  /// @param target_node_name The fully qualified name of the target node
  RemoteParameterClient( const rclcpp::Node::SharedPtr &node, const std::string &target_node_name )
      : node_( node ), target_node_name_( target_node_name )
  {
    // Ensure target node name starts with /
    if ( !target_node_name_.empty() && target_node_name_[0] != '/' ) {
      target_node_name_ = "/" + target_node_name_;
    }

    // Create service clients
    set_params_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(
        target_node_name_ + "/set_parameters" );
    get_params_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>(
        target_node_name_ + "/get_parameters" );
    list_params_client_ = node_->create_client<rcl_interfaces::srv::ListParameters>(
        target_node_name_ + "/list_parameters" );
  }

  /// @brief Wait for the parameter services to be available.
  /// @param executor The executor to spin
  /// @param timeout Maximum time to wait
  /// @return true if services are available, false on timeout
  bool wait_for_service( TestExecutor &executor, std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return executor.spin_until(
        [this]() {
          return set_params_client_->service_is_ready() && get_params_client_->service_is_ready() &&
                 list_params_client_->service_is_ready();
        },
        timeout );
  }

  /// @brief Set a parameter on the remote node.
  ///
  /// @tparam T The type of the parameter value
  /// @param executor The executor to spin
  /// @param name The parameter name
  /// @param value The parameter value
  /// @param timeout Maximum time to wait for the operation
  /// @return ParameterResult indicating success or failure
  template<typename T>
  ParameterResult set_parameter( TestExecutor &executor, const std::string &name, const T &value,
                                 std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    ParameterResult result;

    if ( !set_params_client_->service_is_ready() ) {
      result.reason = "Set parameters service not ready";
      return result;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter param;
    param.name = name;
    param.value = detail::to_parameter_value( value );
    request->parameters.push_back( param );

    auto future = set_params_client_->async_send_request( request );

    if ( !executor.spin_until_future_complete( future, timeout ) ) {
      result.reason = "Timeout waiting for set_parameters response";
      return result;
    }

    auto response = future.get();
    if ( response->results.empty() ) {
      result.reason = "Empty response from set_parameters";
      return result;
    }

    if ( response->results[0].successful ) {
      result.success = true;
    } else {
      result.reason = response->results[0].reason;
    }

    return result;
  }

  /// @brief Set multiple parameters on the remote node.
  ///
  /// @param executor The executor to spin
  /// @param parameters Vector of rclcpp::Parameter to set
  /// @param timeout Maximum time to wait for the operation
  /// @return ParameterResult indicating success or failure
  ParameterResult set_parameters( TestExecutor &executor,
                                  const std::vector<rclcpp::Parameter> &parameters,
                                  std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    ParameterResult result;

    if ( !set_params_client_->service_is_ready() ) {
      result.reason = "Set parameters service not ready";
      return result;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for ( const auto &param : parameters ) {
      request->parameters.push_back( param.to_parameter_msg() );
    }

    auto future = set_params_client_->async_send_request( request );

    if ( !executor.spin_until_future_complete( future, timeout ) ) {
      result.reason = "Timeout waiting for set_parameters response";
      return result;
    }

    auto response = future.get();

    result.success = true;
    for ( size_t i = 0; i < response->results.size(); ++i ) {
      if ( !response->results[i].successful ) {
        result.success = false;
        if ( !result.reason.empty() ) {
          result.reason += "; ";
        }
        result.reason += parameters[i].get_name() + ": " + response->results[i].reason;
      }
    }

    return result;
  }

  /// @brief Get a parameter from the remote node.
  ///
  /// @tparam T The expected type of the parameter value
  /// @param executor The executor to spin
  /// @param name The parameter name
  /// @param timeout Maximum time to wait for the operation
  /// @return The parameter value, or default value if not found
  template<typename T>
  std::optional<T> get_parameter( TestExecutor &executor, const std::string &name,
                                  std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    if ( !get_params_client_->service_is_ready() ) {
      return std::nullopt;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back( name );

    auto future = get_params_client_->async_send_request( request );

    if ( !executor.spin_until_future_complete( future, timeout ) ) {
      return std::nullopt;
    }

    auto response = future.get();
    if ( response->values.empty() ||
         response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET ) {
      return std::nullopt;
    }

    return detail::from_parameter_value<T>( response->values[0] );
  }

  /// @brief List all parameters on the remote node.
  ///
  /// @param executor The executor to spin
  /// @param prefixes Only list parameters with these prefixes (empty = all)
  /// @param timeout Maximum time to wait for the operation
  /// @return Vector of parameter names, or empty vector on failure
  std::vector<std::string> list_parameters( TestExecutor &executor,
                                            const std::vector<std::string> &prefixes = {},
                                            std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    if ( !list_params_client_->service_is_ready() ) {
      return {};
    }

    auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    request->prefixes = prefixes;
    request->depth = 0; // Unlimited depth

    auto future = list_params_client_->async_send_request( request );

    if ( !executor.spin_until_future_complete( future, timeout ) ) {
      return {};
    }

    auto response = future.get();
    return response->result.names;
  }

  /// @brief Check if a parameter exists on the remote node.
  ///
  /// @param executor The executor to spin
  /// @param name The parameter name
  /// @param timeout Maximum time to wait for the operation
  /// @return true if parameter exists, false otherwise
  bool has_parameter( TestExecutor &executor, const std::string &name,
                      std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    auto params = list_parameters( executor, { name }, timeout );
    for ( const auto &param : params ) {
      if ( param == name ) {
        return true;
      }
    }
    return false;
  }

  /// @brief Get the target node name.
  [[nodiscard]] const std::string &target_node_name() const { return target_node_name_; }

private:
  rclcpp::Node::SharedPtr node_;
  std::string target_node_name_;

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_params_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_params_client_;
  rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_params_client_;
};

// =============================================================================
// Convenience Functions
// =============================================================================

/// @brief Set a parameter on a remote node.
///
/// This is a convenience function that creates a temporary RemoteParameterClient.
///
/// @tparam T The type of the parameter value
/// @param node The local node to use for creating service clients
/// @param target_node_name The fully qualified name of the target node
/// @param param_name The parameter name
/// @param value The parameter value
/// @param executor The executor to spin
/// @param timeout Maximum time to wait for the operation
/// @return ParameterResult indicating success or failure
template<typename T>
ParameterResult
set_remote_parameter( const rclcpp::Node::SharedPtr &node, const std::string &target_node_name,
                      const std::string &param_name, const T &value, TestExecutor &executor,
                      std::chrono::nanoseconds timeout = kDefaultTimeout )
{
  RemoteParameterClient client( node, target_node_name );
  if ( !client.wait_for_service( executor, timeout ) ) {
    return { false, "Timeout waiting for parameter services on " + target_node_name };
  }
  return client.set_parameter( executor, param_name, value, timeout );
}

/// @brief Get a parameter from a remote node.
///
/// This is a convenience function that creates a temporary RemoteParameterClient.
///
/// @tparam T The expected type of the parameter value
/// @param node The local node to use for creating service clients
/// @param target_node_name The fully qualified name of the target node
/// @param param_name The parameter name
/// @param executor The executor to spin
/// @param timeout Maximum time to wait for the operation
/// @return The parameter value, or nullopt if not found
template<typename T>
std::optional<T> get_remote_parameter( const rclcpp::Node::SharedPtr &node,
                                       const std::string &target_node_name,
                                       const std::string &param_name, TestExecutor &executor,
                                       std::chrono::nanoseconds timeout = kDefaultTimeout )
{
  RemoteParameterClient client( node, target_node_name );
  if ( !client.wait_for_service( executor, timeout ) ) {
    return std::nullopt;
  }
  return client.get_parameter<T>( executor, param_name, timeout );
}

/// @brief List parameters on a remote node.
///
/// This is a convenience function that creates a temporary RemoteParameterClient.
///
/// @param node The local node to use for creating service clients
/// @param target_node_name The fully qualified name of the target node
/// @param executor The executor to spin
/// @param prefixes Only list parameters with these prefixes (empty = all)
/// @param timeout Maximum time to wait for the operation
/// @return Vector of parameter names, or empty vector on failure
inline std::vector<std::string>
list_remote_parameters( const rclcpp::Node::SharedPtr &node, const std::string &target_node_name,
                        TestExecutor &executor, const std::vector<std::string> &prefixes = {},
                        std::chrono::nanoseconds timeout = kDefaultTimeout )
{
  RemoteParameterClient client( node, target_node_name );
  if ( !client.wait_for_service( executor, timeout ) ) {
    return {};
  }
  return client.list_parameters( executor, prefixes, timeout );
}

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_PARAMETER_HELPERS_HPP
