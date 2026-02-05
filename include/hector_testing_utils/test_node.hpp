#ifndef HECTOR_TESTING_UTILS_TEST_NODE_HPP
#define HECTOR_TESTING_UTILS_TEST_NODE_HPP

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <hector_testing_utils/constants.hpp>
#include <hector_testing_utils/graph_introspection.hpp>
#include <hector_testing_utils/test_executor.hpp>
#include <hector_testing_utils/test_wrappers.hpp>

namespace hector_testing_utils
{

using namespace std::chrono_literals;

/// @brief Extended ROS 2 Node that provides factory methods for test wrappers.
///
/// This node allows creating wrapped publishers, subscribers, clients, and servers that
/// automatically register themselves for connection monitoring.
class TestNode : public rclcpp::Node
{
public:
  explicit TestNode( const std::string &name_space, const std::string &node_name = "test_node" )
      : Node( node_name, name_space )
  {
  }

  TestNode( const std::string &name_space, const std::string &node_name,
            const rclcpp::NodeOptions &options )
      : Node( node_name, name_space, options )
  {
  }

  explicit TestNode( const std::string &name_space, const rclcpp::NodeOptions &options )
      : Node( "test_node", name_space, options )
  {
  }

  // Factory methods that register the objects

  /// @brief Create a TestPublisher and register it.
  template<typename MsgT>
  std::shared_ptr<TestPublisher<MsgT>>
  create_test_publisher( const std::string &topic,
                         const rclcpp::QoS &qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ) )
  {
    auto ptr = std::make_shared<TestPublisher<MsgT>>( shared_from_this(), topic, qos );
    register_connectable( ptr );
    return ptr;
  }

  template<typename MsgT>
  std::shared_ptr<TestSubscription<MsgT>>
  create_test_subscription( const std::string &topic,
                            const rclcpp::QoS &qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ),
                            bool latched = false )
  {
    auto ptr = std::make_shared<TestSubscription<MsgT>>( shared_from_this(), topic, qos, latched );
    register_connectable( ptr );
    return ptr;
  }

  template<typename ServiceT>
  std::shared_ptr<TestClient<ServiceT>> create_test_client( const std::string &service_name )
  {
    auto ptr = std::make_shared<TestClient<ServiceT>>( shared_from_this(), service_name );
    register_connectable( ptr );
    return ptr;
  }

  template<typename ActionT>
  std::shared_ptr<TestActionClient<ActionT>> create_test_action_client( const std::string &action_name )
  {
    auto ptr = std::make_shared<TestActionClient<ActionT>>( shared_from_this(), action_name );
    register_connectable( ptr );
    return ptr;
  }

  template<typename ServiceT>
  std::shared_ptr<TestServiceServer<ServiceT>>
  create_test_service_server( const std::string &service_name,
                              typename TestServiceServer<ServiceT>::Callback callback,
                              const rclcpp::QoS &qos = rclcpp::ServicesQoS() )
  {
    auto ptr = std::make_shared<TestServiceServer<ServiceT>>( shared_from_this(), service_name,
                                                              callback, qos );
    register_connectable( ptr );
    return ptr;
  }

  template<typename ActionT>
  std::shared_ptr<TestActionServer<ActionT>>
  create_test_action_server( const std::string &action_name,
                             typename TestActionServer<ActionT>::GoalCallback goal_cb,
                             typename TestActionServer<ActionT>::CancelCallback cancel_cb,
                             typename TestActionServer<ActionT>::AcceptedCallback accepted_cb )
  {
    auto ptr = std::make_shared<TestActionServer<ActionT>>( shared_from_this(), action_name,
                                                            goal_cb, cancel_cb, accepted_cb );
    register_connectable( ptr );
    return ptr;
  }

  // The main wait function

  /// @brief Wait for all registered connectables to establish connections.
  ///
  /// Monitors all created test wrappers (publishers, subscribers, etc.) and waits until
  /// their `is_connected()` returns true. On timeout, logs suggestions for similar entities.
  ///
  /// @param executor The executor to spin.
  /// @param timeout Maximum wait time.
  /// @return true if all connected, false on timeout.
  bool wait_for_all_connections( TestExecutor &executor,
                                 std::chrono::seconds timeout = std::chrono::seconds( 10 ) )
  {
    std::vector<WaitFailureInfo> failure_infos;
    const bool success = wait_for_all_connections( executor, timeout, failure_infos );
    if ( !success ) {
      RCLCPP_ERROR( this->get_logger(), "%s",
                    format_failure_infos( failure_infos, kDefaultMaxSuggestions ).c_str() );
    }
    return success;
  }

  /// @brief Wait for all registered connectables to establish connections, with detailed failure info.
  ///
  /// This overload provides detailed suggestions for each failed connection, helping
  /// identify similar entities in the ROS graph that might be what the user intended.
  ///
  /// @param executor The executor to spin.
  /// @param timeout Maximum wait time.
  /// @param failure_infos Reference to vector to receive failure information for each disconnected item.
  /// @return true if all connected, false on timeout.
  bool wait_for_all_connections( TestExecutor &executor, std::chrono::seconds timeout,
                                 std::vector<WaitFailureInfo> &failure_infos )
  {
    auto last_print = std::chrono::steady_clock::now();

    auto collect_disconnected_items = [&]() {
      std::vector<std::shared_ptr<Connectable>> disconnected;
      for ( const auto &item : registry_ ) {
        if ( !item->is_connected() ) {
          disconnected.push_back( item );
        }
      }
      return disconnected;
    };

    const bool ok = executor.spin_until(
        [&]() {
          auto disconnected = collect_disconnected_items();
          const bool all_connected = disconnected.empty();

          // Print status every 2 seconds if waiting
          auto now = std::chrono::steady_clock::now();
          if ( !all_connected && ( now - last_print ) > 2s ) {
            std::stringstream ss;
            ss << "Waiting for connections (" << disconnected.size() << " pending): ";
            for ( const auto &item : disconnected ) {
              ss << "[" << item->get_type() << ": " << item->get_name() << "] ";
            }
            RCLCPP_WARN( this->get_logger(), "%s", ss.str().c_str() );
            last_print = now;
          }
          return all_connected;
        },
        timeout );

    if ( !ok ) {
      failure_infos.clear();
      auto disconnected = collect_disconnected_items();

      for ( const auto &item : disconnected ) {
        WaitFailureInfo info;
        info.searched_name = item->get_name();
        info.searched_type = ""; // Type info not available through Connectable interface

        const std::string type = item->get_type();
        if ( type == "Publisher" || type == "Subscription" ) {
          info.entity_kind = "topic";
          info.suggestions = suggest_similar_topics( shared_from_this(), item->get_name() );
        } else if ( type == "Service Client" || type == "Service Server" ) {
          info.entity_kind = "service";
          info.suggestions = suggest_similar_services( shared_from_this(), item->get_name() );
        } else if ( type == "Action Client" || type == "Action Server" ) {
          info.entity_kind = "action";
          info.suggestions = suggest_similar_actions( shared_from_this(), item->get_name() );
        } else {
          info.entity_kind = "unknown";
        }

        failure_infos.push_back( std::move( info ) );
      }
    }

    return ok;
  }

  /// @brief Format a vector of WaitFailureInfo into a human-readable string.
  ///
  /// @param failure_infos The failure information to format.
  /// @param max_suggestions Maximum suggestions per failure (0 = unlimited).
  /// @return A formatted string with all failure details and suggestions.
  [[nodiscard]] static std::string
  format_failure_infos( const std::vector<WaitFailureInfo> &failure_infos, size_t max_suggestions = 0 )
  {
    std::stringstream ss;
    ss << "Connection failures (" << failure_infos.size() << " items):\n";
    for ( const auto &info : failure_infos ) { ss << info.format( max_suggestions ) << "\n"; }
    return ss.str();
  }

private:
  void register_connectable( std::shared_ptr<Connectable> item ) { registry_.push_back( item ); }

  std::vector<std::shared_ptr<Connectable>> registry_;
};

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_TEST_NODE_HPP
