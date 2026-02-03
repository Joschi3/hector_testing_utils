#ifndef HECTOR_TESTING_UTILS_GRAPH_MONITOR_HPP
#define HECTOR_TESTING_UTILS_GRAPH_MONITOR_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <hector_testing_utils/constants.hpp>
#include <hector_testing_utils/test_executor.hpp>

namespace hector_testing_utils
{

// =============================================================================
// Graph Change Types
// =============================================================================

/// @brief Represents a change in the ROS graph.
struct GraphChange {
  enum class Type {
    NODE_ADDED,
    NODE_REMOVED,
    TOPIC_ADDED,
    TOPIC_REMOVED,
    SERVICE_ADDED,
    SERVICE_REMOVED
  };

  Type type;
  std::string name;               ///< Name of the node/topic/service
  std::vector<std::string> types; ///< Types (for topics/services)
  std::chrono::steady_clock::time_point timestamp;

  /// @brief Convert change type to human-readable string.
  [[nodiscard]] static std::string type_to_string( Type t )
  {
    switch ( t ) {
    case Type::NODE_ADDED:
      return "NODE_ADDED";
    case Type::NODE_REMOVED:
      return "NODE_REMOVED";
    case Type::TOPIC_ADDED:
      return "TOPIC_ADDED";
    case Type::TOPIC_REMOVED:
      return "TOPIC_REMOVED";
    case Type::SERVICE_ADDED:
      return "SERVICE_ADDED";
    case Type::SERVICE_REMOVED:
      return "SERVICE_REMOVED";
    default:
      return "UNKNOWN";
    }
  }

  /// @brief Format the change as a human-readable string.
  [[nodiscard]] std::string format() const
  {
    std::stringstream ss;
    ss << "[" << type_to_string( type ) << "] " << name;
    if ( !types.empty() ) {
      ss << " [";
      for ( size_t i = 0; i < types.size(); ++i ) {
        if ( i > 0 )
          ss << ", ";
        ss << types[i];
      }
      ss << "]";
    }
    return ss.str();
  }
};

// =============================================================================
// Graph Monitor Class
// =============================================================================

/// @brief Monitors changes in the ROS graph (nodes, topics, services).
///
/// This class provides utilities for:
/// - Waiting for specific nodes to appear or disappear
/// - Waiting for specific topics or services to be created
/// - Tracking graph changes over time
/// - Getting notified of graph changes via callbacks
///
/// Example usage:
/// @code
///   GraphMonitor monitor(node);
///   monitor.start();
///
///   // Wait for a specific node to appear
///   bool found = monitor.wait_for_node(executor, "/my_node", 5s);
///
///   // Get all changes since monitoring started
///   auto changes = monitor.get_changes();
///
///   monitor.stop();
/// @endcode
class GraphMonitor
{
public:
  using ChangeCallback = std::function<void( const GraphChange & )>;

  /// @brief Construct a new Graph Monitor.
  /// @param node The node to use for graph introspection
  explicit GraphMonitor( const rclcpp::Node::SharedPtr &node ) : node_( node ) { take_snapshot(); }

  /// @brief Start monitoring for graph changes.
  ///
  /// This registers a callback with the node's graph change event.
  void start()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    if ( monitoring_ ) {
      return;
    }

    monitoring_ = true;
    changes_.clear();
    take_snapshot_unlocked();

    // Subscribe to graph changes
    graph_event_ = node_->get_graph_event();
  }

  /// @brief Stop monitoring for graph changes.
  void stop()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    monitoring_ = false;
    graph_event_.reset();
  }

  /// @brief Check if monitoring is active.
  [[nodiscard]] bool is_monitoring() const
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return monitoring_;
  }

  /// @brief Set a callback to be called for each graph change.
  /// @param callback The callback function
  void set_change_callback( ChangeCallback callback )
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    change_callback_ = std::move( callback );
  }

  /// @brief Poll for graph changes and update internal state.
  ///
  /// This should be called periodically (e.g., in a spin loop) to detect changes.
  void poll()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    if ( !monitoring_ ) {
      return;
    }

    // Always check for changes by comparing current state with known state
    // The graph event system is not reliable in all cases
    detect_changes_unlocked();
  }

  /// @brief Get all recorded graph changes.
  /// @return Vector of changes since monitoring started
  [[nodiscard]] std::vector<GraphChange> get_changes() const
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return changes_;
  }

  /// @brief Get changes since a specific time point.
  /// @param since Only return changes after this time
  /// @return Vector of changes after the specified time
  [[nodiscard]] std::vector<GraphChange>
  get_changes_since( std::chrono::steady_clock::time_point since ) const
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    std::vector<GraphChange> result;
    for ( const auto &change : changes_ ) {
      if ( change.timestamp > since ) {
        result.push_back( change );
      }
    }
    return result;
  }

  /// @brief Clear all recorded changes.
  void clear_changes()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    changes_.clear();
  }

  // ===========================================================================
  // Wait Functions
  // ===========================================================================

  /// @brief Wait for a specific node to appear in the graph.
  ///
  /// @param executor The executor to spin
  /// @param node_name The fully qualified name of the node to wait for
  /// @param timeout Maximum time to wait
  /// @return true if the node appeared, false on timeout
  bool wait_for_node( TestExecutor &executor, const std::string &node_name,
                      std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return executor.spin_until(
        [this, &node_name]() {
          poll();
          return has_node( node_name );
        },
        timeout );
  }

  /// @brief Wait for a specific node to disappear from the graph.
  ///
  /// @param executor The executor to spin
  /// @param node_name The fully qualified name of the node to wait for
  /// @param timeout Maximum time to wait
  /// @return true if the node disappeared, false on timeout
  bool wait_for_node_removed( TestExecutor &executor, const std::string &node_name,
                              std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return executor.spin_until(
        [this, &node_name]() {
          poll();
          return !has_node( node_name );
        },
        timeout );
  }

  /// @brief Wait for a specific topic to appear in the graph.
  ///
  /// @param executor The executor to spin
  /// @param topic_name The fully qualified name of the topic to wait for
  /// @param timeout Maximum time to wait
  /// @return true if the topic appeared, false on timeout
  bool wait_for_topic( TestExecutor &executor, const std::string &topic_name,
                       std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return executor.spin_until(
        [this, &topic_name]() {
          poll();
          return has_topic( topic_name );
        },
        timeout );
  }

  /// @brief Wait for a specific topic to disappear from the graph.
  ///
  /// @param executor The executor to spin
  /// @param topic_name The fully qualified name of the topic to wait for
  /// @param timeout Maximum time to wait
  /// @return true if the topic disappeared, false on timeout
  bool wait_for_topic_removed( TestExecutor &executor, const std::string &topic_name,
                               std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return executor.spin_until(
        [this, &topic_name]() {
          poll();
          return !has_topic( topic_name );
        },
        timeout );
  }

  /// @brief Wait for a specific service to appear in the graph.
  ///
  /// @param executor The executor to spin
  /// @param service_name The fully qualified name of the service to wait for
  /// @param timeout Maximum time to wait
  /// @return true if the service appeared, false on timeout
  bool wait_for_service( TestExecutor &executor, const std::string &service_name,
                         std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return executor.spin_until(
        [this, &service_name]() {
          poll();
          return has_service( service_name );
        },
        timeout );
  }

  /// @brief Wait for a specific service to disappear from the graph.
  ///
  /// @param executor The executor to spin
  /// @param service_name The fully qualified name of the service to wait for
  /// @param timeout Maximum time to wait
  /// @return true if the service disappeared, false on timeout
  bool wait_for_service_removed( TestExecutor &executor, const std::string &service_name,
                                 std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return executor.spin_until(
        [this, &service_name]() {
          poll();
          return !has_service( service_name );
        },
        timeout );
  }

  /// @brief Wait for any graph change to occur.
  ///
  /// @param executor The executor to spin
  /// @param timeout Maximum time to wait
  /// @return true if a change occurred, false on timeout
  bool wait_for_any_change( TestExecutor &executor,
                            std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    size_t initial_count;
    {
      std::lock_guard<std::mutex> lock( mutex_ );
      initial_count = changes_.size();
    }

    return executor.spin_until(
        [this, initial_count]() {
          poll();
          std::lock_guard<std::mutex> lock( mutex_ );
          return changes_.size() > initial_count;
        },
        timeout );
  }

  // ===========================================================================
  // Query Functions
  // ===========================================================================

  /// @brief Check if a node currently exists in the graph.
  /// @param node_name The fully qualified node name
  /// @return true if the node exists
  [[nodiscard]] bool has_node( const std::string &node_name ) const
  {
    auto nodes = node_->get_node_names();
    for ( const auto &name : nodes ) {
      if ( name == node_name ) {
        return true;
      }
    }
    return false;
  }

  /// @brief Check if a topic currently exists in the graph.
  /// @param topic_name The fully qualified topic name
  /// @return true if the topic exists
  [[nodiscard]] bool has_topic( const std::string &topic_name ) const
  {
    auto topics = node_->get_topic_names_and_types();
    return topics.find( topic_name ) != topics.end();
  }

  /// @brief Check if a service currently exists in the graph.
  /// @param service_name The fully qualified service name
  /// @return true if the service exists
  [[nodiscard]] bool has_service( const std::string &service_name ) const
  {
    auto services = node_->get_service_names_and_types();
    return services.find( service_name ) != services.end();
  }

  /// @brief Get all currently known nodes.
  /// @return Set of node names
  [[nodiscard]] std::set<std::string> get_nodes() const
  {
    std::set<std::string> result;
    for ( const auto &name : node_->get_node_names() ) { result.insert( name ); }
    return result;
  }

  /// @brief Get all currently known topics.
  /// @return Set of topic names
  [[nodiscard]] std::set<std::string> get_topics() const
  {
    std::set<std::string> result;
    auto topics = node_->get_topic_names_and_types();
    for ( const auto &[name, types] : topics ) { result.insert( name ); }
    return result;
  }

  /// @brief Get all currently known services.
  /// @return Set of service names
  [[nodiscard]] std::set<std::string> get_services() const
  {
    std::set<std::string> result;
    auto services = node_->get_service_names_and_types();
    for ( const auto &[name, types] : services ) { result.insert( name ); }
    return result;
  }

private:
  void take_snapshot()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    take_snapshot_unlocked();
  }

  void take_snapshot_unlocked()
  {
    // Nodes
    known_nodes_.clear();
    for ( const auto &name : node_->get_node_names() ) { known_nodes_.insert( name ); }

    // Topics
    known_topics_.clear();
    auto topics = node_->get_topic_names_and_types();
    for ( const auto &[name, types] : topics ) { known_topics_[name] = types; }

    // Services
    known_services_.clear();
    auto services = node_->get_service_names_and_types();
    for ( const auto &[name, types] : services ) { known_services_[name] = types; }
  }

  void detect_changes_unlocked()
  {
    auto now = std::chrono::steady_clock::now();

    // Check nodes
    std::set<std::string> current_nodes;
    for ( const auto &name : node_->get_node_names() ) { current_nodes.insert( name ); }

    // Detect added nodes
    for ( const auto &name : current_nodes ) {
      if ( known_nodes_.find( name ) == known_nodes_.end() ) {
        GraphChange change;
        change.type = GraphChange::Type::NODE_ADDED;
        change.name = name;
        change.timestamp = now;
        add_change_unlocked( change );
      }
    }

    // Detect removed nodes
    for ( const auto &name : known_nodes_ ) {
      if ( current_nodes.find( name ) == current_nodes.end() ) {
        GraphChange change;
        change.type = GraphChange::Type::NODE_REMOVED;
        change.name = name;
        change.timestamp = now;
        add_change_unlocked( change );
      }
    }

    known_nodes_ = std::move( current_nodes );

    // Check topics
    auto current_topics = node_->get_topic_names_and_types();

    for ( const auto &[name, types] : current_topics ) {
      if ( known_topics_.find( name ) == known_topics_.end() ) {
        GraphChange change;
        change.type = GraphChange::Type::TOPIC_ADDED;
        change.name = name;
        change.types = types;
        change.timestamp = now;
        add_change_unlocked( change );
      }
    }

    for ( const auto &[name, types] : known_topics_ ) {
      if ( current_topics.find( name ) == current_topics.end() ) {
        GraphChange change;
        change.type = GraphChange::Type::TOPIC_REMOVED;
        change.name = name;
        change.types = types;
        change.timestamp = now;
        add_change_unlocked( change );
      }
    }

    known_topics_ = std::move( current_topics );

    // Check services
    auto current_services = node_->get_service_names_and_types();

    for ( const auto &[name, types] : current_services ) {
      if ( known_services_.find( name ) == known_services_.end() ) {
        GraphChange change;
        change.type = GraphChange::Type::SERVICE_ADDED;
        change.name = name;
        change.types = types;
        change.timestamp = now;
        add_change_unlocked( change );
      }
    }

    for ( const auto &[name, types] : known_services_ ) {
      if ( current_services.find( name ) == current_services.end() ) {
        GraphChange change;
        change.type = GraphChange::Type::SERVICE_REMOVED;
        change.name = name;
        change.types = types;
        change.timestamp = now;
        add_change_unlocked( change );
      }
    }

    known_services_ = std::move( current_services );
  }

  void add_change_unlocked( const GraphChange &change )
  {
    changes_.push_back( change );
    if ( change_callback_ ) {
      change_callback_( change );
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Event::SharedPtr graph_event_;

  mutable std::mutex mutex_;
  bool monitoring_{ false };

  std::set<std::string> known_nodes_;
  std::map<std::string, std::vector<std::string>> known_topics_;
  std::map<std::string, std::vector<std::string>> known_services_;

  std::vector<GraphChange> changes_;
  ChangeCallback change_callback_;
};

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_GRAPH_MONITOR_HPP
