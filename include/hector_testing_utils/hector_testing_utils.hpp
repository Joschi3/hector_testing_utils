#ifndef HECTOR_TESTING_UTILS_HECTOR_TESTING_UTILS_HPP
#define HECTOR_TESTING_UTILS_HECTOR_TESTING_UTILS_HPP

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rcutils/logging.h>

#include <gtest/gtest.h>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace hector_testing_utils
{

using namespace std::chrono_literals;

constexpr std::chrono::milliseconds kDefaultSpinPeriod{ 5 };
constexpr std::chrono::seconds kDefaultTimeout{ 5 };
constexpr size_t kDefaultMaxSuggestions{ 10 };

// =============================================================================
// Graph Introspection Types
// =============================================================================

/// @brief Represents an entity (topic, service, or action) in the ROS graph with its types.
struct GraphEntity {
  std::string name;               ///< The fully-qualified name (e.g., "/my_topic")
  std::vector<std::string> types; ///< The message/service/action types
};

/// @brief Represents a suggestion for a similar entity with a similarity score.
struct Suggestion {
  std::string name;               ///< The name of the similar entity
  std::vector<std::string> types; ///< The types of the similar entity
  double similarity_score;        ///< Similarity score (0.0 = no match, 1.0 = exact match)

  /// @brief Compare suggestions by similarity score (descending order).
  bool operator<( const Suggestion &other ) const
  {
    return similarity_score > other.similarity_score; // Higher score = better match
  }
};

/// @brief Result of a wait operation that failed due to timeout.
struct WaitFailureInfo {
  std::string searched_name;           ///< The name that was searched for
  std::string searched_type;           ///< The type that was searched for (if applicable)
  std::string entity_kind;             ///< Kind of entity: "topic", "service", "action"
  std::vector<Suggestion> suggestions; ///< Similar entities sorted by similarity

  /// @brief Format the failure info as a human-readable string.
  /// @param max_suggestions Maximum number of suggestions to include (0 = unlimited)
  [[nodiscard]] std::string format( size_t max_suggestions = 0 ) const
  {
    std::stringstream ss;
    ss << "Failed to find " << entity_kind << " '" << searched_name << "'";
    if ( !searched_type.empty() ) {
      ss << " (type: " << searched_type << ")";
    }
    ss << "\n";

    if ( suggestions.empty() ) {
      ss << "  No similar " << entity_kind << "s found in the graph.\n";
    } else {
      const size_t num_to_show = ( max_suggestions > 0 )
                                     ? std::min( max_suggestions, suggestions.size() )
                                     : suggestions.size();
      const bool truncated = ( max_suggestions > 0 ) && ( suggestions.size() > max_suggestions );

      ss << "  Available " << entity_kind << "s (sorted by similarity):\n";
      for ( size_t i = 0; i < num_to_show; ++i ) {
        const auto &suggestion = suggestions[i];
        ss << "    - " << suggestion.name;
        if ( !suggestion.types.empty() ) {
          ss << " [";
          for ( size_t j = 0; j < suggestion.types.size(); ++j ) {
            if ( j > 0 )
              ss << ", ";
            ss << suggestion.types[j];
          }
          ss << "]";
        }
        ss << " (score: " << std::fixed << std::setprecision( 2 ) << suggestion.similarity_score
           << ")\n";
      }
      if ( truncated ) {
        ss << "    ... and " << ( suggestions.size() - max_suggestions ) << " more.\n";
      }
    }
    return ss.str();
  }
};

// =============================================================================
// String Similarity Utilities
// =============================================================================

namespace detail
{

/// @brief Compute the Levenshtein edit distance between two strings.
///
/// This is a classic dynamic programming algorithm for computing the minimum
/// number of single-character edits (insertions, deletions, substitutions)
/// required to transform one string into another.
///
/// @param s1 First string
/// @param s2 Second string
/// @return The edit distance (0 means identical strings)
inline size_t levenshtein_distance( const std::string &s1, const std::string &s2 )
{
  const size_t m = s1.size();
  const size_t n = s2.size();

  if ( m == 0 )
    return n;
  if ( n == 0 )
    return m;

  // Use two rows instead of full matrix for space efficiency
  std::vector<size_t> prev_row( n + 1 );
  std::vector<size_t> curr_row( n + 1 );

  // Initialize first row
  std::iota( prev_row.begin(), prev_row.end(), 0 );

  for ( size_t i = 1; i <= m; ++i ) {
    curr_row[0] = i;
    for ( size_t j = 1; j <= n; ++j ) {
      const size_t cost = ( s1[i - 1] == s2[j - 1] ) ? 0 : 1;
      curr_row[j] = std::min( {
          prev_row[j] + 1,       // deletion
          curr_row[j - 1] + 1,   // insertion
          prev_row[j - 1] + cost // substitution
      } );
    }
    std::swap( prev_row, curr_row );
  }

  return prev_row[n];
}

/// @brief Convert Levenshtein distance to a similarity score in [0, 1].
///
/// @param s1 First string
/// @param s2 Second string
/// @return Similarity score (1.0 = identical, 0.0 = completely different)
inline double string_similarity( const std::string &s1, const std::string &s2 )
{
  if ( s1.empty() && s2.empty() )
    return 1.0;

  const size_t max_len = std::max( s1.size(), s2.size() );
  const size_t distance = levenshtein_distance( s1, s2 );

  return 1.0 - static_cast<double>( distance ) / static_cast<double>( max_len );
}

/// @brief Check if a string contains another string as a substring (case-insensitive).
inline bool contains_substring_ci( const std::string &haystack, const std::string &needle )
{
  if ( needle.empty() )
    return true;
  if ( haystack.empty() )
    return false;

  std::string haystack_lower = haystack;
  std::string needle_lower = needle;
  std::transform( haystack_lower.begin(), haystack_lower.end(), haystack_lower.begin(), ::tolower );
  std::transform( needle_lower.begin(), needle_lower.end(), needle_lower.begin(), ::tolower );

  return haystack_lower.find( needle_lower ) != std::string::npos;
}

/// @brief Extract the base name from a fully-qualified name (removes leading slashes and namespace).
inline std::string extract_base_name( const std::string &full_name )
{
  auto pos = full_name.rfind( '/' );
  if ( pos != std::string::npos && pos + 1 < full_name.size() ) {
    return full_name.substr( pos + 1 );
  }
  return full_name;
}

/// @brief Compute a composite similarity score considering multiple factors.
///
/// This function considers:
/// - Full name similarity (Levenshtein-based)
/// - Base name similarity (ignoring namespace)
/// - Substring matching (bonus if one contains the other)
/// - Type matching (bonus if types overlap)
///
/// @param searched_name The name being searched for
/// @param candidate_name The candidate name to compare
/// @param searched_types The types being searched for (may be empty)
/// @param candidate_types The candidate's types
/// @return A composite similarity score in [0, 1]
inline double compute_similarity( const std::string &searched_name, const std::string &candidate_name,
                                  const std::vector<std::string> &searched_types,
                                  const std::vector<std::string> &candidate_types )
{
  // Base similarity from Levenshtein distance on full names
  double full_name_sim = string_similarity( searched_name, candidate_name );

  // Also compare base names (last component after final '/')
  std::string searched_base = extract_base_name( searched_name );
  std::string candidate_base = extract_base_name( candidate_name );
  double base_name_sim = string_similarity( searched_base, candidate_base );

  // Start with weighted combination of full and base name similarities
  double score = 0.6 * full_name_sim + 0.4 * base_name_sim;

  // Bonus for substring matching
  if ( contains_substring_ci( candidate_name, searched_name ) ||
       contains_substring_ci( searched_name, candidate_name ) ) {
    score = std::min( 1.0, score + 0.15 );
  }

  // Bonus for type matching
  if ( !searched_types.empty() && !candidate_types.empty() ) {
    for ( const auto &st : searched_types ) {
      for ( const auto &ct : candidate_types ) {
        if ( st == ct ) {
          score = std::min( 1.0, score + 0.2 );
          goto type_bonus_applied; // Only apply once
        }
        // Partial type matching (e.g., same message type, different package)
        std::string st_base = extract_base_name( st );
        std::string ct_base = extract_base_name( ct );
        if ( !st_base.empty() && st_base == ct_base ) {
          score = std::min( 1.0, score + 0.1 );
          goto type_bonus_applied;
        }
      }
    }
  type_bonus_applied:;
  }

  return score;
}

/// @brief Helper to get the type name for a ROS message type.
///
/// Uses rosidl_generator_traits::name() for messages and services.
/// For action types, constructs the name from the Goal type.
template<typename T, typename = void>
struct type_name_helper {
  static std::string get() { return rosidl_generator_traits::name<T>(); }
};

/// @brief Specialization for action types (detected by presence of Goal typedef).
template<typename T>
struct type_name_helper<T, std::void_t<typename T::Goal>> {
  static std::string get()
  {
    // Action goal type name is like "example_interfaces/action/Fibonacci_Goal"
    // We want "example_interfaces/action/Fibonacci"
    std::string goal_name = rosidl_generator_traits::name<typename T::Goal>();
    const std::string suffix = "_Goal";
    if ( goal_name.size() > suffix.size() &&
         goal_name.compare( goal_name.size() - suffix.size(), suffix.size(), suffix ) == 0 ) {
      return goal_name.substr( 0, goal_name.size() - suffix.size() );
    }
    return goal_name;
  }
};

/// @brief Get the type name for any ROS interface type (message, service, or action).
template<typename T>
std::string get_type_name()
{
  return type_name_helper<T>::get();
}

} // namespace detail

// =============================================================================
// Graph Introspection Functions
// =============================================================================

/// @brief Collect all available topics from the ROS graph.
///
/// @param node The node to use for graph introspection
/// @return Vector of GraphEntity containing topic names and their message types
inline std::vector<GraphEntity> collect_available_topics( const rclcpp::Node::SharedPtr &node )
{
  std::vector<GraphEntity> topics;
  auto topic_map = node->get_topic_names_and_types();

  topics.reserve( topic_map.size() );
  for ( const auto &[name, types] : topic_map ) { topics.push_back( { name, types } ); }

  return topics;
}

/// @brief Collect all available services from the ROS graph.
///
/// @param node The node to use for graph introspection
/// @return Vector of GraphEntity containing service names and their types
inline std::vector<GraphEntity> collect_available_services( const rclcpp::Node::SharedPtr &node )
{
  std::vector<GraphEntity> services;
  auto service_map = node->get_service_names_and_types();

  services.reserve( service_map.size() );
  for ( const auto &[name, types] : service_map ) { services.push_back( { name, types } ); }

  return services;
}

/// @brief Collect all available actions from the ROS graph.
///
/// Actions are identified by looking for topics that end with "/_action/status",
/// which every action server publishes.
///
/// @param node The node to use for graph introspection
/// @return Vector of GraphEntity containing action names and their types
inline std::vector<GraphEntity> collect_available_actions( const rclcpp::Node::SharedPtr &node )
{
  std::vector<GraphEntity> actions;
  auto topic_map = node->get_topic_names_and_types();

  const std::string status_suffix = "/_action/status";

  for ( const auto &[name, types] : topic_map ) {
    // Check if this topic ends with "/_action/status"
    if ( name.size() > status_suffix.size() &&
         name.compare( name.size() - status_suffix.size(), status_suffix.size(), status_suffix ) ==
             0 ) {
      // Extract the action name by removing the suffix
      std::string action_name = name.substr( 0, name.size() - status_suffix.size() );

      // Try to infer action type from the goal topic
      std::string goal_topic = action_name + "/_action/send_goal";
      std::vector<std::string> action_types;

      auto goal_it = topic_map.find( goal_topic );
      if ( goal_it != topic_map.end() && !goal_it->second.empty() ) {
        // The goal service type is something like "example_interfaces/action/Fibonacci_SendGoal"
        // We want to extract "example_interfaces/action/Fibonacci"
        for ( const auto &type : goal_it->second ) {
          auto pos = type.rfind( "_SendGoal" );
          if ( pos != std::string::npos ) {
            action_types.push_back( type.substr( 0, pos ) );
          } else {
            action_types.push_back( type );
          }
        }
      }

      actions.push_back( { action_name, action_types } );
    }
  }

  return actions;
}

// =============================================================================
// Suggestion Generation Functions
// =============================================================================

/// @brief Generate suggestions for similar topics.
///
/// @param node The node to use for graph introspection
/// @param searched_topic The topic name that was searched for
/// @param searched_type The message type that was searched for (optional)
/// @param max_suggestions Maximum number of suggestions to return
/// @return Vector of Suggestion sorted by similarity (best matches first)
inline std::vector<Suggestion> suggest_similar_topics( const rclcpp::Node::SharedPtr &node,
                                                       const std::string &searched_topic,
                                                       const std::string &searched_type = "",
                                                       size_t max_suggestions = 10 )
{
  auto topics = collect_available_topics( node );
  std::vector<Suggestion> suggestions;
  suggestions.reserve( topics.size() );

  std::vector<std::string> searched_types;
  if ( !searched_type.empty() ) {
    searched_types.push_back( searched_type );
  }

  for ( const auto &topic : topics ) {
    double score =
        detail::compute_similarity( searched_topic, topic.name, searched_types, topic.types );
    suggestions.push_back( { topic.name, topic.types, score } );
  }

  // Sort by similarity score (descending)
  std::sort( suggestions.begin(), suggestions.end() );

  // Limit to max_suggestions
  if ( suggestions.size() > max_suggestions ) {
    suggestions.resize( max_suggestions );
  }

  return suggestions;
}

/// @brief Generate suggestions for similar services.
///
/// @param node The node to use for graph introspection
/// @param searched_service The service name that was searched for
/// @param searched_type The service type that was searched for (optional)
/// @param max_suggestions Maximum number of suggestions to return
/// @return Vector of Suggestion sorted by similarity (best matches first)
inline std::vector<Suggestion> suggest_similar_services( const rclcpp::Node::SharedPtr &node,
                                                         const std::string &searched_service,
                                                         const std::string &searched_type = "",
                                                         size_t max_suggestions = 10 )
{
  auto services = collect_available_services( node );
  std::vector<Suggestion> suggestions;
  suggestions.reserve( services.size() );

  std::vector<std::string> searched_types;
  if ( !searched_type.empty() ) {
    searched_types.push_back( searched_type );
  }

  for ( const auto &service : services ) {
    double score =
        detail::compute_similarity( searched_service, service.name, searched_types, service.types );
    suggestions.push_back( { service.name, service.types, score } );
  }

  std::sort( suggestions.begin(), suggestions.end() );

  if ( suggestions.size() > max_suggestions ) {
    suggestions.resize( max_suggestions );
  }

  return suggestions;
}

/// @brief Generate suggestions for similar actions.
///
/// @param node The node to use for graph introspection
/// @param searched_action The action name that was searched for
/// @param searched_type The action type that was searched for (optional)
/// @param max_suggestions Maximum number of suggestions to return
/// @return Vector of Suggestion sorted by similarity (best matches first)
inline std::vector<Suggestion> suggest_similar_actions( const rclcpp::Node::SharedPtr &node,
                                                        const std::string &searched_action,
                                                        const std::string &searched_type = "",
                                                        size_t max_suggestions = 10 )
{
  auto actions = collect_available_actions( node );
  std::vector<Suggestion> suggestions;
  suggestions.reserve( actions.size() );

  std::vector<std::string> searched_types;
  if ( !searched_type.empty() ) {
    searched_types.push_back( searched_type );
  }

  for ( const auto &action : actions ) {
    double score =
        detail::compute_similarity( searched_action, action.name, searched_types, action.types );
    suggestions.push_back( { action.name, action.types, score } );
  }

  std::sort( suggestions.begin(), suggestions.end() );

  if ( suggestions.size() > max_suggestions ) {
    suggestions.resize( max_suggestions );
  }

  return suggestions;
}

// =============================================================================
// Context Helper
// =============================================================================

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

// =============================================================================
// Executor Helper
// =============================================================================

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

// =============================================================================
// Interfaces & Wrappers
// =============================================================================

// Abstract base for anything that needs to "connect"
/// @brief Interface for test objects that need to report connection status.
class Connectable
{
public:
  virtual ~Connectable() = default;
  /// @brief Check if the object is connected to its peer(s).
  virtual bool is_connected() const = 0;
  /// @brief Get the name (topic/service/action name).
  virtual std::string get_name() const = 0;
  /// @brief Get the type string (e.g. "Publisher").
  virtual std::string get_type() const = 0; // e.g., "Publisher", "Client"
};

/// @brief Wrapper for a ROS publisher to facilitate testing.
template<typename MsgT>
class TestPublisher : public Connectable
{
public:
  /// @brief Construct a new Test Publisher object.
  TestPublisher( rclcpp::Node::SharedPtr node, const std::string &topic,
                 const rclcpp::QoS &qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ) )
      : node_( node ), topic_( topic )
  {
    pub_ = node->create_publisher<MsgT>( topic, qos );
  }

  /// @brief Publish a message.
  void publish( const MsgT &msg ) { pub_->publish( msg ); }

  bool is_connected() const override { return pub_->get_subscription_count() > 0; }
  std::string get_name() const override { return topic_; }
  std::string get_type() const override { return "Publisher"; }

  /// @brief Wait until at least one subscription is connected.
  ///
  /// On timeout, logs suggestions for similar topics that exist in the ROS graph.
  ///
  /// @param exec The executor to spin.
  /// @param timeout Maximum wait time.
  /// @return true if a subscription connected, false on timeout.
  bool wait_for_subscription( TestExecutor &exec, std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    WaitFailureInfo failure_info;
    const bool success = wait_for_subscription( exec, timeout, failure_info );
    if ( !success ) {
      RCLCPP_ERROR( node_->get_logger(), "%s", failure_info.format( kDefaultMaxSuggestions ).c_str() );
    }
    return success;
  }

  /// @brief Wait until at least one subscription is connected, with failure info on timeout.
  ///
  /// If the wait times out, this method populates the failure_info with suggestions
  /// for similar topics that exist in the ROS graph.
  ///
  /// @param exec The executor to spin.
  /// @param timeout Maximum wait time.
  /// @param failure_info Reference to receive failure information if wait times out.
  /// @return true if a subscription connected, false on timeout.
  bool wait_for_subscription( TestExecutor &exec, std::chrono::nanoseconds timeout,
                              WaitFailureInfo &failure_info )
  {
    const bool success = exec.spin_until( [this]() { return is_connected(); }, timeout );

    if ( !success ) {
      failure_info.searched_name = topic_;
      failure_info.searched_type = detail::get_type_name<MsgT>();
      failure_info.entity_kind = "topic";
      failure_info.suggestions = suggest_similar_topics( node_, topic_, failure_info.searched_type );
    }

    return success;
  }

private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
  std::string topic_;
};

/// @brief Wrapper for a ROS subscription to facilitate testing.
///
/// Handles message capture, counting, and provides wait utilities.
template<class MsgT>
class TestSubscription : public Connectable
{
public:
  /// @brief Construct a new Test Subscription object.
  TestSubscription( const rclcpp::Node::SharedPtr &node, const std::string &topic,
                    const rclcpp::QoS &qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ),
                    bool latched = false )
      : node_( node ), topic_( topic )
  {
    rclcpp::QoS resolved_qos = qos;
    if ( latched )
      resolved_qos.durability( RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL );

    sub_ = node->create_subscription<MsgT>( topic, resolved_qos,
                                            [this]( const std::shared_ptr<MsgT> msg ) {
                                              std::lock_guard<std::mutex> lock( mutex_ );
                                              last_message_ = *msg;
                                              ++message_count_;
                                            } );
  }

  bool is_connected() const override { return sub_->get_publisher_count() > 0; }
  std::string get_name() const override { return topic_; }
  std::string get_type() const override { return "Subscription"; }

  /// @brief Clear captured messages and reset count.
  void reset()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    last_message_.reset();
    message_count_ = 0;
  }

  /// @brief Check if new messages arrived since a starting count.
  bool has_new_message( size_t start_count ) const
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return message_count_ > start_count;
  }

  /// @brief Get the last received message.
  std::optional<MsgT> last_message() const
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return last_message_;
  }

  /// @brief Get the total number of received messages.
  size_t message_count() const
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return message_count_;
  }

  /// @brief Wait until at least 'count' publishers are connected.
  ///
  /// On timeout, logs suggestions for similar topics that exist in the ROS graph.
  ///
  /// @param executor The executor to spin.
  /// @param count Minimum number of publishers required.
  /// @param timeout Maximum wait time.
  /// @return true if enough publishers connected, false on timeout.
  bool wait_for_publishers( TestExecutor &executor, size_t count = 1,
                            std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    WaitFailureInfo failure_info;
    const bool success = wait_for_publishers( executor, count, timeout, failure_info );
    if ( !success ) {
      RCLCPP_ERROR( node_->get_logger(), "%s", failure_info.format( kDefaultMaxSuggestions ).c_str() );
    }
    return success;
  }

  /// @brief Wait until at least 'count' publishers are connected, with failure info on timeout.
  ///
  /// If the wait times out, this method populates the failure_info with suggestions
  /// for similar topics that exist in the ROS graph.
  ///
  /// @param executor The executor to spin.
  /// @param count Minimum number of publishers required.
  /// @param timeout Maximum wait time.
  /// @param failure_info Reference to receive failure information if wait times out.
  /// @return true if enough publishers connected, false on timeout.
  bool wait_for_publishers( TestExecutor &executor, size_t count, std::chrono::nanoseconds timeout,
                            WaitFailureInfo &failure_info )
  {
    const bool success = executor.spin_until(
        [this, count]() { return sub_->get_publisher_count() >= count; }, timeout );

    if ( !success ) {
      failure_info.searched_name = topic_;
      failure_info.searched_type = detail::get_type_name<MsgT>();
      failure_info.entity_kind = "topic";
      failure_info.suggestions = suggest_similar_topics( node_, topic_, failure_info.searched_type );
    }

    return success;
  }

  /// @brief Wait for a message to arrive, optionally matching a predicate.
  /// @param executor The executor to spin.
  /// @param timeout Maximum wait time.
  /// @param predicate Optional function to filter messages.
  /// @return true if a matching message arrived, false on timeout.
  bool wait_for_message( TestExecutor &executor, std::chrono::nanoseconds timeout = kDefaultTimeout,
                         const std::function<bool( const MsgT & )> &predicate = nullptr )
  {
    size_t start_count = 0;
    {
      std::lock_guard<std::mutex> lock( mutex_ );
      start_count = message_count_;
    }

    return executor.spin_until(
        [this, start_count, &predicate]() {
          std::optional<MsgT> msg;
          size_t count = 0;
          {
            std::lock_guard<std::mutex> lock( mutex_ );
            count = message_count_;
            if ( count <= start_count || !last_message_ ) {
              return false;
            }
            msg = last_message_;
          }
          if ( predicate ) {
            return predicate( *msg );
          }
          return true;
        },
        timeout );
  }

  /// @brief Convenience wrapper for wait_for_message without predicate.
  bool wait_for_new_message( TestExecutor &executor,
                             std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return wait_for_message( executor, timeout );
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Subscription<MsgT>> sub_;
  std::string topic_;
  mutable std::mutex mutex_;
  std::optional<MsgT> last_message_;
  size_t message_count_{ 0 };
};

/// @brief Wrapper for a ROS service client to facilitate testing.
template<typename ServiceT>
class TestClient : public Connectable
{
public:
  /// @brief Construct a new Test Client object.
  TestClient( rclcpp::Node::SharedPtr node, const std::string &service_name )
      : node_( node ), service_name_( service_name )
  {
    client_ = node->create_client<ServiceT>( service_name );
  }

  bool is_connected() const override { return client_->service_is_ready(); }
  std::string get_name() const override { return service_name_; }
  std::string get_type() const override { return "Service Client"; }

  /// @brief Get the underlying ROS client.
  typename rclcpp::Client<ServiceT>::SharedPtr get() { return client_; }

  /// @brief Wait for the service to be available.
  ///
  /// On timeout, logs suggestions for similar services that exist in the ROS graph.
  ///
  /// @param exec The executor to spin.
  /// @param timeout Maximum wait time.
  /// @return true if the service became available, false on timeout.
  bool wait_for_service( TestExecutor &exec, std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    WaitFailureInfo failure_info;
    const bool success = wait_for_service( exec, timeout, failure_info );
    if ( !success ) {
      RCLCPP_ERROR( node_->get_logger(), "%s", failure_info.format( kDefaultMaxSuggestions ).c_str() );
    }
    return success;
  }

  /// @brief Wait for the service to be available, with failure info on timeout.
  ///
  /// If the wait times out, this method populates the failure_info with suggestions
  /// for similar services that exist in the ROS graph.
  ///
  /// @param exec The executor to spin.
  /// @param timeout Maximum wait time.
  /// @param failure_info Reference to receive failure information if wait times out.
  /// @return true if the service became available, false on timeout.
  bool wait_for_service( TestExecutor &exec, std::chrono::nanoseconds timeout,
                         WaitFailureInfo &failure_info )
  {
    const bool success = exec.spin_until( [this]() { return client_->service_is_ready(); }, timeout );

    if ( !success ) {
      failure_info.searched_name = service_name_;
      failure_info.searched_type = detail::get_type_name<ServiceT>();
      failure_info.entity_kind = "service";
      failure_info.suggestions =
          suggest_similar_services( node_, service_name_, failure_info.searched_type );
    }

    return success;
  }

private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  std::string service_name_;
};

/// @brief Wrapper for a ROS action client to facilitate testing.
template<typename ActionT>
class TestActionClient : public Connectable
{
public:
  /// @brief Construct a new Test Action Client object.
  TestActionClient( rclcpp::Node::SharedPtr node, const std::string &action_name )
      : node_( node ), action_name_( action_name )
  {
    client_ = rclcpp_action::create_client<ActionT>( node, action_name );
  }

  bool is_connected() const override { return client_->action_server_is_ready(); }
  std::string get_name() const override { return action_name_; }
  std::string get_type() const override { return "Action Client"; }

  /// @brief Get the underlying ROS action client.
  typename rclcpp_action::Client<ActionT>::SharedPtr get() { return client_; }

  /// @brief Wait for the action server to be available.
  ///
  /// On timeout, logs suggestions for similar actions that exist in the ROS graph.
  ///
  /// @param exec The executor to spin.
  /// @param timeout Maximum wait time.
  /// @return true if the action server became available, false on timeout.
  bool wait_for_server( TestExecutor &exec, std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    WaitFailureInfo failure_info;
    const bool success = wait_for_server( exec, timeout, failure_info );
    if ( !success ) {
      RCLCPP_ERROR( node_->get_logger(), "%s", failure_info.format( kDefaultMaxSuggestions ).c_str() );
    }
    return success;
  }

  /// @brief Wait for the action server to be available, with failure info on timeout.
  ///
  /// If the wait times out, this method populates the failure_info with suggestions
  /// for similar actions that exist in the ROS graph.
  ///
  /// @param exec The executor to spin.
  /// @param timeout Maximum wait time.
  /// @param failure_info Reference to receive failure information if wait times out.
  /// @return true if the action server became available, false on timeout.
  bool wait_for_server( TestExecutor &exec, std::chrono::nanoseconds timeout,
                        WaitFailureInfo &failure_info )
  {
    const bool success =
        exec.spin_until( [this]() { return client_->action_server_is_ready(); }, timeout );

    if ( !success ) {
      failure_info.searched_name = action_name_;
      failure_info.searched_type = detail::get_type_name<ActionT>();
      failure_info.entity_kind = "action";
      failure_info.suggestions =
          suggest_similar_actions( node_, action_name_, failure_info.searched_type );
    }

    return success;
  }

private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;
  std::string action_name_;
};

/// @brief Wrapper for a ROS service server to facilitate testing.
template<typename ServiceT>
class TestServiceServer : public Connectable
{
public:
  using Callback = std::function<void( const std::shared_ptr<typename ServiceT::Request>,
                                       std::shared_ptr<typename ServiceT::Response> )>;

  /// @brief Construct a new Test Service Server object.
  TestServiceServer( const rclcpp::Node::SharedPtr &node, const std::string &service_name,
                     const Callback &callback, const rclcpp::QoS &qos = rclcpp::ServicesQoS() )
      : node_( node ), service_name_( service_name )
  {
    service_ = node_->create_service<ServiceT>( service_name_, callback, qos );
  }

  bool is_connected() const override { return node_->count_clients( service_name_ ) > 0; }
  std::string get_name() const override { return service_name_; }
  std::string get_type() const override { return "Service Server"; }

  /// @brief Wait for at least one client to connect.
  ///
  /// On timeout, logs suggestions for similar services that exist in the ROS graph.
  ///
  /// @param exec The executor to spin.
  /// @param timeout Maximum wait time.
  /// @return true if a client connected, false on timeout.
  bool wait_for_client( TestExecutor &exec, std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    WaitFailureInfo failure_info;
    const bool success = wait_for_client( exec, timeout, failure_info );
    if ( !success ) {
      RCLCPP_ERROR( node_->get_logger(), "%s", failure_info.format( kDefaultMaxSuggestions ).c_str() );
    }
    return success;
  }

  /// @brief Wait for at least one client to connect, with failure info on timeout.
  ///
  /// If the wait times out, this method populates the failure_info with suggestions
  /// for similar services that exist in the ROS graph.
  ///
  /// @param exec The executor to spin.
  /// @param timeout Maximum wait time.
  /// @param failure_info Reference to receive failure information if wait times out.
  /// @return true if a client connected, false on timeout.
  bool wait_for_client( TestExecutor &exec, std::chrono::nanoseconds timeout,
                        WaitFailureInfo &failure_info )
  {
    const bool success = exec.spin_until( [this]() { return is_connected(); }, timeout );

    if ( !success ) {
      failure_info.searched_name = service_name_;
      failure_info.searched_type = detail::get_type_name<ServiceT>();
      failure_info.entity_kind = "service";
      failure_info.suggestions =
          suggest_similar_services( node_, service_name_, failure_info.searched_type );
    }

    return success;
  }

  /// @brief Get the underlying ROS service.
  typename rclcpp::Service<ServiceT>::SharedPtr get() { return service_; }

private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Service<ServiceT>::SharedPtr service_;
  std::string service_name_;
};

/// @brief Wrapper for a ROS action server to facilitate testing.
template<typename ActionT>
class TestActionServer : public Connectable
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using GoalCallback = std::function<rclcpp_action::GoalResponse(
      const rclcpp_action::GoalUUID &, const std::shared_ptr<const typename ActionT::Goal> )>;
  using CancelCallback =
      std::function<rclcpp_action::CancelResponse( const std::shared_ptr<GoalHandle> )>;
  using AcceptedCallback = std::function<void( const std::shared_ptr<GoalHandle> )>;

  /// @brief Construct a new Test Action Server object.
  TestActionServer( const rclcpp::Node::SharedPtr &node, const std::string &action_name,
                    GoalCallback goal_cb, CancelCallback cancel_cb, AcceptedCallback accepted_cb )
      : node_( node ), action_name_( action_name )
  {
    server_ =
        rclcpp_action::create_server<ActionT>( node_, action_name_, std::move( goal_cb ),
                                               std::move( cancel_cb ), std::move( accepted_cb ) );
  }

  bool is_connected() const override
  {
    // Clients subscribe to the status topic; use that to infer connectivity.
    return node_->count_subscribers( status_topic() ) > 0;
  }

  std::string get_name() const override { return action_name_; }
  std::string get_type() const override { return "Action Server"; }

  /// @brief Wait for at least one client to connect.
  ///
  /// On timeout, logs suggestions for similar actions that exist in the ROS graph.
  ///
  /// @param exec The executor to spin.
  /// @param timeout Maximum wait time.
  /// @return true if a client connected, false on timeout.
  bool wait_for_client( TestExecutor &exec, std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    WaitFailureInfo failure_info;
    const bool success = wait_for_client( exec, timeout, failure_info );
    if ( !success ) {
      RCLCPP_ERROR( node_->get_logger(), "%s", failure_info.format( kDefaultMaxSuggestions ).c_str() );
    }
    return success;
  }

  /// @brief Wait for at least one client to connect, with failure info on timeout.
  ///
  /// If the wait times out, this method populates the failure_info with suggestions
  /// for similar actions that exist in the ROS graph.
  ///
  /// @param exec The executor to spin.
  /// @param timeout Maximum wait time.
  /// @param failure_info Reference to receive failure information if wait times out.
  /// @return true if a client connected, false on timeout.
  bool wait_for_client( TestExecutor &exec, std::chrono::nanoseconds timeout,
                        WaitFailureInfo &failure_info )
  {
    const bool success = exec.spin_until( [this]() { return is_connected(); }, timeout );

    if ( !success ) {
      failure_info.searched_name = action_name_;
      failure_info.searched_type = detail::get_type_name<ActionT>();
      failure_info.entity_kind = "action";
      failure_info.suggestions =
          suggest_similar_actions( node_, action_name_, failure_info.searched_type );
    }

    return success;
  }

  /// @brief Get the underlying ROS action server.
  typename rclcpp_action::Server<ActionT>::SharedPtr get() { return server_; }

private:
  std::string status_topic() const { return action_name_ + "/_action/status"; }

  rclcpp::Node::SharedPtr node_;
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  std::string action_name_;
};

// =============================================================================
// The Main Testing Node
// =============================================================================

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

// =============================================================================
// Topic and Service/Action Helpers preserved for backward compatibility
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

/// Wait until the topic has at least min_subscribers subscribers.
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

/// Wait for a service to appear on the ROS graph.
template<typename ServiceT>
bool wait_for_service( const typename rclcpp::Client<ServiceT>::SharedPtr &client,
                       TestExecutor &executor, std::chrono::nanoseconds timeout )
{
  return executor.spin_until(
      [&client]() { return client->wait_for_service( std::chrono::nanoseconds( 0 ) ); }, timeout );
}

/// Wait for an action server to appear on the ROS graph.
template<typename ActionT>
bool wait_for_action_server( const typename rclcpp_action::Client<ActionT>::SharedPtr &client,
                             TestExecutor &executor, std::chrono::nanoseconds timeout )
{
  return executor.spin_until(
      [&client]() { return client->wait_for_action_server( std::chrono::nanoseconds( 0 ) ); },
      timeout );
}

struct ServiceCallOptions {
  std::chrono::nanoseconds service_timeout{ kDefaultTimeout };
  std::chrono::nanoseconds response_timeout{ kDefaultTimeout };
};

/// Call a service and wait for the response.
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

struct ActionCallOptions {
  std::chrono::nanoseconds server_timeout{ kDefaultTimeout };
  std::chrono::nanoseconds goal_timeout{ kDefaultTimeout };
  std::chrono::nanoseconds result_timeout{ std::chrono::seconds( 30 ) };
};

/// Send an action goal and wait for the result.
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

// =============================================================================
// Assertions & Macros
// =============================================================================

// Internal helpers
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

// Macros

#define ASSERT_SERVICE_EXISTS( node, service, timeout )                                            \
  ASSERT_TRUE( hector_testing_utils::assert_service_exists(                                        \
      *executor_, node, service, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define EXPECT_SERVICE_EXISTS( node, service, timeout )                                            \
  EXPECT_TRUE( hector_testing_utils::assert_service_exists(                                        \
      *executor_, node, service, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define ASSERT_ACTION_EXISTS( node, action, timeout )                                              \
  ASSERT_TRUE( hector_testing_utils::assert_action_server_exists(                                  \
      *executor_, node, action, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define EXPECT_ACTION_EXISTS( node, action, timeout )                                              \
  EXPECT_TRUE( hector_testing_utils::assert_action_server_exists(                                  \
      *executor_, node, action, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define ASSERT_SERVICE_EXISTS_WITH_EXECUTOR( executor, node, service, timeout )                    \
  ASSERT_TRUE( hector_testing_utils::assert_service_exists(                                        \
      executor, node, service, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define EXPECT_SERVICE_EXISTS_WITH_EXECUTOR( executor, node, service, timeout )                    \
  EXPECT_TRUE( hector_testing_utils::assert_service_exists(                                        \
      executor, node, service, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define ASSERT_ACTION_EXISTS_WITH_EXECUTOR( executor, node, action, timeout )                      \
  ASSERT_TRUE( hector_testing_utils::assert_action_server_exists(                                  \
      executor, node, action, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define EXPECT_ACTION_EXISTS_WITH_EXECUTOR( executor, node, action, timeout )                      \
  EXPECT_TRUE( hector_testing_utils::assert_action_server_exists(                                  \
      executor, node, action, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

// =============================================================================
// Log Verification Helper
// =============================================================================

/// @brief Utility to capture and verify ROS log messages.
///
/// This class intercepts ROS 2 logging calls (via rcutils) and allows tests to assert that specific
/// log messages have been published. It uses regex matching.
class LogCapture
{
public:
  struct LogMessage {
    int severity;
    std::string name;
    std::string message;
  };

  /// @brief Construct a new Log Capture object.
  /// @throws std::runtime_error if another LogCapture instance already exists (singleton enforcement).
  LogCapture()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    if ( active_instance_ ) {
      throw std::runtime_error( "Only one LogCapture instance can be active at a time!" );
    }
    active_instance_ = this;
    previous_handler_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler( &LogCapture::log_handler );
  }

  /// @brief Destroy the Log Capture object and restore the previous log handler.
  ~LogCapture()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    if ( active_instance_ == this ) {
      rcutils_logging_set_output_handler( previous_handler_ );
      active_instance_ = nullptr;
    }
  }

  LogCapture( const LogCapture & ) = delete;
  LogCapture &operator=( const LogCapture & ) = delete;

  /// @brief Wait until a log message matching the regex pattern is captured.
  /// @param executor Executor to spin (for timeout).
  /// @param pattern_regex Regex pattern to look for.
  /// @param timeout Maximum wait time.
  /// @return true if found, false on timeout.
  bool wait_for_log( TestExecutor &executor, const std::string &pattern_regex,
                     std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    std::regex re( pattern_regex );
    return executor.spin_until( [this, &re]() { return has_log( re ); }, timeout );
  }

  /// @brief Check if a log message matching the regex pattern has been captured.
  /// @param pattern_regex Regex pattern.
  /// @return true if found.
  bool has_log( const std::string &pattern_regex ) const
  {
    std::regex re( pattern_regex );
    return has_log( re );
  }

  /// @brief Clear all captured logs.
  void clear()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    captured_logs_.clear();
  }

private:
  bool has_log( const std::regex &re ) const
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    for ( const auto &log : captured_logs_ ) {
      if ( std::regex_search( log.message, re ) ) {
        return true;
      }
    }
    return false;
  }

  static void log_handler( const rcutils_log_location_t *location, int severity, const char *name,
                           rcutils_time_point_value_t timestamp, const char *format, va_list *args )
  {
    char buffer[1024];
    va_list args_copy;
    va_copy( args_copy, *args );
    vsnprintf( buffer, sizeof( buffer ), format, args_copy );
    va_end( args_copy );

    {
      std::lock_guard<std::mutex> lock( mutex_ );
      if ( active_instance_ ) {
        active_instance_->captured_logs_.push_back( { severity, name ? name : "", buffer } );
        if ( active_instance_->previous_handler_ ) {
          active_instance_->previous_handler_( location, severity, name, timestamp, format, args );
        }
      }
    }
  }

  static LogCapture *active_instance_;
  static std::mutex mutex_;
  rcutils_logging_output_handler_t previous_handler_;
  std::vector<LogMessage> captured_logs_;
};

// Static inline definitions (C++17)
inline LogCapture *LogCapture::active_instance_ = nullptr;
inline std::mutex LogCapture::mutex_;

// =============================================================================
// Fixture Update
// =============================================================================

class HectorTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if ( !rclcpp::ok() ) {
      rclcpp::init( 0, nullptr );
    }
    // Allow derived classes to provide a custom executor (e.g. MultiThreaded)
    // by overriding create_test_executor()
    std::shared_ptr<rclcpp::Executor> internal_exec = create_test_executor();
    if ( !internal_exec ) {
      throw std::runtime_error( "create_test_executor() returned nullptr" );
    }

    executor_ = std::make_shared<TestExecutor>( internal_exec );

    // Instantiate our new smart TestNode
    tester_node_ = std::make_shared<TestNode>( "hector_tester_node" );
    executor_->add_node( tester_node_ );
  }

  void TearDown() override
  {
    if ( rclcpp::ok() ) {
      rclcpp::shutdown();
    }
  }

  // Virtual method to allow overriding the executor type definition
  virtual std::shared_ptr<rclcpp::Executor> create_test_executor()
  {
    return std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  std::shared_ptr<TestExecutor> executor_;
  std::shared_ptr<TestNode> tester_node_;
};

class HectorTestFixtureWithContext : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<TestContext>();
    executor_ = std::make_shared<TestExecutor>( context_->context() );
    tester_node_ = std::make_shared<TestNode>( "hector_tester_node", context_->node_options() );
    executor_->add_node( tester_node_ );
  }

  void TearDown() override
  {
    tester_node_.reset();
    executor_.reset();
    context_.reset();
  }

  std::shared_ptr<TestContext> context_;
  std::shared_ptr<TestExecutor> executor_;
  std::shared_ptr<TestNode> tester_node_;
};

/// Create NodeOptions that load parameters from a YAML file.
inline rclcpp::NodeOptions
node_options_from_yaml( const std::string &params_file,
                        const std::vector<std::string> &extra_arguments = {} )
{
  std::vector<std::string> args = { "--ros-args", "--params-file", params_file };
  args.insert( args.end(), extra_arguments.begin(), extra_arguments.end() );
  rclcpp::NodeOptions options;
  options.arguments( args );
  options.automatically_declare_parameters_from_overrides( true );
  return options;
}

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_HECTOR_TESTING_UTILS_HPP
