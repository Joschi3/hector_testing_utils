#ifndef HECTOR_TESTING_UTILS_GRAPH_INTROSPECTION_HPP
#define HECTOR_TESTING_UTILS_GRAPH_INTROSPECTION_HPP

#include <algorithm>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <hector_testing_utils/constants.hpp>

namespace hector_testing_utils
{

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
  std::string name;                   ///< The name of the similar entity
  std::vector<std::string> types;     ///< The types of the similar entity
  double similarity_score;            ///< Similarity score (0.0 = no match, 1.0 = exact match)
  std::string qos_info;               ///< QoS information summary (for topics)
  std::string qos_compatibility_hint; ///< Hint about QoS compatibility issues

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
           << ")";
        if ( !suggestion.qos_compatibility_hint.empty() ) {
          ss << " " << suggestion.qos_compatibility_hint;
        }
        ss << "\n";
        if ( !suggestion.qos_info.empty() ) {
          ss << "      QoS: " << suggestion.qos_info << "\n";
        }
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
    Suggestion suggestion;
    suggestion.name = topic.name;
    suggestion.types = topic.types;
    suggestion.similarity_score = score;
    suggestions.push_back( suggestion );
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
    Suggestion suggestion;
    suggestion.name = service.name;
    suggestion.types = service.types;
    suggestion.similarity_score = score;
    suggestions.push_back( suggestion );
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
    Suggestion suggestion;
    suggestion.name = action.name;
    suggestion.types = action.types;
    suggestion.similarity_score = score;
    suggestions.push_back( suggestion );
  }

  std::sort( suggestions.begin(), suggestions.end() );

  if ( suggestions.size() > max_suggestions ) {
    suggestions.resize( max_suggestions );
  }

  return suggestions;
}

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_GRAPH_INTROSPECTION_HPP
