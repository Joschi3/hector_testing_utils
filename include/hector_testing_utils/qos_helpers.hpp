#ifndef HECTOR_TESTING_UTILS_QOS_HELPERS_HPP
#define HECTOR_TESTING_UTILS_QOS_HELPERS_HPP

#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace hector_testing_utils
{

// =============================================================================
// QoS Information Structures
// =============================================================================

/// @brief Represents QoS settings for a publisher or subscriber endpoint.
struct QoSInfo {
  rmw_qos_reliability_policy_t reliability{ RMW_QOS_POLICY_RELIABILITY_UNKNOWN };
  rmw_qos_durability_policy_t durability{ RMW_QOS_POLICY_DURABILITY_UNKNOWN };
  rmw_qos_history_policy_t history{ RMW_QOS_POLICY_HISTORY_UNKNOWN };
  size_t depth{ 0 };
  rmw_qos_liveliness_policy_t liveliness{ RMW_QOS_POLICY_LIVELINESS_UNKNOWN };
  rmw_time_t deadline{ 0, 0 };
  rmw_time_t lifespan{ 0, 0 };

  /// @brief Convert reliability policy to human-readable string.
  [[nodiscard]] static std::string reliability_to_string( rmw_qos_reliability_policy_t policy )
  {
    switch ( policy ) {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      return "RELIABLE";
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      return "BEST_EFFORT";
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
      return "SYSTEM_DEFAULT";
    case RMW_QOS_POLICY_RELIABILITY_UNKNOWN:
    default:
      return "UNKNOWN";
    }
  }

  /// @brief Convert durability policy to human-readable string.
  [[nodiscard]] static std::string durability_to_string( rmw_qos_durability_policy_t policy )
  {
    switch ( policy ) {
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      return "VOLATILE";
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      return "TRANSIENT_LOCAL";
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
      return "SYSTEM_DEFAULT";
    case RMW_QOS_POLICY_DURABILITY_UNKNOWN:
    default:
      return "UNKNOWN";
    }
  }

  /// @brief Convert history policy to human-readable string.
  [[nodiscard]] static std::string history_to_string( rmw_qos_history_policy_t policy )
  {
    switch ( policy ) {
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      return "KEEP_LAST";
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      return "KEEP_ALL";
    case RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT:
      return "SYSTEM_DEFAULT";
    case RMW_QOS_POLICY_HISTORY_UNKNOWN:
    default:
      return "UNKNOWN";
    }
  }

  /// @brief Convert liveliness policy to human-readable string.
  [[nodiscard]] static std::string liveliness_to_string( rmw_qos_liveliness_policy_t policy )
  {
    switch ( policy ) {
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
      return "AUTOMATIC";
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
      return "MANUAL_BY_TOPIC";
    case RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT:
      return "SYSTEM_DEFAULT";
    case RMW_QOS_POLICY_LIVELINESS_UNKNOWN:
    default:
      return "UNKNOWN";
    }
  }

  /// @brief Format QoS info as a human-readable string.
  [[nodiscard]] std::string format() const
  {
    std::stringstream ss;
    ss << "reliability=" << reliability_to_string( reliability );
    ss << ", durability=" << durability_to_string( durability );
    ss << ", history=" << history_to_string( history );
    if ( history == RMW_QOS_POLICY_HISTORY_KEEP_LAST ) {
      ss << "(" << depth << ")";
    }
    return ss.str();
  }

  /// @brief Create QoSInfo from rclcpp::QoS.
  [[nodiscard]] static QoSInfo from_rclcpp_qos( const rclcpp::QoS &qos )
  {
    QoSInfo info;
    const auto &rmw_qos = qos.get_rmw_qos_profile();
    info.reliability = rmw_qos.reliability;
    info.durability = rmw_qos.durability;
    info.history = rmw_qos.history;
    info.depth = rmw_qos.depth;
    info.liveliness = rmw_qos.liveliness;
    info.deadline = rmw_qos.deadline;
    info.lifespan = rmw_qos.lifespan;
    return info;
  }

  /// @brief Create QoSInfo from rmw_qos_profile_t.
  [[nodiscard]] static QoSInfo from_rmw_qos( const rmw_qos_profile_t &rmw_qos )
  {
    QoSInfo info;
    info.reliability = rmw_qos.reliability;
    info.durability = rmw_qos.durability;
    info.history = rmw_qos.history;
    info.depth = rmw_qos.depth;
    info.liveliness = rmw_qos.liveliness;
    info.deadline = rmw_qos.deadline;
    info.lifespan = rmw_qos.lifespan;
    return info;
  }
};

/// @brief Represents an endpoint (publisher or subscriber) with its QoS settings.
struct EndpointInfo {
  std::string node_name;      ///< Name of the node owning this endpoint
  std::string node_namespace; ///< Namespace of the node
  std::string topic_type;     ///< The message type
  QoSInfo qos;                ///< QoS settings for this endpoint

  /// @brief Get the fully qualified node name.
  [[nodiscard]] std::string full_node_name() const
  {
    if ( node_namespace.empty() || node_namespace == "/" ) {
      return "/" + node_name;
    }
    return node_namespace + "/" + node_name;
  }

  /// @brief Format endpoint info as a human-readable string.
  [[nodiscard]] std::string format() const
  {
    std::stringstream ss;
    ss << full_node_name() << " [" << topic_type << "] (" << qos.format() << ")";
    return ss.str();
  }
};

/// @brief Result of QoS compatibility check.
struct QoSCompatibilityResult {
  bool compatible{ true };           ///< Whether the QoS settings are compatible
  std::vector<std::string> warnings; ///< Non-fatal issues that may cause problems
  std::vector<std::string> errors;   ///< Fatal incompatibilities

  /// @brief Check if there are any issues (warnings or errors).
  [[nodiscard]] bool has_issues() const { return !warnings.empty() || !errors.empty(); }

  /// @brief Format the result as a human-readable string.
  [[nodiscard]] std::string format() const
  {
    std::stringstream ss;
    if ( compatible && !has_issues() ) {
      ss << "QoS settings are compatible.\n";
      return ss.str();
    }

    if ( !compatible ) {
      ss << "QoS INCOMPATIBLE:\n";
    } else {
      ss << "QoS compatible with warnings:\n";
    }

    for ( const auto &error : errors ) { ss << "  [ERROR] " << error << "\n"; }
    for ( const auto &warning : warnings ) { ss << "  [WARN] " << warning << "\n"; }
    return ss.str();
  }
};

// =============================================================================
// QoS Compatibility Checking
// =============================================================================

/// @brief Check if reliability policies are compatible.
///
/// A RELIABLE subscriber requires a RELIABLE publisher.
/// A BEST_EFFORT subscriber accepts both RELIABLE and BEST_EFFORT publishers.
///
/// @param pub_reliability Publisher's reliability policy
/// @param sub_reliability Subscriber's reliability policy
/// @return Compatibility result with detailed explanation
inline QoSCompatibilityResult
check_reliability_compatibility( rmw_qos_reliability_policy_t pub_reliability,
                                 rmw_qos_reliability_policy_t sub_reliability )
{
  QoSCompatibilityResult result;

  // Handle unknown/system default as best effort for compatibility purposes
  auto effective_pub = pub_reliability;
  auto effective_sub = sub_reliability;

  if ( effective_pub == RMW_QOS_POLICY_RELIABILITY_UNKNOWN ||
       effective_pub == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT ) {
    effective_pub = RMW_QOS_POLICY_RELIABILITY_RELIABLE; // Assume reliable
  }
  if ( effective_sub == RMW_QOS_POLICY_RELIABILITY_UNKNOWN ||
       effective_sub == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT ) {
    effective_sub = RMW_QOS_POLICY_RELIABILITY_RELIABLE; // Assume reliable
  }

  // RELIABLE subscriber requires RELIABLE publisher
  if ( effective_sub == RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
       effective_pub == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT ) {
    result.compatible = false;
    result.errors.push_back(
        "Subscriber requires RELIABLE but publisher offers BEST_EFFORT. "
        "Messages may not be delivered. Consider changing subscriber to BEST_EFFORT "
        "or publisher to RELIABLE." );
  }

  return result;
}

/// @brief Check if durability policies are compatible.
///
/// A TRANSIENT_LOCAL subscriber expects late-joining to receive cached messages.
/// A VOLATILE publisher does not cache messages.
///
/// @param pub_durability Publisher's durability policy
/// @param sub_durability Subscriber's durability policy
/// @return Compatibility result with detailed explanation
inline QoSCompatibilityResult
check_durability_compatibility( rmw_qos_durability_policy_t pub_durability,
                                rmw_qos_durability_policy_t sub_durability )
{
  QoSCompatibilityResult result;

  auto effective_pub = pub_durability;
  auto effective_sub = sub_durability;

  if ( effective_pub == RMW_QOS_POLICY_DURABILITY_UNKNOWN ||
       effective_pub == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT ) {
    effective_pub = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  }
  if ( effective_sub == RMW_QOS_POLICY_DURABILITY_UNKNOWN ||
       effective_sub == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT ) {
    effective_sub = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  }

  // TRANSIENT_LOCAL subscriber with VOLATILE publisher
  if ( effective_sub == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL &&
       effective_pub == RMW_QOS_POLICY_DURABILITY_VOLATILE ) {
    result.compatible = false;
    result.errors.push_back(
        "Subscriber requires TRANSIENT_LOCAL (latched) but publisher is VOLATILE. "
        "Late-joining subscribers will not receive cached messages. "
        "Consider changing publisher to TRANSIENT_LOCAL or subscriber to VOLATILE." );
  }

  return result;
}

/// @brief Check if two QoS profiles are compatible for pub/sub communication.
///
/// This function checks for common incompatibilities that would prevent
/// message delivery between a publisher and subscriber.
///
/// @param pub_qos Publisher's QoS settings
/// @param sub_qos Subscriber's QoS settings
/// @return Compatibility result with detailed explanation
inline QoSCompatibilityResult check_qos_compatibility( const QoSInfo &pub_qos, const QoSInfo &sub_qos )
{
  QoSCompatibilityResult result;

  // Check reliability
  auto reliability_result =
      check_reliability_compatibility( pub_qos.reliability, sub_qos.reliability );
  if ( !reliability_result.compatible ) {
    result.compatible = false;
  }
  result.errors.insert( result.errors.end(), reliability_result.errors.begin(),
                        reliability_result.errors.end() );
  result.warnings.insert( result.warnings.end(), reliability_result.warnings.begin(),
                          reliability_result.warnings.end() );

  // Check durability
  auto durability_result = check_durability_compatibility( pub_qos.durability, sub_qos.durability );
  if ( !durability_result.compatible ) {
    result.compatible = false;
  }
  result.errors.insert( result.errors.end(), durability_result.errors.begin(),
                        durability_result.errors.end() );
  result.warnings.insert( result.warnings.end(), durability_result.warnings.begin(),
                          durability_result.warnings.end() );

  return result;
}

/// @brief Check if two rclcpp::QoS profiles are compatible.
inline QoSCompatibilityResult check_qos_compatibility( const rclcpp::QoS &pub_qos,
                                                       const rclcpp::QoS &sub_qos )
{
  return check_qos_compatibility( QoSInfo::from_rclcpp_qos( pub_qos ),
                                  QoSInfo::from_rclcpp_qos( sub_qos ) );
}

// =============================================================================
// Topic QoS Introspection
// =============================================================================

/// @brief Get all publisher endpoints for a topic.
///
/// @param node The node to use for introspection
/// @param topic_name The topic to query
/// @return Vector of EndpointInfo for each publisher
inline std::vector<EndpointInfo> get_publishers_info( const rclcpp::Node::SharedPtr &node,
                                                      const std::string &topic_name )
{
  std::vector<EndpointInfo> result;

  auto endpoints = node->get_publishers_info_by_topic( topic_name );
  for ( const auto &endpoint : endpoints ) {
    EndpointInfo info;
    info.node_name = endpoint.node_name();
    info.node_namespace = endpoint.node_namespace();
    info.topic_type = endpoint.topic_type();
    info.qos = QoSInfo::from_rmw_qos( endpoint.qos_profile().get_rmw_qos_profile() );
    result.push_back( info );
  }

  return result;
}

/// @brief Get all subscriber endpoints for a topic.
///
/// @param node The node to use for introspection
/// @param topic_name The topic to query
/// @return Vector of EndpointInfo for each subscriber
inline std::vector<EndpointInfo> get_subscribers_info( const rclcpp::Node::SharedPtr &node,
                                                       const std::string &topic_name )
{
  std::vector<EndpointInfo> result;

  auto endpoints = node->get_subscriptions_info_by_topic( topic_name );
  for ( const auto &endpoint : endpoints ) {
    EndpointInfo info;
    info.node_name = endpoint.node_name();
    info.node_namespace = endpoint.node_namespace();
    info.topic_type = endpoint.topic_type();
    info.qos = QoSInfo::from_rmw_qos( endpoint.qos_profile().get_rmw_qos_profile() );
    result.push_back( info );
  }

  return result;
}

/// @brief Diagnose QoS issues on a topic.
///
/// This function checks all publisher-subscriber pairs on a topic and
/// reports any QoS incompatibilities that could prevent communication.
///
/// @param node The node to use for introspection
/// @param topic_name The topic to diagnose
/// @return Diagnostic string with QoS information and any issues found
inline std::string diagnose_topic_qos( const rclcpp::Node::SharedPtr &node,
                                       const std::string &topic_name )
{
  std::stringstream ss;
  ss << "QoS diagnosis for topic: " << topic_name << "\n";

  auto publishers = get_publishers_info( node, topic_name );
  auto subscribers = get_subscribers_info( node, topic_name );

  ss << "\nPublishers (" << publishers.size() << "):\n";
  if ( publishers.empty() ) {
    ss << "  (none)\n";
  } else {
    for ( const auto &pub : publishers ) { ss << "  - " << pub.format() << "\n"; }
  }

  ss << "\nSubscribers (" << subscribers.size() << "):\n";
  if ( subscribers.empty() ) {
    ss << "  (none)\n";
  } else {
    for ( const auto &sub : subscribers ) { ss << "  - " << sub.format() << "\n"; }
  }

  // Check compatibility for all pub-sub pairs
  bool any_issues = false;
  std::stringstream issues_ss;

  for ( const auto &pub : publishers ) {
    for ( const auto &sub : subscribers ) {
      auto compat = check_qos_compatibility( pub.qos, sub.qos );
      if ( !compat.compatible || compat.has_issues() ) {
        any_issues = true;
        issues_ss << "\nIncompatibility between:\n";
        issues_ss << "  Publisher: " << pub.full_node_name() << "\n";
        issues_ss << "  Subscriber: " << sub.full_node_name() << "\n";
        issues_ss << compat.format();
      }
    }
  }

  if ( any_issues ) {
    ss << "\n" << issues_ss.str();
  } else if ( !publishers.empty() && !subscribers.empty() ) {
    ss << "\nAll QoS settings are compatible.\n";
  }

  return ss.str();
}

/// @brief Check if a specific QoS is compatible with all existing endpoints on a topic.
///
/// Useful for checking before creating a publisher/subscriber whether it will
/// be compatible with existing endpoints.
///
/// @param node The node to use for introspection
/// @param topic_name The topic to check
/// @param qos The QoS settings to check
/// @param is_publisher True if checking for a new publisher, false for subscriber
/// @return Compatibility result with all existing endpoints
inline QoSCompatibilityResult check_topic_qos_compatibility( const rclcpp::Node::SharedPtr &node,
                                                             const std::string &topic_name,
                                                             const rclcpp::QoS &qos,
                                                             bool is_publisher )
{
  QoSCompatibilityResult combined_result;
  auto my_qos = QoSInfo::from_rclcpp_qos( qos );

  if ( is_publisher ) {
    // Check against all existing subscribers
    auto subscribers = get_subscribers_info( node, topic_name );
    for ( const auto &sub : subscribers ) {
      auto compat = check_qos_compatibility( my_qos, sub.qos );
      if ( !compat.compatible ) {
        combined_result.compatible = false;
      }
      for ( const auto &err : compat.errors ) {
        combined_result.errors.push_back( "With " + sub.full_node_name() + ": " + err );
      }
      for ( const auto &warn : compat.warnings ) {
        combined_result.warnings.push_back( "With " + sub.full_node_name() + ": " + warn );
      }
    }
  } else {
    // Check against all existing publishers
    auto publishers = get_publishers_info( node, topic_name );
    for ( const auto &pub : publishers ) {
      auto compat = check_qos_compatibility( pub.qos, my_qos );
      if ( !compat.compatible ) {
        combined_result.compatible = false;
      }
      for ( const auto &err : compat.errors ) {
        combined_result.errors.push_back( "With " + pub.full_node_name() + ": " + err );
      }
      for ( const auto &warn : compat.warnings ) {
        combined_result.warnings.push_back( "With " + pub.full_node_name() + ": " + warn );
      }
    }
  }

  return combined_result;
}

// =============================================================================
// QoS Summary for Suggestions
// =============================================================================

/// @brief Get a compact QoS summary string for display in suggestions.
///
/// @param node The node to use for introspection
/// @param topic_name The topic to summarize
/// @return Compact string showing publisher and subscriber QoS info
inline std::string get_topic_qos_summary( const rclcpp::Node::SharedPtr &node,
                                          const std::string &topic_name )
{
  auto publishers = get_publishers_info( node, topic_name );
  auto subscribers = get_subscribers_info( node, topic_name );

  std::stringstream ss;

  if ( !publishers.empty() ) {
    ss << "pubs: ";
    for ( size_t i = 0; i < publishers.size() && i < 3; ++i ) {
      if ( i > 0 )
        ss << ", ";
      ss << publishers[i].qos.format();
    }
    if ( publishers.size() > 3 ) {
      ss << " (+" << ( publishers.size() - 3 ) << " more)";
    }
  }

  if ( !publishers.empty() && !subscribers.empty() ) {
    ss << " | ";
  }

  if ( !subscribers.empty() ) {
    ss << "subs: ";
    for ( size_t i = 0; i < subscribers.size() && i < 3; ++i ) {
      if ( i > 0 )
        ss << ", ";
      ss << subscribers[i].qos.format();
    }
    if ( subscribers.size() > 3 ) {
      ss << " (+" << ( subscribers.size() - 3 ) << " more)";
    }
  }

  return ss.str();
}

/// @brief Check QoS compatibility with a topic and return a hint string.
///
/// @param node The node to use for introspection
/// @param topic_name The topic to check
/// @param my_qos The QoS settings of the local endpoint
/// @param is_publisher True if checking for a publisher, false for subscriber
/// @return Empty string if compatible, otherwise a hint about the incompatibility
inline std::string get_qos_compatibility_hint( const rclcpp::Node::SharedPtr &node,
                                               const std::string &topic_name,
                                               const rclcpp::QoS &my_qos, bool is_publisher )
{
  auto result = check_topic_qos_compatibility( node, topic_name, my_qos, is_publisher );

  if ( result.compatible && !result.has_issues() ) {
    return "";
  }

  std::stringstream ss;
  if ( !result.compatible ) {
    ss << " [QoS INCOMPATIBLE]";
  } else if ( !result.warnings.empty() ) {
    ss << " [QoS WARNING]";
  }

  return ss.str();
}

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_QOS_HELPERS_HPP
