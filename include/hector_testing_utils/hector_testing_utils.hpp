#ifndef HECTOR_TESTING_UTILS_HECTOR_TESTING_UTILS_HPP
#define HECTOR_TESTING_UTILS_HECTOR_TESTING_UTILS_HPP

/// @file hector_testing_utils.hpp
/// @brief Main header that includes all hector_testing_utils components.
///
/// This header provides backward compatibility by including all sub-headers.
/// Users can include individual headers for finer-grained control over dependencies.
///
/// Available headers:
/// - constants.hpp: Common constants (timeouts, defaults)
/// - graph_introspection.hpp: Graph introspection types and suggestion generation
/// - qos_helpers.hpp: QoS validation, compatibility checking, and diagnostics
/// - graph_monitor.hpp: Graph change monitoring and waiting utilities
/// - parameter_helpers.hpp: Remote parameter manipulation for testing
/// - test_context.hpp: TestContext for isolated ROS contexts
/// - test_executor.hpp: TestExecutor with spin utilities
/// - test_wrappers.hpp: Test wrappers for publishers, subscribers, services, actions
/// - test_node.hpp: TestNode with factory methods for test wrappers
/// - wait_helpers.hpp: Helper functions for waiting on ROS entities
/// - assertions.hpp: GTest assertion macros for ROS entities
/// - log_capture.hpp: LogCapture for verifying log messages
/// - test_fixtures.hpp: GTest fixtures for ROS tests

// Core components
#include <hector_testing_utils/constants.hpp>
#include <hector_testing_utils/graph_introspection.hpp>
#include <hector_testing_utils/graph_monitor.hpp>
#include <hector_testing_utils/parameter_helpers.hpp>
#include <hector_testing_utils/qos_helpers.hpp>
#include <hector_testing_utils/test_context.hpp>
#include <hector_testing_utils/test_executor.hpp>

// Wrappers and node
#include <hector_testing_utils/test_node.hpp>
#include <hector_testing_utils/test_wrappers.hpp>

// Utilities
#include <hector_testing_utils/assertions.hpp>
#include <hector_testing_utils/log_capture.hpp>
#include <hector_testing_utils/wait_helpers.hpp>

// Test fixtures
#include <hector_testing_utils/test_fixtures.hpp>

#endif // HECTOR_TESTING_UTILS_HECTOR_TESTING_UTILS_HPP
