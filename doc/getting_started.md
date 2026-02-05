# Getting Started

## What is hector_testing_utils?

**hector_testing_utils** is a library for writing robust ROS 2 integration tests that interact with the real DDS/RMW middleware. It provides:

### Robust Test Infrastructure

- **Connection-aware wrappers** for publishers, subscribers, services, and actions that track when peers connect
- **Deterministic waiting** with `wait_for_*` helpers that avoid race conditions and flaky tests
- **Automatic connection tracking** via `TestNode` factory methods
- **Background spinning** support for complex multi-node test scenarios
- **Test context isolation** to prevent interference between tests

### Debugging Helpers

When tests fail, finding the root cause can be difficult. hector_testing_utils provides several tools to help:

- **Auto-suggestions on timeout**: When a wait operation times out, the library automatically logs similar topics, services, or actions that exist in the ROS graph - catching typos and namespace issues
- **QoS diagnostics**: Detect and diagnose QoS incompatibilities that cause silent connection failures (e.g., RELIABLE subscriber + BEST_EFFORT publisher)
- **Graph monitoring**: Track nodes, topics, and services appearing or disappearing in the ROS graph
- **Structured failure info**: Programmatic access to detailed failure information for custom error handling

### Testing Utilities

- **Remote parameter manipulation**: Set and get parameters on other nodes during tests
- **Log capture**: Verify that specific ROS log messages were emitted
- **Service/Action call helpers**: Simplified service and action calls with automatic waiting
- **Pre-configured test fixtures**: Google Test fixtures with executor and test node ready to use

## Integration

**Compatibility:**
* **ROS 2 Distributions**: Jazzy, Kilted, Rolling
* **C++ Standard**: C++17 or later

### package.xml

```xml
<test_depend>hector_testing_utils</test_depend>
```

### CMakeLists.txt

```cmake
if(BUILD_TESTING)
  find_package(hector_testing_utils REQUIRED)
  ament_add_gtest(my_integration_test test/my_integration_test.cpp)
  ament_target_dependencies(my_integration_test rclcpp hector_testing_utils)
endif()
```

## Modular Headers

Include the main header for full functionality:

```cpp
#include <hector_testing_utils/hector_testing_utils.hpp>  // Includes everything
```

Or include individual headers for finer-grained control:

```cpp
#include <hector_testing_utils/constants.hpp>           // kDefaultTimeout, kDefaultSpinPeriod
#include <hector_testing_utils/test_executor.hpp>       // TestExecutor
#include <hector_testing_utils/test_wrappers.hpp>       // TestPublisher, TestSubscription, etc.
#include <hector_testing_utils/test_node.hpp>           // TestNode
#include <hector_testing_utils/test_fixtures.hpp>       // HectorTestFixture
#include <hector_testing_utils/graph_introspection.hpp> // Suggestion generation
#include <hector_testing_utils/qos_helpers.hpp>         // QoS validation & diagnostics
#include <hector_testing_utils/graph_monitor.hpp>       // Graph change monitoring
#include <hector_testing_utils/parameter_helpers.hpp>   // Remote parameter manipulation
#include <hector_testing_utils/log_capture.hpp>         // LogCapture
#include <hector_testing_utils/assertions.hpp>          // ASSERT_SERVICE_EXISTS, etc.
#include <hector_testing_utils/wait_helpers.hpp>        // call_service, call_action
#include <hector_testing_utils/test_context.hpp>        // TestContext
```

## Comparison with rtest

While **[rtest](https://github.com/Beam-and-Spyrosoft/rtest)** is an excellent tool for unit testing, it mocks `rclcpp` entirely. This makes it fast but unusable for code that relies on real middleware behavior.

**hector_testing_utils** fills that gap by providing a stable environment for integration tests where mocking isn't an option.

| Feature | rtest | hector_testing_utils |
| --- | --- | --- |
| **Middleware** | **Mocked** (No DDS) | **Real** (Actual DDS/RMW) |
| **Execution Speed** | Instant | Slower |
| **Scope** | Unit Logic Only | Full Integration |
| **Limitations** | Cannot test complex library interactions | Subject to OS scheduling/timing |

**When to use which:**
* Use **rtest** for instant, flake-free feedback on logic or single node ROS 2 communication
* Use **hector_testing_utils** when rtest is too restrictive (complex node interactions, opaque libraries that manage their own ROS entities)
