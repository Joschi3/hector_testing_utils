[![ROS2](https://img.shields.io/badge/ROS2-Jazzy%20|%20Kilted%20|%20Rolling-blue)](https://docs.ros.org)
![Lint](https://github.com/Joschi3/hector_testing_utils/actions/workflows/lint_build_test.yaml/badge.svg)
[![codecov](https://codecov.io/gh/Joschi3/hector_testing_utils/graph/badge.svg?token=RYR8J8FNC8)](https://codecov.io/gh/Joschi3/hector_testing_utils)
[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://joschi3.github.io/hector_testing_utils/)

# hector_testing_utils

**Helper classes and utilities for writing robust ROS 2 integration tests using the real ROS graph and middleware.**

`hector_testing_utils` is designed for testing ROS 2 systems **where mocking rclcpp is not sufficient**.
It enables deterministic, connection-aware Google Tests that interact with **actual DDS/RMW behavior**, while providing structured helpers to reduce flakiness.

## Documentation

**[Full Documentation](https://joschi3.github.io/hector_testing_utils/)** - User guide, API reference, and examples.

## Quick Start

### Installation

**package.xml**:
```xml
<test_depend>hector_testing_utils</test_depend>
```

**CMakeLists.txt**:
```cmake
if(BUILD_TESTING)
  find_package(hector_testing_utils REQUIRED)
  ament_add_gtest(my_integration_test test/my_integration_test.cpp)
  ament_target_dependencies(my_integration_test rclcpp hector_testing_utils)
endif()
```

### Basic Example

```cpp
#include <gtest/gtest.h>
#include <std_msgs/msg/int32.hpp>
#include <hector_testing_utils/hector_testing_utils.hpp>

using namespace std::chrono_literals;

TEST_F(HectorTestFixture, SimplePublisherSubscriber)
{
  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>("/topic");
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>("/topic");

  ASSERT_TRUE(tester_node_->wait_for_all_connections(*executor_, 5s));

  std_msgs::msg::Int32 msg;
  msg.data = 42;
  pub->publish(msg);

  ASSERT_TRUE(sub->wait_for_message(*executor_, 5s));
  EXPECT_EQ(sub->last_message()->data, 42);
}
```

## Features

| Feature | Description |
|---------|-------------|
| **TestExecutor** | Executor wrapper with `spin_until()` and background spinning |
| **TestNode** | Node with factory methods and automatic connection tracking |
| **Connection-Aware Wrappers** | Publisher, Subscription, Service, Action wrappers with wait helpers |
| **Auto-Suggestions** | Timeout messages show similar topics/services to help debug |
| **QoS Diagnostics** | Detect and diagnose QoS incompatibilities |
| **Graph Monitoring** | Wait for nodes, topics, services to appear/disappear |
| **Remote Parameters** | Set/get parameters on other nodes during tests |
| **Log Capture** | Verify ROS log messages in tests |
| **Test Fixtures** | Pre-configured GTest fixtures with optional context isolation |

## When to Use

| Use Case | Tool |
|----------|------|
| Unit testing node logic, fast feedback | [rtest](https://github.com/Beam-and-Spyrosoft/rtest) |
| Integration testing with real DDS/RMW | **hector_testing_utils** |
| Testing complex node interactions | **hector_testing_utils** |
| Testing opaque libraries that manage ROS entities | **hector_testing_utils** |

## Modular Headers

```cpp
#include <hector_testing_utils/hector_testing_utils.hpp>  // Everything

// Or individual headers:
#include <hector_testing_utils/test_executor.hpp>       // TestExecutor
#include <hector_testing_utils/test_wrappers.hpp>       // TestPublisher, TestSubscription, etc.
#include <hector_testing_utils/test_node.hpp>           // TestNode
#include <hector_testing_utils/test_fixtures.hpp>       // HectorTestFixture
#include <hector_testing_utils/qos_helpers.hpp>         // QoS validation & diagnostics
#include <hector_testing_utils/graph_monitor.hpp>       // Graph change monitoring
#include <hector_testing_utils/parameter_helpers.hpp>   // Remote parameter manipulation
#include <hector_testing_utils/graph_introspection.hpp> // Suggestion generation
#include <hector_testing_utils/log_capture.hpp>         // LogCapture
#include <hector_testing_utils/assertions.hpp>          // ASSERT_SERVICE_EXISTS, etc.
```

## Contributing

PRs are welcome! Please ensure:
* New features include tests in `test/`
* `colcon test` passes locally
* `pre-commit run --all-files` passes

```bash
colcon build --packages-select hector_testing_utils
colcon test --packages-select hector_testing_utils
colcon test-result --verbose
```

## License

Apache 2.0
