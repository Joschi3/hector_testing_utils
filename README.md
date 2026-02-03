
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy%20|%20Kilted%20|%20Rolling-blue)](https://docs.ros.org)
![Lint](https://github.com/Joschi3/hector_testing_utils/actions/workflows/lint_build_test.yaml/badge.svg)
[![codecov](https://codecov.io/gh/Joschi3/hector_testing_utils/graph/badge.svg?token=RYR8J8FNC8)](https://codecov.io/gh/Joschi3/hector_testing_utils)

# hector_testing_utils

**Helper classes and utilities for writing robust ROS 2 integration tests using the real ROS graph and middleware.**

`hector_testing_utils` is designed for testing ROS 2 systems **where mocking rclcpp is not sufficient**.
It enables deterministic, connection-aware Google Tests that interact with **actual DDS/RMW behavior**, while providing structured helpers to reduce flakiness.


## Table of Contents
* [Comparison with rtest](#comparison-with-rtest)
* [Integration](#integration)
* [Core Components](#core-components)
* [Basic Example](#basic-example)
* [Advanced Usage](#advanced-executor-usage)
* [Tips and Best Practices](#tips-and-best-practices)


## Comparison with [rtest](https://github.com/Beam-and-Spyrosoft/rtest)

While **rtest** is an excellent tool for unit testing, it is limited by its design: it mocks `rclcpp` entirely. This makes it fast but unusable for code that relies on real middleware behavior or external libraries that manage their own ROS entities.

**hector_testing_utils** is designed to fill that gap. It provides a stable environment for the "general cases" where mocking isn't an option, aiming to make real integration tests as robust as possible.

| Feature | rtest | hector_testing_utils |
| --- | --- | --- |
| **Middleware** | **Mocked** (No DDS) | **Real** (Actual DDS/RMW) |
| **Execution Speed** | ⚡ Instant | 🐢 Slower |
| **Scope** | Unit Logic Only | Full Integration |
| **Limitations** | Cannot test complex library interactions | Subject to OS scheduling/timing |

**Summary:**

* **Use `rtest`** whenever possible for instant, flake-free feedback on logic, or single node ros2 communication
* **Use `hector_testing_utils`** when `rtest` is too restrictive (e.g. complex node interactions, or opaque libraries that manage their own ROS entities internally).

## Integration

**Compatibility:**
* **ROS 2 Distributions**: Jazzy, Kilted, Rolling (Iron/Humble should work but are not actively CI tested)
* **C++ Standard**: C++17 or later

To use `hector_testing_utils` in your ROS 2 package, add it as a test dependency.

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

### Modular Headers

The library is organized into modular headers. You can include the main header for full functionality:

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

## What is included

### Core Components

* **TestExecutor**: Wraps an executor with convenient helpers for spinning until conditions are met
* **TestNode**: Enhanced node class with factory methods and connection tracking (e.g. wait until all entities are connected)
* **TestContext**: Manages a scoped ROS 2 context for test isolation
* **HectorTestFixture / HectorTestFixtureWithContext**: Google Test fixtures with pre-configured executor and test node

### Connection-Aware Wrappers

* **TestPublisher**: Publisher wrapper that tracks subscriber connections
* **TestSubscription**: Subscription wrapper that caches messages and tracks publisher connections
* **TestClient**: Service client wrapper with connection awareness
* **TestServiceServer**: Service server wrapper that tracks client connections
* **TestActionClient**: Action client wrapper with server readiness checking
* **TestActionServer**: Action server wrapper that tracks client connections


### Wait Helpers

* `wait_for_publishers`: Wait for publishers to appear on a topic
* `wait_for_subscribers`: Wait for subscribers to appear on a topic
* `wait_for_service`: Wait for a service to become available
* `wait_for_action_server`: Wait for an action server to become available
* `wait_for_message`: Wait for a message, optionally with a predicate
* `wait_for_new_message`: Wait for any new message after the current count
* `call_service`: Call a service with automatic waiting and timeout handling
* `call_action`: Send an action goal with automatic waiting and result retrieval
* `wait_for_all_connections`: Wait for all entities to be connected

### Graph Introspection & Suggestions

When a wait operation times out, the library automatically provides **suggestions for similar entities** in the ROS graph. This helps debug test failures by showing what's actually available:

* `collect_available_topics`: Get all topics in the ROS graph
* `collect_available_services`: Get all services in the ROS graph
* `collect_available_actions`: Get all actions in the ROS graph
* `suggest_similar_topics`: Find topics similar to a given name (using Levenshtein distance)
* `suggest_similar_services`: Find services similar to a given name
* `suggest_similar_actions`: Find actions similar to a given name

### QoS Validation & Diagnostics

Diagnose QoS incompatibilities that can cause silent connection failures:

* `QoSInfo`: Struct representing QoS settings with human-readable formatting
* `check_qos_compatibility`: Check if publisher/subscriber QoS settings are compatible
* `get_publishers_info` / `get_subscribers_info`: Get QoS info for all endpoints on a topic
* `diagnose_topic_qos`: Generate a full diagnostic report for a topic
* `get_topic_qos_summary`: Get a compact QoS summary string
* `get_qos_compatibility_hint`: Get a hint about QoS incompatibilities

### Graph Change Monitoring

Monitor the ROS graph for changes (nodes, topics, services appearing/disappearing):

* `GraphMonitor`: Class for tracking graph changes over time
* `wait_for_node` / `wait_for_node_removed`: Wait for nodes to appear or disappear
* `wait_for_topic` / `wait_for_topic_removed`: Wait for topics to appear or disappear
* `wait_for_service` / `wait_for_service_removed`: Wait for services to appear or disappear
* `has_node` / `has_topic` / `has_service`: Query current graph state
* `get_changes` / `get_changes_since`: Get recorded graph changes

### Remote Parameter Manipulation

Set and get parameters on other nodes during testing:

* `RemoteParameterClient`: Client for manipulating parameters on remote nodes
* `set_parameter<T>`: Set a parameter with automatic type inference
* `get_parameter<T>`: Get a parameter with type conversion
* `list_parameters`: List all parameters on a remote node
* `has_parameter`: Check if a parameter exists
* `set_remote_parameter` / `get_remote_parameter`: Convenience functions

### Assertions & Macros

* `ASSERT_SERVICE_EXISTS`: Assert that a service exists on the graph
* `EXPECT_SERVICE_EXISTS`: Expect that a service exists on the graph
* `ASSERT_ACTION_EXISTS`: Assert that an action server exists on the graph
* `EXPECT_ACTION_EXISTS`: Expect that an action server exists on the graph
* `ASSERT_SERVICE_EXISTS_WITH_EXECUTOR`: Assert service exists with custom executor
* `EXPECT_SERVICE_EXISTS_WITH_EXECUTOR`: Expect service exists with custom executor
* `ASSERT_ACTION_EXISTS_WITH_EXECUTOR`: Assert action exists with custom executor
* `EXPECT_ACTION_EXISTS_WITH_EXECUTOR`: Expect action exists with custom executor

### Utility Functions

* `node_options_from_yaml`: Load parameters from a YAML file into NodeOptions
* `LogCapture`: Helper to verify ROS log messages (singleton)


## Basic Example

```cpp
#include <gtest/gtest.h>
#include <std_msgs/msg/int32.hpp>

#include <hector_testing_utils/hector_testing_utils.hpp>

using namespace std::chrono_literals;

TEST(Example, TestSubscription)
{
  auto node = std::make_shared<rclcpp::Node>("example_node");
  hector_testing_utils::TestExecutor executor;
  executor.add_node(node);

  const std::string topic = "/example/int32";
  auto pub = node->create_publisher<std_msgs::msg::Int32>(topic, 10);
  hector_testing_utils::TestSubscription<std_msgs::msg::Int32> sub(node, topic);

  ASSERT_TRUE(sub.wait_for_publishers(executor, 1, 5s));

  std_msgs::msg::Int32 msg;
  msg.data = 42;
  pub->publish(msg);

  ASSERT_TRUE(sub.wait_for_message(executor, 5s));
  auto last = sub.last_message();
  ASSERT_TRUE(last.has_value());
  EXPECT_EQ(last->data, 42);
}
```

## Using Test Fixtures

The `HectorTestFixture` class provides a convenient base for tests with a pre-configured executor and test node:

```cpp
#include <gtest/gtest.h>
#include <hector_testing_utils/hector_testing_utils.hpp>

using namespace std::chrono_literals;
using hector_testing_utils::HectorTestFixture;

TEST_F(HectorTestFixture, SimplePublisherSubscriber)
{
  const std::string topic = "/test_topic";

  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>(topic);
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>(topic);

  // Wait for all connections to be established
  ASSERT_TRUE(tester_node_->wait_for_all_connections(*executor_, 5s));

  std_msgs::msg::Int32 msg;
  msg.data = 100;
  pub->publish(msg);

  ASSERT_TRUE(sub->wait_for_message(*executor_, 5s));
  auto received = sub->last_message();
  ASSERT_TRUE(received.has_value());
  EXPECT_EQ(received->data, 100);
}
```

## TestNode Factory Methods

`TestNode` provides factory methods that automatically register connectables and enable connection tracking:

```cpp
TEST_F(HectorTestFixture, FactoryMethods)
{
  // Create publisher and subscriber with factory methods
  auto pub = tester_node_->create_test_publisher<std_msgs::msg::String>("/topic");
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::String>("/topic");

  // Create service client and server
  auto client = tester_node_->create_test_client<example_interfaces::srv::AddTwoInts>("/service");
  auto server = tester_node_->create_test_service_server<example_interfaces::srv::AddTwoInts>(
    "/service",
    [](auto request, auto response) { response->sum = request->a + request->b; });

  // Wait for everything to connect
  ASSERT_TRUE(tester_node_->wait_for_all_connections(*executor_, 5s));
}
```

## Service Testing

```cpp
TEST_F(HectorTestFixture, ServiceTest)
{
  using Service = example_interfaces::srv::AddTwoInts;
  const std::string service_name = "/add_two_ints";

  auto server = tester_node_->create_test_service_server<Service>(
    service_name,
    [](const std::shared_ptr<Service::Request> request,
       std::shared_ptr<Service::Response> response) {
      response->sum = request->a + request->b;
    });

  auto client = tester_node_->create_test_client<Service>(service_name);

  ASSERT_TRUE(tester_node_->wait_for_all_connections(*executor_, 5s));
  ASSERT_SERVICE_EXISTS(tester_node_, service_name, 2s);

  auto request = std::make_shared<Service::Request>();
  request->a = 5;
  request->b = 7;

  auto future = client->get()->async_send_request(request);
  ASSERT_TRUE(executor_->spin_until_future_complete(future, 5s));
  auto response = future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->sum, 12);
}
```

## Action Testing

```cpp
TEST_F(HectorTestFixture, ActionTest)
{
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
  const std::string action_name = "/fibonacci";

  auto handle_goal =
    [](const rclcpp_action::GoalUUID &, const std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

  auto handle_cancel = [](const std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    };

  auto handle_accepted = [](const std::shared_ptr<GoalHandle> goal_handle) {
      auto result = std::make_shared<Fibonacci::Result>();
      result->sequence = {0, 1, 1, 2, 3};
      goal_handle->succeed(result);
    };

  auto server = tester_node_->create_test_action_server<Fibonacci>(
    action_name, handle_goal, handle_cancel, handle_accepted);
  auto client = tester_node_->create_test_action_client<Fibonacci>(action_name);

  ASSERT_TRUE(tester_node_->wait_for_all_connections(*executor_, 5s));
  ASSERT_ACTION_EXISTS(tester_node_, action_name, 2s);

  Fibonacci::Goal goal;
  goal.order = 5;

  auto goal_future = client->get()->async_send_goal(goal);
  ASSERT_TRUE(executor_->spin_until_future_complete(goal_future, 5s));
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);
}
```

## Scoped Context

If you need to avoid global `rclcpp::shutdown()` in shared test processes, use
`HectorTestFixtureWithContext` or manage a `TestContext` directly:

```cpp
class ScopedExample : public hector_testing_utils::HectorTestFixtureWithContext
{
  // Use tester_node_ and executor_ as usual.
};

TEST_F(ScopedExample, IsolatedTest)
{
  // Each test has its own isolated ROS 2 context
  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>("/topic");
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>("/topic");

  ASSERT_TRUE(tester_node_->wait_for_all_connections(*executor_, 5s));
}
```

## Parameter Loading

Load parameters from YAML files into your test nodes:

```cpp
TEST(ParameterTest, LoadFromYaml)
{
  const std::string params_file = "path/to/params.yaml";
  auto options = hector_testing_utils::node_options_from_yaml(params_file);
  auto node = std::make_shared<rclcpp::Node>("param_node", options);

  int64_t my_param;
  ASSERT_TRUE(node->get_parameter("my_param", my_param));
}
```

## Message Predicates

Wait for specific messages using predicates:

```cpp
TEST_F(HectorTestFixture, PredicateWait)
{
  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>("/topic");
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>("/topic");

  ASSERT_TRUE(tester_node_->wait_for_all_connections(*executor_, 5s));

  // Publish multiple messages
  for (int i = 1; i <= 10; ++i) {
    std_msgs::msg::Int32 msg;
    msg.data = i;
    pub->publish(msg);
  }

  // Wait for a message greater than 7
  ASSERT_TRUE(sub->wait_for_message(
    *executor_, 5s,
    [](const std_msgs::msg::Int32 &msg) { return msg.data > 7; }));
}
```

You can also wait for any new message without a predicate:

```cpp
// Wait for the next message after current state
ASSERT_TRUE(sub->wait_for_new_message(*executor_, 5s));
```

## Latched/Transient Local Messages

Test latched messages with transient local QoS:

```cpp
TEST_F(HectorTestFixture, LatchedMessage)
{
  const std::string topic = "/latched_topic";

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  auto pub = tester_node_->create_publisher<std_msgs::msg::String>(topic, qos);

  std_msgs::msg::String msg;
  msg.data = "Latched message";
  pub->publish(msg);

  std::this_thread::sleep_for(100ms); // Allow DDS to persist

  // Create subscriber after publishing
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::String>(
    topic, qos, true /* latched */);

  // Should receive the latched message
  ASSERT_TRUE(sub->wait_for_message(*executor_, 5s));
}
```

## Connection Diagnostics & Auto-Suggestions

When a wait operation times out, the library **automatically logs suggestions** showing similar entities in the ROS graph. This is helpful for debugging why tests aren't working - often due to incorrect namespaces, topic/service names or missing nodes.

### Automatic Suggestions on Timeout

All `wait_for_*` functions automatically log helpful suggestions when they timeout:

```cpp
TEST_F(HectorTestFixture, AutoSuggestionExample)
{
  // Typo in topic name: "temperatur" instead of "temperature"
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Float32>("/sensor/temperatur");

  // This will timeout and automatically log suggestions like:
  // [ERROR] Failed to find topic '/sensor/temperatur'
  //   Available topics (sorted by similarity):
  //     - /sensor/temperature [std_msgs/msg/Float32] (score: 0.95)
  //     - /sensor/pressure [std_msgs/msg/Float32] (score: 0.42)
  //     - /rosout [rcl_interfaces/msg/Log] (score: 0.18)
  sub->wait_for_publishers(*executor_, 1, 2s);
}
```

### Detailed Failure Information

For programmatic access to failure details, use the overloads that accept a `WaitFailureInfo` reference:

```cpp
TEST_F(HectorTestFixture, DetailedFailureInfo)
{
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>("/my_topic");

  hector_testing_utils::WaitFailureInfo failure_info;
  bool connected = sub->wait_for_publishers(*executor_, 1, 2s, failure_info);

  if (!connected) {
    // Access structured failure information
    std::cout << "Searched for: " << failure_info.searched_name << "\n";
    std::cout << "Entity kind: " << failure_info.entity_kind << "\n";

    for (const auto& suggestion : failure_info.suggestions) {
      std::cout << "  Similar: " << suggestion.name
                << " (score: " << suggestion.similarity_score << ")\n";
    }
  }
}
```

### Connection Diagnostics for Multiple Entities

For `wait_for_all_connections`, use a vector of `WaitFailureInfo`:

```cpp
TEST_F(HectorTestFixture, ConnectionDiagnostics)
{
  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>("/topic");
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>("/topic");

  std::vector<hector_testing_utils::WaitFailureInfo> failure_infos;
  bool connected = tester_node_->wait_for_all_connections(*executor_, 5s, failure_infos);

  if (!connected) {
    // Format all failures into a readable string
    std::string report = hector_testing_utils::TestNode::format_failure_infos(failure_infos);
    RCLCPP_ERROR(tester_node_->get_logger(), "Failed to connect:\n%s", report.c_str());
  }
  ASSERT_TRUE(connected);
}
```

## QoS Diagnostics

When wait operations timeout, suggestions include **QoS information** to help diagnose silent connection failures caused by incompatible QoS settings:

```cpp
TEST_F(HectorTestFixture, QoSDiagnosticsExample)
{
  // Create a subscriber with RELIABLE QoS
  rclcpp::QoS sub_qos(10);
  sub_qos.reliable();

  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>(
    "/sensor/data", sub_qos);

  // If a publisher exists with BEST_EFFORT QoS, the timeout message will show:
  // [ERROR] Failed to find topic '/sensor/data'
  //   Available topics (sorted by similarity):
  //     - /sensor/data [std_msgs/msg/Int32] (score: 1.00) [QoS INCOMPATIBLE]
  //       QoS: pubs: reliability=BEST_EFFORT, durability=VOLATILE, history=KEEP_LAST(10)
  sub->wait_for_publishers(*executor_, 1, 2s);
}
```

You can also diagnose QoS issues programmatically:

```cpp
TEST_F(HectorTestFixture, QoSDiagnosticReport)
{
  // Get a full diagnostic report for a topic
  std::string diagnosis = hector_testing_utils::diagnose_topic_qos(
    tester_node_, "/my_topic");
  std::cout << diagnosis;

  // Check compatibility before creating endpoints
  rclcpp::QoS my_qos(10);
  my_qos.reliable();

  auto result = hector_testing_utils::check_topic_qos_compatibility(
    tester_node_, "/my_topic", my_qos, false /* is_subscriber */);

  if (!result.compatible) {
    for (const auto& error : result.errors) {
      std::cerr << "QoS Error: " << error << "\n";
    }
  }
}
```

## Graph Change Monitoring

Monitor the ROS graph for nodes, topics, and services appearing or disappearing:

```cpp
TEST_F(HectorTestFixture, GraphMonitorExample)
{
  hector_testing_utils::GraphMonitor monitor(tester_node_);
  monitor.start();

  // Wait for a specific node to appear
  bool found = monitor.wait_for_node(*executor_, "/my_node", 5s);
  EXPECT_TRUE(found);

  // Wait for a topic to be created
  found = monitor.wait_for_topic(*executor_, "/my_topic", 5s);
  EXPECT_TRUE(found);

  // Query current graph state
  EXPECT_TRUE(monitor.has_node("/my_node"));
  EXPECT_TRUE(monitor.has_topic("/my_topic"));

  // Get all recorded changes
  auto changes = monitor.get_changes();
  for (const auto& change : changes) {
    std::cout << change.format() << "\n";
  }

  monitor.stop();
}
```

You can also set up callbacks for change notifications:

```cpp
TEST_F(HectorTestFixture, GraphMonitorCallback)
{
  hector_testing_utils::GraphMonitor monitor(tester_node_);

  monitor.set_change_callback([](const hector_testing_utils::GraphChange& change) {
    std::cout << "Graph changed: " << change.format() << "\n";
  });

  monitor.start();

  // Changes will now trigger the callback
  auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>("/new_topic", 10);
  monitor.wait_for_any_change(*executor_, 2s);

  monitor.stop();
}
```

## Remote Parameter Manipulation

Set and get parameters on other nodes during testing:

```cpp
TEST_F(HectorTestFixture, RemoteParameterExample)
{
  // Create a node with parameters to manipulate
  auto target_node = std::make_shared<rclcpp::Node>("target_node");
  target_node->declare_parameter("my_param", 42);
  target_node->declare_parameter("my_string", "hello");
  executor_->add_node(target_node);

  // Create a parameter client for the target node
  hector_testing_utils::RemoteParameterClient params(tester_node_, "target_node");
  ASSERT_TRUE(params.wait_for_service(*executor_, 5s));

  // Get a parameter
  auto value = params.get_parameter<int>(*executor_, "my_param");
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(value.value(), 42);

  // Set a parameter
  auto result = params.set_parameter(*executor_, "my_param", 100);
  EXPECT_TRUE(result.success);

  // Verify the change
  value = params.get_parameter<int>(*executor_, "my_param");
  EXPECT_EQ(value.value(), 100);

  // List all parameters
  auto param_names = params.list_parameters(*executor_);
  EXPECT_TRUE(params.has_parameter(*executor_, "my_param"));
}
```

Convenience functions for one-off operations:

```cpp
TEST_F(HectorTestFixture, RemoteParameterConvenience)
{
  // Set a parameter on a remote node
  auto result = hector_testing_utils::set_remote_parameter(
    tester_node_, "target_node", "config_value", 3.14, *executor_);
  EXPECT_TRUE(result.success);

  // Get a parameter from a remote node
  auto value = hector_testing_utils::get_remote_parameter<double>(
    tester_node_, "target_node", "config_value", *executor_);
  EXPECT_DOUBLE_EQ(value.value(), 3.14);
}
```

## Log Verification

You can verify that specific log messages (warnings, errors, etc.) were published using `LogCapture`. It intercepts ROS 2 logs via `rcutils`.

```cpp
TEST_F(HectorTestFixture, LogCheck)
{
  LogCapture capture; // Automatically registers log handler

  // Trigger something that logs
  RCLCPP_WARN(tester_node_->get_logger(), "Something happened!");

  // Assert log exists (regex support)
  ASSERT_TRUE(capture.wait_for_log(*executor_, "Something happened.*", 2s));
}
```

## Timeouts and Robustness

The helpers default to conservative timeouts (see `kDefaultTimeout` and the call option structs).
For slow CI pipelines, use longer timeouts instead of tight sleeps, and prefer the `wait_for_*`
helpers to avoid race conditions.

```cpp
// Custom timeout configuration
hector_testing_utils::ServiceCallOptions options;
options.service_timeout = 10s;
options.response_timeout = 10s;

auto response = hector_testing_utils::call_service<MyService>(
  client, request, executor, options);
```

## Call Helpers

Convenient helpers for calling services and actions:

```cpp
// Service call with automatic waiting
auto request = std::make_shared<AddTwoInts::Request>();
request->a = 1;
request->b = 2;

auto response = hector_testing_utils::call_service<AddTwoInts>(
  client, request, executor);

ASSERT_NE(response, nullptr);
EXPECT_EQ(response->sum, 3);

// Action call with automatic waiting
MyAction::Goal goal;
goal.target = 100;

auto result = hector_testing_utils::call_action<MyAction>(
  action_client, goal, executor);

ASSERT_TRUE(result.has_value());
EXPECT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED);
```

## Advanced Executor Usage

 ### Custom Spinning
 The `TestExecutor` provides flexible spinning options:

 ```cpp
 TEST_F(HectorTestFixture, CustomSpinning)
 {
   // Spin until a condition is met
   bool result = executor_->spin_until(
     [this]() { return some_condition(); },
     5s
   );

   // Spin until a future completes
   auto future = client->async_send_request(request);
   ASSERT_TRUE(executor_->spin_until_future_complete(future, 5s));
 }
 ```

 ### Background Spinning

 Sometimes, especially when testing launch files or complex node interactions, you need the "System Under Test" to run continuously in the background while your test code performs checks nicely.

 Use `start_background_spinner()` to spawn a thread that spins the executor.

 ```cpp
 TEST_F(HectorTestFixture, BackgroundSpinning)
 {
   // Start spinning the nodes in a background thread
   executor_->start_background_spinner();

   // ... perform actions that require the system to be live ...

   // Use wait helpers (they also work with background spinner active!)
   ASSERT_TRUE(sub->wait_for_message(*executor_, 5s));

   // Stop spinning before tearing down (optional, destructor does it too)
   executor_->stop_background_spinner();
 }
 ```

 > [!NOTE]
 > `spin_until` and wait helpers automatically detect if the background spinner is active. If it is, they switch to "wait mode" (sleeping and checking predicate) instead of trying to spin the executor themselves.

 ### Single vs Multi-Threaded Executor

 By default, `HectorTestFixture` uses a `SingleThreadedExecutor`.

 **Use `SingleThreadedExecutor` (Default) when:**
 * You want deterministic, sequential execution.
 * Your callbacks are fast and non-blocking.
 * You want simpler debugging (no race conditions in your test nodes).

 **Use `MultiThreadedExecutor` when:**
 * You rely on Callback Groups for concurrent execution (e.g. Action Servers with parallel goal execution).
 * You have blocking callbacks (bad practice, but happens).
 * You want to simulate real deployment behavior more closely.

 **How to override:**

 ```cpp
 class MyMultiThreadedTest : public hector_testing_utils::HectorTestFixture
 {
 protected:
   std::shared_ptr<rclcpp::Executor> create_test_executor() override
   {
     return std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
   }
 };
 ```

## Tips and Best Practices

1. **Use factory methods**: Prefer `create_test_publisher()` over `create_publisher()` to get automatic connection tracking
2. **Always wait for connections**: Use `wait_for_all_connections()` before publishing to avoid race conditions
3. **Use predicates**: Filter messages with predicates instead of checking values in a loop
4. **Set appropriate timeouts**: Use longer timeouts on CI systems; prefer waiting over sleeping
5. **Reset subscriptions**: Call `reset()` on subscriptions between test phases to clear cached messages
6. **Check auto-suggestions on timeout**: When a wait times out, look at the logged suggestions - they show similar entities that exist in the graph, helping catch typos and misconfiguration
7. **Isolate contexts**: Use `HectorTestFixtureWithContext` when running tests in shared processes
8. **Match QoS policies**: Ensure publishers and subscribers use compatible QoS settings (reliability, durability) to avoid silent connection failures
9. **Use `wait_for_new_message()`**: When you need to wait for the next message regardless of content
10. **Leverage timing helpers**: Use the built-in timing and sequencing helpers for robust, deterministic tests
11. **Domain ID Isolation**: When running tests in parallel on the same machine, ensure your test fixture handles Domain ID isolation if you are not using `TestContext` (which attempts to handle this). `hector_testing_utils` attempts to generate unique Domain IDs, but be aware of shared DDS limits.
12. **Use detailed failure info**: For programmatic debugging, use the `WaitFailureInfo` overloads to get structured information about what went wrong
13. **Check QoS compatibility**: When connections aren't working, use `diagnose_topic_qos()` or check the QoS hints in timeout suggestions - QoS mismatches are a common cause of silent failures
14. **Use GraphMonitor for dynamic tests**: When testing systems where nodes/topics appear dynamically, use `GraphMonitor` to wait for specific graph states instead of arbitrary sleeps
15. **Manipulate remote parameters**: Use `RemoteParameterClient` to dynamically configure nodes under test - useful for testing parameter change handlers and different configurations

### Testing Lifecycle Nodes

When testing `LifecycleNodes`, remember that they do not automatically start. You must trigger their transitions:

```cpp
// Assuming 'my_node' is a LifecycleNode
auto state = my_node->configure();
ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

state = my_node->activate();
ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

// Now your node is active and should be communicating
```

## Contributing

PRs are welcome! Please ensure that:

* New features include a corresponding test case in `test/`.
* `colcon test` passes locally.
* pre-commit checks pass (`pre-commit run --all-files`)


### Running Tests

Build and run the tests:

```bash
colcon build --packages-select hector_testing_utils
colcon test --packages-select hector_testing_utils
colcon test-result --verbose
```

View detailed test logs:

```bash
# Summary of all tests
colcon test-result --verbose

# Individual test logs
less log/latest_test/hector_testing_utils/stdout.log
less log/latest_test/hector_testing_utils/stderr.log
```

### Coverage Reporting

Generate coverage reports locally:

```bash
# Build with coverage enabled
colcon build --packages-select hector_testing_utils \
  --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug

# Run tests
colcon test --packages-select hector_testing_utils

# Generate coverage report
cd build/hector_testing_utils
make hector_testing_utils_coverage

# View HTML report
xdg-open coverage_report/index.html
```

Or use the provided script:

```bash
cd /path/to/ros2_workspace
./src/hector_testing_utils/scripts/generate_coverage.sh
```

The CI pipeline automatically generates coverage reports and uploads them to [Codecov](https://codecov.io/gh/Joschi3/hector_testing_utils).
