# Advanced Features

## Graph Introspection & Auto-Suggestions

When a wait operation times out, the library automatically provides **suggestions for similar entities** in the ROS graph, helping debug test failures.

### Automatic Suggestions on Timeout

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
  sub->wait_for_publishers(*executor_, 1, 2s);
}
```

### Detailed Failure Information

```cpp
TEST_F(HectorTestFixture, DetailedFailureInfo)
{
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>("/my_topic");

  hector_testing_utils::WaitFailureInfo failure_info;
  bool connected = sub->wait_for_publishers(*executor_, 1, 2s, failure_info);

  if (!connected) {
    std::cout << "Searched for: " << failure_info.searched_name << "\n";
    std::cout << "Entity kind: " << failure_info.entity_kind << "\n";

    for (const auto& suggestion : failure_info.suggestions) {
      std::cout << "  Similar: " << suggestion.name
                << " (score: " << suggestion.similarity_score << ")\n";
    }
  }
}
```

## QoS Validation & Diagnostics

Diagnose QoS incompatibilities that can cause silent connection failures.

### QoS Hints in Timeout Messages

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
  //       QoS: pubs: reliability=BEST_EFFORT, durability=VOLATILE
  sub->wait_for_publishers(*executor_, 1, 2s);
}
```

### Programmatic QoS Diagnosis

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

### QoS Helper Functions

| Function | Description |
|----------|-------------|
| `QoSInfo` | Struct representing QoS settings with human-readable formatting |
| `check_qos_compatibility` | Check if publisher/subscriber QoS settings are compatible |
| `get_publishers_info` / `get_subscribers_info` | Get QoS info for all endpoints on a topic |
| `diagnose_topic_qos` | Generate a full diagnostic report for a topic |
| `get_topic_qos_summary` | Get a compact QoS summary string |
| `get_qos_compatibility_hint` | Get a hint about QoS incompatibilities |

## Graph Change Monitoring

Monitor the ROS graph for nodes, topics, and services appearing or disappearing.

### Basic Monitoring

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

### Change Callbacks

```cpp
TEST_F(HectorTestFixture, GraphMonitorCallback)
{
  hector_testing_utils::GraphMonitor monitor(tester_node_);

  monitor.set_change_callback([](const hector_testing_utils::GraphChange& change) {
    std::cout << "Graph changed: " << change.format() << "\n";
  });

  monitor.start();

  auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>("/new_topic", 10);
  monitor.wait_for_any_change(*executor_, 2s);

  monitor.stop();
}
```

### GraphMonitor Functions

| Function | Description |
|----------|-------------|
| `wait_for_node` / `wait_for_node_removed` | Wait for nodes to appear or disappear |
| `wait_for_topic` / `wait_for_topic_removed` | Wait for topics to appear or disappear |
| `wait_for_service` / `wait_for_service_removed` | Wait for services to appear or disappear |
| `has_node` / `has_topic` / `has_service` | Query current graph state |
| `get_changes` / `get_changes_since` | Get recorded graph changes |

## Remote Parameter Manipulation

Set and get parameters on other nodes during testing.

### RemoteParameterClient

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

### Convenience Functions

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

## Background Spinning

```cpp
TEST_F(HectorTestFixture, BackgroundSpinning)
{
  // Start spinning the nodes in a background thread
  executor_->start_background_spinner();

  // ... perform actions that require the system to be live ...

  // Use wait helpers (they also work with background spinner active!)
  ASSERT_TRUE(sub->wait_for_message(*executor_, 5s));

  // Stop spinning before tearing down
  executor_->stop_background_spinner();
}
```

> **Note:** `spin_until` and wait helpers automatically detect if the background spinner is active. If it is, they switch to "wait mode" (sleeping and checking predicate) instead of trying to spin the executor themselves.

## Custom Executor Types

By default, `HectorTestFixture` uses a `SingleThreadedExecutor`.

**Use `SingleThreadedExecutor` (Default) when:**
* You want deterministic, sequential execution
* Your callbacks are fast and non-blocking
* You want simpler debugging

**Use `MultiThreadedExecutor` when:**
* You rely on Callback Groups for concurrent execution
* You have blocking callbacks
* You want to simulate real deployment behavior

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
