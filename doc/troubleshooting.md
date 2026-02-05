# Troubleshooting

This guide covers common issues encountered when using `hector_testing_utils` and how to resolve them.

## Timeout Issues

### Connection Timeouts

**Symptom:** Tests fail with "Timeout waiting for connection" or similar messages.

**Common Causes:**

1. **QoS Mismatch** - Publisher and subscriber have incompatible QoS settings
2. **Missing spin** - The executor isn't spinning during the wait
3. **Wrong topic name** - Typo in topic name or namespace issues
4. **Node not started** - The node under test hasn't been added to the executor

**Solutions:**

```cpp
// 1. Check QoS compatibility first
auto compat = hector_testing_utils::check_topic_qos_compatibility(
    node, "/my_topic", my_publisher_qos);
if (!compat.compatible) {
    std::cerr << "QoS issues: " << compat.errors << std::endl;
}

// 2. Ensure executor has all nodes
executor.add_node(test_node_);
executor.add_node(node_under_test_);

// 3. Use the auto-suggestion feature to find similar topics
auto pub = test_node_->create_test_publisher<std_msgs::msg::String>("/my_topic");
// If connection fails, error message will suggest similar topic names
```

### Service/Action Timeouts

**Symptom:** `call_service()` or `call_action()` returns nullptr/nullopt.

**Solutions:**

```cpp
// Check if service exists first
GraphMonitor monitor(test_node_);
monitor.start();
bool found = monitor.wait_for_service(executor, "/my_service", 5s);
if (!found) {
    // Service doesn't exist - check node is running
    auto services = monitor.get_services();
    // Print available services for debugging
}

// Use custom timeouts for slow services
ServiceCallOptions options;
options.service_timeout = std::chrono::seconds(10);
options.response_timeout = std::chrono::seconds(30);
auto response = call_service<MySrv>(client, request, executor, options);
```

## QoS Issues

### Diagnosing QoS Problems

When publishers and subscribers don't connect, QoS incompatibility is often the cause.

```cpp
// Get detailed QoS information for a topic
auto summary = hector_testing_utils::get_topic_qos_summary(node, "/my_topic");
std::cout << summary << std::endl;

// Check specific compatibility
auto result = hector_testing_utils::check_topic_qos_compatibility(
    node, "/my_topic", expected_qos);
if (!result.compatible) {
    std::cerr << "QoS mismatch:\n" << result.errors << std::endl;
}
```

### Common QoS Mismatches

| Publisher | Subscriber | Result |
|-----------|------------|--------|
| `RELIABLE` | `BEST_EFFORT` | ✓ Works |
| `BEST_EFFORT` | `RELIABLE` | ✗ No connection |
| `TRANSIENT_LOCAL` | `VOLATILE` | ✓ Works (no late-join) |
| `VOLATILE` | `TRANSIENT_LOCAL` | ✗ No late-join data |

**Fix:** Match durability and use compatible reliability settings:

```cpp
// For latched/transient-local topics
rclcpp::QoS qos(10);
qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

auto sub = test_node_->create_test_subscription<MyMsg>(
    "/latched_topic", qos, callback, /*latched=*/true);
```

## Graph Discovery Issues

### Node/Topic Not Found

**Symptom:** `GraphMonitor::has_node()` or `has_topic()` returns false unexpectedly.

**Solutions:**

```cpp
// 1. Ensure graph has time to propagate
GraphMonitor monitor(test_node_);
monitor.start();

// Wait for the node explicitly
bool found = monitor.wait_for_node(executor, "/target_node", 5s);

// 2. Check exact naming (namespaces matter)
auto nodes = monitor.get_nodes();
for (const auto& name : nodes) {
    std::cout << "Found node: " << name << std::endl;
}

// 3. Remember that topic names must include leading slash
// Wrong: "my_topic"
// Right: "/my_topic" or "/namespace/my_topic"
```

### Race Conditions in Discovery

DDS discovery takes time. Don't assume immediate availability.

```cpp
// Bad: Assumes immediate connection
auto pub = test_node_->create_test_publisher<MyMsg>("/topic");
pub->publish(msg);  // May be lost if no subscribers yet!

// Good: Wait for connection first
auto pub = test_node_->create_test_publisher<MyMsg>("/topic");
ASSERT_TRUE(pub->wait_for_subscription(executor, 5s));
pub->publish(msg);
```

## Parameter Issues

### Remote Parameter Service Not Ready

**Symptom:** `RemoteParameterClient` operations fail or timeout.

**Solutions:**

```cpp
RemoteParameterClient params(test_node_, "/target_node");

// Always wait for service first
if (!params.wait_for_service(executor, 10s)) {
    // Node might not have parameter services enabled
    // Or node name might be wrong
    FAIL() << "Parameter services not available on /target_node";
}

// Now safe to use
auto result = params.set_parameter(executor, "my_param", 42);
```

### Parameter Type Mismatch

**Symptom:** `get_parameter<T>()` returns wrong value or nullopt.

**Solution:** Ensure template type matches the actual parameter type:

```cpp
// If parameter was set as int64_t
auto value = params.get_parameter<int64_t>(executor, "my_int_param");  // Correct
auto bad = params.get_parameter<int>(executor, "my_int_param");        // May fail

// Check if parameter exists first
if (params.has_parameter(executor, "my_param")) {
    // Parameter exists, safe to get
}
```

## Test Fixture Issues

### Tests Interfering With Each Other

**Symptom:** Tests pass individually but fail when run together.

**Solutions:**

1. **Use unique namespaces per test:**

```cpp
class MyTest : public HectorTestFixture {
protected:
    void SetUp() override {
        // Create unique namespace using test name
        auto info = ::testing::UnitTest::GetInstance()->current_test_info();
        std::string ns = std::string("/") + info->name();

        rclcpp::NodeOptions options;
        options.arguments({"--ros-args", "-r", "__ns:=" + ns});
        test_node_ = create_test_node("test_node", options);

        HectorTestFixture::SetUp();
    }
};
```

2. **Use `HectorTestFixtureWithContext` for isolation:**

```cpp
class IsolatedTest : public HectorTestFixtureWithContext {
    // Each test gets its own rclcpp context
    // Complete isolation from other tests
};
```

3. **Clear subscription buffers between tests:**

```cpp
// In test body, reset subscription to clear old messages
subscription->reset();
```

### Leftover State From Previous Tests

**Symptom:** Test receives unexpected messages from previous test.

**Solution:**

```cpp
void SetUp() override {
    HectorTestFixture::SetUp();

    // Wait a moment for cleanup from previous test
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Or use context isolation
}
```

## Log Capture Issues

### Logs Not Being Captured

**Symptom:** `LogCapture::wait_for_log()` times out even though logs are being printed.

**Solutions:**

```cpp
// 1. Create LogCapture before the action that generates logs
LogCapture capture(test_node_);

// 2. Then trigger the action
node_under_test->do_something_that_logs();

// 3. Wait for the log
bool found = capture.wait_for_log(
    executor,
    "expected message",
    RCUTILS_LOG_SEVERITY_INFO,
    5s);

// 4. If still not working, check log level
// Logs below the current threshold won't be captured
```

## Build and CI Issues

### Documentation Build Fails

If `rosdoc2 build` fails:

```bash
# Ensure ROS 2 is sourced
source /opt/ros/jazzy/setup.bash

# Install rosdoc2 in a virtual environment
python3 -m venv .venv
source .venv/bin/activate
pip install rosdoc2

# Build documentation
rosdoc2 build --package-path .
```

### Tests Flaky in CI

**Symptom:** Tests pass locally but fail intermittently in CI.

**Solutions:**

1. **Increase timeouts for CI:**

```cpp
// Use longer timeouts in CI environments
auto timeout = std::getenv("CI") ? 10s : 5s;
ASSERT_TRUE(pub->wait_for_subscription(executor, timeout));
```

2. **Add explicit waits for graph discovery:**

```cpp
// Don't assume immediate discovery
executor.spin_some();
std::this_thread::sleep_for(100ms);
executor.spin_some();
```

3. **Use deterministic test ordering** if tests share resources.

## Getting Help

If you're still stuck:

1. **Enable verbose logging:**

```cpp
// Set ROS log level to DEBUG
rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
```

2. **Use the auto-suggestion feature** - timeout errors will suggest similar names

3. **Check QoS compatibility** - most connection issues are QoS related

4. **Use GraphMonitor** to inspect the actual graph state:

```cpp
GraphMonitor monitor(test_node_);
std::cout << "Nodes: ";
for (const auto& n : monitor.get_nodes()) std::cout << n << " ";
std::cout << "\nTopics: ";
for (const auto& t : monitor.get_topics()) std::cout << t << " ";
std::cout << "\nServices: ";
for (const auto& s : monitor.get_services()) std::cout << s << " ";
```

5. **File an issue** with a minimal reproducible example at the project repository.
