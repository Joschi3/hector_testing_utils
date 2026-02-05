# Core Components

## TestExecutor

Wraps an executor with convenient helpers for spinning until conditions are met.

```cpp
hector_testing_utils::TestExecutor executor;
executor.add_node(node);

// Spin until a condition is met
bool result = executor.spin_until(
  []() { return some_condition(); },
  5s
);

// Spin until a future completes
auto future = client->async_send_request(request);
ASSERT_TRUE(executor.spin_until_future_complete(future, 5s));
```

## TestNode

Enhanced node class with factory methods and connection tracking.

```cpp
auto pub = tester_node_->create_test_publisher<std_msgs::msg::String>("/topic");
auto sub = tester_node_->create_test_subscription<std_msgs::msg::String>("/topic");

// Wait for all entities to connect
ASSERT_TRUE(tester_node_->wait_for_all_connections(*executor_, 5s));
```

## TestContext

Manages a scoped ROS 2 context for test isolation.

```cpp
hector_testing_utils::TestContext context;
auto options = context.node_options();
auto node = std::make_shared<rclcpp::Node>("my_node", options);
```

## Test Fixtures

### HectorTestFixture

Base fixture with pre-configured executor and test node.

```cpp
TEST_F(HectorTestFixture, MyTest)
{
  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>("/topic");
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>("/topic");
  ASSERT_TRUE(tester_node_->wait_for_all_connections(*executor_, 5s));
}
```

### HectorTestFixtureWithContext

Isolated context per test for shared test processes.

```cpp
class MyTest : public hector_testing_utils::HectorTestFixtureWithContext
{
  // Each test has its own isolated ROS 2 context
};
```

## Connection-Aware Wrappers

| Class | Description |
|-------|-------------|
| `TestPublisher` | Publisher wrapper that tracks subscriber connections |
| `TestSubscription` | Subscription wrapper that caches messages and tracks publisher connections |
| `TestClient` | Service client wrapper with connection awareness |
| `TestServiceServer` | Service server wrapper that tracks client connections |
| `TestActionClient` | Action client wrapper with server readiness checking |
| `TestActionServer` | Action server wrapper that tracks client connections |

## Wait Helpers

| Function | Description |
|----------|-------------|
| `wait_for_publishers` | Wait for publishers to appear on a topic |
| `wait_for_subscribers` | Wait for subscribers to appear on a topic |
| `wait_for_service` | Wait for a service to become available |
| `wait_for_action_server` | Wait for an action server to become available |
| `wait_for_message` | Wait for a message, optionally with a predicate |
| `wait_for_new_message` | Wait for any new message after the current count |
| `call_service` | Call a service with automatic waiting and timeout handling |
| `call_action` | Send an action goal with automatic waiting and result retrieval |
| `wait_for_all_connections` | Wait for all entities to be connected |

## Assertions & Macros

```cpp
ASSERT_SERVICE_EXISTS(node, "/my_service", 2s);
EXPECT_SERVICE_EXISTS(node, "/my_service", 2s);
ASSERT_ACTION_EXISTS(node, "/my_action", 2s);
EXPECT_ACTION_EXISTS(node, "/my_action", 2s);
```

## Utility Functions

| Function | Description |
|----------|-------------|
| `node_options_from_yaml` | Load parameters from a YAML file into NodeOptions |
| `LogCapture` | Helper to verify ROS log messages (singleton) |
