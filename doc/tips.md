# Tips and Best Practices

## General Guidelines

1. **Use factory methods**: Prefer `create_test_publisher()` over `create_publisher()` to get automatic connection tracking

2. **Always wait for connections**: Use `wait_for_all_connections()` before publishing to avoid race conditions

3. **Use predicates**: Filter messages with predicates instead of checking values in a loop

4. **Set appropriate timeouts**: Use longer timeouts on CI systems; prefer waiting over sleeping

5. **Reset subscriptions**: Call `reset()` on subscriptions between test phases to clear cached messages

6. **Check auto-suggestions on timeout**: When a wait times out, look at the logged suggestions - they show similar entities that exist in the graph

7. **Isolate contexts**: Use `HectorTestFixtureWithContext` when running tests in shared processes

8. **Match QoS policies**: Ensure publishers and subscribers use compatible QoS settings to avoid silent connection failures

9. **Use `wait_for_new_message()`**: When you need to wait for the next message regardless of content

10. **Leverage timing helpers**: Use the built-in timing and sequencing helpers for robust, deterministic tests

11. **Domain ID Isolation**: When running tests in parallel on the same machine, ensure your test fixture handles Domain ID isolation

12. **Use detailed failure info**: For programmatic debugging, use the `WaitFailureInfo` overloads to get structured information

13. **Check QoS compatibility**: When connections aren't working, use `diagnose_topic_qos()` or check the QoS hints

14. **Use GraphMonitor for dynamic tests**: When testing systems where nodes/topics appear dynamically, use `GraphMonitor` instead of arbitrary sleeps

15. **Manipulate remote parameters**: Use `RemoteParameterClient` to dynamically configure nodes under test

## Testing Lifecycle Nodes

When testing `LifecycleNodes`, remember they do not automatically start. You must trigger their transitions:

```cpp
// Assuming 'my_node' is a LifecycleNode
auto state = my_node->configure();
ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

state = my_node->activate();
ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

// Now your node is active and should be communicating
```

## Timeouts and Robustness

The helpers default to conservative timeouts. For slow CI pipelines, use longer timeouts instead of tight sleeps:

```cpp
hector_testing_utils::ServiceCallOptions options;
options.service_timeout = 10s;
options.response_timeout = 10s;

auto response = hector_testing_utils::call_service<MyService>(
  client, request, executor, options);
```

## Running Tests

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

## Coverage Reporting

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
