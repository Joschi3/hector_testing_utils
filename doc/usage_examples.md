# Usage Examples

## Basic Publisher/Subscriber Test

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

```cpp
TEST_F(HectorTestFixture, SimplePublisherSubscriber)
{
  const std::string topic = "/test_topic";

  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>(topic);
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>(topic);

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

## Message Predicates

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

## Latched/Transient Local Messages

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

## Parameter Loading

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

## Log Verification

```cpp
TEST_F(HectorTestFixture, LogCheck)
{
  LogCapture capture;

  RCLCPP_WARN(tester_node_->get_logger(), "Something happened!");

  ASSERT_TRUE(capture.wait_for_log(*executor_, "Something happened.*", 2s));
}
```

## Call Helpers

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
