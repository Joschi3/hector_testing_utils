#include <chrono>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>

#include <hector_testing_utils/hector_testing_utils.hpp>

using namespace std::chrono_literals;
using hector_testing_utils::HectorTestFixture;

// Integration test demonstrating multi-node communication
TEST_F( HectorTestFixture, MultiNodeCommunication )
{
  // Create a publisher node
  auto publisher_node = std::make_shared<rclcpp::Node>( "publisher_node" );
  executor_->add_node( publisher_node );

  // Create a subscriber node
  auto subscriber_node = std::make_shared<rclcpp::Node>( "subscriber_node" );
  executor_->add_node( subscriber_node );

  const std::string topic = "/multi_node/test_topic";

  auto pub = publisher_node->create_publisher<std_msgs::msg::String>( topic, 10 );
  hector_testing_utils::TestSubscription<std_msgs::msg::String> sub( subscriber_node, topic );

  ASSERT_TRUE( sub.wait_for_publishers( *executor_, 1, 5s ) );

  std_msgs::msg::String msg;
  msg.data = "Hello from integration test!";
  pub->publish( msg );

  ASSERT_TRUE( sub.wait_for_message( *executor_, 5s ) );
  auto last = sub.last_message();
  ASSERT_TRUE( last.has_value() );
  EXPECT_EQ( last->data, "Hello from integration test!" );
}

// Test parameter loading and service communication
TEST_F( HectorTestFixture, ParameterLoadingWithService )
{
  const std::string params_file = std::string( TEST_DATA_DIR ) + "/test_params.yaml";
  auto options = hector_testing_utils::node_options_from_yaml( params_file );
  auto param_node = std::make_shared<rclcpp::Node>( "param_node", options );
  executor_->add_node( param_node );

  int64_t foo = 0;
  ASSERT_TRUE( param_node->get_parameter( "foo", foo ) );
  EXPECT_EQ( foo, 42 );

  // Now use this node to create a service
  using Service = example_interfaces::srv::AddTwoInts;
  const std::string service_name = "/add_with_params";

  auto server = tester_node_->create_test_service_server<Service>(
      service_name, [foo]( const std::shared_ptr<Service::Request> request,
                           std::shared_ptr<Service::Response> response ) {
        response->sum = request->a + request->b + foo;
      } );

  auto client = tester_node_->create_test_client<Service>( service_name );

  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 5s ) );

  auto request = std::make_shared<Service::Request>();
  request->a = 10;
  request->b = 20;

  auto future = client->get()->async_send_request( request );
  ASSERT_TRUE( executor_->spin_until_future_complete( future, 5s ) );
  auto response = future.get();
  ASSERT_NE( response, nullptr );
  EXPECT_EQ( response->sum, 72 ); // 10 + 20 + 42
}

// Test bidirectional communication with multiple publishers/subscribers
TEST_F( HectorTestFixture, BidirectionalCommunication )
{
  const std::string topic_a = "/bidirectional/topic_a";
  const std::string topic_b = "/bidirectional/topic_b";

  auto pub_a = tester_node_->create_test_publisher<std_msgs::msg::Int32>( topic_a );
  auto sub_a = tester_node_->create_test_subscription<std_msgs::msg::Int32>( topic_a );

  auto pub_b = tester_node_->create_test_publisher<std_msgs::msg::Int32>( topic_b );
  auto sub_b = tester_node_->create_test_subscription<std_msgs::msg::Int32>( topic_b );

  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 5s ) );

  // Send message on topic A
  std_msgs::msg::Int32 msg_a;
  msg_a.data = 100;
  pub_a->publish( msg_a );

  ASSERT_TRUE( sub_a->wait_for_message( *executor_, 5s ) );
  auto received_a = sub_a->last_message();
  ASSERT_TRUE( received_a.has_value() );
  EXPECT_EQ( received_a->data, 100 );

  // Send message on topic B
  std_msgs::msg::Int32 msg_b;
  msg_b.data = 200;
  pub_b->publish( msg_b );

  ASSERT_TRUE( sub_b->wait_for_message( *executor_, 5s ) );
  auto received_b = sub_b->last_message();
  ASSERT_TRUE( received_b.has_value() );
  EXPECT_EQ( received_b->data, 200 );
}

// Test multiple messages with predicates
TEST_F( HectorTestFixture, MessagePredicateFiltering )
{
  const std::string topic = "/predicate_test";

  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>( topic );
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>( topic );

  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 5s ) );

  // Publish multiple messages
  for ( int i = 1; i <= 10; ++i ) {
    std_msgs::msg::Int32 msg;
    msg.data = i;
    pub->publish( msg );
    std::this_thread::sleep_for( 10ms );
  }

  // Wait for a message that's greater than 7
  ASSERT_TRUE( sub->wait_for_message(
      *executor_, 5s, []( const std_msgs::msg::Int32 &msg ) { return msg.data > 7; } ) );

  auto last = sub->last_message();
  ASSERT_TRUE( last.has_value() );
  EXPECT_GT( last->data, 7 );
}

// Test latched/transient local messages
TEST_F( HectorTestFixture, LatchedMessages )
{
  const std::string topic = "/latched_test";

  // Create publisher with transient local durability
  auto qos = rclcpp::QoS( rclcpp::KeepLast( 1 ) );
  qos.durability( RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL );
  qos.reliability( RMW_QOS_POLICY_RELIABILITY_RELIABLE );

  auto pub = tester_node_->create_publisher<std_msgs::msg::String>( topic, qos );

  // Publish message before subscriber exists
  std_msgs::msg::String msg;
  msg.data = "Latched message";
  pub->publish( msg );

  std::this_thread::sleep_for( 100ms ); // Give DDS time to persist

  // Now create subscriber with matching QoS and latched flag
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::String>( topic, qos,
                                                                            true /* latched */ );

  // Subscriber should receive the latched message
  ASSERT_TRUE( sub->wait_for_message( *executor_, 5s ) );
  auto received = sub->last_message();
  ASSERT_TRUE( received.has_value() );
  EXPECT_EQ( received->data, "Latched message" );
}

// Test waiting for multiple connections with diagnostic output
TEST_F( HectorTestFixture, ConnectionDiagnostics )
{
  const std::string topic = "/diagnostic_test";

  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>( topic );
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>( topic );

  std::vector<hector_testing_utils::WaitFailureInfo> failure_infos;
  bool result = tester_node_->wait_for_all_connections( *executor_, 5s, failure_infos );

  ASSERT_TRUE( result );
  EXPECT_TRUE( failure_infos.empty() ); // Should be empty on success
}

// Test helper functions for waiting on publishers and subscribers
TEST_F( HectorTestFixture, WaitHelpers )
{
  const std::string topic = "/wait_helpers_test";

  auto pub = tester_node_->create_publisher<std_msgs::msg::Int32>( topic, 10 );

  // Create a shared pointer to store the subscription
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> sub_holder;

  // Start a background thread that creates a subscriber after a delay
  std::thread delayed_sub( [this, topic, &sub_holder]() {
    std::this_thread::sleep_for( 500ms );
    sub_holder = tester_node_->create_subscription<std_msgs::msg::Int32>(
        topic, 10, []( std_msgs::msg::Int32::SharedPtr ) { } );
  } );

  // Wait for at least 1 subscriber
  ASSERT_TRUE( hector_testing_utils::wait_for_subscribers( *executor_, tester_node_, topic, 1, 2s ) );

  delayed_sub.join();
}

// Test call_service helper
TEST_F( HectorTestFixture, CallServiceHelper )
{
  using Service = example_interfaces::srv::AddTwoInts;
  const std::string service_name = "/call_service_test";

  auto server = tester_node_->create_test_service_server<Service>(
      service_name, []( const std::shared_ptr<Service::Request> request,
                        std::shared_ptr<Service::Response> response ) {
        response->sum = request->a + request->b;
      } );

  auto client = tester_node_->create_test_client<Service>( service_name );
  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 5s ) );

  auto request = std::make_shared<Service::Request>();
  request->a = 15;
  request->b = 27;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 3s;
  options.response_timeout = 3s;

  auto response =
      hector_testing_utils::call_service<Service>( client->get(), request, *executor_, options );

  ASSERT_NE( response, nullptr );
  EXPECT_EQ( response->sum, 42 );
}
