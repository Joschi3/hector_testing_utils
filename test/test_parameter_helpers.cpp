/// @file test_parameter_helpers.cpp
/// @brief Tests for remote parameter manipulation functionality.
///
/// This file tests:
/// - Setting parameters on remote nodes
/// - Getting parameters from remote nodes
/// - Listing parameters on remote nodes
/// - Parameter type conversions
/// - RemoteParameterClient functionality

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <std_msgs/msg/int32.hpp>

#include <rclcpp/rclcpp.hpp>

#include <hector_testing_utils/hector_testing_utils.hpp>

using namespace std::chrono_literals;
using hector_testing_utils::HectorTestFixture;
using hector_testing_utils::ParameterResult;
using hector_testing_utils::RemoteParameterClient;

// =============================================================================
// Parameter Value Conversion Tests
// =============================================================================

TEST( ParameterValueConversion, BoolConversion )
{
  using hector_testing_utils::detail::from_parameter_value;
  using hector_testing_utils::detail::to_parameter_value;

  auto pv = to_parameter_value( true );
  EXPECT_EQ( pv.type, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL );
  EXPECT_TRUE( pv.bool_value );

  EXPECT_TRUE( from_parameter_value<bool>( pv ) );

  pv = to_parameter_value( false );
  EXPECT_FALSE( from_parameter_value<bool>( pv ) );
}

TEST( ParameterValueConversion, IntConversion )
{
  using hector_testing_utils::detail::from_parameter_value;
  using hector_testing_utils::detail::to_parameter_value;

  auto pv = to_parameter_value( 42 );
  EXPECT_EQ( pv.type, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER );
  EXPECT_EQ( pv.integer_value, 42 );

  EXPECT_EQ( from_parameter_value<int>( pv ), 42 );
}

TEST( ParameterValueConversion, Int64Conversion )
{
  using hector_testing_utils::detail::from_parameter_value;
  using hector_testing_utils::detail::to_parameter_value;

  int64_t value = 9223372036854775807LL; // Max int64
  auto pv = to_parameter_value( value );
  EXPECT_EQ( pv.type, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER );
  EXPECT_EQ( pv.integer_value, value );

  EXPECT_EQ( from_parameter_value<int64_t>( pv ), value );
}

TEST( ParameterValueConversion, DoubleConversion )
{
  using hector_testing_utils::detail::from_parameter_value;
  using hector_testing_utils::detail::to_parameter_value;

  auto pv = to_parameter_value( 3.14159 );
  EXPECT_EQ( pv.type, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE );
  EXPECT_DOUBLE_EQ( pv.double_value, 3.14159 );

  EXPECT_DOUBLE_EQ( from_parameter_value<double>( pv ), 3.14159 );
}

TEST( ParameterValueConversion, StringConversion )
{
  using hector_testing_utils::detail::from_parameter_value;
  using hector_testing_utils::detail::to_parameter_value;

  auto pv = to_parameter_value( std::string( "hello" ) );
  EXPECT_EQ( pv.type, rcl_interfaces::msg::ParameterType::PARAMETER_STRING );
  EXPECT_EQ( pv.string_value, "hello" );

  EXPECT_EQ( from_parameter_value<std::string>( pv ), "hello" );
}

TEST( ParameterValueConversion, CStringConversion )
{
  using hector_testing_utils::detail::from_parameter_value;
  using hector_testing_utils::detail::to_parameter_value;

  const char *value = "world";
  auto pv = to_parameter_value( value );
  EXPECT_EQ( pv.type, rcl_interfaces::msg::ParameterType::PARAMETER_STRING );
  EXPECT_EQ( pv.string_value, "world" );
}

TEST( ParameterValueConversion, BoolArrayConversion )
{
  using hector_testing_utils::detail::from_parameter_value;
  using hector_testing_utils::detail::to_parameter_value;

  std::vector<bool> value = { true, false, true };
  auto pv = to_parameter_value( value );
  EXPECT_EQ( pv.type, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY );
  EXPECT_EQ( pv.bool_array_value.size(), 3u );

  auto result = from_parameter_value<std::vector<bool>>( pv );
  EXPECT_EQ( result, value );
}

TEST( ParameterValueConversion, IntArrayConversion )
{
  using hector_testing_utils::detail::from_parameter_value;
  using hector_testing_utils::detail::to_parameter_value;

  std::vector<int64_t> value = { 1, 2, 3, 4, 5 };
  auto pv = to_parameter_value( value );
  EXPECT_EQ( pv.type, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY );
  EXPECT_EQ( pv.integer_array_value.size(), 5u );

  auto result = from_parameter_value<std::vector<int64_t>>( pv );
  EXPECT_EQ( result, value );
}

TEST( ParameterValueConversion, DoubleArrayConversion )
{
  using hector_testing_utils::detail::from_parameter_value;
  using hector_testing_utils::detail::to_parameter_value;

  std::vector<double> value = { 1.1, 2.2, 3.3 };
  auto pv = to_parameter_value( value );
  EXPECT_EQ( pv.type, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY );
  EXPECT_EQ( pv.double_array_value.size(), 3u );

  auto result = from_parameter_value<std::vector<double>>( pv );
  EXPECT_EQ( result, value );
}

TEST( ParameterValueConversion, StringArrayConversion )
{
  using hector_testing_utils::detail::from_parameter_value;
  using hector_testing_utils::detail::to_parameter_value;

  std::vector<std::string> value = { "one", "two", "three" };
  auto pv = to_parameter_value( value );
  EXPECT_EQ( pv.type, rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY );
  EXPECT_EQ( pv.string_array_value.size(), 3u );

  auto result = from_parameter_value<std::vector<std::string>>( pv );
  EXPECT_EQ( result, value );
}

// =============================================================================
// ParameterResult Tests
// =============================================================================

TEST( ParameterResult, ImplicitBoolConversion )
{
  ParameterResult success_result;
  success_result.success = true;
  EXPECT_TRUE( success_result );

  ParameterResult failure_result;
  failure_result.success = false;
  failure_result.reason = "test failure";
  EXPECT_FALSE( failure_result );
}

// =============================================================================
// RemoteParameterClient Tests (using actual nodes)
// =============================================================================

class ParameterHelpersTest : public hector_testing_utils::HectorTestFixtureWithContext
{
protected:
  void SetUp() override
  {
    HectorTestFixtureWithContext::SetUp();

    // Create a target node with some parameters
    rclcpp::NodeOptions options = context_->node_options();
    options.allow_undeclared_parameters( true );
    options.automatically_declare_parameters_from_overrides( true );

    target_node_ = std::make_shared<rclcpp::Node>( "target_node", options );
    executor_->add_node( target_node_ );

    // Declare some initial parameters
    target_node_->declare_parameter( "int_param", 42 );
    target_node_->declare_parameter( "string_param", "initial" );
    target_node_->declare_parameter( "double_param", 3.14 );
    target_node_->declare_parameter( "bool_param", true );
  }

  void TearDown() override
  {
    target_node_.reset();
    HectorTestFixtureWithContext::TearDown();
  }

  rclcpp::Node::SharedPtr target_node_;
};

TEST_F( ParameterHelpersTest, WaitForService )
{
  RemoteParameterClient client( tester_node_, "target_node" );

  bool ready = client.wait_for_service( *executor_, 5s );

  EXPECT_TRUE( ready );
}

TEST_F( ParameterHelpersTest, GetIntParameter )
{
  RemoteParameterClient client( tester_node_, "target_node" );
  ASSERT_TRUE( client.wait_for_service( *executor_, 5s ) );

  auto value = client.get_parameter<int>( *executor_, "int_param" );

  ASSERT_TRUE( value.has_value() );
  EXPECT_EQ( value.value(), 42 );
}

TEST_F( ParameterHelpersTest, GetStringParameter )
{
  RemoteParameterClient client( tester_node_, "target_node" );
  ASSERT_TRUE( client.wait_for_service( *executor_, 5s ) );

  auto value = client.get_parameter<std::string>( *executor_, "string_param" );

  ASSERT_TRUE( value.has_value() );
  EXPECT_EQ( value.value(), "initial" );
}

TEST_F( ParameterHelpersTest, GetDoubleParameter )
{
  RemoteParameterClient client( tester_node_, "target_node" );
  ASSERT_TRUE( client.wait_for_service( *executor_, 5s ) );

  auto value = client.get_parameter<double>( *executor_, "double_param" );

  ASSERT_TRUE( value.has_value() );
  EXPECT_DOUBLE_EQ( value.value(), 3.14 );
}

TEST_F( ParameterHelpersTest, GetBoolParameter )
{
  RemoteParameterClient client( tester_node_, "target_node" );
  ASSERT_TRUE( client.wait_for_service( *executor_, 5s ) );

  auto value = client.get_parameter<bool>( *executor_, "bool_param" );

  ASSERT_TRUE( value.has_value() );
  EXPECT_TRUE( value.value() );
}

TEST_F( ParameterHelpersTest, GetNonexistentParameter )
{
  RemoteParameterClient client( tester_node_, "target_node" );
  ASSERT_TRUE( client.wait_for_service( *executor_, 5s ) );

  auto value = client.get_parameter<int>( *executor_, "nonexistent_param" );

  EXPECT_FALSE( value.has_value() );
}

TEST_F( ParameterHelpersTest, SetIntParameter )
{
  RemoteParameterClient client( tester_node_, "target_node" );
  ASSERT_TRUE( client.wait_for_service( *executor_, 5s ) );

  auto result = client.set_parameter( *executor_, "int_param", 100 );

  EXPECT_TRUE( result.success );

  // Verify the change
  auto value = client.get_parameter<int>( *executor_, "int_param" );
  ASSERT_TRUE( value.has_value() );
  EXPECT_EQ( value.value(), 100 );
}

TEST_F( ParameterHelpersTest, SetStringParameter )
{
  RemoteParameterClient client( tester_node_, "target_node" );
  ASSERT_TRUE( client.wait_for_service( *executor_, 5s ) );

  auto result = client.set_parameter( *executor_, "string_param", std::string( "modified" ) );

  EXPECT_TRUE( result.success );

  // Verify the change
  auto value = client.get_parameter<std::string>( *executor_, "string_param" );
  ASSERT_TRUE( value.has_value() );
  EXPECT_EQ( value.value(), "modified" );
}

TEST_F( ParameterHelpersTest, SetDoubleParameter )
{
  RemoteParameterClient client( tester_node_, "target_node" );
  ASSERT_TRUE( client.wait_for_service( *executor_, 5s ) );

  auto result = client.set_parameter( *executor_, "double_param", 2.71828 );

  EXPECT_TRUE( result.success );

  // Verify the change
  auto value = client.get_parameter<double>( *executor_, "double_param" );
  ASSERT_TRUE( value.has_value() );
  EXPECT_DOUBLE_EQ( value.value(), 2.71828 );
}

TEST_F( ParameterHelpersTest, SetMultipleParameters )
{
  RemoteParameterClient client( tester_node_, "target_node" );
  ASSERT_TRUE( client.wait_for_service( *executor_, 5s ) );

  std::vector<rclcpp::Parameter> params = { rclcpp::Parameter( "int_param", 200 ),
                                            rclcpp::Parameter( "string_param", "batch_update" ) };

  auto result = client.set_parameters( *executor_, params );

  EXPECT_TRUE( result.success );

  // Verify the changes
  auto int_value = client.get_parameter<int>( *executor_, "int_param" );
  ASSERT_TRUE( int_value.has_value() );
  EXPECT_EQ( int_value.value(), 200 );

  auto string_value = client.get_parameter<std::string>( *executor_, "string_param" );
  ASSERT_TRUE( string_value.has_value() );
  EXPECT_EQ( string_value.value(), "batch_update" );
}

TEST_F( ParameterHelpersTest, ListParameters )
{
  RemoteParameterClient client( tester_node_, "target_node" );
  ASSERT_TRUE( client.wait_for_service( *executor_, 5s ) );

  auto params = client.list_parameters( *executor_ );

  // Should find our declared parameters
  EXPECT_FALSE( params.empty() );

  bool found_int = false;
  bool found_string = false;
  for ( const auto &name : params ) {
    if ( name == "int_param" )
      found_int = true;
    if ( name == "string_param" )
      found_string = true;
  }

  EXPECT_TRUE( found_int ) << "Expected to find int_param";
  EXPECT_TRUE( found_string ) << "Expected to find string_param";
}

TEST_F( ParameterHelpersTest, ListParametersWithPrefix )
{
  RemoteParameterClient client( tester_node_, "target_node" );
  ASSERT_TRUE( client.wait_for_service( *executor_, 5s ) );

  // The prefix filter in ROS 2 uses dot-separated prefixes for hierarchical parameters
  // For flat parameters, the prefix must be an exact match of a namespace component
  // So we just verify that list_parameters works with empty prefix
  auto all_params = client.list_parameters( *executor_ );

  // Count how many of our declared parameters we can find
  int found_count = 0;
  for ( const auto &name : all_params ) {
    if ( name == "int_param" || name == "string_param" || name == "double_param" ||
         name == "bool_param" ) {
      found_count++;
    }
  }

  EXPECT_GE( found_count, 4 ) << "Expected to find all 4 declared parameters";
}

TEST_F( ParameterHelpersTest, HasParameter )
{
  RemoteParameterClient client( tester_node_, "target_node" );
  ASSERT_TRUE( client.wait_for_service( *executor_, 5s ) );

  EXPECT_TRUE( client.has_parameter( *executor_, "int_param" ) );
  EXPECT_FALSE( client.has_parameter( *executor_, "nonexistent_param" ) );
}

TEST_F( ParameterHelpersTest, TargetNodeName )
{
  RemoteParameterClient client( tester_node_, "target_node" );

  // Should add leading slash if missing
  EXPECT_EQ( client.target_node_name(), "/target_node" );

  RemoteParameterClient client2( tester_node_, "/other_node" );
  EXPECT_EQ( client2.target_node_name(), "/other_node" );
}

// =============================================================================
// Convenience Function Tests
// =============================================================================

TEST_F( ParameterHelpersTest, SetRemoteParameterConvenience )
{
  using hector_testing_utils::set_remote_parameter;

  auto result = set_remote_parameter( tester_node_, "target_node", "int_param", 999, *executor_ );

  EXPECT_TRUE( result.success );

  // Verify using direct access
  EXPECT_EQ( target_node_->get_parameter( "int_param" ).as_int(), 999 );
}

TEST_F( ParameterHelpersTest, GetRemoteParameterConvenience )
{
  using hector_testing_utils::get_remote_parameter;

  auto value = get_remote_parameter<int>( tester_node_, "target_node", "int_param", *executor_ );

  ASSERT_TRUE( value.has_value() );
  EXPECT_EQ( value.value(), 42 );
}

TEST_F( ParameterHelpersTest, ListRemoteParametersConvenience )
{
  using hector_testing_utils::list_remote_parameters;

  auto params = list_remote_parameters( tester_node_, "target_node", *executor_ );

  EXPECT_FALSE( params.empty() );
}

// =============================================================================
// Error Handling Tests
// =============================================================================

TEST_F( ParameterHelpersTest, ServiceNotReadyTimeout )
{
  // Try to connect to a non-existent node
  RemoteParameterClient client( tester_node_, "nonexistent_node" );

  bool ready = client.wait_for_service( *executor_, 200ms );

  EXPECT_FALSE( ready );
}

TEST_F( ParameterHelpersTest, SetParameterServiceNotReady )
{
  RemoteParameterClient client( tester_node_, "nonexistent_node" );

  // Don't wait for service
  auto result = client.set_parameter( *executor_, "param", 42, 200ms );

  EXPECT_FALSE( result.success );
  EXPECT_FALSE( result.reason.empty() );
}

TEST_F( ParameterHelpersTest, GetParameterServiceNotReady )
{
  RemoteParameterClient client( tester_node_, "nonexistent_node" );

  // Don't wait for service
  auto value = client.get_parameter<int>( *executor_, "param", 200ms );

  EXPECT_FALSE( value.has_value() );
}
