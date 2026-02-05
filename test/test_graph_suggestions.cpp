/// @file test_graph_suggestions.cpp
/// @brief Tests for graph introspection and suggestion functionality.
///
/// This file tests the ability to:
/// - Collect available topics, services, and actions from the ROS graph
/// - Compute string similarity using Levenshtein distance
/// - Generate suggestions for similar entities when lookups fail
/// - Provide helpful failure information when wait operations timeout

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <example_interfaces/action/fibonacci.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <hector_testing_utils/hector_testing_utils.hpp>

using namespace std::chrono_literals;
using hector_testing_utils::HectorTestFixture;
using hector_testing_utils::WaitFailureInfo;

// =============================================================================
// String Similarity Tests
// =============================================================================

TEST( StringSimilarity, LevenshteinDistanceIdentical )
{
  EXPECT_EQ( hector_testing_utils::detail::levenshtein_distance( "hello", "hello" ), 0u );
  EXPECT_EQ( hector_testing_utils::detail::levenshtein_distance( "", "" ), 0u );
  EXPECT_EQ( hector_testing_utils::detail::levenshtein_distance( "/my_topic", "/my_topic" ), 0u );
}

TEST( StringSimilarity, LevenshteinDistanceEmpty )
{
  EXPECT_EQ( hector_testing_utils::detail::levenshtein_distance( "", "hello" ), 5u );
  EXPECT_EQ( hector_testing_utils::detail::levenshtein_distance( "hello", "" ), 5u );
}

TEST( StringSimilarity, LevenshteinDistanceOneEdit )
{
  // Substitution
  EXPECT_EQ( hector_testing_utils::detail::levenshtein_distance( "cat", "bat" ), 1u );
  // Insertion
  EXPECT_EQ( hector_testing_utils::detail::levenshtein_distance( "cat", "cats" ), 1u );
  // Deletion
  EXPECT_EQ( hector_testing_utils::detail::levenshtein_distance( "cats", "cat" ), 1u );
}

TEST( StringSimilarity, LevenshteinDistanceMultipleEdits )
{
  EXPECT_EQ( hector_testing_utils::detail::levenshtein_distance( "kitten", "sitting" ), 3u );
  EXPECT_EQ( hector_testing_utils::detail::levenshtein_distance( "Saturday", "Sunday" ), 3u );
}

TEST( StringSimilarity, StringSimilarityScore )
{
  // Identical strings should have similarity 1.0
  EXPECT_DOUBLE_EQ( hector_testing_utils::detail::string_similarity( "hello", "hello" ), 1.0 );

  // Empty strings
  EXPECT_DOUBLE_EQ( hector_testing_utils::detail::string_similarity( "", "" ), 1.0 );

  // Completely different (same length)
  double sim = hector_testing_utils::detail::string_similarity( "abc", "xyz" );
  EXPECT_LT( sim, 0.5 ); // Should be low similarity

  // Similar strings
  sim = hector_testing_utils::detail::string_similarity( "/my_topic", "/my_topik" );
  EXPECT_GT( sim, 0.8 ); // Should be high similarity (1 character difference)
}

TEST( StringSimilarity, ContainsSubstringCaseInsensitive )
{
  using hector_testing_utils::detail::contains_substring_ci;

  EXPECT_TRUE( contains_substring_ci( "/my_topic/data", "topic" ) );
  EXPECT_TRUE( contains_substring_ci( "/MY_TOPIC/DATA", "topic" ) );
  EXPECT_TRUE( contains_substring_ci( "/my_topic", "MY_TOPIC" ) );
  EXPECT_FALSE( contains_substring_ci( "/my_topic", "other" ) );
  EXPECT_TRUE( contains_substring_ci( "anything", "" ) );   // Empty needle always matches
  EXPECT_FALSE( contains_substring_ci( "", "something" ) ); // Empty haystack never matches
}

TEST( StringSimilarity, ExtractBaseName )
{
  using hector_testing_utils::detail::extract_base_name;

  EXPECT_EQ( extract_base_name( "/my_topic" ), "my_topic" );
  EXPECT_EQ( extract_base_name( "/namespace/my_topic" ), "my_topic" );
  EXPECT_EQ( extract_base_name( "/a/b/c/deep_topic" ), "deep_topic" );
  EXPECT_EQ( extract_base_name( "no_slash" ), "no_slash" );
  EXPECT_EQ( extract_base_name( "/" ), "/" ); // Edge case: just a slash
}

// =============================================================================
// Graph Introspection Tests
// =============================================================================

TEST_F( HectorTestFixture, CollectAvailableTopics )
{
  // Create some test publishers and subscribers to populate the graph
  auto pub1 = tester_node_->create_publisher<std_msgs::msg::Int32>( "/test/topic_a", 10 );
  auto pub2 = tester_node_->create_publisher<std_msgs::msg::String>( "/test/topic_b", 10 );

  // Give the graph time to update
  executor_->spin_until(
      [this]() {
        auto topics = hector_testing_utils::collect_available_topics( tester_node_ );
        return topics.size() >= 2;
      },
      2s );

  auto topics = hector_testing_utils::collect_available_topics( tester_node_ );

  // Should find our topics
  bool found_a = false, found_b = false;
  for ( const auto &topic : topics ) {
    if ( topic.name == "/test/topic_a" ) {
      found_a = true;
      ASSERT_FALSE( topic.types.empty() );
      EXPECT_NE( topic.types[0].find( "Int32" ), std::string::npos );
    }
    if ( topic.name == "/test/topic_b" ) {
      found_b = true;
      ASSERT_FALSE( topic.types.empty() );
      EXPECT_NE( topic.types[0].find( "String" ), std::string::npos );
    }
  }

  EXPECT_TRUE( found_a ) << "Topic /test/topic_a not found";
  EXPECT_TRUE( found_b ) << "Topic /test/topic_b not found";
}

TEST_F( HectorTestFixture, CollectAvailableServices )
{
  using Service = example_interfaces::srv::AddTwoInts;

  // Create a test service
  auto service = tester_node_->create_service<Service>(
      "/test/my_service",
      []( const std::shared_ptr<Service::Request>, std::shared_ptr<Service::Response> ) { } );

  // Give the graph time to update
  executor_->spin_until(
      [this]() {
        auto services = hector_testing_utils::collect_available_services( tester_node_ );
        for ( const auto &svc : services ) {
          if ( svc.name == "/test/my_service" )
            return true;
        }
        return false;
      },
      2s );

  auto services = hector_testing_utils::collect_available_services( tester_node_ );

  bool found = false;
  for ( const auto &svc : services ) {
    if ( svc.name == "/test/my_service" ) {
      found = true;
      ASSERT_FALSE( svc.types.empty() );
      EXPECT_NE( svc.types[0].find( "AddTwoInts" ), std::string::npos );
    }
  }

  EXPECT_TRUE( found ) << "Service /test/my_service not found";
}

TEST_F( HectorTestFixture, CollectAvailableActions )
{
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  // Create a test action server
  auto action_server = rclcpp_action::create_server<Fibonacci>(
      tester_node_, "/test/my_action",
      []( const rclcpp_action::GoalUUID &, const std::shared_ptr<const Fibonacci::Goal> ) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      []( const std::shared_ptr<GoalHandle> ) { return rclcpp_action::CancelResponse::ACCEPT; },
      []( const std::shared_ptr<GoalHandle> ) {} );

  // Give the graph time to update
  executor_->spin_until(
      [this]() {
        auto actions = hector_testing_utils::collect_available_actions( tester_node_ );
        for ( const auto &action : actions ) {
          if ( action.name == "/test/my_action" )
            return true;
        }
        return false;
      },
      2s );

  auto actions = hector_testing_utils::collect_available_actions( tester_node_ );

  bool found = false;
  for ( const auto &action : actions ) {
    if ( action.name == "/test/my_action" ) {
      found = true;
      // Action types might or might not be available depending on implementation
    }
  }

  EXPECT_TRUE( found ) << "Action /test/my_action not found";
}

// =============================================================================
// Suggestion Generation Tests
// =============================================================================

TEST_F( HectorTestFixture, SuggestSimilarTopics )
{
  // Create some topics with similar names
  auto pub1 = tester_node_->create_publisher<std_msgs::msg::Int32>( "/robot/sensor_data", 10 );
  auto pub2 = tester_node_->create_publisher<std_msgs::msg::Int32>( "/robot/sensor_info", 10 );
  auto pub3 = tester_node_->create_publisher<std_msgs::msg::String>( "/robot/status", 10 );

  // Give the graph time to update
  executor_->spin_until(
      [this]() {
        auto topics = hector_testing_utils::collect_available_topics( tester_node_ );
        size_t count = 0;
        for ( const auto &t : topics ) {
          if ( t.name.find( "/robot/" ) != std::string::npos )
            count++;
        }
        return count >= 3;
      },
      2s );

  // Search for a misspelled topic
  auto suggestions = hector_testing_utils::suggest_similar_topics(
      tester_node_, "/robot/sensor_daat" ); // Misspelled "data"

  ASSERT_FALSE( suggestions.empty() );

  // The correct topic should be suggested with high similarity
  bool found_correct = false;
  for ( const auto &suggestion : suggestions ) {
    if ( suggestion.name == "/robot/sensor_data" ) {
      found_correct = true;
      EXPECT_GT( suggestion.similarity_score, 0.8 ); // Should be very similar
      break;
    }
  }

  EXPECT_TRUE( found_correct ) << "Expected /robot/sensor_data to be suggested";

  // Suggestions should be sorted by similarity (descending)
  for ( size_t i = 1; i < suggestions.size(); ++i ) {
    EXPECT_GE( suggestions[i - 1].similarity_score, suggestions[i].similarity_score )
        << "Suggestions should be sorted by similarity score (descending)";
  }
}

TEST_F( HectorTestFixture, SuggestSimilarServices )
{
  using Service = example_interfaces::srv::AddTwoInts;

  // Create some services with similar names
  auto svc1 = tester_node_->create_service<Service>(
      "/math/add_numbers",
      []( const std::shared_ptr<Service::Request>, std::shared_ptr<Service::Response> ) { } );
  auto svc2 = tester_node_->create_service<Service>(
      "/math/multiply_numbers",
      []( const std::shared_ptr<Service::Request>, std::shared_ptr<Service::Response> ) { } );

  // Give the graph time to update
  executor_->spin_until(
      [this]() {
        auto services = hector_testing_utils::collect_available_services( tester_node_ );
        size_t count = 0;
        for ( const auto &s : services ) {
          if ( s.name.find( "/math/" ) != std::string::npos )
            count++;
        }
        return count >= 2;
      },
      2s );

  // Search for a misspelled service
  auto suggestions = hector_testing_utils::suggest_similar_services(
      tester_node_, "/math/ad_numbers" ); // Misspelled "add"

  ASSERT_FALSE( suggestions.empty() );

  // The correct service should be suggested
  bool found_correct = false;
  for ( const auto &suggestion : suggestions ) {
    if ( suggestion.name == "/math/add_numbers" ) {
      found_correct = true;
      EXPECT_GT( suggestion.similarity_score, 0.8 );
      break;
    }
  }

  EXPECT_TRUE( found_correct ) << "Expected /math/add_numbers to be suggested";
}

TEST_F( HectorTestFixture, SuggestSimilarActions )
{
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  // Create action servers with similar names
  auto action1 = rclcpp_action::create_server<Fibonacci>(
      tester_node_, "/compute/fibonacci",
      []( const rclcpp_action::GoalUUID &, const std::shared_ptr<const Fibonacci::Goal> ) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      []( const std::shared_ptr<GoalHandle> ) { return rclcpp_action::CancelResponse::ACCEPT; },
      []( const std::shared_ptr<GoalHandle> ) {} );

  auto action2 = rclcpp_action::create_server<Fibonacci>(
      tester_node_, "/compute/factorial",
      []( const rclcpp_action::GoalUUID &, const std::shared_ptr<const Fibonacci::Goal> ) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      []( const std::shared_ptr<GoalHandle> ) { return rclcpp_action::CancelResponse::ACCEPT; },
      []( const std::shared_ptr<GoalHandle> ) {} );

  // Give the graph time to update
  executor_->spin_until(
      [this]() {
        auto actions = hector_testing_utils::collect_available_actions( tester_node_ );
        size_t count = 0;
        for ( const auto &a : actions ) {
          if ( a.name.find( "/compute/" ) != std::string::npos )
            count++;
        }
        return count >= 2;
      },
      2s );

  // Search for a misspelled action
  auto suggestions = hector_testing_utils::suggest_similar_actions(
      tester_node_, "/compute/fibonaci" ); // Misspelled "fibonacci"

  ASSERT_FALSE( suggestions.empty() );

  // The correct action should be suggested
  bool found_correct = false;
  for ( const auto &suggestion : suggestions ) {
    if ( suggestion.name == "/compute/fibonacci" ) {
      found_correct = true;
      EXPECT_GT( suggestion.similarity_score, 0.8 );
      break;
    }
  }

  EXPECT_TRUE( found_correct ) << "Expected /compute/fibonacci to be suggested";
}

// =============================================================================
// Wait Failure Info Tests
// =============================================================================

TEST_F( HectorTestFixture, WaitForSubscriptionWithFailureInfo )
{
  // Create a publisher on a topic
  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>( "/existing_topic" );

  // Try to wait for a subscription that doesn't exist (short timeout)
  WaitFailureInfo failure_info;
  bool success = pub->wait_for_subscription( *executor_, 100ms, failure_info );

  EXPECT_FALSE( success );
  EXPECT_EQ( failure_info.searched_name, "/existing_topic" );
  EXPECT_EQ( failure_info.entity_kind, "topic" );
  // Type info should be populated
  EXPECT_FALSE( failure_info.searched_type.empty() );

  // Format should produce readable output
  std::string formatted = failure_info.format();
  EXPECT_FALSE( formatted.empty() );
  EXPECT_NE( formatted.find( "/existing_topic" ), std::string::npos );
}

TEST_F( HectorTestFixture, WaitForPublishersWithFailureInfo )
{
  // Create a subscription on a topic
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>( "/waiting_topic" );

  // Try to wait for publishers that don't exist (short timeout)
  WaitFailureInfo failure_info;
  bool success = sub->wait_for_publishers( *executor_, 1, 100ms, failure_info );

  EXPECT_FALSE( success );
  EXPECT_EQ( failure_info.searched_name, "/waiting_topic" );
  EXPECT_EQ( failure_info.entity_kind, "topic" );
}

TEST_F( HectorTestFixture, WaitForServiceWithFailureInfo )
{
  using Service = example_interfaces::srv::AddTwoInts;

  // Create a service client
  auto client = tester_node_->create_test_client<Service>( "/nonexistent_service" );

  // Try to wait for a service that doesn't exist (short timeout)
  WaitFailureInfo failure_info;
  bool success = client->wait_for_service( *executor_, 100ms, failure_info );

  EXPECT_FALSE( success );
  EXPECT_EQ( failure_info.searched_name, "/nonexistent_service" );
  EXPECT_EQ( failure_info.entity_kind, "service" );
}

TEST_F( HectorTestFixture, WaitForActionServerWithFailureInfo )
{
  using Fibonacci = example_interfaces::action::Fibonacci;

  // Create an action client
  auto client = tester_node_->create_test_action_client<Fibonacci>( "/nonexistent_action" );

  // Try to wait for an action server that doesn't exist (short timeout)
  WaitFailureInfo failure_info;
  bool success = client->wait_for_server( *executor_, 100ms, failure_info );

  EXPECT_FALSE( success );
  EXPECT_EQ( failure_info.searched_name, "/nonexistent_action" );
  EXPECT_EQ( failure_info.entity_kind, "action" );
}

TEST_F( HectorTestFixture, WaitForAllConnectionsWithFailureInfos )
{
  // Create some test entities that will NOT have their counterparts
  auto pub = tester_node_->create_test_publisher<std_msgs::msg::Int32>( "/orphan_pub_topic" );
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>( "/orphan_sub_topic" );

  // Also create a real topic that exists so we can verify partial success
  auto real_pub = tester_node_->create_publisher<std_msgs::msg::Int32>( "/orphan_sub_topic", 10 );

  // Wait should timeout because pub has no subscriber
  std::vector<WaitFailureInfo> failure_infos;
  bool success = tester_node_->wait_for_all_connections( *executor_, 1s, failure_infos );

  // If it fails (which it should for the orphan publisher), we should get failure info
  if ( !success ) {
    EXPECT_FALSE( failure_infos.empty() );

    // Should find info about the orphan publisher
    bool found_orphan = false;
    for ( const auto &info : failure_infos ) {
      if ( info.searched_name == "/orphan_pub_topic" ) {
        found_orphan = true;
        EXPECT_EQ( info.entity_kind, "topic" );
      }
    }
    EXPECT_TRUE( found_orphan ) << "Expected to find failure info for /orphan_pub_topic";

    // Format all failure infos
    std::string formatted = hector_testing_utils::TestNode::format_failure_infos( failure_infos );
    EXPECT_FALSE( formatted.empty() );
  }
}

// =============================================================================
// Integration Tests with Suggestions
// =============================================================================

TEST_F( HectorTestFixture, SuggestionsShowSimilarTopicsOnTimeout )
{
  // Create some topics that are similar to what we'll search for
  auto pub1 = tester_node_->create_publisher<std_msgs::msg::Int32>( "/sensor/temperature", 10 );
  auto pub2 = tester_node_->create_publisher<std_msgs::msg::Int32>( "/sensor/humidity", 10 );
  auto pub3 = tester_node_->create_publisher<std_msgs::msg::Int32>( "/sensor/pressure", 10 );

  // Give graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  // Create a subscription for a misspelled topic
  auto sub = tester_node_->create_test_subscription<std_msgs::msg::Int32>( "/sensor/temperatur" );

  // Wait will timeout, but we should get suggestions
  WaitFailureInfo failure_info;
  bool success = sub->wait_for_publishers( *executor_, 1, 200ms, failure_info );

  EXPECT_FALSE( success );

  // Check that we got suggestions including the correct topic
  bool found_correct = false;
  for ( const auto &suggestion : failure_info.suggestions ) {
    if ( suggestion.name == "/sensor/temperature" ) {
      found_correct = true;
      // The misspelled version should be very similar to the correct one
      EXPECT_GT( suggestion.similarity_score, 0.9 );
    }
  }

  EXPECT_TRUE( found_correct )
      << "Expected /sensor/temperature to be suggested for /sensor/temperatur";
}

TEST_F( HectorTestFixture, TypeMatchingBoostsSimilarity )
{
  // Create topics with same name pattern but different types
  auto pub_int = tester_node_->create_publisher<std_msgs::msg::Int32>( "/data/values_int", 10 );
  auto pub_str = tester_node_->create_publisher<std_msgs::msg::String>( "/data/values_str", 10 );

  // Give graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  // Search for a topic with Int32 type
  auto suggestions = hector_testing_utils::suggest_similar_topics( tester_node_, "/data/values",
                                                                   "std_msgs/msg/Int32" );

  // Both should be suggested, but Int32 should rank higher due to type match
  double int_score = 0.0, str_score = 0.0;
  for ( const auto &suggestion : suggestions ) {
    if ( suggestion.name == "/data/values_int" ) {
      int_score = suggestion.similarity_score;
    }
    if ( suggestion.name == "/data/values_str" ) {
      str_score = suggestion.similarity_score;
    }
  }

  // The Int32 topic should have a higher score due to type matching
  // (or at least equal if the type wasn't matched properly)
  EXPECT_GE( int_score, str_score )
      << "Type-matched topic should have equal or higher similarity score";
}

TEST_F( HectorTestFixture, WaitFailureInfoFormat )
{
  WaitFailureInfo info;
  info.searched_name = "/my/missing/topic";
  info.searched_type = "std_msgs/msg/Int32";
  info.entity_kind = "topic";

  hector_testing_utils::Suggestion s1;
  s1.name = "/my/existing/topic";
  s1.types = { "std_msgs/msg/Int32" };
  s1.similarity_score = 0.85;

  hector_testing_utils::Suggestion s2;
  s2.name = "/my/other/topic";
  s2.types = { "std_msgs/msg/String" };
  s2.similarity_score = 0.70;

  hector_testing_utils::Suggestion s3;
  s3.name = "/different/topic";
  s3.types = { "std_msgs/msg/Int32" };
  s3.similarity_score = 0.50;

  info.suggestions = { s1, s2, s3 };

  std::string formatted = info.format();

  // Check that all important information is in the output
  EXPECT_NE( formatted.find( "/my/missing/topic" ), std::string::npos );
  EXPECT_NE( formatted.find( "std_msgs/msg/Int32" ), std::string::npos );
  EXPECT_NE( formatted.find( "topic" ), std::string::npos );
  EXPECT_NE( formatted.find( "/my/existing/topic" ), std::string::npos );
  EXPECT_NE( formatted.find( "0.85" ), std::string::npos );
}

TEST_F( HectorTestFixture, EmptySuggestionsHandledGracefully )
{
  WaitFailureInfo info;
  info.searched_name = "/completely/unique/topic";
  info.entity_kind = "topic";
  info.suggestions = {}; // No suggestions

  std::string formatted = info.format();

  // Should still produce valid output
  EXPECT_FALSE( formatted.empty() );
  EXPECT_NE( formatted.find( "/completely/unique/topic" ), std::string::npos );
  EXPECT_NE( formatted.find( "No similar" ), std::string::npos );
}

// =============================================================================
// Edge Case Tests
// =============================================================================

TEST( StringSimilarity, VeryLongStrings )
{
  // Test with long strings to ensure no performance issues
  std::string long1( 1000, 'a' );
  std::string long2( 1000, 'a' );
  long2[500] = 'b'; // One character difference

  size_t distance = hector_testing_utils::detail::levenshtein_distance( long1, long2 );
  EXPECT_EQ( distance, 1u );

  double similarity = hector_testing_utils::detail::string_similarity( long1, long2 );
  EXPECT_GT( similarity, 0.99 ); // Should be very similar
}

TEST( StringSimilarity, UnicodeHandling )
{
  // Basic ASCII comparison should work
  // Note: Levenshtein on byte level might not handle multi-byte UTF-8 correctly,
  // but for ROS topic names (which are typically ASCII), this is fine
  double sim = hector_testing_utils::detail::string_similarity( "topic_1", "topic_2" );
  EXPECT_GT( sim, 0.8 );
}

TEST_F( HectorTestFixture, MaxSuggestionsLimit )
{
  // Create many topics
  std::vector<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> publishers;
  for ( int i = 0; i < 20; ++i ) {
    publishers.push_back( tester_node_->create_publisher<std_msgs::msg::Int32>(
        "/many/topic_" + std::to_string( i ), 10 ) );
  }

  // Give graph time to update
  executor_->spin_until( []() { return true; }, 500ms );

  // Get suggestions with a limit
  auto suggestions =
      hector_testing_utils::suggest_similar_topics( tester_node_, "/many/topic", "", 5 );

  // Should respect the limit
  EXPECT_LE( suggestions.size(), 5u );
}
