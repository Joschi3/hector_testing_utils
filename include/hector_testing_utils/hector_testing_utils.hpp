#ifndef HECTOR_TESTING_UTILS_HECTOR_TESTING_UTILS_HPP
#define HECTOR_TESTING_UTILS_HECTOR_TESTING_UTILS_HPP

#include <chrono>
#include <cstdarg>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rcutils/logging.h>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace hector_testing_utils
{

using namespace std::chrono_literals;

constexpr std::chrono::milliseconds kDefaultSpinPeriod{ 5 };
constexpr std::chrono::seconds kDefaultTimeout{ 5 };

// =============================================================================
// Context Helper
// =============================================================================

class TestContext
{
public:
  explicit TestContext( int argc = 0, char **argv = nullptr )
  {
    context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    init_options.auto_initialize_logging( false );
    context_->init( argc, argv, init_options );
  }

  ~TestContext()
  {
    if ( context_ && context_->is_valid() ) {
      rclcpp::shutdown( context_ );
    }
  }

  rclcpp::Context::SharedPtr context() const { return context_; }

  rclcpp::NodeOptions node_options() const
  {
    rclcpp::NodeOptions options;
    options.context( context_ );
    return options;
  }

private:
  rclcpp::Context::SharedPtr context_;
};

// =============================================================================
// Executor Helper
// =============================================================================

class TestExecutor
{
public:
  TestExecutor() : context_( std::make_shared<rclcpp::Context>() )
  {
    rclcpp::InitOptions init_options;
    init_options.auto_initialize_logging( false );
    context_->init( 0, nullptr, init_options );
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(
        make_executor_options( context_ ) );
  }

  explicit TestExecutor( const rclcpp::Context::SharedPtr &context ) : context_( context )
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(
        make_executor_options( context_ ) );
  }

  explicit TestExecutor( const std::shared_ptr<rclcpp::Executor> &executor )
      : context_( rclcpp::contexts::get_global_default_context() ), executor_( executor )
  {
  }

  ~TestExecutor() { stop_background_spinner(); }

  void add_node( const rclcpp::Node::SharedPtr &node ) { executor_->add_node( node ); }

  // Spin until predicate is true
  bool spin_until( const std::function<bool()> &predicate,
                   std::chrono::nanoseconds timeout = kDefaultTimeout,
                   std::chrono::nanoseconds spin_period = kDefaultSpinPeriod )
  {
    const auto start = std::chrono::steady_clock::now();
    while ( rclcpp::ok( context_ ) ) {
      if ( predicate() )
        return true;
      if ( std::chrono::steady_clock::now() - start > timeout )
        return false;

      // If background spinner is active, we just wait (passive mode).
      // Otherwise, we drive the executor (active mode).
      if ( !background_spinner_thread_.joinable() ) {
        executor_->spin_some();
      }
      std::this_thread::sleep_for( spin_period );
    }
    return false;
  }

  template<typename FutureT>
  bool spin_until_future_complete( FutureT &future,
                                   std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    // If background spinning is active, we can't use spin_until_future_complete because
    // it tries to spin the executor which is already spinning.
    // So we fall back to our predicate-based wait.
    if ( background_spinner_thread_.joinable() ) {
      return spin_until(
          [&future]() {
            return future.wait_for( std::chrono::seconds( 0 ) ) == std::future_status::ready;
          },
          timeout );
    }
    auto result = executor_->spin_until_future_complete( future, timeout );
    return result == rclcpp::FutureReturnCode::SUCCESS;
  }

  void spin_some()
  {
    if ( background_spinner_thread_.joinable() ) {
      throw std::runtime_error( "Cannot call spin_some() while background spinner is active!" );
    }
    executor_->spin_some();
  }

  void start_background_spinner()
  {
    if ( background_spinner_thread_.joinable() ) {
      return; // Already spinning
    }
    stop_signal_ = false;
    background_spinner_thread_ = std::thread( [this]() {
      while ( rclcpp::ok( context_ ) && !stop_signal_ ) {
        executor_->spin_some();
        // Small sleep to yield if spin_some returns immediately (no work)
        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
      }
    } );
  }

  void stop_background_spinner()
  {
    if ( background_spinner_thread_.joinable() ) {
      stop_signal_ = true;
      executor_->cancel();
      background_spinner_thread_.join();
    }
  }

  // RAII Helper for background spinning
  struct ScopedSpinner {
    explicit ScopedSpinner( TestExecutor &exec ) : exec_( exec )
    {
      exec_.start_background_spinner();
    }
    ~ScopedSpinner() { exec_.stop_background_spinner(); }
    TestExecutor &exec_;
  };

private:
  static rclcpp::ExecutorOptions make_executor_options( const rclcpp::Context::SharedPtr &context )
  {
    rclcpp::ExecutorOptions options;
    options.context = context;
    return options;
  }

  rclcpp::Context::SharedPtr context_;
  std::shared_ptr<rclcpp::Executor> executor_;
  std::thread background_spinner_thread_;
  std::atomic<bool> stop_signal_{ false };
};

// =============================================================================
// Interfaces & Wrappers
// =============================================================================

// Abstract base for anything that needs to "connect"
class Connectable
{
public:
  virtual ~Connectable() = default;
  virtual bool is_connected() const = 0;
  virtual std::string get_name() const = 0;
  virtual std::string get_type() const = 0; // e.g., "Publisher", "Client"
};

/// Wrapper for Publisher
template<typename MsgT>
class TestPublisher : public Connectable
{
public:
  TestPublisher( rclcpp::Node::SharedPtr node, const std::string &topic,
                 const rclcpp::QoS &qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ) )
      : topic_( topic )
  {
    pub_ = node->create_publisher<MsgT>( topic, qos );
  }

  void publish( const MsgT &msg ) { pub_->publish( msg ); }

  bool is_connected() const override { return pub_->get_subscription_count() > 0; }
  std::string get_name() const override { return topic_; }
  std::string get_type() const override { return "Publisher"; }

  // Specific wait helper
  bool wait_for_subscription( TestExecutor &exec, std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return exec.spin_until( [this]() { return is_connected(); }, timeout );
  }

private:
  typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
  std::string topic_;
};

/// Wrapper for Subscription
template<class MsgT>
class TestSubscription : public Connectable
{
public:
  TestSubscription( const rclcpp::Node::SharedPtr &node, const std::string &topic,
                    const rclcpp::QoS &qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ),
                    bool latched = false )
      : topic_( topic )
  {
    rclcpp::QoS resolved_qos = qos;
    if ( latched )
      resolved_qos.durability( RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL );

    sub_ = node->create_subscription<MsgT>( topic, resolved_qos,
                                            [this]( const std::shared_ptr<MsgT> msg ) {
                                              std::lock_guard<std::mutex> lock( mutex_ );
                                              last_message_ = *msg;
                                              ++message_count_;
                                            } );
  }

  bool is_connected() const override { return sub_->get_publisher_count() > 0; }
  std::string get_name() const override { return topic_; }
  std::string get_type() const override { return "Subscription"; }

  void reset()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    last_message_.reset();
    message_count_ = 0;
  }

  bool has_new_message( size_t start_count ) const
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return message_count_ > start_count;
  }

  std::optional<MsgT> last_message() const
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return last_message_;
  }

  size_t message_count() const
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return message_count_;
  }

  bool wait_for_publishers( TestExecutor &executor, size_t count = 1,
                            std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return executor.spin_until( [this, count]() { return sub_->get_publisher_count() >= count; },
                                timeout );
  }

  // Test Helpers
  bool wait_for_message( TestExecutor &executor, std::chrono::nanoseconds timeout = kDefaultTimeout,
                         const std::function<bool( const MsgT & )> &predicate = nullptr )
  {
    size_t start_count = 0;
    {
      std::lock_guard<std::mutex> lock( mutex_ );
      start_count = message_count_;
    }

    return executor.spin_until(
        [this, start_count, &predicate]() {
          std::optional<MsgT> msg;
          size_t count = 0;
          {
            std::lock_guard<std::mutex> lock( mutex_ );
            count = message_count_;
            if ( count <= start_count || !last_message_ ) {
              return false;
            }
            msg = last_message_;
          }
          if ( predicate ) {
            return predicate( *msg );
          }
          return true;
        },
        timeout );
  }

  bool wait_for_new_message( TestExecutor &executor,
                             std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return wait_for_message( executor, timeout );
  }

private:
  std::shared_ptr<rclcpp::Subscription<MsgT>> sub_;
  std::string topic_;
  mutable std::mutex mutex_;
  std::optional<MsgT> last_message_;
  size_t message_count_{ 0 };
};

/// Wrapper for Service Client
template<typename ServiceT>
class TestClient : public Connectable
{
public:
  TestClient( rclcpp::Node::SharedPtr node, const std::string &service_name )
      : service_name_( service_name )
  {
    client_ = node->create_client<ServiceT>( service_name );
  }

  bool is_connected() const override { return client_->service_is_ready(); }
  std::string get_name() const override { return service_name_; }
  std::string get_type() const override { return "Service Client"; }

  typename rclcpp::Client<ServiceT>::SharedPtr get() { return client_; }

  bool wait_for_service( TestExecutor &exec, std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    // client->wait_for_service is blocking, so we use spin_until with non-blocking check
    return exec.spin_until( [this]() { return client_->service_is_ready(); }, timeout );
  }

private:
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  std::string service_name_;
};

/// Wrapper for Action Client
template<typename ActionT>
class TestActionClient : public Connectable
{
public:
  TestActionClient( rclcpp::Node::SharedPtr node, const std::string &action_name )
      : action_name_( action_name )
  {
    client_ = rclcpp_action::create_client<ActionT>( node, action_name );
  }

  bool is_connected() const override { return client_->action_server_is_ready(); }
  std::string get_name() const override { return action_name_; }
  std::string get_type() const override { return "Action Client"; }

  typename rclcpp_action::Client<ActionT>::SharedPtr get() { return client_; }

  bool wait_for_server( TestExecutor &exec, std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return exec.spin_until( [this]() { return client_->action_server_is_ready(); }, timeout );
  }

private:
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;
  std::string action_name_;
};

/// Wrapper for Service Server
template<typename ServiceT>
class TestServiceServer : public Connectable
{
public:
  using Callback = std::function<void( const std::shared_ptr<typename ServiceT::Request>,
                                       std::shared_ptr<typename ServiceT::Response> )>;

  TestServiceServer( const rclcpp::Node::SharedPtr &node, const std::string &service_name,
                     const Callback &callback, const rclcpp::QoS &qos = rclcpp::ServicesQoS() )
      : node_( node ), service_name_( service_name )
  {
    service_ = node_->create_service<ServiceT>( service_name_, callback, qos );
  }

  bool is_connected() const override { return node_->count_clients( service_name_ ) > 0; }
  std::string get_name() const override { return service_name_; }
  std::string get_type() const override { return "Service Server"; }

  bool wait_for_client( TestExecutor &exec, std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return exec.spin_until( [this]() { return is_connected(); }, timeout );
  }

  typename rclcpp::Service<ServiceT>::SharedPtr get() { return service_; }

private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Service<ServiceT>::SharedPtr service_;
  std::string service_name_;
};

/// Wrapper for Action Server
template<typename ActionT>
class TestActionServer : public Connectable
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using GoalCallback = std::function<rclcpp_action::GoalResponse(
      const rclcpp_action::GoalUUID &, const std::shared_ptr<const typename ActionT::Goal> )>;
  using CancelCallback =
      std::function<rclcpp_action::CancelResponse( const std::shared_ptr<GoalHandle> )>;
  using AcceptedCallback = std::function<void( const std::shared_ptr<GoalHandle> )>;

  TestActionServer( const rclcpp::Node::SharedPtr &node, const std::string &action_name,
                    GoalCallback goal_cb, CancelCallback cancel_cb, AcceptedCallback accepted_cb )
      : node_( node ), action_name_( action_name )
  {
    server_ =
        rclcpp_action::create_server<ActionT>( node_, action_name_, std::move( goal_cb ),
                                               std::move( cancel_cb ), std::move( accepted_cb ) );
  }

  bool is_connected() const override
  {
    // Clients subscribe to the status topic; use that to infer connectivity.
    return node_->count_subscribers( status_topic() ) > 0;
  }

  std::string get_name() const override { return action_name_; }
  std::string get_type() const override { return "Action Server"; }

  bool wait_for_client( TestExecutor &exec, std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    return exec.spin_until( [this]() { return is_connected(); }, timeout );
  }

  typename rclcpp_action::Server<ActionT>::SharedPtr get() { return server_; }

private:
  std::string status_topic() const { return action_name_ + "/_action/status"; }

  rclcpp::Node::SharedPtr node_;
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  std::string action_name_;
};

// =============================================================================
// The Main Testing Node
// =============================================================================

class TestNode : public rclcpp::Node
{
public:
  explicit TestNode( const std::string &name_space, const std::string &node_name = "test_node" )
      : Node( node_name, name_space )
  {
  }
  TestNode( const std::string &name_space, const std::string &node_name,
            const rclcpp::NodeOptions &options )
      : Node( node_name, name_space, options )
  {
  }
  explicit TestNode( const std::string &name_space, const rclcpp::NodeOptions &options )
      : Node( "test_node", name_space, options )
  {
  }

  // Factory methods that register the objects
  template<typename MsgT>
  std::shared_ptr<TestPublisher<MsgT>>
  create_test_publisher( const std::string &topic,
                         const rclcpp::QoS &qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ) )
  {
    auto ptr = std::make_shared<TestPublisher<MsgT>>( shared_from_this(), topic, qos );
    register_connectable( ptr );
    return ptr;
  }

  template<typename MsgT>
  std::shared_ptr<TestSubscription<MsgT>>
  create_test_subscription( const std::string &topic,
                            const rclcpp::QoS &qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ),
                            bool latched = false )
  {
    auto ptr = std::make_shared<TestSubscription<MsgT>>( shared_from_this(), topic, qos, latched );
    register_connectable( ptr );
    return ptr;
  }

  template<typename ServiceT>
  std::shared_ptr<TestClient<ServiceT>> create_test_client( const std::string &service_name )
  {
    auto ptr = std::make_shared<TestClient<ServiceT>>( shared_from_this(), service_name );
    register_connectable( ptr );
    return ptr;
  }

  template<typename ActionT>
  std::shared_ptr<TestActionClient<ActionT>> create_test_action_client( const std::string &action_name )
  {
    auto ptr = std::make_shared<TestActionClient<ActionT>>( shared_from_this(), action_name );
    register_connectable( ptr );
    return ptr;
  }

  template<typename ServiceT>
  std::shared_ptr<TestServiceServer<ServiceT>>
  create_test_service_server( const std::string &service_name,
                              typename TestServiceServer<ServiceT>::Callback callback,
                              const rclcpp::QoS &qos = rclcpp::ServicesQoS() )
  {
    auto ptr = std::make_shared<TestServiceServer<ServiceT>>( shared_from_this(), service_name,
                                                              callback, qos );
    register_connectable( ptr );
    return ptr;
  }

  template<typename ActionT>
  std::shared_ptr<TestActionServer<ActionT>>
  create_test_action_server( const std::string &action_name,
                             typename TestActionServer<ActionT>::GoalCallback goal_cb,
                             typename TestActionServer<ActionT>::CancelCallback cancel_cb,
                             typename TestActionServer<ActionT>::AcceptedCallback accepted_cb )
  {
    auto ptr = std::make_shared<TestActionServer<ActionT>>( shared_from_this(), action_name,
                                                            goal_cb, cancel_cb, accepted_cb );
    register_connectable( ptr );
    return ptr;
  }

  // The main wait function
  bool wait_for_all_connections( TestExecutor &executor,
                                 std::chrono::seconds timeout = std::chrono::seconds( 10 ),
                                 std::string *pending_report = nullptr )
  {
    auto last_print = std::chrono::steady_clock::now();

    auto collect_disconnected = [&]() {
      std::vector<std::string> disconnected_names;
      for ( const auto &item : registry_ ) {
        if ( !item->is_connected() ) {
          disconnected_names.push_back( item->get_type() + ": " + item->get_name() );
        }
      }
      return disconnected_names;
    };

    auto format_pending = []( const std::vector<std::string> &names ) {
      std::stringstream ss;
      ss << "Waiting for connections (" << names.size() << " pending): ";
      for ( const auto &name : names ) ss << "[" << name << "] ";
      return ss.str();
    };

    const bool ok = executor.spin_until(
        [&]() {
          auto disconnected_names = collect_disconnected();
          const bool all_connected = disconnected_names.empty();

          // Print status every 2 seconds if waiting
          auto now = std::chrono::steady_clock::now();
          if ( !all_connected && ( now - last_print ) > 2s ) {
            const auto message = format_pending( disconnected_names );
            RCLCPP_WARN( this->get_logger(), "%s", message.c_str() );
            last_print = now;
          }
          return all_connected;
        },
        timeout );

    if ( !ok && pending_report == nullptr ) {
      // If no report pointer was provided, log the failure details here
      const auto disconnected_names = collect_disconnected();
      RCLCPP_ERROR( this->get_logger(), "Timeout waiting for connections! %s",
                    format_pending( disconnected_names ).c_str() );
    }

    if ( pending_report ) {
      if ( ok ) {
        pending_report->clear();
      } else {
        const auto disconnected_names = collect_disconnected();
        *pending_report = format_pending( disconnected_names );
      }
    }

    return ok;
  }

private:
  void register_connectable( std::shared_ptr<Connectable> item ) { registry_.push_back( item ); }

  std::vector<std::shared_ptr<Connectable>> registry_;
};

// =============================================================================
// Topic and Service/Action Helpers preserved for backward compatibility
// =============================================================================

/// Wait until the topic has at least min_publishers publishers.
inline bool wait_for_publishers( TestExecutor &executor, const rclcpp::Node::SharedPtr &node,
                                 const std::string &topic, size_t min_publishers,
                                 std::chrono::nanoseconds timeout )
{
  return executor.spin_until(
      [&node, &topic, min_publishers]() { return node->count_publishers( topic ) >= min_publishers; },
      timeout );
}

/// Wait until the topic has at least min_subscribers subscribers.
inline bool wait_for_subscribers( TestExecutor &executor, const rclcpp::Node::SharedPtr &node,
                                  const std::string &topic, size_t min_subscribers,
                                  std::chrono::nanoseconds timeout )
{
  return executor.spin_until(
      [&node, &topic, min_subscribers]() {
        return node->count_subscribers( topic ) >= min_subscribers;
      },
      timeout );
}

/// Wait for a service to appear on the ROS graph.
template<typename ServiceT>
bool wait_for_service( const typename rclcpp::Client<ServiceT>::SharedPtr &client,
                       TestExecutor &executor, std::chrono::nanoseconds timeout )
{
  return executor.spin_until(
      [&client]() { return client->wait_for_service( std::chrono::nanoseconds( 0 ) ); }, timeout );
}

/// Wait for an action server to appear on the ROS graph.
template<typename ActionT>
bool wait_for_action_server( const typename rclcpp_action::Client<ActionT>::SharedPtr &client,
                             TestExecutor &executor, std::chrono::nanoseconds timeout )
{
  return executor.spin_until(
      [&client]() { return client->wait_for_action_server( std::chrono::nanoseconds( 0 ) ); },
      timeout );
}

struct ServiceCallOptions {
  std::chrono::nanoseconds service_timeout{ kDefaultTimeout };
  std::chrono::nanoseconds response_timeout{ kDefaultTimeout };
};

/// Call a service and wait for the response.
template<typename ServiceT>
typename ServiceT::Response::SharedPtr
call_service( const typename rclcpp::Client<ServiceT>::SharedPtr &client,
              const typename ServiceT::Request::SharedPtr &request, TestExecutor &executor,
              const ServiceCallOptions &options = ServiceCallOptions() )
{
  if ( !wait_for_service<ServiceT>( client, executor, options.service_timeout ) ) {
    return nullptr;
  }
  auto future = client->async_send_request( request );
  if ( !executor.spin_until_future_complete( future, options.response_timeout ) ) {
    return nullptr;
  }
  return future.get();
}

struct ActionCallOptions {
  std::chrono::nanoseconds server_timeout{ kDefaultTimeout };
  std::chrono::nanoseconds goal_timeout{ kDefaultTimeout };
  std::chrono::nanoseconds result_timeout{ std::chrono::seconds( 30 ) };
};

/// Send an action goal and wait for the result.
template<typename ActionT>
std::optional<typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult>
call_action( const typename rclcpp_action::Client<ActionT>::SharedPtr &client,
             const typename ActionT::Goal &goal, TestExecutor &executor,
             const ActionCallOptions &options = ActionCallOptions() )
{
  if ( !wait_for_action_server<ActionT>( client, executor, options.server_timeout ) ) {
    return std::nullopt;
  }

  auto goal_future = client->async_send_goal( goal );
  if ( !executor.spin_until_future_complete( goal_future, options.goal_timeout ) ) {
    return std::nullopt;
  }
  auto goal_handle = goal_future.get();
  if ( !goal_handle ) {
    return std::nullopt;
  }

  auto result_future = client->async_get_result( goal_handle );
  if ( !executor.spin_until_future_complete( result_future, options.result_timeout ) ) {
    return std::nullopt;
  }
  return result_future.get();
}

// =============================================================================
// Assertions & Macros
// =============================================================================

// Internal helpers
inline ::testing::AssertionResult assert_service_exists( TestExecutor &executor,
                                                         rclcpp::Node::SharedPtr node,
                                                         const std::string &service_name,
                                                         std::chrono::nanoseconds timeout )
{
  bool found = executor.spin_until(
      [&]() {
        auto services = node->get_service_names_and_types();
        return services.find( service_name ) != services.end();
      },
      timeout );

  if ( found )
    return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "Service '" << service_name << "' failed to appear.";
}

inline ::testing::AssertionResult assert_action_server_exists( TestExecutor &executor,
                                                               rclcpp::Node::SharedPtr node,
                                                               const std::string &action_name,
                                                               std::chrono::nanoseconds timeout )
{
  // Note: checking graph for actions is tricky because actions expand to multiple topics/services.
  // Reliable way: create a temp action client and check readiness, or check for the 'action_name/_action/status' topic.
  // Here we check for the status topic which every action server publishes.
  std::string status_topic = action_name + "/_action/status";

  bool found = executor.spin_until(
      [&]() {
        auto topics = node->get_topic_names_and_types();
        return topics.find( status_topic ) != topics.end();
      },
      timeout );

  if ( found )
    return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "Action Server '" << action_name << "' failed to appear.";
}

// Macros

#define ASSERT_SERVICE_EXISTS( node, service, timeout )                                            \
  ASSERT_TRUE( hector_testing_utils::assert_service_exists(                                        \
      *executor_, node, service, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define EXPECT_SERVICE_EXISTS( node, service, timeout )                                            \
  EXPECT_TRUE( hector_testing_utils::assert_service_exists(                                        \
      *executor_, node, service, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define ASSERT_ACTION_EXISTS( node, action, timeout )                                              \
  ASSERT_TRUE( hector_testing_utils::assert_action_server_exists(                                  \
      *executor_, node, action, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define EXPECT_ACTION_EXISTS( node, action, timeout )                                              \
  EXPECT_TRUE( hector_testing_utils::assert_action_server_exists(                                  \
      *executor_, node, action, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define ASSERT_SERVICE_EXISTS_WITH_EXECUTOR( executor, node, service, timeout )                    \
  ASSERT_TRUE( hector_testing_utils::assert_service_exists(                                        \
      executor, node, service, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define EXPECT_SERVICE_EXISTS_WITH_EXECUTOR( executor, node, service, timeout )                    \
  EXPECT_TRUE( hector_testing_utils::assert_service_exists(                                        \
      executor, node, service, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define ASSERT_ACTION_EXISTS_WITH_EXECUTOR( executor, node, action, timeout )                      \
  ASSERT_TRUE( hector_testing_utils::assert_action_server_exists(                                  \
      executor, node, action, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

#define EXPECT_ACTION_EXISTS_WITH_EXECUTOR( executor, node, action, timeout )                      \
  EXPECT_TRUE( hector_testing_utils::assert_action_server_exists(                                  \
      executor, node, action, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) ) )

// =============================================================================
// Log Verification Helper
// =============================================================================

class LogCapture
{
public:
  struct LogMessage {
    int severity;
    std::string name;
    std::string message;
  };

  LogCapture()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    if ( active_instance_ ) {
      throw std::runtime_error( "Only one LogCapture instance can be active at a time!" );
    }
    active_instance_ = this;
    previous_handler_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler( &LogCapture::log_handler );
  }

  ~LogCapture()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    if ( active_instance_ == this ) {
      rcutils_logging_set_output_handler( previous_handler_ );
      active_instance_ = nullptr;
    }
  }

  LogCapture( const LogCapture & ) = delete;
  LogCapture &operator=( const LogCapture & ) = delete;

  // Wait for a log message matching the regex pattern
  bool wait_for_log( TestExecutor &executor, const std::string &pattern_regex,
                     std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    std::regex re( pattern_regex );
    return executor.spin_until(
        [this, &re]() {
          std::lock_guard<std::mutex> lock( mutex_ );
          for ( const auto &log : captured_logs_ ) {
            if ( std::regex_search( log.message, re ) ) {
              return true;
            }
          }
          return false;
        },
        timeout );
  }

  // Check if a log message matching the regex pattern exists (non-blocking)
  bool has_log( const std::string &pattern_regex ) const
  {
    std::regex re( pattern_regex );
    std::lock_guard<std::mutex> lock( mutex_ );
    for ( const auto &log : captured_logs_ ) {
      if ( std::regex_search( log.message, re ) ) {
        return true;
      }
    }
    return false;
  }

  // Clear captured logs
  void clear()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    captured_logs_.clear();
  }

private:
  static void log_handler( const rcutils_log_location_t *location, int severity, const char *name,
                           rcutils_time_point_value_t timestamp, const char *format, va_list *args )
  {
    char buffer[1024];
    // Copy args because vsnprintf modifies them
    va_list args_copy;
    va_copy( args_copy, *args );
    vsnprintf( buffer, sizeof( buffer ), format, args_copy );
    va_end( args_copy );

    {
      std::lock_guard<std::mutex> lock( mutex_ );
      if ( active_instance_ ) {
        active_instance_->captured_logs_.push_back( { severity, name ? name : "", buffer } );

        // Use the previous handler to print to console
        if ( active_instance_->previous_handler_ ) {
          active_instance_->previous_handler_( location, severity, name, timestamp, format, args );
        }
      }
    }
  }

  static LogCapture *active_instance_;
  static std::mutex mutex_;
  rcutils_logging_output_handler_t previous_handler_;
  std::vector<LogMessage> captured_logs_;
};

// Static inline definitions (C++17)
inline LogCapture *LogCapture::active_instance_ = nullptr;
inline std::mutex LogCapture::mutex_;

// =============================================================================
// Fixture Update
// =============================================================================

class HectorTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if ( !rclcpp::ok() ) {
      rclcpp::init( 0, nullptr );
    }
    // Allow derived classes to provide a custom executor (e.g. MultiThreaded)
    // by overriding create_test_executor()
    std::shared_ptr<rclcpp::Executor> internal_exec = create_test_executor();
    if ( !internal_exec ) {
      // Fallback if the user returns nullptr, though they shouldn't
      internal_exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }

    executor_ = std::make_shared<TestExecutor>( internal_exec );

    // Instantiate our new smart TestNode
    tester_node_ = std::make_shared<TestNode>( "hector_tester_node" );
    executor_->add_node( tester_node_ );
  }

  void TearDown() override
  {
    if ( rclcpp::ok() ) {
      rclcpp::shutdown();
    }
  }

  // Virtual method to allow overriding the executor type definition
  virtual std::shared_ptr<rclcpp::Executor> create_test_executor()
  {
    return std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  std::shared_ptr<TestExecutor> executor_;
  std::shared_ptr<TestNode> tester_node_;
};

class HectorTestFixtureWithContext : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<TestContext>();
    executor_ = std::make_shared<TestExecutor>( context_->context() );
    tester_node_ = std::make_shared<TestNode>( "hector_tester_node", context_->node_options() );
    executor_->add_node( tester_node_ );
  }

  void TearDown() override
  {
    tester_node_.reset();
    executor_.reset();
    context_.reset();
  }

  std::shared_ptr<TestContext> context_;
  std::shared_ptr<TestExecutor> executor_;
  std::shared_ptr<TestNode> tester_node_;
};

/// Create NodeOptions that load parameters from a YAML file.
inline rclcpp::NodeOptions
node_options_from_yaml( const std::string &params_file,
                        const std::vector<std::string> &extra_arguments = {} )
{
  std::vector<std::string> args = { "--ros-args", "--params-file", params_file };
  args.insert( args.end(), extra_arguments.begin(), extra_arguments.end() );
  rclcpp::NodeOptions options;
  options.arguments( args );
  options.automatically_declare_parameters_from_overrides( true );
  return options;
}

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_HECTOR_TESTING_UTILS_HPP
