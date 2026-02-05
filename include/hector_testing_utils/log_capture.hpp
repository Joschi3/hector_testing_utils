#ifndef HECTOR_TESTING_UTILS_LOG_CAPTURE_HPP
#define HECTOR_TESTING_UTILS_LOG_CAPTURE_HPP

#include <chrono>
#include <cstdarg>
#include <mutex>
#include <regex>
#include <string>
#include <vector>

#include <rcutils/logging.h>

#include <hector_testing_utils/constants.hpp>
#include <hector_testing_utils/test_executor.hpp>

namespace hector_testing_utils
{

/// @brief Utility to capture and verify ROS log messages.
///
/// This class intercepts ROS 2 logging calls (via rcutils) and allows tests to assert that specific
/// log messages have been published. It uses regex matching.
class LogCapture
{
public:
  /// @brief A captured log message.
  struct LogMessage {
    int severity;
    std::string name;
    std::string message;
  };

  /// @brief Construct a new Log Capture object.
  /// @throws std::runtime_error if another LogCapture instance already exists (singleton enforcement).
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

  /// @brief Destroy the Log Capture object and restore the previous log handler.
  ~LogCapture()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    if ( active_instance_ == this ) {
      rcutils_logging_set_output_handler( previous_handler_ );
      active_instance_ = nullptr;
    }
  }

  // Non-copyable
  LogCapture( const LogCapture & ) = delete;
  LogCapture &operator=( const LogCapture & ) = delete;

  /// @brief Wait until a log message matching the regex pattern is captured.
  /// @param executor Executor to spin (for timeout).
  /// @param pattern_regex Regex pattern to look for.
  /// @param timeout Maximum wait time.
  /// @return true if found, false on timeout.
  bool wait_for_log( TestExecutor &executor, const std::string &pattern_regex,
                     std::chrono::nanoseconds timeout = kDefaultTimeout )
  {
    std::regex re( pattern_regex );
    return executor.spin_until( [this, &re]() { return has_log( re ); }, timeout );
  }

  /// @brief Check if a log message matching the regex pattern has been captured.
  /// @param pattern_regex Regex pattern.
  /// @return true if found.
  bool has_log( const std::string &pattern_regex ) const
  {
    std::regex re( pattern_regex );
    return has_log( re );
  }

  /// @brief Clear all captured logs.
  void clear()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    captured_logs_.clear();
  }

  /// @brief Get all captured logs.
  /// @return Vector of captured log messages.
  std::vector<LogMessage> get_logs() const
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return captured_logs_;
  }

private:
  bool has_log( const std::regex &re ) const
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    for ( const auto &log : captured_logs_ ) {
      if ( std::regex_search( log.message, re ) ) {
        return true;
      }
    }
    return false;
  }

  static void log_handler( const rcutils_log_location_t *location, int severity, const char *name,
                           rcutils_time_point_value_t timestamp, const char *format, va_list *args )
  {
    char buffer[1024];
    va_list args_copy;
    va_copy( args_copy, *args );
    vsnprintf( buffer, sizeof( buffer ), format, args_copy );
    va_end( args_copy );

    {
      std::lock_guard<std::mutex> lock( mutex_ );
      if ( active_instance_ ) {
        active_instance_->captured_logs_.push_back( { severity, name ? name : "", buffer } );
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

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_LOG_CAPTURE_HPP
