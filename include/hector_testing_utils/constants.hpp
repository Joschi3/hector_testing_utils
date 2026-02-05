#ifndef HECTOR_TESTING_UTILS_CONSTANTS_HPP
#define HECTOR_TESTING_UTILS_CONSTANTS_HPP

#include <chrono>
#include <cstddef>

namespace hector_testing_utils
{

using namespace std::chrono_literals;

/// @brief Default spin period for executor loops.
constexpr std::chrono::milliseconds kDefaultSpinPeriod{ 5 };

/// @brief Default timeout for wait operations.
constexpr std::chrono::seconds kDefaultTimeout{ 5 };

/// @brief Default maximum number of suggestions to display in failure messages.
constexpr size_t kDefaultMaxSuggestions{ 10 };

} // namespace hector_testing_utils

#endif // HECTOR_TESTING_UTILS_CONSTANTS_HPP
