cmake_minimum_required(VERSION 3.8)
project(hector_testing_utils)

option(ENABLE_COVERAGE_TESTING "Enable coverage reporting" OFF)

# --- MOVED BLOCK: MUST BE HERE ---
if(ENABLE_COVERAGE_TESTING)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(--coverage)
    add_link_options(--coverage)
    message(STATUS "Coverage testing enabled for ${PROJECT_NAME}")
  endif()
endif()
# ---------------------------------

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_library(hector_testing_utils INTERFACE)
target_include_directories(
  hector_testing_utils
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)

ament_target_dependencies(hector_testing_utils INTERFACE rclcpp rclcpp_action)

install(TARGETS hector_testing_utils EXPORT export_${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(example_interfaces REQUIRED)

  ament_add_gtest(test_hector_testing_utils test/test_hector_testing_utils.cpp)
  target_link_libraries(test_hector_testing_utils hector_testing_utils)
  ament_target_dependencies(test_hector_testing_utils rclcpp rclcpp_action
                            example_interfaces std_msgs)
  target_compile_definitions(
    test_hector_testing_utils
    PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")

  ament_add_gtest(test_integration test/test_integration.cpp)
  target_link_libraries(test_integration hector_testing_utils)
  ament_target_dependencies(test_integration rclcpp rclcpp_action
                            example_interfaces std_msgs)
  target_compile_definitions(
    test_integration PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")

  ament_add_gtest(test_timing test/test_timing.cpp)
  target_link_libraries(test_timing hector_testing_utils)
  ament_target_dependencies(test_timing rclcpp std_msgs)

  ament_add_gtest(test_context test/test_context.cpp)
  target_link_libraries(test_context hector_testing_utils)
  ament_target_dependencies(test_context rclcpp example_interfaces std_msgs)
  target_compile_definitions(
    test_context PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")

  ament_add_gtest(test_log_capture test/test_log_capture.cpp)
  target_link_libraries(test_log_capture hector_testing_utils)
  ament_target_dependencies(test_log_capture rclcpp std_msgs)

  ament_add_gtest(test_background_executor test/test_background_executor.cpp)
  target_link_libraries(test_background_executor hector_testing_utils)
  ament_target_dependencies(test_background_executor rclcpp std_msgs
                            example_interfaces)
  target_compile_definitions(
    test_background_executor
    PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")

  ament_add_gtest(test_executor_types test/test_executor_types.cpp)
  target_link_libraries(test_executor_types hector_testing_utils)
  ament_target_dependencies(test_executor_types rclcpp)

endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action)
ament_export_include_directories(include)

ament_package()
