cmake_minimum_required(VERSION 3.5)
project(test_tracetools)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(WIN32 OR APPLE OR ANDROID)
  set(DISABLED_DEFAULT ON)
else()
  set(DISABLED_DEFAULT OFF)
endif()
option(TRACETOOLS_DISABLED "Explicitly disable support for tracing" ${DISABLED_DEFAULT})
option(TRACETOOLS_TRACEPOINTS_EXCLUDED "Do not include tracepoints" OFF)

if(TRACETOOLS_DISABLED)
  set(TRACETOOLS_TRACEPOINTS_EXCLUDED TRUE)
endif()

if(NOT TRACETOOLS_TRACEPOINTS_EXCLUDED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LTTNG REQUIRED lttng-ust)
endif()

# Tests
if(BUILD_TESTING)
  # tracetools is exported by rclcpp
  find_package(lifecycle_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(rcpputils REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  # The utility lib is needed even if TRACETOOLS_DISABLED; it's just empty
  add_library(${PROJECT_NAME}_mark_process src/mark_process.cpp)
  if(NOT TRACETOOLS_DISABLED)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LTTNG REQUIRED lttng-ust)
    target_link_libraries(${PROJECT_NAME}_mark_process PRIVATE
      ${LTTNG_LIBRARIES}
      rcpputils::rcpputils
    )
  endif()
  target_include_directories(${PROJECT_NAME}_mark_process PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  )
  if(TRACETOOLS_DISABLED)
    target_compile_definitions(${PROJECT_NAME}_mark_process PRIVATE TRACETOOLS_DISABLED)
  endif()
  install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )
  install(
    TARGETS ${PROJECT_NAME}_mark_process
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )

  add_executable(test_publisher
    src/test_publisher.cpp
  )
  target_link_libraries(test_publisher PRIVATE
    ${PROJECT_NAME}_mark_process
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_intra
    src/test_intra.cpp
  )
  target_link_libraries(test_intra PRIVATE
    ${PROJECT_NAME}_mark_process
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_lifecycle_node
    src/test_lifecycle_node.cpp
  )
  target_link_libraries(test_lifecycle_node PRIVATE
    ${PROJECT_NAME}_mark_process
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  )

  add_executable(test_lifecycle_client
    src/test_lifecycle_client.cpp
  )
  target_link_libraries(test_lifecycle_client PRIVATE
    ${PROJECT_NAME}_mark_process
    ${lifecycle_msgs_TARGETS}
    rclcpp::rclcpp
  )

  add_executable(test_ping
    src/test_ping.cpp
  )
  target_link_libraries(test_ping PRIVATE
    ${PROJECT_NAME}_mark_process
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_pong
    src/test_pong.cpp
  )
  target_link_libraries(test_pong PRIVATE
    ${PROJECT_NAME}_mark_process
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_generic_ping
    src/test_generic_ping.cpp
  )
  target_link_libraries(test_generic_ping PRIVATE
    ${PROJECT_NAME}_mark_process
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_generic_pong
    src/test_generic_pong.cpp
  )
  target_link_libraries(test_generic_pong PRIVATE
    ${PROJECT_NAME}_mark_process
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )

  add_executable(test_timer
    src/test_timer.cpp
  )
  target_link_libraries(test_timer PRIVATE
    ${PROJECT_NAME}_mark_process
    rclcpp::rclcpp
  )

  add_executable(test_service_ping
    src/test_service_ping.cpp
  )
  target_link_libraries(test_service_ping PRIVATE
    ${PROJECT_NAME}_mark_process
    rclcpp::rclcpp
    ${std_srvs_TARGETS}
  )

  add_executable(test_service_pong
    src/test_service_pong.cpp
  )
  target_link_libraries(test_service_pong PRIVATE
    ${PROJECT_NAME}_mark_process
    rclcpp::rclcpp
    ${std_srvs_TARGETS}
  )

  install(TARGETS
    test_intra
    test_lifecycle_node
    test_lifecycle_client
    test_ping
    test_pong
    test_generic_ping
    test_generic_pong
    test_publisher
    test_service_ping
    test_service_pong
    test_timer
    DESTINATION lib/${PROJECT_NAME}
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Utils do not exist if TRACETOOLS_DISABLED
  if(NOT TRACETOOLS_DISABLED)
    find_package(tracetools REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_utils test/test_utils.cpp)
    if(TARGET test_utils)
      target_link_libraries(test_utils
        tracetools::tracetools
      )
    endif()
  endif()

  # Only run tracing tests if instrumentation and tracepoints are included
  if(NOT TRACETOOLS_TRACEPOINTS_EXCLUDED)
    find_package(ament_cmake_pytest REQUIRED)
    set(_test_tracetools_pytest_tests
      test/test_buffer.py
      test/test_executor.py
      test/test_intra.py
      test/test_intra_pub_sub.py
      test/test_lifecycle_node.py
      test/test_node.py
      test/test_pub_sub.py
      test/test_generic_pub_sub.py
      test/test_publisher.py
      test/test_service.py
      test/test_subscription.py
      test/test_generic_subscription.py
      test/test_timer.py
    )
    foreach(_test_path ${_test_tracetools_pytest_tests})
      get_filename_component(_test_name ${_test_path} NAME_WE)
      ament_add_pytest_test(${_test_name} ${_test_path}
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 60
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
    endforeach()
  endif()
endif()

ament_package()
