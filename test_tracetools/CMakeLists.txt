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
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  add_executable(test_publisher
    src/test_publisher.cpp
  )
  ament_target_dependencies(test_publisher
    rclcpp
    std_msgs
  )
  add_executable(test_intra
    src/test_intra.cpp
  )
  ament_target_dependencies(test_intra
    rclcpp
    std_msgs
  )
  add_executable(test_lifecycle_node
    src/test_lifecycle_node.cpp
  )
  ament_target_dependencies(test_lifecycle_node
    rclcpp
    rclcpp_lifecycle
  )
  add_executable(test_lifecycle_client
    src/test_lifecycle_client.cpp
  )
  ament_target_dependencies(test_lifecycle_client
    lifecycle_msgs
    rclcpp
  )
  add_executable(test_ping
    src/test_ping.cpp
  )
  ament_target_dependencies(test_ping
    rclcpp
    std_msgs
  )
  add_executable(test_pong
    src/test_pong.cpp
  )
  ament_target_dependencies(test_pong
    rclcpp
    std_msgs
  )
  add_executable(test_timer
    src/test_timer.cpp
  )
  ament_target_dependencies(test_timer
    rclcpp
  )
  add_executable(test_service_ping
    src/test_service_ping.cpp
  )
  ament_target_dependencies(test_service_ping
    rclcpp
    std_srvs
  )
  add_executable(test_service_pong
    src/test_service_pong.cpp
  )
  ament_target_dependencies(test_service_pong
    rclcpp
    std_srvs
  )

  install(TARGETS
    test_intra
    test_lifecycle_node
    test_lifecycle_client
    test_ping
    test_pong
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
      ament_target_dependencies(test_utils
        tracetools
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
      test/test_publisher.py
      test/test_service.py
      test/test_subscription.py
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
