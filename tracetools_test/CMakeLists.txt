cmake_minimum_required(VERSION 3.5)
project(tracetools_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Tests
if(BUILD_TESTING)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(tracetools REQUIRED)

  add_executable(test_publisher
    src/test_publisher.cpp
  )
  ament_target_dependencies(test_publisher
    rclcpp
    std_msgs
    tracetools
  )
  target_link_libraries(test_publisher -rdynamic)
  add_executable(test_subscription
    src/test_subscription.cpp
  )
  ament_target_dependencies(test_subscription
    rclcpp
    std_msgs
    tracetools
  )
  target_link_libraries(test_subscription -rdynamic)
  add_executable(test_intra
    src/test_intra.cpp
  )
  ament_target_dependencies(test_intra
    rclcpp
    std_msgs
    tracetools
  )
  target_link_libraries(test_intra -rdynamic)
  add_executable(test_ping
    src/test_ping.cpp
  )
  ament_target_dependencies(test_ping
    rclcpp
    std_msgs
    tracetools
  )
  target_link_libraries(test_ping -rdynamic)
  add_executable(test_pong
    src/test_pong.cpp
  )
  ament_target_dependencies(test_pong
    rclcpp
    std_msgs
    tracetools
  )
  target_link_libraries(test_pong -rdynamic)
  add_executable(test_timer
    src/test_timer.cpp
  )
  ament_target_dependencies(test_timer
    rclcpp
    tracetools
  )
  target_link_libraries(test_timer -rdynamic)
  add_executable(test_service
    src/test_service.cpp
  )
  ament_target_dependencies(test_service
    rclcpp
    std_srvs
    tracetools
  )
  target_link_libraries(test_service -rdynamic)
  add_executable(test_service_ping
    src/test_service_ping.cpp
  )
  ament_target_dependencies(test_service_ping
    rclcpp
    std_srvs
    tracetools
  )
  target_link_libraries(test_service_ping -rdynamic)
  add_executable(test_service_pong
    src/test_service_pong.cpp
  )
  ament_target_dependencies(test_service_pong
    rclcpp
    std_srvs
    tracetools
  )
  target_link_libraries(test_service_pong -rdynamic)

  install(TARGETS
    test_intra
    test_ping
    test_pong
    test_publisher
    test_service
    test_service_ping
    test_service_pong
    test_subscription
    test_timer
    DESTINATION lib/${PROJECT_NAME}
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  # Run each test in its own pytest invocation
  set(_tracetools_test_pytest_tests
    test/test_intra.py
    test/test_node.py
    test/test_publisher.py
    test/test_service.py
    test/test_service_callback.py
    test/test_subscription.py
    test/test_subscription_callback.py
    test/test_timer.py
  )

  foreach(_test_path ${_tracetools_test_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
      APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
        PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
    )
  endforeach()
endif()

ament_package()
