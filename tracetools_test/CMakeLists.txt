cmake_minimum_required(VERSION 3.5)
project(tracetools_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Tests
if(BUILD_TESTING)
  add_executable(test_publisher
    src/test_publisher.cpp
  )
  ament_target_dependencies(test_publisher
    rclcpp
    std_msgs
  )
  add_executable(test_subscription
    src/test_subscription.cpp
  )
  ament_target_dependencies(test_subscription
    rclcpp
    std_msgs
  )

  install(TARGETS
    test_publisher
    test_subscription
    DESTINATION lib/${PROJECT_NAME}
  )

  find_package(ament_cmake_pytest REQUIRED)

  # Run each test in its own pytest invocation
  set(_tracetools_test_pytest_tests
    test/test_publisher.py
    test/test_subscription.py
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
