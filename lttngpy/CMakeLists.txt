cmake_minimum_required(VERSION 3.20)

project(lttngpy)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
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
option(
  LTTNGPY_DISABLED
  "Explicitly disable support, don't link against liblttng-ctl"
  ${DISABLED_DEFAULT})

# By default, without the settings below, find_package(Python3) will attempt
# to find the newest python version it can, and additionally will find the
# most specific version.  For instance, on a system that has
# /usr/bin/python3.10, /usr/bin/python3.11, and /usr/bin/python3, it will find
# /usr/bin/python3.11, even if /usr/bin/python3 points to /usr/bin/python3.10.
# The behavior we want is to prefer the "system" installed version unless the
# user specifically tells us othewise through the Python3_EXECUTABLE hint.
# Setting CMP0094 to NEW means that the search will stop after the first
# python version is found.  Setting Python3_FIND_UNVERSIONED_NAMES means that
# the search will prefer /usr/bin/python3 over /usr/bin/python3.11.  And that
# latter functionality is only available in CMake 3.20 or later, so we need
# at least that version.
cmake_policy(SET CMP0094 NEW)
set(Python3_FIND_UNVERSIONED_NAMES FIRST)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

if(NOT LTTNGPY_DISABLED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LTTNG_CTL REQUIRED lttng-ctl)
  set(LTTNG_CTL_VERSION ${LTTNG_CTL_VERSION})

  string(REGEX MATCH "([0-9]+)\.([0-9]+)\.([0-9]+)" dummy "${LTTNG_CTL_VERSION}")
  set(LTTNG_CTL_VERSION_MAJOR "${CMAKE_MATCH_1}")
  set(LTTNG_CTL_VERSION_MINOR "${CMAKE_MATCH_2}")
  set(LTTNG_CTL_VERSION_PATCH "${CMAKE_MATCH_3}")
else()
  set(LTTNG_CTL_VERSION "")
  set(LTTNG_CTL_VERSION_MAJOR "0")
  set(LTTNG_CTL_VERSION_MINOR "0")
  set(LTTNG_CTL_VERSION_PATCH "0")
endif()

# Store configuration variable for buildtime use
#   LTTNGPY_DISABLED
#   LTTNG_CTL_VERSION
#   LTTNG_CTL_VERSION_MAJOR
#   LTTNG_CTL_VERSION_MINOR
#   LTTNG_CTL_VERSION_PATCH
configure_file(src/lttngpy/config.hpp.in src/lttngpy/config.hpp)

ament_python_install_package(${PROJECT_NAME})

set(SOURCES
  src/lttngpy/_lttngpy_pybind11.cpp
  src/lttngpy/status.cpp
)
if(NOT LTTNGPY_DISABLED)
  list(APPEND SOURCES
    src/lttngpy/channel.cpp
    src/lttngpy/context_app.cpp
    src/lttngpy/context_lttng.cpp
    src/lttngpy/context_perf.cpp
    src/lttngpy/event.cpp
    src/lttngpy/lttng.cpp
    src/lttngpy/session.cpp
  )
endif()

pybind11_add_module(_lttngpy_pybind11 SHARED ${SOURCES})

if(CMAKE_C_COMPILER_ID MATCHES "Clang" AND NOT APPLE)
  target_link_libraries(_lttngpy_pybind11 PRIVATE atomic)
endif()

target_include_directories(_lttngpy_pybind11 PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/src>"
)
if(NOT LTTNGPY_DISABLED)
  target_link_libraries(_lttngpy_pybind11 PRIVATE ${LTTNG_CTL_LIBRARIES})
endif()

# Set the build location and install location for a CPython extension
install(TARGETS _lttngpy_pybind11
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  if(NOT LTTNGPY_DISABLED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_pytest REQUIRED)

    # Using source files, because I can't seem to be able to link against _lttngpy_pybind11
    ament_add_gtest(test_context_app test/test_context_app.cpp src/lttngpy/context_app.cpp)
    if(TARGET test_context_app)
      target_include_directories(test_context_app PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/src>"
      )
    endif()

    ament_add_gtest(test_context_lttng test/test_context_lttng.cpp src/lttngpy/context_lttng.cpp)
    if(TARGET test_context_lttng)
      target_link_libraries(test_context_lttng ${LTTNG_CTL_LIBRARIES})
      target_include_directories(test_context_lttng PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/src>"
      )
    endif()

    ament_add_gtest(test_context_perf test/test_context_perf.cpp src/lttngpy/context_perf.cpp)
    if(TARGET test_context_perf)
      target_link_libraries(test_context_perf ${LTTNG_CTL_LIBRARIES})
      target_include_directories(test_context_perf PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/src>"
      )
    endif()

    set(_lttngpy_pytest_tests
      test/test_constants.py
      test/test_session.py
    )

    foreach(_test_path ${_lttngpy_pytest_tests})
      get_filename_component(_test_name ${_test_path} NAME_WE)
      ament_add_pytest_test(${_test_name} ${_test_path}
        APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
          PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 120
        WERROR ON
      )
    endforeach()
  endif()
endif()

ament_package()
