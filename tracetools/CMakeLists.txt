cmake_minimum_required(VERSION 3.5)
project(tracetools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

option(TRACETOOLS_DISABLED "Explicitly disable support for tracing with LTTng" OFF)
if(NOT TRACETOOLS_DISABLED)
  # Set TRACING_ENABLED if we can find lttng-ust
  find_package(PkgConfig)
  if(PkgConfig_FOUND)
    pkg_check_modules(LTTNG lttng-ust)
    if(LTTNG_FOUND)
      set(TRACING_ENABLED TRUE)
      message("LTTng found: tracing enabled")
    endif()
  endif()
endif()

# store configuration variables for runtime use
configure_file(include/${PROJECT_NAME}/config.h.in include/${PROJECT_NAME}/config.h)

# add both source and output include, to capture config.h
include_directories(include ${PROJECT_BINARY_DIR}/include)

# Status checking tool
add_executable(status
  src/status.c
  src/tracetools.c
  src/utils.cpp
)
target_link_libraries(status
  ${PROJECT_NAME}
)
install(TARGETS
  status
  DESTINATION lib/${PROJECT_NAME}
)

# Tracetools lib
set(SOURCES
  src/tracetools.c
  src/utils.cpp
)
if(TRACING_ENABLED)
  list(APPEND SOURCES
    include/tracetools/tp_call.h
    src/tp_call.c
  )
endif()

add_library(${PROJECT_NAME} SHARED ${SOURCES})
if(TRACING_ENABLED)
  target_compile_definitions(${PROJECT_NAME} PUBLIC TRACETOOLS_LTTNG_ENABLED)
  target_link_libraries(${PROJECT_NAME} ${LTTNG_LIBRARIES} -ldl)
else()
  target_link_libraries(${PROJECT_NAME})
endif()

ament_export_interfaces(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  FILES ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/config.h
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
if(TRACING_ENABLED)
  ament_export_libraries(${PROJECT_NAME} ${LTTNG_LIBRARIES})
else()
  ament_export_libraries(${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Only build tracetools utils tests if LTTng is enabled and found
  if(TRACING_ENABLED)
    ament_add_gtest(test_utils test/test_utils.cpp)
    if(TARGET test_utils)
      target_link_libraries(test_utils ${PROJECT_NAME} -rdynamic)
    endif()
  endif()
endif()

ament_package()

add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")
