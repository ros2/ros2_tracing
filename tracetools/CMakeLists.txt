cmake_minimum_required(VERSION 3.5)
project(tracetools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)

option(WITH_LTTNG "Include support for tracing with LTTng" OFF)
if(WITH_LTTNG)
  # Try to find LTTng
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LTTNG REQUIRED lttng-ust)
endif()
if(LTTNG_FOUND)
  # Generate necessary LTTng files
  # If successful, enable tracing
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/src/tp_call.c ${PROJECT_SOURCE_DIR}/include/tp_call.h
    COMMAND lttng-gen-tp tp_call.tp -o tp_call.c -o ../include/tp_call.h
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/src
    DEPENDS ${PROJECT_SOURCE_DIR}/src/tp_call.tp
  )
  set(LTTNG_GENERATED
    include/tp_call.h
    src/tp_call.c
  )
  set(TRACING_ENABLED TRUE)
  message("LTTng found: tracing enabled")
elseif(WITH_LTTNG)
  message("LTTng NOT found: tracing disabled")
endif()

include_directories(include)

# Status checking tool
add_executable(status
  src/status.c
  src/tracetools.c
  src/utils.cpp
)
target_link_libraries(status
  ${PROJECT_NAME}
)
ament_target_dependencies(status
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
  list(APPEND SOURCES ${LTTNG_GENERATED})
endif()

add_library(${PROJECT_NAME} ${SOURCES})
if(TRACING_ENABLED)
  target_compile_definitions(${PROJECT_NAME} PUBLIC WITH_LTTNG)
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
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
