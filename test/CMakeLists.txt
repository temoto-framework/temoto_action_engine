cmake_minimum_required(VERSION 3.1)
project(temoto_action_engine_tests)

add_compile_options(-std=c++17)

find_package(temoto_action_engine REQUIRED)
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

# Build the main test files
add_executable(tests 
  src/tests.cpp
)

target_link_libraries(tests 
  ${GTEST_LIBRARIES}
  gtest_main
  pthread
  temoto_action_engine
)

# Builds the actions and copies the .so and .json files to build dir POST_BUILD
function(build_action action_name)
  add_subdirectory(actions/${action_name})

  add_custom_target(action_lib)
  add_dependencies(action_lib ${action_name})

  add_custom_command(TARGET action_lib POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_BINARY_DIR}/actions/${action_name}/${action_name}.umrf.json
            ${CMAKE_CURRENT_BINARY_DIR})

  add_custom_command(TARGET action_lib POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_BINARY_DIR}/actions/${action_name}/lib${action_name}.so
            ${CMAKE_CURRENT_BINARY_DIR})
endfunction()

# Add actions
build_action(ta_example_1)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/umrf_graphs/example_b.json
  ${CMAKE_CURRENT_BINARY_DIR}/example_b.json COPYONLY
)