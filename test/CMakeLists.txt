cmake_minimum_required(VERSION 3.5)
project(temoto_action_engine_tests)

add_compile_options(-std=c++20 -g)

find_package(temoto_action_engine REQUIRED)
find_package(GTest REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

# Build the main test files
add_executable(tests
  src/json_tests.cpp
  src/engine_tests.cpp
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

  add_custom_target(action_lib_${action_name})
  add_dependencies(action_lib_${action_name} ${action_name})

  add_custom_command(TARGET action_lib_${action_name} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_BINARY_DIR}/actions/${action_name}/${action_name}.umrf.json
            ${CMAKE_CURRENT_BINARY_DIR})

  add_custom_command(TARGET action_lib_${action_name} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_BINARY_DIR}/actions/${action_name}/lib${action_name}.so
            ${CMAKE_CURRENT_BINARY_DIR})
endfunction()

# Add actions
build_action(ta_add_string)
build_action(ta_compare_number)
build_action(ta_compare_string)
build_action(ta_relay_string)
build_action(ta_ingest_result)
build_action(ta_trigger_result)
build_action(ta_trigger_result_multi)
build_action(ta_timer)

# Build the commandline graph executor
add_executable(run_graph_cmd
  src/run_graph_cmd.cpp
)

add_dependencies(run_graph_cmd
  temoto_action_engine
)

target_link_libraries(run_graph_cmd
  temoto_action_engine
  ${Boost_LIBRARIES}
)

# Copy the UMRF graphs
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/umrf_graphs/
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
        FILES_MATCHING PATTERN "*.graph.json" )
