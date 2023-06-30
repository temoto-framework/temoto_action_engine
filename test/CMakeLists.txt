cmake_minimum_required(VERSION 3.1)
project(temoto_action_engine_tests)

add_compile_options(-std=c++17)

find_package(temoto_action_engine REQUIRED)
find_package(GTest REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

add_executable(tests src/tests.cpp)

target_link_libraries(tests 
  ${GTEST_LIBRARIES}
  gtest_main
  pthread
  temoto_action_engine
  # nlohmann_json
)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/umrf_graphs/example_b.json
  ${CMAKE_CURRENT_BINARY_DIR}/example_b.json COPYONLY
)