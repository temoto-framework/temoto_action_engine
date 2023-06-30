cmake_minimum_required(VERSION 3.1)
project(ta_example_1)

add_compile_options(-std=c++17)

find_package(temoto_action_engine REQUIRED)

include_directories(include)

add_library(ta_example_1 SHARED
  src/ta_example_1.cpp
)
target_link_libraries(ta_example_1
  temoto_action_engine
)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ta_example_1.umrf.json
  ${CMAKE_CURRENT_BINARY_DIR}/ta_example_1.umrf.json COPYONLY
)
class_loader_hide_library_symbols(ta_example_1)