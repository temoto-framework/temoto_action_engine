cmake_minimum_required(VERSION 3.1)
project(ta_append_str_a)

add_compile_options(-std=c++17)

find_package(temoto_action_engine REQUIRED)

include_directories(include)

add_library(ta_append_str_a SHARED
  src/ta_append_str_a.cpp
)
target_link_libraries(ta_append_str_a
  temoto_action_engine
)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ta_append_str_a.umrf.json
  ${CMAKE_CURRENT_BINARY_DIR}/ta_append_str_a.umrf.json COPYONLY
)
class_loader_hide_library_symbols(ta_append_str_a)