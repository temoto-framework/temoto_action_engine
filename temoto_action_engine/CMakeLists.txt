cmake_minimum_required(VERSION 3.5)
project(temoto_action_engine)

add_compile_options(-std=c++11)

# Check which compiler is required
if( "$ENV{TEMOTO_COMPILER}" STREQUAL "clang")
  set(CMAKE_CXX_COMPILER "clang++-6.0")
  add_compile_options(-Wthread-safety)
endif()

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(class_loader REQUIRED)

set(boost_libraries)
if(${Boost_VERSION_MINOR} GREATER_EQUAL 70)
  # Get the full paths to boost libraries
  foreach(lib_target ${Boost_LIBRARIES})
    get_target_property(lib_location ${lib_target} LOCATION)
    list(APPEND boost_libraries ${lib_location})
  endforeach()  
else()
  set(boost_libraries ${Boost_LIBRARIES})
endif()

set(headers
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/rapidjson/include
  ${Boost_INCLUDE_DIRS}
  ${class_loader_INCLUDE_DIRS}
)

set(libraries
  ${boost_libraries}
  ${class_loader_LIBRARIES}
)

include_directories(
  ${headers}
)

# Library that combines core components of the action engine
add_library(${PROJECT_NAME} SHARED
  # ${headers}
  src/umrf.cpp
  src/umrf_node.cpp
  src/umrf_node_exec.cpp
  src/umrf_graph.cpp
  src/umrf_graph_exec.cpp
  src/action_indexer.cpp
  src/action_match_finder.cpp
  src/action_engine.cpp
  src/umrf_json_converter.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${libraries}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION bin
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/temoto_action_engine/
  DESTINATION include/${PROJECT_NAME}
)

string(REPLACE ";" " " libraries "${libraries}")
set(TARGET_NAME ${PROJECT_NAME})
set(PKGCONFIG_LIBS
  ${libraries}
)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.pc.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)

install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)