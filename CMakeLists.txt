cmake_minimum_required(VERSION 3.5)
project(temoto_action_engine)

add_compile_options(-std=gnu++20 -g -fPIC)

# Check which compiler is required
if( "$ENV{TEMOTO_COMPILER}" STREQUAL "clang")
  set(CMAKE_CXX_COMPILER "clang++-6.0")
  add_compile_options(-Wthread-safety)
endif()

option(TESTS "Build tests" OFF)

find_package(Boost REQUIRED COMPONENTS filesystem system program_options )
find_package(nlohmann_json REQUIRED)

set(boost_libraries)
if(${Boost_MINOR_VERSION} GREATER_EQUAL 70)
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
  ${Boost_INCLUDE_DIRS}
)

set(libraries
  ${boost_libraries}
)

include_directories(
  ${headers}
)

# Library that combines core components of the action engine
add_library(${PROJECT_NAME}
  src/umrf.cpp
  src/umrf_node.cpp
  src/umrf_node_exec.cpp
  src/umrf_graph.cpp
  src/umrf_graph_exec.cpp
  src/action_indexer.cpp
  src/action_match_finder.cpp
  src/action_engine.cpp
  src/arg_parser.cpp
  src/umrf_json.cpp
  src/umrf_graph_fs.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${libraries}
)

#
# TESTS
#

if (TESTS)
  add_subdirectory(test)
endif()

#
# INSTALL
#

# Install the headers and the library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION bin
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION "lib/cmake/${PROJECT_NAME}"
  # NAMESPACE ${PROJECT_NAME}::
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
)

install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    DESTINATION "lib/cmake/${PROJECT_NAME}"
)

install(DIRECTORY include/temoto_action_engine/
  DESTINATION include/${PROJECT_NAME}
)

string(REPLACE ";" " " libraries "${libraries}")
set(TARGET_NAME ${PROJECT_NAME})
set(PKGCONFIG_LIBS
  ${libraries}
)
