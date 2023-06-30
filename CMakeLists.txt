cmake_minimum_required(VERSION 3.1)
project(temoto_action_engine)

add_compile_options(-std=c++17)

# Check which compiler is required
if( "$ENV{TEMOTO_COMPILER}" STREQUAL "clang")
  set(CMAKE_CXX_COMPILER "clang++-6.0")
  add_compile_options(-Wthread-safety)
endif()

option(TESTS "Build tests" OFF)

find_package(Boost COMPONENTS filesystem program_options REQUIRED)
find_package(class_loader REQUIRED)
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
  ${class_loader_INCLUDE_DIRS}
  test/actions/ta_example_1/include
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
  src/umrf.cpp
  src/umrf_node.cpp
  src/umrf_node_exec.cpp
  src/umrf_graph.cpp
  src/umrf_graph_exec.cpp
  src/action_indexer.cpp
  src/action_match_finder.cpp
  src/action_engine.cpp
  src/arg_parser.cpp
  src/umrf_json_converter.cpp
  src/umrf_graph_fs.cpp
)
target_link_libraries(${PROJECT_NAME}
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
