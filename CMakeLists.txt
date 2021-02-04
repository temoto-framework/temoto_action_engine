cmake_minimum_required(VERSION 2.8.3)
project(temoto_action_engine)

add_compile_options(-std=c++11)

# Check which compiler is required
if( "$ENV{TEMOTO_COMPILER}" STREQUAL "clang")
  set(CMAKE_CXX_COMPILER "clang++-6.0")
  add_compile_options(-Wthread-safety)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  class_loader
  message_generation
)

add_message_files(FILES
  UmrfGraph.msg
  UmrfGraphDiff.msg
  StopUmrfGraph.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Build yaml-cpp version 0.6.2
include(ExternalProject)

ExternalProject_Add(yaml-cpp062
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp
  BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/build
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DYAML_CPP_BUILD_TESTS=OFF
    -Wno-dev
  BUILD_COMMAND $(MAKE) -j
  INSTALL_COMMAND cmake -E echo "Skipping install step."
)

set(headers
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/rapidjson/include
)

set(libraries
  ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/build/libyaml-cpp.so
)

catkin_package(
  INCLUDE_DIRS ${headers}
  LIBRARIES temoto_ae_components temoto_ae_umrf_json
  CATKIN_DEPENDS roscpp std_msgs class_loader
  #DEPENDS
) 

include_directories(
  ${headers}
  ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/include
)

# Library that combines core components of the action engine
add_library(temoto_ae_components 
  src/umrf.cpp
  src/umrf_node.cpp
  src/umrf_node_exec.cpp
  src/umrf_graph.cpp
  src/umrf_graph_exec.cpp
  src/action_indexer.cpp
  src/action_match_finder.cpp
  src/action_engine.cpp
)
add_dependencies(temoto_ae_components 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  yaml-cpp062
)
target_link_libraries(temoto_ae_components 
  ${catkin_LIBRARIES}
  ${libraries}
)

install(TARGETS temoto_ae_components
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Library that combines UMRF json converter components
add_library(temoto_ae_umrf_json
  src/umrf_json_converter.cpp
)
add_dependencies(temoto_ae_umrf_json
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  yaml-cpp062
)

target_link_libraries(temoto_ae_umrf_json 
  ${catkin_LIBRARIES}
  ${libraries}
)

install(TARGETS temoto_ae_umrf_json
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Action engine node
add_executable(action_engine_node
  src/action_engine_node.cpp
  src/umrf_json_converter.cpp
)

add_dependencies(action_engine_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  yaml-cpp062
)

target_link_libraries(action_engine_node
  ${catkin_LIBRARIES}
  temoto_ae_components
  ${libraries}
)

install(TARGETS action_engine_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Parser node
add_executable(parser_node
  src/parser_node.cpp
  src/umrf_json_converter.cpp
)

add_dependencies(parser_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  yaml-cpp062
)

target_link_libraries(parser_node
  ${catkin_LIBRARIES}
  temoto_ae_components
  ${libraries}
)

install(TARGETS parser_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Graph modifier node
add_executable(graph_modifier_node
  src/graph_modifier_node.cpp
  src/umrf_json_converter.cpp
)

add_dependencies(graph_modifier_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  yaml-cpp062
)

target_link_libraries(graph_modifier_node
  ${catkin_LIBRARIES}
  temoto_ae_components
  ${libraries}
)

install(TARGETS graph_modifier_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install other stuff
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rapidjson/include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})