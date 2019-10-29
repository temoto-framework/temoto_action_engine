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
  UmrfJsonGraph.msg
  StopUmrfJsonGraph.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include "${CMAKE_CURRENT_SOURCE_DIR}/rapidjson/include"
  LIBRARIES temoto_ae_components temoto_ae_umrf_json
  CATKIN_DEPENDS roscpp std_msgs class_loader
#  DEPENDS system_lib
) 

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/rapidjson/include"
)

# Library that combines core components of the action engine
add_library(temoto_ae_components 
  src/action_handle.cpp
  src/umrf_graph_helper.cpp
  src/umrf.cpp
  src/action_indexer.cpp
  src/action_match_finder.cpp
  src/action_executor.cpp
  src/action_engine.cpp
)
add_dependencies(temoto_ae_components ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(temoto_ae_components 
  ${catkin_LIBRARIES}
)

install(TARGETS temoto_ae_components
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Library that combines UMRF json converter components
add_library(temoto_ae_umrf_json
  src/umrf_json_converter.cpp
)
add_dependencies(temoto_ae_umrf_json ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(temoto_ae_umrf_json 
  ${catkin_LIBRARIES}
)

install(TARGETS temoto_ae_umrf_json
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Action engine test
add_executable(temoto_ae_test
  src/temoto_ae_base.cpp
  src/umrf_json_converter.cpp
)

add_dependencies(temoto_ae_test
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(temoto_ae_test
  ${catkin_LIBRARIES}
  temoto_ae_components 
) 

# Action engine node
add_executable(action_engine_node
  src/action_engine_node.cpp
  src/umrf_json_converter.cpp
)

add_dependencies(action_engine_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(action_engine_node
  ${catkin_LIBRARIES}
  temoto_ae_components yaml-cpp
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
)

target_link_libraries(parser_node
  ${catkin_LIBRARIES}
  temoto_ae_components 
)

install(TARGETS parser_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rapidjson/include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

