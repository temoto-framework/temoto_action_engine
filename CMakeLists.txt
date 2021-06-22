cmake_minimum_required(VERSION 2.8.3)
project(temoto_action_engine)

add_compile_options(-std=c++11 -Wno-dev)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  class_loader
  message_generation
)

find_package(Boost COMPONENTS program_options REQUIRED)

add_message_files(FILES
  BroadcastStartUmrfGraph.msg
  BroadcastStopUmrfGraph.msg
  UmrfGraphDiff.msg
)

add_service_files(FILES
  StartUmrfGraph.srv
  StopUmrfGraph.srv
  GetUmrfGraphs.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include(ExternalProject)

ExternalProject_Add(temoto_action_engine_000
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/temoto_action_engine
  BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/temoto_action_engine/build
  CMAKE_ARGS
    -Wno-dev
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND cmake -E echo "Skipping install step."
  BUILD_ALWAYS true
)

set(headers
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_action_engine/include
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_action_engine/rapidjson/include
)

set(libraries
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_action_engine/build/libtemoto_action_engine.so
)

catkin_package(
  INCLUDE_DIRS ${headers}
  LIBRARIES ${libraries}
  CATKIN_DEPENDS roscpp std_msgs class_loader
  #DEPENDS
) 

include_directories(
  ${headers}
)

# Action engine node
add_executable(action_engine_node
  src/action_engine_node.cpp
  src/action_engine_ros1.cpp
)

add_dependencies(action_engine_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  temoto_action_engine_000
)

target_link_libraries(action_engine_node
  ${catkin_LIBRARIES}
  ${libraries}
  ${Boost_LIBRARIES}
)

install(TARGETS action_engine_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Parser node
add_executable(parser_node
  src/parser_node.cpp
)

add_dependencies(parser_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  temoto_action_engine_000
)

target_link_libraries(parser_node
  ${catkin_LIBRARIES}
  ${libraries}
)

install(TARGETS parser_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Graph modifier node
add_executable(graph_modifier_node
  src/graph_modifier_node.cpp
)

add_dependencies(graph_modifier_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  temoto_action_engine_000
)

target_link_libraries(graph_modifier_node
  ${catkin_LIBRARIES}
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