cmake_minimum_required(VERSION 3.5)
project(ta_logger)

add_compile_options(-std=c++20)

find_package(temoto_action_engine REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)
target_link_libraries(${PROJECT_NAME}
  temoto_action_engine
)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.umrf.json
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.umrf.json COPYONLY
)
