project(act_map_ros)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra -Wno-unused-parameter)

set(HEADERS 
  include/act_map_ros/act_map_server.h
  include/act_map_ros/act_map_server_inl.h
  include/act_map_ros/simulator_server.h
  include/act_map_ros/conversion_ros.h
  include/act_map_ros/common_ros.h
  include/act_map_ros/params_reader.h
  include/act_map_ros/depth_map_server.h
  include/act_map_ros/ue_provider.h
  )
file(GLOB_RECURSE VOXBLOX_ROS_HEADERS include/act_map_ros/voxblox_ros "*.h")

set(SOURCES 
  src/act_map_server.cpp
  src/simulator_server.cpp
  src/conversion_ros.cpp
  src/common_ros.cpp
  src/params_reader.cpp
  src/depth_map_server.cpp
  src/ue_provider.cpp
  )
file(GLOB_RECURSE VOXBLOX_ROS_SOURCES src/voxblox_ros "*.cc")

cs_add_library(${PROJECT_NAME}
  ${SOURCES} ${HEADERS}
  ${VOXBLOX_ROS_SOURCES} ${VOXBLOX_ROS_HEADERS}
  )

# nodes
cs_add_executable(trace_map_server_node src/trace_map_server_node.cpp)
target_link_libraries(trace_map_server_node ${PROJECT_NAME})

cs_add_executable(info_map_server_node src/info_map_server_node.cpp)
target_link_libraries(info_map_server_node ${PROJECT_NAME})

cs_add_executable(gp_info_map_server_node src/gp_info_map_server_node.cpp)
target_link_libraries(gp_info_map_server_node ${PROJECT_NAME})

cs_add_executable(gp_trace_map_server_node src/gp_trace_map_server_node.cpp)
target_link_libraries(gp_trace_map_server_node ${PROJECT_NAME})

cs_add_executable(quadpoly_info_map_server_node src/quadpoly_info_map_server_node.cpp)
target_link_libraries(quadpoly_info_map_server_node ${PROJECT_NAME})

cs_add_executable(quadpoly_trace_map_server_node src/quadpoly_trace_map_server_node.cpp)
target_link_libraries(quadpoly_trace_map_server_node ${PROJECT_NAME})

cs_add_executable(simulator_server_node src/simulator_server_node.cpp)
target_link_libraries(simulator_server_node ${PROJECT_NAME})

cs_add_executable(depth_map_server_node src/depth_map_server_node.cpp)
target_link_libraries(depth_map_server_node ${PROJECT_NAME})

cs_add_executable(ue_provider_node src/ue_provider_node.cpp)
target_link_libraries(ue_provider_node ${PROJECT_NAME})

cs_install()
cs_export()
