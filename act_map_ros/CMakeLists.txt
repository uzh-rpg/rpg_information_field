#Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
#You can contact the author at <zzhang at ifi dot uzh dot ch>
#Unauthorized copying of this file, via any medium is strictly prohibited
#Proprietary and confidential
project(act_map_ros)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

set(HEADERS 
  include/act_map_ros/act_map_server.h
  include/act_map_ros/act_map_server_inl.h
  include/act_map_ros/simulator_server.h
  include/act_map_ros/conversion_ros.h
  include/act_map_ros/common_ros.h
  include/act_map_ros/params_reader.h
  )
file(GLOB_RECURSE VOXBLOX_ROS_HEADERS include/voxblox "*.h")

set(SOURCES 
#  src/act_map_server.cpp
  src/simulator_server.cpp
  src/conversion_ros.cpp
  src/common_ros.cpp
  src/params_reader.cpp
  )
file(GLOB_RECURSE VOXBLOX_ROS_SOURCES src/voxblox "*.cc")

cs_add_library(${PROJECT_NAME}
  ${SOURCES} ${HEADERS}
  ${VOXBLOX_ROS_SOURCES} ${VOXBLOX_ROS_HEADERS}
  )

# nodes
cs_add_executable(trace_map_server_node src/trace_map_server_node.cpp)
target_link_libraries(trace_map_server_node ${PROJECT_NAME})

cs_add_executable(info_map_server_node src/info_map_server_node.cpp)
target_link_libraries(info_map_server_node ${PROJECT_NAME})

cs_add_executable(simulator_server_node src/simulator_server_node.cpp)
target_link_libraries(simulator_server_node ${PROJECT_NAME})

cs_install()
cs_export()
