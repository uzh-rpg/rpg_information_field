#Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
#You can contact the author at <zzhang at ifi dot uzh dot ch>
#Unauthorized copying of this file, via any medium is strictly prohibited
#Proprietary and confidential
cmake_minimum_required(VERSION 2.8.3)
project(act_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

set(HEADERS
  include/act_map/act_map.h
  include/act_map/vis_score.h
  include/act_map/info_kernels.h
  include/act_map/trace_kernels.h
  include/act_map/info_utils.h
  include/act_map/optim_orient.h
  include/act_map/sampler.h
  include/act_map/internal/operators.h
  include/act_map/simulator.h
  include/act_map/conversion.h
  include/act_map/common.h
  include/act_map/kernel_voxels.h
  include/act_map/kernel_ops.h
  include/act_map/kernel_layer_integrator.h
  include/act_map/voxblox_utils.h
  include/act_map/functions.h
  include/act_map/kernel_voxel_ops.h
  include/act_map/layer_evaluator.h
  )

file(GLOB_RECURSE VOXBLOX_HEADERS include/voxblox "*.h")

set(SOURCES
  src/vis_score.cpp
  src/info_kernels.cpp
  src/trace_kernels.cpp
  src/info_utils.cpp
  src/sampler.cpp
  src/optim_orient.cpp
  src/simulator.cpp
  src/conversion.cpp
  src/common.cpp
  src/kernel_layer_integrator.cpp
  src/voxblox_utils.cpp
  src/kernel_blocks_serialization.cpp
  )

file(GLOB_RECURSE VOXBLOX_SOURCES src/voxblox "*.cc")


# Copied from Voxblox
# General idea: first check if we have protobuf catkin, then use that.
# Otherwise use system protobuf.
set(PROTO_DEFNS proto/voxblox/Block.proto
                proto/voxblox/Layer.proto)
set(ADDITIONAL_LIBRARIES "")

find_package(protobuf_catkin QUIET)
if (protobuf_catkin_FOUND)
    message(STATUS "Using protobuf_catkin")
    list(APPEND catkin_INCLUDE_DIRS ${protobuf_catkin_INCLUDE_DIRS})
    list(APPEND catkin_LIBRARIES ${protobuf_catkin_LIBRARIES})
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
    set(ADDITIONAL_LIBRARIES ${protobuf_catkin_LIBRARIES})
else()
  message(STATUS "Using system protobuf")
  find_package(Protobuf REQUIRED)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
  set(ADDITIONAL_LIBRARIES ${PROTOBUF_LIBRARY})
endif()

cs_add_library(${PROJECT_NAME}_proto
  ${PROTO_SRCS}
)
target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

cs_add_library(${PROJECT_NAME}
  ${SOURCES} ${HEADERS}
  ${VOXBLOX_HEADERS} ${VOXBLOX_SOURCES})
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES}
  ${OMPL_LIBRARIES})

# experiments
cs_add_executable(exp_invariant_info exp/exp_invariant_info.cpp)
target_link_libraries(exp_invariant_info ${PROJECT_NAME})

cs_add_executable(exp_approx_decomp exp/exp_approx_decomp.cpp)
target_link_libraries(exp_approx_decomp ${PROJECT_NAME})

cs_add_executable(exp_complexity exp/exp_complexity.cpp)
target_link_libraries(exp_complexity ${PROJECT_NAME})

cs_add_executable(exp_optim_orient exp/exp_optim_orient.cpp)
target_link_libraries(exp_optim_orient ${PROJECT_NAME})

cs_add_executable(exp_comp_optim_orient exp/exp_comp_optim_orient.cpp)
target_link_libraries(exp_comp_optim_orient ${PROJECT_NAME})

cs_add_executable(exp_pipeline exp/exp_pipeline.cpp)
target_link_libraries(exp_pipeline ${PROJECT_NAME} ${OpenCV_LIBRARIES})

# test
catkin_add_gtest(test_act_map tests/test_act_map.cpp)
target_link_libraries(test_act_map ${PROJECT_NAME})

catkin_add_gtest(test_vis_score tests/test_vis_score.cpp)
target_link_libraries(test_vis_score ${PROJECT_NAME})

catkin_add_gtest(test_info_kernels tests/test_info_kernels.cpp)
target_link_libraries(test_info_kernels ${PROJECT_NAME})

catkin_add_gtest(test_trace_kernels tests/test_trace_kernels.cpp)
target_link_libraries(test_trace_kernels ${PROJECT_NAME})

catkin_add_gtest(test_simulator tests/test_simulator.cpp)
target_link_libraries(test_simulator ${PROJECT_NAME})

catkin_add_gtest(test_kernel_layer_integrator tests/test_kernel_layer_integrator.cpp)
target_link_libraries(test_kernel_layer_integrator ${PROJECT_NAME})

catkin_add_gtest(test_misc tests/test_misc.cpp)
target_link_libraries(test_misc ${PROJECT_NAME})

catkin_add_gtest(test_voxblox tests/test_voxblox.cpp)
target_link_libraries(test_voxblox ${PROJECT_NAME})

catkin_add_gtest(test_protobuf tests/test_protobuf.cpp)
target_link_libraries(test_protobuf ${PROJECT_NAME})

catkin_add_gtest(test_kernel_layer_evaluator
  tests/test_kernel_layer_evaluator.cpp)
target_link_libraries(test_kernel_layer_evaluator ${PROJECT_NAME})

cs_install()
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
          LIBRARIES ${ADDITIONAL_LIBRARIES})
