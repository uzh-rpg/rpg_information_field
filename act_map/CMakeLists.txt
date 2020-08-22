cmake_minimum_required(VERSION 2.8.3)
project(act_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra -Wno-unused-parameter)

#add_definitions(-DACTMAP_DEBUG)

set(HEADERS
  include/act_map/act_map.h
  include/act_map/internal/operators.h
  include/act_map/positional_factor_layer_integrator.h
  include/act_map/pos_factor_layer_evaluator.h
  # different simplified visibility models
  include/act_map/sigmoid_vis_score.h
  include/act_map/quadratic_vis_score.h
  # general visibilty approximator
  include/act_map/visibility_approximator.h
  # GP and quadractic polynomial visibility approximators
  include/act_map/gp_vis_approximator.h
  include/act_map/quadpoly_vis_approximator.h
  # general positional factor voxels
  include/act_map/positional_factor_voxel_ops.h
  include/act_map/positional_factor_voxel.h
  # quadratic factors and voxels (ICRA19 version)
  include/act_map/quadratic_factor_voxel.h
  include/act_map/quadratic_factor_ops.h
  include/act_map/quad_info_factors.h
  include/act_map/quad_trace_factors.h
  # misc. utilities
  include/act_map/info_utils.h
  include/act_map/common.h
  include/act_map/voxel_traits.h
  include/act_map/optim_orient.h
  include/act_map/simulator.h
  include/act_map/conversion.h
  include/act_map/sampler.h
  include/act_map/interpolation.h
  include/act_map/functions.h
  include/act_map/voxblox_utils.h
  include/act_map/collision_check.h
  include/act_map/depth_voxel.h
  include/act_map/depth_map.h
  include/act_map/visibility_checker.h
  include/act_map/information_potential.h
  include/act_map/info_calculator.h
  )

file(GLOB_RECURSE VOXBLOX_HEADERS include/act_map/voxblox "*.h")

set(SOURCES
  src/quadratic_vis_score.cpp
  src/gp_vis_approximator.cpp
  src/quad_info_factors.cpp
  src/quad_trace_factors.cpp
  src/pos_factor_layer_evaluator.cpp
  src/info_utils.cpp
  src/sampler.cpp
  src/optim_orient.cpp
  src/simulator.cpp
  src/conversion.cpp
  src/common.cpp
  src/positional_factor_layer_integrator.cpp
  src/voxblox_utils.cpp
  src/blocks_serialization.cpp
  src/quadratic_factor_voxel.cpp
  src/positional_factor_voxel.cpp
  src/collision_check.cpp
  src/depth_voxel.cpp
  src/depth_map.cpp
  src/act_map.cpp
  src/functions.cpp
  src/visibility_checker.cpp
  src/information_potential.cpp
  src/info_calculator.cpp
  )

file(GLOB_RECURSE VOXBLOX_SOURCES src/voxblox "*.cc")


# Copied from Voxblox
# General idea: first check if we have protobuf catkin, then use that.
# Otherwise use system protobuf.
set(PROTO_DEFNS proto/voxblox/Block_act_map.proto
                proto/voxblox/Layer_act_map.proto)
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

cs_add_executable(exp_pipeline exp/exp_pipeline.cpp)
target_link_libraries(exp_pipeline ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(exp_pose_gradient exp/exp_pose_gradient.cpp)
target_link_libraries(exp_pose_gradient ${PROJECT_NAME})

cs_add_executable(exp_build_depth_map exp/exp_build_depth_map.cpp)
target_link_libraries(exp_build_depth_map ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(exp_compare_diff_maps exp/exp_compare_diff_maps.cpp)
target_link_libraries(exp_compare_diff_maps ${PROJECT_NAME})

# test
catkin_add_gtest(test_act_map tests/test_act_map.cpp)
target_link_libraries(test_act_map ${PROJECT_NAME})

catkin_add_gtest(test_vis_score tests/test_vis_score.cpp)
target_link_libraries(test_vis_score ${PROJECT_NAME})

catkin_add_gtest(test_quad_info_factor tests/test_quad_info_factor.cpp)
target_link_libraries(test_quad_info_factor ${PROJECT_NAME})

catkin_add_gtest(test_quad_trace_factor tests/test_quad_trace_factor.cpp)
target_link_libraries(test_quad_trace_factor ${PROJECT_NAME})

catkin_add_gtest(test_simulator tests/test_simulator.cpp)
target_link_libraries(test_simulator ${PROJECT_NAME})

catkin_add_gtest(test_pos_factor_layer_integrator tests/test_pos_factor_layer_integrator.cpp)
target_link_libraries(test_pos_factor_layer_integrator ${PROJECT_NAME})

catkin_add_gtest(test_misc tests/test_misc.cpp)
target_link_libraries(test_misc ${PROJECT_NAME})

catkin_add_gtest(test_voxblox tests/test_voxblox.cpp)
target_link_libraries(test_voxblox ${PROJECT_NAME})

catkin_add_gtest(test_protobuf tests/test_protobuf.cpp)
target_link_libraries(test_protobuf ${PROJECT_NAME})

catkin_add_gtest(test_pos_factor_layer_evaluator
        tests/test_pos_factor_layer_evaluator.cpp)
target_link_libraries(test_pos_factor_layer_evaluator ${PROJECT_NAME})

catkin_add_gtest(test_gp_visibility tests/test_gp_visibility.cpp)
target_link_libraries(test_gp_visibility ${PROJECT_NAME})

catkin_add_gtest(test_quadpoly_visibility tests/test_quadpoly_visibility.cpp)
target_link_libraries(test_quadpoly_visibility ${PROJECT_NAME})

catkin_add_gtest(test_gp_factor_voxel tests/test_gp_factor_voxel.cpp)
target_link_libraries(test_gp_factor_voxel ${PROJECT_NAME})

catkin_add_gtest(test_quadpoly_factor_voxel tests/test_quadpoly_factor_voxel.cpp)
target_link_libraries(test_quadpoly_factor_voxel ${PROJECT_NAME})

catkin_add_gtest(test_pos_factor_vox_ops tests/test_pos_factor_vox_ops.cpp)
target_link_libraries(test_pos_factor_vox_ops ${PROJECT_NAME})

catkin_add_gtest(test_pos_factor_voxel_serialization tests/test_pos_factor_voxel_serialization.cpp)
target_link_libraries(test_pos_factor_voxel_serialization ${PROJECT_NAME})

catkin_add_gtest(test_interpolation tests/test_interpolation.cpp)
target_link_libraries(test_interpolation ${PROJECT_NAME})

catkin_add_gtest(test_depth_voxel tests/test_depth_voxel.cpp)
target_link_libraries(test_depth_voxel ${PROJECT_NAME})

catkin_add_gtest(test_depth_map tests/test_depth_map.cpp)
target_link_libraries(test_depth_map ${PROJECT_NAME})

catkin_add_gtest(test_visibility_checker tests/test_visibility_checker.cpp)
target_link_libraries(test_visibility_checker ${PROJECT_NAME})

catkin_add_gtest(test_information_potential tests/test_information_potential.cpp)
target_link_libraries(test_information_potential ${PROJECT_NAME})

catkin_add_gtest(test_info_calculator tests/test_info_calculator.cpp)
target_link_libraries(test_info_calculator ${PROJECT_NAME})

cs_install()
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
          LIBRARIES ${ADDITIONAL_LIBRARIES})
