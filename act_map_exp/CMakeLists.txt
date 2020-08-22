cmake_minimum_required(VERSION 2.8.3)
project(act_map_exp)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_definitions(-std=c++11 -Wall -Wextra -Wno-unused-parameter)

set(HEADERS
  include/act_map_exp/planner_base.h
  include/act_map_exp/quad_traj_opt.h
  include/act_map_exp/quad_traj_opt_impl.h
  include/act_map_exp/quad_rrt.h
  include/act_map_exp/quad_rrt_impl.h
  include/act_map_exp/exp_utils.h
  include/act_map_exp/numdiff-jacobian-tester.h
  include/act_map_exp/potential_cost.h
  include/act_map_exp/viz_utils.h
  include/act_map_exp/poly_traj_helper.h
  include/act_map_exp/ompl_utils.h
  )

set(SOURCES
  src/planner_base.cpp
  src/quad_traj_opt.cpp
  src/quad_rrt.cpp
  src/exp_utils.cpp
  src/potential_cost.cpp
  src/viz_utils.cpp
  src/poly_traj_helper.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} yaml-cpp ${OMPL_LIBRARIES} ${CERES_LIBRARIES})

# nodes
cs_add_executable(planner_base_node src/planner_base_node.cpp)
target_link_libraries(planner_base_node ${PROJECT_NAME})

cs_add_executable(quad_traj_opt_node src/quad_traj_opt_node.cpp)
target_link_libraries(quad_traj_opt_node ${PROJECT_NAME})

cs_add_executable(quad_rrt_node src/quad_rrt_node.cpp)
target_link_libraries(quad_rrt_node ${PROJECT_NAME})

# tests
catkin_add_gtest(test_exp_utils tests/test_exp_utils.cpp)
target_link_libraries(test_exp_utils ${PROJECT_NAME})

catkin_add_gtest(test_potential_cost tests/test_potential_cost.cpp)
target_link_libraries(test_potential_cost ${PROJECT_NAME})

catkin_add_gtest(test_poly_traj_helper tests/test_poly_traj_helper.cpp)
target_link_libraries(test_poly_traj_helper ${PROJECT_NAME})

cs_install()
cs_export()
