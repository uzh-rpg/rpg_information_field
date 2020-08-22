cmake_minimum_required(VERSION 2.8.3)
project(unrealcv_bridge)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# python
catkin_python_setup()

# cpp
find_package(OpenCV REQUIRED)
add_definitions(-std=c++11 -Wall -Wextra)
set(HEADERS
  include/unrealcv_bridge/unrealcv_client.hpp
  include/unrealcv_bridge/unrealcv_render.h
  include/unrealcv_bridge/ue_utils.hpp
  include/unrealcv_bridge/esim_utils.hpp)
set(SRC
  src/unrealcv_client.cpp
  src/unrealcv_render.cpp
  src/ue_utils.cpp)

cs_add_library(${PROJECT_NAME} ${SRC} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES}
  -lpthread -lboost_filesystem -lboost_system)

cs_add_executable(test_unrealcv_bridge tests/test_unrealcv_client.cpp)
target_link_libraries(test_unrealcv_bridge ${PROJECT_NAME})

# gtest
#catkin_add_gtest(test_unrealcv_render tests/test_unrealcv_render.cpp)
#target_link_libraries(test_unrealcv_render ${PROJECT_NAME})

catkin_add_gtest(test_pose_conversion tests/test_pose_conversion.cpp)
target_link_libraries(test_pose_conversion ${PROJECT_NAME})

cs_install()
cs_export()
