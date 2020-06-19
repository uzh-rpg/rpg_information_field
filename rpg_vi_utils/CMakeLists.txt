#Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
#You can contact the author at <zzhang at ifi dot uzh dot ch>
#Unauthorized copying of this file, via any medium is strictly prohibited
#Proprietary and confidential
cmake_minimum_required(VERSION 2.8.3)
project(rpg_vi_utils)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_python_setup()

set(HEADERS
  include/vi_utils/vi_jacobians.h
  include/vi_utils/vi_basic_types.h
  include/vi_utils/vi_measurements.h
  include/vi_utils/states.h
  include/vi_utils/map.h
  include/vi_utils/common_utils.h
  include/vi_utils/cam_min.h
  )
set(SOURCES
  src/vi_jacobians.cpp
  src/states.cpp
  src/map.cpp
  src/common_utils.cpp
  src/cam_min.cpp
  src/vi_measurements.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

cs_install()
cs_export()
