#Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
#You can contact the author at <zzhang at ifi dot uzh dot ch>
#Unauthorized copying of this file, via any medium is strictly prohibited
#Proprietary and confidential
cmake_minimum_required(VERSION 2.8.3)
project(act_map_msgs)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

##########
# EXPORT #
##########

cs_install()
cs_export()
