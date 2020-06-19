//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <gflags/gflags.h>
#include <glog/logging.h>

// Does vanilla initialization of glog and gflags.
// Usage:
// RPG_COMMON_MAIN
// {
//   // same body as usually, with argc and argv
// }
// You will need to add gflags_catkin and glog_catkin
// as dependency to package.xml .

#define RPG_COMMON_MAIN \
int rpg_common_main(int argc, char** argv); \
int main(int argc, char** argv) \
{ \
  google::InitGoogleLogging(argv[0]); \
  google::ParseCommandLineFlags(&argc, &argv, true); \
  google::InstallFailureSignalHandler(); \
  FLAGS_alsologtostderr = true; \
  FLAGS_colorlogtostderr = true; \
  return rpg_common_main(argc, argv); \
} \
int rpg_common_main(int argc, char** argv)
