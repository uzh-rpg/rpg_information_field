#pragma once

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

// main.h equivalent for unit tests, this will automatically run tests.
// You can use the body to do diverse default initializations, e.g. ROS:
//
// RPG_COMMON_TEST_MAIN
// {
//   ros::init(argc, argv, "remode");
// }

#define RPG_COMMON_TEST_MAIN \
  void rpg_common_test_main(int argc, char** argv); \
int main(int argc, char** argv) { \
  ::testing::InitGoogleTest(&argc, argv); \
  google::InitGoogleLogging(argv[0]); \
  google::InstallFailureSignalHandler(); \
  ::testing::FLAGS_gtest_death_test_style = "threadsafe"; \
  FLAGS_alsologtostderr = true; \
  FLAGS_colorlogtostderr = true; \
\
  rpg_common_test_main(argc, argv); \
\
  return RUN_ALL_TESTS(); \
} \
void rpg_common_test_main(int argc, char** argv)
