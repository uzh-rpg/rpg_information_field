#include "act_map_exp/quad_rrt.h"

#include <rpg_common/main.h>

DEFINE_string(kernel_type, "GPTrace", "kernel type");

RPG_COMMON_MAIN
{
  CHECK(!FLAGS_kernel_type.empty());
  std::string sufix("");
  if (FLAGS_kernel_type == "GPTrace" || FLAGS_kernel_type == "GPInfo")
  {
    sufix = std::string("_gp");
  }
  else if (FLAGS_kernel_type == "QTrace" || FLAGS_kernel_type == "QInfo")
  {
    sufix = std::string("_quad");
  }
  else
  {
    LOG(FATAL) << "Unknown kernel type: " << FLAGS_kernel_type;
  }

  ros::init(argc, argv, std::string("quad_traj_opt") + sufix);

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  if (FLAGS_kernel_type == "GPTrace")
  {
    act_map_exp::QuadRRT<act_map::GPTraceVoxel> planner(nh, pnh);
    ros::spin();
  }
  else if (FLAGS_kernel_type == "GPInfo")
  {
    act_map_exp::QuadRRT<act_map::GPInfoVoxel> planner(nh, pnh);
    ros::spin();
  }
  else if (FLAGS_kernel_type == "QTrace")
  {
    act_map_exp::QuadRRT<act_map::QuadTraceVoxel> planner(nh, pnh);
    ros::spin();
  }
  else if (FLAGS_kernel_type == "QInfo")
  {
    act_map_exp::QuadRRT<act_map::QuadInfoVoxel> planner(nh, pnh);
    ros::spin();
  }
  else
  {
    LOG(FATAL) << "Unknown kernel type: " << FLAGS_kernel_type;
  }

  return 0;
}
