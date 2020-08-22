#include "act_map_exp/planner_base.h"

#include <rpg_common/main.h>


RPG_COMMON_MAIN
{
  ros::init(argc, argv, "planner_base");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  act_map_exp::PlannerBase<act_map::GPTraceVoxel> planner(nh, pnh);

  ros::spin();

  return 0;
}
