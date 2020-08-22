#include "act_map_ros/depth_map_server.h"

#include <rpg_common/main.h>

RPG_COMMON_MAIN
{
  ros::init(argc, argv, "depth_map_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  act_map_ros::DepthMapServer dm_server(nh, pnh);

  ros::spin();

  return 0;
}
