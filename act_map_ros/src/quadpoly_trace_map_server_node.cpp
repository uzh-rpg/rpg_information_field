#include "act_map_ros/act_map_server.h"

#include <rpg_common/main.h>


RPG_COMMON_MAIN
{
  ros::init(argc, argv, "quadpoly_tracemap_server_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  act_map_ros::QuadPolyTraceMapServer actmap_server(nh, pnh);

  ros::spin();

  return 0;
}
