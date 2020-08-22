#include "act_map_ros/ue_provider.h"

#include <rpg_common/main.h>

RPG_COMMON_MAIN
{
  ros::init(argc, argv, "ue_provider_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  act_map_ros::UEProvider ue_provider(nh, pnh);

  ros::spin();

  return 0;
}
