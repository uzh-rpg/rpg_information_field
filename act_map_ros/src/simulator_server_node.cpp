#include "act_map_ros/simulator_server.h"

#include <rpg_common/main.h>


RPG_COMMON_MAIN
{
  ros::init(argc, argv, "simulator_server_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  act_map_ros::SimulatorServer sim_server(nh, pnh);

  while(ros::ok())
  {
    bool res = sim_server.stepAndPublish();
    sim_server.sleep();
    if (!res)
    {
      std::cout << "Simulation finishes.\n";
      break;
    }
  }

  return 0;
}
