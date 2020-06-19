#!/usr/bin/env bash
rosservice call /voxblox_node/save_map "file_path: '/home/zichao/sources/act_map_ws/src/rpg_vi_analyzer/act_map_ros/trace/esdf.vxblx'"
rostopic pub /act_map_cmd std_msgs/String s
