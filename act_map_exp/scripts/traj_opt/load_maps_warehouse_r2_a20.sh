#!/usr/bin/env bash

rosservice call /quad_traj_opt_gp_info/clear_act_map_layers
rosservice call /quad_traj_opt_gp_info/load_act_map_layers  "file_path: '/home/zichao/sources/FIF_ws/src/rpg_information_field/act_map_exp/exp_data/warehouse_FIF/gp_info_r2_a20'"

rosservice call /quad_traj_opt_gp_trace/clear_act_map_layers
rosservice call /quad_traj_opt_gp_trace/load_act_map_layers  "file_path: '/home/zichao/sources/FIF_ws/src/rpg_information_field/act_map_exp/exp_data/warehouse_FIF/gp_trace_r2_a20'"

rosservice call /quad_traj_opt_quadratic_info/clear_act_map_layers
rosservice call /quad_traj_opt_quadratic_info/load_act_map_layers  "file_path: '/home/zichao/sources/FIF_ws/src/rpg_information_field/act_map_exp/exp_data/warehouse_FIF/quad_info_r2_a20'"

rosservice call /quad_traj_opt_quadratic_trace/clear_act_map_layers
rosservice call /quad_traj_opt_quadratic_trace/load_act_map_layers  "file_path: '/home/zichao/sources/FIF_ws/src/rpg_information_field/act_map_exp/exp_data/warehouse_FIF/quad_trace_r2_a20'"
