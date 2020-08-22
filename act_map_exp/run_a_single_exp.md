

- [Pre-requisite](#pre-requisite)
- [The workflow](#the-workflow)
  - [Step 0: Start `roscore` and `rviz`](#step-0-start-roscore-and-rviz)
  - [Step 1: Launch the planner and get the maps ready](#step-1-launch-the-planner-and-get-the-maps-ready)
    - [2.1 Launch the planners](#21-launch-the-planners)
    - [2.2 Get Voxblox Layers](#22-get-voxblox-layers)
    - [2.3 Get FIF Layers](#23-get-fif-layers)
  - [Step 2: Run the planners and evaluate](#step-2-run-the-planners-and-evaluate)
    - [3.1 Run the planners](#31-run-the-planners)
    - [3.2 Evaluation](#32-evaluation)


# Pre-requisite
We assume that all the maps required for planning are ready under `exp_data`. You can either download the pre-built maps or create them by yourself (see [README](./README.md)).

# The workflow

## Step 0: Start `roscore` and `rviz`
Start `roscore` and rviz. In rviz, you can select the configurations under `act_map_exp/rviz_cfgs`:
* `quad_rrt_warehouse.rviz`: for the RRT* experiment
* `quad_opt_warehouse.rviz`: for the trajectory optimization experiment

## Step 1: Launch the planner and get the maps ready

### 2.1 Launch the planners
The planners are in the following launch files, with different map types
* RRT
  * `launch/quad_rrt_warehouse_quadratic.launch`
  * `launch/quad_rrt_warehouse_gp.launch`
* Trajectory optimization
  * `launch/quad_traj_opt_warehouse_quadratic.launch`
  * `launch/quad_traj_opt_warehouse_gp.launch`

One can configure whether full FIM or trace should be used, as well as the node name. For example:
```
roslaunch act_map_exp quad_rrt_warehouse_gp.launch node_name:=quad_rrt kernel_type:=GPInfo
```
will create a information field with full FIM and GP visibility approximation. Available options for `kernel_type` are
* `GPInfo` and `GPTrace`: GP visibility approximations
* `QInfo` and `QTrace`: Quadratic visibility approximations (ICRA10 implementation)

See the launch files and `quad_rrt_node.cpp`/`quad_traj_opt_node.cpp` for details.

### 2.2 Get Voxblox Layers
For ESDF, we launch a `voxblox` node and pass the voxblox layers per topic.
> Alternatively, we can also directly load the ESDF/TSDF layers in the planner.

***First*** launch the node
```
roslaunch act_map_ros voxblox_warehouse.launch
```
***Then*** load the layers
```
rosservice call /voxblox_node/load_map "file_path: '<abs_path_to_vxblx_file>'"
rosservice call /voxblox_node/generate_mesh
```
***Last*** publish the layers
```bash
# in act_map_exp/scripts
./ask_for_esdf.sh
```
You should see corresponding output in the terminals where the planners are launched.

### 2.3 Get FIF Layers

One can load the saved map with 
```bash
rosservice call /<node_name>/load_act_map_layers "file_path: '<abs_path>'"
```
where the '<abs_path>' should points to a folder contains both the FIF layer and occupancy layer serializations (e.g., one of the folders under `exp_data/warehouse_FIF`).

> Alternatively, we can also pass the FIF via topic, as in the ESDF case.

Note that the used map here should be consistent with the `kernel_type` selected above, otherwise an warning will be raised in the process of de-serialization.


## Step 2: Run the planners and evaluate

### 3.1 Run the planners
Since the planners (in our case, `RRT*` and trajectory optimization) inherits the `PlannerBase` class, they share the same interfaces (e.g., services) for planning tasks. Therefore, the workflow for the RRT* and trajectory optimization experiments are the same, as described below.

With a planner where the Voxblox layers and information field are loaded, we first need to set the planner state (e.g., start, goal, cost. etc) and then start planning.

***First*** set the planner state
```bash
rosservice call /<planner_node>/set_planner_state "config: '<abs_path_to_config>'"
```
where `<abs_path_to_config>` is a yaml file containing all the parameters. An example can be found under `act_map_exp/params/quad_rrt/warehouse/warehouse_rrt_trial.yaml` and `act_map_exp/params/quad_traj_opt/warehouse_traj_opt_trial.yaml` for the RRT* and trajectory optimization experiment respectively.

***Then*** run the planner
```bash
rosservice call /quad_rrt_gp_info/plan_vis_save
```
The results (e.g., sampled poses on the planned motion, time consumed) are saved in the folder specified in the configuration file,  and you can see the visualization in RViZ.

At this point, you can call the `set_planner_state` service to run the experiment again without having to setup the maps from scratch.

### 3.2 Evaluation

> This section is under development.

The planning results (time, planned motion) are saved in the `save_abs_dir` as specified in the parameter file. We do not have a script for analyzing the result of a single experiment. You can have a look at the `scripts/analyze_rrt.py` and `scripts/analyze_traj_opt.py`, which are designed for analyzing many experiment results altogether, to see how to parse the experiment results.

For the localization accuracy, one needs to be able to render images from the poses of the planned motion and localize the images against the SfM model. This part will be added soon.

