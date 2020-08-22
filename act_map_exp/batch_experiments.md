- [Pre-requisite](#pre-requisite)
- [Workflow](#workflow)
  - [Step 1: Start the Planners of different configurations](#step-1-start-the-planners-of-different-configurations)
  - [Step 2: Get Voxblox Layers](#step-2-get-voxblox-layers)
  - [Step 3 Get FIF Layers](#step-3-get-fif-layers)
  - [Step 4: Run All the Planners](#step-4-run-all-the-planners)
  - [Step 5: Analyzing the Results](#step-5-analyzing-the-results)

# Pre-requisite
We assume that all the maps required for planning are ready under `exp_data`. You can either download the pre-built maps or create them by yourself (see [README](./README.md)).

It is recommended to go through the [instructions](./run_a_single_exp.md) to run a single experiment first to get familiar with the process.

These instructions aim to automate the execution of multiple motion planning experiments with different FIF voxels to compare their performance.


> :warning: Note that the automatic experiment runner ([step 4](#step-4-run-all-the-planners)) also evaluates the poses on the planner motion automatically -- this relies on the simulator and COLMAP. Therefore, the instructions in Step 4 and onwards cannot be used as it is until the simulator and COLMAP scripts are added.

# Workflow

## Step 1: Start the Planners of different configurations
There are four `launch_*.sh` under `scripts/rrt` and `scripts/traj_opt` for creating typical planner nodes with unique names. Start them in four different terminals, e.g, for the trajectory optimization:

> :pushpin: Running many planners in parallel requires a significant amount of memory (since there are many copies of the maps). We ran our experiments on a workstation with 32 GB memory. You can reduce the number of experiments as needed.

```sh
# in act_map_exp
# terminal 1
./scripts/traj_opt/launch_quad_opt_gp_info_warehouse.sh
# terminal 2
./scripts/traj_opt/launch_quad_opt_gp_trace_warehouse.sh
# terminal 3
./scripts/traj_opt/launch_quad_opt_quadratic_info_warehouse.sh
# terminal 4
./scripts/traj_opt/launch_quad_opt_quadratic_info_warehouse.sh
```

## Step 2: Get Voxblox Layers
***First*** launch a voxlblox node
```
roslaunch act_map_ros voxblox_warehouse.launch
```
***Then*** load the layers (built from previous steps)
```
rosservice call /voxblox_node/load_map "file_path: '<abs_path_to_vxblx_file>'"
```
***Last*** publish the layers to all the planners
```bash
# in act_map_exp/scripts
./ask_for_esdf.sh
```
You should see corresponding output in the terminals where the planners are launched.

## Step 3 Get FIF Layers
There are scripts under `scripts/rrt` and `scripts/traj_opt` to load the FIF layers into all the planners for convenience. For example, for the trajectory optimization experiment, run 
```bash
# in scripts/traj_opt
./load_maps_warehouse_r1_a30.sh
```
to load the maps built before for the landmarks `r1_a30` into all the planners. Again, you should see corresponding output in the terminals where the planners are launched.


## Step 4: Run All the Planners

> :warning: This section is under development: the rendering, registration and evaluation in `run_planner_exp.py` cannot be used at this moment.

`scripts/run_planner_exp.py` provides an interface to run different configurations (map representations, information metrics) conveniently. 
It accepts a configuration file that specifies
* `M` base planning objectives (start, end, different costs and their weights)
* `N` variations (planner node to call, information metric to use, whether to compute the information from point clouds)

Then, it runs `M * N` experiments and the evaluation in sequence and saves the results in the specified output directory. Example configuration files can be found in:

* `params/quad_rrt/warehouse/warehouse_all.yaml` for the RRT experiments
* `params/quad_traj_opt/warehouse/warehouse_all.yaml` for the trajectory optimization experiments

Therefore you could run all the experiments in one command like:

```sh
./scripts/run_planner_exp.py <config_file> --top_outdir <outdir> --colmap_script_dir <colmap_scripts> --base_model <colmap_base_model>
```
We also pass in `<colmap_scripts>` and `<colmap_base_model>` to let the script automatically render images from the sampled poses of the planned motion, localize against `<colmap_base_model>` and calculate the localization errors.

## Step 5: Analyzing the Results

> :warning: This section is under development: include more details when the simulator and COLMAP scripts are added.

The following scripts are provided to analyze the experimental results:
* `scripts/analyze_traj_opt.py`
* `scripts/analyze_rrt.py`