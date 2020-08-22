# ActMap
This package contains the core implementation of the Fisher Information Field. The map is designed for perception-aware motion planning (active perception) and thus the name.

- [ActMap](#actmap)
  - [Package Structure](#package-structure)
  - [Experiments](#experiments)
    - [Exp. 1: Optimal orientations](#exp-1-optimal-orientations)
    - [Exp. 2: Compare different maps](#exp-2-compare-different-maps)
  - [Additional Information about the Implementation](#additional-information-about-the-implementation)

## Package Structure
Different folders in this packages are:

* `include` and `src`: source code of core functionality
* `exp`: some experiments (including the ones in the paper) using the FIF
* `tests`: unit tests
* `proto`: protobuf stuff for serialization

There are some accompanying code/documentation

* `scripts`: mainly scripts for analyzing experimental results
* `notebooks`: python notebooks for demonstrating/experimenting some ideas

And some files/folders related to experiments are
* `params`:
  * `fov_approximator_gp`: the parameters for different Gaussian process visibility approximations (see [visibility_approximation.md](./visibility_approximations.md)).
  * `cameras`: simple pinhole camera models
  * some other parameter files for experiments
* `maps` and `trajectories`: 3D points and 6 DoF trajectories for experiments
* `trace`: place for conveniently storing experiments/tests output (the content in the folder is gitingored by default) 

If you would like to know in details about the implementation, you can have a look at the [additional information](#additional-information-about-the-implementation) at the end of the document. You can also try the experiments below first.

## Experiments

> Remember to source the workspace first before running the experiments.

There are several experiments under `exp` folder. For some experiments, there are corresponding python scripts under `scripts` acting as an entry point or for results analysis/plotting. 
Typically, the experiments write results to subdirectories in the `trace` folder.

These experiments are provided to illustrate and validate the properties of the map representations in simplified simulation environment (i.e., consider only the geometric layout of the landmarks/3D points).
Photorealistic experiments can be found in package `act_map_exp`.

In the source code of each experiment, there is a brief description at the beginning of the main function describing the purpose of the experiments, as well as the parameters that are needed.
We next describe two experiments that are used in our paper in details.

### Exp. 1: Optimal orientations
This experiment aims to calculate and visualize the optimal orientation at different locations. This can be used as a visual check of the correctness.

Example usage:

```
rosrun act_map exp_optim_orient.py two_walls --v=1
```
where `two_walls` is the map name to be found under `act_map/maps`.

This command will calculate the optimal orientations with different maps and point clouds. The results and plots will be written in a sub-folder in `trace/exp_optim_orient`.

The following options can be added:
* `--paper_plot`: generate the plots used in the paper
* `--animated`: automatically rotate the plots in 3D for better visual check

### Exp. 2: Compare different maps

This experiment aims to analyze the properties of different map representations at the same time:

* Memory and query time
* Fisher information matrix difference w.r.t. the actual FIM using the point clouds
* Difference in terms of optimal orientations

> Note that the results may differ slightly from the ones in the paper, since the landmarks are generated randomly.

**Step 1**: run the experiments:
```
rosrun act_map exp_compare_diff_maps -v=1 -gp_list sim_gp_list.txt
```
where `sim_gp_list.txt` is to be found under `params` folder and specifies the visibility approximation (and the corresponding names to save the results). The results are saved in `trace/exp_compare_diff_maps` folder.

With the above command, the experiments generates random 3D landmarks as map. You can also specify a specific map, see `exp_compare_diff_maps.cpp` for details.

**Step 2**: in the result folder `trace/exp_compare_diff_maps`, run the analysis:
```
rosrun act_map exp_compare_diff_maps.py --res_dir ./ --analyze_config ../../params/analyze_sim_maps_config.yaml
```
The analysis configuration specifies which results will be included in which analysis. The analysis results are written to the `analysis_results` in the result folder.


## Additional Information about the Implementation
There are several topics you can read about the code:
* Read [design_and_concepts.md](./design_and_concepts.md) to better understand the implementation.
* Read [visibility_approximations.md](./visibility_approximations.md) for details of different visibility approximations implemented in the code.
* Read [advanced.md](./advanced.md) for:
  * How to add your own voxel in our framework
  * Description of some auxiliary classes
  * Some implementation notes



