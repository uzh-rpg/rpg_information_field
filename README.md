# Fisher Information Field for Active Visual Localization
- [Fisher Information Field for Active Visual Localization](#fisher-information-field-for-active-visual-localization)
    - [Credit](#credit)
    - [Contained packages](#contained-packages)
    - [Install](#install)
    - [Run the code](#run-the-code)
      - [Simulation](#simulation)
      - [Real data with ROS](#real-data-with-ros)

This repository contains an implementation of the following paper

    @INPROCEEDINGS{Zhang19icra, 
      author    = {Zichao Zhang and Davide Scaramuzza}, 
      booktitle = {{IEEE} Int. Conf. Robot. Autom. ({ICRA})},
      title     = {Beyond Point Clouds: Fisher Information Field for Active Visual Localization}, 
      year      = {2019}, 
      month     = {May}
    }
    
If you use this code in academic work, please cite the above paper.

### Credit
The implementation of the information field uses the [voxel hashing](http://niessnerlab.org/papers/2013/4hashing/niessner2013hashing.pdf) algorithm implemented in [Voxblox](https://arxiv.org/abs/1611.03631).
The code we extracted from [Voxblox repository](https://github.com/ethz-asl/voxblox) is put in separate folders (`voxblox` and `voxblox_ros`), and the license is retained in `LICENSE_voxblox`. If you use the code specific to `Voxblox`, please see their repository and cite relevant publications accordingly.

### Contained packages

* `act_map`: implementation and experiments in the paper
  * `src` and `include`: implementations of core functions
  * `exp`: experiment files in cpp (there is a short message at the beginning of each file explaining the experiment)
  * `scripts`: python scripts that act as entry points for some experiments in `exp`. We use python to handle filesystems, IO and plotting to make things easier on the cpp side.
  * `maps` and `trajectories`: data used in the experiments
  * `tests`: unit tests
  * `trace`: where the output of the experiment is saved
* `act_map_ros` and `act_map_msgs`: ROS interface to `act_map`
  * `include/act_map_server.h` and `include/act_map_server_inl.h`: templated impelmentation of the ROS wrapper
  * `src/*_node.cpp`: simple wrapper of the instantiation of the `act_map_server`
  * `launch`: launch files to run the experiments with a SLAM/VIO in the loop
  * `params`: parameters for the `act_map_server`
* Other packages are dependencies.


### Install
You need to have ROS and [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) installed. After you put the package in your workspace, clone all the depencencies by

    vcs-import < rpg_information_field/dependencies.yaml
    
Then do `catkin build` in your workspace.

The code was tested on Ubuntu 16.04 and 18.04 with ROS Kinetic and Melodic respectively.


### Run the code
After sourcing the workspace via

```sh
source <path-to-the-workspace>/setup.sh
```
you can run the following experiments.
#### Simulation
For experiments in `exp` (without corresponding entry points in `scripts`), you can run them directly, for example:

    rosrun act_map exp_complexity --n_voxels=75000 --n_queries=1000000 --n_viz_ratio=0.01 --v=1
    
For experiments that have entry points in `scripts`, you can run the python script directly, for example:

    rosrun act_map exp_optim_orient.py two_walls --v=1 

The results are saved in `trace` folder.

#### Real data with ROS
One can customize the launch files in `act_map_ros/launch`. The following is a breakdown of an example: 

        <launch>
            <arg name="rviz" default="false" />  
            <include file="$(find act_map_ros)/launch/external/euroc_stereo_imu.launch" />  ---> run VIO that publishes corresponding topics

            <include file="$(find act_map_ros)/launch/external/voxblox.launch" />  ---> not necessary for the information field function, only for visualization

            <node name="act_map" pkg="act_map_ros" type="info_map_server_node" output="screen" args="-alsologtostderr --v=1" clear_params="true">
                <rosparam command="load" file="$(find act_map_ros)/params/act_map_euroc.yaml"/>  ---> parameters for the information field
                <param name="features3d_topic_name" value="/svo/features3d" /> ---> landmarks in worldframe
                <param name="body_pose_topic_name" value="/svo/Twb" />         ---> body pose in worldframe
            </node>

            <node type="rviz" name="rviz" pkg="rviz" args="-d $(find act_map_ros)/cfgs/full_exp.rviz" if="$(arg rviz)" />

        </launch>

Basically, you need to provide the following topics to `act_map_server`:
* `features3d_topic_name`: 3D points in world frame. Upon receiving this message, the server will update the information field internally.
* `body_pose_topic_name`: pose of the robot in world frame. This is used to determine whether we need allocate new blocks as the robot moves.
