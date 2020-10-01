# unrealcv_bridge
This package contains code to interact with UnrealEngine via UnrealCV, including
* python scripts to extract information/send commands to UnrealEngine
* C++ client and renderer adpated from esim.

The code in this package assumes that you have a UnrealEngine game running (either within the editor or as a packaged application) with UnrealCV plugin.

Before tying the instructions below, download the environment we used for our experiments [here](http://rpg.ifi.uzh.ch/datasets/FIF/warehouse_bin.zip). Extract and start the simulation:

```sh
# under LinuxNoEditor
./IsaacSimProject.sh -WINDOWED
```

## Python Scripts

#### `record_pose_ue.py`
This script record the poses of the camera via UnrealCV and save them in the formats of UnrealEngine and transformation matrices (i.e., `Twc` using the convention [here](http://paulfurgale.info/news/2014/6/9/representing-robot-pose-the-good-the-bad-and-the-ugly)).

Run

```sh
rosrun unrealcv_bridge record_pose_ue.py --out_dir ./ --Hz 10 --timeout 30
```

and use your mouse and keyboard to control the camera move in the UnrealEngine simulation. The script will record the pose of the camera for `30` seconds at `10` Hz.

Then you will have two files for the poses `poses_xyzpyr_ue.txt` and `poses_Twc.txt`, which we can use to render images and depths from.

#### `render_from_poses.py`
Render images via UnrealCV from UE poses (e.g., from the previous scripts). 

```sh
rosrun unrealcv_bridge render_from_poses.py ./poses_xyzpyr_ue.txt --unreal_ini <unrealcv_ini> --save_dir ./ue_rendering --vis_depth --save_depth
```

The `<unrealcv_ini>` should point to `IsaacSimProject/Binaries/Linux/unrealcv.ini` (or `Engine/Binaries/Linux/unrealcv.ini` if you are running the simulation from the editor), and to change the camera intrinsics, you also need to modify this file **before** starting the editor/simulation.

The output will be a folder containing:

* images
* poses
* camera intrinsics
These files are organized to be used with COLMAP to reconstruct a 3D model (e.g., see [our COLMAP scripts](https://github.com/uzh-rpg/colmap_utils)).

## C++ Classes

There are two C++ class implemented:

* `UnrealCvClient`: handle connection and work with UnrealCV commands
* `UnrealCVRender`: interfaces for commonly used functionalities (render images/depth, get camera poses, etc). This can be used, e.g., to simulate a depth camera (see `ue_provider.h` in `act_map_ros`).

Please refer to the unit tests under `tests` for using these classes.

