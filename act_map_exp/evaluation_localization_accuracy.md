# Evaluating the Localization Accuracy

Given the sampled poses from a planned motion, the goal of this instruction is to render images from theses poses, localize the images against a SfM model, and calculate the localization error. 

You should have the following ready before proceeding

* `stamped_Twc_ue.txt`: stamped poses in UnrealEngine format, generated from the provided expriments
* the simulation environment running (see the [main readme](../README.md))
* the SfM model to localize against (download to `exp_data` according to the [README](./README.md))
* COLMAP and [the scripts to work with COLMAP](https://github.com/uzh-rpg/colmap_utils) (assumed to be located at `<colmap_utils_path>`)



## Step 1 Render images from the poses

Run 

```sh
# where the stamped_Twc_ue.txt is
rosrun unrealcv_bridge render_from_poses.py ./stamped_Twc_ue.txt --save_dir ./rendering --save_sleep_sec 0.1 --unrealcv_ini <unrealcv_ini in your simulator>
```

See the documentation in `unrealcv_bridge` for interacting with the simulation (e.g., where the configuration is).

This command will generate a `rendering` folder contained the rendered images, poses and intrinsics.

## Step 2 Localize images against the SfM model

First, generate necessary text files for localization using COLMAP:

```sh
<colmap_utils_path>/generate_img_rel_path.py --base_dir <path_to_base_model>/images --img_dir ./rendering/images --img_nm_to_cam_list ./rendering/img_nm_to_colmap_cam.txt
```

Then, run

```sh
<colmap_utils_path>/register_images_to_model.py <path_to_base_model>  --reg_name trial --reg_list_fn ./rendering/images/rel_img_path.txt  --img_nm_to_colmap_cam_list ./rendering/images/rel_img_nm_to_cam_list.txt
```

This command will localize the images with respect to the model specified. In the folder of the base model, you should have something that looks like

```
├── 20200930-145547_trial_database.db  --> COLMAP database
├── 20200930-145547_trial_sparse       --> COLMAP SfM model
```



## Step 3 Calculate the localization error

Run

```sh
<colmap_utils_path>/calculate_pose_errors.py --reg_model_dir <path_to_base_model>/20200930-145547_trial_sparse  --reg_img_name_to_colmap_Tcw ./rendering/img_name_to_colmap_Tcw.txt --reg_img_dir ./rendering/images --output_path ./
```

will calculate the pose errors and save them in `pose_errors.txt` like

```
# image_name trans_e_m rot_e_deg
00000.png 8.06430544332839 26.450579124751982
00001.png 0.8710143379797417 1.188787309065598
00002.png 0.0780438867647918 0.11073231405955443
....
```

## Further Documentation

The above steps uses the scripts in `unrealcv_bridge` and `colmap_utils` ([link](https://github.com/uzh-rpg/colmap_utils)) to interact with UnrealEngine simulation and COLMAP for rendering and localization. Please refer to the details in these packages if you would like to customize the above steps. 

Moreover, to compare the localization accuracy of (many) different planning settings conveniently, we embed the above evaluation process in the script `run_planner_exp.py` for [running and comparing different settings in a batch](./batch_experiments.md).