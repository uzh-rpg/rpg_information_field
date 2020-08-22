# unrealcv_bridge

This package contains code to interact with UnrealEngine, including
* python scripts to extract information/send commands to UnrealEngine
* A C++ client adpated from esim.

## Workflows

### record_pose_ue.py
Record the poses of the camera via UnrealCV, saved in UE and COLMAP formats.

Steps:
1. Start UnrealEngine editor with UnrealCV enabled.
2. run the script, and the poses (both UE format and colmap format) will be written in the output directory.

### render_from_poses.py
Render images via UnrealCV from UE poses (e.g., from the previous scripts). The output will be a folder containing:
* images
* poses
* camera intrinsics

These files are organized to be used with COLMAP to reconstruct a 3D model.

### reconstruct a SfM model and get the landmarks
We can do the following based on the output of the above steps:
* reconstruct from poses
* extract the output 3D points from COLMAP

