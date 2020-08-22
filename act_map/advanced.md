# Advanced topics

- [Advanced topics](#advanced-topics)
  - [Implementation Notes](#implementation-notes)
  - [Adding Your Own Voxel Type](#adding-your-own-voxel-type)
    - [Step 1: Define your visibility approximation and typedef the voxel type](#step-1-define-your-visibility-approximation-and-typedef-the-voxel-type)
    - [Step 2: Add specializations and traits](#step-2-add-specializations-and-traits)
    - [Step 3: (optional but recommended) Add to the typed test](#step-3-optional-but-recommended-add-to-the-typed-test)
    - [Step 4: (needed for ROS integration) Adapt `ActMapServer`](#step-4-needed-for-ros-integration-adapt-actmapserver)
  - [Auxiliary classes](#auxiliary-classes)
    - [`DepthVoxel` and `DepthMap`](#depthvoxel-and-depthmap)
    - [`VisibilityChecker`](#visibilitychecker)
    - [`InfoCalculator`](#infocalculator)

## Implementation Notes
* In our ICRA 19 paper, we call the positional factors the *kernel*, which is not very meaningful. Therefore we change it to *positional factors* in the arXiv version. However, the renaming is not reflected fully in the code. When you see the term *kernel* in the code, it basically means the *positional factor*.

* Also, the ICRA 19 implementation of the quadratic visibility approximation (`QuadInfo\TraceVoxel`) follows a different formulation as the one in the arXiv version (`QuadPolyInfo\TraceVoxel`). They generates the same results but the later is more efficient (in terms of memory and time). We enforce the same interface for them in `positional_factor_voxel_ops.h` to keep both implementations.

## Adding Your Own Voxel Type

The voxel is implemented as a template class in `positional_factor_voxel.h`, templated on the type of the visibility approximation.
Therefore, to add your own voxel, the main effort is to implement the visibility approximation, which is derived from the `VisibilityApproximator` class.
We now go through the steps that are needed to add your own voxel.

### Step 1: Define your visibility approximation and typedef the voxel type
As in `quadpoly_vis_approximator.h`, you need to define a visibility approximation.
It should be derived from the `VisibilityApproximator` class using [CRTP](https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern) idiom and implements necessary functions.

After the visibility approximator is implemented, you can add your voxel type in `positional_factor_voxel.h` as a specialization of the `PositionalFactorVoxel`, for example:
```c++
// 6: information voxel, 6x6 output
using QuadPolyInfoVoxel = PositionalFactorVoxel<QuadPolyVisApproximator, 6>;
// 1: trace voxel, 1x1 output
using QuadPolyTraceVoxel = PositionalFactorVoxel<QuadPolyVisApproximator, 1>;
```

### Step 2: Add specializations and traits
Most of the stuff is handled by the class template per default, but you still need to manually add the following.

**Factor update and initialization**
* How should the voxel be updated?
  * See `QuadPolyInfoVoxel::updateFactorSingle` and `QuadPolyTraceVoxel::updateFactorSingle`. You basically need to copy these functions for your voxel type, depending on whether it is an information voxel or trace voxel.
* How should the visibility approximation be initialized?
  * Add a specialized initialized function for your visibility approximator type, similar to `setVisApproxFromFolderGP` and `setVisApproxQuadVisOpt`. This should depends on your visibility approximation.

**type traits**
* See the `act_map::traits` namespace in `positional_factor_voxel.h` and add the traits similar to other voxel types. These are useful to provide some flexibility at compiling time.
**Specializations required for Voxblox**

There are some specializations required by the `Voxblox` framework
* See the `act_map::voxblox` namespace in `positional_factor_voxel.h` and reproduce the same for your voxels.
* Add simple serialization functions in `src/blocks_serialization.cpp` (copy-paste).

### Step 3: (optional but recommended) Add to the typed test 
Related unit tests are templated, and thus it is very easy to add your own voxel types to these tests, namely:
* `test_pos_factor_ops.cpp`
* `test_pos_factor_voxel_serialization.cpp`
* `test_pos_factor_layer_integrator.cpp`
* `test_pos_factor_layer_evaluator.cpp`
* `test_act_map.cpp`
You simply need to add your voxel types in the `PosFactorVoxelTypes`.

### Step 4: (needed for ROS integration) Adapt `ActMapServer`
Finally, we need to take care to initialize the visibility approximators properly in the ROS node as well.
This is handled by the `typedInit()` functions in `act_map_ros/act_map_server.h` and `act_map_ros/act_map_server_inl.h` using the type traits defined above (with [SFINAE](https://eli.thegreenplace.net/2014/sfinae-and-enable_if/)).
It should be straightforward to reproduce the same for your visibility approximators.

## Auxiliary classes

### `DepthVoxel` and `DepthMap`
A `DepthVoxel` is a voxel type following the Voxblox framework. It stores a spherical depth map (indexed by discretized latitude and longitude).
A `DepthMap` is a collection of such voxels. Therefore, it has the capability of querying the depth at arbitrary poses by interpolating the depth voxels.
The `DepthMap`, in our experiment, is built by densely sampling the depth images in a simulation environment. See `exp_build_depth_map.cpp` for details.
It is convenient in that with the depth map, we can query the depth images at arbitrary poses without accessing the simulator.

### `VisibilityChecker`
The class is used to determine whether a point is visible from a position or point. It has the ability to check the visibility by:
* Whether a point is occluded: using the `DepthMap`
* Whether a viewpoint change with respect to the average view direction is too large
* Whether a point is in a distance range

These criteria are chosen by the available information at hand. For example, if we do not have the dpeth information or the view directions, only a simple distance range check is used.

### `InfoCalculator`
The `InfoCalculator` class calculates the FIM from the point clouds for a given pose. It uses the `VisibilityChecker` to get the visible points. It is not needed for the functionality of FIF and only used for the purpose of comparison.