- [General Design](#general-design)
  - [Data structure](#data-structure)
- [Important classes/concepts in FIF](#important-classesconcepts-in-fif)
  - [Visibility Approximators](#visibility-approximators)
  - [Voxel](#voxel)
  - [Layer, Integrator and Evaluator](#layer-integrator-and-evaluator)
  - [The `ActMap` class](#the-actmap-class)
## General Design
A Fisher Information Field (FIF) is a collection of voxels in 3D space, organized using the voxel hashing method as a layer.
Each voxel stores the *positional factors* (corresponding to the voxel's center) from the full Fisher Information Matrix (FIM), which can later be used to recover the full FIM for arbitrary poses located at or within (by interpolation) this voxel.

The basic workflow of using FIF is:
1. **Build**: given known landmarks, allocate voxels within a given range and compute the positional factors for each voxel from the known landmarks.
2. **Query**: for arbitrary 6 DoF poses within the range, the approximated FIM is recovered in constant time from the positional factors.

The FIM (and its determinant, trace, etc.) can then be used to quantify the predicted localization quality at the poses, which can used, e.g., in perception-aware planning.

### Data structure
We use the same data structure as Voxblox: voxels are organized in `Block`s, which are further organized as `Layer`s. Voxel hashing is used to efficiently access the voxels according to 3D coordinates.
The functionality thus depends on the type of the voxel: different types of voxels are built differently and queried differently, as described below.

In addition, we (ab)use the `OccupancyVoxel` type to also store the landmark positions. We further modify the voxel to contain the average view direction of the landmark to allow filtering the visible landmarks better.

## Important classes/concepts in FIF

Most of the implemented classes have corresponding unit tests in `tests`, which are probably the best starting point to understand how each class works.
In the following, we describe the purpose and usage of several core concepts.

### Visibility Approximators
Separating the positional factors from the full FIM is a key step in our method. This is made possible by using functions of certain forms to approximate the actual FoV constraint.
Two types of such visibility approximators are implemented:
* `quadpoly_vis_approximator.h`: using quadratic polynomial
* `gp_vis_approximator.h`: using Gaussian Process

For details, please refer to [visibility_approximations.md](./visibility_approximations.md).

### Voxel

A voxel is the smallest unit in our map and stores the positional factors. Building and querying the map ultimately boil down to the operations on the voxels. Different types of voxels are implemented (corresponding to the aforementioned visibility approximators) and share the same interface.

**Voxel types**

Currently, the following voxel types are implemented:
* `GPInfo/TraceVoxel`: using Gaussian Process for visibility approximation
* `QuadPolyInfo/TraceVoxel`: using quadratic polynomial for visibility approximation.
* `QuadInfo/TraceVoxel`: the legacy implementation using the quadratic approximation according to our ICRA19 paper - it generates the same result as the `QuadPolyInfo/TraceVoxel` (see `test_quadpoly_factor_voxel.cpp`) but takes slightly more memory and is slower.

The `*TraceVoxel` is a simplified voxel that only encodes the trace of the FIM, but is more efficient in terms of memory and time.

**Interface**

All voxel types follow the same interface for manipulation and query, as defined in `positional_factor_voxel_ops.h`. Specifically,
* **Building/manipulating the factors for a given landmark**
  * `assignToFactorVoxel`: compute the positional factor from a given point
  * `addToFactorVoxel`: add new information to the positional factors
  * `substractFromFactorVoxel`: subtract information from the positional factors 
* **Querying FIM related metrics**
  * `getInfoMetricAtRotationFromPositionalFactor`: get the determinant/trace/smallest eigen value from the FIM at a given rotation at the voxel.

See `test_pos_factor_vox_ops.cpp` for these functions in action.

**Initialize Visibility Approximators**

For each voxel, the visibility approximator is stored as a static member.
Therefore, before using each type of voxel, we need to initialize its visibility approximator:

```c++
GPInfoVoxel::setVisApproxFromFolderGP(folder);
GPTraceVoxel::setVisApproxFromFolderGP(folder);
QuadPolyInfoVoxel::setVisApproxQuadVisOpt(quad_vis_opt);
QuadPolyTraceVoxel::setVisApproxQuadVisOpt(quad_vis_opt);
```
please see [`visibility_approximations.md`](./visibility_approximations.md) for how to provide the parameters/information for the initialization of each visibility approximation type.

### Layer, Integrator and Evaluator
In practice, we need the FIF to provide information in a 3D range.
To cover a 3D space, we need to (continuously) allocate voxels, which are organized as a `Layer`. This is the same from the Voxblox implementation.
Similarly, to build and query the layer, we use the following classes
* `PositionalFactorLayerIntegrator`: it basically wrap around the operations on voxels mentioned above.
* `PositionalFactorLayerEvaluator`: query the information at arbitrary poses by interpolating nearby voxels or simply querying the nearest voxel

### The `ActMap` class
Finally, we put everything in a `ActMap` class that wrap around all the functionalities.
In addition to the FIF, it also includes the following:

* `VisibilityChecker`: a class for checking whether a point can be observed from a position using various criteria (see [advanced.md](./advanced.md)).
* A `OccupancyLayer` for storing the known landmarks. In addition to the landmark positions, it also stores the average view directions for filtering visible points (used in the `VisibilityChecker`)
* A `InfoCalculator` for calculating the FIM from the point cloud directly (for comparison purpose).

Therefore, a typical (batch processing) workflow is to set the necessary information (landmarks, view directions, depth map, etc) in the `ActMap` class and then build the FIF accordingly.
Then we can query the FIM efficiently using the FIF.
Alternatively, the `ActMap` class has an interface to build the FIF incrementally as new landmarks becomes available.
You can find the examples of both in `test_act_map.cpp`
