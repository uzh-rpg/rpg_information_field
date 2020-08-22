# Visibility Approximations
The visibility essentially means the FoV constraint. We use the term `visibility` to be consistent with the code.
The general visibility formulation in our [paper](http://rpg.ifi.uzh.ch/docs/Arxiv20_Zhang_FIF.pdf) is implemented as a base class `VisibilityApproximator`, and two specific visibility approximations are implemented as child classes:
* `QuadPolyVisibilityApproximator`: in `quadpoly_vis_approximator.h`
* `GPVisibilityApproximator`: in `gp_vis_approximator.h`

## `QuadPolyVisibilityApproximator`
This model uses a quadratic polynomial of the cosine of the angle between the bearing vector and the optical axis to approximate the actual visibility. To determine the coefficients of the polynomial, several control points are specified. The computation of the coefficients is implemented in an auxiliary class `QuadraticVisScore`.

The actual visibility class `QuadPolyVisibilityApproximator` uses the `QuadraticVisScore` to initialize the coefficients internally. For example, we can initialize the visibility approximator in `QuadPolyInfoVoxel` as

```c++
// option for the quadratic approximation
QuadVisScoreOptions opts;
// what is the half FoV in radian
opts.half_fov_rad = M_PI_4;
// what should be the visibility value at the FoV boundary
opts.boundary_value = 0.9;
// what should be the ratio of the visibility value at the boundary w.r.t. at the center
// -> in this case, the visibility at the center of the FoV will be 1
opts.boundary_to_mid_ratio = 0.9;
QuadPolyInfoVoxel::setVisApproxQuadVisOpt(opts);
```
The members in `QuadVisScoreOptions` can be changed for different quadratic functions.

> Legacy implementation: `quad_info_factors.h` and `quad_trace_factors.h` use the same quadratic approximation but follow the formulation in our ICRA 19 paper. They generate the same results but are less efficient.

## `GPVisibilityApproximator`
This model uses a non-parametric model (Gaussian Process) for visibility approximation. The Gaussian process tries to approximate a sigmoid function implemented in `SigmoidVisScore`. Basically, the GP model regresses the visibility value (of the sigmoid function) from the orientation of the optical axis of the camera.

Specifying a GP approximator, however, is more complicated. The following parameters are needed:
* what are the sampled camera optical axis directions?
* what are the parameters for the squared exponential kernel?
* what are the parameters for the sigmoid function (e.g, its logistic growth rate)?

To simplify the setup of `GPVisibilityApproximator`, we provide a `load()` function to read these settings from a folder, for example

```c++
// gp_vis_folder contains the parameters for GP visibility approximation
GPInfoVoxel::setVisApproxFromFolderGP(gp_vis_folder);
```

### Generate GP Visibility Configurations

To generate such configurations, we use the notebook `notebooks/gp_fit_diff_landmarks.ipynb`. The parameters for the GP visibility and where it is saved can be adjusted at the beginning of the notebook.
There is also a notebook `notebooks/eval_gp_fov_approx.ipynb` to visually check how good a visibility approximation is.
You can start working with the notebooks using
```sh
# under act_map/notebooks
jupyter-notebook
```

Under `params/fov_approximator_gp`, we provide several ready-to-use approximation configurations, in the name convention `fov{M}_fs{N}_lm{Q}_k{K}` where
* `M`: the degree of the **half** FoV
* `N`: the number of sampled camera optical axis directions
* `Q`: the number of randomly selected landmarks to train the GP
* `K`: (the logistic growth rate) the value specifying the steepness of the sigmoid function

You can find that many experiments/unit tests use these visibility approximations by simply using one of the folders.
