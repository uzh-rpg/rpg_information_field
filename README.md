# Fisher Information Field: an Efficient and Differentiable Map for Perception-aware Planning

[![Fisher Information Field for Perception-aware Planning](./doc/ytb_thumbnail.png)](https://youtu.be/auQCljSmDKI)

> **For an earlier version of this work that was published at ICRA 19, please checkout the `icra19` branch.**

> [21-06-2020] The code and preprint will be available in the next weeks.

> [09-08-2020] Added paper and video. The code will be released soon.

This repository contains an implementation of the **Fisher Information Field** (FIF for short), a map representation designed for perception-aware planning.
The core function of the map is to evaluate the visual localization quality at a given 6 DoF pose in a known environment (consisting of 3D landmarks to localize against).
It can be used with different motion planning algorithms (e.g., RRT-star, trajectory optimization) to take localization quality into consideration, in addition to common planning objectives (collision free, low dynamic cost, start and end states etc).
The main advantage of FIF is its efficiency at the planning time: it is 1~2 order-of-magnitude faster than using the landmarks directly in our experiments.

Please read the following preprint and watch the video for details:

* **Fisher Information Field: an Efficient and Differentiable Map for Perception-aware Planning**. arXiv preprint, 2020. [PDF](http://rpg.ifi.uzh.ch/docs/Arxiv20_Zhang_FIF.pdf) [Video](https://youtu.be/auQCljSmDKI)

#### A quick example of trajectory optimization
* 4 DoF piecewise polynomial trajectory for quadrotors. 10 seconds duration, 5 segments.
* Optimization slowed down for visualization.
<!-- ![traj_opt_mid](doc/traj_opt_mid.gif) -->
![traj_opt_bottom](doc/traj_opt_bottom.gif)

For illustration purpose, the trajectory optimization is done in two steps with the following costs: 1) collision + dynamic; 2) collision + dynamic + localization quality.
The optimized trajectory of the first step is shown in red, and the second step in green.
The yellow points are the landmarks for localization, and the colored overlay is ESDF built with [voxblox](https://github.com/ethz-asl/voxblox).
The green trajectory, which in addition considers the localization quality, prefers to look at landmark-rich regions and move close to landmarks.

More experimental results can be found in the [paper](http://rpg.ifi.uzh.ch/docs/Arxiv20_Zhang_FIF.pdf) and the [video](https://youtu.be/auQCljSmDKI).