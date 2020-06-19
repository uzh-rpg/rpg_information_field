//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map_ros/params_reader.h"

namespace act_map_ros
{
voxblox::OccupancyIntegrator::Config
readOccupancyIntegratorConfig(const ros::NodeHandle& pnh)
{
  voxblox::OccupancyIntegrator::Config cfg;
  cfg.probability_hit = rpg_ros::param(pnh, "occ_prob_hit", 0.65f);
  cfg.probability_miss = rpg_ros::param(pnh, "occ_prob_miss", 0.4f);
  cfg.threshold_min = rpg_ros::param(pnh, "occ_thresh_min", 0.12f);
  cfg.threshold_max = rpg_ros::param(pnh, "occ_thresh_max", 0.97f);
  cfg.threshold_occupancy = rpg_ros::param(pnh, "occ_thresh_occupancy", 0.7f);
  cfg.min_ray_length_m = rpg_ros::param(pnh, "occ_min_ray_m", 0.1);
  cfg.max_ray_length_m = rpg_ros::param(pnh, "occ_max_ray_m", 5.0);

  return cfg;
}

act_map::VisScoreOptions readVisScoreOptions(const ros::NodeHandle& pnh,
                                             const std::string& suf)
{
  act_map::VisScoreOptions options;
  options.half_fov_rad = rpg_ros::param(pnh, "vis_half_fov_rad" + suf, M_PI_4);
  options.boundary_to_mid_ratio =
      rpg_ros::param(pnh, "vis_boundary_ratio" + suf, 0.5);
  options.boundary_value = rpg_ros::param(pnh, "vis_boundary_val" + suf, 0.5);
  return options;
}

act_map::LayerOptions readLayerOptions(const ros::NodeHandle& pnh,
                                       const std::string& pref)
{
  act_map::LayerOptions layer_options;
  layer_options.vox_size = rpg_ros::param(pnh, pref + "vox_size", 0.2);
  layer_options.vox_per_side =
      static_cast<size_t>(rpg_ros::param(pnh, pref + "vox_per_side", 16));
  return layer_options;
}

act_map::KernelLayerIntegratorOptions
readKernelIntegratorOptions(const ros::NodeHandle& pnh)
{
  act_map::KernelLayerIntegratorOptions options;
  options.occ_thresh_ = rpg_ros::param(pnh, "ker_inte_occ_thresh", 0.7f);
  options.min_vis_dist = rpg_ros::param(pnh, "ker_inte_min_vis_dist", 0.1f);
  options.max_vis_dist = rpg_ros::param(pnh, "ker_inte_max_vis_dist", 10.0f);
  return options;
}

act_map::utils::CollisionCheckerOptions
readCollisionCheckerOptions(const ros::NodeHandle& pnh)
{
  act_map::utils::CollisionCheckerOptions options;
  options.min_dist_thresh_ = rpg_ros::param(pnh, "occ_min_dist_thresh", 0.1);
  options.average_dist_thresh =
      rpg_ros::param(pnh, "occ_average_dist_thresh", 0.2);
  return options;
}
}
