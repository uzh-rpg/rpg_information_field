#include "act_map_ros/params_reader.h"

namespace act_map_ros
{
act_map::voxblox::OccupancyIntegrator::Config
readOccupancyIntegratorConfig(const ros::NodeHandle& pnh)
{
  act_map::voxblox::OccupancyIntegrator::Config cfg;
  cfg.probability_hit = rpg_ros::param(pnh, "occ_prob_hit", 0.65f);
  cfg.probability_miss = rpg_ros::param(pnh, "occ_prob_miss", 0.4f);
  cfg.threshold_min = rpg_ros::param(pnh, "occ_thresh_min", 0.12f);
  cfg.threshold_max = rpg_ros::param(pnh, "occ_thresh_max", 0.97f);
  cfg.threshold_occupancy = rpg_ros::param(pnh, "occ_thresh_occupancy", 0.7f);
  cfg.min_ray_length_m = rpg_ros::param(pnh, "occ_min_ray_m", 0.1);
  cfg.max_ray_length_m = rpg_ros::param(pnh, "occ_max_ray_m", 5.0);

  return cfg;
}

act_map::QuadVisScoreOptions readQuadVisScoreOptions(const ros::NodeHandle& pnh,
                                             const std::string& suf)
{
  act_map::QuadVisScoreOptions options;
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

act_map::VisibilityCheckerOptions
readVisibilityCheckerOptions(const ros::NodeHandle& pnh)
{
  act_map::VisibilityCheckerOptions options;
  options.min_dist = rpg_ros::param(pnh, "vis_check_min_dist", 0.7);
  options.max_dist = rpg_ros::param(pnh, "vis_check_max_dist", 10);
  options.use_view_filtering =
      rpg_ros::param<bool>(pnh, "vis_check_use_view_filter", false);
  const double max_ang_deg =
      rpg_ros::param(pnh, "vis_check_max_ang_deg", 180.0);
  options.min_view_angle_cos = std::cos(max_ang_deg / 180.0 * M_PI);

  options.use_depth_layer_ =
      rpg_ros::param(pnh, "vis_check_use_depth_layer", false);
  options.depth_layer_proto_fn_ = rpg_ros::param<std::string>(
      pnh, "vis_check_depth_layer_proto_fn", std::string(""));
  options.dm_options_ = readDepthMapOptions(pnh);

  options.use_camera_ =
      rpg_ros::param<bool>(pnh, "vis_check_use_camera", false);
  options.cam_dir_ =
      rpg_ros::param<std::string>(pnh, "vis_check_cam_dir", std::string(""));

  return options;
}

act_map::PositionalFactorLayerIntegratorOptions
readKernelIntegratorOptions(const ros::NodeHandle& pnh)
{
  act_map::PositionalFactorLayerIntegratorOptions options;
  options.occ_thresh_ = rpg_ros::param(pnh, "ker_inte_occ_thresh", 0.7f);
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

act_map::DepthMapOptions readDepthMapOptions(const ros::NodeHandle& pnh)
{
  act_map::DepthMapOptions dm_ops;
  dm_ops.depth_layer_opts_ = readLayerOptions(pnh, "depth_layer_");
  dm_ops.depth_voxel_step_deg_ =
      rpg_ros::param(pnh, "depth_vox_deg_step", -1.0);
  return dm_ops;
}

act_map::ActMapOptions readActMapOptions(const ros::NodeHandle& pnh,
                                         const size_t n_cam)
{
  act_map::ActMapOptions map_options;
  map_options.use_collision_checker_ =
      rpg_ros::param(pnh, "use_collision_checker", false);
  map_options.occ_layer_options_ = readLayerOptions(pnh, "occ_");
  map_options.occ_integrator_options_ = readOccupancyIntegratorConfig(pnh);
  map_options.vis_options_.resize(n_cam);
  for (size_t i = 0; i < n_cam; i++)
  {
    map_options.vis_options_[i] = readQuadVisScoreOptions(pnh, std::to_string(i));
  }
  map_options.pos_factor_layer_options_ = readLayerOptions(pnh, "ker_");
  map_options.pos_fac_integrator_options_ = readKernelIntegratorOptions(pnh);
  map_options.col_ops_ = readCollisionCheckerOptions(pnh);

  map_options.vis_checker_options_ = readVisibilityCheckerOptions(pnh);

  return map_options;
}

act_map::InfoPotentialOptions
readInfoPotentialOptions(const ros::NodeHandle& pnh)
{
  act_map::InfoPotentialOptions ip_options;
  ip_options.min_depth_m_ =
      rpg_ros::param<double>(pnh, "info_pot_min_depth_m", 1.0);
  ip_options.max_depth_m_ =
      rpg_ros::param<double>(pnh, "info_pot_max_depth_m", 3.0);
  ip_options.px_noise_sigma_px_ =
      rpg_ros::param<double>(pnh, "info_pot_px_noise_sig", 1.0);
  ip_options.n_random_landmarks_ =
      rpg_ros::param<int>(pnh, "info_pot_n_random_lm", 50);
  ip_options.fov_deg_ = rpg_ros::param<double>(pnh, "info_pot_fov_deg", 90.0);
  ip_options.val_at_zero_ =
      rpg_ros::param<double>(pnh, "info_pot_val_at_zero", 100.0);
  ip_options.use_negative_info_ =
      rpg_ros::param<bool>(pnh, "info_pot_use_negative_info", false);
  ip_options.average_over_n_ =
      rpg_ros::param<int>(pnh, "info_pot_average_over_n", 100);
  return ip_options;
}

}  // namespace act_map_ros
