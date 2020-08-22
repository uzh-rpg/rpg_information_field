#pragma once

#include <act_map/voxblox/integrator/occupancy_integrator.h>
#include <act_map/positional_factor_layer_integrator.h>
#include <act_map/quadratic_vis_score.h>
#include <act_map/common.h>
#include <act_map/voxblox_utils.h>
#include <act_map/collision_check.h>
#include <act_map/depth_map.h>
#include <act_map/act_map.h>
#include <act_map/information_potential.h>

#include <rpg_common_ros/params_helper.h>

namespace act_map_ros
{
act_map::voxblox::OccupancyIntegrator::Config
readOccupancyIntegratorConfig(const ros::NodeHandle& pnh);

act_map::QuadVisScoreOptions
readQuadVisScoreOptions(const ros::NodeHandle& pnh,
                    const std::string& suf=std::string());

act_map::LayerOptions
readLayerOptions(const ros::NodeHandle& pnh,
                 const std::string& pre=std::string());

act_map::VisibilityCheckerOptions
readVisibilityCheckerOptions(const ros::NodeHandle& pnh);

act_map::PositionalFactorLayerIntegratorOptions
readKernelIntegratorOptions(const ros::NodeHandle& pnh);

act_map::utils::CollisionCheckerOptions
readCollisionCheckerOptions(const ros::NodeHandle& pnh);

act_map::DepthMapOptions
readDepthMapOptions(const ros::NodeHandle& pnh);

act_map::ActMapOptions
readActMapOptions(const ros::NodeHandle& pnh, const size_t n_cam=1);

act_map::InfoPotentialOptions
readInfoPotentialOptions(const ros::NodeHandle& pnh);
}
