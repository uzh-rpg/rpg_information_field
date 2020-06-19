//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <voxblox/integrator/occupancy_integrator.h>
#include <act_map/kernel_layer_integrator.h>
#include <act_map/vis_score.h>
#include <act_map/common.h>
#include <act_map/voxblox_utils.h>

#include <rpg_common_ros/params_helper.h>

namespace act_map_ros
{
voxblox::OccupancyIntegrator::Config
readOccupancyIntegratorConfig(const ros::NodeHandle& pnh);

act_map::VisScoreOptions
readVisScoreOptions(const ros::NodeHandle& pnh,
                    const std::string& suf=std::string());

act_map::LayerOptions
readLayerOptions(const ros::NodeHandle& pnh,
                 const std::string& pre=std::string());

act_map::KernelLayerIntegratorOptions
readKernelIntegratorOptions(const ros::NodeHandle& pnh);

act_map::utils::CollisionCheckerOptions
readCollisionCheckerOptions(const ros::NodeHandle& pnh);
}
