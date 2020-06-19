//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <Eigen/Core>

#include <rpg_common/aligned.h>

#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include "act_map/kernel_voxels.h"

namespace act_map
{
// Using Mat to ease vectorization
using Mat3XdVec = rpg::Aligned<std::vector, Eigen::Matrix3Xd>;
using Mat2XdVec = rpg::Aligned<std::vector, Eigen::Matrix2Xd>;

using Vec3dVec = rpg::Aligned<std::vector, Eigen::Vector3d>;
using Vec2dVec = rpg::Aligned<std::vector, Eigen::Vector2d>;

using V3dVecVec = std::vector<Vec3dVec>;
using V2dVecVec = std::vector<Vec2dVec>;

// order is needed for correspondences
using PointIdsVec = std::vector<std::vector<int>>;

// voxels
using OccupancyVoxel = voxblox::OccupancyVoxel;

// blocks
using OccupancyBlock = voxblox::Block<voxblox::OccupancyVoxel>;
using InfoBlock = voxblox::Block<InfoVoxel>;
using TraceBlock = voxblox::Block<TraceVoxel>;

// layers
using OccupancyLayer = voxblox::Layer<voxblox::OccupancyVoxel>;
using InfoLayer = voxblox::Layer<InfoVoxel>;
using TraceLayer = voxblox::Layer<TraceVoxel>;

// options: better put into the layer class
struct LayerOptions
{
  double vox_size = 0.2;
  size_t vox_per_side = 16u;
};

inline bool isPointOutOfRange(const Eigen::Vector3d pos,
                              const Eigen::Vector3d pt,
                              const double min_vis_dist,
                              const double max_vis_dist)
{
  double dist = (pt - pos).norm();
  return dist < min_vis_dist || dist > max_vis_dist;
}

bool isPointOcculuded(const Eigen::Vector3d pos,
                      const Eigen::Vector3d pt,
                      const OccupancyLayer& occ_layer);
}
