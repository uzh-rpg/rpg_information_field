#pragma once

#include <Eigen/Core>

#include <type_traits>

#include <rpg_common/aligned.h>
#include <rpg_common/eigen_type.h>

#include "act_map/voxblox/core/layer.h"
#include "act_map/voxblox/core/voxel.h"
#include "act_map/quadratic_factor_voxel.h"
#include "act_map/positional_factor_voxel.h"
#include "act_map/depth_voxel.h"

namespace act_map
{
// Using Mat to ease vectorization
using Mat3XdVec = rpg::Aligned<std::vector, Eigen::Matrix3Xd>;
using Mat2XdVec = rpg::Aligned<std::vector, Eigen::Matrix2Xd>;

using Vec3dVec = rpg::Aligned<std::vector, Eigen::Vector3d>;
using Vec2dVec = rpg::Aligned<std::vector, Eigen::Vector2d>;

using V3dVecVec = std::vector<Vec3dVec>;
using V2dVecVec = std::vector<Vec2dVec>;

using RotMatVec = rpg::Aligned<std::vector, Eigen::Matrix3d>;
using PoseInfoMatVec = rpg::Aligned<std::vector, rpg::Matrix66>;

// order is needed for correspondences
using PointIdsVec = std::vector<std::vector<int>>;

// voxels
using OccupancyVoxel = voxblox::OccupancyVoxel;

// blocks
using OccupancyBlock = voxblox::Block<voxblox::OccupancyVoxel>;
using QuadInfoBlock = voxblox::Block<QuadInfoVoxel>;
using QuadTraceBlock = voxblox::Block<QuadTraceVoxel>;
using GPInfoBlock = voxblox::Block<GPInfoVoxel>;
using GPTraceBlock = voxblox::Block<GPTraceVoxel>;
// depth
using DepthBlock = voxblox::Block<DepthVoxel>;

// layers
using OccupancyLayer = voxblox::Layer<voxblox::OccupancyVoxel>;
using QuadInfoLayer = voxblox::Layer<QuadInfoVoxel>;
using QuadTraceLayer = voxblox::Layer<QuadTraceVoxel>;
using GPInfoLayer = voxblox::Layer<GPInfoVoxel>;
using GPTraceLayer = voxblox::Layer<GPTraceVoxel>;
// depth
using DepthLayer = voxblox::Layer<DepthVoxel>;

// options: better put into the layer class
struct LayerOptions
{
  double vox_size = 0.2;
  size_t vox_per_side = 16u;
};

template <typename T>
std::string getVoxelType()
{
  return voxblox::getVoxelType<T>();
}
namespace voxel_types = voxblox::voxel_types;


template <typename VoxT>
inline typename std::enable_if<traits::is_info_vox<VoxT>::value, InfoMetricTypeVec>::type
supportedInfoMetricTypesVoxT()
{
  return InfoMetricTypeVec {InfoMetricType::kDet, InfoMetricType::kMinEig, InfoMetricType::kTrace};
}

template <typename VoxT>
inline typename std::enable_if<traits::is_info_vox<VoxT>::value, bool>::type
supportFullFIMVoxT()
{
  return true;
}

template <typename VoxT>
inline typename std::enable_if<traits::is_trace_vox<VoxT>::value, InfoMetricTypeVec>::type
supportedInfoMetricTypesVoxT()
{
  return InfoMetricTypeVec {InfoMetricType::kTrace};
}

template <typename VoxT>
inline typename std::enable_if<traits::is_trace_vox<VoxT>::value, bool>::type
supportFullFIMVoxT()
{
  return false;
}

}  // namespace act_map
