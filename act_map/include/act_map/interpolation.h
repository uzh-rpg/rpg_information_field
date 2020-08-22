#pragma once

#include <array>
#include <Eigen/Core>

#include "act_map/common.h"

namespace act_map
{
template <typename ValT>
using InterpValues = std::array<const ValT*, 8>;
template <typename VoxT>
using InterpVoxels = std::array<const VoxT*, 8>;
using InterpVoxCenters = Eigen::Matrix<double, 3, 8>;

template <typename ValT>
void trilinearInterpolation(const Eigen::Vector3d& target,
                            const Eigen::Vector3d& corner_closest_to_origin,
                            const double grid_size,
                            const InterpValues<ValT>& values, ValT* out)
{
  // we do not check whether the target is within the grid
  const double dx = target.x() - corner_closest_to_origin.x();
  const double dy = target.y() - corner_closest_to_origin.y();
  const double dz = target.z() - corner_closest_to_origin.z();
  const double cdx = grid_size - dx;
  const double cdy = grid_size - dy;
  const double cdz = grid_size - dz;
  const double denom = std::pow(grid_size, 3);

  rpg::Vector8 ws;
  ws(0) = (cdx * cdy * cdz) / denom;
  ws(1) = (dx * cdy * cdz) / denom;
  ws(2) = (cdx * dy * cdz) / denom;
  ws(3) = (dx * dy * cdz) / denom;
  ws(4) = (cdx * cdy * dz) / denom;
  ws(5) = (dx * cdy * dz) / denom;
  ws(6) = (cdx * dy * dz) / denom;
  ws(7) = (dx * dy * dz) / denom;

  //  CHECK_EQ(ws.sum(), 1.0);

  (*out) = (*values[0]) * ws(0);
  for (size_t i = 1; i < 8; i++)
  {
    (*out) += (*values[i]) * ws(i);
  }
}

enum class QueryVoxelsRes
{
  // reserved
  kUndefine = 0,
  // the queried point is outside all the allocated blocks
  // there is not much we can do in this case
  kPointOutside = 1,
  // the query point is very near,: simply using the nearest voxel
  kNearest = 2,
  // there is no adjacent blocks to get all the voxels needed
  kNoAdjacentBlocks = 3,
  // success: we have all surrounding voxels
  kSurround = 4
};

template <typename VoxelT>
QueryVoxelsRes getSurroundVoxels(const voxblox::Layer<VoxelT>& layer,
                                 const Eigen::Vector3d& pt,
                                 InterpVoxels<VoxelT>* sur_voxels,
                                 InterpVoxCenters* sur_vox_centers,
                                 const double nearest_thresh = 1e-4)
{
  voxblox::BlockIndex blk_idx = layer.computeBlockIndexFromCoordinates(pt);
  typename voxblox::Block<VoxelT>::ConstPtr blk_ptr =
      layer.getBlockPtrByIndex(blk_idx);
  // we do not find it in any block
  if (!blk_ptr)
  {
    LOG(WARNING) << "Get voxel failed: Cannot find block.";
    return QueryVoxelsRes::kPointOutside;
  }
  voxblox::VoxelIndex vox_idx = blk_ptr->computeVoxelIndexFromCoordinates(pt);
  if (!blk_ptr->isValidVoxelIndex(vox_idx))
  {
    LOG(WARNING) << "Get voxel failed: Cannot find voxel in block:"
                 << vox_idx.transpose();
    return QueryVoxelsRes::kPointOutside;
  }
  const voxblox::IndexElement nvox_ps =
      static_cast<voxblox::IndexElement>(blk_ptr->voxels_per_side());

  // try to find the surrounding voxels in all blocks
  // Get indices: Adapted from
  // https://github.com/ethz-asl/voxblox/blob/master/voxblox/include/voxblox/interpolator/interpolator_inl.h
  // step 1: get the indices
  // get lowest voxel and block index
  // if the voxel is very near to the center, just use one
  Eigen::Vector3d offset_from_c =
      pt - blk_ptr->computeCoordinatesFromVoxelIndex(vox_idx);
  if ((offset_from_c.cwiseAbs().array() < nearest_thresh).all())
  {
    sur_voxels->at(0) = &(blk_ptr->getVoxelByVoxelIndex(vox_idx));
    return QueryVoxelsRes::kNearest;
  }
  voxblox::VoxelIndex lowest_vox_idx = vox_idx;
  voxblox::VoxelIndex lowest_blk_idx = blk_idx;
  for (int i = 0; i < offset_from_c.rows(); ++i)
  {
    if (offset_from_c(i) < 0)
    {
      lowest_vox_idx(i)--;
      // move to the next block
      if (lowest_vox_idx(i) < 0)
      {
        lowest_blk_idx(i)--;
        lowest_vox_idx(i) += nvox_ps;
      }
    }
  }
  if (!layer.getBlockPtrByIndex(lowest_blk_idx))
  {
    return QueryVoxelsRes::kNoAdjacentBlocks;
  }

  // get surrounding voxel and block indices
  Eigen::Matrix<voxblox::IndexElement, 3, 8> sur_vox_idxs;
  // clang-format off
  // follow the same convention as the linear index in block
  sur_vox_idxs <<
    0, 1, 0, 1, 0, 1, 0, 1,
    0, 0, 1, 1, 0, 0, 1, 1,
    0, 0, 0, 0, 1, 1, 1, 1;
  // clang-format on
  sur_vox_idxs.colwise() += lowest_vox_idx;
  Eigen::Matrix<voxblox::IndexElement, 3, 8> sur_blk_idxs;
  for (int vox_i = 0; vox_i < 8; vox_i++)
  {
    sur_blk_idxs.col(vox_i) = lowest_blk_idx;
    // check whether we need to shift blocks
    // since we add from the lowest, we only need to check the upper bound
    if ((sur_vox_idxs.col(vox_i).array() >= nvox_ps).any())
    {
      for (int coord_i = 0; coord_i < 3; coord_i++)
      {
        if (sur_vox_idxs(coord_i, vox_i) >= nvox_ps)
        {
          sur_blk_idxs(coord_i, vox_i)++;
          sur_vox_idxs(coord_i, vox_i) -= nvox_ps;
        }
      }
    }
    // check
    if (!layer.getBlockPtrByIndex(sur_blk_idxs.col(vox_i)))
    {
      return QueryVoxelsRes::kNoAdjacentBlocks;
    }
  }

  // step 2: get all the voxels
  for (int i = 0; i < 8; i++)
  {
    const voxblox::Block<VoxelT>& blk =
        layer.getBlockByIndex(sur_blk_idxs.col(i));
    sur_voxels->at(i) = &(blk.getVoxelByVoxelIndex(sur_vox_idxs.col(i)));
    sur_vox_centers->col(i) =
        blk.computeCoordinatesFromVoxelIndex(sur_vox_idxs.col(i));
  }
  return QueryVoxelsRes::kSurround;

  // should not reach here
  return QueryVoxelsRes::kUndefine;
}
}  // namespace act_map
