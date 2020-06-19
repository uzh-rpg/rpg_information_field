//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <voxblox/core/layer.h>

#include "act_map/common.h"
#include "act_map/conversion.h"
#include "act_map/functions.h"

namespace act_map
{
namespace utils
{
// NOTE: These functions are not necessarily the best in efficiency,
// since get indices from the layer already goes through the block_map once.
// These functions are for convenience to not change voxblox implementation.
// Check the functions in layer class first before adding something here.
template <typename VoxelT>
bool checkAllBlocks(const voxblox::Layer<VoxelT>& layer,
                    const BlockCheckerType<VoxelT>& checker)
{
  voxblox::BlockIndexList blk_idxs;
  layer.getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    if (!checker(layer.getBlockByIndex(blk_idx)))
    {
      return false;
    }
  }
  return true;
}

template <typename VoxelT>
void modifyAllBlocks(voxblox::Layer<VoxelT>* layer,
                     const BlockModifierType<VoxelT>& modifier)
{
  voxblox::BlockIndexList blk_idxs;
  layer->getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    typename voxblox::Block<VoxelT>::Ptr blk_ptr =
        layer->getBlockPtrByIndex(blk_idx);
    modifier(blk_ptr.get());
  }
}

template <typename VoxelT>
void modifyAllBlocksIf(voxblox::Layer<VoxelT>* layer,
                       const BlockModifierType<VoxelT>& modifier,
                       const VoxelCheckerType<VoxelT>& checker,
                       voxblox::BlockIndexList* modified)
{
  modified->clear();
  voxblox::BlockIndexList blk_idxs;
  layer->getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    typename voxblox::Block<VoxelT>::Ptr blk_ptr =
        layer->getBlockPtrByIndex(blk_idx);
    if (checker(*blk_ptr))
    {
      modifier(blk_ptr.get());
      if (modified)
      {
        modified->push_back(blk_idx);
      }
    }
  }
}

template <typename VoxelT, typename OutT>
void getFromAllVoxels(const voxblox::Block<VoxelT>& blk,
                      const VoxelGetterType<VoxelT, OutT>& getter,
                      const VoxelCheckerType<VoxelT>& checker,
                      rpg::Aligned<std::vector, OutT>* out)
{
  out->clear();
  for (size_t vox_idx = 0; vox_idx < blk.num_voxels(); vox_idx++)
  {
    const VoxelT& vox = blk.getVoxelByLinearIndex(vox_idx);
    if (checker(vox))
    {
      OutT v;
      getter(blk, vox_idx, &v);
      out->emplace_back(v);
    }
  }
}

template <typename VoxelT, typename OutT>
void getFromAllVoxels(const voxblox::Layer<VoxelT>& layer,
                      const VoxelGetterType<VoxelT, OutT>& getter,
                      const VoxelCheckerType<VoxelT>& checker,
                      rpg::Aligned<std::vector, OutT>* out)
{
  out->clear();

  voxblox::BlockIndexList blk_idxs;
  layer.getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    typename voxblox::Block<VoxelT>::ConstPtr blk_ptr =
        layer.getBlockPtrByIndex(blk_idx);
    for (size_t vox_idx = 0; vox_idx < blk_ptr->num_voxels(); vox_idx++)
    {
      const VoxelT& vox = blk_ptr->getVoxelByLinearIndex(vox_idx);
      if (checker(vox))
      {
        OutT v;
        getter(*blk_ptr, vox_idx, &v);
        out->emplace_back(v);
      }
    }
  }
}

template <typename VoxelT, typename OutT>
void getFromAllVoxels(const voxblox::Layer<VoxelT>& layer,
                      const VoxelGetterType<VoxelT, OutT>& getter,
                      rpg::Aligned<std::vector, OutT>* out)
{
  out->clear();

  voxblox::BlockIndexList blk_idxs;
  layer.getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    typename voxblox::Block<VoxelT>::ConstPtr blk_ptr =
        layer.getBlockPtrByIndex(blk_idx);
    for (size_t vox_idx = 0; vox_idx < blk_ptr->num_voxels(); vox_idx++)
    {
      const VoxelT& vox = blk_ptr->getVoxelByLinearIndex(vox_idx);
      OutT v;
      getter(*blk_ptr, vox_idx, &v);
      out->emplace_back(v);
    }
  }
}

template <typename VoxelT, typename OutT>
void getFromAllBlocks(const voxblox::Layer<VoxelT>& layer,
                      const BlockGetterType<VoxelT, OutT>& getter,
                      rpg::Aligned<std::vector, OutT>* out)
{
  out->clear();

  voxblox::BlockIndexList blk_idxs;
  layer.getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    OutT v;
    getter(layer, blk_idx, &v);
    out->emplace_back(v);
  }
}

template <typename VoxelT, typename OutT>
void getFromAllBlocksIf(const voxblox::Layer<VoxelT>& layer,
                        const BlockGetterType<VoxelT, OutT>& getter,
                        const BlockCheckerType<VoxelT>& checker,
                        rpg::Aligned<std::vector, OutT>* out)
{
  out->clear();

  voxblox::BlockIndexList blk_idxs;
  layer.getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    typename voxblox::Block<VoxelT>::ConstPtr blk_ptr =
        layer.getBlockPtrByIndex(blk_idx);
    if (checker(*blk_ptr))
    {
      OutT v;
      getter(layer, blk_idx, &v);
      out->emplace_back(v);
    }
  }
}

template <typename T>
bool checkAllBlocksUpdated(const voxblox::Layer<T>& layer)
{
  return checkAllBlocks<T>(layer, isBlockUpdated<T>);
}

template <typename T>
bool checkLayerUpdated(const voxblox::Layer<T>& layer)
{
  return !checkAllBlocks<T>(layer, isBlockNotUpdated<T>);
}

void getCentersOfOccupiedVoxels(const OccupancyLayer& layer,
                                const float occ_thresh,
                                Vec3dVec* points_w);

void getCentersOfOccupiedVoxels(const OccupancyLayer& layer,
                                const float occ_thresh,
                                Vec3dVec* blk_cs,
                                V3dVecVec* points_w);

void getCentersOfOccupiedVoxels(const voxblox::Block<OccupancyVoxel>& blk,
                                const float occ_thresh,
                                Vec3dVec* points_w);

template <typename T>
void getCentersOfAllVoxels(const voxblox::Layer<T>& layer, Vec3dVec* points_w)
{
  getFromAllVoxels<T, Eigen::Vector3d>(layer, getVoxelCenter<T>, points_w);
}

template <typename T>
void getCentersOfAllBlocks(const voxblox::Layer<T>& layer, Vec3dVec* points_w)
{
  getFromAllBlocks<T, Eigen::Vector3d>(layer, getBlockCenter<T>, points_w);
}

template <typename T>
void getCentersOfAllActivatedBlocks(
    const voxblox::Layer<T>& layer, Vec3dVec* points_w)
{
  getFromAllBlocksIf<T, Eigen::Vector3d>(layer, getBlockCenter<T>,
                                         isBlockActivated<T>,
                                         points_w);
}

template <typename T>
size_t getNumOfUpdatedBlocks(const voxblox::Layer<T>& layer)
{
  voxblox::BlockIndexList updated;
  layer.getAllUpdatedBlocks(&updated);
  return updated.size();
}

template <typename T>
double getUpdatedRatio(const voxblox::Layer<T>& layer)
{
  size_t n_all_blks = layer.getNumberOfAllocatedBlocks();
  if (n_all_blks == 0)
  {
    return 0.0;
  }
  return getNumOfUpdatedBlocks(layer) * 1.0 / n_all_blks;
}

template <typename T>
void clearLayerUpdate(voxblox::Layer<T>* layer)
{
  modifyAllBlocks<T>(layer, clearBlockUpdate<T>);
}

// other templated functions
template <typename T>
void subsampleVoxelIndices(const voxblox::Block<T>& blk,
                           const int samples_per_side,
                           voxblox::VoxelIndexList* vox_indicies)
{
  CHECK_GE(samples_per_side, 1);
  if (samples_per_side == 1)
  {
    int mid_idx = static_cast<int>(blk.voxels_per_side() / 2.0);
    vox_indicies->emplace_back(voxblox::VoxelIndex(mid_idx, mid_idx, mid_idx));
  }
  else
  {
    int N = static_cast<int>(blk.voxels_per_side());
    std::div_t div = std::div(N, samples_per_side);

    int step = div.quot;

    vox_indicies->clear();
    for (int ix = 0; ix < samples_per_side; ix++)
    {
      for (int iy = 0; iy < samples_per_side; iy++)
      {
        for (int iz = 0; iz < samples_per_side; iz++)
        {
          vox_indicies->emplace_back(
              voxblox::VoxelIndex(ix * step, iy * step, iz * step));
        }
      }
    }
  }
}

template <typename T>
void allocateBlocksByCoordinatesBatch(const act_map::Vec3dVec& points_w,
                                      voxblox::Layer<T>* layer,
                                      voxblox::BlockIndexList* newly_added_blks,
                                      voxblox::IndexSet* covered_blks)
{
  CHECK_NOTNULL(newly_added_blks);
  CHECK_NOTNULL(layer);
  newly_added_blks->clear();

  for (const Eigen::Vector3d& pt : points_w)
  {
    voxblox::BlockIndex blk_idx = layer->computeBlockIndexFromCoordinates(pt);
    if (!layer->getBlockPtrByIndex(blk_idx))
    {
      layer->allocateBlockPtrByIndex(blk_idx);
      newly_added_blks->push_back(blk_idx);
    }
    covered_blks->insert(blk_idx);
  }
}

// non-templated functions
// basically integrate points multiple times
void setPointsInOccupancyLayer(const Eigen::Matrix3Xd& points,
                               act_map::OccupancyLayer* occ_layer);

void getBestViewsClosed(const act_map::TraceLayer& tl,
                        const voxblox::BlockIndexList& blk_idxs,
                        const double k1,
                        const double k2,
                        const int samples_per_side,
                        act_map::Vec3dVec* vox_cs,
                        act_map::Vec3dVec* best_views,
                        voxblox::LongIndexVector* global_idxs);

void getBestViewsSample(const act_map::TraceLayer& tl,
                        const voxblox::BlockIndexList& blk_idxs,
                        const double k1,
                        const double k2,
                        const double k3,
                        const int samples_per_side,
                        act_map::Vec3dVec* vox_cs,
                        act_map::Vec3dVec* best_views,
                        std::vector<double>* values,
                        voxblox::LongIndexVector* global_idxs);

struct CollisionCheckerOptions
{
  double min_dist_thresh_ = 0.1;
  double average_dist_thresh = 0.2;
};

enum class CollisionRes
{
  kUnknown,
  kCollide,
  kFree
};

CollisionRes
doesPointCollideWithOccLayer(const OccupancyLayer& occ_layer,
                             const Eigen::Vector3d pt_w,
                             const float occ_thresh,
                             const CollisionCheckerOptions& col_ops);
template <typename T>
int maskCollidedVoxels(const OccupancyLayer& occ_layer,
                       voxblox::Block<T>* blk,
                       const float occ_thresh,
                       const CollisionCheckerOptions& col_ops)
{
  int n_masked = 0;
  for (size_t vox_i = 0; vox_i < blk->num_voxels(); vox_i++)
  {
    Eigen::Vector3d vc = blk->computeCoordinatesFromLinearIndex(vox_i);
    CollisionRes res =
        doesPointCollideWithOccLayer(occ_layer, vc, occ_thresh, col_ops);
    if (res == CollisionRes::kCollide)
    {
      blk->setVoxMasked(true, vox_i);
      n_masked++;
    }
  }
  return n_masked;
}

template <typename T>
inline size_t numOfMaskedVoxels(const voxblox::Block<T>& blk)
{
  size_t n_masked = 0;
  for (size_t i = 0; i < blk.num_voxels(); i++)
  {
    if (blk.isVoxMasked(i))
    {
      n_masked++;
    }
  }
  return n_masked;
}
}
}
