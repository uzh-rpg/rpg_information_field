#pragma once

#include "act_map/voxblox/core/layer.h"
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
bool checkAllBlocks(const voxblox::Layer<VoxelT>& layer, const BlockCheckerType<VoxelT>& checker)
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
void modifyAllBlocks(voxblox::Layer<VoxelT>* layer, const BlockModifierType<VoxelT>& modifier)
{
  voxblox::BlockIndexList blk_idxs;
  layer->getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    typename voxblox::Block<VoxelT>::Ptr blk_ptr = layer->getBlockPtrByIndex(blk_idx);
    modifier(blk_ptr.get());
  }
}

template <typename VoxelT>
void modifyAllVoxels(voxblox::Layer<VoxelT>* layer, const VoxelModifierType<VoxelT>& modifier)
{
  voxblox::BlockIndexList blk_idxs;
  layer->getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    typename voxblox::Block<VoxelT>::Ptr blk_ptr = layer->getBlockPtrByIndex(blk_idx);
    for (size_t vox_i = 0; vox_i < blk_ptr->num_voxels; vox_i++)
    {
      VoxelT& vox = blk_ptr->getVoxelByLinearIndex(vox_i);
      modifier(&vox);
    }
  }
}

template <typename VoxelT>
void modifyAllBlocksIf(voxblox::Layer<VoxelT>* layer, const BlockModifierType<VoxelT>& modifier,
                       const VoxelCheckerType<VoxelT>& checker, voxblox::BlockIndexList* modified)
{
  modified->clear();
  voxblox::BlockIndexList blk_idxs;
  layer->getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    typename voxblox::Block<VoxelT>::Ptr blk_ptr = layer->getBlockPtrByIndex(blk_idx);
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
                      const VoxelCheckerType<VoxelT>& checker, rpg::Aligned<std::vector, OutT>* out)
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
void getFromAllVoxels(const voxblox::Block<VoxelT>& blk,
                      const VoxelGetterType<VoxelT, OutT>& getter,
                      rpg::Aligned<std::vector, OutT>* out)
{
  out->clear();
  for (size_t vox_idx = 0; vox_idx < blk.num_voxels(); vox_idx++)
  {
    OutT v;
    getter(blk, vox_idx, &v);
    out->emplace_back(v);
  }
}

template <typename VoxelT>
size_t countAllVoxelsIf(const voxblox::Layer<VoxelT>& layer,
                        const VoxelCheckerType<VoxelT>& checker)
{
  size_t cnt = 0;
  voxblox::BlockIndexList blk_idxs;
  layer.getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    typename voxblox::Block<VoxelT>::ConstPtr blk_ptr = layer.getBlockPtrByIndex(blk_idx);
    for (size_t vox_idx = 0; vox_idx < blk_ptr->num_voxels(); vox_idx++)
    {
      const VoxelT& vox = blk_ptr->getVoxelByLinearIndex(vox_idx);
      if (checker(vox))
      {
        cnt++;
      }
    }
  }

  return cnt;
}

template <typename VoxelT, typename OutT>
void getFromAllVoxels(const voxblox::Layer<VoxelT>& layer,
                      const VoxelGetterType<VoxelT, OutT>& getter,
                      const VoxelCheckerType<VoxelT>& checker, rpg::Aligned<std::vector, OutT>* out)
{
  out->clear();

  voxblox::BlockIndexList blk_idxs;
  layer.getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    typename voxblox::Block<VoxelT>::ConstPtr blk_ptr = layer.getBlockPtrByIndex(blk_idx);
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
    typename voxblox::Block<VoxelT>::ConstPtr blk_ptr = layer.getBlockPtrByIndex(blk_idx);
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
    typename voxblox::Block<VoxelT>::ConstPtr blk_ptr = layer.getBlockPtrByIndex(blk_idx);
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

void getViewDirsOfOccupiedVoxels(const OccupancyLayer& layer, const float occ_thresh,
                                 Vec3dVec* view_dirs);

void getCentersOfOccupiedVoxels(const OccupancyLayer& layer, const float occ_thresh,
                                Vec3dVec* points_w);

void getCentersOfOccupiedVoxels(const OccupancyLayer& layer, const float occ_thresh,
                                Vec3dVec* blk_cs, V3dVecVec* points_w, V3dVecVec* blk_view_dirs);

size_t countNumOccupiedVoxels(const OccupancyLayer& layer, const float occ_thres);

void getCentersOfOccupiedVoxels(const voxblox::Block<OccupancyVoxel>& blk, const float occ_thresh,
                                Vec3dVec* points_w);

template <typename T>
void getCentersOfAllVoxels(const voxblox::Layer<T>& layer, Vec3dVec* points_w)
{
  getFromAllVoxels<T, Eigen::Vector3d>(layer, getVoxelCenter<T>, points_w);
}

template <typename T>
void getCentersOfAllVoxels(const voxblox::Block<T>& blk, Vec3dVec* points_w)
{
  getFromAllVoxels<T, Eigen::Vector3d>(blk, getVoxelCenter<T>, points_w);
}

template <typename T>
void getCentersOfAllBlocks(const voxblox::Layer<T>& layer, Vec3dVec* points_w)
{
  getFromAllBlocks<T, Eigen::Vector3d>(layer, getBlockCenter<T>, points_w);
}

template <typename T>
void getCentersOfAllActivatedBlocks(const voxblox::Layer<T>& layer, Vec3dVec* points_w)
{
  getFromAllBlocksIf<T, Eigen::Vector3d>(layer, getBlockCenter<T>, isBlockActivated<T>, points_w);
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

template <typename T>
void setLayerValid(voxblox::Layer<T>* layer)
{
  modifyAllBlocks<T>(layer, setBlockDataValid<T>);
}

// other templated functions
template <typename T>
void subsampleVoxelIndices(const voxblox::Block<T>& blk, const int samples_per_side,
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
          vox_indicies->emplace_back(voxblox::VoxelIndex(ix * step, iy * step, iz * step));
        }
      }
    }
  }
}

template <typename T>
void allocateBlocksByCoordinatesBatch(const act_map::Vec3dVec& points_w, voxblox::Layer<T>* layer,
                                      voxblox::BlockIndexList* newly_added_blks,
                                      voxblox::IndexSet* covered_blks)
{
  CHECK_NOTNULL(layer);

  if (newly_added_blks)
  {
    newly_added_blks->clear();
  }

  for (const Eigen::Vector3d& pt : points_w)
  {
    voxblox::BlockIndex blk_idx = layer->computeBlockIndexFromCoordinates(pt);
    if (!layer->getBlockPtrByIndex(blk_idx))
    {
      layer->allocateBlockPtrByIndex(blk_idx);
      if (newly_added_blks)
      {
        newly_added_blks->push_back(blk_idx);
      }
    }
    if (covered_blks)
    {
      covered_blks->insert(blk_idx);
    }
  }
}

// this function assumes we already found the block via coordinates
template <typename T>
const T& getVoxelAndCenterFromCoordinates(const voxblox::Block<T>& blk, const Eigen::Vector3d& pos,
                                          Eigen::Vector3d* vox_c)
{
  const size_t vox_idx = blk.computeLinearIndexFromCoordinates(pos);
  *vox_c = blk.computeCoordinatesFromLinearIndex(vox_idx);
  return blk.getVoxelByLinearIndex(vox_idx);
}

// non-templated functions
// basically integrate points multiple times
void setPointsInOccupancyLayer(const Eigen::Matrix3Xd& points, act_map::OccupancyLayer* occ_layer);

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
}  // namespace utils
}  // namespace act_map
