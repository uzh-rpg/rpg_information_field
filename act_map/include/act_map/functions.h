#pragma once

#include "act_map/voxblox/core/layer.h"

namespace act_map
{
template <typename VoxelT>
inline void getBlockCenterFromBlk(const voxblox::Block<VoxelT>& blk,
                                  Eigen::Vector3d* c)
{
  voxblox::Point org = blk.origin();
  const double blk_hsize = blk.block_size() / 2.0;
  c->x() = org(0, 0) + blk_hsize;
  c->y() = org(1, 0) + blk_hsize;
  c->z() = org(2, 0) + blk_hsize;
}
}  // namespace act_map

namespace act_map
{
template <typename VoxelT>
using VoxelCheckerType = std::function<bool(const VoxelT&)>;

template <typename VoxelT>
using BlockCheckerType = std::function<bool(const voxblox::Block<VoxelT>&)>;

template <typename VoxelT, typename OutT>
using VoxelGetterType = std::function<void(const voxblox::Block<VoxelT>&,
                                           const int lin_idx, OutT* out)>;

template <typename VoxelT, typename OutT>
using BlockGetterType = std::function<void(
    const voxblox::Layer<VoxelT>&, const voxblox::BlockIndex, OutT* out)>;

template <typename VoxelT>
using BlockModifierType = std::function<void(voxblox::Block<VoxelT>*)>;

template <typename VoxelT>
using VoxelModifierType = std::function<void(VoxelT*)>;

template <typename VoxelT>
inline bool isBlockUpdated(const voxblox::Block<VoxelT>& blk)
{
  return blk.updated();
}

template <typename VoxelT>
inline bool isBlockActivated(const voxblox::Block<VoxelT>& blk)
{
  return blk.activated();
}

template <typename VoxelT>
inline bool isBlockNotUpdated(const voxblox::Block<VoxelT>& blk)
{
  return !blk.updated();
}

template <typename VoxelT>
inline bool isBlockNearThan(const voxblox::Block<VoxelT>& blk,
                            const Eigen::Vector3d& pos, const double thresh)
{
  Eigen::Vector3d c;
  getBlockCenterFromBlk<VoxelT>(blk, &c);
  const double d = (c - pos).norm();
  return d < thresh;
}

template <typename VoxelT>
inline bool isBlockFartherThan(const voxblox::Block<VoxelT>& blk,
                               const Eigen::Vector3d& pos, const double thresh)
{
  Eigen::Vector3d c;
  getBlockCenterFromBlk<VoxelT>(blk, &c);
  const double d = (c - pos).norm();
  return d > thresh;
}

template <typename VoxelT>
inline void clearBlockUpdate(voxblox::Block<VoxelT>* blk)
{
  blk->set_updated(false);
}

template <typename VoxelT>
inline void setBlockDataValid(voxblox::Block<VoxelT>* blk)
{
  blk->setVoxDataValid(true);
}

template <typename VoxelT>
inline void activateBlock(voxblox::Block<VoxelT>* blk)
{
  blk->set_activated(true);
}

template <typename VoxelT>
inline void deactivateBlock(voxblox::Block<VoxelT>* blk)
{
  blk->set_activated(false);
}

inline bool isOccupancyVoxelOccupied(const voxblox::OccupancyVoxel& vox,
                                     const float thresh_occ_prob)
{
  return vox.probability_log > voxblox::logOddsFromProbability(thresh_occ_prob);
}

inline void setOccupancyVoxelOccupied(voxblox::OccupancyVoxel* vox)
{
  vox->probability_log = 1.0f;
  vox->observed = true;
}

template <typename VoxelT>
inline void getVoxelCenter(const voxblox::Block<VoxelT>& blk, const int lin_idx,
                           Eigen::Vector3d* c)
{
  (*c) = blk.computeCoordinatesFromLinearIndex(lin_idx);
}

template <typename VoxelT>
inline void getBlockCenter(const voxblox::Layer<VoxelT>& layer,
                           const voxblox::BlockIndex blk_idx,
                           Eigen::Vector3d* c)
{
  const voxblox::Block<VoxelT>& blk = layer.getBlockByIndex(blk_idx);
  getBlockCenterFromBlk<VoxelT>(blk, c);
}

inline void
getViewDirOccVoxel(const voxblox::Block<voxblox::OccupancyVoxel>& blk,
                   const int lin_idx, Eigen::Vector3d* c)
{
  const voxblox::OccupancyVoxel& vox =
      blk.getVoxelByLinearIndex(static_cast<size_t>(lin_idx));
  (*c) = vox.aver_view_from_pt.cast<double>();
}
}  // namespace act_map
