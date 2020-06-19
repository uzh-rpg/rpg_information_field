//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <voxblox/core/layer.h>

#include "act_map/kernel_voxel_ops.h"
#include "act_map/common.h"
#include "act_map/conversion.h"
#include "act_map/voxblox_utils.h"

namespace act_map
{
struct KernelLayerIntegratorOptions
{
  float occ_thresh_ = 0.7f;
  float min_vis_dist = 0.1f;
  float max_vis_dist = 10.0f;
};

enum class KernelUpdateAction
{
  kAdd,
  kDelete,
  kReCompute
};

// KVType: KernelVoxelType
template <typename KVType>
class KernelLayerIntegrator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KernelLayerIntegrator() = delete;
  KernelLayerIntegrator(const KernelLayerIntegrator&) = delete;
  KernelLayerIntegrator(
      const KernelLayerIntegratorOptions& options,
      const OccupancyLayer::Ptr& occ_layer_ptr,
      const typename voxblox::Layer<KVType>::Ptr& ker_layer_ptr)
    : options_(options)
    , occ_layer_ptr_(occ_layer_ptr)
    , ker_layer_ptr_(ker_layer_ptr)
  {
    blk_center_check_thresh_ =
        occ_layer_ptr->block_size() * 0.866 + options_.max_vis_dist;
  }

  ~KernelLayerIntegrator()
  {
  }

  template <typename ContainerType>
  void recomputeFromOccupancyLayer(const ContainerType& cand_blks,
                                   voxblox::BlockIndexList* updated = nullptr);

  // these functions do not depend on the occupancy layer, but only input points
  // NOTE: the visiblity checking is performed for eahc point individually
  template <typename ContainerType>
  void recomputeKernelLayer(const act_map::Vec3dVec& points_w,
                            const ContainerType& cand_blks,
                            voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void addPointsToKernelLayer(const act_map::Vec3dVec& points_w,
                              const ContainerType& cand_blks,
                              voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void deletePointsFromKernelLayer(const act_map::Vec3dVec& points_w,
                                   const ContainerType& cand_blks,
                                   voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void updateKernelLayerFromPoints(const Vec3dVec& points,
                                   const ContainerType& cand_blks,
                                   const KernelUpdateAction& action,
                                   voxblox::BlockIndexList* updated = nullptr);
  // pre-filter with block centers
  template <typename ContainerType>
  void recomputeKernelLayer(const Vec3dVec& blc_cs,
                            const V3dVecVec& points,
                            const ContainerType& cand_blks,
                            voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void addPointsToKernelLayer(const voxblox::HierarchicalIndexMap& added_points,
                              const ContainerType& cand_blks,
                              voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void
  deletePointsFromKernelLayer(const voxblox::HierarchicalIndexMap& del_points,
                              const ContainerType& cand_blks,
                              voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void
  updateKernelLayerFromPoints(const voxblox::HierarchicalIndexMap& points_list,
                              const double dist_thresh,
                              const ContainerType& cand_blks,
                              const KernelUpdateAction& action,
                              voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void updateKernelLayerFromPoints(const Vec3dVec& blc_cs,
                                   const V3dVecVec& points,
                                   const double thresh,
                                   const ContainerType& cand_blks,
                                   const KernelUpdateAction& action,
                                   voxblox::BlockIndexList* updated = nullptr);

  // function that does the actual update
  bool updateKernelVoxelFromPoints(const Vec3dVec& points,
                                   const KernelUpdateAction& action,
                                   voxblox::Block<KVType>* blk,
                                   const size_t lin_idx);

  inline double getBlockCenterCheckThresh() const
  {
    return blk_center_check_thresh_;
  }

  KernelLayerIntegratorOptions options_;

private:
  OccupancyLayer::Ptr occ_layer_ptr_;
  typename voxblox::Layer<KVType>::Ptr ker_layer_ptr_;
  // if the distance between a point and the occupancy block is above this,
  // the voxels in that block is not visible to the point anyway
  double blk_center_check_thresh_;
};

template <typename KVType>
template <typename ContainerType>
void KernelLayerIntegrator<KVType>::recomputeFromOccupancyLayer(
    const ContainerType& cand_blks, voxblox::BlockIndexList* updated_blks)
{
  if (updated_blks)
  {
    updated_blks->clear();
  }

  if (cand_blks.size() == 0 ||
      ker_layer_ptr_->getNumberOfAllocatedBlocks() == 0)
  {
    return;
  }

  Vec3dVec points_w;
  utils::getCentersOfOccupiedVoxels(
      *occ_layer_ptr_, options_.occ_thresh_, &points_w);

  if (points_w.size() == 0)
  {
    return;
  }
  updateKernelLayerFromPoints(
      points_w, cand_blks, KernelUpdateAction::kReCompute, updated_blks);
}

template <typename KVType>
template <typename ContainerType>
void KernelLayerIntegrator<KVType>::recomputeKernelLayer(
    const act_map::Vec3dVec& points_w,
    const ContainerType& cand_blks,
    voxblox::BlockIndexList* updated_blks)
{
  updateKernelLayerFromPoints(
      points_w, cand_blks, KernelUpdateAction::kReCompute, updated_blks);
}

template <typename KVType>
template <typename ContainerType>
void KernelLayerIntegrator<KVType>::addPointsToKernelLayer(
    const act_map::Vec3dVec& points_w,
    const ContainerType& cand_blks,
    voxblox::BlockIndexList* updated_blks)
{
  updateKernelLayerFromPoints(
      points_w, cand_blks, KernelUpdateAction::kAdd, updated_blks);
}

template <typename KVType>
template <typename ContainerType>
void KernelLayerIntegrator<KVType>::deletePointsFromKernelLayer(
    const act_map::Vec3dVec& points_w,
    const ContainerType& cand_blks,
    voxblox::BlockIndexList* updated_blks)
{
  updateKernelLayerFromPoints(
      points_w, cand_blks, KernelUpdateAction::kDelete, updated_blks);
}

template <typename KVType>
template <typename ContainerType>
void KernelLayerIntegrator<KVType>::updateKernelLayerFromPoints(
    const Vec3dVec& points_w,
    const ContainerType& cand_blks,
    const KernelUpdateAction& action,
    voxblox::BlockIndexList* updated_blks)
{
  if (updated_blks)
  {
    updated_blks->clear();
  }

  if (cand_blks.size() == 0 ||
      ker_layer_ptr_->getNumberOfAllocatedBlocks() == 0 || points_w.size() == 0)
  {
    return;
  }

  for (const voxblox::BlockIndex& blk_idx : cand_blks)
  {
    typename voxblox::Block<KVType>::Ptr blk_ptr =
        ker_layer_ptr_->getBlockPtrByIndex(blk_idx);
    if (!blk_ptr->activated())
    {
      continue;
    }
    bool blk_updated = false;
    for (size_t vox_i = 0; vox_i < blk_ptr->num_voxels(); vox_i++)
    {
      if (blk_ptr->isVoxMasked(vox_i))
      {
        continue;
      }
      bool vox_updated =
          updateKernelVoxelFromPoints(points_w, action, blk_ptr.get(), vox_i);
      if (vox_updated)
      {
        blk_updated = true;
        blk_ptr->setVoxUpdated(true, vox_i);
        blk_ptr->setVoxDataValid(true, vox_i);
      }
    }  // each voxel

    if (blk_updated)
    {
      blk_ptr->set_updated(true);
      if (updated_blks)
      {
        updated_blks->emplace_back(blk_idx);
      }
    }
  }  // each block
}

template <typename KVType>
template <typename ContainerType>
void KernelLayerIntegrator<KVType>::recomputeKernelLayer(
    const Vec3dVec& occ_blk_cs,
    const V3dVecVec& occ_points_w,
    const ContainerType& cand_blks,
    voxblox::BlockIndexList* updated_blks)
{
  updateKernelLayerFromPoints(occ_blk_cs,
                              occ_points_w,
                              blk_center_check_thresh_,
                              cand_blks,
                              KernelUpdateAction::kReCompute,
                              updated_blks);
}

template <typename KVType>
template <typename ContainerType>
void KernelLayerIntegrator<KVType>::addPointsToKernelLayer(
    const voxblox::HierarchicalIndexMap& added_points,
    const ContainerType& cand_blks,
    voxblox::BlockIndexList* updated_blks)
{
  updateKernelLayerFromPoints(added_points,
                              blk_center_check_thresh_,
                              cand_blks,
                              KernelUpdateAction::kAdd,
                              updated_blks);
}

template <typename KVType>
template <typename ContainerType>
void KernelLayerIntegrator<KVType>::deletePointsFromKernelLayer(
    const voxblox::HierarchicalIndexMap& added_points,
    const ContainerType& cand_blks,
    voxblox::BlockIndexList* updated_blks)
{
  updateKernelLayerFromPoints(added_points,
                              blk_center_check_thresh_,
                              cand_blks,
                              KernelUpdateAction::kDelete,
                              updated_blks);
}

template <typename KVType>
template <typename ContainerType>
void KernelLayerIntegrator<KVType>::updateKernelLayerFromPoints(
    const voxblox::HierarchicalIndexMap& points_list,
    const double dist_thresh,
    const ContainerType& cand_blks,
    const KernelUpdateAction& action,
    voxblox::BlockIndexList* updated_blks)
{
  Vec3dVec occ_blk_cs;
  V3dVecVec occ_points_w;
  for (const auto& pair : points_list)
  {
    Eigen::Vector3d blk_c;
    const OccupancyBlock& blk = occ_layer_ptr_->getBlockByIndex(pair.first);
    getBlockCenterFromBlk(blk, &blk_c);
    occ_blk_cs.emplace_back(blk_c);
    Vec3dVec blk_points_w;
    for (const auto& vidx : pair.second)
    {
      blk_points_w.emplace_back(blk.computeCoordinatesFromVoxelIndex(vidx));
    }
    occ_points_w.emplace_back(blk_points_w);
  }
  updateKernelLayerFromPoints(
      occ_blk_cs, occ_points_w, dist_thresh, cand_blks, action, updated_blks);
}

template <typename KVType>
template <typename ContainerType>
void KernelLayerIntegrator<KVType>::updateKernelLayerFromPoints(
    const Vec3dVec& occ_blk_cs,
    const V3dVecVec& occ_points_w,
    const double dist_thresh,
    const ContainerType& cand_blks,
    const KernelUpdateAction& action,
    voxblox::BlockIndexList* updated_blks)
{
  if (updated_blks)
  {
    updated_blks->clear();
  }
  if (cand_blks.size() == 0 ||
      ker_layer_ptr_->getNumberOfAllocatedBlocks() == 0 ||
      occ_points_w.size() == 0)
  {
    return;
  }
  CHECK_EQ(occ_blk_cs.size(), occ_points_w.size());

  for (const voxblox::BlockIndex& blk_idx : cand_blks)
  {
    typename voxblox::Block<KVType>::Ptr blk_ptr =
        ker_layer_ptr_->getBlockPtrByIndex(blk_idx);
    if (!blk_ptr->activated())
    {
      continue;
    }
    bool blk_updated = false;
    for (size_t vox_i = 0; vox_i < blk_ptr->num_voxels(); vox_i++)
    {
      Eigen::Vector3d voxc = blk_ptr->computeCoordinatesFromLinearIndex(vox_i);
      for (size_t occ_blk_i = 0; occ_blk_i < occ_blk_cs.size(); occ_blk_i++)
      {
        if ((occ_blk_cs[occ_blk_i] - voxc).norm() > dist_thresh)
        {
          continue;
        }
        bool vox_updated = updateKernelVoxelFromPoints(
            occ_points_w[occ_blk_i], action, blk_ptr.get(), vox_i);
        if (vox_updated)
        {
          blk_updated = true;
          blk_ptr->setVoxUpdated(true, vox_i);
          blk_ptr->setVoxDataValid(true, vox_i);
        }
      }
    }
    if (blk_updated)
    {
      blk_ptr->set_updated(true);
      if (updated_blks)
      {
        updated_blks->emplace_back(blk_idx);
      }
    }
  }
}

template <typename KVType>
bool KernelLayerIntegrator<KVType>::updateKernelVoxelFromPoints(
    const Vec3dVec& points_w,
    const KernelUpdateAction& action,
    voxblox::Block<KVType>* blk,
    size_t lin_idx)
{
  bool vox_updated = false;

  Eigen::Vector3d c = blk->computeCoordinatesFromLinearIndex(lin_idx);
  bool first_pt = true;
  for (size_t pt_i = 0; pt_i < points_w.size(); pt_i++)
  {
    const Eigen::Vector3d& pw = points_w[pt_i];
    if (isPointOutOfRange(
            c, pw, options_.min_vis_dist, options_.max_vis_dist) ||
        isPointOcculuded(c, pw, *occ_layer_ptr_))
    {
      continue;
    }

    if (first_pt && action == KernelUpdateAction::kReCompute)
    {
      assignToKernelVoxel(*blk, lin_idx, pw);
      first_pt = false;
    }
    else if (action == KernelUpdateAction::kDelete)
    {
      substractFromKernelVoxel(*blk, lin_idx, pw);
    }
    else if (action == KernelUpdateAction::kReCompute ||
             action == KernelUpdateAction::kAdd)
    {
      addToKernelVoxel(*blk, lin_idx, pw);
    }

    if (!vox_updated)
    {
      vox_updated = true;
    }
  }
  return vox_updated;
}
}
