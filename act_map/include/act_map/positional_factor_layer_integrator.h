#pragma once

#include "act_map/voxblox/core/layer.h"
#include "act_map/positional_factor_voxel_ops.h"
#include "act_map/common.h"
#include "act_map/conversion.h"
#include "act_map/voxblox_utils.h"
#include "act_map/visibility_checker.h"

namespace act_map
{
struct PositionalFactorLayerIntegratorOptions
{
  float occ_thresh_ = 0.7f;
};

enum class FactorUpdateAction
{
  kAdd,
  kDelete,
  kReCompute
};

// PVType: Positional factor Voxel type
template <typename PVType>
class PositionalFactorLayerIntegrator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionalFactorLayerIntegrator() = delete;
  PositionalFactorLayerIntegrator(const PositionalFactorLayerIntegrator&) =
      delete;
  PositionalFactorLayerIntegrator(
      const PositionalFactorLayerIntegratorOptions& options,
      const OccupancyLayer::Ptr& occ_layer_ptr,
      const typename voxblox::Layer<PVType>::Ptr& factor_layer_ptr,
      const VisibilityCheckerPtr& vis_ptr)
    : options_(options)
    , occ_layer_ptr_(occ_layer_ptr)
    , factor_layer_ptr_(factor_layer_ptr)
    , vis_checker_ptr_(vis_ptr)
  {
    if (vis_checker_ptr_->options_.max_dist ==
        std::numeric_limits<double>::infinity())
    {
      blk_center_check_thresh_ = std::numeric_limits<double>::infinity();
    }
    else
    {
      blk_center_check_thresh_ = occ_layer_ptr->block_size() * 0.866 +
                                 vis_checker_ptr_->options_.max_dist;
    }
  }

  ~PositionalFactorLayerIntegrator()
  {
  }

  void setVisibilityChecker(const VisibilityCheckerPtr& vis_ptr)
  {
    vis_checker_ptr_ = vis_ptr;
  }

  template <typename ContainerType>
  void recomputeFromOccupancyLayer(const ContainerType& cand_blks,
                                   voxblox::BlockIndexList* updated = nullptr);

  // these functions do not depend on the occupancy layer, but only input points
  // NOTE: the visiblity checking is performed for eahc point individually
  template <typename ContainerType>
  void recomputeFactorLayer(const act_map::Vec3dVec& points_w,
                            const act_map::Vec3dVec& view_dirs,
                            const ContainerType& cand_blks,
                            voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void addPointsToFactorLayer(const act_map::Vec3dVec& points_w,
                              const act_map::Vec3dVec& view_dirs,
                              const ContainerType& cand_blks,
                              voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void deletePointsFromFactorLayer(const act_map::Vec3dVec& points_w,
                                   const act_map::Vec3dVec& view_dirs,
                                   const ContainerType& cand_blks,
                                   voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void updateFactorLayerFromPoints(const Vec3dVec& points,
                                   const Vec3dVec& view_dirs,
                                   const ContainerType& cand_blks,
                                   const FactorUpdateAction& action,
                                   voxblox::BlockIndexList* updated = nullptr);

  // pre-filter with block centers
  template <typename ContainerType>
  void recomputeFactorLayer(const Vec3dVec& blc_cs, const V3dVecVec& points,
                            const V3dVecVec& view_dirs,
                            const ContainerType& cand_blks,
                            voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void addPointsToFactorLayer(const voxblox::HierarchicalIndexMap& added_points,
                              const ContainerType& cand_blks,
                              voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void
  deletePointsFromFactorLayer(const voxblox::HierarchicalIndexMap& del_points,
                              const ContainerType& cand_blks,
                              voxblox::BlockIndexList* updated = nullptr);

  // update the layer from points with distance check
  template <typename ContainerType>
  void
  updateFactorLayerFromPoints(const voxblox::HierarchicalIndexMap& points_list,
                              const double dist_thresh,
                              const ContainerType& cand_blks,
                              const FactorUpdateAction& action,
                              voxblox::BlockIndexList* updated = nullptr);
  template <typename ContainerType>
  void updateFactorLayerFromPoints(const Vec3dVec& blc_cs,
                                   const V3dVecVec& points,
                                   const V3dVecVec& view_dirs,
                                   const double thresh,
                                   const ContainerType& cand_blks,
                                   const FactorUpdateAction& action,
                                   voxblox::BlockIndexList* updated = nullptr);

  // function that does the actual update
  bool updatePositionalFactorVoxelFromPoints(const Vec3dVec& points,
                                             const Vec3dVec& view_dirs,
                                             const FactorUpdateAction& action,
                                             voxblox::Block<PVType>* blk,
                                             const size_t lin_idx);

  inline double getBlockCenterCheckThresh() const
  {
    return blk_center_check_thresh_;
  }

  PositionalFactorLayerIntegratorOptions options_;

private:
  OccupancyLayer::Ptr occ_layer_ptr_;
  typename voxblox::Layer<PVType>::Ptr factor_layer_ptr_;
  VisibilityCheckerPtr vis_checker_ptr_;
  // if the distance between a point and the occupancy block is above this,
  // the voxels in that block is not visible to the point anyway
  // used to filter some blocks
  double blk_center_check_thresh_;
};

template <typename PVType>
template <typename ContainerType>
void PositionalFactorLayerIntegrator<PVType>::recomputeFromOccupancyLayer(
    const ContainerType& cand_blks, voxblox::BlockIndexList* updated_blks)
{
  if (updated_blks)
  {
    updated_blks->clear();
  }

  if (cand_blks.size() == 0 ||
      factor_layer_ptr_->getNumberOfAllocatedBlocks() == 0)
  {
    return;
  }

  Vec3dVec points_w;
  utils::getCentersOfOccupiedVoxels(*occ_layer_ptr_, options_.occ_thresh_,
                                    &points_w);
  Vec3dVec view_dirs;
  utils::getViewDirsOfOccupiedVoxels(*occ_layer_ptr_, options_.occ_thresh_,
                                     &view_dirs);

  if (points_w.size() == 0)
  {
    return;
  }
  updateFactorLayerFromPoints(points_w, view_dirs, cand_blks,
                              FactorUpdateAction::kReCompute, updated_blks);
}

template <typename PVType>
template <typename ContainerType>
void PositionalFactorLayerIntegrator<PVType>::recomputeFactorLayer(
    const act_map::Vec3dVec& points_w, const act_map::Vec3dVec& view_dirs,
    const ContainerType& cand_blks, voxblox::BlockIndexList* updated_blks)
{
  updateFactorLayerFromPoints(points_w, view_dirs, cand_blks,
                              FactorUpdateAction::kReCompute, updated_blks);
}

template <typename PVType>
template <typename ContainerType>
void PositionalFactorLayerIntegrator<PVType>::addPointsToFactorLayer(
    const act_map::Vec3dVec& points_w, const act_map::Vec3dVec& view_dirs,
    const ContainerType& cand_blks, voxblox::BlockIndexList* updated_blks)
{
  updateFactorLayerFromPoints(points_w, view_dirs, cand_blks,
                              FactorUpdateAction::kAdd, updated_blks);
}

template <typename PVType>
template <typename ContainerType>
void PositionalFactorLayerIntegrator<PVType>::deletePointsFromFactorLayer(
    const act_map::Vec3dVec& points_w, const act_map::Vec3dVec& view_dirs,
    const ContainerType& cand_blks, voxblox::BlockIndexList* updated_blks)
{
  updateFactorLayerFromPoints(points_w, view_dirs, cand_blks,
                              FactorUpdateAction::kDelete, updated_blks);
}

template <typename PVType>
template <typename ContainerType>
void PositionalFactorLayerIntegrator<PVType>::updateFactorLayerFromPoints(
    const Vec3dVec& points_w, const Vec3dVec& view_dirs,
    const ContainerType& cand_blks, const FactorUpdateAction& action,
    voxblox::BlockIndexList* updated_blks)
{
  if (updated_blks)
  {
    updated_blks->clear();
  }

  if (cand_blks.size() == 0 ||
      factor_layer_ptr_->getNumberOfAllocatedBlocks() == 0 ||
      points_w.size() == 0)
  {
    VLOG(1) << "Update factor layer: nothing to update";
    return;
  }

  for (const voxblox::BlockIndex& blk_idx : cand_blks)
  {
    typename voxblox::Block<PVType>::Ptr blk_ptr =
        factor_layer_ptr_->getBlockPtrByIndex(blk_idx);
    if (!blk_ptr->activated())
    {
      VLOG(1) << "Block " << blk_idx << " not activated.";
      continue;
    }
    bool blk_updated = false;
    for (size_t vox_i = 0; vox_i < blk_ptr->num_voxels(); vox_i++)
    {
      if (blk_ptr->isVoxMasked(vox_i))
      {
        VLOG(1) << "Voxel " << vox_i << " in block " << blk_idx << " masked";
        continue;
      }
      bool vox_updated = updatePositionalFactorVoxelFromPoints(
          points_w, view_dirs, action, blk_ptr.get(), vox_i);
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

template <typename PVType>
template <typename ContainerType>
void PositionalFactorLayerIntegrator<PVType>::recomputeFactorLayer(
    const Vec3dVec& occ_blk_cs, const V3dVecVec& occ_points_w,
    const V3dVecVec& occ_view_dirs, const ContainerType& cand_blks,
    voxblox::BlockIndexList* updated_blks)
{
  updateFactorLayerFromPoints(occ_blk_cs, occ_points_w, occ_view_dirs,
                              blk_center_check_thresh_, cand_blks,
                              FactorUpdateAction::kReCompute, updated_blks);
}

template <typename PVType>
template <typename ContainerType>
void PositionalFactorLayerIntegrator<PVType>::addPointsToFactorLayer(
    const voxblox::HierarchicalIndexMap& added_points,
    const ContainerType& cand_blks, voxblox::BlockIndexList* updated_blks)
{
  updateFactorLayerFromPoints(added_points, blk_center_check_thresh_, cand_blks,
                              FactorUpdateAction::kAdd, updated_blks);
}

template <typename PVType>
template <typename ContainerType>
void PositionalFactorLayerIntegrator<PVType>::deletePointsFromFactorLayer(
    const voxblox::HierarchicalIndexMap& added_points,
    const ContainerType& cand_blks, voxblox::BlockIndexList* updated_blks)
{
  updateFactorLayerFromPoints(added_points, blk_center_check_thresh_, cand_blks,
                              FactorUpdateAction::kDelete, updated_blks);
}

template <typename PVType>
template <typename ContainerType>
void PositionalFactorLayerIntegrator<PVType>::updateFactorLayerFromPoints(
    const voxblox::HierarchicalIndexMap& points_list, const double dist_thresh,
    const ContainerType& cand_blks, const FactorUpdateAction& action,
    voxblox::BlockIndexList* updated_blks)
{
  Vec3dVec occ_blk_cs;
  V3dVecVec occ_points_w;
  V3dVecVec occ_view_dirs;
  for (const auto& pair : points_list)
  {
    Eigen::Vector3d blk_c;
    const OccupancyBlock& blk = occ_layer_ptr_->getBlockByIndex(pair.first);
    getBlockCenterFromBlk(blk, &blk_c);
    occ_blk_cs.emplace_back(blk_c);
    Vec3dVec blk_points_w;
    Vec3dVec blk_view_dirs;
    for (const auto& vidx : pair.second)
    {
      blk_points_w.emplace_back(blk.computeCoordinatesFromVoxelIndex(vidx));
      blk_view_dirs.emplace_back(
          blk.getVoxelByVoxelIndex(vidx).aver_view_from_pt.cast<double>());
    }
    occ_points_w.emplace_back(blk_points_w);
    occ_view_dirs.emplace_back(blk_view_dirs);
  }
  updateFactorLayerFromPoints(occ_blk_cs, occ_points_w, occ_view_dirs,
                              dist_thresh, cand_blks, action, updated_blks);
}

template <typename PVType>
template <typename ContainerType>
void PositionalFactorLayerIntegrator<PVType>::updateFactorLayerFromPoints(
    const Vec3dVec& occ_blk_cs, const V3dVecVec& occ_points_w,
    const V3dVecVec& occ_view_dirs, const double dist_thresh,
    const ContainerType& cand_blks, const FactorUpdateAction& action,
    voxblox::BlockIndexList* updated_blks)
{
  if (updated_blks)
  {
    updated_blks->clear();
  }
  if (cand_blks.size() == 0 ||
      factor_layer_ptr_->getNumberOfAllocatedBlocks() == 0 ||
      occ_points_w.size() == 0)
  {
    return;
  }
  CHECK_EQ(occ_blk_cs.size(), occ_points_w.size());

  for (const voxblox::BlockIndex& blk_idx : cand_blks)
  {
    typename voxblox::Block<PVType>::Ptr blk_ptr =
        factor_layer_ptr_->getBlockPtrByIndex(blk_idx);
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
        bool vox_updated = updatePositionalFactorVoxelFromPoints(
            occ_points_w[occ_blk_i], occ_view_dirs[occ_blk_i], action,
            blk_ptr.get(), vox_i);
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

template <typename PVType>
bool PositionalFactorLayerIntegrator<PVType>::
    updatePositionalFactorVoxelFromPoints(const Vec3dVec& points_w,
                                          const Vec3dVec& view_dirs,
                                          const FactorUpdateAction& action,
                                          voxblox::Block<PVType>* blk,
                                          size_t lin_idx)
{
  bool vox_updated = false;

  Eigen::Vector3d c = blk->computeCoordinatesFromLinearIndex(lin_idx);
  bool first_pt = true;

  std::set<size_t> viz_idx;
  vis_checker_ptr_->getVisibleIdx(c, points_w, view_dirs, &viz_idx);

  // update with each point
  for (const size_t pt_i : viz_idx)
  {
    const Eigen::Vector3d& pw = points_w[pt_i];

    if (first_pt && action == FactorUpdateAction::kReCompute)
    {
      assignToFactorVoxel(*blk, lin_idx, pw);
      first_pt = false;
    }
    else if (action == FactorUpdateAction::kDelete)
    {
      substractFromFactorVoxel(*blk, lin_idx, pw);
    }
    else if (action == FactorUpdateAction::kReCompute ||
             action == FactorUpdateAction::kAdd)
    {
      addToFactorVoxel(*blk, lin_idx, pw);
    }

    if (!vox_updated)
    {
      vox_updated = true;
    }
  }
  return vox_updated;
}
}  // namespace act_map
