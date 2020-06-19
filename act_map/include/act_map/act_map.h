//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <iostream>
#include <memory>

#include <rpg_common/pose.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <rpg_common/timer.h>

#include "act_map/vis_score.h"
#include "act_map/common.h"
#include "act_map/sampler.h"
#include "act_map/kernel_layer_integrator.h"

namespace act_map
{
struct ActMapOptions
{
  LayerOptions occ_layer_options_;
  voxblox::OccupancyIntegrator::Config occ_integrator_options_;

  std::vector<VisScoreOptions> vis_options_;
  LayerOptions ker_layer_options_;
  act_map::KernelLayerIntegratorOptions ker_integrator_options_;

  bool use_collision_checker_ = false;
  utils::CollisionCheckerOptions col_ops_;
};

template <typename T>
class ActMap
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ActMap() = delete;
  ActMap(const ActMap&) = delete;
  ActMap& operator=(const ActMap&) = delete;

  ActMap(const ActMapOptions& options);
  ~ActMap()
  {
  }

  template <typename KVType>
  friend std::ostream& operator<<(std::ostream& os, const ActMap<KVType>& am);

  ActMapOptions options_;

  inline voxblox::Layer<voxblox::OccupancyVoxel>::Ptr occLayerPtr()
  {
    return occ_layer_;
  }

  inline const voxblox::Layer<voxblox::OccupancyVoxel>& occLayerCRef() const
  {
    return *occ_layer_;
  }

  // kernel layer related
  void allocateKernelLayerUniform(const std::vector<double>& ranges);

  inline typename voxblox::Layer<T>::Ptr kerLayerPtr()
  {
    return ker_layer_;
  }

  inline const voxblox::Layer<T>& kerLayerCRef() const
  {
    return *ker_layer_;
  }

  // map update
  void integratePointCloudOccupancy(const rpg::Pose& T_w_c,
                                    const Eigen::Matrix3Xd& points_c);
  void recomputeKernelLayer();

  void updateKernelLayerIncremental();

  void addRegionToKernelLayer(const rpg::Pose& Twb,
                              const std::vector<double>& ranges);

  // map query
  void getBestViewsAt(const size_t cam_id,
                      const int samples_per_side,
                      const bool use_sampling,
                      const bool only_updated,
                      Vec3dVec* vox_cs,
                      Vec3dVec* best_views,
                      std::vector<double>* values,
                      voxblox::LongIndexVector* global_idxs) const;

  // misc
  inline const voxblox::BlockIndexList&
  getAccumulatedUpdatedKernelBlocksIndices() const
  {
    return accumulated_updated_kblk_idxs_;
  }
  inline void clearAccumulatedUpdatedKernelBlockIndices()
  {
    accumulated_updated_kblk_idxs_.clear();
  }
  inline double updatedBlkRatioOcc() const
  {
    return utils::getUpdatedRatio<OccupancyVoxel>(*occ_layer_);
  }
  inline double updatedBlkRatioKernel() const
  {
    return utils::getUpdatedRatio<T>(*ker_layer_);
  }
  inline double accumulatedUpdatedBlkRatioKernel() const
  {
    size_t n_all_blks = ker_layer_->getNumberOfAllocatedBlocks();
    if (n_all_blks == 0)
    {
      return 0.0;
    }
    return accumulated_updated_kblk_idxs_.size() / n_all_blks;
  }

  inline const voxblox::HierarchicalIndexMap& getLastDeletedOccPoints()
  {
    return last_deleted_occ_pts_;
  }

  inline const voxblox::HierarchicalIndexMap& getLastAddedOccPoints()
  {
    return last_added_occ_pts_;
  }

  inline const voxblox::IndexSet getNewlyAllocatedKernelBlockIndices()
  {
    return kblk_idxs_to_recompute_;
  }

  inline const voxblox::IndexSet getPrevAllocatedKernelBlockIndices()
  {
    return kblk_idxs_to_update_;
  }

  double occBlockHalfDiagonal() const
  {
    return occ_block_half_diagonal_;
  }

  double kerBlockHalfDiagonal() const
  {
    return ker_block_half_diagonal_;
  }

  void getCentersOfOccupiedVoxels(act_map::Vec3dVec* points_w) const;
  void getCentersOfOccupiedVoxels(act_map::Vec3dVec* blk_cs,
                                  act_map::V3dVecVec* blk_points_w) const;

  void activateBlocksByDistance(const Eigen::Vector3d& pos,
                                const double thresh);

  void saveLayers(const std::string& occ_layer_fn,
                  const std::string& ker_layer_fn) const
  {
    rpg::Timer timer;
    {
      std::lock_guard<std::mutex> ker_lock(ker_mutex_);
      LOG(WARNING) << "Saving kernel layer...";
      timer.start();
      voxblox::io::SaveLayer<T>(*ker_layer_, ker_layer_fn);
      VLOG(1) << "Saving kernel layer took " << timer.stop() * 1000 << " ms";
    }
    {
      std::lock_guard<std::mutex> occ_lock(occ_mutex_);
      LOG(WARNING) << "Saving occupancy layer...";
      timer.start();
      voxblox::io::SaveLayer<OccupancyVoxel>(*occ_layer_, occ_layer_fn);
      VLOG(1) << "Saving occupancy layer took " << timer.stop() * 1000 << " ms";
    }
  }

private:
  mutable std::mutex occ_mutex_;
  OccupancyLayer::Ptr occ_layer_;
  std::unique_ptr<voxblox::OccupancyIntegrator> occ_integrator_;

  mutable std::mutex ker_mutex_;
  typename voxblox::Layer<T>::Ptr ker_layer_;
  std::unique_ptr<act_map::KernelLayerIntegrator<T>> ker_integrator_;

  std::vector<VisScore> vis_scores_;  // only used for visualization

  // update results
  // We do not lock this one: shoule be used after the update is done
  voxblox::BlockIndexList accumulated_updated_kblk_idxs_;
  // these varibles will be used by both occupancy integration and kernel
  // integration, should be locked
  voxblox::HierarchicalIndexMap last_deleted_occ_pts_;
  voxblox::HierarchicalIndexMap last_added_occ_pts_;

  // used to maintain the kernel layer blocks
  // These variable are used both by the expansion and update.
  voxblox::IndexSet kblk_idxs_to_update_;
  voxblox::IndexSet kblk_idxs_to_recompute_;

  // other variables
  double occ_block_half_diagonal_;
  double ker_block_half_diagonal_;
};

template <typename T>
using ActMapPtr = std::shared_ptr<ActMap<T>>;
using TraceMap = ActMap<TraceVoxel>;
using TraceMapPtr = std::shared_ptr<TraceMap>;
using InfoMap = ActMap<InfoVoxel>;
using InfoMapPtr = std::shared_ptr<InfoMap>;

template <typename T>
ActMap<T>::ActMap(const ActMapOptions& options)
  : options_(options)
{
  occ_layer_ = std::make_shared<voxblox::Layer<voxblox::OccupancyVoxel>>(
      options_.occ_layer_options_.vox_size,
      options_.occ_layer_options_.vox_per_side);
  occ_integrator_.reset(new voxblox::OccupancyIntegrator(
      options_.occ_integrator_options_, occ_layer_.get()));

  ker_layer_ = std::make_shared<voxblox::Layer<T>>(
      options_.ker_layer_options_.vox_size,
      options_.ker_layer_options_.vox_per_side);
  ker_integrator_.reset(new KernelLayerIntegrator<T>(
      options_.ker_integrator_options_, occ_layer_, ker_layer_));

  CHECK_GT(options_.vis_options_.size(), 0u);
  for (size_t i = 0; i < options_.vis_options_.size(); i++)
  {
    const VisScoreOptions& op = options_.vis_options_[i];
    vis_scores_.emplace_back(op.half_fov_rad);
    vis_scores_[i].initSecondOrderApprox(op.boundary_to_mid_ratio,
                                         op.boundary_value);
  }
  occ_block_half_diagonal_ = occ_layer_->block_size() * 0.866;
  ker_block_half_diagonal_ = ker_layer_->block_size() * 0.866;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const ActMap<T>& am)
{
  os << "ActMap:\n"
     << "- Occupancy:\n"
     << "  - voxel size: " << am.occ_layer_->voxel_size() << std::endl
     << "  - voxel per side: " << am.occ_layer_->voxels_per_side() << std::endl
     << "  - block size: " << am.occ_layer_->block_size() << std::endl;
  return os;
}

template <typename T>
void ActMap<T>::getCentersOfOccupiedVoxels(act_map::Vec3dVec* points_w) const
{
  std::lock_guard<std::mutex> lock_occ(occ_mutex_);
  utils::getCentersOfOccupiedVoxels(
      *occ_layer_, options_.ker_integrator_options_.occ_thresh_, points_w);
}

template <typename T>
void ActMap<T>::getCentersOfOccupiedVoxels(
    act_map::Vec3dVec* blk_cs, act_map::V3dVecVec* blk_points_w) const
{
  std::lock_guard<std::mutex> lock_occ(occ_mutex_);
  CHECK_NOTNULL(blk_cs);
  utils::getCentersOfOccupiedVoxels(
      *occ_layer_,
      options_.ker_integrator_options_.occ_thresh_,
      blk_cs,
      blk_points_w);
}

template <typename T>
void ActMap<T>::integratePointCloudOccupancy(const rpg::Pose& T_w_c,
                                             const Eigen::Matrix3Xd& points_c)
{
  voxblox::Pointcloud pc_c;
  act_map::eigenKXToVecKVec(points_c, &pc_c);
  {
    std::lock_guard<std::mutex> lock(occ_mutex_);
    occ_integrator_->integratePointCloud(
        T_w_c, pc_c, &last_deleted_occ_pts_, &last_added_occ_pts_);
  }
}

template <typename T>
void ActMap<T>::allocateKernelLayerUniform(const std::vector<double>& ranges)
{
  rpg::PositionVec points;
  if (ranges.size() == 3)
  {
    utils::generateUniformPointsWithin(options_.ker_layer_options_.vox_size,
                                       ranges[0],
                                       ranges[1],
                                       ranges[2],
                                       &points);
  }
  else if (ranges.size() == 6)
  {
    utils::generateUniformPointsWithin(options_.ker_layer_options_.vox_size,
                                       ranges[0],
                                       ranges[1],
                                       ranges[2],
                                       ranges[3],
                                       ranges[4],
                                       ranges[5],
                                       &points);
  }
  else
  {
    LOG(FATAL) << "Wrong range specification.";
  }

  {
    std::lock_guard<std::mutex> lock_ker(ker_mutex_);
    std::lock_guard<std::mutex> lock_occ(occ_mutex_);
    voxblox::BlockIndexList new_blks;
    voxblox::IndexSet covered_blks;
    utils::allocateBlocksByCoordinatesBatch(
        points, ker_layer_.get(), &new_blks, &covered_blks);
    int n_masked = 0;
    if (options_.use_collision_checker_)
    {
      for (const voxblox::BlockIndex& idx : new_blks)
      {
        typename voxblox::Block<T>::Ptr blk_ptr =
            ker_layer_->getBlockPtrByIndex(idx);
        n_masked += utils::maskCollidedVoxels(
            *occ_layer_,
            blk_ptr.get(),
            options_.ker_integrator_options_.occ_thresh_,
            options_.col_ops_);
      }
    }
    else
    {
      VLOG(3) << "Not using collision checker.";
    }
    VLOG(3) << "Kernel Expand: allocated " << new_blks.size() << " new blocks, "
            << "covered " << covered_blks.size() << " blocks, "
            << "and masked " << n_masked << " voxels in covered blocks.";
    kblk_idxs_to_recompute_.insert(new_blks.begin(), new_blks.end());
  }
}

template <typename T>
void ActMap<T>::recomputeKernelLayer()
{
  Vec3dVec points_w;
  clearAccumulatedUpdatedKernelBlockIndices();
  getCentersOfOccupiedVoxels(&points_w);
  {
    std::lock_guard<std::mutex> lock_ker(ker_mutex_);
    voxblox::BlockIndexList cand_blks;
    ker_layer_->getAllAllocatedBlocks(&cand_blks);
    ker_integrator_->recomputeKernelLayer(
        points_w, cand_blks, &accumulated_updated_kblk_idxs_);
  }
}

template <typename T>
void ActMap<T>::updateKernelLayerIncremental()
{
  std::lock_guard<std::mutex> lock_ker(ker_mutex_);

  voxblox::BlockIndexList del_block_idxs;
  voxblox::BlockIndexList add_block_idxs;
  rpg::Timer timer;
  timer.start();
  ker_integrator_->deletePointsFromKernelLayer(
      last_deleted_occ_pts_, kblk_idxs_to_update_, &del_block_idxs);

  ker_integrator_->addPointsToKernelLayer(
      last_added_occ_pts_, kblk_idxs_to_update_, &add_block_idxs);

  VLOG(3) << "Kernel Updater: "
          << "processed " << last_deleted_occ_pts_.size() << " deleted points, "
          << last_added_occ_pts_.size() << " added points.";
  VLOG(2) << "Update kernels took " << timer.stop() * 1000 << " ms.";

  timer.start();
  voxblox::BlockIndexList recompute_blk_idxs;
  if (kblk_idxs_to_recompute_.size() != 0)
  {
    Vec3dVec blk_cs;
    V3dVecVec blk_points;
    getCentersOfOccupiedVoxels(&blk_cs, &blk_points);
    ker_integrator_->recomputeKernelLayer(
        blk_cs, blk_points, kblk_idxs_to_recompute_, &recompute_blk_idxs);
  }
  VLOG(2) << "Recomptue kernels took " << timer.stop() * 1000 << " ms.";

  const size_t n_to_recompute = kblk_idxs_to_recompute_.size();
  const size_t n_recomputed = recompute_blk_idxs.size();

  voxblox::IndexSet merged_idxs(del_block_idxs.begin(), del_block_idxs.end());
  merged_idxs.insert(add_block_idxs.begin(), add_block_idxs.end());
  const size_t n_modified = merged_idxs.size();
  merged_idxs.insert(recompute_blk_idxs.begin(), recompute_blk_idxs.end());
  const size_t n_updated = merged_idxs.size();
  accumulated_updated_kblk_idxs_.insert(accumulated_updated_kblk_idxs_.end(),
                                        merged_idxs.begin(),
                                        merged_idxs.end());
  const size_t n_accumulate_updated = accumulated_updated_kblk_idxs_.size();
  VLOG(3) << "Kernel Updater: "
          << "to recompute " << n_to_recompute << " blocks, "
          << "recomputed " << n_recomputed << " blocks, "
          << "modified " << n_modified << " blocks, "
          << "total updated " << n_updated << " blocks, "
          << "accumulated updated " << n_accumulate_updated << " blocks.";

  kblk_idxs_to_update_.insert(kblk_idxs_to_recompute_.begin(),
                              kblk_idxs_to_recompute_.end());
  kblk_idxs_to_recompute_.clear();
}

template <typename T>
void ActMap<T>::addRegionToKernelLayer(const rpg::Pose& Twb,
                                       const std::vector<double>& ranges)
{
  Eigen::Vector3d center = Twb.getPosition();
  std::vector<double> alloc_ranges(6);
  if (ranges.size() == 3)
  {
    std::vector<double> half_ranges;
    for (const double v : ranges)
    {
      CHECK_GT(v, 0.1);
      half_ranges.push_back(0.5 * v);
    }
    alloc_ranges = { center.x() - half_ranges[0], center.x() + half_ranges[0],
                     center.y() - half_ranges[1], center.y() + half_ranges[1],
                     center.z() - half_ranges[2], center.z() + half_ranges[2] };
  }
  else if (ranges.size() == 6)
  {
    alloc_ranges = { center.x() + ranges[0], center.x() + ranges[1],
                     center.y() + ranges[2], center.y() + ranges[3],
                     center.z() + ranges[4], center.z() + ranges[5] };
  }
  allocateKernelLayerUniform(alloc_ranges);
}

template <typename T>
void ActMap<T>::getBestViewsAt(const size_t cam_id,
                               const int samples_per_side,
                               const bool use_sampling,
                               const bool only_updated,
                               Vec3dVec* vox_cs,
                               Vec3dVec* best_views,
                               std::vector<double>* values,
                               voxblox::LongIndexVector* global_idxs) const
{
  std::lock_guard<std::mutex> ker_lock(ker_mutex_);
  voxblox::BlockIndexList viz_blks;
  if (only_updated)
  {
    viz_blks = accumulated_updated_kblk_idxs_;
  }
  else
  {
    // useful to have consistent color scale
    ker_layer_->getAllAllocatedBlocks(&viz_blks);
  }

  if (use_sampling)
  {
    utils::getBestViewsSample(*ker_layer_,
                              viz_blks,
                              vis_scores_[cam_id].k1(),
                              vis_scores_[cam_id].k2(),
                              vis_scores_[cam_id].k3(),
                              samples_per_side,
                              vox_cs,
                              best_views,
                              values,
                              global_idxs);
  }
  else
  {
    LOG(FATAL) << "Closed form is not implemented correctly currently.";
    utils::getBestViewsClosed(*ker_layer_,
                              viz_blks,
                              vis_scores_[cam_id].k1(),
                              vis_scores_[cam_id].k2(),
                              samples_per_side,
                              vox_cs,
                              best_views,
                              global_idxs);
  }
}

template <typename T>
void ActMap<T>::activateBlocksByDistance(const Eigen::Vector3d& pos,
                                         const double thresh)
{
  voxblox::BlockIndexList reactivated_idxs;
  voxblox::BlockIndexList deactivated_idxs;

  std::lock_guard<std::mutex> lock(ker_mutex_);

  voxblox::BlockIndexList blk_idxs;
  ker_layer_->getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& idx : blk_idxs)
  {
    typename voxblox::Block<T>::Ptr blk_ptr =
        ker_layer_->getBlockPtrByIndex(idx);
    Eigen::Vector3d c;
    getBlockCenterFromBlk(*blk_ptr, &c);

    if ((c - pos).norm() < thresh)
    {
      if (!blk_ptr->activated())
      {
        reactivated_idxs.push_back(idx);
        blk_ptr->set_activated(true);
      }
    }
    else
    {
      if (blk_ptr->activated())
      {
        deactivated_idxs.push_back(idx);
        blk_ptr->set_activated(false);
      }
    }
  }

  VLOG(3) << "Blocks: Active -> Deactive: " << deactivated_idxs.size()
          << ", Deactive -> Active: " << reactivated_idxs.size();
  for (const voxblox::BlockIndex& idx : deactivated_idxs)
  {
    kblk_idxs_to_recompute_.erase(idx);
    kblk_idxs_to_update_.erase(idx);
  }
  for (const voxblox::BlockIndex& idx : reactivated_idxs)
  {
    kblk_idxs_to_recompute_.insert(idx);
  }
}

template <>
void ActMap<InfoVoxel>::getBestViewsAt(
    const size_t cam_id,
    const int samples_per_side,
    const bool use_sampling,
    const bool only_updated,
    Vec3dVec* vox_cs,
    Vec3dVec* best_views,
    std::vector<double>* values,
    voxblox::LongIndexVector* global_idxs) const
{
  LOG(WARNING) << "Best view visualization not supported for this kernel,"
                  " doing nothing.";
}
}
