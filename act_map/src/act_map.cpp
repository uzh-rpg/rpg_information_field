#include "act_map/act_map.h"

#include "act_map/optim_orient.h"

namespace act_map
{
template <>
void ActMap<QuadTraceVoxel>::getBestViewsAt(const size_t cam_id, const int samples_per_side,
                                        const bool use_sampling, const bool only_updated,
                                        Vec3dVec* vox_cs, Vec3dVec* best_views,
                                        std::vector<double>* values,
                                        voxblox::LongIndexVector* global_idxs) const
{
  std::lock_guard<std::mutex> ker_lock(pos_factor_mutex_);
  voxblox::BlockIndexList viz_blks;
  if (only_updated)
  {
    viz_blks = accumulated_updated_kblk_idxs_;
  }
  else
  {
    // useful to have consistent color scale
    pos_factor_layer_->getAllAllocatedBlocks(&viz_blks);
  }

  if (use_sampling)
  {
    optim_orient::getBestViewsSample(*pos_factor_layer_, viz_blks, vis_scores_[cam_id].k1(),
                                     vis_scores_[cam_id].k2(), vis_scores_[cam_id].k3(),
                                     samples_per_side, vox_cs, best_views, values, global_idxs);
  }
  else
  {
    LOG(FATAL) << "Closed form is not implemented correctly currently.";
    optim_orient::getBestViewsClosed(*pos_factor_layer_, viz_blks, vis_scores_[cam_id].k1(),
                                     vis_scores_[cam_id].k2(), samples_per_side, vox_cs, best_views,
                                     global_idxs);
  }
}

template <>
void ActMap<GPTraceVoxel>::getBestViewsAt(const size_t cam_id, const int samples_per_side,
                                          const bool use_sampling, const bool only_updated,
                                          Vec3dVec* vox_cs, Vec3dVec* best_views,
                                          std::vector<double>* values,
                                          voxblox::LongIndexVector* global_idxs) const
{
  std::lock_guard<std::mutex> ker_lock(pos_factor_mutex_);
  voxblox::BlockIndexList viz_blks;
  if (only_updated)
  {
    viz_blks = accumulated_updated_kblk_idxs_;
  }
  else
  {
    // useful to have consistent color scale
    pos_factor_layer_->getAllAllocatedBlocks(&viz_blks);
  }

  optim_orient::getBestViewsSample(*pos_factor_layer_, viz_blks, samples_per_side, vox_cs, best_views,
                                   values, global_idxs);
}

template <>
void ActMap<QuadInfoVoxel>::getBestViewFromNearestVoxel(const Eigen::Vector3d& pos,
                                                    const InfoMetricType& type,
                                                    Eigen::Vector3d* vox_c,
                                                    Eigen::Vector3d* best_view) const
{
  sampleRotations();
  std::shared_ptr<QuadInfoBlock> blk_ptr = pos_factor_layer_->getBlockPtrByCoordinates(pos);
  if (!blk_ptr)
  {
    LOG(WARNING) << "Cannot find block/voxel near " << pos.transpose();
    vox_c->setConstant(0.0);
    best_view->setConstant(0.0);
    return;
  }
  const QuadInfoVoxel& vox = utils::getVoxelAndCenterFromCoordinates(*blk_ptr, pos, vox_c);

  const QuadraticVisScore& quad_vis = vis_scores_.front();
  double optim_val;
  optim_orient::getOptimViewFromInfoKernels(rot_samples_, quad_vis.k1(), quad_vis.k2(),
                                            quad_vis.k3(), vox.K1, vox.K2, vox.K3, type, best_view,
                                            &optim_val);
}

template <>
void ActMap<QuadTraceVoxel>::getBestViewFromNearestVoxel(const Eigen::Vector3d& pos,
                                                     const InfoMetricType& type,
                                                     Eigen::Vector3d* vox_c,
                                                     Eigen::Vector3d* best_view) const
{
  sampleRotations();
  std::shared_ptr<QuadTraceBlock> blk_ptr = pos_factor_layer_->getBlockPtrByCoordinates(pos);
  if (!blk_ptr)
  {
    LOG(WARNING) << "Cannot find block/voxel near " << pos.transpose();
    vox_c->setConstant(0.0);
    best_view->setConstant(0.0);
    return;
  }
  const QuadTraceVoxel& vox = utils::getVoxelAndCenterFromCoordinates(*blk_ptr, pos, vox_c);

  const QuadraticVisScore& quad_vis = vis_scores_.front();
  double optim_val;
  optim_orient::getOptimViewFromTraceKernels(rot_samples_, quad_vis.k1(), quad_vis.k2(),
                                             quad_vis.k3(), vox.K1, vox.K2, vox.K3, best_view,
                                             &optim_val);
}

template <>
void ActMap<GPInfoVoxel>::getBestViewFromNearestVoxel(const Eigen::Vector3d& pos,
                                                      const InfoMetricType& type,
                                                      Eigen::Vector3d* vox_c,
                                                      Eigen::Vector3d* best_view) const
{
  sampleRotations();
  std::shared_ptr<GPInfoBlock> blk_ptr = pos_factor_layer_->getBlockPtrByCoordinates(pos);
  if (!blk_ptr)
  {
    LOG(WARNING) << "Cannot find block/voxel near " << pos.transpose();
    vox_c->setConstant(0.0);
    best_view->setConstant(0.0);
    return;
  }
  const GPInfoVoxel& vox = utils::getVoxelAndCenterFromCoordinates(*blk_ptr, pos, vox_c);

  double optim_val;
  optim_orient::getOptimViewFromGPInfoVoxel(rot_samples_, vox, type, best_view, &optim_val);
}

template <>
void ActMap<GPTraceVoxel>::getBestViewFromNearestVoxel(const Eigen::Vector3d& pos,
                                                       const InfoMetricType& type,
                                                       Eigen::Vector3d* vox_c,
                                                       Eigen::Vector3d* best_view) const
{
  sampleRotations();
  std::shared_ptr<GPTraceBlock> blk_ptr = pos_factor_layer_->getBlockPtrByCoordinates(pos);
  if (!blk_ptr)
  {
    LOG(WARNING) << "Cannot find block/voxel near " << pos.transpose();
    vox_c->setConstant(0.0);
    best_view->setConstant(0.0);
    return;
  }
  const GPTraceVoxel& vox = utils::getVoxelAndCenterFromCoordinates(*blk_ptr, pos, vox_c);

  double optim_val;
  optim_orient::getOptimViewFromGPTraceVoxel(rot_samples_, vox, best_view, &optim_val);
}

template <>
void ActMap<QuadInfoVoxel>::getFIMFromNearestVoxel(const rpg::Pose& Twc, Eigen::Vector3d* vox_c,
                                               Info* fim) const
{
  const Eigen::Vector3d& pos = Twc.getPosition();
  std::shared_ptr<QuadInfoBlock> blk_ptr = pos_factor_layer_->getBlockPtrByCoordinates(pos);
  if (!blk_ptr)
  {
    LOG(WARNING) << "Cannot find block/voxel near " << pos.transpose();
    vox_c->setConstant(0.0);
    fim->setConstant(0.0);
    return;
  }
  const QuadInfoVoxel& vox = utils::getVoxelAndCenterFromCoordinates(*blk_ptr, pos, vox_c);

  const QuadraticVisScore& quad_vis = vis_scores_.front();
  getInfoAtRotation(Twc.getRotationMatrix(), quad_vis.k1(), quad_vis.k2(), quad_vis.k3(), vox.K1,
                    vox.K2, vox.K3, fim);
}

template <>
void ActMap<GPInfoVoxel>::getFIMFromNearestVoxel(const rpg::Pose& Twc, Eigen::Vector3d* vox_c,
                                                 Info* fim) const
{
  const Eigen::Vector3d& pos = Twc.getPosition();
  std::shared_ptr<GPInfoBlock> blk_ptr = pos_factor_layer_->getBlockPtrByCoordinates(pos);
  if (!blk_ptr)
  {
    LOG(WARNING) << "Cannot find block/voxel near " << pos.transpose();
    vox_c->setConstant(0.0);
    fim->setConstant(0.0);
    return;
  }
  const GPInfoVoxel& vox = utils::getVoxelAndCenterFromCoordinates(*blk_ptr, pos, vox_c);
  vox.queryAtRotation(Twc.getRotationMatrix(), fim);
}

}  // namespace act_map
