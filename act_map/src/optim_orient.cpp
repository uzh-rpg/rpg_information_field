#include "act_map/optim_orient.h"

#include <numeric>

#include <rpg_common/save.h>
#include <vi_utils/cam_min.h>
#include <vi_utils/common_utils.h>

#include "act_map/conversion.h"
#include "act_map/voxblox_utils.h"
#include "act_map/sampler.h"

namespace act_map
{
namespace optim_orient
{
void OptimOrientRes::resize(const size_t N)
{
  CHECK_GT(N, 0u);
  positions_.resize(N);
  optim_views_.resize(N);
  optim_vals_.resize(static_cast<Eigen::Index>(N));
}

void OptimOrientRes::save(const std::string& abs_save_dir,
                          const bool save_pos) const
{
  Eigen::MatrixX3d optim_view_mat;
  VecKVecToEigenXK(optim_views_, &optim_view_mat);
  rpg::save(abs_save_dir + "/optim_view_" + name_ + ".txt", optim_view_mat);
  rpg::save(abs_save_dir + "/optim_value_" + name_ + ".txt", optim_vals_);

  if (save_pos)
  {
    Eigen::MatrixX3d pos_mat;
    VecKVecToEigenXK(positions_, &pos_mat);
    rpg::save(abs_save_dir + "/vox_pos.txt", pos_mat);
  }
}

void getOptimViewFromInfoKernels(const rpg::RotationVec& rot_samples,
                                 const double k1, const double k2,
                                 const double k3, const act_map::InfoK1& K1,
                                 const act_map::InfoK2& K2,
                                 const act_map::InfoK3& K3,
                                 const act_map::InfoMetricType type,
                                 Eigen::Vector3d* optim_view, double* optim)
{
  double max = -1;
  size_t optim_idx = 0u;
  for (size_t i = 0; i < rot_samples.size(); i++)
  {
    rpg::Matrix66 info_i;
    act_map::getInfoAtRotation(rot_samples[i].getRotationMatrix(), k1, k2, k3,
                               K1, K2, K3, &info_i);

    double cur = 0;

    cur = act_map::getInfoMetric(info_i, type);

    //    Eigen::Matrix3Xd pose_mag = info_i.block<3, 3>(0, 0) -
    //                                info_i.block<3, 3>(0, 3) *
    //                                    (info_i.block<3, 3>(3, 3)).inverse() *
    //                                    info_i.block<3, 3>(3, 0);
    //    cur = getInfoMetric(pose_mag, InfoMetricType::kMinEig);

    //    cur = info_i.determinant();

    if (cur > max)
    {
      max = cur;
      optim_idx = i;
    }
  }

  (*optim_view) = (rot_samples[optim_idx]).rotate(Eigen::Vector3d(0, 0, 1));
  if (optim)
  {
    (*optim) = max;
  }
}

void getOptimViewFromGPInfoVoxel(const rpg::RotationVec& rot_samples,
                                 const GPInfoVoxel& vox,
                                 const act_map::InfoMetricType type,
                                 Eigen::Vector3d* optim_view, double* optim)
{
  double max = -1;
  size_t optim_idx = 0u;
  for (size_t i = 0; i < rot_samples.size(); i++)
  {
    rpg::Matrix66 info_i;
    vox.queryAtRotation(rot_samples[i].getRotationMatrix(), &info_i);

    double cur = 0;

    cur = act_map::getInfoMetric(info_i, type);
    if (cur > max)
    {
      max = cur;
      optim_idx = i;
    }
  }

  (*optim_view) = (rot_samples[optim_idx]).rotate(Eigen::Vector3d(0, 0, 1));
  if (optim)
  {
    (*optim) = max;
  }
}

void getOptimViewFromGPTraceVoxel(const rpg::RotationVec& rot_samples,
                                  const GPTraceVoxel& vox,
                                  Eigen::Vector3d* optim_view, double* optim)
{
  double max = -1;
  size_t optim_idx = 0u;
  for (size_t i = 0; i < rot_samples.size(); i++)
  {
    rpg::Matrix11 trace_i;
    vox.queryAtRotation(rot_samples[i].getRotationMatrix(), &trace_i);
    double cur = trace_i(0, 0);
    if (cur > max)
    {
      max = cur;
      optim_idx = i;
    }
  }

  (*optim_view) = (rot_samples[optim_idx]).rotate(Eigen::Vector3d(0, 0, 1));
  if (optim)
  {
    (*optim) = max;
  }
}

void getOptimViewFromTraceKernels(const rpg::RotationVec& rot_samples,
                                  const double k1, const double k2,
                                  const double k3, const act_map::TraceK1& K1,
                                  const act_map::TraceK2& K2,
                                  const act_map::TraceK3& K3,
                                  Eigen::Vector3d* optim_view, double* optim)
{
  double max_trace = -1;
  size_t optim_idx = 0u;
  for (size_t i = 0; i < rot_samples.size(); i++)
  {
    double cur_trace = act_map::getTraceAtRotation(
        rot_samples[i].getRotationMatrix(), k1, k2, k3, K1, K2, K3);
    if (cur_trace > max_trace)
    {
      max_trace = cur_trace;
      optim_idx = i;
    }
  }
  (*optim_view) = (rot_samples[optim_idx]).rotate(Eigen::Vector3d(0, 0, 1));
  if (optim)
  {
    (*optim) = max_trace;
  }
}

void getWorstViewFromTraceKernels(const rpg::RotationVec& rot_samples,
                                  const double k1, const double k2,
                                  const double k3, const act_map::TraceK1& K1,
                                  const act_map::TraceK2& K2,
                                  const act_map::TraceK3& K3,
                                  Eigen::Vector3d* worst_view, double* worst)
{
  double min_trace = std::numeric_limits<double>::infinity();
  size_t worst_idx = 0u;
  for (size_t i = 0; i < rot_samples.size(); i++)
  {
    double cur_trace = act_map::getTraceAtRotation(
        rot_samples[i].getRotationMatrix(), k1, k2, k3, K1, K2, K3);
    if (cur_trace < min_trace)
    {
      min_trace = cur_trace;
      worst_idx = i;
    }
  }
  (*worst_view) = (rot_samples[worst_idx]).rotate(Eigen::Vector3d(0, 0, 1));
  (*worst) = min_trace;
}

void getMinTraceDirectionFromTraceKernelsUnconstrained(
    const double k1, const double k2, const double k3,
    const act_map::TraceK1& K1, const act_map::TraceK2& K2,
    const act_map::TraceK3& K3, Eigen::Vector3d* view, double* val)
{
  act_map::minTraceDirectionUnconstrained(k1, k2, K1, K2, view);
  (*val) = k1 * view->transpose() * K1.cast<double>() * (*view) +
           k2 * (view->transpose() * K2.cast<double>()).value() +
           k3 * static_cast<double>(K3(0, 0));
}

void getOptimalViewFromExactInfo(const rpg::RotationVec& rot_samples,
                                 const Eigen::Vector3d& twc,
                                 const Eigen::Matrix3Xd& pws,
                                 const vi::PinholeCam& cam,
                                 const act_map::InfoMetricType& type,
                                 Eigen::Vector3d* optim_view, double* optim)
{
  double max = -1;
  size_t optim_idx = 0u;
  const size_t kNPts = static_cast<size_t>(pws.cols());

  for (size_t rot_i = 0; rot_i < rot_samples.size(); rot_i++)
  {
    rpg::Pose Twc(rot_samples[rot_i], twc);
 
    Eigen::Matrix3Xd pcs = Twc.inverse().transformVectorized(pws);
    Eigen::Matrix2Xd us;
    us.resize(Eigen::NoChange, pws.cols());
    std::vector<bool> is_visible(kNPts);
    cam.project3dBatch(pcs, &us, &is_visible);

    rpg::Matrix66 info_pose_i;
    info_pose_i.setZero();
    for (size_t pt_i = 0; pt_i < kNPts; pt_i++)
    {
      if (is_visible[pt_i])
      {
        act_map::addPoseInfoBearingGlobal(Twc, pws.col(static_cast<int>(pt_i)),
                                          nullptr, &info_pose_i);
      }
    }

    double cur = act_map::getInfoMetric(info_pose_i, type);
    if (cur > max)
    {
      max = cur;
      optim_idx = rot_i;
    }
  }

  (*optim_view) = (rot_samples[optim_idx]).rotate(Eigen::Vector3d(0, 0, 1));
  (*optim) = max;
}

void getBestViewsClosed(const act_map::QuadTraceLayer& tl,
                        const voxblox::BlockIndexList& blk_idxs,
                        const double k1, const double k2,
                        const int samples_per_side, act_map::Vec3dVec* vox_cs,
                        act_map::Vec3dVec* best_views,
                        voxblox::LongIndexVector* global_idxs)
{
  CHECK_NOTNULL(vox_cs);
  CHECK_NOTNULL(best_views);
  CHECK_GT(samples_per_side, 0);

  for (const voxblox::BlockIndex blk_idx : blk_idxs)
  {
    const QuadTraceBlock& blk = tl.getBlockByIndex(blk_idx);
    voxblox::VoxelIndexList sub_vox_idxs;
    utils::subsampleVoxelIndices(blk, samples_per_side, &sub_vox_idxs);
    for (const voxblox::VoxelIndex& vox_idx : sub_vox_idxs)
    {
      size_t lin_idx = blk.computeLinearIndexFromVoxelIndex(vox_idx);
      if (!blk.isVoxDataValid(lin_idx))
      {
        continue;
      }
      const QuadTraceVoxel& vox = blk.getVoxelByVoxelIndex(vox_idx);
      Eigen::Vector3d pos = blk.computeCoordinatesFromVoxelIndex(vox_idx);
      Eigen::Vector3d bview;
      minTraceDirectionUnconstrained(k1, k2, vox.K1, vox.K2, &bview);

      vox_cs->emplace_back(pos);
      best_views->emplace_back(bview * (-1));
      global_idxs->emplace_back(
          voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
              blk_idx, vox_idx, static_cast<int>(blk.voxels_per_side())));
    }
  }
}

void getBestViewsSample(const act_map::QuadTraceLayer& tl,
                        const voxblox::BlockIndexList& blk_idxs,
                        const double k1, const double k2, const double k3,
                        const int samples_per_side, act_map::Vec3dVec* vox_cs,
                        act_map::Vec3dVec* best_views,
                        std::vector<double>* values,
                        voxblox::LongIndexVector* global_idxs)
{
  CHECK_NOTNULL(vox_cs);
  vox_cs->clear();
  CHECK_NOTNULL(best_views);
  best_views->clear();
  CHECK_NOTNULL(values);
  values->clear();
  CHECK_NOTNULL(global_idxs);
  global_idxs->clear();
  static rpg::RotationVec rot_samples;
  if (rot_samples.empty())
  {
    utils::sampleRotation(10.0, &rot_samples);
  }

  for (const voxblox::BlockIndex blk_idx : blk_idxs)
  {
    const QuadTraceBlock& blk = tl.getBlockByIndex(blk_idx);
    if (!blk.activated())
    {
      continue;
    }
    voxblox::VoxelIndexList sub_vox_idxs;
    utils::subsampleVoxelIndices(blk, samples_per_side, &sub_vox_idxs);
    for (const voxblox::VoxelIndex& vox_idx : sub_vox_idxs)
    {
      size_t lin_idx = blk.computeLinearIndexFromVoxelIndex(vox_idx);
      if (!blk.isVoxDataValid(lin_idx))
      {
        continue;
      }
      const QuadTraceVoxel& vox = blk.getVoxelByVoxelIndex(vox_idx);
      Eigen::Vector3d pos = blk.computeCoordinatesFromVoxelIndex(vox_idx);
      Eigen::Vector3d bview;
      double val;
      optim_orient::getOptimViewFromTraceKernels(
          rot_samples, k1, k2, k3, vox.K1, vox.K2, vox.K3, &bview, &val);

      vox_cs->emplace_back(pos);
      best_views->emplace_back(bview);
      global_idxs->emplace_back(
          voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
              blk_idx, vox_idx, static_cast<int>(blk.voxels_per_side())));
      values->emplace_back(val);
    }
  }
}

void getBestViewsSample(const act_map::GPTraceLayer& tl,
                        const voxblox::BlockIndexList& blk_idxs,
                        const int samples_per_side, act_map::Vec3dVec* vox_cs,
                        act_map::Vec3dVec* best_views,
                        std::vector<double>* values,
                        voxblox::LongIndexVector* global_idxs)
{
  CHECK_NOTNULL(vox_cs);
  vox_cs->clear();
  CHECK_NOTNULL(best_views);
  best_views->clear();
  CHECK_NOTNULL(values);
  values->clear();
  CHECK_NOTNULL(global_idxs);
  global_idxs->clear();
  static rpg::RotationVec rot_samples;
  if (rot_samples.empty())
  {
    utils::sampleRotation(10.0, &rot_samples);
  }

  for (const voxblox::BlockIndex blk_idx : blk_idxs)
  {
    const GPTraceBlock& blk = tl.getBlockByIndex(blk_idx);
    if (!blk.activated())
    {
      VLOG(5) << "Block " << blk_idx << " not activated, skip";
      continue;
    }
    voxblox::VoxelIndexList sub_vox_idxs;
    utils::subsampleVoxelIndices(blk, samples_per_side, &sub_vox_idxs);
    for (const voxblox::VoxelIndex& vox_idx : sub_vox_idxs)
    {
      size_t lin_idx = blk.computeLinearIndexFromVoxelIndex(vox_idx);
      if (!blk.isVoxDataValid(lin_idx))
      {
        VLOG(5) << "Voxel " << lin_idx << " in block " << blk_idx
                << " does not have valid data, skip.";
        continue;
      }
      const GPTraceVoxel& vox = blk.getVoxelByVoxelIndex(vox_idx);
      Eigen::Vector3d pos = blk.computeCoordinatesFromVoxelIndex(vox_idx);
      Eigen::Vector3d bview;
      double val;
      optim_orient::getOptimViewFromGPTraceVoxel(rot_samples, vox, &bview,
                                                 &val);

      vox_cs->emplace_back(pos);
      best_views->emplace_back(bview);
      global_idxs->emplace_back(
          voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
              blk_idx, vox_idx, static_cast<int>(blk.voxels_per_side())));
      values->emplace_back(val);
    }
  }
}
}  // optim_orient
}
