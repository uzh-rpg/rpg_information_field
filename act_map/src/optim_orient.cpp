//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/optim_orient.h"

#include <numeric>

#include <rpg_common/save.h>
#include <vi_utils/cam_min.h>
#include <vi_utils/common_utils.h>

#include "act_map/conversion.h"

namespace act_map
{
namespace optim_orient
{
void OptimOrientRes::resize(const size_t N)
{
  CHECK_GT(N, 0);
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
                                 const double k1,
                                 const double k2,
                                 const double k3,
                                 const act_map::InfoK1& K1,
                                 const act_map::InfoK2& K2,
                                 const act_map::InfoK3& K3,
                                 const act_map::InfoMetricType type,
                                 Eigen::Vector3d* optim_view,
                                 double* optim)
{
  double max = -1;
  size_t optim_idx = 0u;
  for (size_t i = 0; i < rot_samples.size(); i++)
  {
    rpg::Matrix66 info_i;
    act_map::getInfoAtRotation(
        rot_samples[i].getRotationMatrix(), k1, k2, k3, K1, K2, K3, &info_i);

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

void getOptimViewFromTraceKernels(const rpg::RotationVec& rot_samples,
                                  const double k1,
                                  const double k2,
                                  const double k3,
                                  const act_map::TraceK1& K1,
                                  const act_map::TraceK2& K2,
                                  const act_map::TraceK3& K3,
                                  Eigen::Vector3d* optim_view,
                                  double* optim)
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
                                  const double k1,
                                  const double k2,
                                  const double k3,
                                  const act_map::TraceK1& K1,
                                  const act_map::TraceK2& K2,
                                  const act_map::TraceK3& K3,
                                  Eigen::Vector3d* worst_view,
                                  double* worst)
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
    const double k1,
    const double k2,
    const double k3,
    const act_map::TraceK1& K1,
    const act_map::TraceK2& K2,
    const act_map::TraceK3& K3,
    Eigen::Vector3d* view,
    double* val)
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
                                 Eigen::Vector3d* optim_view,
                                 double* optim)
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
        act_map::addPoseInfoBearingGlobal(
            Twc, pws.col(static_cast<int>(pt_i)), nullptr, &info_pose_i);
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
}
}
