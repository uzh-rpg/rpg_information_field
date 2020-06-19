//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include "act_map/kernel_ops.h"
#include "act_map/info_utils.h"

namespace vi_utils
{
class PinholeCam;
}

namespace act_map
{
namespace optim_orient
{
struct OptimOrientRes
{
  // this struct stores the optim orientation at positions
  std::string name_;
  rpg::PositionVec positions_;
  rpg::PositionVec optim_views_;
  Eigen::VectorXd optim_vals_;

  OptimOrientRes(const size_t N, const std::string& name)
    : name_(name), positions_(N), optim_views_(N)
  {
    optim_vals_.resize(N);
  }

  OptimOrientRes()
  {
  }

  void resize(const size_t N);
  void save(const std::string& abs_save_dir, const bool save_pos = false) const;
};

void getOptimViewFromInfoKernels(const rpg::RotationVec& rot_samples,
                                 const double k1,
                                 const double k2,
                                 const double k3,
                                 const act_map::InfoK1& K1,
                                 const act_map::InfoK2& K2,
                                 const act_map::InfoK3& K3,
                                 const act_map::InfoMetricType type,
                                 Eigen::Vector3d* optim_view,
                                 double* optim);

void getOptimViewFromTraceKernels(const rpg::RotationVec& rot_samples,
                                  const double k1,
                                  const double k2,
                                  const double k3,
                                  const act_map::TraceK1& K1,
                                  const act_map::TraceK2& K2,
                                  const act_map::TraceK3& K3,
                                  Eigen::Vector3d* optim_view,
                                  double* optim);

void getWorstViewFromTraceKernels(const rpg::RotationVec& rot_samples,
                                  const double k1,
                                  const double k2,
                                  const double k3,
                                  const act_map::TraceK1& K1,
                                  const act_map::TraceK2& K2,
                                  const act_map::TraceK3& K3,
                                  Eigen::Vector3d* worst_view,
                                  double* worst);

// closed form unconstrained minimal
void getMinTraceDirectionFromTraceKernelsUnconstrained(
    const double k1,
    const double k2,
    const double k3,
    const act_map::TraceK1& K1,
    const act_map::TraceK2& K2,
    const act_map::TraceK3& K3,
    Eigen::Vector3d* view,
    double* val);

void getOptimalViewFromExactInfo(const rpg::RotationVec& rot_samples,
                                 const Eigen::Vector3d& twc,
                                 const Eigen::Matrix3Xd& pws,
                                 const vi_utils::PinholeCam& cam,
                                 const act_map::InfoMetricType& type,
                                 Eigen::Vector3d* optim_view,
                                 double* optim);
}
}
