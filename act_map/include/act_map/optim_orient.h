#pragma once

#include "act_map/quadratic_factor_ops.h"
#include "act_map/positional_factor_voxel_ops.h"
#include "act_map/info_utils.h"
#include "act_map/common.h"

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
                                 const double k1, const double k2,
                                 const double k3, const act_map::InfoK1& K1,
                                 const act_map::InfoK2& K2,
                                 const act_map::InfoK3& K3,
                                 const act_map::InfoMetricType type,
                                 Eigen::Vector3d* optim_view, double* optim);

void getOptimViewFromGPInfoVoxel(const rpg::RotationVec& rot_samples,
                                 const GPInfoVoxel& vox,
                                 const act_map::InfoMetricType type,
                                 Eigen::Vector3d* optim_view, double* optim);

void getOptimViewFromGPTraceVoxel(const rpg::RotationVec& rot_samples,
                                  const GPTraceVoxel& vox,
                                  Eigen::Vector3d* optim_view, double* optim);

void getOptimViewFromTraceKernels(const rpg::RotationVec& rot_samples,
                                  const double k1, const double k2,
                                  const double k3, const act_map::TraceK1& K1,
                                  const act_map::TraceK2& K2,
                                  const act_map::TraceK3& K3,
                                  Eigen::Vector3d* optim_view, double* optim);

void getWorstViewFromTraceKernels(const rpg::RotationVec& rot_samples,
                                  const double k1, const double k2,
                                  const double k3, const act_map::TraceK1& K1,
                                  const act_map::TraceK2& K2,
                                  const act_map::TraceK3& K3,
                                  Eigen::Vector3d* worst_view, double* worst);

// closed form unconstrained minimal
// NOT WORKING YET
void getMinTraceDirectionFromTraceKernelsUnconstrained(
    const double k1, const double k2, const double k3,
    const act_map::TraceK1& K1, const act_map::TraceK2& K2,
    const act_map::TraceK3& K3, Eigen::Vector3d* view, double* val);

void getOptimalViewFromExactInfo(const rpg::RotationVec& rot_samples,
                                 const Eigen::Vector3d& twc,
                                 const Eigen::Matrix3Xd& pws,
                                 const vi_utils::PinholeCam& cam,
                                 const act_map::InfoMetricType& type,
                                 Eigen::Vector3d* optim_view, double* optim);

// get optimal orientations for layers
void getBestViewsClosed(const act_map::QuadTraceLayer& tl,
                        const voxblox::BlockIndexList& blk_idxs,
                        const double k1,
                        const double k2,
                        const int samples_per_side,
                        act_map::Vec3dVec* vox_cs,
                        act_map::Vec3dVec* best_views,
                        voxblox::LongIndexVector* global_idxs);

void getBestViewsSample(const act_map::QuadTraceLayer& tl,
                        const voxblox::BlockIndexList& blk_idxs,
                        const double k1,
                        const double k2,
                        const double k3,
                        const int samples_per_side,
                        act_map::Vec3dVec* vox_cs,
                        act_map::Vec3dVec* best_views,
                        std::vector<double>* values,
                        voxblox::LongIndexVector* global_idxs);

void getBestViewsSample(const act_map::GPTraceLayer& tl,
                        const voxblox::BlockIndexList& blk_idxs,
                        const int samples_per_side,
                        act_map::Vec3dVec* vox_cs,
                        act_map::Vec3dVec* best_views,
                        std::vector<double>* values,
                        voxblox::LongIndexVector* global_idxs);
}
}
