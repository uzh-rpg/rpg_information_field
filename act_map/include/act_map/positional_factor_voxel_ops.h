#pragma once

#include "act_map/voxblox/core/block.h"
#include "act_map/internal/operators.h"

#include "act_map/quad_trace_factors.h"
#include "act_map/quad_info_factors.h"

#include "act_map/positional_factor_voxel.h"
#include "act_map/common.h"

namespace act_map
{
// this should be the interface for different positional factor voxel types to implememnt
// workaround the lack the function partial specialization in c++11
template <typename F, typename KVType>
inline typename std::enable_if<traits::is_quad_vox<KVType>::value>::type
manipulateFactorVoxelSingle(const Eigen::Vector3d& pw,
                            const Eigen::Vector3d& vox_twc,
                            KVType* vox)
{
  act_map::manipulateFactorSingle<F>(
      pw, vox_twc, &(vox->K1), &(vox->K2), &(vox->K3));
}

template <typename F, typename KVType>
inline typename std::enable_if<traits::is_vis_vox<KVType>::value>::type
manipulateFactorVoxelSingle(const Eigen::Vector3d& pw,
                            const Eigen::Vector3d& vox_twc,
                            KVType* vox)
{
  vox->template updateFactorSingle<F>(pw, vox_twc);
}
} // implementations of different positional factor voxel types

namespace act_map
{
// convenient interface with linear index
template <typename F, typename KVType>
void manipulateFactorVoxelSingle(voxblox::Block<KVType>& blk,
                                 const size_t lin_idx,
                                 const Eigen::Vector3d& pw)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);

  manipulateFactorVoxelSingle<F, KVType>(pw, vox_twc, &vox);
}

template <typename KVType>
void assignToFactorVoxel(const Eigen::Vector3d& pw,
                         const Eigen::Vector3d& vox_twc,
                         KVType* vox)
{
  manipulateFactorVoxelSingle<internal::blockEqual, KVType>(pw, vox_twc, vox);
}

template <typename KVType>
void assignToFactorVoxel(voxblox::Block<KVType>& blk,
                         const size_t lin_idx,
                         const Eigen::Vector3d& pw)
{
  manipulateFactorVoxelSingle<internal::blockEqual, KVType>(blk, lin_idx, pw);
}

// add
// by pointer
template <typename KVType>
void addToFactorVoxel(const Eigen::Matrix3Xd& pws,
                      const Eigen::Vector3d& vox_twc,
                      KVType* vox)
{
  for (int i = 0; i < pws.cols(); i++)
  {
    manipulateFactorVoxelSingle<internal::blockPlusEqual, KVType>(
        pws.col(i), vox_twc, vox);
  }
}

template <typename KVType>
void addToFactorVoxel(const act_map::Vec3dVec& pws,
                      const Eigen::Vector3d& vox_twc,
                      KVType* vox)
{
  for (size_t i = 0; i < pws.size(); i++)
  {
    manipulateFactorVoxelSingle<internal::blockPlusEqual, KVType>(
        pws[i], vox_twc, vox);
  }
}

template <typename KVType>
void addToFactorVoxel(const act_map::Vec3dVec& pws,
                      const Eigen::Vector3d& vox_twc,
                      const size_t s,
                      const size_t e,
                      KVType* vox)
{
  for (size_t i = s; i < e; i++)
  {
    manipulateFactorVoxelSingle<internal::blockPlusEqual, KVType>(
        pws[i], vox_twc, vox);
  }
}

// by linear index
template <typename KVType>
void addToFactorVoxel(voxblox::Block<KVType>& blk,
                      const size_t lin_idx,
                      const Eigen::Matrix3Xd& pws)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);
  addToFactorVoxel<KVType>(pws, vox_twc, &vox);
}

template <typename KVType>
void addToFactorVoxel(voxblox::Block<KVType>& blk,
                      const size_t lin_idx,
                      const act_map::Vec3dVec& pws)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);
  addToFactorVoxel<KVType>(pws, vox_twc, &vox);
}

template <typename KVType>
void addToFactorVoxel(voxblox::Block<KVType>& blk,
                      const size_t lin_idx,
                      const act_map::Vec3dVec& pws,
                      const size_t s,
                      const size_t e)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);
  addToFactorVoxel<KVType>(pws, vox_twc, s, e, &vox);
}

// substract
template <typename KVType>
void substractFromFactorVoxel(const Eigen::Matrix3Xd& pws,
                              const Eigen::Vector3d& vox_twc,
                              KVType* vox)
{
  for (int i = 0; i < pws.cols(); i++)
  {
    manipulateFactorVoxelSingle<internal::blockMinusEqual, KVType>(
        pws.col(i), vox_twc, vox);
  }
}

template <typename KVType>
void substractFromFactorVoxel(voxblox::Block<KVType>& blk,
                              const size_t lin_idx,
                              const Eigen::Matrix3Xd& pws)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);
  substractFromFactorVoxel<KVType>(pws, vox_twc, &vox);
}

template <typename KVType>
void constructFactorVoxelBatch(const Eigen::Matrix3Xd& pws,
                               const Eigen::Vector3d& vox_twc,
                               KVType* vox)
{
  const long int kNpts = pws.cols();
  assignToFactorVoxel(pws.col(0), vox_twc, vox);
  addToFactorVoxel(pws.block(0, 1, 3, kNpts - 1), vox_twc, vox);
}

template <typename KVType>
void constructFactorVoxelBatch(voxblox::Block<KVType>& blk,
                               const size_t lin_idx,
                               const Eigen::Matrix3Xd& pws)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);
  constructFactorVoxelBatch(pws, vox_twc, &vox);
}

template <typename KVType>
void constructFactorVoxelBatch(voxblox::Block<KVType>& blk,
                               const size_t lin_idx,
                               const act_map::Vec3dVec& pws)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);
  assignToFactorVoxel(pws[0], vox_twc, &vox);
  addToFactorVoxel(pws, vox_twc, 1, pws.size(), &vox);
}
}
