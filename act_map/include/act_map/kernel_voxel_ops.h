//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <voxblox/core/block.h>
#include "act_map/internal/operators.h"
#include "act_map/trace_kernels.h"
#include "act_map/info_kernels.h"
#include "act_map/common.h"

namespace act_map
{
template <typename F, typename KVType>
void manipulateKernelVoxelSingle(const Eigen::Vector3d& pw,
                                 const Eigen::Vector3d& vox_twc,
                                 KVType* vox)
{
  manipulateKernelSingle<F>(pw, vox_twc, &(vox->K1), &(vox->K2), &(vox->K3));
}

template <typename F, typename KVType>
void manipulateKernelVoxelSingle(voxblox::Block<KVType>& blk,
                                 const size_t lin_idx,
                                 const Eigen::Vector3d& pw)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);

  manipulateKernelVoxelSingle<F, KVType>(pw, vox_twc, &vox);
}

template <typename KVType>
void assignToKernelVoxel(const Eigen::Vector3d& pw,
                         const Eigen::Vector3d& vox_twc,
                         KVType* vox)
{
  manipulateKernelVoxelSingle<internal::blockEqual, KVType>(pw, vox_twc, vox);
}

template <typename KVType>
void assignToKernelVoxel(voxblox::Block<KVType>& blk,
                         const size_t lin_idx,
                         const Eigen::Vector3d& pw)
{
  manipulateKernelVoxelSingle<internal::blockEqual, KVType>(blk, lin_idx, pw);
}

// add
// by pointer
template <typename KVType>
void addToKernelVoxel(const Eigen::Matrix3Xd& pws,
                      const Eigen::Vector3d& vox_twc,
                      KVType* vox)
{
  for (int i = 0; i < pws.cols(); i++)
  {
    manipulateKernelVoxelSingle<internal::blockPlusEqual, KVType>(
        pws.col(i), vox_twc, vox);
  }
}

template <typename KVType>
void addToKernelVoxel(const act_map::Vec3dVec& pws,
                      const Eigen::Vector3d& vox_twc,
                      KVType* vox)
{
  for (size_t i = 0; i < pws.size(); i++)
  {
    manipulateKernelVoxelSingle<internal::blockPlusEqual, KVType>(
        pws[i], vox_twc, vox);
  }
}

template <typename KVType>
void addToKernelVoxel(const act_map::Vec3dVec& pws,
                      const Eigen::Vector3d& vox_twc,
                      const size_t s,
                      const size_t e,
                      KVType* vox)
{
  for (size_t i = s; i < e; i++)
  {
    manipulateKernelVoxelSingle<internal::blockPlusEqual, KVType>(
        pws[i], vox_twc, vox);
  }
}

// by linear index
template <typename KVType>
void addToKernelVoxel(voxblox::Block<KVType>& blk,
                      const size_t lin_idx,
                      const Eigen::Matrix3Xd& pws)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);
  addToKernelVoxel<KVType>(pws, vox_twc, &vox);
}

template <typename KVType>
void addToKernelVoxel(voxblox::Block<KVType>& blk,
                      const size_t lin_idx,
                      const act_map::Vec3dVec& pws)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);
  addToKernelVoxel<KVType>(pws, vox_twc, &vox);
}

template <typename KVType>
void addToKernelVoxel(voxblox::Block<KVType>& blk,
                      const size_t lin_idx,
                      const act_map::Vec3dVec& pws,
                      const size_t s,
                      const size_t e)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);
  addToKernelVoxel<KVType>(pws, vox_twc, s, e, &vox);
}

// substract
template <typename KVType>
void substractFromKernelVoxel(const Eigen::Matrix3Xd& pws,
                              const Eigen::Vector3d& vox_twc,
                              KVType* vox)
{
  for (int i = 0; i < pws.cols(); i++)
  {
    manipulateKernelVoxelSingle<internal::blockMinusEqual, KVType>(
        pws.col(i), vox_twc, vox);
  }
}

template <typename KVType>
void substractFromKernelVoxel(voxblox::Block<KVType>& blk,
                              const size_t lin_idx,
                              const Eigen::Matrix3Xd& pws)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);
  substractFromKernelVoxel<KVType>(pws, vox_twc, &vox);
}

template <typename KVType>
void constructKernelVoxelBatch(voxblox::Block<KVType>& blk,
                               const size_t lin_idx,
                               const Eigen::Matrix3Xd& pws)
{
  const int kNpts = pws.cols();
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);
  assignToKernelVoxel(pws.col(0), vox_twc, &vox);
  addToKernelVoxel(pws.block(0, 1, 3, kNpts - 1), vox_twc, &vox);
}

template <typename KVType>
void constructKernelVoxelBatch(voxblox::Block<KVType>& blk,
                               const size_t lin_idx,
                               const act_map::Vec3dVec& pws)
{
  KVType& vox = blk.getVoxelByLinearIndex(lin_idx);
  Eigen::Vector3d vox_twc = blk.computeCoordinatesFromLinearIndex(lin_idx);
  assignToKernelVoxel(pws[0], vox_twc, &vox);
  addToKernelVoxel(pws, vox_twc, 1, pws.size(), &vox);
}
}
