//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include "act_map/internal/operators.h"
#include "act_map/info_kernels.h"
#include "act_map/trace_kernels.h"

namespace act_map
{

template <typename T1, typename T2, typename T3>
void assignToKernel(const Eigen::Vector3d& pw,
                    const Eigen::Vector3d& twc,
                    T1* K1,
                    T2* K2,
                    T3* K3)
{
  manipulateKernelSingle<internal::blockEqual>(pw, twc, K1, K2, K3);
}

template <typename T1, typename T2, typename T3>
void addToKernel(const Eigen::Matrix3Xd& pws,
                 const Eigen::Vector3d& twc,
                 T1* K1,
                 T2* K2,
                 T3* K3)
{
  for (int i = 0; i < pws.cols(); i++)
  {
    manipulateKernelSingle<internal::blockPlusEqual>(
        pws.col(i), twc, K1, K2, K3);
  }
}

template <typename T1, typename T2, typename T3>
void substractFromKernel(const Eigen::Matrix3Xd& pws,
                         const Eigen::Vector3d& twc,
                         T1* K1,
                         T2* K2,
                         T3* K3)
{
  for (int i = 0; i < pws.cols(); i++)
  {
    manipulateKernelSingle<internal::blockMinusEqual>(
        pws.col(i), twc, K1, K2, K3);
  }
}

template <typename T1, typename T2, typename T3>
void constructKernelBatch(const Eigen::Matrix3Xd& pws,
                          const Eigen::Vector3d& twc,
                          T1* K1,
                          T2* K2,
                          T3* K3)
{
  const int kNpts = pws.cols();
  CHECK_GT(kNpts, 0);
  assignToKernel(pws.col(0), twc, K1, K2, K3);
  addToKernel(pws.block(0, 1, 3, kNpts - 1), twc, K1, K2, K3);
}
}
