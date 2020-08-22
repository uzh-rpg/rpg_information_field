#pragma once

#include "act_map/internal/operators.h"
#include "act_map/quad_info_factors.h"
#include "act_map/quad_trace_factors.h"

// These functions directly operate on the invariant kernels instead of voxels
// Convenient for testing
namespace act_map
{
template <typename T1, typename T2, typename T3>
void assignToFactor(const Eigen::Vector3d& pw,
                    const Eigen::Vector3d& twc,
                    T1* K1,
                    T2* K2,
                    T3* K3)
{
  manipulateFactorSingle<internal::blockEqual>(pw, twc, K1, K2, K3);
}

template <typename T1, typename T2, typename T3>
void addToFactor(const Eigen::Matrix3Xd& pws,
                 const Eigen::Vector3d& twc,
                 T1* K1,
                 T2* K2,
                 T3* K3)
{
  for (int i = 0; i < pws.cols(); i++)
  {
    manipulateFactorSingle<internal::blockPlusEqual>(
        pws.col(i), twc, K1, K2, K3);
  }
}

template <typename T1, typename T2, typename T3>
void substractFromFactor(const Eigen::Matrix3Xd& pws,
                         const Eigen::Vector3d& twc,
                         T1* K1,
                         T2* K2,
                         T3* K3)
{
  for (int i = 0; i < pws.cols(); i++)
  {
    manipulateFactorSingle<internal::blockMinusEqual>(
        pws.col(i), twc, K1, K2, K3);
  }
}

template <typename T1, typename T2, typename T3>
void constructFactorBatch(const Eigen::Matrix3Xd& pws,
                          const Eigen::Vector3d& twc,
                          T1* K1,
                          T2* K2,
                          T3* K3)
{
  const int kNpts = pws.cols();
  CHECK_GT(kNpts, 0);
  assignToFactor(pws.col(0), twc, K1, K2, K3);
  addToFactor(pws.block(0, 1, 3, kNpts - 1), twc, K1, K2, K3);
}
}
