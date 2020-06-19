//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include "act_map/info_kernels.h"
#include "act_map/trace_kernels.h"

#include <voxblox/utils/layer_utils.h>

namespace act_map
{
template <typename T1, typename T2, typename T3>
struct KernelVoxel
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using K1Type = T1;
  using K2Type = T2;
  using K3Type = T3;

  KernelVoxel()
  {
    K1.setZero();
    K2.setZero();
    K3.setZero();
  }

  inline size_t numCoefs() const
  {
    return K1.size() + K2.size() + K3.size();
  }

  bool isTheSame(const KernelVoxel& v) const
  {
    bool is_the_same = true;
    constexpr double kTolerance = 1e-10;
    is_the_same &= (K1 - v.K1).cwiseAbs().maxCoeff() < kTolerance;
    is_the_same &= (K2 - v.K2).cwiseAbs().maxCoeff() < kTolerance;
    is_the_same &= (K3 - v.K3).cwiseAbs().maxCoeff() < kTolerance;
    return is_the_same;
  }

  template <typename intT>
  void serializeToIntegers(std::vector<intT>* data) const
  {
    CHECK_NOTNULL(data);
    static size_t kNumInt = numCoefs();
    data->clear();
    data->reserve(kNumInt);
    for (int ri = 0; ri < K1.rows(); ri++)
    {
      for (int ci = 0; ci < K1.cols(); ci++)
      {
        const intT* int_v_ptr = reinterpret_cast<const intT*>(&(K1(ri, ci)));
        data->push_back(*int_v_ptr);
      }
    }
    for (int ri = 0; ri < K2.rows(); ri++)
    {
      for (int ci = 0; ci < K2.cols(); ci++)
      {
        const intT* int_v_ptr = reinterpret_cast<const intT*>(&(K2(ri, ci)));
        data->push_back(*int_v_ptr);
      }
    }
    for (int ri = 0; ri < K3.rows(); ri++)
    {
      for (int ci = 0; ci < K3.cols(); ci++)
      {
        const intT* int_v_ptr = reinterpret_cast<const intT*>(&(K3(ri, ci)));
        data->push_back(*int_v_ptr);
      }
    }
    CHECK_EQ(kNumInt, data->size());
  }
  template <typename intT>
  void deserializeFromIntegers(const std::vector<intT>& data, const size_t s)
  {
    static size_t kNumInt = numCoefs();
    CHECK_LE(kNumInt, data.size() - s);
    size_t data_cnt = s;
    size_t int_size = sizeof(data[0]);
    for (int ri = 0; ri < K1.rows(); ri++)
    {
      for (int ci = 0; ci < K1.cols(); ci++)
      {
        memcpy(&(K1(ri, ci)), &data[data_cnt++], int_size);
      }
    }
    for (int ri = 0; ri < K2.rows(); ri++)
    {
      for (int ci = 0; ci < K2.cols(); ci++)
      {
        memcpy(&(K2(ri, ci)), &data[data_cnt++], int_size);
      }
    }
    for (int ri = 0; ri < K3.rows(); ri++)
    {
      for (int ci = 0; ci < K3.cols(); ci++)
      {
        memcpy(&(K3(ri, ci)), &data[data_cnt++], int_size);
      }
    }
  }

  T1 K1;
  T2 K2;
  T3 K3;
};

using InfoVoxel = KernelVoxel<InfoK1, InfoK2, InfoK3>;
using TraceVoxel = KernelVoxel<TraceK1, TraceK2, TraceK3>;
using InfoVoxelPtr = std::shared_ptr<InfoVoxel>;
using TraceVoxelPtr = std::shared_ptr<TraceVoxel>;
using InfoVoxelConstPtr = std::shared_ptr<const InfoVoxel>;
using TraceVoxelConstPtr = std::shared_ptr<const TraceVoxel>;

template <typename T>
std::string getVoxelType()
{
  return voxblox::getVoxelType<T>();
}
namespace voxel_types = voxblox::voxel_types;
}

namespace voxblox
{
namespace utils
{
template <>
inline bool isSameVoxel(const act_map::TraceVoxel& A,
                        const act_map::TraceVoxel& B)
{
  return A.isTheSame(B);
}
template <>
inline bool isSameVoxel(const act_map::InfoVoxel& A,
                        const act_map::InfoVoxel& B)
{
  return A.isTheSame(B);
}

}  // utils

namespace voxel_types
{
const std::string kTraceKernel = "trace_kernel";
const std::string kInfoKernel = "info_kernel";
}  // voxel types

template <>
inline std::string getVoxelType<act_map::TraceVoxel>()
{
  return voxel_types::kTraceKernel;
}
template <>
inline std::string getVoxelType<act_map::InfoVoxel>()
{
  return voxel_types::kInfoKernel;
}

}  // voxblox
