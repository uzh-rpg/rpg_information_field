#pragma once

#include "act_map/quad_info_factors.h"
#include "act_map/quad_trace_factors.h"

#include "act_map/voxblox/utils/layer_utils.h"
#include "act_map/voxblox/utils/voxel_utils.h"

#include "act_map/conversion.h"
#include "act_map/voxel_traits.h"

namespace act_map
{
template <typename T1, typename T2, typename T3>
struct QuadraticFactorVoxel
{
  static const size_t kSerializationSize;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using K1Type = T1;
  using K2Type = T2;
  using K3Type = T3;

  using OutMatType = K3Type;

  QuadraticFactorVoxel()
  {
    K1.setZero();
    K2.setZero();
    K3.setZero();
  }

  inline size_t serializationSize() const
  {
    return kSerializationSize;
  }

  inline size_t numCoefs() const
  {
    return K1.size() + K2.size() + K3.size();
  }

  bool isTheSame(const QuadraticFactorVoxel& v, const double tol = 1e-8) const
  {
    bool is_the_same = true;
    is_the_same &= (K1 - v.K1).cwiseAbs().maxCoeff() < tol;
    is_the_same &= (K2 - v.K2).cwiseAbs().maxCoeff() < tol;
    is_the_same &= (K3 - v.K3).cwiseAbs().maxCoeff() < tol;
    return is_the_same;
  }

  template <typename intT>
  void serializeToIntegers(std::vector<intT>* data) const
  {
    CHECK_NOTNULL(data);
    static size_t kNumInt = numCoefs();
    data->clear();
    data->resize(kNumInt);
    size_t data_cnt = 0;
    for (int ri = 0; ri < K1.rows(); ri++)
    {
      for (int ci = 0; ci < K1.cols(); ci++)
      {
        serializeElem<intT>(K1(ri, ci), &((*data)[data_cnt++]));
      }
    }
    for (int ri = 0; ri < K2.rows(); ri++)
    {
      for (int ci = 0; ci < K2.cols(); ci++)
      {
        serializeElem<intT>(K2(ri, ci), &((*data)[data_cnt++]));
      }
    }
    for (int ri = 0; ri < K3.rows(); ri++)
    {
      for (int ci = 0; ci < K3.cols(); ci++)
      {
        serializeElem<intT>(K3(ri, ci), &((*data)[data_cnt++]));
      }
    }
    CHECK_EQ(kNumInt, data_cnt);
  }

  template <typename intT>
  void deserializeFromIntegers(const std::vector<intT>& data, const size_t s)
  {
    static size_t kNumInt = numCoefs();
    CHECK_LE(kNumInt, data.size() - s);
    size_t data_cnt = s;
    for (int ri = 0; ri < K1.rows(); ri++)
    {
      for (int ci = 0; ci < K1.cols(); ci++)
      {
        deserializeElem<intT>(data[data_cnt++], &(K1(ri, ci)));
      }
    }
    for (int ri = 0; ri < K2.rows(); ri++)
    {
      for (int ci = 0; ci < K2.cols(); ci++)
      {
        deserializeElem<intT>(data[data_cnt++], &(K2(ri, ci)));
      }
    }
    for (int ri = 0; ri < K3.rows(); ri++)
    {
      for (int ci = 0; ci < K3.cols(); ci++)
      {
        deserializeElem<intT>(data[data_cnt++], &(K3(ri, ci)));
      }
    }
  }

  T1 K1;
  T2 K2;
  T3 K3;
};

using QuadInfoVoxel = QuadraticFactorVoxel<InfoK1, InfoK2, InfoK3>;
using QuadInfoVoxelPtr = std::shared_ptr<QuadInfoVoxel>;
using QuadTraceVoxel = QuadraticFactorVoxel<TraceK1, TraceK2, TraceK3>;
using QuadTraceVoxelPtr = std::shared_ptr<QuadTraceVoxel>;
using QuadInfoVoxelConstPtr = std::shared_ptr<const QuadInfoVoxel>;
using QuadTraceVoxelConstPtr = std::shared_ptr<const QuadTraceVoxel>;

}  // namespace act_map

namespace act_map
{
namespace traits
{
// clang-format off
template <>
struct is_quad_vox<QuadInfoVoxel> : std::true_type { };
template <>
struct is_quad_vox<QuadTraceVoxel> : std::true_type { };
template <>
struct is_info_vox<QuadInfoVoxel> : std::true_type { };
template <>
struct is_trace_vox<QuadTraceVoxel> : std::true_type { };
// clang-format on
}
}

namespace act_map
{
namespace voxblox
{
namespace utils
{
template <>
inline bool isSameVoxel(const act_map::QuadTraceVoxel& A,
                        const act_map::QuadTraceVoxel& B)
{
  return A.isTheSame(B);
}
template <>
inline bool isSameVoxel(const act_map::QuadInfoVoxel& A,
                        const act_map::QuadInfoVoxel& B)
{
  return A.isTheSame(B);
}

}  // namespace utils

namespace voxel_types
{
extern const std::string kQuadTrace;
extern const std::string kQuadInfo;
}  // namespace voxel_types

template <>
inline std::string getVoxelType<act_map::QuadTraceVoxel>()
{
  return voxel_types::kQuadTrace;
}
template <>
inline std::string getVoxelType<act_map::QuadInfoVoxel>()
{
  return voxel_types::kQuadInfo;
}

// for serialization
template <>
inline void mergeVoxelAIntoVoxelB(const act_map::QuadInfoVoxel& voxel_A,
                                  act_map::QuadInfoVoxel* voxel_B)
{
  LOG(FATAL) << "Merge is not supported for InfoVoxel";
}

template <>
inline void mergeVoxelAIntoVoxelB(const act_map::QuadTraceVoxel& voxel_A,
                                  act_map::QuadTraceVoxel* voxel_B)
{
  LOG(FATAL) << "Merge is not supported for TraceVoxel";
}

}  // namespace voxblox
}  // namespace voxblox
