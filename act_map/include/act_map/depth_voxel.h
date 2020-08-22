#pragma once

#include <Eigen/Core>

#include <opencv2/core.hpp>

#include <vi_utils/cam_min.h>
#include <rpg_common/pose.h>

#include "act_map/voxblox/core/voxel.h"
#include "act_map/voxblox/utils/layer_utils.h"
#include "act_map/conversion.h"

namespace act_map
{
enum class VisStatus : size_t
{
  kUnknownDepth = 0,
  kNotCovered = 1,
  kVisibile = 2,
  kOccluded = 3
};

using VisStatusVec = std::vector<VisStatus>;
using VisIdx = std::set<size_t>;

class DepthVoxel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using FloatType = float;
  using Vec3 = Eigen::Matrix<FloatType, 3, 1>;
  using Vec3Vec = rpg::Aligned<std::vector, Vec3>;
  using DepthMat = Eigen::Matrix<FloatType, -1, -1>;
  using MinMaxD = std::array<FloatType, 2>;
  static constexpr size_t kMinDIdx = 0;
  static constexpr size_t kMaxDIdx = 1;
  static constexpr int kOpenCVType = CV_32F;
  static constexpr FloatType kInfD = std::numeric_limits<FloatType>::infinity();

  DepthVoxel()
  {
    initialized_ = false;
  }

  DepthVoxel(const rpg::Position& pos, const FloatType deg_step)
  {
    init(pos, deg_step);
  }

  virtual ~DepthVoxel();

  inline FloatType idxToLatitudeDeg(const int ridx) const
  {
    return ridx * step_deg_ + static_cast<FloatType>(-90.0);
  }

  inline FloatType idxToLongtitude(const int cidx) const
  {
    return cidx * step_deg_ + static_cast<FloatType>(-180.0);
  }

  inline int latitudeDegToIdx(const FloatType deg) const
  {
    return static_cast<int>(std::round((deg - (-90.0)) / step_deg_));
  }

  inline int longtitudeDegToIdx(const FloatType deg) const
  {
    return static_cast<int>(std::round((deg - (-180.0)) / step_deg_));
  }

  inline std::tuple<FloatType, FloatType>
  vecWToLatiLongti(const Vec3& vec_w) const
  {
    const FloatType lat_deg =
        std::asin(vec_w.z() / vec_w.norm()) * 180.0 / M_PI;
    const FloatType longt_deg = std::atan2(vec_w.y(), vec_w.x()) * 180.0 / M_PI;

    return std::tuple<FloatType, FloatType>(lat_deg, longt_deg);
  }

  inline void latiLongtiToVecW(const FloatType lati_deg,
                               const FloatType longti_deg, Vec3* vec_w) const
  {
    FloatType lati_rad = lati_deg / 180.0 * M_PI;
    FloatType longti_rad = longti_deg / 180.0 * M_PI;
    vec_w->z() = std::sin(lati_rad);
    FloatType yzss = std::cos(lati_rad);
    vec_w->x() = yzss * std::cos(longti_rad);
    vec_w->y() = yzss * std::sin(longti_rad);
  }

  inline std::tuple<int, int> vecWorldToDepthMapIndices(const Vec3& vec_w) const
  {
    FloatType lat_deg, longt_deg;
    std::tie(lat_deg, longt_deg) = vecWToLatiLongti(vec_w);

    const int ridx = latitudeDegToIdx(lat_deg);
    const int cidx = longtitudeDegToIdx(longt_deg);

    return std::tuple<int, int>(ridx, cidx);
  }

  inline void depthMapIndicesToVecWorld(const int ridx, const int cidx,
                                        Vec3* vec_w) const
  {
    FloatType lat = idxToLatitudeDeg(ridx);
    FloatType lon = idxToLongtitude(cidx);
    latiLongtiToVecW(lat, lon, vec_w);
  }

  inline size_t serializationSize() const
  {
    CHECK(initialized());
    return static_cast<size_t>(pos_.size() + 1 + ray_depth_map_.size());
  }

  inline void assignDepth(const int ridx, const int cidx, const FloatType val)
  {
    if (ridx == 0 || ridx == (ray_depth_map_.rows() - 1))
    {
      ray_depth_map_.row(ridx).setConstant(val);
    }
    else
    {
      ray_depth_map_(ridx, cidx) = val;
    }
  }

  // query functions
  inline FloatType queryDepthWorldVector(const Vec3& vec_w,
                                         const bool neighbor = false) const
  {
    int ridx, cidx;
    std::tie(ridx, cidx) = vecWorldToDepthMapIndices(vec_w);

    if (!neighbor)
    {
      return ray_depth_map_(ridx, cidx);
    }
    else
    {
      FloatType depth_sum = 0;
      size_t depth_cnt = 0;
      const int min_ridx = (ridx > 0 ? ridx - 1 : ridx);
      const int max_ridx = (ridx < ray_depth_map_.rows() - 1 ? ridx + 1 : ridx);
      const int min_cidx = (cidx > 0 ? cidx - 1 : cidx);
      const int max_cidx = (cidx < ray_depth_map_.cols() - 1 ? cidx + 1 : cidx);
      for (int ridx_i = min_ridx; ridx_i <= max_ridx; ridx_i++)
      {
        for (int cidx_i = min_cidx; cidx_i <= max_cidx; cidx_i++)
        {
          if (ray_depth_map_(ridx_i, cidx_i) != kInfD)
          {
            depth_sum += ray_depth_map_(ridx_i, cidx_i);
            depth_cnt++;
          }
        }
      }

      if (depth_cnt > 0)
      {
        return depth_sum / depth_cnt;
      }
      else
      {
        return ray_depth_map_(ridx, cidx);
      }
    }
  }

  inline FloatType queryDepthWorldPoint(const Eigen::Vector3d& pw) const
  {
    return queryDepthWorldVector(pw.cast<FloatType>() - pos_);
  }

  // set function
  inline void setDepthFromWorldVector(const Vec3& vec_w)
  {
    FloatType ray_d = static_cast<FloatType>(vec_w.norm());

    int ridx, cidx;
    std::tie(ridx, cidx) = vecWorldToDepthMapIndices(vec_w);
    assignDepth(ridx, cidx, ray_d);
  }

  inline void setDepthFromWorldPoint(const Eigen::Vector3d& pw)
  {
    setDepthFromWorldVector(pw.cast<FloatType>() - pos_);
  }

  inline void setDepthMap(const FloatType val)
  {
    CHECK(initialized_);
    ray_depth_map_.setConstant(val);
  }

  void setFromDepthImages(const vi_utils::PinholeCamVec& cam_vec,
                          const std::vector<cv::Mat>& zd_imgs,
                          const rpg_common::PoseVec& pose_vec,
                          const bool z_depth = true);

  // check occluded / visibility
  inline bool isPointOccluded(const Eigen::Vector3d& pw) const
  {
    return queryVisibilityPoint(pw) == VisStatus::kOccluded;
  }

  inline bool isVectorWorldOccluded(const Vec3& vec_w) const
  {
    return queryVisibilityWorldVec(vec_w) == VisStatus::kOccluded;
  }

  inline VisStatus queryVisibilityPoint(const Eigen::Vector3d& pw) const
  {
    return queryVisibilityWorldVec(pw.cast<FloatType>() - pos_);
  }

  inline VisStatus queryVisibilityWorldVec(const Vec3& vec_w) const
  {
    FloatType stored_d = queryDepthWorldVector(vec_w);
    if (stored_d == kInfD)
    {
      return VisStatus::kUnknownDepth;
    }
    else
    {
      FloatType query_d = static_cast<FloatType>(vec_w.norm());
      if (query_d > (stored_d + static_cast<FloatType>(0.01)))
      {
        return VisStatus::kOccluded;
      }
      else
      {
        return VisStatus::kVisibile;
      }
    }
  }

  // directly get indices without using the above functions
  // - include min max distance check
  void getVisibleIdxFromPoints(const rpg::PositionVec& points, VisIdx* vis_idx,
                               const MinMaxD& min_max_dept = MinMaxD{
                                   -kInfD, kInfD }) const;

  // misc
  size_t numValidDepths() const;
  size_t numMissingDepths() const;
  inline size_t numTotalDepths() const
  {
    return static_cast<size_t>(ray_depth_map_.size());
  }

  friend std::ostream& operator<<(std::ostream& os, const DepthVoxel& ds);

  void resetDepthmap()
  {
    ray_depth_map_.setConstant(kInfD);
  }

  void printMinMaxDepthMap() const;

  // for full coverage
  static void generateSampleTws(const rpg_common::Pose& Twc,
                                const bool inc_top_down,
                                rpg_common::PoseVec* Tcs);

  void queryDepthImage(const vi_utils::PinholeCamVec& cam_vec,
                       const rpg_common::PoseVec& pose_vec,
                       std::vector<cv::Mat>* dimgs) const;

  void getDepthPoints(rpg::PositionVec* points_w) const;

  void init(const rpg_common::Position& pos, const double deg_step);

  //
  inline bool initialized() const
  {
    return initialized_;
  }

  inline const DepthMat& rayDepthMatCRef() const
  {
    return ray_depth_map_;
  }

  inline DepthMat& rayDepthMatRef()
  {
    return ray_depth_map_;
  }

  template <typename intT>
  void serializeToIntegers(std::vector<intT>* data) const;

  template <typename intT>
  void deserializeFromIntegers(const std::vector<intT>& data, const size_t s);

  bool isTheSame(const DepthVoxel& v) const;

  inline Eigen::Vector3d center() const
  {
    return Eigen::Vector3d(pos_.cast<double>());
  }

  inline double getStepDeg() const
  {
    return static_cast<double>(step_deg_);
  }

  template <typename T>
  static inline MinMaxD getMinMaxD(const T min, const T max)
  {
    return MinMaxD{ static_cast<FloatType>(min), static_cast<FloatType>(max) };
  }

private:
  void getVisibleIdxInternal(const Vec3Vec& points, VisIdx* vis_idx,
                             const MinMaxD& min_max_dept = MinMaxD{
                                 -kInfD, kInfD }) const;

  Eigen::Matrix<FloatType, 3, 1> pos_;
  FloatType step_deg_;
  DepthMat ray_depth_map_;

  bool initialized_ = false;
};

using DepthVoxelPtr = std::shared_ptr<DepthVoxel>;

template <typename intT>
void DepthVoxel::serializeToIntegers(std::vector<intT>* data) const
{
  CHECK(initialized());
  const size_t kNumInt = this->serializationSize();
  data->clear();
  data->resize(kNumInt);

  size_t data_cnt = 0;
  for (int i = 0; i < pos_.size(); i++)
  {
    serializeElem<intT>(pos_(i), &((*data)[data_cnt++]));
  }
  serializeElem<intT>(step_deg_, &((*data)[data_cnt++]));
  for (int ri = 0; ri < ray_depth_map_.rows(); ri++)
  {
    for (int ci = 0; ci < ray_depth_map_.cols(); ci++)
    {
      serializeElem<intT>(ray_depth_map_(ri, ci), &((*data)[data_cnt++]));
    }
  }
  CHECK_EQ(data_cnt, kNumInt);
}

template <typename intT>
void DepthVoxel::deserializeFromIntegers(const std::vector<intT>& data,
                                         const size_t s)
{
  size_t data_cnt = s;
  double tmp;
  for (int i = 0; i < 3; i++)
  {
    deserializeElem<intT>(data[data_cnt++], &tmp);
    pos_(i) = static_cast<FloatType>(tmp);
  }
  deserializeElem<intT>(data[data_cnt++], &tmp);
  step_deg_ = static_cast<FloatType>(tmp);

  init(pos_.cast<double>(), static_cast<double>(step_deg_));
  const size_t kNumInt = this->serializationSize();
  CHECK_LE(kNumInt, data.size() - s);

  for (int ri = 0; ri < ray_depth_map_.rows(); ri++)
  {
    for (int ci = 0; ci < ray_depth_map_.cols(); ci++)
    {
      deserializeElem<intT>(data[data_cnt++], &(tmp));
      ray_depth_map_(ri, ci) = static_cast<FloatType>(tmp);
    }
  }
}

}  // namespace act_map

// specialization
namespace act_map
{
namespace voxblox
{
namespace utils
{
template <>
inline bool isSameVoxel(const act_map::DepthVoxel& A,
                        const act_map::DepthVoxel& B)
{
  return A.isTheSame(B);
}

}  // namespace utils
namespace voxel_types
{
const std::string kDepthVoxel = "depth_voxel";
}
template <>
inline std::string getVoxelType<act_map::DepthVoxel>()
{
  return voxel_types::kDepthVoxel;
}

}  // namespace voxblox
}  // namespace act_map
