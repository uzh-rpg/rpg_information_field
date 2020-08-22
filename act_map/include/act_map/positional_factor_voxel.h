#pragma once

#include "act_map/gp_vis_approximator.h"
#include "act_map/quadpoly_vis_approximator.h"

#include <iostream>

#include <vi_utils/vi_jacobians.h>
#include <rpg_common/timer.h>

#include "act_map/voxblox/utils/layer_utils.h"
#include "act_map/voxblox/utils/voxel_utils.h"
#include "act_map/internal/operators.h"
#include "act_map/info_utils.h"
#include "act_map/sampler.h"
#include "act_map/conversion.h"
#include "act_map/voxel_traits.h"

namespace act_map
{
template <typename T, int NumY>
class PositionalFactorVoxel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using VisApproxType = VisibilityApproximator<T>;
  using FactorType = Eigen::Matrix<double, Eigen::Dynamic, NumY>;
  using OutMatType = Eigen::Matrix<double, NumY, NumY>;
  constexpr static int kNumOut = NumY;

  PositionalFactorVoxel()
  {
    CHECK(vis_approx_) << "call setVisApprox first?";
    CHECK_GT(dim_ftrs_, 0) << "call setVisApprox first?";
    positional_factor_.resize(dim_ftrs_ * NumY, Eigen::NoChange);
    positional_factor_.setZero();
  }

  virtual ~PositionalFactorVoxel()
  {
  }

  static void resetVisApprox()
  {
    vis_approx_.reset();
    dim_ftrs_ = -1;
  }

  static void setVisApprox(const VisApproxPtr<T>& vis_ptr)
  {
    vis_approx_ = vis_ptr;
    dim_ftrs_ = vis_approx_->dimFeatureSpace();
  }

  // specific functions for different visibility approximations
  template <typename ST = T>
  static typename std::enable_if<
      std::is_same<ST, GPVisibilityApproximator>::value>::type
  setVisApproxFromFolderGP(const std::string& vis_dir)
  {
    GPVisApproxPtr gp_vis_ptr = std::make_shared<GPVisibilityApproximator>();
    gp_vis_ptr->load(vis_dir);
    setVisApprox(gp_vis_ptr);
  }

  template <typename ST = T>
  static typename std::enable_if<
      std::is_same<ST, QuadPolyVisApproximator>::value>::type
  setVisApproxQuadVisOpt(const QuadVisScoreOptions& quad_vis_opt)
  {
    QuadPolyVisApproxPtr quad_vis_ptr =
        std::make_shared<QuadPolyVisApproximator>(quad_vis_opt);
    setVisApprox(quad_vis_ptr);
  }

  static bool isVisApproxValid()
  {
    return vis_approx_ != nullptr;
  }

  static int dimFeatures()
  {
    return dim_ftrs_;
  }

  inline size_t serializationSize() const
  {
    CHECK_GT(dim_ftrs_, 0) << "call setVisApprox first?";
    return dim_ftrs_ * NumY * NumY + 2;
  }

  inline size_t numCoefs() const
  {
    CHECK_GT(dim_ftrs_, 0) << "call setVisApprox first?";
    return dim_ftrs_ * NumY * NumY;
  }

  const FactorType& getFactorConstRef() const
  {
    return positional_factor_;
  }

  FactorType& getFactorRef()
  {
    return positional_factor_;
  }

  void resetFactor()
  {
    positional_factor_.setZero();
  }

  static void resetTimer()
  {
    tbuild_unscaled = 0;
    tbuild_lm_ftrs = 0;
    tbuild_write_to_mat = 0;
    tquery_camz_ftrs = 0;
    tquery_write_to_mat = 0;
  }

  template <typename F>
  void updateFactorSingle(const Eigen::Vector3d& pw, const Eigen::Vector3d& twc)
  {
    LOG(FATAL) << "Only specialization is allowed.";
  }

  inline static void queryAtRotation(const Eigen::Matrix3d& Rwc,
                                     const FactorType& pos_factor,
                                     OutMatType* out)
  {
#ifdef ACTMAP_DEBUG
    timer.start();
#endif
    Eigen::Vector3d camf_w = Rwc * Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::VectorXd camz_ftrs;
    vis_approx_->calCamZFeatureVector(camf_w, &camz_ftrs);

#ifdef ACTMAP_DEBUG
    tquery_camz_ftrs += timer.stop();
    timer.start();
#endif
    for (int ri = 0; ri < kNumOut; ri++)
    {
      out->row(ri) = camz_ftrs.transpose() *
                     pos_factor.block(ri * dim_ftrs_, 0, dim_ftrs_, kNumOut);
    }
#ifdef ACTMAP_DEBUG
    tquery_write_to_mat += timer.stop();
#endif
  }

  void queryAtRotation(const Eigen::Matrix3d& Rwc, OutMatType* out) const
  {
    queryAtRotation(Rwc, positional_factor_, out);
  }

  static void printTiming()
  {
    std::cout << "Build:\n"
              << "- unscaled: " << tbuild_unscaled << "\n"
              << "- lm_ftrs: " << tbuild_lm_ftrs << "\n"
              << "- write to mat: " << tbuild_write_to_mat << "\n"
              << "Query:\n"
              << "- camz_ftrs: " << tquery_camz_ftrs << "\n"
              << "- write to mat: " << tquery_write_to_mat << "\n";
  }

  bool isTheSame(const PositionalFactorVoxel& v, const double tol = 1e-8) const
  {
    bool is_the_same = true;
    const double max_diff =
        (positional_factor_ - v.getFactorConstRef()).cwiseAbs().maxCoeff();
    is_the_same &= (max_diff < tol);
    //    if (!is_the_same)
    //    {
    //      std::cout << "\this:\n" << positional_factor_ << std::endl;
    //      std::cout << "others:\n" << v.getFactorConstRef() << std::endl;
    //      std::cout << "max diff.: " << max_diff;
    //    }
    return is_the_same;
  }

  template <typename intT>
  void serializeToIntegers(std::vector<intT>* data) const
  {
    CHECK(data);
    data->clear();
    data->resize(serializationSize());
    size_t data_cnt = 0;
    (*data)[data_cnt++] = static_cast<int>(dim_ftrs_);
    (*data)[data_cnt++] = static_cast<int>(NumY);

    for (int ri = 0; ri < positional_factor_.rows(); ri++)
    {
      for (int ci = 0; ci < positional_factor_.cols(); ci++)
      {
        serializeElem<intT>(positional_factor_(ri, ci), &((*data)[data_cnt++]));
      }
    }
    CHECK_EQ(data_cnt, serializationSize());
  }

  template <typename intT>
  void deserializeFromIntegers(const std::vector<intT>& data, const size_t s)
  {
    size_t data_cnt = s;

    const int rd_dim_ftrs = static_cast<int>(data[data_cnt++]);
    const int rd_numy = static_cast<int>(data[data_cnt++]);
    CHECK_EQ(rd_dim_ftrs, dim_ftrs_) << "Read voxel of different number of "
                                        "features.";
    CHECK_EQ(rd_numy, NumY) << "Read voxel of NumY";

    for (int ri = 0; ri < positional_factor_.rows(); ri++)
    {
      for (int ci = 0; ci < positional_factor_.cols(); ci++)
      {
        deserializeElem<intT>(data[data_cnt++], &(positional_factor_(ri, ci)));
      }
    }
  }

private:
  // factor update implementation
  template <typename F>
  void updateFactorSingleInfo(const Eigen::Vector3d& pw,
                              const Eigen::Vector3d& twc)
  {
    static_assert(NumY == 6, "need 6x6 output forr info update");
#ifdef ACTMAP_DEBUG
    timer.start();
#endif
    rpg::Rotation rot;
    rot.setIdentity();
    rpg::Matrix36 J0 =
        vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw, rpg::Pose(rot, -twc));
    rpg::Matrix66 H0 = J0.transpose() * J0;
#ifdef ACTMAP_DEBUG
    tbuild_unscaled += timer.stop();
    timer.start();
#endif
    Eigen::VectorXd lm_ftrs;
    vis_approx_->calLandmarkFeatureVector(pw - twc, &lm_ftrs);
#ifdef ACTMAP_DEBUG
    tbuild_lm_ftrs += timer.stop();
    timer.start();
#endif
    F func;
    for (int ri = 0; ri < kNumOut; ri++)
    {
      int cur_blk_s = ri * dim_ftrs_;
      for (int ftri = 0; ftri < dim_ftrs_; ftri++)
      {
        Eigen::Matrix<double, 1, kNumOut> cur_r_scaled =
            H0.row(ri) * lm_ftrs(ftri);
        Eigen::Block<FactorType, 1, kNumOut> target_r =
            positional_factor_.template block<1, kNumOut>(cur_blk_s + ftri, 0);
        func.operator()(target_r, cur_r_scaled);
      }
    }
#ifdef ACTMAP_DEBUG
    tbuild_write_to_mat += timer.stop();
#endif
  }

  template <typename F>
  void updateFactorSingleTrace(const Eigen::Vector3d& pw,
                               const Eigen::Vector3d& twc)
  {
    static_assert(NumY == 1, "need 1x1 output forr info update");
#ifdef ACTMAP_DEBUG
    timer.start();
#endif
    rpg::Rotation rot;
    rot.setIdentity();
    rpg::Matrix36 J0 =
        vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw, rpg::Pose(rot, -twc));
    OutMatType H0;
    H0(0, 0) = J0.squaredNorm();
#ifdef ACTMAP_DEBUG
    tbuild_unscaled += timer.stop();

    timer.start();
#endif
    Eigen::VectorXd lm_ftrs;
    vis_approx_->calLandmarkFeatureVector(pw - twc, &lm_ftrs);

#ifdef ACTMAP_DEBUG
    tbuild_lm_ftrs += timer.stop();

    timer.start();
#endif
    F func;
    for (int ri = 0; ri < kNumOut; ri++)
    {
      int cur_blk_s = ri * dim_ftrs_;
      for (int ftri = 0; ftri < dim_ftrs_; ftri++)
      {
        Eigen::Matrix<double, 1, kNumOut> cur_r_scaled =
            H0.row(ri) * lm_ftrs(ftri);
        Eigen::Block<FactorType, 1, kNumOut> target_r =
            positional_factor_.template block<1, kNumOut>(cur_blk_s + ftri, 0);
        func.operator()(target_r, cur_r_scaled);
      }
    }
#ifdef ACTMAP_DEBUG
    tbuild_write_to_mat += timer.stop();
#endif
  }

  FactorType positional_factor_;
  static VisApproxPtr<T> vis_approx_;
  static int dim_ftrs_;

  // profile
  static double tbuild_unscaled;
  static double tbuild_lm_ftrs;
  static double tbuild_write_to_mat;
  static double tquery_camz_ftrs;
  static double tquery_write_to_mat;
  static rpg::Timer timer;
};

template <typename T, int NumY>
VisApproxPtr<T> PositionalFactorVoxel<T, NumY>::vis_approx_ = nullptr;
template <typename T, int NumY>
int PositionalFactorVoxel<T, NumY>::dim_ftrs_ = -1;
template <typename T, int NumY>
double PositionalFactorVoxel<T, NumY>::tbuild_unscaled = 0;
template <typename T, int NumY>
double PositionalFactorVoxel<T, NumY>::tbuild_lm_ftrs = 0;
template <typename T, int NumY>
double PositionalFactorVoxel<T, NumY>::tbuild_write_to_mat = 0;
template <typename T, int NumY>
double PositionalFactorVoxel<T, NumY>::tquery_camz_ftrs = 0;
template <typename T, int NumY>
double PositionalFactorVoxel<T, NumY>::tquery_write_to_mat = 0;
template <typename T, int NumY>
rpg::Timer PositionalFactorVoxel<T, NumY>::timer = rpg::Timer();

using InfoFactorType = Eigen::Matrix<double, Eigen::Dynamic, 6>;
using TraceFactorType = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using InfoOutMatType = Eigen::Matrix<double, 6, 6>;
using TraceOutMatType = Eigen::Matrix<double, 1, 1>;

using GPInfoVoxel = PositionalFactorVoxel<GPVisibilityApproximator, 6>;
using GPInfoVoxelVec = std::vector<GPInfoVoxel>;
using GPTraceVoxel = PositionalFactorVoxel<GPVisibilityApproximator, 1>;
using GPTraceVoxelVec = std::vector<GPTraceVoxel>;
using GPInfoVoxelConstPtr = std::shared_ptr<const GPInfoVoxel>;
using GPTraceVoxelConstPtr = std::shared_ptr<const GPTraceVoxel>;

using QuadPolyInfoVoxel = PositionalFactorVoxel<QuadPolyVisApproximator, 6>;
using QuadPolyInfoVoxelVec = std::vector<QuadPolyInfoVoxel>;
using QuadPolyTraceVoxel = PositionalFactorVoxel<QuadPolyVisApproximator, 1>;
using QuadPolyTraceVoxelVec = std::vector<QuadPolyTraceVoxel>;
using QuadPolyInfoVoxelConstPtr = std::shared_ptr<const QuadPolyInfoVoxel>;
using QuadPolyTraceVoxelConstPtr = std::shared_ptr<const QuadPolyTraceVoxel>;

template <>
template <typename F>
void GPInfoVoxel::updateFactorSingle(const Eigen::Vector3d& pw,
                                     const Eigen::Vector3d& twc)
{
  updateFactorSingleInfo<F>(pw, twc);
}

template <>
template <typename F>
void GPTraceVoxel::updateFactorSingle(const Eigen::Vector3d& pw,
                                      const Eigen::Vector3d& twc)
{
  updateFactorSingleTrace<F>(pw, twc);
}

template <>
template <typename F>
void QuadPolyInfoVoxel::updateFactorSingle(const Eigen::Vector3d& pw,
                                           const Eigen::Vector3d& twc)
{
  updateFactorSingleInfo<F>(pw, twc);
}

template <>
template <typename F>
void QuadPolyTraceVoxel::updateFactorSingle(const Eigen::Vector3d& pw,
                                            const Eigen::Vector3d& twc)
{
  updateFactorSingleTrace<F>(pw, twc);
}

// convenient functions
inline void setGPVisiblityFromFolder(const std::string& folder)
{
  GPInfoVoxel::setVisApproxFromFolderGP(folder);
  GPTraceVoxel::setVisApproxFromFolderGP(folder);
}

inline void setQuadPolyVisiblity(const QuadVisScoreOptions& quad_vis_opt)
{
  QuadPolyInfoVoxel::setVisApproxQuadVisOpt(quad_vis_opt);
  QuadPolyTraceVoxel::setVisApproxQuadVisOpt(quad_vis_opt);
}

}  // namespace act_map

namespace act_map
{
namespace traits
{
// clang-format off
template <>
struct is_vis_vox<GPInfoVoxel> : std::true_type {};
template <>
struct is_vis_vox<GPTraceVoxel> : std::true_type {};
template <>
struct is_vis_vox<QuadPolyInfoVoxel> : std::true_type {};
template <>
struct is_vis_vox<QuadPolyTraceVoxel> : std::true_type {};
template <>
struct is_info_vox<GPInfoVoxel> : std::true_type {};
template <>
struct is_info_vox<QuadPolyInfoVoxel> : std::true_type {};
template <>
struct is_trace_vox<GPTraceVoxel> : std::true_type {};
template <>
struct is_trace_vox<QuadPolyTraceVoxel> : std::true_type {};
// specific visibility approximation
// GP
template <typename T>
struct is_gp_vis_vox : std::false_type {};
template <>
struct is_gp_vis_vox<GPInfoVoxel> : std::true_type {};
template <>
struct is_gp_vis_vox<GPTraceVoxel> : std::true_type {};
// quad. poly.
template <typename T>
struct is_quadpoly_vis_vox : std::false_type {};
template <>
struct is_quadpoly_vis_vox<QuadPolyInfoVoxel> : std::true_type {};
template <>
struct is_quadpoly_vis_vox<QuadPolyTraceVoxel> : std::true_type {};
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
inline bool isSameVoxel(const act_map::GPTraceVoxel& A,
                        const act_map::GPTraceVoxel& B)
{
  return A.isTheSame(B);
}

template <>
inline bool isSameVoxel(const act_map::GPInfoVoxel& A,
                        const act_map::GPInfoVoxel& B)
{
  return A.isTheSame(B);
}

template <>
inline bool isSameVoxel(const act_map::QuadPolyInfoVoxel& A,
                        const act_map::QuadPolyInfoVoxel& B)
{
  return A.isTheSame(B);
}

template <>
inline bool isSameVoxel(const act_map::QuadPolyTraceVoxel& A,
                        const act_map::QuadPolyTraceVoxel& B)
{
  return A.isTheSame(B);
}

}  // namespace utils

namespace voxel_types
{
extern const std::string kGPTrace;
extern const std::string kGPInfo;
// Quadratic Polynomial
extern const std::string kQPTrace;
extern const std::string kQPInfo;
}  // namespace voxel_types

template <>
inline std::string getVoxelType<act_map::GPTraceVoxel>()
{
  return voxel_types::kGPTrace;
}
template <>
inline std::string getVoxelType<act_map::GPInfoVoxel>()
{
  return voxel_types::kGPInfo;
}
template <>
inline std::string getVoxelType<act_map::QuadPolyTraceVoxel>()
{
  return voxel_types::kQPTrace;
}
template <>
inline std::string getVoxelType<act_map::QuadPolyInfoVoxel>()
{
  return voxel_types::kQPInfo;
}

// for serialization
template <>
inline void mergeVoxelAIntoVoxelB(const act_map::GPTraceVoxel& voxel_A,
                                  act_map::GPTraceVoxel* voxel_B)
{
  LOG(FATAL) << "Merge is not supported for GPTraceVoxel";
}

template <>
inline void mergeVoxelAIntoVoxelB(const act_map::GPInfoVoxel& voxel_A,
                                  act_map::GPInfoVoxel* voxel_B)
{
  LOG(FATAL) << "Merge is not supported for GPInfoVoxel";
}

template <>
inline void mergeVoxelAIntoVoxelB(const act_map::QuadPolyInfoVoxel& voxel_A,
                                  act_map::QuadPolyInfoVoxel* voxel_B)
{
  LOG(FATAL) << "Merge is not supported for QuadPolyInfoVoxel";
}

template <>
inline void mergeVoxelAIntoVoxelB(const act_map::QuadPolyTraceVoxel& voxel_A,
                                  act_map::QuadPolyTraceVoxel* voxel_B)
{
  LOG(FATAL) << "Merge is not supported for QuadPolyTraceVoxel";
}

}  // namespace voxblox
}  // namespace act_map

namespace act_map
{
template <typename PFType>
inline double getInfoMetricAtRotationFromPositionalFactor(
    const Eigen::Matrix3d& Rwc, const InfoFactorType& pos_factor,
    const InfoMetricType& v, Eigen::Vector3d* drot_global = nullptr)
{
  InfoOutMatType out;
  PFType::queryAtRotation(Rwc, pos_factor, &out);
  static double dtheta = (1.0 / 180.0) * M_PI;
  if (drot_global)
  {
    for (int i = 0; i < 3; i++)
    {
      Eigen::Matrix3d Rwcm;
      Eigen::Matrix3d Rwcp;
      utils::getRotationPerturbTwoSides(Rwc, dtheta, i, &Rwcm, &Rwcp);

      InfoOutMatType outm, outp;
      PFType::queryAtRotation(Rwcm, pos_factor, &outm);
      PFType::queryAtRotation(Rwcp, pos_factor, &outp);
      double valm = getInfoMetric(outm, v);
      double valp = getInfoMetric(outp, v);
      (*drot_global)(i) = (valp - valm) / (2 * dtheta);
    }
  }
  return getInfoMetric(out, v);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
template <typename PFType>
inline double getInfoMetricAtRotationFromPositionalFactor(
    const Eigen::Matrix3d& Rwc, const TraceFactorType& pos_factor,
    const InfoMetricType& v, Eigen::Vector3d* drot_global = nullptr)
{
  if (v != InfoMetricType::kTrace)
  {
    LOG_FIRST_N(WARNING, 100) << "Query " << kInfoMetricNames.at(v)
                              << " from trace factor. Falling back to trace.";
  }
  TraceOutMatType out;
  PFType::queryAtRotation(Rwc, pos_factor, &out);
  static double dtheta = (0.5 / 180.0) * M_PI;
  if (drot_global)
  {
    for (int i = 0; i < 3; i++)
    {
      Eigen::Matrix3d Rwcm;
      Eigen::Matrix3d Rwcp;
      utils::getRotationPerturbTwoSides(Rwc, dtheta, i, &Rwcm, &Rwcp);

      TraceOutMatType outm, outp;
      PFType::queryAtRotation(Rwcm, pos_factor, &outm);
      PFType::queryAtRotation(Rwcp, pos_factor, &outp);
      double valm = outm(0, 0);
      double valp = outp(0, 0);
      (*drot_global)(i) = (valp - valm) / (2 * dtheta);
    }
  }
  return out(0, 0);
}
#pragma GCC diagnostic pop
}  // namespace act_map
