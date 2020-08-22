#pragma once

#include "act_map/visibility_approximator.h"

#include "act_map/quadratic_vis_score.h"

#include <glog/logging.h>

namespace act_map
{
class QuadPolyVisApproximator
    : public VisibilityApproximator<QuadPolyVisApproximator>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int kDimFtr = 10;

  QuadPolyVisApproximator()
  {
    is_initialized_ = false;
  }

  QuadPolyVisApproximator(const QuadVisScoreOptions& opt)
  {
    QuadraticVisScore qvis_score(opt);
    k1_ = qvis_score.k1();
    k2_ = qvis_score.k2();
    k3_ = qvis_score.k3();
    is_initialized_ = true;
  }

  QuadPolyVisApproximator(const double k1, const double k2, const double k3)
    : k1_(k1), k2_(k2), k3_(k3)
  {
    is_initialized_ = true;
  }

  inline void setQuadPolyCoefs(const double k1, const double k2,
                               const double k3)
  {
    k1_ = k1;
    k2_ = k2;
    k3_ = k3;
    is_initialized_ = true;
  }

  // required functions
  bool isInitialized() const
  {
    return is_initialized_;
  }

  void load(const std::string& folder)
  {
    LOG(WARNING) << "Not implemented for quadratic polynomial, doing nothing";
  }

  inline void calLandmarkFeatureVector(const Eigen::Vector3d& fpt_w,
                                       Eigen::VectorXd* lm_ftrs) const
  {
    lm_ftrs->resize(kDimFtr);
    const Eigen::Vector3d n_fpt_w = fpt_w.normalized();
    const double& p1 = n_fpt_w.x();
    const double& p2 = n_fpt_w.y();
    const double& p3 = n_fpt_w.z();
    (*lm_ftrs)(0) = p1 * p1;
    (*lm_ftrs)(1) = p2 * p2;
    (*lm_ftrs)(2) = p3 * p3;
    (*lm_ftrs)(3) = p1 * p2;
    (*lm_ftrs)(4) = p1 * p3;
    (*lm_ftrs)(5) = p2 * p3;
    (*lm_ftrs)(6) = p1;
    (*lm_ftrs)(7) = p2;
    (*lm_ftrs)(8) = p3;
    (*lm_ftrs)(9) = 1;
  }

  inline void calCamZFeatureVector(const Eigen::Vector3d& camz_w,
                                   Eigen::VectorXd* camz_ftrs) const
  {
    camz_ftrs->resize(kDimFtr);
    Eigen::Vector3d camf_w = camz_w.normalized();
    const double& z1 = camf_w.x();
    const double& z2 = camf_w.y();
    const double& z3 = camf_w.z();

    (*camz_ftrs)(0) = k1() * z1 * z1;
    (*camz_ftrs)(1) = k1() * z2 * z2;
    (*camz_ftrs)(2) = k1() * z3 * z3;
    (*camz_ftrs)(3) = 2 * k1() * z1 * z2;
    (*camz_ftrs)(4) = 2 * k1() * z1 * z3;
    (*camz_ftrs)(5) = 2 * k1() * z2 * z3;
    (*camz_ftrs)(6) = k2() * z1;
    (*camz_ftrs)(7) = k2() * z2;
    (*camz_ftrs)(8) = k2() * z3;
    (*camz_ftrs)(9) = k3();
  }

  inline int dimFeatureSpace() const
  {
    return kDimFtr;
  }

  inline double k1() const
  {
    return k1_;
  }
  inline double k2() const
  {
    return k2_;
  }

  inline double k3() const
  {
    return k3_;
  }

private:
  double k1_ = -1;
  double k2_ = -1;
  double k3_ = -1;

  bool is_initialized_ = false;
};

using QuadPolyVisApproxPtr =
    std::shared_ptr<VisibilityApproximator<QuadPolyVisApproximator>>;
}
