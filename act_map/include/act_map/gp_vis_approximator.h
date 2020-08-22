#pragma once

#include "act_map/visibility_approximator.h"
#include "act_map/sigmoid_vis_score.h"

#include <iostream>

#include <glog/logging.h>
#include <rpg_common/timer.h>

namespace act_map
{
class GPVisibilityApproximator
    : public VisibilityApproximator<GPVisibilityApproximator>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GPVisibilityApproximator()
  {
  }

  inline bool isInitialized() const
  {
    return initialized_;
  }

  void load(const std::string& folder);

  inline void calLandmarkFeatureVector(const Eigen::Vector3d& fpt_w,
                                       Eigen::VectorXd* lm_ftrs) const
  {
#ifdef ACTMAP_DEBUG
    timer.start();
#endif
    Eigen::VectorXd sampled_soft_vis;
    sig_vis_ptr_->softVisiblityBatch(sampled_fs_, fpt_w, &sampled_soft_vis);
#ifdef ACTMAP_DEBUG
    t_check_vis += timer.stop();
    timer.start();
#endif
    (*lm_ftrs) = inv_K_ * sampled_soft_vis;
#ifdef ACTMAP_DEBUG
    t_mat_mul += timer.stop();
#endif
  }

  inline void calCamZFeatureVector(const Eigen::Vector3d& camz_w,
                                   Eigen::VectorXd* camz_ftrs) const
  {
    Eigen::Vector3d camf_w = camz_w.normalized();

    camz_ftrs->resize(sampled_fs_.rows());
    for (int i = 0; i < sampled_fs_.rows(); i++)
    {
      (*camz_ftrs)(i) = rbf(camf_w, sampled_fs_.row(i));
    }
  }

  int dimFeatureSpace() const
  {
    return static_cast<int>(sampled_fs_.rows());
  }

  std::string str() const;

  inline const SigmoidVisScore& getSigmoidVisConstRef() const
  {
    CHECK(sig_vis_ptr_);
    return *sig_vis_ptr_;
  }

  static void resetTimer()
  {
    t_check_vis = 0;
    t_mat_mul = 0;
  }
  static void printTiming()
  {
    std::cout << "Time in GP visiblity check:\n"
              << "- sigmoid vis: " << t_check_vis << "\n"
              << "- matrix mul.: " << t_mat_mul << "\n";
  }

  // file names for FoV, kernel and sampled bearing vectors respectively
  static const std::string kFovFn;
  static const std::string kKernelFn;
  static const std::string kFsFn;

private:
  inline double rbf(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) const
  {
    double dn_sqr = (v1 - v2).squaredNorm();
    return rbf_var_ *
           std::exp(-0.5 * dn_sqr / (rbf_lengthscale_ * rbf_lengthscale_));
  }
  void initializeK();

  bool initialized_ = false;

  double sigmoid_k_ = 0.0;
  double hfov_rad_ = 0.0;

  double rbf_var_ = 0.0;
  double rbf_lengthscale_ = 0.0;
  double white_var_ = 0.0;

  bool fast_sigmoid_ = false;

  Eigen::Matrix<double, Eigen::Dynamic, 3> sampled_fs_;

  SigmoidVisPtr sig_vis_ptr_ = nullptr;

  Eigen::MatrixXd K_;
  Eigen::MatrixXd inv_K_;

  // profile
  static double t_check_vis;
  static double t_mat_mul;
  static rpg::Timer timer;
};

using GPVisApprox = GPVisibilityApproximator;
// using the interface in the base class
// not the same as the shared pointer of the specific type
using GPVisApproxPtr =
    std::shared_ptr<VisibilityApproximator<GPVisibilityApproximator>>;
}
