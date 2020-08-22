#pragma once

#include <memory>
#include <cmath>

#include <Eigen/Core>

namespace act_map
{
class SigmoidVisScore
{
public:
  SigmoidVisScore() = delete;
  SigmoidVisScore(const double hfov_rad, const double sigmoid_k,
                  const bool fast_sigmoid)
    : hfov_rad_(hfov_rad), sigmoid_k_(sigmoid_k), fast_sigmoid_(fast_sigmoid)
  {
    hfov_cos_ = std::cos(hfov_rad_);
  }

  inline double fastSoftVisiblityFromCosine(const double cos) const
  {
    double x = sigmoid_k_ * (cos - hfov_cos_);
    return 0.5 * (x / (1 + std::abs(x))) + 0.5;
  }

  inline double softVisiblityFromCosine(const double cos) const
  {
    return 1.0 / (1.0 + std::exp(-sigmoid_k_ * (cos - hfov_cos_)));
  }

  inline double softVisibility(const Eigen::Vector3d& cam_z_w,
                               const Eigen::Vector3d& fpt_w) const
  {
    const Eigen::Vector3d n_cam_z_w = cam_z_w.normalized();
    const Eigen::Vector3d n_fpt_w = fpt_w.normalized();
    double cos = n_cam_z_w.dot(n_fpt_w);

    if (fast_sigmoid_)
    {
      return fastSoftVisiblityFromCosine(cos);
    }
    else
    {
      return softVisiblityFromCosine(cos);
    }
  }

  inline double softVisibility(const Eigen::Vector3d& cam_z_w,
                               const Eigen::Vector3d& pt_w,
                               const Eigen::Vector3d& twc) const
  {
    return softVisibility(cam_z_w, pt_w - twc);
  }

  template <int Rows>
  inline void softVisiblityBatch(const Eigen::Matrix<double, Rows, 3>& cam_f_ws,
                                 const Eigen::Vector3d& fpt_w,
                                 Eigen::VectorXd* vis_score) const
  {
    const Eigen::Vector3d n_fpt_w = fpt_w.normalized();
    const Eigen::VectorXd cos_vals = cam_f_ws * n_fpt_w;
    vis_score->resize(cos_vals.size());
    if (fast_sigmoid_)
    {
      for (int i = 0; i < cos_vals.size(); i++)
      {
        (*vis_score)[i] = fastSoftVisiblityFromCosine(cos_vals[i]);
      }
    }
    else
    {
      for (int i = 0; i < cos_vals.size(); i++)
      {
        (*vis_score)[i] = softVisiblityFromCosine(cos_vals[i]);
      }
    }
  }

  template <int Rows>
  inline void softVisiblityBatch(const Eigen::Matrix<double, Rows, 3>& cam_f_ws,
                                 const Eigen::Vector3d& pt_w,
                                 const Eigen::Vector3d& twc,
                                 Eigen::VectorXd* vis_score) const
  {
    softVisiblityBatch(cam_f_ws, pt_w - twc, vis_score);
  }

  inline double hFoVRad() const
  {
    return hfov_rad_;
  }

private:
  double hfov_rad_;
  double hfov_cos_;
  double sigmoid_k_;
  bool fast_sigmoid_ = false;
};

using SigmoidVisPtr = std::shared_ptr<SigmoidVisScore>;
}
