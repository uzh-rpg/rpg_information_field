#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Core>

namespace act_map
{
struct VisScoreOptions
{
  double half_fov_rad = 0.7854;
  double boundary_to_mid_ratio = 0.5;
  double boundary_value = 0.5;
};

class VisScore
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VisScore() = delete;
  VisScore(const VisScoreOptions& options)
    : VisScore(options.half_fov_rad)
  {
    initSecondOrderApprox(options.boundary_to_mid_ratio,
                          options.boundary_value);
  }
  VisScore(double hfov_rad)
    : hfov_rad_(hfov_rad), cos_hfov_(std::cos(hfov_rad_))
  {
  }

  inline double exactVisibility(const double cos_view_angle) const
  {
    return cos_view_angle > cos_hfov_ ? 1.0 : 0.0;
  }

  inline double exactVisibility(const Eigen::Vector3d& f) const
  {
    Eigen::Vector3d uf = f.normalized();
    return exactVisibility(uf.dot(kEz));
  }

  inline bool secondOrderApproxInitialized() const
  {
    return second_order_initialized_;
  }

  void initSecondOrderApprox(const double boundary_to_mid_ratio,
                             const double boundary_value);

  void createSamples(const double step_deg, std::vector<double>* samples);

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

  inline double secondOrderVisibility(const double cos) const
  {
    return k1_ * cos * cos + k2_ * cos + k3_;
  }

  inline double secondOrderVisibility(const Eigen::Vector3d& f) const
  {
    Eigen::Vector3d uf = f.normalized();
    double cos_uf = uf.dot(kEz);
    return secondOrderVisibility(cos_uf);
  }
  static const Eigen::Vector3d kEz;

  static std::shared_ptr<VisScore> load(const std::string& cfg);
  static std::vector<std::shared_ptr<VisScore>>
  loadMultiple(const std::string& dir);

  static const std::string kFnPre;
private:
  double hfov_rad_;
  double cos_hfov_;

  bool second_order_initialized_ = false;
  double k1_ = 0;
  double k2_ = 0;
  double k3_ = 0;
};
using VisScorePtr = std::shared_ptr<VisScore>;
}
