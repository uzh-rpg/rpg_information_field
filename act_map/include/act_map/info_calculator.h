#pragma once

#include "act_map/visibility_checker.h"

namespace act_map
{
class InfoCalculator
{
public:
  InfoCalculator(const double pos_step, const double rot_step_deg,
                 const VisibilityCheckerPtr& vis_check_ptr = nullptr)
    : pos_step_(pos_step)
    , rot_step_deg_(rot_step_deg)
    , rot_step_rad_(rot_step_deg_ * M_PI / 180.0)
    , pos_stepx2_(pos_step_ * 2)
    , rot_step_radx2_(rot_step_rad_ * 2)
    , vis_checker_(vis_check_ptr)
  {
  }

  InfoCalculator()
  {
  }

  InfoCalculator(const InfoCalculator& rhs)
    : InfoCalculator(rhs.pos_step_, rhs.rot_step_deg_, rhs.vis_checker_)
  {
  }

  void calculateInfoAt(const rpg::Pose& Twc, const Vec3dVec& points_w,
                       const Vec3dVec& view_dirs_from_pt, Info* fim) const;

  bool calculateInfoMetricAt(const rpg::Pose& Twc, const Vec3dVec& points,
                             const Vec3dVec& view_dirs_from_pt,
                             const InfoMetricType& info_t, double* val,
                             Eigen::Vector3d* dpos = nullptr,
                             Eigen::Vector3d* drot_g = nullptr) const;

  double pos_step_ = 0.05;
  double rot_step_deg_ = 0.5;
  double rot_step_rad_ = 0.5 * M_PI / 180.0;
  double pos_stepx2_ = 0.1;
  double rot_step_radx2_ = 1.0 * M_PI / 180.0;
  VisibilityCheckerPtr vis_checker_ = nullptr;
};
using InfoCalculatorPtr = std::shared_ptr<InfoCalculator>;

}  // namespace act_map
