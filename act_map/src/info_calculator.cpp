#include "act_map/info_calculator.h"

namespace act_map
{
void InfoCalculator::calculateInfoAt(const rpg::Pose& Twc,
                                     const Vec3dVec& points_w,
                                     const Vec3dVec& view_dirs_from_pt,
                                     Info* fim) const
{
  CHECK_NOTNULL(fim);
  VisIdx vis_idx;

  if (vis_checker_)
  {
    vis_checker_->getVisibleIdx(Twc, points_w, view_dirs_from_pt, &vis_idx);
  }
  else
  {
    for (size_t i = 0; i < points_w.size(); i++)
    {
      vis_idx.insert(i);
    }
  }

  fim->setZero();
  for (const size_t idx : vis_idx)
  {
    addPoseInfoBearingGlobal(Twc, points_w[idx], nullptr, fim);
  }
}

bool InfoCalculator::calculateInfoMetricAt(const rpg::Pose& Twc,
                                           const Vec3dVec& points,
                                           const Vec3dVec& view_dirs_from_pt,
                                           const InfoMetricType& info_t,
                                           double* val, Eigen::Vector3d* dpos,
                                           Eigen::Vector3d* drot_g) const
{
  rpg::Matrix66 fim;
  calculateInfoAt(Twc, points, view_dirs_from_pt, &fim);
  Eigen::FullPivLU<rpg::Matrix66> lu_decomp(fim);
  Eigen::Index rank = lu_decomp.rank();
  if (rank < 6)
  {
    LOG_EVERY_N(WARNING, 100)
        << "FIM is rank deficient, return 0.0 for everything.";
    if (dpos)
    {
      dpos->setZero();
    }
    if (drot_g)
    {
      drot_g->setZero();
    }
    (*val) = 0.0;
    return false;
  }

  (*val) = getInfoMetric(fim, info_t);
  if (dpos)
  {
    for (int i = 0; i < 3; i++)
    {
      rpg::Pose Twc_m = Twc;
      rpg::Pose Twc_p = Twc;

      double val_m = 0.0;
      double val_p = 0.0;
      bool res = true;
      (Twc_m.getPosition()(i)) -= pos_step_;
      (Twc_p.getPosition()(i)) += pos_step_;
      res &= calculateInfoMetricAt(Twc_m, points, view_dirs_from_pt, info_t,
                                   &val_m, nullptr, nullptr);
      res &= calculateInfoMetricAt(Twc_p, points, view_dirs_from_pt, info_t,
                                   &val_p, nullptr, nullptr);
      if (res == false)
      {
        return false;
      }
      (*dpos)(i) = (val_p - val_m) / pos_stepx2_;
    }
  }
  if (drot_g)
  {
    for (int i = 0; i < 3; i++)
    {
      Eigen::Matrix3d Rwcm;
      Eigen::Matrix3d Rwcp;
      utils::getRotationPerturbTwoSides(Twc.getRotationMatrix(), rot_step_rad_,
                                        i, &Rwcm, &Rwcp);
      rpg::Pose Twc_m = Twc;
      rpg::Pose Twc_p = Twc;
      Twc_m.getRotation() = rpg::Rotation(Rwcm);
      Twc_p.getRotation() = rpg::Rotation(Rwcp);

      double val_m = 0.0;
      double val_p = 0.0;
      bool res = true;
      res &= calculateInfoMetricAt(Twc_m, points, view_dirs_from_pt, info_t,
                                   &val_m, nullptr, nullptr);
      res &= calculateInfoMetricAt(Twc_p, points, view_dirs_from_pt, info_t,
                                   &val_p, nullptr, nullptr);
      if (res == false)
      {
        return false;
      }
      (*drot_g)(i) = (val_p - val_m) / rot_step_radx2_;
    }
  }
  return true;
}

}  // namespace act_map
