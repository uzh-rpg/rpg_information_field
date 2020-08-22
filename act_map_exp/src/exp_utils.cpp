#include "act_map_exp/exp_utils.h"

namespace act_map_exp
{
void TimeValues::save(const std::string& fn) const
{
  saveStampedScalars(times, values, fn);
}

void jacVector3Normalization(const Eigen::Vector3d& v, Eigen::Matrix3d* jac)
{
  const double n = v.norm();
  const double n3 = std::pow(n, 3);

  for (int ri = 0; ri < 3; ri++)
  {
    for (int ci = 0; ci < 3; ci++)
    {
      if (ri == ci)
      {
        (*jac)(ri, ci) = 1 / n - std::pow(v(ri), 2) / n3;
      }
      else
      {
        (*jac)(ri, ci) = -(v(ri) * v(ci)) / n3;
      }
    }
  }
}

void quadAccYawToRwb(const Eigen::Vector3d& acc_w, const double yaw_rad,
                     rpg::Rotation::RotationMatrix* rot_mat,
                     rpg::Matrix93* drotmat_dacc, rpg::Matrix91* drotmat_dyaw)
{
  bool cal_jac = drotmat_dacc == nullptr ? false : true;
  if (cal_jac)
  {
    CHECK(drotmat_dyaw);
  }

  // z_b: thrust direction
  const Eigen::Vector3d z_b = acc_w + Eigen::Vector3d(0, 0, 9.81);
  const Eigen::Vector3d z_b_u = z_b / z_b.norm();

  // c: the intermediate frame after yaw rotation
  const Eigen::Vector3d x_c(std::cos(yaw_rad), std::sin(yaw_rad), 0.0);
  const Eigen::Vector3d y_b = z_b_u.cross(x_c);
  const Eigen::Vector3d y_b_u = y_b / y_b.norm();

  // x_b
  const Eigen::Vector3d x_b = y_b_u.cross(z_b_u);
  const Eigen::Vector3d x_b_u = x_b / x_b.norm();

  // output
  rot_mat->col(0) = x_b_u;
  rot_mat->col(1) = y_b_u;
  rot_mat->col(2) = z_b_u;

  if (cal_jac)
  {
    // jacobian w.r.t. the acceleration
    Eigen::Matrix3d dzbu_dacc;
    jacVector3Normalization(z_b, &dzbu_dacc);  // dzbu_dzb * Identity(3)
    Eigen::Vector3d dzbu_dyaw = Eigen::Vector3d::Zero();

    const Eigen::Vector3d dxc_dyaw(-std::sin(yaw_rad), std::cos(yaw_rad), 0.0);
    Eigen::Matrix3d dybu_dyb;
    jacVector3Normalization(y_b, &dybu_dyb);
    // jacobian w.r.t. acc.
    // x_c does not depend on acc, so write it as [x_c]_x^T (dot) z_b_u
    Eigen::Matrix3d dyb_dzbu;
    skew(x_c, &dyb_dzbu);
    dyb_dzbu.transposeInPlace();
    Eigen::Matrix3d dybu_dacc = dybu_dyb * dyb_dzbu * dzbu_dacc;
    // jacobian w.r.t yaw
    // z_b_u does not depend on yaw, write it as [z_b_u]_x (dot) x_c
    Eigen::Matrix3d dyb_dxc;
    skew(z_b_u, &dyb_dxc);
    Eigen::Vector3d dybu_dyaw = dybu_dyb * dyb_dxc * dxc_dyaw;

    Eigen::Matrix3d dxbu_dxb;
    jacVector3Normalization(x_b, &dxbu_dxb);
    Eigen::Matrix3d dxb_dacc;
    jacVecCrossProd(y_b_u, z_b_u, dybu_dacc, dzbu_dacc, &dxb_dacc);
    Eigen::Vector3d dxb_dyaw;
    jacVecCrossProd(y_b_u, z_b_u, dybu_dyaw, dzbu_dyaw, &dxb_dyaw);
    Eigen::Matrix3d dxbu_dacc = dxbu_dxb * dxb_dacc;
    Eigen::Vector3d dxbu_dyaw = dxbu_dxb * dxb_dyaw;

    drotmat_dacc->block<1, 3>(0, 0) = dxbu_dacc.row(0);
    drotmat_dacc->block<1, 3>(3, 0) = dxbu_dacc.row(1);
    drotmat_dacc->block<1, 3>(6, 0) = dxbu_dacc.row(2);

    drotmat_dacc->block<1, 3>(1, 0) = dybu_dacc.row(0);
    drotmat_dacc->block<1, 3>(4, 0) = dybu_dacc.row(1);
    drotmat_dacc->block<1, 3>(7, 0) = dybu_dacc.row(2);

    drotmat_dacc->block<1, 3>(2, 0) = dzbu_dacc.row(0);
    drotmat_dacc->block<1, 3>(5, 0) = dzbu_dacc.row(1);
    drotmat_dacc->block<1, 3>(8, 0) = dzbu_dacc.row(2);

    (*drotmat_dyaw)(0) = dxbu_dyaw(0);
    (*drotmat_dyaw)(3) = dxbu_dyaw(1);
    (*drotmat_dyaw)(6) = dxbu_dyaw(2);

    (*drotmat_dyaw)(1) = dybu_dyaw(0);
    (*drotmat_dyaw)(4) = dybu_dyaw(1);
    (*drotmat_dyaw)(7) = dybu_dyaw(2);

    (*drotmat_dyaw)(2) = dzbu_dyaw(0);
    (*drotmat_dyaw)(5) = dzbu_dyaw(1);
    (*drotmat_dyaw)(8) = dzbu_dyaw(2);
  }
}

void saveStampedPoses(const std::vector<double>& times,
                      const rpg::PoseVec& poses, const std::string& abs_fn)
{
  rpg::Aligned<std::vector, rpg::Matrix44> T_mat;
  T_mat.reserve(poses.size());
  for (size_t i = 0; i < poses.size(); i++)
  {
    T_mat.emplace_back(poses[i].getTransformationMatrix());
  }
  saveStampedEigenMatrices(times, T_mat, abs_fn,
                           std::string("transformation matrices - "));
}

void saveStampedPoses(const std::vector<double>& times,
                      const unrealcv_bridge::UEPoseVec& poses,
                      const std::string& abs_fn)
{
  rpg::Aligned<std::vector, rpg::Matrix16> uepose_mat;
  uepose_mat.reserve(poses.size());
  for (size_t i = 0; i < poses.size(); i++)
  {
    rpg::Matrix16 cur_pose;
    const unrealcv_bridge::UEPose& uep = poses[i];
    cur_pose(0, 0) = uep.x;
    cur_pose(0, 1) = uep.y;
    cur_pose(0, 2) = uep.z;
    cur_pose(0, 3) = uep.pitch;
    cur_pose(0, 4) = uep.yaw;
    cur_pose(0, 5) = uep.roll;
    uepose_mat.emplace_back(cur_pose);
  }
  saveStampedEigenMatrices(times, uepose_mat, abs_fn,
                           std::string("ue poses - "));
}

double getPathLength(const rpg::PoseVec& poses)
{
  double length = 0.0;
  for (size_t i = 0; i < poses.size() - 1; i++)
  {
    length += (poses[i].getPosition() - poses[i+1].getPosition()).norm();
  }
  return length;
}

}  // namespace act_map_exp
