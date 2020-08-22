#pragma once

#include <array>
#include <cmath>

#include <rpg_common/pose.h>

#include <opencv2/opencv.hpp>

#include "unrealcv_bridge/esim_utils.hpp"

namespace unrealcv_bridge
{
struct UEPose
{
  double pitch;
  double yaw;
  double roll;
  double x;
  double y;
  double z;

  void fromEigen(const Eigen::MatrixXd& row_vec, const bool pos_first = true);

  void fromUnrealCVStr(const std::string& str);

  inline void mulPos(const double scale)
  {
    x *= scale;
    y *= scale;
    z *= scale;
  }

  inline void toTwcUE(rpg::Pose* Twc_ue)
  {
    Twc_ue->setIdentity();
    esim::unrealEulerToQuaternion(pitch, yaw, roll, &(Twc_ue->getRotation()));
    Twc_ue->getPosition() = rpg::Position(x, y, z);
  }

  inline void toTwc(rpg::Pose* Twc)
  {
    rpg::Pose Twc_ue;
    toTwcUE(&Twc_ue);

    Eigen::Matrix4d T_wue_w;
    T_wue_w.setIdentity();
    T_wue_w(1, 1) = -1;

    Eigen::Matrix4d T_c_cue;
    T_c_cue.setZero();
    T_c_cue(0, 1) = 1;
    T_c_cue(1, 2) = -1;
    T_c_cue(2, 0) = 1;
    T_c_cue(3, 3) = 1;

    Eigen::Matrix4d Twc_mat = T_wue_w.inverse() *
                              Twc_ue.getTransformationMatrix() *
                              T_c_cue.inverse();
    (*Twc) = rpg::Pose(Twc_mat);
  }

  inline void print()
  {
    std::cout << "pitch: " << pitch << "; "
              << "yaw: " << yaw << "; "
              << "roll: " << roll << "; "
              << "x: " << x << "; "
              << "y: " << y << "; "
              << "z: " << z << "\n";
  }

  inline bool isSame(const UEPose& rv, const double eps = 1e-3) const
  {
    bool same = true;
    same &= (std::fabs(pitch - rv.pitch) < eps);
    same &= (std::fabs(yaw - rv.yaw) < eps);
    same &= (std::fabs(roll - rv.roll) < eps);
    same &= (std::fabs(x - rv.x) < eps);
    same &= (std::fabs(y - rv.y) < eps);
    same &= (std::fabs(z - rv.z) < eps);

    return same;
  }
};

using UEPoseVec = std::vector<UEPose>;

inline void TwcToUEPose(const rpg_common::Pose& Twc, UEPose* ue_p)
{
  const rpg_common::Pose::TransformationMatrix Twc_mat =
      Twc.getTransformationMatrix();
  rpg_common::Pose::TransformationMatrix Tuew_w_mat;
  Tuew_w_mat << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  rpg_common::Pose::TransformationMatrix Tc_uec_mat;
  Tc_uec_mat << 0, 1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;

  const rpg_common::Pose::TransformationMatrix ue_Twc_mat =
      Tuew_w_mat * Twc_mat * Tc_uec_mat;
  const rpg_common::Pose ue_Twc(ue_Twc_mat);

  double yaw, pitch, roll;
  esim::quaternionToEulerUnrealEngine(ue_Twc.getRotation(), yaw, pitch, roll);

  ue_p->x = ue_Twc.getPosition()[0];
  ue_p->y = ue_Twc.getPosition()[1];
  ue_p->z = ue_Twc.getPosition()[2];
  ue_p->pitch = pitch;
  ue_p->yaw = yaw;
  ue_p->roll = roll;
}

void rayDepthToZDepth(const cv::Mat& ray_depth, const float f,
                      cv::Mat* z_depth);

}  // namespace unrealcv_bridge
