//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include <vi_utils/vi_jacobians.h>

#include <ctime>
#include <cstdlib>

#include <rpg_common/main.h>
#include <rpg_common/pose.h>
#include <rpg_common/eigen_type.h>

RPG_COMMON_MAIN
{
  std::cout << "This is to test the rotation invariance of the information "
               "matrix, depending on the rotation parameterization used.\n";
  std::cout << "We assume that the camera/imu is the same pose.\n";

  std::srand(std::time(0));

  rpg::Pose Twi;
  Twi.setRandom();
  std::cout << "Twi is\n" << Twi.getTransformationMatrix() << std::endl;

  rpg::Pose Tic;
  Tic.setIdentity();
  std::cout << "Tic is\n" << Tic.getTransformationMatrix() << std::endl;
  rpg::Pose Twc = Twi * Tic;

  Eigen::Vector3d pw;
  pw.setRandom();
  std::cout << "pw is\n" << pw << std::endl;

  rpg::Rotation dR;
  dR.setRandom();
  std::cout << "dR is\n" << dR.getRotationMatrix() << std::endl;

  rpg::Pose Twi_d(Twi.getRotation() * dR, Twi.getPosition());
  rpg::Pose Twc_d = Twi_d * Tic;

  //  rpg::Pose Tic_d(Tic.getRotation() * dR, Tic.getPosition());
  //  rpg::Pose Twc_d = Twi * Tic_d;

  rpg::Matrix36 J_global =
      vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw, Twc.inverse());
  rpg::Matrix66 info_global = J_global.transpose() * J_global;

  rpg::Matrix36 J_global_d =
      vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw, Twc_d.inverse());
  rpg::Matrix66 info_global_d = J_global_d.transpose() * J_global_d;

  rpg::Matrix36 J_local =
      vi_utils::jacobians::dBearing_dIMUPose(pw, Twi.inverse(), Tic.inverse());
  rpg::Matrix66 info_local = J_local.transpose() * J_local;

  rpg::Matrix36 J_local_d = vi_utils::jacobians::dBearing_dIMUPose(
      pw, Twi_d.inverse(), Tic.inverse());
  rpg::Matrix66 info_local_d = J_local_d.transpose() * J_local_d;

  std::cout << ">>> Global disturbance parameterization:\n"
            << "- Before\n" << info_global << std::endl
            << "- After\n" << info_global_d << std::endl
            << "- Diff\n" << info_global - info_global << std::endl;

  std::cout << ">>> Local disturbance parameterization:\n"
            << "- Before\n" << info_local << std::endl
            << "- After\n" << info_local_d << std::endl
            << "- Diff\n" << info_local - info_local_d << std::endl;
}
