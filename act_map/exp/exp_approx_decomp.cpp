//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/vis_score.h"

#include <ctime>
#include <cstdlib>

#include <rpg_common/main.h>
#include <rpg_common/pose.h>
#include <rpg_common/eigen_type.h>
#include <vi_utils/vi_jacobians.h>

using namespace act_map;

RPG_COMMON_MAIN
{
  std::cout << "This is to test whether we can decompose the second order "
               "visibility approximation into a rotation-dependent part and "
               "a rotation-independent one.\n";

  std::srand(std::time(0));

  double hfov_rad = M_PI_4;
  VisScore vscore(hfov_rad);
  vscore.initSecondOrderApprox(0.9, 0.9);

  rpg::Pose Twc;
  Twc.setRandom();
  //  rpg::Pose Twc; Twc.setIdentity();
  std::cout << "Twc is\n" << Twc.getTransformationMatrix() << std::endl;

  Eigen::Vector3d pw;
  pw.setRandom();
  //  Eigen::Vector3d pw (1.2, 2.5, 3.9);
  std::cout << "pw is\n" << pw << std::endl;

  Eigen::Vector3d pc = Twc.inverse() * pw;
  std::cout << "pc is\n" << pc << std::endl;

  double weight = vscore.secondOrderVisibility(pc);
  std::cout << "visibility weight is " << weight << std::endl;

  rpg::Matrix36 Jc =
      vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw, Twc.inverse());
  rpg::Matrix66 Hc = Jc.transpose() * Jc;

  std::cout << "Computing kernels...\n";
  rpg::Rotation rot0;
  rot0.setIdentity();
  rpg::Pose Tw0(rot0, Twc.getPosition());
  Eigen::Vector3d p0 = Tw0.inverse() * pw;
  Eigen::Matrix3d p0p0T = p0 * p0.transpose();
  double d0 = p0.norm();
  rpg::Matrix36 J0 =
      vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw, Tw0.inverse());
  rpg::Matrix66 H0 = J0.transpose() * J0;
  rpg::Matrix66 H0_over_d = H0 / d0;
  rpg::Matrix66 H0_over_d2 = H0_over_d / d0;

  // K1
  Eigen::Matrix<double, 18, 18> info_K1;
  Eigen::Matrix<double, 3, 3> trace_K1;
  trace_K1.setZero();
  for (int ri = 0; ri < 6; ri++)
  {
    for (int ci = 0; ci < 6; ci++)
    {
      info_K1.block(ri * 3, ci * 3, 3, 3) = p0p0T * H0_over_d2(ri, ci);
    }
  }
  for (int i = 0; i < 6; i++)
  {
    trace_K1 += p0p0T * H0_over_d2(i, i);
  }
  std::cout << "Info K1 is:\n" << info_K1 << std::endl;
  std::cout << "Trace K1 is:\n" << trace_K1 << std::endl;

  // K2
  Eigen::Matrix<double, 18, 6> info_K2;
  Eigen::Matrix<double, 3, 1> trace_K2;
  trace_K2.setZero();
  for (int ri = 0; ri < 6; ri++)
  {
    for (int ci = 0; ci < 6; ci++)
    {
      info_K2.block(ri * 3, ci, 3, 1) = p0 * H0_over_d(ri, ci);
    }
  }
  for (int i = 0; i < 6; i++)
  {
    trace_K2 += p0 * H0_over_d(i, i);
  }
  std::cout << "Info K2 is:\n" << info_K2 << std::endl;
  std::cout << "Trace K2 is:\n" << trace_K2 << std::endl;

  //
  Eigen::Matrix3d rot = Twc.getRotationMatrix().transpose();
  rpg::Matrix13 e3TR = VisScore::kEz.transpose() * rot;
  rpg::Matrix31 e3TR_trans = e3TR.transpose();

  rpg::Matrix66 H_decomp;

  double trace_a_sum = 0;
  double trace_b_sum = 0;
  double trace_c_sum = 0;
  for (int ri = 0; ri < 6; ri++)
  {
    for (int ci = 0; ci < 6; ci++)
    {
      double c1 =
          vscore.k1() * e3TR * info_K1.block(ri * 3, ci * 3, 3, 3) * e3TR_trans;
      double c2 =
          (vscore.k2() * e3TR * info_K2.block(ri * 3, ci, 3, 1)).value();
      double c3 = vscore.k3() * Hc(ri, ci);
      H_decomp(ri, ci) = c1 + c2 + c3;
      if (ri == ci)
      {
        trace_a_sum += c1;
        trace_b_sum += c2;
        trace_c_sum += c3;
      }
    }
  }
  double trace_a = vscore.k1() * e3TR * trace_K1 * e3TR_trans;
  double trace_b = vscore.k2() * (e3TR * trace_K2).value();
  double trace_c = vscore.k3() * Hc.trace();
  double trace_decomp = vscore.k1() * e3TR * trace_K1 * e3TR_trans +
                        vscore.k2() * (e3TR * trace_K2).value() +
                        vscore.k3() * Hc.trace();

  std::cout << ">>> Compare the information computed differently:\n"
            << "- weighted by visibility score:\n" << Hc *weight << std::endl
            << "- computed by decomposition:\n" << H_decomp << std::endl
            << "- the difference is: \n" << H_decomp - Hc *weight << std::endl;

  std::cout << ">>> Compare the trace computed differently:\n"
            << "- from full info: " << H_decomp.trace() << std::endl
            << "\t" << trace_a_sum << ", " << trace_b_sum << ", " << trace_c_sum
            << std::endl
            << "- from trace kernels: " << trace_decomp << std::endl
            << "\t" << trace_a << ", " << trace_b << ", " << trace_c
            << std::endl;
}
