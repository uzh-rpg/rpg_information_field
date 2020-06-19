//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/info_kernels.h"

namespace act_map
{
void getInfoAtRotation(const Eigen::Matrix3d& Rwc,
                       const double k1,
                       const double k2,
                       const double k3,
                       const InfoK1& K1,
                       const InfoK2& K2,
                       const InfoK3& K3,
                       Eigen::Matrix<double, 6, 6>* H)

{
  Eigen::Matrix3d rot = Rwc.transpose();
  rpg::Matrix13 e3TR = Eigen::Vector3d(0, 0, 1).transpose() * rot;
  rpg::Matrix31 e3TR_trans = e3TR.transpose();
  for (int ri = 0; ri < 6; ri++)
  {
    for (int ci = 0; ci < 6; ci++)
    {
      double c1 = k1 * e3TR * K1.cast<double>().block(ri * 3, ci * 3, 3, 3) *
                  e3TR_trans;
      double c2 =
          (k2 * e3TR * K2.cast<double>().block(ri * 3, ci, 3, 1)).value();
      double c3 = k3 * static_cast<double>(K3(ri, ci));
      (*H)(ri, ci) = c1 + c2 + c3;
    }
  }
}
}
