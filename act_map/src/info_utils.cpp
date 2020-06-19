//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/info_utils.h"

#include <rpg_common/eigen_type.h>
#include <vi_utils/vi_jacobians.h>

#include "act_map/internal/operators.h"

namespace act_map
{
std::map<InfoMetricType, std::string> kInfoMetricNames = {
  { InfoMetricType::kMinEigRank, "mineigrank" },
  { InfoMetricType::kMinEig, "mineig" },
  { InfoMetricType::kTrace, "trace" },
  { InfoMetricType::kDet, "det" },
  { InfoMetricType::kCovTraceReciprocal, "covtraceinv" }
};

std::map<PoseMarginalizeType, std::string> kPoseMarginalizationNames {
  { PoseMarginalizeType::kFull, "full"},
  { PoseMarginalizeType::kPosition,  "position" },
  { PoseMarginalizeType::kRotation,  "rotation" },
};

void addPoseInfoBearingGlobal(const rpg_common::Pose &Twc,
                              const Eigen::Vector3d &pw,
                              const VisScorePtr &vscore,
                              Eigen::Matrix<double, 6, 6> *H)
{
  manipulatePoseInfoBearingGlobal<internal::blockPlusEqual>(Twc, pw, vscore, H);
}

void assignPoseInfoBearingGlobal(const rpg_common::Pose &Twc,
                                 const Eigen::Vector3d &pw,
                                 const VisScorePtr &vscore,
                                 Eigen::Matrix<double, 6, 6> *H)
{
  manipulatePoseInfoBearingGlobal<internal::blockEqual>(Twc, pw, vscore, H);
}

double getInfoMetric(const Eigen::MatrixXd &info,
                     const InfoMetricType &metric_type)
{
  if (InfoMetricType::kTrace == metric_type)
  {
    return info.trace();
  }
  else if (InfoMetricType::kMinEigRank == metric_type)
  {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_info(
        info, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s_values = svd_info.singularValues();
    //    CHECK_EQ(svd_info.rank(), info.rows());
    return s_values(svd_info.rank() - 1);
  }
  else if (InfoMetricType::kMinEig == metric_type)
  {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_info(
        info, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s_values = svd_info.singularValues();
    return s_values(info.rows() - 1);
  }
  else if (InfoMetricType::kDet == metric_type)
  {
    return info.determinant();
  }
  else if (InfoMetricType::kCovTraceReciprocal == metric_type)
  {
    LOG(FATAL) << "Not implemented yet.";
  }
  else
  {
    LOG(FATAL) << "Unknown metric type";
  }
}

void marginalizePoseInfo(const rpg::Matrix66 &full_info,
                         const PoseMarginalizeType &type,
                         Eigen::MatrixXd *mag_info)
{
  CHECK_NOTNULL(mag_info);
  if (type == PoseMarginalizeType::kFull)
  {
    (*mag_info) = full_info;
  }
  else if (type == PoseMarginalizeType::kPosition)
  {
    Eigen::JacobiSVD<rpg::Matrix66> svd_info(
        full_info, Eigen::ComputeThinU | Eigen::ComputeThinV);
    CHECK_EQ(svd_info.rank(), 6);
    (*mag_info) = full_info.block<3, 3>(0, 0) -
                  full_info.block<3, 3>(0, 3) *
                      full_info.block<3, 3>(3, 3).inverse() *
                      full_info.block<3, 3>(3, 0);
  }
  else if (type == PoseMarginalizeType::kRotation)
  {
    Eigen::JacobiSVD<rpg::Matrix66> svd_info(
        full_info, Eigen::ComputeThinU | Eigen::ComputeThinV);
    CHECK_EQ(svd_info.rank(), 6);
    (*mag_info) = full_info.block<3, 3>(3, 3) -
                  full_info.block<3, 3>(3, 0) *
                      full_info.block<3, 3>(0, 0).inverse() *
                      full_info.block<3, 3>(0, 3);
  }
  else
  {
    LOG(FATAL) << "Unknown marginalization type";
  }

  return;
}
}
