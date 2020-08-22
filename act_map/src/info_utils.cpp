#include "act_map/info_utils.h"

#include <rpg_common/eigen_type.h>
#include <vi_utils/vi_jacobians.h>

#include "act_map/internal/operators.h"

namespace act_map
{
std::map<InfoMetricType, std::string> kInfoMetricNames = {
  { InfoMetricType::kNone, "none" },
  { InfoMetricType::kMinEigRank, "mineigrank" },
  { InfoMetricType::kMinEig, "mineig" },
  { InfoMetricType::kTrace, "trace" },
  { InfoMetricType::kDet, "det" },
  { InfoMetricType::kCovTraceReciprocal, "covtraceinv" }
};

std::map<std::string, InfoMetricType> kNameToInfoMetric = {
  { "none", InfoMetricType::kNone },
  { "mineigrank", InfoMetricType::kMinEigRank },
  { "mineig", InfoMetricType::kMinEig },
  { "trace", InfoMetricType::kTrace },
  { "det", InfoMetricType::kDet },
  { "covtraceinv", InfoMetricType::kCovTraceReciprocal }
};

std::map<PoseMarginalizeType, std::string> kPoseMarginalizationNames{
  { PoseMarginalizeType::kFull, "full" },
  { PoseMarginalizeType::kPosition, "position" },
  { PoseMarginalizeType::kRotation, "rotation" },
};

bool kInfoMetricUseLogDet = true;

void addPoseInfoBearingGlobal(const rpg_common::Pose& Twc,
                              const Eigen::Vector3d& pw,
                              const QuadVisScorePtr& vscore,
                              Eigen::Matrix<double, 6, 6>* H)
{
  manipulatePoseInfoBearingGlobal<internal::blockPlusEqual>(Twc, pw, vscore, H);
}

void assignPoseInfoBearingGlobal(const rpg_common::Pose& Twc,
                                 const Eigen::Vector3d& pw,
                                 const QuadVisScorePtr& vscore,
                                 Eigen::Matrix<double, 6, 6>* H)
{
  manipulatePoseInfoBearingGlobal<internal::blockEqual>(Twc, pw, vscore, H);
}

double getInfoMetric(const Eigen::MatrixXd& info,
                     const InfoMetricType& metric_type)
{
  if (InfoMetricType::kTrace == metric_type)
  {
    return info.trace();
  }
  else if (InfoMetricType::kMinEigRank == metric_type)
  {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_info(info, Eigen::ComputeThinU |
                                                         Eigen::ComputeThinV);
    Eigen::VectorXd s_values = svd_info.singularValues();
    //    CHECK_EQ(svd_info.rank(), info.rows());
    return s_values(svd_info.rank() - 1);
  }
  else if (InfoMetricType::kMinEig == metric_type)
  {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_info(info, Eigen::ComputeThinU |
                                                         Eigen::ComputeThinV);
    Eigen::VectorXd s_values = svd_info.singularValues();
    return s_values(info.rows() - 1);
  }
  else if (InfoMetricType::kDet == metric_type)
  {
    if (kInfoMetricUseLogDet)
    {
      // mapping det. to above 1, so that log will be always greater than 0
      constexpr double kDetThresh = 1e-6;
      // offset = std::exp(kDetThresh) + 1.0 - kDetThresh to be continuous
      // and still have some gradient...
      constexpr double offset = 2.0 + 5e-13;
      double det = info.determinant();

      if (std::isnan(det))
      {
        det = kDetThresh;
      }
      if (det < kDetThresh)
      {
        // LOG(WARNING) << "Det. lower than thresh.";
        det = std::exp(det) + 1.0;
      }
      else
      {
        det += offset;
      }
      return std::log(det);
    }
    else
    {
      return info.determinant();
    }
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

void marginalizePoseInfo(const rpg::Matrix66& full_info,
                         const PoseMarginalizeType& type,
                         Eigen::MatrixXd* mag_info)
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
}  // namespace act_map
