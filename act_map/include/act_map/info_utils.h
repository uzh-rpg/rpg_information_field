//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <memory>
#include <map>

#include <rpg_common/pose.h>
#include <vi_utils/vi_jacobians.h>

#include "act_map/vis_score.h"

namespace act_map
{
class VisScore;
using VisScorePtr = std::shared_ptr<VisScore>;

enum class InfoMetricType
{
  kMinEigRank,
  kMinEig,
  kTrace,
  kDet,
  kCovTraceReciprocal
};

enum class PoseMarginalizeType
{
  kFull,
  kPosition,
  kRotation
};

extern std::map<InfoMetricType, std::string> kInfoMetricNames;
extern std::map<PoseMarginalizeType, std::string> kPoseMarginalizationNames;

template <typename F>
void manipulatePoseInfoBearingGlobal(const rpg::Pose& Twc,
                                     const Eigen::Vector3d& pw,
                                     const VisScorePtr& vscore,
                                     rpg::Matrix66* H)
{
  Eigen::Vector3d pc = Twc.inverse() * pw;
  rpg::Matrix36 J =
      vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw, Twc.inverse());
  F func;
  rpg::Matrix66 cur_info;
  if (vscore)
  {
    cur_info = J.transpose() * (J * vscore->secondOrderVisibility(pc));
  }
  else
  {
    cur_info = J.transpose() * J;
  }
  func.operator()(*H, cur_info);
}

void addPoseInfoBearingGlobal(const rpg::Pose& Twc,
                              const Eigen::Vector3d& pw,
                              const VisScorePtr& vscore,
                              rpg::Matrix66* H);

void assignPoseInfoBearingGlobal(const rpg::Pose& Twc,
                                 const Eigen::Vector3d& pw,
                                 const VisScorePtr& vscore,
                                 rpg::Matrix66* H);

double getInfoMetric(const Eigen::MatrixXd& info,
                     const InfoMetricType& metric_type);

void marginalizePoseInfo(const rpg::Matrix66& full_info,
                         const PoseMarginalizeType& type,
                         Eigen::MatrixXd* mag_info);
}
