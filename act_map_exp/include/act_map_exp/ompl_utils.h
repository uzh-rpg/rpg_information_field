#pragma once

#include <rpg_common/pose.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>

#include <voxblox/core/esdf_map.h>
#include <act_map/act_map.h>
#include <act_map/common.h>
#include <act_map/information_potential.h>

#include "act_map_exp/exp_utils.h"

namespace ob = ompl::base;

namespace act_map_exp
{
constexpr int kPos = 0;
constexpr int kYaw = 1;
constexpr double kYawDiscountFactor = 0.5;
namespace ompl_utils
{
inline void flatStateToPositionYaw(const ob::State& s, Eigen::Vector3d* pos,
                                   double* yaw)
{
  if (pos)
  {
    pos->x() = s.as<ob::CompoundStateSpace::StateType>()
                   ->as<ob::RealVectorStateSpace::StateType>(kPos)
                   ->values[0];
    pos->y() = s.as<ob::CompoundStateSpace::StateType>()
                   ->as<ob::RealVectorStateSpace::StateType>(kPos)
                   ->values[1];
    pos->z() = s.as<ob::CompoundStateSpace::StateType>()
                   ->as<ob::RealVectorStateSpace::StateType>(kPos)
                   ->values[2];
  }
  if (yaw)
  {
    (*yaw) = s.as<ob::CompoundStateSpace::StateType>()
                 ->as<ob::SO2StateSpace::StateType>(kYaw)
                 ->value;
  }
}

inline void positionYawToFlatState(const Eigen::Vector3d& pos, const double yaw,
                                   ob::State* s)
{
  s->as<ob::CompoundStateSpace::StateType>()
      ->as<ob::RealVectorStateSpace::StateType>(kPos)
      ->values[0] = pos.x();
  s->as<ob::CompoundStateSpace::StateType>()
      ->as<ob::RealVectorStateSpace::StateType>(kPos)
      ->values[1] = pos.y();
  s->as<ob::CompoundStateSpace::StateType>()
      ->as<ob::RealVectorStateSpace::StateType>(kPos)
      ->values[2] = pos.z();
  s->as<ob::CompoundStateSpace::StateType>()
      ->as<ob::SO2StateSpace::StateType>(kYaw)
      ->value = yaw;
}

class DistanceCost : public ob::PathLengthOptimizationObjective
{
public:
  DistanceCost(const ob::SpaceInformationPtr& si)
    : ob::PathLengthOptimizationObjective(si)
  {
  }

  ob::Cost stateCost(const ob::State* s) const override
  {
    return ob::Cost(0);
  }

  ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override
  {
    Eigen::Vector3d pos1, pos2;
    flatStateToPositionYaw(*s1, &pos1, nullptr);
    flatStateToPositionYaw(*s2, &pos2, nullptr);
    return ob::Cost((pos1 - pos2).norm());
  }

  ob::Cost motionCostHeuristic(const ob::State* s1,
                               const ob::State* s2) const override
  {
    return this->motionCost(s1, s2);
  }
};

class YawCost : public ob::PathLengthOptimizationObjective
{
public:
  YawCost(const ob::SpaceInformationPtr& si)
    : ob::PathLengthOptimizationObjective(si)
  {
  }

  ob::Cost stateCost(const ob::State* s) const override
  {
    return ob::Cost(0);
  }

  ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override
  {
    double yaw1, yaw2;
    flatStateToPositionYaw(*s1, nullptr, &yaw1);
    flatStateToPositionYaw(*s2, nullptr, &yaw2);
    return ob::Cost(kYawDiscountFactor * std::fabs(yaw1 - yaw2));
  }

  ob::Cost motionCostHeuristic(const ob::State* s1,
                               const ob::State* s2) const override
  {
    return this->motionCost(s1, s2);
  }
};

class ESDFChecker : public ob::StateValidityChecker
{
public:
  ESDFChecker(const ob::SpaceInformationPtr& si,
              const std::shared_ptr<const voxblox::EsdfMap> esdf_map_cptr,
              const double clearance_thresh)
    : ob::StateValidityChecker(si)
    , esdf_map_cptr_(esdf_map_cptr)
    , clearance_thresh_(clearance_thresh)
  {
  }

  bool isValid(const ob::State* state) const override
  {
    Eigen::Vector3d pos;
    flatStateToPositionYaw(*state, &pos, nullptr);

    double dist = 0;
    bool res = esdf_map_cptr_->getDistanceAtPosition(pos, true, &dist);
    if (res == false)
    {
      LOG_EVERY_N(WARNING, 200)
          << "Position " << pos.transpose() << " query failed.";
      return false;
    }

    return (dist > clearance_thresh_);
  }

private:
  std::shared_ptr<const voxblox::EsdfMap> esdf_map_cptr_;
  double clearance_thresh_;
};

template <typename T>
class JointChecker : public ob::StateValidityChecker
{
public:
  JointChecker(const ob::SpaceInformationPtr& si,
               const std::shared_ptr<const voxblox::EsdfMap> esdf_map_cptr,
               const double clearance_thresh, const act_map::ActMap<T>& am_cref,
               const act_map::InfoPotentialPtr<T>& info_pot,
               const rpg::Pose& Tbc, const act_map::InfoMetricType& info_t,
               const bool use_pc)
    : ob::StateValidityChecker(si)
    , esdf_map_cptr_(esdf_map_cptr)
    , clearance_thresh_(clearance_thresh)
    , am_cref_(am_cref)
    , info_pot_ptr_(info_pot)
    , Tbc_(Tbc)
    , info_metric_t_(info_t)
    , cal_from_pc_(use_pc)
  {
  }

  bool isValid(const ob::State* state) const override
  {
    Eigen::Vector3d pos;
    double yaw;
    flatStateToPositionYaw(*state, &pos, &yaw);

    double dist = 0;
    bool res = esdf_map_cptr_->getDistanceAtPosition(pos, true, &dist);
    if (res == false)
    {
      LOG_EVERY_N(WARNING, 200)
          << "Position " << pos.transpose() << " query failed.";
      return false;
    }
    if (dist < clearance_thresh_)
    {
      return false;
    }

    Eigen::Matrix3d rot_mat;
    quadAccYawToRwb(Eigen::Vector3d::Zero(), yaw, &rot_mat, nullptr, nullptr);
    rpg::Pose Twb(rpg::Rotation(rot_mat), pos);
    rpg::Pose Twc = Twb * Tbc_;

    res = false;
    double info_metric = 0.0;
    if (cal_from_pc_)
    {
      res = am_cref_.getInfoMetricFromPC(Twc, info_metric_t_, &info_metric,
                                         nullptr, nullptr);
      if (!res)
      {
        info_metric = 1e-6;
      }
      if (info_metric < info_pot_ptr_->getMetricThresh(info_metric_t_))
      {
        return false;
      }
    }
    else
    {
      res = am_cref_.getInfoMetricAt(Twc, info_metric_t_, &info_metric, nullptr,
                                     nullptr);
      if (!res)
      {
        info_metric = 1e-6;
      }
      if (info_metric < info_pot_ptr_->getMetricThreshApproxVis(info_metric_t_))
      {
        return false;
      }
    }

    return true;
  }

private:
  // ESDF related
  std::shared_ptr<const voxblox::EsdfMap> esdf_map_cptr_;
  double clearance_thresh_;
  // information related
  const act_map::ActMap<T>& am_cref_;
  act_map::InfoPotentialPtr<T> info_pot_ptr_;
  rpg::Pose Tbc_;
  act_map::InfoMetricType info_metric_t_;
  bool cal_from_pc_;
};

class DummyChecker : public ob::StateValidityChecker
{
public:
  DummyChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si)
  {
  }

  inline bool isValid(const ob::State* state) const
  {
    return this->clearance(state) > 0.0;
  }

  inline double clearance(const ob::State* state) const
  {
    return 1.0;
  }
};

template <typename T>
class InfoCost : public ob::StateCostIntegralObjective
{
public:
  InfoCost(const ob::SpaceInformationPtr& si, const act_map::ActMap<T>& am_cref,
           const act_map::InfoPotentialPtr<T>& info_pot,
           const act_map::InfoMetricType& info_metric_t, const rpg::Pose& Tbc,
           const bool use_pc)
    : ob::StateCostIntegralObjective(si, true)
    , am_cref_(am_cref)
    , info_pot_ptr_(info_pot)
    , info_metric_t_(info_metric_t)
    , Tbc_(Tbc)
    , cal_from_pc_(use_pc)
  {
  }

  ob::Cost stateCost(const ob::State* s) const override
  {
    Eigen::Vector3d pos;
    double yaw;
    flatStateToPositionYaw(*s, &pos, &yaw);
    Eigen::Matrix3d rot_mat;
    quadAccYawToRwb(Eigen::Vector3d::Zero(), yaw, &rot_mat, nullptr, nullptr);
    rpg::Pose Twb(rpg::Rotation(rot_mat), pos);
    rpg::Pose Twc = Twb * Tbc_;

    double info_metric = 0.0;
    bool res = false;
    if (cal_from_pc_)
    {
      res = am_cref_.getInfoMetricFromPC(Twc, info_metric_t_, &info_metric,
                                         nullptr, nullptr);
      if (!res)
      {
        info_metric = 1e-6;
      }
      const double pot_cost =
          info_pot_ptr_->eval(info_metric, info_metric_t_, nullptr);
      return ob::Cost(pot_cost);
    }
    else
    {
      bool res = false;
      res = am_cref_.getInfoMetricAt(Twc, info_metric_t_, &info_metric, nullptr,
                                     nullptr);
      if (!res)
      {
        info_metric = 1e-6;
      }
      const double pot_cost =
          info_pot_ptr_->evalApproxVis(info_metric, info_metric_t_, nullptr);
      return ob::Cost(pot_cost);
    }
  }

private:
  const act_map::ActMap<T>& am_cref_;
  act_map::InfoPotentialPtr<T> info_pot_ptr_;
  act_map::InfoMetricType info_metric_t_;
  rpg::Pose Tbc_;
  bool cal_from_pc_;
};

inline void getVerticesFromPlannerData(const ob::PlannerDataPtr& planner_data,
                                       act_map::Vec3dVec* vert_pos)
{
  vert_pos->resize(planner_data->numVertices());
  for (unsigned int vert_i = 0; vert_i < planner_data->numVertices(); vert_i++)
  {
    const ob::PlannerDataVertex& v = planner_data->getVertex(vert_i);
    const ob::State* s = v.getState();
    flatStateToPositionYaw(*s, &((*vert_pos)[vert_i]), nullptr);
  }
}

}  // namespace ompl_utils
}  // namespace act_map_exp
