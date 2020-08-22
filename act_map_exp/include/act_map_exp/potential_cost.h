#pragma once

#include <memory>
// adapted from
// https://github.com/ethz-asl/mav_voxblox_planning/tree/master/loco_planner

namespace act_map_exp
{
class PotentialCost
{
public:
  PotentialCost() = delete;
  virtual ~PotentialCost();

  PotentialCost(const double robot_radius, const double dist_margin)
    : robot_radius_(robot_radius), dist_margin_(dist_margin)
  {
  }

  double operator()(const double dist, double* grad = nullptr) const;

private:
  double robot_radius_;
  double dist_margin_;
};
using PotentialCostPtr = std::shared_ptr<PotentialCost>;
}  // namespace act_map_exp
