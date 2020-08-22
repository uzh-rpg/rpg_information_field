#include "act_map_exp/potential_cost.h"

namespace act_map_exp
{
PotentialCost::~PotentialCost()
{
}

double PotentialCost::operator()(const double dist, double* grad) const
{
  double d = dist - robot_radius_;
  double result;

  if (d < 0)
  {
    result = -d + 0.5 * dist_margin_;
    if (grad)
    {
      (*grad) = -1.0;
    }
  }
  else if (d <= dist_margin_)
  {
    double epsilon_distance = d - dist_margin_;
    result = 0.5 * 1.0 / (dist_margin_)*epsilon_distance * epsilon_distance;
    if (grad)
    {
      (*grad) = (d - dist_margin_) / dist_margin_;
    }
  }
  else
  {
    result = 0.0;
    if (grad)
    {
      (*grad) = 0.0;
    }
  }
  return result;
}

}  // namespace act_map_exp
