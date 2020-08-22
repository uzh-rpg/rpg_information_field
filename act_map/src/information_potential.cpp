#include "act_map/information_potential.h"

#include <random>

#include <vi_utils/vi_jacobians.h>

#include "act_map/positional_factor_voxel.h"
#include "act_map/conversion.h"

namespace act_map
{

namespace traits
{
constexpr typename ValidPotentialMetrics<QuadTraceVoxel>::type
    ValidPotentialMetrics<QuadTraceVoxel>::values;
constexpr typename ValidPotentialMetrics<GPTraceVoxel>::type
    ValidPotentialMetrics<GPTraceVoxel>::values;
}

InfoPotentialFunc::InfoPotentialFunc(const double thresh,
                                     const double val_at_zero)
  : thresh_(thresh)
  , val_at_zero_(val_at_zero)
  , quad_k_(val_at_zero_ / std::pow(thresh_, 2))
  , linear_k_(2 * quad_k_ * (-thresh_))
  , linear_b_(quad_k_ * std::pow(-thresh, 2))
{
  CHECK_GT(thresh, 0.0);
  CHECK_GT(quad_k_, 0.0);
}

InfoPotentialFunc::InfoPotentialFunc(const InfoPotentialFunc& rhs)
  : InfoPotentialFunc(rhs.thresh_, rhs.val_at_zero_)
{
}

double InfoPotentialFunc::eval(const double query, double* grad) const
{
  double cost = -std::numeric_limits<double>::infinity();
  if (query > thresh_)
  {
    cost = 0.0;
    if (grad)
    {
      (*grad) = 0.0;
    }
  }
  else if (query <= thresh_ && query >= 0.0)
  {
    cost = quad_k_ * std::pow(query - thresh_, 2);
    if (grad)
    {
      (*grad) = 2 * quad_k_ * (query - thresh_);
    }
  }
  else
  {
    cost = linear_k_ * query + linear_b_;
    if (grad)
    {
      (*grad) = linear_k_;
    }
  }
  return cost;
}

}  // namespace act_map
