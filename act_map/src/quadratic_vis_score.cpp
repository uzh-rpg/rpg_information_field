#include "act_map/quadratic_vis_score.h"

#include <rpg_common/fs.h>
#include <rpg_common/load.h>
#include <vi_utils/common_utils.h>

namespace act_map
{
const Eigen::Vector3d QuadraticVisScore::kEz = Eigen::Vector3d(0, 0, 1);
const std::string QuadraticVisScore::kFnPre = "vis_score";

void QuadraticVisScore::initSecondOrderApprox(const double boundary_to_mid_ratio,
                                     const double boundary_value)
{
  double cos_hfov_sq = cos_hfov_ * cos_hfov_;
  double denom = cos_hfov_ - 2 * boundary_to_mid_ratio + 1;
  k1_ = boundary_value * denom /
        (2 * boundary_to_mid_ratio - 2 * boundary_to_mid_ratio * cos_hfov_sq);
  k2_ = k1_ * (-cos_hfov_sq + 1) / denom;
  k3_ = k1_ * (2 * boundary_to_mid_ratio - cos_hfov_ - cos_hfov_sq) / denom;

  second_order_initialized_ = true;
}

QuadVisScorePtr QuadraticVisScore::load(const std::string& fn)
{
  Eigen::MatrixXd params;
  rpg::load(fn, &params);
  CHECK_EQ(1, params.rows());
  CHECK_EQ(3, params.cols());

  QuadVisScorePtr vis_ptr = std::make_shared<QuadraticVisScore>(params(0, 0));
  vis_ptr->initSecondOrderApprox(params(0, 1), params(0, 2));

  return vis_ptr;
}

std::vector<QuadVisScorePtr> QuadraticVisScore::loadMultiple(const std::string& dir)
{
  std::vector<QuadVisScorePtr> vis_scores;
  int n_vis = 0;
  while (1)
  {
    const std::string fn =
        dir + "/" + QuadraticVisScore::kFnPre + std::to_string(n_vis) + ".txt";
    if (!rpg::fs::fileExists(fn))
    {
      break;
    }
    vis_scores.push_back(QuadraticVisScore::load(fn));
    n_vis++;
  }
  return vis_scores;
}

void QuadraticVisScore::createSamples(const double step_deg,
                             std::vector<double>* samples)
{
  CHECK_NOTNULL(samples);
  samples->clear();

  const double step_rad = step_deg / 180.0 * M_PI;
  std::vector<double> sampled_rads;
  vi_utils::linspace(-M_PI, M_PI, step_rad, &sampled_rads);

  samples->resize(sampled_rads.size());
  for (size_t i = 0; i < sampled_rads.size(); i++)
  {
    (*samples)[i] = secondOrderVisibility(std::cos(sampled_rads[i]));
  }
}
}
