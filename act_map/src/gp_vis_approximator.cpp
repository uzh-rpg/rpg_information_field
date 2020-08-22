#include "act_map/gp_vis_approximator.h"

#include <cmath>

#include <rpg_common/fs.h>
#include <rpg_common/load.h>

namespace act_map
{
const std::string GPVisibilityApproximator::kFovFn = "fov_params.txt";
const std::string GPVisibilityApproximator::kKernelFn = "kernel_params.txt";
const std::string GPVisibilityApproximator::kFsFn = "sampled_fs.txt";

double GPVisibilityApproximator::t_check_vis = 0;
double GPVisibilityApproximator::t_mat_mul = 0;
rpg::Timer GPVisibilityApproximator::timer = rpg::Timer();

void GPVisibilityApproximator::load(const std::string& folder)
{
  const std::string fov_abs_fn = folder + "/" + kFovFn;
  const std::string kernel_abs_fn = folder + "/" + kKernelFn;
  const std::string fs_abs_fn = folder + "/" + kFsFn;

  CHECK(rpg::fs::fileExists(fov_abs_fn));
  CHECK(rpg::fs::fileExists(kernel_abs_fn));
  CHECK(rpg::fs::fileExists(fs_abs_fn));

  Eigen::MatrixXd fov_params;
  rpg::load(fov_abs_fn, &fov_params);
  CHECK(fov_params.cols() == 1 && fov_params.rows() == 2);
  sigmoid_k_ = fov_params(0, 0);
  hfov_rad_ = M_PI * (fov_params(1, 0) / 180.0);

  Eigen::MatrixXd kernel_params;
  rpg::load(kernel_abs_fn, &kernel_params);
  CHECK(kernel_params.cols() == 1 && kernel_params.rows() == 3);
  rbf_var_ = kernel_params(0, 0);
  rbf_lengthscale_ = kernel_params(1, 0);
  white_var_ = kernel_params(2, 0);

  rpg::load(fs_abs_fn, &sampled_fs_);
  for (int i = 0; i < sampled_fs_.rows(); i++)
  {
    sampled_fs_.block<1, 3>(i, 0).normalize();
  }

  if (folder.find("fast") != std::string::npos)
  {
    fast_sigmoid_ = true;
  }

  sig_vis_ptr_.reset(new SigmoidVisScore(hfov_rad_, sigmoid_k_,
                                         fast_sigmoid_));

  initializeK();

  initialized_ = true;
}

void GPVisibilityApproximator::initializeK()
{
  const long int n_fs = sampled_fs_.rows();
  K_.resize(n_fs, n_fs);
  for (int ri = 0; ri < n_fs; ri++)
  {
    for (int ci = ri; ci < n_fs; ci++)
    {
      K_(ri, ci) = rbf(sampled_fs_.row(ri), sampled_fs_.row(ci));
      if (ci == ri)
      {
        K_(ri, ci) += white_var_;
      }
      else
      {
        K_(ci, ri) = K_(ri, ci);
      }
    }
  }
  inv_K_ = K_.inverse();
}

std::string GPVisibilityApproximator::str() const
{
  std::string str = std::string("GPVisbilityApproximator:\n") +
                    "- sigmoid_k: " + std::to_string(sigmoid_k_) + "\n" +
                    "- hfov_rad: " + std::to_string(hfov_rad_) + "\n" +
                    "- fast_sigmoid: " + std::to_string(fast_sigmoid_) + "\n" +
                    "- rbf_var: " + std::to_string(rbf_var_) + "\n" +
                    "- rbf_lengthscale: " + std::to_string(rbf_lengthscale_) +
                    "\n" + "- white_var: " + std::to_string(white_var_) + "\n" +
                    "- number of sample bearings: " +
                    std::to_string(sampled_fs_.rows()) + "\n";
  return str;
}
}
