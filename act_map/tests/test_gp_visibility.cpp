#include "act_map/gp_vis_approximator.h"

#include <cmath>

#include <rpg_common/pose.h>
#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>

using namespace act_map;

class GPVisTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    abs_cfg_folder_ = dir + "/test_data/fov50_fs50_lm1000_k10";
  }
  std::string abs_cfg_folder_;
};

TEST_F(GPVisTest, testInitialization)
{
  GPVisibilityApproximator gp_vis;
  std::cout << "Before:\n" << gp_vis;
  gp_vis.load(abs_cfg_folder_);
  EXPECT_TRUE(gp_vis.isInitialized());
  std::cout << "After:\n" << gp_vis;

  EXPECT_EQ(50, gp_vis.dimFeatureSpace());
}

TEST_F(GPVisTest, testSigmoidVisiblity)
{
  GPVisibilityApproximator gp_vis;
  gp_vis.load(abs_cfg_folder_);
  const SigmoidVisScore& sig_vis = gp_vis.getSigmoidVisConstRef();
  EXPECT_NEAR(sig_vis.softVisiblityFromCosine(1.0), 1.0, 0.05);
  EXPECT_NEAR(sig_vis.softVisiblityFromCosine(std::cos(sig_vis.hFoVRad())), 0.5,
              1e-5);
  EXPECT_NEAR(sig_vis.softVisiblityFromCosine(0.0), 0.0, 0.05);

  rpg::Pose Twc;
  Twc.setRandom();
  Eigen::Vector3d twc = Twc.getPosition();
  Eigen::Vector3d pt_w, cam_z_w;
  pt_w.setRandom();
  cam_z_w.setRandom();
  Eigen::Vector3d fpt_w = pt_w - twc;

  double cos = (fpt_w.normalized()).dot(cam_z_w.normalized());
  EXPECT_NEAR(sig_vis.softVisiblityFromCosine(cos),
              sig_vis.softVisibility(cam_z_w, pt_w, twc), 1e-10);
  EXPECT_NEAR(sig_vis.softVisiblityFromCosine(cos),
              sig_vis.softVisibility(cam_z_w, fpt_w), 1e-10);

  Eigen::Matrix<double, 100, 3> cam_z_w_mul;
  cam_z_w_mul.setRandom();
  Eigen::Matrix<double, 100, 3> cam_f_w_mul;
  for (int i = 0; i < cam_z_w_mul.rows(); i++)
  {
    cam_f_w_mul.row(i) = cam_z_w_mul.row(i).normalized();
  }
  Eigen::VectorXd vis_batch, vis_batch2;
  vis_batch.resize(cam_f_w_mul.rows());
  vis_batch2.resize(cam_f_w_mul.rows());
  sig_vis.softVisiblityBatch(cam_f_w_mul, pt_w, twc, &vis_batch);
  sig_vis.softVisiblityBatch(cam_f_w_mul, fpt_w, &vis_batch2);
  for (int i = 0; i < cam_z_w_mul.rows(); i++)
  {
    EXPECT_NEAR(
        vis_batch(i),
        sig_vis.softVisibility(cam_z_w_mul.row(i), pt_w, twc),
        1e-10);
    EXPECT_NEAR(
        vis_batch2(i),
        sig_vis.softVisibility(cam_z_w_mul.row(i), pt_w, twc),
        1e-10);
  }
}

TEST_F(GPVisTest, testGPApprox)
{
  GPVisibilityApproximator gp_vis;
  gp_vis.load(abs_cfg_folder_);
  const SigmoidVisScore& sig_vis = gp_vis.getSigmoidVisConstRef();

  Eigen::Vector3d lm_w;
  lm_w.setRandom();
  Eigen::VectorXd lm_ftrs;
  gp_vis.calLandmarkFeatureVector(lm_w, &lm_ftrs);

  const size_t n_tests = 10;

  for (size_t i = 0; i < n_tests; i++)
  {
    Eigen::Vector3d camz_w;
    camz_w.setRandom();

    // use the approximator
    Eigen::VectorXd camz_ftrs;
    gp_vis.calCamZFeatureVector(camz_w, &camz_ftrs);
    double gp_soft_vis = camz_ftrs.dot(lm_ftrs);

    double sig_soft_vis = sig_vis.softVisibility(camz_w, lm_w);

    EXPECT_NEAR(gp_soft_vis, sig_soft_vis, 0.12);
  }
}

RPG_COMMON_TEST_MAIN
{
}
