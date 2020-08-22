#include "act_map/quadpoly_vis_approximator.h"

#include <cmath>

#include <rpg_common/pose.h>
#include <rpg_common/test_main.h>

using namespace act_map;

class QuadPolyVisTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    quad_vis_options_.half_fov_rad = 0.7854;
    quad_vis_options_.boundary_value = 0.5;
    quad_vis_options_.boundary_to_mid_ratio = 0.5;
    quad_vis_ptr_ = std::make_shared<QuadraticVisScore>(quad_vis_options_);
  }
  QuadVisScorePtr quad_vis_ptr_ = nullptr;
  QuadVisScoreOptions quad_vis_options_;
};

TEST_F(QuadPolyVisTest, initialization)
{
  QuadPolyVisApproximator approx0;
  EXPECT_FALSE(approx0.isInitialized());

  QuadPolyVisApproximator approx1(quad_vis_options_);
  EXPECT_TRUE(approx1.isInitialized());

  EXPECT_DOUBLE_EQ(approx1.k1(), quad_vis_ptr_->k1());
  EXPECT_DOUBLE_EQ(approx1.k2(), quad_vis_ptr_->k2());
  EXPECT_DOUBLE_EQ(approx1.k3(), quad_vis_ptr_->k3());

  QuadPolyVisApproximator approx2(quad_vis_ptr_->k1(), quad_vis_ptr_->k2(),
                                  quad_vis_ptr_->k3());
  EXPECT_TRUE(approx2.isInitialized());
  EXPECT_DOUBLE_EQ(approx2.k1(), quad_vis_ptr_->k1());
  EXPECT_DOUBLE_EQ(approx2.k2(), quad_vis_ptr_->k2());
  EXPECT_DOUBLE_EQ(approx2.k3(), quad_vis_ptr_->k3());
}

TEST_F(QuadPolyVisTest, equalQuadScore)
{
  QuadPolyVisApproximator qp_approx(quad_vis_options_);
  for (size_t i = 0; i < 100; i++)
  {
    rpg::Pose Twc;
    Twc.setRandom();
    Eigen::Vector3d ptw;
    ptw.setRandom();

    const Eigen::Vector3d f_cam = Twc.inverse() * ptw;
    const double quad_vis = quad_vis_ptr_->secondOrderVisibility(f_cam);

    Eigen::VectorXd camz_ftrs, lm_ftrs;
    qp_approx.calCamZFeatureVector(
        Twc.getRotationMatrix() * Eigen::Vector3d(0.0, 0.0, 1.0), &camz_ftrs);
    qp_approx.calLandmarkFeatureVector(ptw - Twc.getPosition(), &lm_ftrs);

    const double quadpoly_vis = camz_ftrs.dot(lm_ftrs);

    EXPECT_NEAR(quad_vis, quadpoly_vis, 1e-8);
  }
}

RPG_COMMON_TEST_MAIN
{
}
