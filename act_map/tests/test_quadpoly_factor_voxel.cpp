#include "act_map/positional_factor_voxel.h"
#include "act_map/positional_factor_voxel_ops.h"

#include <rpg_common/pose.h>
#include <rpg_common/test_main.h>
#include <rpg_common/timer.h>

#include "act_map/quad_info_factors.h"
#include "act_map/quad_trace_factors.h"
#include "act_map/quadratic_factor_ops.h"

using namespace act_map;

class QPFactorVoxelTest : public ::testing::Test
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

TEST_F(QPFactorVoxelTest, init)
{
  EXPECT_FALSE(QuadPolyInfoVoxel::isVisApproxValid());
  EXPECT_FALSE(QuadPolyTraceVoxel::isVisApproxValid());

  QuadPolyInfoVoxel::setVisApproxQuadVisOpt(quad_vis_options_);
  QuadPolyTraceVoxel::setVisApproxQuadVisOpt(quad_vis_options_);

  EXPECT_TRUE(QuadPolyInfoVoxel::isVisApproxValid());
  EXPECT_TRUE(QuadPolyTraceVoxel::isVisApproxValid());
}

TEST_F(QPFactorVoxelTest, testRecoverInfo)
{
  QuadPolyInfoVoxel::setVisApproxQuadVisOpt(quad_vis_options_);
  QuadPolyVisApproximator quadpoly_vis_approx(quad_vis_options_);

  Eigen::Vector3d pw;
  pw.setRandom();
  rpg::Pose Twc;
  Twc.setRandom();
  rpg::Matrix36 J0 =
      vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw, Twc.inverse());
  Info H0 = J0.transpose() * J0;

  QuadPolyInfoVoxel qp_info_vox;
  qp_info_vox.updateFactorSingle<internal::blockEqual>(pw, Twc.getPosition());

  QuadPolyInfoVoxel::OutMatType out;
  qp_info_vox.queryAtRotation(Twc.getRotationMatrix(), &out);

  Eigen::Vector3d camf_w =
      Twc.getRotation().rotate(Eigen::Vector3d(0.0, 0.0, 1.0));
  Eigen::VectorXd cam_ftrs;
  quadpoly_vis_approx.calCamZFeatureVector(camf_w, &cam_ftrs);
  Eigen::VectorXd lm_ftrs;
  quadpoly_vis_approx.calLandmarkFeatureVector(pw - Twc.getPosition(), &lm_ftrs);

  rpg::Matrix66 H0_scaled = H0 * (cam_ftrs.dot(lm_ftrs));
  EXPECT_TRUE(H0_scaled.isApprox(out));
}

TEST_F(QPFactorVoxelTest, equalOld)
{
  QuadPolyInfoVoxel::setVisApproxQuadVisOpt(quad_vis_options_);

  const int kNpts = 100;
  Eigen::Matrix3Xd points_w;
  points_w.resize(Eigen::NoChange, kNpts);
  points_w.setRandom();
  rpg::Pose Twc;
  Twc.setRandom();

  rpg::Timer timer;
  InfoK1 K1;
  InfoK2 K2;
  InfoK3 K3;
  const Eigen::Vector3d twc = Twc.getPosition();
  timer.start();
  constructFactorBatch(points_w, twc, &K1, &K2, &K3);
  std::cout << "Construct (old) took (ms) " << timer.stop() * 1e3 << std::endl;
  Info H_old;
  timer.start();
  getInfoAtRotation(Twc.getRotationMatrix(), quad_vis_ptr_->k1(), quad_vis_ptr_->k2(),
                    quad_vis_ptr_->k3(), K1, K2, K3, &H_old);
  std::cout << "Query (old) took (ms) " << timer.stop() * 1e3 << std::endl;
  std::cout << "H_old:\n" << H_old << std::endl;

  QuadPolyInfoVoxel quadpoly_vox;
  timer.start();
  constructFactorVoxelBatch(points_w, Twc.getPosition(), &quadpoly_vox);
  std::cout << "Construct (new) took (ms) " << timer.stop() * 1e3 << std::endl;
  Info H_new;
  timer.start();
  quadpoly_vox.queryAtRotation(Twc.getRotationMatrix(), &H_new);
  std::cout << "Query (new) took (ms) " << timer.stop() * 1e3 << std::endl;
  std::cout << "H_new:\n" << H_new << std::endl;

  EXPECT_TRUE(H_old.isApprox(H_new));
}

TEST_F(QPFactorVoxelTest, traceEqual)
{
  QuadPolyInfoVoxel::setVisApproxQuadVisOpt(quad_vis_options_);
  QuadPolyTraceVoxel::setVisApproxQuadVisOpt(quad_vis_options_);

  const int kNpts = 100;
  Eigen::Matrix3Xd points_w;
  points_w.resize(Eigen::NoChange, kNpts);
  points_w.setRandom();
  rpg::Pose Twc;
  Twc.setRandom();

  QuadPolyInfoVoxel info_vox;
  constructFactorVoxelBatch(points_w, Twc.getPosition(), &info_vox);
  QuadPolyInfoVoxel::OutMatType out_info;
  info_vox.queryAtRotation(Twc.getRotationMatrix(), &out_info);

  QuadPolyTraceVoxel trace_vox;
  constructFactorVoxelBatch(points_w, Twc.getPosition(), &trace_vox);
  QuadPolyTraceVoxel::OutMatType out_trace;
  trace_vox.queryAtRotation(Twc.getRotationMatrix(), &out_trace);

  EXPECT_NEAR(out_trace(0, 0), out_info.trace(), 1e-10);
}


RPG_COMMON_TEST_MAIN
{
}
