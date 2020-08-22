#include "act_map/positional_factor_voxel.h"
#include "act_map/positional_factor_voxel_ops.h"

#include <cmath>

#include <rpg_common/pose.h>
#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>
#include <vi_utils/vi_jacobians.h>

using namespace act_map;

class GPFactorVoxelTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    abs_cfg_folder_ = dir + "/test_data/fov50_fs50_lm1000_k10";

    Twc_.setRandom();
    pw_.setRandom();
    rpg::Matrix36 J0 =
        vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw_, Twc_.inverse());
    H0_ = J0.transpose() * J0;

    gp_vis_ptr_ = std::make_shared<GPVisibilityApproximator>();
    gp_vis_ptr_->load(abs_cfg_folder_);
  }
  std::string abs_cfg_folder_;
  VisApproxPtr<GPVisApprox> gp_vis_ptr_ = nullptr;
  rpg::Pose Twc_;
  Eigen::Vector3d pw_;
  rpg::Matrix66 H0_;
};

TEST_F(GPFactorVoxelTest, testSetVisApprox)
{
  EXPECT_FALSE(GPInfoVoxel::isVisApproxValid());

  GPInfoVoxel::setVisApprox(gp_vis_ptr_);
  EXPECT_TRUE(GPInfoVoxel::isVisApproxValid());

  GPInfoVoxel::resetVisApprox();
  EXPECT_FALSE(GPInfoVoxel::isVisApproxValid());
}

TEST_F(GPFactorVoxelTest, testInitialization)
{
  GPInfoVoxel::setVisApprox(gp_vis_ptr_);

  GPInfoVoxel gp_info_vox;
  EXPECT_TRUE(gp_info_vox.isVisApproxValid());
  EXPECT_EQ(gp_vis_ptr_->dimFeatureSpace(), gp_info_vox.dimFeatures());
}

TEST_F(GPFactorVoxelTest, testRecoverInfo)
{
  GPInfoVoxel::setVisApprox(gp_vis_ptr_);
  GPInfoVoxel gp_info_vox;
  gp_info_vox.updateFactorSingle<internal::blockEqual>(pw_, Twc_.getPosition());

  GPInfoVoxel::OutMatType out;
  gp_info_vox.queryAtRotation(Twc_.getRotationMatrix(), &out);

  Eigen::Vector3d camf_w =
      Twc_.getRotation().rotate(Eigen::Vector3d(0.0, 0.0, 1.0));
  Eigen::VectorXd cam_ftrs;
  gp_vis_ptr_->calCamZFeatureVector(camf_w, &cam_ftrs);
  Eigen::VectorXd lm_ftrs;
  gp_vis_ptr_->calLandmarkFeatureVector(pw_ - Twc_.getPosition(), &lm_ftrs);

  rpg::Matrix66 H0_scaled = H0_ * (cam_ftrs.dot(lm_ftrs));
  EXPECT_TRUE(H0_scaled.isApprox(out));
}

TEST_F(GPFactorVoxelTest, testTraceVoxelEqual)
{
  // these are different classes
  GPTraceVoxel::resetVisApprox();
  GPInfoVoxel::setVisApprox(gp_vis_ptr_);
  EXPECT_FALSE(GPTraceVoxel::isVisApproxValid());
  GPTraceVoxel::setVisApprox(gp_vis_ptr_);
  EXPECT_TRUE(GPTraceVoxel::isVisApproxValid());

  const Eigen::Vector3d twc = Twc_.getPosition();
  const Eigen::Matrix3Xd Rwc = Twc_.getRotationMatrix();

  GPTraceVoxel gp_trace_vox;
  addToFactorVoxel(pw_, twc, &gp_trace_vox);
  GPInfoVoxel gp_info_vox;
  addToFactorVoxel(pw_, twc, &gp_info_vox);

  GPTraceVoxel::OutMatType out_trace;
  gp_trace_vox.queryAtRotation(Twc_.getRotationMatrix(), &out_trace);

  GPInfoVoxel::OutMatType out_info;
  gp_info_vox.queryAtRotation(Twc_.getRotationMatrix(), &out_info);

  EXPECT_NEAR(out_trace(0, 0), out_info.trace(), 1e-10);
}

RPG_COMMON_TEST_MAIN
{
}
