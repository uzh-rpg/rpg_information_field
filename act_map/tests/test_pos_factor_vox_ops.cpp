#include "act_map/positional_factor_voxel.h"
#include "act_map/positional_factor_voxel_ops.h"

#include <cmath>

#include <rpg_common/pose.h>
#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>
#include <vi_utils/vi_jacobians.h>

using namespace act_map;

template <typename T>
class PosFactorVoxelOpsTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    // gp visibility
    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    const std::string abs_cfg_folder = dir + "/test_data/fov50_fs50_lm1000_k10";

    setGPVisiblityFromFolder(abs_cfg_folder);
    setQuadPolyVisiblity(QuadVisScoreOptions());

    // random pose and point
    Twc_.setRandom();
    pw_.setRandom();
    rpg::Matrix36 J0 =
        vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw_, Twc_.inverse());
    H0_ = J0.transpose() * J0;
  }

  rpg::Pose Twc_;
  Eigen::Vector3d pw_;
  rpg::Matrix66 H0_;
};

using PostionalFactorVoxelTypes =
    ::testing::Types<GPInfoVoxel, GPTraceVoxel, QuadPolyInfoVoxel,
                     QuadPolyTraceVoxel>;
TYPED_TEST_CASE(PosFactorVoxelOpsTest, PostionalFactorVoxelTypes);

TYPED_TEST(PosFactorVoxelOpsTest, testManipulateVoxelInterface)
{
  TypeParam vox;
  vox.template updateFactorSingle<internal::blockEqual>(
      this->pw_, this->Twc_.getPosition());
}

TYPED_TEST(PosFactorVoxelOpsTest, testVoxbloxInterface)
{
  TypeParam vox1, vox2, vox3;
  vox1.template updateFactorSingle<internal::blockEqual>(
      this->pw_, this->Twc_.getPosition());
  vox2.template updateFactorSingle<internal::blockEqual>(
      this->pw_, this->Twc_.getPosition());
  vox3.template updateFactorSingle<internal::blockEqual>(
      this->pw_ * 0.5, this->Twc_.getPosition());

  EXPECT_TRUE(vox1.isTheSame(vox2));
  EXPECT_FALSE(vox1.isTheSame(vox3));
}

TYPED_TEST(PosFactorVoxelOpsTest, testManipulateFactor)
{
  Eigen::Vector3d pw2;
  pw2.setRandom();
  TypeParam vox;

  const Eigen::Vector3d twc = this->Twc_.getPosition();

  typename TypeParam::FactorType init_factor = vox.getFactorConstRef();
  EXPECT_TRUE(init_factor.isConstant(0.0));

  vox.template updateFactorSingle<internal::blockEqual>(this->pw_, twc);
  vox.template updateFactorSingle<internal::blockMinusEqual>(this->pw_, twc);
  init_factor = vox.getFactorConstRef();
  EXPECT_TRUE(init_factor.isConstant(0.0));

  vox.template updateFactorSingle<internal::blockPlusEqual>(this->pw_, twc);
  typename TypeParam::FactorType pw_factor = vox.getFactorConstRef();
  vox.template updateFactorSingle<internal::blockPlusEqual>(pw2, twc);
  typename TypeParam::FactorType pw2_factor = vox.getFactorConstRef();
  EXPECT_FALSE(pw2_factor.isApprox(pw_factor));

  vox.template updateFactorSingle<internal::blockMinusEqual>(pw2, twc);
  EXPECT_TRUE(pw_factor.isApprox(vox.getFactorConstRef()));
}

TYPED_TEST(PosFactorVoxelOpsTest, testManipulateVoxel)
{
  TypeParam vox;

  Eigen::Vector3d pw2;
  pw2.setRandom();

  const Eigen::Vector3d twc = this->Twc_.getPosition();

  addToFactorVoxel(this->pw_, twc, &vox);
  typename TypeParam::FactorType pw_factor = vox.getFactorConstRef();
  addToFactorVoxel(pw2, twc, &vox);
  typename TypeParam::FactorType all_factor = vox.getFactorConstRef();
  substractFromFactorVoxel(pw2, twc, &vox);
  EXPECT_TRUE(pw_factor.isApprox(vox.getFactorConstRef()));

  TypeParam vox2;
  Vec3dVec points;
  points.push_back(this->pw_);
  points.push_back(pw2);
  addToFactorVoxel(points, twc, &vox2);
  EXPECT_TRUE(all_factor.isApprox(vox2.getFactorConstRef()));
}

TYPED_TEST(PosFactorVoxelOpsTest, testGetInfoMetricInterface)
{
  const Eigen::Vector3d twc = this->Twc_.getPosition();
  const Eigen::Matrix3Xd Rwc = this->Twc_.getRotationMatrix();

  TypeParam vox;
  addToFactorVoxel(this->pw_, twc, &vox);
  typename TypeParam::OutMatType out;
  vox.queryAtRotation(Rwc, &out);

  InfoMetricTypeVec test_metrics = supportedInfoMetricTypesVoxT<TypeParam>();

  for (const auto& v : test_metrics)
  {
    if (supportFullFIMVoxT<TypeParam>)
    {
      EXPECT_DOUBLE_EQ(getInfoMetricAtRotationFromPositionalFactor<TypeParam>(
                           Rwc, vox.getFactorConstRef(), v),
                       getInfoMetric(out, v));
    }
    else
    {
      // everything falls to trace
      EXPECT_DOUBLE_EQ(getInfoMetricAtRotationFromPositionalFactor<TypeParam>(
                           Rwc, vox.getFactorConstRef(), v),
                       out(0, 0));
    }
  }
}

RPG_COMMON_TEST_MAIN
{
}
