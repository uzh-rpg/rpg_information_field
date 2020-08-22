#include "act_map/quadratic_factor_ops.h"

#include <rpg_common/test_main.h>
#include <rpg_common/pose.h>
#include <rpg_common/eigen_type.h>
#include <rpg_common/timer.h>

#include "act_map/info_utils.h"

using namespace act_map;

class QuadraticInfoFactorTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::srand(std::time(0));

    double hfov_rad = M_PI_4;
    vscore_ = std::make_shared<QuadraticVisScore>(hfov_rad);
    vscore_->initSecondOrderApprox(0.9, 0.9);

    Twc_.setRandom();
    pw_.setRandom();

    assignPoseInfoBearingGlobal(Twc_, pw_, vscore_, &H_weighted_);
    assignPoseInfoBearingGlobal(Twc_, pw_, nullptr, &H_);

    test_info_types_.push_back(InfoMetricType::kDet);
    test_info_types_.push_back(InfoMetricType::kTrace);
    test_info_types_.push_back(InfoMetricType::kMinEig);
  }

  QuadVisScorePtr vscore_;
  rpg::Pose Twc_;
  Eigen::Vector3d pw_;
  rpg::Matrix66 H_weighted_;
  rpg::Matrix66 H_;
  std::vector<InfoMetricType> test_info_types_;
};

TEST_F(QuadraticInfoFactorTest, testKernelSingle)
{
  InfoK1 K1;
  InfoK2 K2;
  InfoK3 K3;

  assignToFactor(pw_, Twc_.getPosition(), &K1, &K2, &K3);

  InfoK1 K1a;
  K1a.setZero();
  InfoK2 K2a;
  K2a.setZero();
  InfoK3 K3a;
  K3a.setZero();

  addToFactor(pw_, Twc_.getPosition(), &K1a, &K2a, &K3a);

  EXPECT_TRUE(K1.isApprox(K1a));
  EXPECT_TRUE(K2.isApprox(K2a));
  EXPECT_TRUE(K3.isApprox(K3a));

  substractFromFactor(pw_, Twc_.getPosition(), &K1a, &K2a, &K3a);
  EXPECT_TRUE(K1a.isZero());
  EXPECT_TRUE(K2a.isZero());
  EXPECT_TRUE(K3a.isZero());
}

TEST_F(QuadraticInfoFactorTest, testInfoEqual)
{
  InfoK1 K1;
  InfoK2 K2;
  InfoK3 K3;

  assignToFactor(pw_, Twc_.getPosition(), &K1, &K2, &K3);

  rpg::Matrix66 H;
  getInfoAtRotation(Twc_.getRotationMatrix(), vscore_->k1(), vscore_->k2(),
                    vscore_->k3(), K1, K2, K3, &H);
  EXPECT_TRUE(H.isApprox(H_weighted_));
  EXPECT_FALSE(H.isApprox(H_));
}

TEST_F(QuadraticInfoFactorTest, testMultiplePoints)
{
  const int kNpts = 100;
  Eigen::Matrix3Xd points_w;
  points_w.resize(Eigen::NoChange, kNpts);
  points_w.setRandom();

  rpg::Timer timer;
  timer.start();
  rpg::Matrix66 H_weighted;
  H_weighted.setZero();
  for (int i = 0; i < kNpts; i++)
  {
    addPoseInfoBearingGlobal(Twc_, points_w.col(i), vscore_, &H_weighted);
  }
  std::cout << "Weigthed sum took (ms) " << timer.stop() * 1e3 << std::endl;
  std::cout << "All points takes " << sizeof(double) * 3 * kNpts << " bytes.\n";

  timer.start();
  InfoK1 K1;
  InfoK2 K2;
  InfoK3 K3;
  rpg::Matrix66 H_K;
  const Eigen::Vector3d twc = Twc_.getPosition();
  constructFactorBatch(points_w, twc, &K1, &K2, &K3);
  std::cout << "constructing factors took (ms) " << timer.stop() * 1e3
            << std::endl;

  timer.start();
  getInfoAtRotation(Twc_.getRotationMatrix(), vscore_->k1(), vscore_->k2(),
                    vscore_->k3(), K1, K2, K3, &H_K);
  std::cout << "using factors took (ms) " << timer.stop() * 1e3 << std::endl;
  std::cout << "K1 takes " << sizeof(K1) << " bytes.\n";
  std::cout << "K2 takes " << sizeof(K2) << " bytes.\n";
  std::cout << "K3 takes " << sizeof(K3) << " bytes.\n";
  std::cout << "Kernels takes " << sizeof(K1) + sizeof(K2) + sizeof(K3)
            << " bytes.\n";

  EXPECT_TRUE(H_weighted.isApprox(H_K));

  for (const auto& v : test_info_types_)
  {
    EXPECT_EQ(getInfoMetric(H_K, v),
              getInfoMetricAtRotationFromPositionalFactor(Twc_.getRotationMatrix(),
                                                vscore_->k1(), vscore_->k2(),
                                                vscore_->k3(), K1, K2, K3, v));
  }
}

RPG_COMMON_TEST_MAIN
{
}
