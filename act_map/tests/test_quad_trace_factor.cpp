#include "act_map/quadratic_factor_ops.h"

#include <rpg_common/test_main.h>
#include <rpg_common/pose.h>
#include <rpg_common/timer.h>

#include "act_map/quadratic_vis_score.h"
#include "act_map/quad_info_factors.h"

using namespace act_map;

class QuadraticTraceFactorTest : public ::testing::Test
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
    //    Twc_.setIdentity();
    //    pw_ = Eigen::Vector3d(1.2, 2.5, 3.9);
  }

  QuadVisScorePtr vscore_;
  rpg::Pose Twc_;
  Eigen::Vector3d pw_;
};

TEST_F(QuadraticTraceFactorTest, testKernelSingle)
{
  TraceK1 K1;
  TraceK2 K2;
  TraceK3 K3;

  assignToFactor(pw_, Twc_.getPosition(), &K1, &K2, &K3);

  TraceK1 K1a;
  K1a.setZero();
  TraceK2 K2a;
  K2a.setZero();
  TraceK3 K3a;
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

TEST_F(QuadraticTraceFactorTest, testTraceEqual)
{
  TraceK1 trace_K1;
  TraceK2 trace_K2;
  TraceK3 trace_K3;

  assignToFactor(pw_, Twc_.getPosition(), &trace_K1, &trace_K2, &trace_K3);
  double trace =
      getTraceAtRotation(Twc_.getRotationMatrix(), vscore_->k1(), vscore_->k2(),
                         vscore_->k3(), trace_K1, trace_K2, trace_K3);
  std::cout << "Trace K1 is:\n" << trace_K1 << std::endl;
  std::cout << "Trace K2 is:\n" << trace_K2 << std::endl;

  InfoK1 info_K1;
  InfoK2 info_K2;
  InfoK3 info_K3;

  assignToFactor(pw_, Twc_.getPosition(), &info_K1, &info_K2, &info_K3);

  rpg::Matrix66 H;
  getInfoAtRotation(Twc_.getRotationMatrix(), vscore_->k1(), vscore_->k2(),
                    vscore_->k3(), info_K1, info_K2, info_K3, &H);

  EXPECT_DOUBLE_EQ(trace, H.trace());
}

TEST_F(QuadraticTraceFactorTest, testMultiplePoints)
{
  const int kNpts = 100;
  Eigen::Matrix3Xd points_w;
  points_w.resize(Eigen::NoChange, kNpts);
  points_w.setRandom();

  const Eigen::Vector3d twc = Twc_.getPosition();
  rpg::Timer timer;

  timer.start();
  InfoK1 info_K1;
  InfoK2 info_K2;
  InfoK3 info_K3;
  rpg::Matrix66 H_K;
  constructFactorBatch(points_w, twc, &info_K1, &info_K2, &info_K3);
  std::cout << "constructing info factors took (ms) " << timer.stop() * 1e3
            << std::endl;

  timer.start();
  getInfoAtRotation(Twc_.getRotationMatrix(), vscore_->k1(), vscore_->k2(),
                    vscore_->k3(), info_K1, info_K2, info_K3, &H_K);
  double trace_HK = H_K.trace();
  std::cout << "using info factors took (ms) " << timer.stop() * 1e3
            << std::endl;
  std::cout << "- K1 takes " << sizeof(info_K1) << " bytes.\n";
  std::cout << "- K2 takes " << sizeof(info_K2) << " bytes.\n";
  std::cout << "- K3 takes " << sizeof(info_K3) << " bytes.\n";
  std::cout << "- Kernels takes "
            << sizeof(info_K1) + sizeof(info_K2) + sizeof(info_K3)
            << " bytes.\n";

  timer.start();
  TraceK1 trace_K1;
  TraceK2 trace_K2;
  TraceK3 trace_K3;
  constructFactorBatch(points_w, twc, &trace_K1, &trace_K2, &trace_K3);
  std::cout << "constructing trace kernels took (ms) " << timer.stop() * 1e3
            << std::endl;
  timer.start();
  double trace =
      getTraceAtRotation(Twc_.getRotationMatrix(), vscore_->k1(), vscore_->k2(),
                         vscore_->k3(), trace_K1, trace_K2, trace_K3);
  std::cout << "using trace factors took (ms) " << timer.stop() * 1e3
            << std::endl;
  std::cout << "- K1 takes " << sizeof(trace_K1) << " bytes.\n";
  std::cout << "- K2 takes " << sizeof(trace_K2) << " bytes.\n";
  std::cout << "- K3 takes " << sizeof(trace_K3) << " bytes.\n";
  std::cout << "- Kernels takes "
            << sizeof(trace_K1) + sizeof(trace_K2) + sizeof(trace_K3)
            << " bytes.\n";
  EXPECT_DOUBLE_EQ(trace, trace_HK);

  EXPECT_EQ(trace, getInfoMetricAtRotationFromPositionalFactor(
                       Twc_.getRotationMatrix(), vscore_->k1(), vscore_->k2(),
                       vscore_->k3(), trace_K1, trace_K2, trace_K3,
                       InfoMetricType::kTrace));
  EXPECT_EQ(trace, getInfoMetricAtRotationFromPositionalFactor(
                       Twc_.getRotationMatrix(), vscore_->k1(), vscore_->k2(),
                       vscore_->k3(), trace_K1, trace_K2, trace_K3,
                       InfoMetricType::kDet));
}

RPG_COMMON_TEST_MAIN
{
}
