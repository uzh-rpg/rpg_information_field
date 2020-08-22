#include "act_map/info_calculator.h"

#include <rpg_common/test_main.h>

using namespace act_map;

class InfoCalculatorTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    VisibilityCheckerOptions options;
    options.use_view_filtering = true;
    options.min_view_angle_cos = 0.0;
    options.max_dist = 10.0;
    vis_checker_.reset(new VisibilityChecker(options));
  }
  VisibilityCheckerPtr vis_checker_;
};

TEST_F(InfoCalculatorTest, init)
{
  InfoCalculator info_cal_def;
  InfoCalculator info_cal(0.01, 0.5);
  InfoCalculator info_cal_vis(0.01, 0.5, vis_checker_);
}

TEST_F(InfoCalculatorTest, cal_info)
{
  constexpr size_t kN = 100;

  Vec3dVec points_w(kN);
  Vec3dVec views(kN);
  for (size_t i = 0; i < points_w.size(); i++)
  {
    points_w[i].setRandom();
    points_w[i](0) += 2.0;
    views[i] = Eigen::Vector3d(1, 0, 0);
  }

  InfoCalculator info_cal(0.01, 0.5, vis_checker_);
  rpg::Pose Twc;
  Twc.setIdentity();
  rpg::Matrix66 info_mat;
  info_cal.calculateInfoAt(Twc, points_w, {}, &info_mat);
  EXPECT_FALSE(info_mat.isApproxToConstant(0.0));

  Twc.getPosition() = Eigen::Vector3d(-20.0, 0.0, 0.0);
  info_cal.calculateInfoAt(Twc, points_w, {}, &info_mat);
  EXPECT_TRUE(info_mat.isApproxToConstant(0.0));

  Twc.setIdentity();
  info_cal.calculateInfoAt(Twc, points_w, views, &info_mat);
  EXPECT_TRUE(info_mat.isApproxToConstant(0.0));
}

TEST_F(InfoCalculatorTest, cal_metric)
{
  constexpr size_t kN = 100;
  Vec3dVec points_w(kN);
  for (size_t i = 0; i < points_w.size(); i++)
  {
    points_w[i].setRandom();
    points_w[i](0) += 2.0;
  }
  rpg::Pose Twc;
  Twc.setIdentity();
  Twc.getPosition() = Eigen::Vector3d(-20.0, 0.0, 0.0);
  InfoCalculator info_cal(0.01, 0.5, vis_checker_);

  Eigen::Vector3d dpos, drot_g;
  dpos.setRandom();
  drot_g.setRandom();
  double info_metric;
  bool res = info_cal.calculateInfoMetricAt(
      Twc, points_w, {}, InfoMetricType::kTrace, &info_metric, &dpos, &drot_g);
  EXPECT_FALSE(res);
  EXPECT_DOUBLE_EQ(0.0, info_metric);
  EXPECT_TRUE(dpos.isApproxToConstant(0.0));
  EXPECT_TRUE(drot_g.isApproxToConstant(0.0));

  Twc.setIdentity();
  res = info_cal.calculateInfoMetricAt(
      Twc, points_w, {}, InfoMetricType::kTrace, &info_metric, &dpos, &drot_g);
  EXPECT_TRUE(res);
  EXPECT_FALSE(dpos.isApproxToConstant(0.0));
  EXPECT_FALSE(drot_g.isApproxToConstant(0.0));
}

RPG_COMMON_TEST_MAIN
{
}
